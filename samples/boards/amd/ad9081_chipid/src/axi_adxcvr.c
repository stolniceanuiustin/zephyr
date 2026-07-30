/*
 * AXI ADXCVR -- GT transceiver (PHY) bring-up, minimal fixed-rate variant.
 *
 * This drives the ADI AXI-ADXCVR wrapper around the Xilinx GTH4 transceivers on
 * ZynqMP for the AD9081 JESD204B link (TX -> QPLL0, RX -> CPLL, out clock via
 * PROGDIV, 10 Gbps lanes off a 500 MHz refclk).
 *
 * Scope decision: the bitstream is synthesised by the Xilinx Transceiver Wizard
 * for exactly this rate, so the GT's PLL/CDR/divider attributes are already
 * correct at configuration time. On UltraScale the no-OS driver's own DRP
 * "configure_cdr"/"configure_lpm_dfe_mode" paths are empty stubs for precisely
 * this reason ("the UltraScale Transceiver Wizard should be used"). We therefore
 * do NOT re-program the GT over DRP -- we only:
 *   1. select the PLL source + output-clock mux + LPM/DFE mode (REG_CONTROL),
 *   2. pulse the core reset and poll for the transceiver-ready status,
 *   3. clear any elastic-buffer under/overflow (newer PCORE).
 * This mirrors adxcvr_init() steps 1-6 + adxcvr_clk_enable() from no-OS
 * axi_adxcvr.c, minus adxcvr_clk_set_rate() (the DRP layer we trust the
 * bitstream to have set).
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/kernel/mm.h>
#include <zephyr/sys/device_mmio.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(axi_adxcvr, LOG_LEVEL_INF);

#include "axi_adxcvr.h"
#include "xilinx_transceiver.h"

/*
 * AXI base addresses (from the on-board bitstream system.hwh). Not in the A53
 * SoC MMU table -- mapped 1:1 non-cached at PRE_KERNEL_1, like the other PL
 * pages.
 */
#define ADXCVR_RX_BASE 0x84A60000UL
#define ADXCVR_TX_BASE 0x84B60000UL
#define ADXCVR_SIZE    0x10000UL

/* Register offsets (no-OS axi_adxcvr.c). */
#define ADXCVR_REG_VERSION 0x0000
#define ADXCVR_REG_FPGA_INFO    0x001C
#define ADXCVR_REG_FPGA_VOLTAGE 0x0140
#define ADXCVR_REG_RESETN  0x0010
#define ADXCVR_REG_STATUS  0x0014
#define ADXCVR_REG_CONTROL 0x0020
#define ADXCVR_REG_SYNTH   0x0024

/* FPGA info decode (no-OS xilinx_transceiver.h AXI_INFO_*). */
#define ADXCVR_INFO_TECH(x)        ((x) >> 24)
#define ADXCVR_INFO_FAMILY(x)      (((x) >> 16) & 0xff)
#define ADXCVR_INFO_SPEED_GRADE(x) (((x) >> 8) & 0xff)
#define ADXCVR_INFO_DEV_PACKAGE(x) ((x) & 0xff)
#define ADXCVR_INFO_VOLTAGE(x)     ((x) & 0xffff)

/* Link-mode field of REG_SYNTH: 1 = 204B (8b/10b), 2 = 204C (64b/66b). */
#define ADXCVR_SYNTH_LINK_MODE(x)  (((x) >> 12) & 0x3)
#define ADXCVR_LINK_MODE_204B      1
#define ADXCVR_LINK_MODE_204C      2

/* DRP indirection registers (no-OS axi_adxcvr.c). */
#define ADXCVR_REG_DRP_SEL(x)    (0x0040 + (x))
#define ADXCVR_REG_DRP_CTRL(x)   (0x0044 + (x))
#define ADXCVR_DRP_CTRL_WR       BIT(28)
#define ADXCVR_DRP_CTRL_ADDR(x)  (((x) & 0xFFF) << 16)
#define ADXCVR_DRP_CTRL_WDATA(x) (((x) & 0xFFFF) << 0)
#define ADXCVR_REG_DRP_STATUS(x) (0x0048 + (x))
#define ADXCVR_DRP_STATUS_BUSY   BIT(16)
#define ADXCVR_DRP_STATUS_RDATA(x) (((x) & 0xFFFF) << 0)

#define ADXCVR_DRP_PORT_ADDR_COMMON  0x00
#define ADXCVR_DRP_PORT_ADDR_CHANNEL 0x20
#define ADXCVR_DRP_PORT_COMMON(x)    (x)
#define ADXCVR_DRP_PORT_CHANNEL(x)   (0x100 + (x))

/* REG_RESETN bits. */
#define ADXCVR_RESETN        BIT(0)
#define ADXCVR_BUFSTATUS_RST BIT(1)

/*
 * REG_STATUS bits. Layout from the IP itself (HDL axi_adxcvr_up.v:537):
 *   {25'd0, bufstatus[1], bufstatus[0], ~pll_locked, 3'b0, status}
 * so bit4 is active-high "PLL *not* locked" -- the inverse sense of the others.
 */
#define ADXCVR_STATUS              BIT(0)
#define ADXCVR_PLL_NOT_LOCKED      BIT(4)
#define ADXCVR_BUFSTATUS_UNDERFLOW BIT(5)
#define ADXCVR_BUFSTATUS_OVERFLOW  BIT(6)

/* REG_CONTROL fields. */
#define ADXCVR_LPM_DFE_N     BIT(12)
#define ADXCVR_SYSCLK_SEL(x) (((x) & 0x3) << 4)
#define ADXCVR_OUTCLK_SEL(x) (((x) & 0x7) << 0)

/* REG_SYNTH decode. */
#define ADXCVR_SYNTH_NUM_LANES(x) ((x) & 0xff)
#define ADXCVR_SYNTH_TX_ENABLE(x) (((x) >> 8) & 1)
#define ADXCVR_SYNTH_XCVR_TYPE(x) (((x) >> 16) & 0xf)

/* sys_clk_sel values (axi_adxcvr.h). */
#define ADXCVR_SYS_CLK_CPLL  0x00
#define ADXCVR_SYS_CLK_QPLL1 0x02
#define ADXCVR_SYS_CLK_QPLL0 0x03

/* out_clk_sel values (axi_adxcvr.h). PROGDIV is GTHE3/4, GTYE4 only. */
#define ADXCVR_REFCLK      3
#define ADXCVR_PROGDIV_CLK 5

/* xcvr_type: UltraScale+ GTH4 as reported in REG_SYNTH for this board. */
#define XILINX_XCVR_TYPE_US_GTH4 8

/* PCORE version helpers -- buffer-status handling exists from 17.5a. */
#define ADXCVR_PCORE_VER(major, minor, patch) \
	(((major) << 16) | ((minor) << 8) | (patch))
#define PCORE_VER_MAJOR(x) (((x) >> 16) & 0xff)
#define PCORE_VER_MINOR(x) (((x) >> 8) & 0xff)
#define PCORE_VER_PATCH(x) ((x) & 0xff)

/*
 * One GT transceiver instance (TX or RX side). Note struct adxcvr is forward-
 * declared to the verbatim xilinx_transceiver.c via xcvr_shim.h; that file only
 * ever handles a pointer to it and calls adxcvr_drp_read/write (below), so the
 * full definition living here is fine.
 */
struct adxcvr {
	const char *name;
	uintptr_t base;
	uint8_t sys_clk_sel;
	uint8_t out_clk_sel;
	bool lpm_enable;
	bool cpll_enable;
	bool qpll_enable;
	/* target rates for the DRP divider solve (kHz). */
	uint32_t lane_rate_khz;
	uint32_t ref_rate_khz;
	/* read back from the core */
	uint32_t version;
	uint32_t num_lanes;
	bool tx_enable;
	uint32_t xcvr_type;
	/* Xilinx GT reconfiguration state (drives the verbatim DRP math). */
	struct xilinx_xcvr xlx_xcvr;
};

/*
 * zcu102 ad9081_m8_l4 profile (no-OS ADXCVR_*_KHZ): 10 Gbps lanes off a 500 MHz
 * GT refclk. TX uses QPLL0, RX uses CPLL. These are the rates no-OS feeds to
 * adxcvr_clk_set_rate() -- programming the GT dividers over DRP at runtime
 * rather than trusting whatever the bitstream synthesised.
 */
#define ADXCVR_REF_CLK_KHZ  500000
#define ADXCVR_LANE_CLK_KHZ 10000000

static struct adxcvr adxcvr_tx = {
	.name = "tx_adxcvr",
	.base = ADXCVR_TX_BASE,
	.sys_clk_sel = ADXCVR_SYS_CLK_QPLL0,
	.out_clk_sel = ADXCVR_PROGDIV_CLK,
	.lpm_enable = false,
	.lane_rate_khz = ADXCVR_LANE_CLK_KHZ,
	.ref_rate_khz = ADXCVR_REF_CLK_KHZ,
};

static struct adxcvr adxcvr_rx = {
	.name = "rx_adxcvr",
	.base = ADXCVR_RX_BASE,
	.sys_clk_sel = ADXCVR_SYS_CLK_CPLL,
	.out_clk_sel = ADXCVR_PROGDIV_CLK,
	.lpm_enable = true,
	.lane_rate_khz = ADXCVR_LANE_CLK_KHZ,
	.ref_rate_khz = ADXCVR_REF_CLK_KHZ,
};

static inline uint32_t adxcvr_read(const struct adxcvr *x, uint32_t reg)
{
	return sys_read32(x->base + reg);
}

static inline void adxcvr_write(const struct adxcvr *x, uint32_t reg,
				uint32_t val)
{
	sys_write32(val, x->base + reg);
}

/*
 * DRP (Dynamic Reconfiguration Port) accessors. The verbatim GT divider math in
 * xilinx_transceiver.c reaches the transceiver only through these two functions
 * (declared in xcvr_shim.h). Ports of no-OS adxcvr_drp_read/write +
 * adxcvr_drp_wait_idle; plain AXI MMIO under the hood.
 */
static int adxcvr_drp_wait_idle(struct adxcvr *x, uint32_t drp_addr)
{
	uint32_t val;
	int timeout = 20;

	while (timeout-- > 0) {
		val = adxcvr_read(x, ADXCVR_REG_DRP_STATUS(drp_addr));
		if (!(val & ADXCVR_DRP_STATUS_BUSY)) {
			return ADXCVR_DRP_STATUS_RDATA(val);
		}
		k_msleep(1);
	}

	LOG_ERR("%s: DRP wait idle timeout", x->name);
	return -1;
}

int adxcvr_drp_read(struct adxcvr *x, unsigned int drp_port,
		    unsigned int reg, unsigned int *val)
{
	uint32_t drp_addr = (drp_port < ADXCVR_DRP_PORT_CHANNEL(0)) ?
			    ADXCVR_DRP_PORT_ADDR_COMMON :
			    ADXCVR_DRP_PORT_ADDR_CHANNEL;
	int ret;

	adxcvr_write(x, ADXCVR_REG_DRP_SEL(drp_addr), drp_port & 0xFF);
	adxcvr_write(x, ADXCVR_REG_DRP_CTRL(drp_addr), ADXCVR_DRP_CTRL_ADDR(reg));

	ret = adxcvr_drp_wait_idle(x, drp_addr);
	if (ret < 0) {
		return ret;
	}

	*val = ret & 0xFFFF;
	return 0;
}

int adxcvr_drp_write(struct adxcvr *x, unsigned int drp_port,
		     unsigned int reg, unsigned int val)
{
	uint32_t drp_addr = (drp_port < ADXCVR_DRP_PORT_CHANNEL(0)) ?
			    ADXCVR_DRP_PORT_ADDR_COMMON :
			    ADXCVR_DRP_PORT_ADDR_CHANNEL;
	int ret;

	adxcvr_write(x, ADXCVR_REG_DRP_SEL(drp_addr), drp_port & 0xFF);
	adxcvr_write(x, ADXCVR_REG_DRP_CTRL(drp_addr),
		     ADXCVR_DRP_CTRL_WR | ADXCVR_DRP_CTRL_ADDR(reg) |
		     ADXCVR_DRP_CTRL_WDATA(val));

	ret = adxcvr_drp_wait_idle(x, drp_addr);
	if (ret < 0) {
		return ret;
	}
	return 0;
}

static int axi_adxcvr_map(void)
{
	mm_reg_t virt;

	device_map(&virt, ADXCVR_RX_BASE, ADXCVR_SIZE, K_MEM_CACHE_NONE);
	if (virt != ADXCVR_RX_BASE) {
		LOG_ERR("RX XCVR not identity-mapped: virt=0x%lx phys=0x%lx",
			(unsigned long)virt, ADXCVR_RX_BASE);
		return -EIO;
	}

	device_map(&virt, ADXCVR_TX_BASE, ADXCVR_SIZE, K_MEM_CACHE_NONE);
	if (virt != ADXCVR_TX_BASE) {
		LOG_ERR("TX XCVR not identity-mapped: virt=0x%lx phys=0x%lx",
			(unsigned long)virt, ADXCVR_TX_BASE);
		return -EIO;
	}
	return 0;
}

SYS_INIT(axi_adxcvr_map, PRE_KERNEL_1, 0);

/*
 * Configure one transceiver: assert reset and program the clock muxes. Mirrors
 * no-OS adxcvr_init() steps 1-6 (minus the DRP set_rate). Returns 0, or negative
 * errno if the core reports an unexpected transceiver type.
 */
/*
 * Read the FPGA identity registers into xlx_xcvr. The verbatim VCO-range setup
 * (xilinx_xcvr_setup_cpll/qpll_vco_range) consults tech/family/speed/package/
 * voltage when PCORE major > 0x10, so they must be populated. Port of no-OS
 * adxcvr_get_info().
 */
static void adxcvr_get_info(struct adxcvr *x)
{
	uint32_t info = adxcvr_read(x, ADXCVR_REG_FPGA_INFO);
	uint32_t volt = adxcvr_read(x, ADXCVR_REG_FPGA_VOLTAGE);

	x->xlx_xcvr.tech = ADXCVR_INFO_TECH(info);
	x->xlx_xcvr.family = ADXCVR_INFO_FAMILY(info);
	x->xlx_xcvr.speed_grade = ADXCVR_INFO_SPEED_GRADE(info);
	x->xlx_xcvr.dev_package = ADXCVR_INFO_DEV_PACKAGE(info);
	x->xlx_xcvr.voltage = ADXCVR_INFO_VOLTAGE(volt);
}

/*
 * Program the GT dividers over DRP for the target lane rate. Verbatim port of
 * no-OS adxcvr_clk_set_rate(): solve the CPLL/QPLL config, then write OUT_DIV,
 * PROGDIV (+rate), CDR and CLK25_DIV per lane. This is the step the "trust the
 * bitstream" build skipped -- without it the GT output clock never runs and
 * RESET_DONE never asserts (STATUS stays 0x0).
 */
static int adxcvr_clk_set_rate(struct adxcvr *x, unsigned long rate,
			       unsigned long parent_rate)
{
	struct xilinx_xcvr_cpll_config cpll_conf;
	struct xilinx_xcvr_qpll_config qpll_conf;
	uint32_t out_div, clk25_div, prog_div;
	uint32_t i;
	int ret = 0;

	clk25_div = DIV_ROUND_CLOSEST(parent_rate, 25000);

	if (x->cpll_enable) {
		ret = xilinx_xcvr_calc_cpll_config(&x->xlx_xcvr, parent_rate, rate,
						   &cpll_conf, &out_div);
	} else {
		ret = xilinx_xcvr_calc_qpll_config(&x->xlx_xcvr, x->sys_clk_sel,
						   parent_rate, rate, &qpll_conf,
						   &out_div);
	}
	if (ret < 0) {
		LOG_ERR("%s: no %s config for %lu kHz @ ref %lu kHz", x->name,
			x->cpll_enable ? "CPLL" : "QPLL", rate, parent_rate);
		return ret;
	}

	for (i = 0; i < x->num_lanes; i++) {
		if (x->cpll_enable) {
			ret = xilinx_xcvr_cpll_write_config(&x->xlx_xcvr,
					ADXCVR_DRP_PORT_CHANNEL(i), &cpll_conf);
		} else if ((i % 4 == 0) && x->qpll_enable) {
			ret = xilinx_xcvr_qpll_write_config(&x->xlx_xcvr,
					x->sys_clk_sel,
					ADXCVR_DRP_PORT_COMMON(i), &qpll_conf);
		}
		if (ret < 0) {
			return ret;
		}

		ret = xilinx_xcvr_write_out_div(&x->xlx_xcvr,
				ADXCVR_DRP_PORT_CHANNEL(i),
				x->tx_enable ? -1 : (int32_t)out_div,
				x->tx_enable ? (int32_t)out_div : -1);
		if (ret < 0) {
			return ret;
		}

		if (x->out_clk_sel == ADXCVR_PROGDIV_CLK) {
			unsigned int max_progdiv, div = 1, ratio;

			if (x->xlx_xcvr.encoding == ENC_66B64B) {
				ratio = 66;
			} else {
				ratio = 40;
			}

			/* Set RX|TX_PROGDIV_RATE = 2 on GTY4. */
			ret = xilinx_xcvr_write_prog_div_rate(&x->xlx_xcvr,
					ADXCVR_DRP_PORT_CHANNEL(i),
					x->tx_enable ? -1 : 2,
					x->tx_enable ? 2 : -1);
			if (!ret) {
				div = 2;
			}

			switch (x->xlx_xcvr.type) {
			case XILINX_XCVR_TYPE_US_GTH3:
				max_progdiv = 100;
				if (x->xlx_xcvr.encoding == ENC_66B64B) {
					div = 2;
				}
				break;
			case XILINX_XCVR_TYPE_US_GTH4:
				max_progdiv = 132;
				if (x->xlx_xcvr.encoding == ENC_66B64B) {
					div = 2;
				}
				break;
			case XILINX_XCVR_TYPE_US_GTY4:
				max_progdiv = 100;
				break;
			default:
				return -EINVAL;
			}

			prog_div = DIV_ROUND_CLOSEST(ratio * out_div, 2 * div);

			if (prog_div > max_progdiv) {
				prog_div = 0; /* disabled */
				LOG_WRN("%s: no PROGDIV for OUTDIV=%u, disabling output",
					x->name, out_div);
			}

			ret = xilinx_xcvr_write_prog_div(&x->xlx_xcvr,
					ADXCVR_DRP_PORT_CHANNEL(i),
					x->tx_enable ? -1 : (int32_t)prog_div,
					x->tx_enable ? (int32_t)prog_div : -1);
			if (ret < 0) {
				return ret;
			}
		}

		if (!x->tx_enable) {
			ret = xilinx_xcvr_configure_cdr(&x->xlx_xcvr,
					ADXCVR_DRP_PORT_CHANNEL(i), rate, out_div,
					x->lpm_enable);
			if (ret < 0) {
				return ret;
			}
			ret = xilinx_xcvr_write_rx_clk25_div(&x->xlx_xcvr,
					ADXCVR_DRP_PORT_CHANNEL(i), clk25_div);
		} else {
			ret = xilinx_xcvr_write_tx_clk25_div(&x->xlx_xcvr,
					ADXCVR_DRP_PORT_CHANNEL(i), clk25_div);
		}
		if (ret < 0) {
			return ret;
		}
	}

	x->lane_rate_khz = rate;
	return 0;
}

/* REG_CONTROL as this instance's configuration says it should be. */
static uint32_t adxcvr_control_word(const struct adxcvr *x)
{
	return (x->lpm_enable ? ADXCVR_LPM_DFE_N : 0) |
	       ADXCVR_SYSCLK_SEL(x->sys_clk_sel) |
	       ADXCVR_OUTCLK_SEL(x->out_clk_sel);
}

static int adxcvr_configure(struct adxcvr *x)
{
	uint32_t synth = adxcvr_read(x, ADXCVR_REG_SYNTH);
	uint32_t control;
	int ret;

	x->version = adxcvr_read(x, ADXCVR_REG_VERSION);
	x->num_lanes = ADXCVR_SYNTH_NUM_LANES(synth);
	x->tx_enable = ADXCVR_SYNTH_TX_ENABLE(synth);
	x->xcvr_type = ADXCVR_SYNTH_XCVR_TYPE(synth);
	x->qpll_enable = (synth >> 20) & 1;
	x->cpll_enable = (x->sys_clk_sel == ADXCVR_SYS_CLK_CPLL);

	LOG_INF("%s @ 0x%08lx: PCORE v%u.%u.%u, %u lanes, type=%u, %s",
		x->name, (unsigned long)x->base,
		PCORE_VER_MAJOR(x->version), PCORE_VER_MINOR(x->version),
		PCORE_VER_PATCH(x->version), x->num_lanes, x->xcvr_type,
		x->tx_enable ? "TX" : "RX");

	if (x->xcvr_type != XILINX_XCVR_TYPE_US_GTH4) {
		LOG_WRN("%s: xcvr_type=%u, expected GTH4(%u) -- clock mux/PROGDIV "
			"assumptions may not hold", x->name, x->xcvr_type,
			XILINX_XCVR_TYPE_US_GTH4);
	}

	/* Populate the Xilinx GT reconfiguration context for the DRP math. */
	x->xlx_xcvr.ad_xcvr = x;
	x->xlx_xcvr.type = x->xcvr_type;
	x->xlx_xcvr.version = x->version;
	x->xlx_xcvr.refclk_ppm = PM_200;
	x->xlx_xcvr.encoding =
		(ADXCVR_SYNTH_LINK_MODE(synth) == ADXCVR_LINK_MODE_204C) ?
		ENC_66B64B : ENC_8B10B;
	if (PCORE_VER_MAJOR(x->version) > 0x10) {
		adxcvr_get_info(x);
	}

	/* Assert reset while we set the clock selection. */
	adxcvr_write(x, ADXCVR_REG_RESETN, 0);

	control = adxcvr_control_word(x);
	adxcvr_write(x, ADXCVR_REG_CONTROL, control);

	LOG_INF("%s: CONTROL=0x%04x (sysclk=%u outclk=%u lpm=%u) enc=%s",
		x->name, control, x->sys_clk_sel, x->out_clk_sel,
		x->lpm_enable,
		x->xlx_xcvr.encoding == ENC_66B64B ? "64b/66b" : "8b/10b");

	/*
	 * Program the GT dividers over DRP (no-OS did this in adxcvr_init). Held
	 * in reset above; the FSM releases reset later in adxcvr_clk_enable().
	 */
	ret = adxcvr_clk_set_rate(x, x->lane_rate_khz, x->ref_rate_khz);
	if (ret) {
		LOG_ERR("%s: GT divider programming failed (%d)", x->name, ret);
		return ret;
	}
	LOG_INF("%s: GT dividers programmed for %u kHz lane @ %u kHz ref",
		x->name, x->lane_rate_khz, x->ref_rate_khz);

	return 0;
}

/*
 * Pulse the core reset and poll for transceiver-ready. Mirrors no-OS
 * adxcvr_reset()/adxcvr_status_error(): up to two reset attempts, each polling
 * STATUS bit0 for <=100 ms. Returns 0 when ready, -ETIMEDOUT otherwise.
 */
static int adxcvr_reset(struct adxcvr *x)
{
	int retry = 1;
	uint32_t status = 0;

	do {
		int timeout = 100;

		adxcvr_write(x, ADXCVR_REG_RESETN, 0);
		k_busy_wait(2);
		adxcvr_write(x, ADXCVR_REG_RESETN, ADXCVR_RESETN);

		while (timeout--) {
			k_msleep(1);
			status = adxcvr_read(x, ADXCVR_REG_STATUS);
			if (status & ADXCVR_STATUS) {
				return 0;
			}
		}
	} while (retry--);

	/*
	 * RESET_DONE never asserted. The raw STATUS word says why, so decode the
	 * common cause rather than leaving the reader to look up the bit layout:
	 * bit4 is ~pll_locked (axi_adxcvr_up.v:537), and a GT that cannot lock its
	 * PLL almost always has a missing or wrong reference clock (QPLL0 for TX,
	 * CPLL for RX) rather than a reset-pulse problem.
	 *
	 * Verified by fault injection: pointing the RX GT at an undriven QPLL1
	 * produces exactly STATUS=0x10 here.
	 */
	if (status & ADXCVR_PLL_NOT_LOCKED) {
		LOG_ERR("%s: GT PLL not locked after 2x100ms (STATUS=0x%08x) -- "
			"check the reference clock", x->name, status);
	} else {
		LOG_ERR("%s: RESET_DONE not set after 2x100ms (raw STATUS=0x%08x)",
			x->name, status);
	}
	return -ETIMEDOUT;
}

/*
 * Bring one transceiver's lane clock up. Mirrors no-OS adxcvr_clk_enable():
 * reset+status, then (on PCORE >= 17.5a) clear elastic-buffer under/overflow.
 */
static int adxcvr_clk_enable(struct adxcvr *x)
{
	uint32_t status;
	int retry = 100;
	int ret;

	ret = adxcvr_reset(x);
	if (ret) {
		LOG_ERR("%s: transceiver not ready after reset (%d)",
			x->name, ret);
		return ret;
	}

	if (x->version < ADXCVR_PCORE_VER(17, 5, 'a')) {
		LOG_INF("%s: lane clock up", x->name);
		return 0;
	}

	do {
		bool buf_err;

		adxcvr_write(x, ADXCVR_REG_RESETN,
			     ADXCVR_BUFSTATUS_RST | ADXCVR_RESETN);
		adxcvr_write(x, ADXCVR_REG_RESETN, ADXCVR_RESETN);
		k_msleep(1);
		status = adxcvr_read(x, ADXCVR_REG_STATUS);
		buf_err = status & (ADXCVR_BUFSTATUS_UNDERFLOW |
				    ADXCVR_BUFSTATUS_OVERFLOW);
		if (!buf_err) {
			break;
		}
		ret = adxcvr_reset(x);
		if (ret) {
			LOG_ERR("%s: reset failed clearing buffer status (%d)",
				x->name, ret);
			return ret;
		}
	} while (retry--);

	if (status & ADXCVR_BUFSTATUS_UNDERFLOW) {
		LOG_ERR("%s: buffer underflow, status=0x%x", x->name, status);
		return -EIO;
	}
	if (status & ADXCVR_BUFSTATUS_OVERFLOW) {
		LOG_ERR("%s: buffer overflow, status=0x%x", x->name, status);
		return -EIO;
	}

	LOG_INF("%s: lane clock up (status=0x%x)", x->name, status);
	return 0;
}

int axi_adxcvr_configure(void)
{
	int ret;

	ret = adxcvr_configure(&adxcvr_tx);
	if (ret) {
		return ret;
	}
	return adxcvr_configure(&adxcvr_rx);
}

int axi_adxcvr_enable(void)
{
	int ret;

	ret = adxcvr_clk_enable(&adxcvr_tx);
	if (ret) {
		return ret;
	}
	return adxcvr_clk_enable(&adxcvr_rx);
}

int axi_adxcvr_tx_enable(void)
{
	return adxcvr_clk_enable(&adxcvr_tx);
}

int axi_adxcvr_rx_enable(void)
{
	return adxcvr_clk_enable(&adxcvr_rx);
}

#ifdef CONFIG_AD9081_FAULT_INJECTION

int axi_adxcvr_fi_rx_break_refclk(void)
{
	struct adxcvr *x = &adxcvr_rx;
	uint32_t control;

	/*
	 * Point the RX GT at QPLL1, which this bitstream does not drive. The GT
	 * then has no reference clock, its PLL cannot lock and RESET_DONE never
	 * asserts -- the exact condition adxcvr_reset() reports -ETIMEDOUT for,
	 * and the one a missing FMC clock would produce.
	 *
	 * Reset is asserted first so the GT is not running on the old selection
	 * while the mux changes under it.
	 */
	adxcvr_write(x, ADXCVR_REG_RESETN, 0);

	control = (x->lpm_enable ? ADXCVR_LPM_DFE_N : 0) |
		  ADXCVR_SYSCLK_SEL(ADXCVR_SYS_CLK_QPLL1) |
		  ADXCVR_OUTCLK_SEL(x->out_clk_sel);
	adxcvr_write(x, ADXCVR_REG_CONTROL, control);

	LOG_WRN("FI: rx_adxcvr sysclk forced to QPLL1 (undriven), CONTROL=0x%04x",
		control);
	return 0;
}

int axi_adxcvr_fi_rx_restore_refclk(void)
{
	struct adxcvr *x = &adxcvr_rx;
	uint32_t control = adxcvr_control_word(x);

	adxcvr_write(x, ADXCVR_REG_RESETN, 0);
	adxcvr_write(x, ADXCVR_REG_CONTROL, control);

	/*
	 * The DRP dividers were programmed against CPLL at configure time and the
	 * break above did not touch them, so restoring the mux is enough -- but
	 * re-solve them anyway rather than assume, since a failed lock attempt is
	 * not a state this code has any right to make assumptions about.
	 */
	LOG_INF("FI: rx_adxcvr sysclk restored to CPLL, CONTROL=0x%04x", control);
	return adxcvr_clk_set_rate(x, x->lane_rate_khz, x->ref_rate_khz);
}

uint32_t axi_adxcvr_fi_rx_status(void)
{
	return adxcvr_read(&adxcvr_rx, ADXCVR_REG_STATUS);
}

#endif /* CONFIG_AD9081_FAULT_INJECTION */
