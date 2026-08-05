/*
 * Copyright (c) 2026 Analog Devices, Inc.
 *
 * AXI ADXCVR -- GT transceiver (PHY) bring-up, minimal fixed-rate variant.
 *
 * This drives the ADI AXI-ADXCVR wrapper around the Xilinx GTH4 transceivers on
 * ZynqMP for the AD9081 JESD204B link (TX -> QPLL0, RX -> CPLL, out clock via
 * PROGDIV, 10 Gbps lanes off a 500 MHz refclk). One devicetree node per
 * direction; everything that used to be a file-scope singleton is now that
 * node's config.
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

#define DT_DRV_COMPAT adi_axi_adxcvr_1_0

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/init.h>
#include <zephyr/kernel/mm.h>
#include <zephyr/sys/device_mmio.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(axi_adxcvr, LOG_LEVEL_INF);

#include <zephyr/drivers/misc/jesd204/axi_adxcvr.h>
#include <zephyr/dt-bindings/jesd204/adxcvr.h>
#include <zephyr/drivers/clock_control/hmc7044.h>
#include "xilinx_transceiver.h"

/*
 * virt == phys for every PL page this file touches: the bases come straight from
 * devicetree reg and are used as addresses. device_map() only returns the
 * physical address when the kernel maps 1:1.
 */
BUILD_ASSERT(IS_ENABLED(CONFIG_KERNEL_DIRECT_MAP),
	     "adxcvr assumes virt == phys for the PL pages");

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

/*
 * sys_clk_sel / out_clk_sel values. These come from devicetree now, where they
 * are spelled XCVR_CPLL / XCVR_QPLL / XCVR_PROGDIV_CLK
 * (<zephyr/dt-bindings/jesd204/adxcvr.h>) -- the same numbers ADI's Linux devicetrees
 * use, which are the IP's REG_CONTROL field encodings. Kept here only for the
 * two places the driver has to reason about the choice rather than write it.
 */
#define ADXCVR_SYS_CLK_CPLL  0x00
#define ADXCVR_SYS_CLK_QPLL1 0x02
#define ADXCVR_PROGDIV_CLK   5

/* xcvr_type: UltraScale+ GTH4 as reported in REG_SYNTH for this board. */
#define XILINX_XCVR_TYPE_US_GTH4 8

/* PCORE version helpers -- buffer-status handling exists from 17.5a. */
#define ADXCVR_PCORE_VER(major, minor, patch) \
	(((major) << 16) | ((minor) << 8) | (patch))
#define PCORE_VER_MAJOR(x) (((x) >> 16) & 0xff)
#define PCORE_VER_MINOR(x) (((x) >> 8) & 0xff)
#define PCORE_VER_PATCH(x) ((x) & 0xff)

/* Expected GT refclk, kept only to sanity-check what the clock tree reports. */
#define ADXCVR_REF_CLK_KHZ_EXPECTED 500000

/* Devicetree configuration -- ROM, one per node. */
struct adxcvr_config {
	uintptr_t base;
	const struct device *refclk_dev;
	clock_control_subsys_t refclk_subsys;
	uint32_t refclk_out;
	uint32_t lane_rate_khz;
	uint8_t sys_clk_sel;
	uint8_t out_clk_sel;
	bool lpm_enable;
	/* VCO range overrides; 0 means "use the derived range". */
	uint32_t vco0_min;
	uint32_t vco0_max;
	uint32_t vco1_min;
	uint32_t vco1_max;
};

/*
 * Per-instance state -- RAM. Named `struct adxcvr` because that is the name the
 * verbatim xilinx_transceiver.c holds a pointer to (forward-declared via
 * xcvr_shim.h); it only ever passes it back to adxcvr_drp_read/write below, so
 * the definition living here is fine and both vendor files stay byte-identical
 * to no-OS.
 */
struct adxcvr {
	/* Back-pointer, so the DRP accessors can reach config and the name. */
	const struct device *dev;
	bool cpll_enable;
	bool qpll_enable;
	/* GT reference rate the DRP divider solve targets (kHz). */
	uint32_t ref_rate_khz;
	/* read back from the core */
	uint32_t version;
	uint32_t num_lanes;
	bool tx_enable;
	uint32_t xcvr_type;
	/* Xilinx GT reconfiguration state (drives the verbatim DRP math). */
	struct xilinx_xcvr xlx_xcvr;
};

static inline uint32_t adxcvr_read(const struct device *dev, uint32_t reg)
{
	const struct adxcvr_config *cfg = dev->config;

	return sys_read32(cfg->base + reg);
}

static inline void adxcvr_write(const struct device *dev, uint32_t reg,
				uint32_t val)
{
	const struct adxcvr_config *cfg = dev->config;

	sys_write32(val, cfg->base + reg);
}

/*
 * DRP (Dynamic Reconfiguration Port) accessors. The verbatim GT divider math in
 * xilinx_transceiver.c reaches the transceiver only through these two functions
 * (declared in xcvr_shim.h), passing back the struct adxcvr * it was handed.
 * Ports of no-OS adxcvr_drp_read/write + adxcvr_drp_wait_idle; plain AXI MMIO
 * under the hood.
 */
static int adxcvr_drp_wait_idle(struct adxcvr *x, uint32_t drp_addr)
{
	uint32_t val;
	int timeout = 20;

	while (timeout-- > 0) {
		val = adxcvr_read(x->dev, ADXCVR_REG_DRP_STATUS(drp_addr));
		if (!(val & ADXCVR_DRP_STATUS_BUSY)) {
			return ADXCVR_DRP_STATUS_RDATA(val);
		}
		k_msleep(1);
	}

	LOG_ERR("%s: DRP wait idle timeout", x->dev->name);
	return -ETIMEDOUT;
}

int adxcvr_drp_read(struct adxcvr *x, unsigned int drp_port,
		    unsigned int reg, unsigned int *val)
{
	uint32_t drp_addr = (drp_port < ADXCVR_DRP_PORT_CHANNEL(0)) ?
			    ADXCVR_DRP_PORT_ADDR_COMMON :
			    ADXCVR_DRP_PORT_ADDR_CHANNEL;
	int ret;

	adxcvr_write(x->dev, ADXCVR_REG_DRP_SEL(drp_addr), drp_port & 0xFF);
	adxcvr_write(x->dev, ADXCVR_REG_DRP_CTRL(drp_addr),
		     ADXCVR_DRP_CTRL_ADDR(reg));

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

	adxcvr_write(x->dev, ADXCVR_REG_DRP_SEL(drp_addr), drp_port & 0xFF);
	adxcvr_write(x->dev, ADXCVR_REG_DRP_CTRL(drp_addr),
		     ADXCVR_DRP_CTRL_WR | ADXCVR_DRP_CTRL_ADDR(reg) |
		     ADXCVR_DRP_CTRL_WDATA(val));

	ret = adxcvr_drp_wait_idle(x, drp_addr);
	if (ret < 0) {
		return ret;
	}
	return 0;
}

/*
 * 1:1 non-cached mapping of the PL pages. Not in the A53 SoC MMU table, and an
 * AXI access to an unmapped PL page faults, so this has to happen before any
 * register touch.
 *
 * Still a SYS_INIT over both nodes rather than DEVICE_MMIO_MAP in each device's
 * own init(): that swap is a behaviour change (different init level, different
 * failure reporting) and this commit is the structural half. See PLAN step 3.
 */
#define ADXCVR_MAP_ONE(inst)                                                                       \
	do {                                                                                       \
		mm_reg_t virt;                                                                     \
		uintptr_t phys = DT_INST_REG_ADDR(inst);                                           \
                                                                                                   \
		device_map(&virt, phys, DT_INST_REG_SIZE(inst), K_MEM_CACHE_NONE);                 \
		if (virt != phys) {                                                                \
			LOG_ERR("%s not identity-mapped: virt=0x%lx phys=0x%lx",                   \
				DT_NODE_FULL_NAME(DT_DRV_INST(inst)), (unsigned long)virt,         \
				(unsigned long)phys);                                              \
			return -EIO;                                                               \
		}                                                                                  \
	} while (0);

static int axi_adxcvr_map(void)
{
	DT_INST_FOREACH_STATUS_OKAY(ADXCVR_MAP_ONE)
	return 0;
}

SYS_INIT(axi_adxcvr_map, PRE_KERNEL_1, 0);

/*
 * Read the FPGA identity registers into xlx_xcvr. The verbatim VCO-range setup
 * (xilinx_xcvr_setup_cpll/qpll_vco_range) consults tech/family/speed/package/
 * voltage when PCORE major > 0x10, so they must be populated. Port of no-OS
 * adxcvr_get_info().
 */
static void adxcvr_get_info(const struct device *dev)
{
	struct adxcvr *x = dev->data;
	uint32_t info = adxcvr_read(dev, ADXCVR_REG_FPGA_INFO);
	uint32_t volt = adxcvr_read(dev, ADXCVR_REG_FPGA_VOLTAGE);

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
static int adxcvr_clk_set_rate(const struct device *dev, unsigned long rate,
			       unsigned long parent_rate)
{
	const struct adxcvr_config *cfg = dev->config;
	struct adxcvr *x = dev->data;
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
		ret = xilinx_xcvr_calc_qpll_config(&x->xlx_xcvr, cfg->sys_clk_sel,
						   parent_rate, rate, &qpll_conf,
						   &out_div);
	}
	if (ret < 0) {
		LOG_ERR("%s: no %s config for %lu kHz @ ref %lu kHz", dev->name,
			x->cpll_enable ? "CPLL" : "QPLL", rate, parent_rate);
		return ret;
	}

	for (i = 0; i < x->num_lanes; i++) {
		if (x->cpll_enable) {
			ret = xilinx_xcvr_cpll_write_config(&x->xlx_xcvr,
					ADXCVR_DRP_PORT_CHANNEL(i), &cpll_conf);
		} else if ((i % 4 == 0) && x->qpll_enable) {
			ret = xilinx_xcvr_qpll_write_config(&x->xlx_xcvr,
					cfg->sys_clk_sel,
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

		if (cfg->out_clk_sel == ADXCVR_PROGDIV_CLK) {
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
					dev->name, out_div);
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
					cfg->lpm_enable);
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

	return 0;
}

/* REG_CONTROL as this instance's devicetree configuration says it should be. */
static uint32_t adxcvr_control_word(const struct device *dev)
{
	const struct adxcvr_config *cfg = dev->config;

	return (cfg->lpm_enable ? ADXCVR_LPM_DFE_N : 0) |
	       ADXCVR_SYSCLK_SEL(cfg->sys_clk_sel) |
	       ADXCVR_OUTCLK_SEL(cfg->out_clk_sel);
}

/*
 * Fill in ref_rate_khz from the clock tree rather than from a #define. The GT
 * DRP divider solve in xilinx_transceiver.c is entirely driven by the ratio of
 * the lane rate to the reference rate, so a refclk that does not match the
 * hardware silently produces wrong dividers -- and a GT that never reports
 * ready, with nothing in the log pointing at the reference clock.
 *
 * `clocks = <&hmc7044 12>` names the output wired to the GT refclk input; the
 * HMC7044 driver knows its rate because it programmed the divider.
 */
static const struct device *logged_refclk_dev;
static uint32_t logged_refclk_out;

static int adxcvr_query_ref_rate(const struct device *dev)
{
	const struct adxcvr_config *cfg = dev->config;
	struct adxcvr *x = dev->data;
	uint32_t rate_hz;
	uint32_t rate_khz;
	int ret;

	if (!device_is_ready(cfg->refclk_dev)) {
		LOG_ERR("GT refclk provider %s is not ready",
			cfg->refclk_dev->name);
		return -ENODEV;
	}

	ret = clock_control_get_rate(cfg->refclk_dev, cfg->refclk_subsys,
				     &rate_hz);
	if (ret) {
		LOG_ERR("could not read the GT refclk rate from %s out%u (%d)",
			cfg->refclk_dev->name, cfg->refclk_out, ret);
		return ret;
	}

	rate_khz = rate_hz / 1000U;
	if (rate_khz == 0) {
		LOG_ERR("GT refclk reported as 0 Hz");
		return -EINVAL;
	}

	if (rate_khz != ADXCVR_REF_CLK_KHZ_EXPECTED) {
		/*
		 * Not an error: the queried rate is the authoritative one and is
		 * what gets used. But this bitstream's GT attributes were
		 * synthesised for 500 MHz, so a different tree is worth saying
		 * out loud rather than discovering as a GT that will not lock.
		 */
		LOG_WRN("GT refclk is %u kHz, but this bitstream was synthesised "
			"for %u kHz -- using the queried rate",
			rate_khz, ADXCVR_REF_CLK_KHZ_EXPECTED);
	}

	x->ref_rate_khz = rate_khz;

	/*
	 * Reported once per distinct clock output, not once per transceiver: the
	 * rate is a property of the HMC7044 output, and both directions take the
	 * same one here, so a line per instance would just repeat itself. The
	 * comparison (rather than a plain "first time" flag) keeps the report
	 * correct if the two ever point at different outputs.
	 */
	if (logged_refclk_dev != cfg->refclk_dev ||
	    logged_refclk_out != cfg->refclk_out) {
		logged_refclk_dev = cfg->refclk_dev;
		logged_refclk_out = cfg->refclk_out;
		LOG_INF("GT refclk from %s out%u: %u kHz", cfg->refclk_dev->name,
			cfg->refclk_out, rate_khz);
	}

	return 0;
}

int axi_adxcvr_configure(const struct device *dev)
{
	const struct adxcvr_config *cfg;
	struct adxcvr *x;
	uint32_t synth;
	uint32_t control;
	int ret;

	if (!device_is_ready(dev)) {
		return -ENODEV;
	}
	cfg = dev->config;
	x = dev->data;

	/* Must precede the divider solve below -- it is one of its two inputs. */
	ret = adxcvr_query_ref_rate(dev);
	if (ret) {
		return ret;
	}

	synth = adxcvr_read(dev, ADXCVR_REG_SYNTH);
	x->version = adxcvr_read(dev, ADXCVR_REG_VERSION);
	x->num_lanes = ADXCVR_SYNTH_NUM_LANES(synth);
	x->tx_enable = ADXCVR_SYNTH_TX_ENABLE(synth);
	x->xcvr_type = ADXCVR_SYNTH_XCVR_TYPE(synth);
	x->qpll_enable = (synth >> 20) & 1;
	x->cpll_enable = (cfg->sys_clk_sel == ADXCVR_SYS_CLK_CPLL);

	LOG_INF("%s @ 0x%08lx: PCORE v%u.%u.%u, %u lanes, type=%u, %s",
		dev->name, (unsigned long)cfg->base,
		PCORE_VER_MAJOR(x->version), PCORE_VER_MINOR(x->version),
		PCORE_VER_PATCH(x->version), x->num_lanes, x->xcvr_type,
		x->tx_enable ? "TX" : "RX");

	if (x->xcvr_type != XILINX_XCVR_TYPE_US_GTH4) {
		LOG_WRN("%s: xcvr_type=%u, expected GTH4(%u) -- clock mux/PROGDIV "
			"assumptions may not hold", dev->name, x->xcvr_type,
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
	/* Zero means "derive it", which is what the vendor code does with 0. */
	x->xlx_xcvr.vco0_min = cfg->vco0_min;
	x->xlx_xcvr.vco0_max = cfg->vco0_max;
	x->xlx_xcvr.vco1_min = cfg->vco1_min;
	x->xlx_xcvr.vco1_max = cfg->vco1_max;
	if (PCORE_VER_MAJOR(x->version) > 0x10) {
		adxcvr_get_info(dev);
	}

	/* Assert reset while we set the clock selection. */
	adxcvr_write(dev, ADXCVR_REG_RESETN, 0);

	control = adxcvr_control_word(dev);
	adxcvr_write(dev, ADXCVR_REG_CONTROL, control);

	LOG_INF("%s: CONTROL=0x%04x (sysclk=%u outclk=%u lpm=%u) enc=%s",
		dev->name, control, cfg->sys_clk_sel, cfg->out_clk_sel,
		cfg->lpm_enable,
		x->xlx_xcvr.encoding == ENC_66B64B ? "64b/66b" : "8b/10b");

	/*
	 * Program the GT dividers over DRP (no-OS did this in adxcvr_init). Held
	 * in reset above; the FSM releases reset later in axi_adxcvr_enable().
	 */
	ret = adxcvr_clk_set_rate(dev, cfg->lane_rate_khz, x->ref_rate_khz);
	if (ret) {
		LOG_ERR("%s: GT divider programming failed (%d)", dev->name, ret);
		return ret;
	}
	LOG_INF("%s: GT dividers programmed for %u kHz lane @ %u kHz ref",
		dev->name, cfg->lane_rate_khz, x->ref_rate_khz);

	return 0;
}

/*
 * Pulse the core reset and poll for transceiver-ready. Mirrors no-OS
 * adxcvr_reset()/adxcvr_status_error(): up to two reset attempts, each polling
 * STATUS bit0 for <=100 ms. Returns 0 when ready, -ETIMEDOUT otherwise.
 */
static int adxcvr_reset(const struct device *dev)
{
	int retry = 1;
	uint32_t status = 0;

	do {
		int timeout = 100;

		adxcvr_write(dev, ADXCVR_REG_RESETN, 0);
		k_busy_wait(2);
		adxcvr_write(dev, ADXCVR_REG_RESETN, ADXCVR_RESETN);

		while (timeout--) {
			k_msleep(1);
			status = adxcvr_read(dev, ADXCVR_REG_STATUS);
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
			"check the reference clock", dev->name, status);
	} else {
		LOG_ERR("%s: RESET_DONE not set after 2x100ms (raw STATUS=0x%08x)",
			dev->name, status);
	}
	return -ETIMEDOUT;
}

/*
 * Bring one transceiver's lane clock up. Mirrors no-OS adxcvr_clk_enable():
 * reset+status, then (on PCORE >= 17.5a) clear elastic-buffer under/overflow.
 */
int axi_adxcvr_enable(const struct device *dev)
{
	struct adxcvr *x;
	uint32_t status;
	int retry = 100;
	int ret;

	if (!device_is_ready(dev)) {
		return -ENODEV;
	}
	x = dev->data;

	ret = adxcvr_reset(dev);
	if (ret) {
		LOG_ERR("%s: transceiver not ready after reset (%d)",
			dev->name, ret);
		return ret;
	}

	if (x->version < ADXCVR_PCORE_VER(17, 5, 'a')) {
		LOG_INF("%s: lane clock up", dev->name);
		return 0;
	}

	do {
		bool buf_err;

		adxcvr_write(dev, ADXCVR_REG_RESETN,
			     ADXCVR_BUFSTATUS_RST | ADXCVR_RESETN);
		adxcvr_write(dev, ADXCVR_REG_RESETN, ADXCVR_RESETN);
		k_msleep(1);
		status = adxcvr_read(dev, ADXCVR_REG_STATUS);
		buf_err = status & (ADXCVR_BUFSTATUS_UNDERFLOW |
				    ADXCVR_BUFSTATUS_OVERFLOW);
		if (!buf_err) {
			break;
		}
		ret = adxcvr_reset(dev);
		if (ret) {
			LOG_ERR("%s: reset failed clearing buffer status (%d)",
				dev->name, ret);
			return ret;
		}
	} while (retry--);

	if (status & ADXCVR_BUFSTATUS_UNDERFLOW) {
		LOG_ERR("%s: buffer underflow, status=0x%x", dev->name, status);
		return -EIO;
	}
	if (status & ADXCVR_BUFSTATUS_OVERFLOW) {
		LOG_ERR("%s: buffer overflow, status=0x%x", dev->name, status);
		return -EIO;
	}

	LOG_INF("%s: lane clock up (status=0x%x)", dev->name, status);
	return 0;
}

/*
 * Nothing to do at init: configure() needs the GT reference clock rate, and the
 * HMC7044 that provides it is SPI-attached and so programs itself at
 * POST_KERNEL. Reading its rate from an init() would read an unprogrammed chip
 * whatever priority this driver picked. The device exists so that devicetree can
 * describe it and device_is_ready() means something; the work is an explicit
 * axi_adxcvr_configure() call from main(), in the order the bring-up needs.
 */
static int adxcvr_init(const struct device *dev)
{
	struct adxcvr *x = dev->data;

	x->dev = dev;
	return 0;
}

/*
 * CPLL has one VCO and takes adi,vco-{min,max}-khz; QPLL has two and takes the
 * vco0/vco1 pairs. Both fold into one field pair below, as the vendor struct
 * has, so the CPLL/QPLL choice stays in adi,sys-clk-select alone.
 */
#define ADXCVR_DEFINE(inst)                                                                        \
	BUILD_ASSERT(DT_INST_PROP(inst, adi_out_clk_select) != XCVR_OUTCLK_PCS &&                  \
			     DT_INST_PROP(inst, adi_out_clk_select) != XCVR_OUTCLK_PMA,            \
		     "PCS/PMA out-clk-select is not ported: the vendor DRP "                       \
		     "path only programs dividers for REFCLK and PROGDIV");                        \
                                                                                                   \
	static struct adxcvr adxcvr_data_##inst;                                                   \
                                                                                                   \
	static const struct adxcvr_config adxcvr_config_##inst = {                                 \
		.base = DT_INST_REG_ADDR(inst),                                                    \
		.refclk_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(inst)),                            \
		.refclk_out = DT_INST_CLOCKS_CELL(inst, output),                                   \
		.refclk_subsys = HMC7044_CLK_OUT(DT_INST_CLOCKS_CELL(inst, output)),               \
		.lane_rate_khz = DT_INST_PROP(inst, adi_lane_rate_khz),                            \
		.sys_clk_sel = DT_INST_PROP(inst, adi_sys_clk_select),                             \
		.out_clk_sel = DT_INST_PROP(inst, adi_out_clk_select),                             \
		.lpm_enable = DT_INST_PROP(inst, adi_use_lpm_enable),                              \
		.vco0_min = DT_INST_PROP_OR(inst, adi_vco_min_khz,                                 \
					    DT_INST_PROP_OR(inst, adi_vco0_min_khz, 0)),           \
		.vco0_max = DT_INST_PROP_OR(inst, adi_vco_max_khz,                                 \
					    DT_INST_PROP_OR(inst, adi_vco0_max_khz, 0)),           \
		.vco1_min = DT_INST_PROP_OR(inst, adi_vco1_min_khz, 0),                            \
		.vco1_max = DT_INST_PROP_OR(inst, adi_vco1_max_khz, 0),                            \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, adxcvr_init, NULL, &adxcvr_data_##inst, &adxcvr_config_##inst, \
			      POST_KERNEL, CONFIG_JESD204_AXI_ADXCVR_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(ADXCVR_DEFINE)

/*
 * Not a preference: the nodes declare `clocks = <&hmc7044 N>`, so Zephyr's
 * scripts/build/check_init_priorities.py fails the build if this driver runs
 * before the clock provider. The HMC7044 in turn asserts it is above
 * CONFIG_SPI_INIT_PRIORITY.
 */
BUILD_ASSERT(CONFIG_JESD204_AXI_ADXCVR_INIT_PRIORITY > CONFIG_CLOCK_CONTROL_HMC7044_INIT_PRIORITY,
	     "adxcvr must initialise after its GT reference clock provider");
