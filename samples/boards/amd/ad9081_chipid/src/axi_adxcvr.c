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
#define ADXCVR_REG_RESETN  0x0010
#define ADXCVR_REG_STATUS  0x0014
#define ADXCVR_REG_CONTROL 0x0020
#define ADXCVR_REG_SYNTH   0x0024

/* REG_RESETN bits. */
#define ADXCVR_RESETN        BIT(0)
#define ADXCVR_BUFSTATUS_RST BIT(1)

/* REG_STATUS bits. */
#define ADXCVR_STATUS              BIT(0)
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

/* One GT transceiver instance (TX or RX side). */
struct adxcvr {
	const char *name;
	uintptr_t base;
	uint8_t sys_clk_sel;
	uint8_t out_clk_sel;
	bool lpm_enable;
	/* read back from the core */
	uint32_t version;
	uint32_t num_lanes;
	bool tx_enable;
	uint32_t xcvr_type;
};

static struct adxcvr adxcvr_tx = {
	.name = "tx_adxcvr",
	.base = ADXCVR_TX_BASE,
	.sys_clk_sel = ADXCVR_SYS_CLK_QPLL0,
	.out_clk_sel = ADXCVR_PROGDIV_CLK,
	.lpm_enable = false,
};

static struct adxcvr adxcvr_rx = {
	.name = "rx_adxcvr",
	.base = ADXCVR_RX_BASE,
	.sys_clk_sel = ADXCVR_SYS_CLK_CPLL,
	.out_clk_sel = ADXCVR_PROGDIV_CLK,
	.lpm_enable = true,
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
static int adxcvr_configure(struct adxcvr *x)
{
	uint32_t synth = adxcvr_read(x, ADXCVR_REG_SYNTH);
	uint32_t control;

	x->version = adxcvr_read(x, ADXCVR_REG_VERSION);
	x->num_lanes = ADXCVR_SYNTH_NUM_LANES(synth);
	x->tx_enable = ADXCVR_SYNTH_TX_ENABLE(synth);
	x->xcvr_type = ADXCVR_SYNTH_XCVR_TYPE(synth);

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

	/* Assert reset while we set the clock selection. */
	adxcvr_write(x, ADXCVR_REG_RESETN, 0);

	control = (x->lpm_enable ? ADXCVR_LPM_DFE_N : 0) |
		  ADXCVR_SYSCLK_SEL(x->sys_clk_sel) |
		  ADXCVR_OUTCLK_SEL(x->out_clk_sel);
	adxcvr_write(x, ADXCVR_REG_CONTROL, control);

	LOG_INF("%s: CONTROL=0x%04x (sysclk=%u outclk=%u lpm=%u)",
		x->name, control, x->sys_clk_sel, x->out_clk_sel,
		x->lpm_enable);

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
	 * RESET_DONE never asserted. The raw STATUS word tells us why: 0x0 means
	 * the GT PLL never locked -- almost always a missing/wrong reference clock
	 * at the transceiver (QPLL0 for TX, CPLL for RX) rather than a reset-pulse
	 * problem. A nonzero value with bit0 clear points at the elastic buffer.
	 */
	LOG_ERR("%s: RESET_DONE not set after 2x100ms (raw STATUS=0x%08x)",
		x->name, status);
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
