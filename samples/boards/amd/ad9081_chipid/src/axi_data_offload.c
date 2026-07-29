/*
 * ADI AXI data-offload cores -- store-and-replay buffers between each AXI DMAC
 * and its JESD204 transport core.
 *
 * See axi_data_offload.h for what this block does and why the port must
 * configure it. Register map from hdl/library/data_offload/data_offload_regmap.v.
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
LOG_MODULE_REGISTER(axi_data_offload, LOG_LEVEL_INF);

#include "axi_data_offload.h"

/*
 * Base addresses. The block design assigns 0x7c440000 / 0x7c450000
 * (ad_cpu_interconnect in ad9081_fmca_ebz_bd.tcl); this board remaps the PL
 * aperture by +0x20000000 -- the same offset that puts the DMACs at
 * 0x9c420000 / 0x9c430000 rather than 0x7c42/0x7c43.
 */
#define DO_TX_BASE 0x9C440000UL
#define DO_RX_BASE 0x9C450000UL
#define DO_SIZE    0x10000UL

/* data_offload_regmap.v decodes *word* addresses, so byte offset = word * 4. */
#define DO_REG_MAGIC        0x000C
#define DO_REG_CONFIG       0x0010
#define DO_REG_SIZE_LSB     0x0014
#define DO_REG_SIZE_MSB     0x0018
#define DO_REG_XFER_LEN     0x001C
#define DO_REG_MEM_STATUS   0x0080
#define DO_REG_RESETN       0x0084
#define DO_REG_CONTROL      0x0088
#define DO_REG_SYNC_CONFIG  0x0104
#define DO_REG_FSM_DEBUG    0x0200

/* Identification register reads "DAOF". Checked before anything else: without it
 * a readback of zeroes is indistinguishable from addressing empty space. */
#define DO_MAGIC 0x44414F46UL

#define DO_CONFIG_MEM_TYPE   BIT(0)
#define DO_CONFIG_TX_PATH    BIT(1)
#define DO_CONFIG_HAS_BYPASS BIT(2)

#define DO_STATUS_CALIB_DONE BIT(0)
#define DO_STATUS_SRC_OVF    BIT(4)
#define DO_STATUS_DST_UNF    BIT(5)

#define DO_CTRL_BYPASS  BIT(0)
#define DO_CTRL_ONESHOT BIT(1)

#define DO_FSM_WR_MASK GENMASK(4, 0)
#define DO_FSM_RD_MASK GENMASK(11, 8)

/* Beat width of the JESD stream, for turning a buffer size into a duration.
 * NP16 x 8 converters x 1 sample/channel = 128 bits. Matches what the DMAC
 * reports in INTF_DESC. */
#define DO_BEAT_BYTES  16U
#define DO_SAMPLE_RATE 250000000U

/*
 * PL apertures are not mapped by default on this platform, and these two cores
 * are not devicetree nodes, so nothing maps them for us. Same approach as the TPL
 * and XCVR cores (axi_tpl.c, axi_adxcvr.c): map at PRE_KERNEL_1 and require the
 * mapping to be identity, since the base addresses are compile-time constants
 * paired with the register offsets above -- a relocated virtual base would
 * silently address the wrong core rather than fail.
 */
static int axi_data_offload_map(void)
{
	mm_reg_t virt;

	device_map(&virt, DO_TX_BASE, DO_SIZE, K_MEM_CACHE_NONE);
	if (virt != DO_TX_BASE) {
		LOG_ERR("TX offload not identity-mapped: virt=0x%lx phys=0x%lx",
			(unsigned long)virt, DO_TX_BASE);
		return -EIO;
	}

	device_map(&virt, DO_RX_BASE, DO_SIZE, K_MEM_CACHE_NONE);
	if (virt != DO_RX_BASE) {
		LOG_ERR("RX offload not identity-mapped: virt=0x%lx phys=0x%lx",
			(unsigned long)virt, DO_RX_BASE);
		return -EIO;
	}
	return 0;
}

SYS_INIT(axi_data_offload_map, PRE_KERNEL_1, 0);

static bool do_present(uintptr_t base, const char *tag)
{
	uint32_t magic = sys_read32(base + DO_REG_MAGIC);

	if (magic != DO_MAGIC) {
		LOG_ERR("%s @ 0x%08lx: MAGIC 0x%08x, expected 0x%08x (\"DAOF\")", tag,
			(unsigned long)base, magic, (unsigned int)DO_MAGIC);
		return false;
	}
	return true;
}

static int do_set_bypass(uintptr_t base, const char *tag, bool enable)
{
	uint32_t cfg, ctrl, rb;

	if (!do_present(base, tag)) {
		return -ENODEV;
	}

	cfg = sys_read32(base + DO_REG_CONFIG);
	if (!(cfg & DO_CONFIG_HAS_BYPASS)) {
		LOG_ERR("%s: built without bypass support (HAS_BYPASS=0), so the bit is",
			tag);
		LOG_ERR("%s: tied off. Continuous streaming needs either a bitstream with",
			tag);
		LOG_ERR("%s: bypass, or a transfer length short enough that fill and drain",
			tag);
		LOG_ERR("%s: overlap (DO_REG_XFER_LEN).", tag);
		return -ENOTSUP;
	}

	ctrl = sys_read32(base + DO_REG_CONTROL);
	if (enable) {
		ctrl |= DO_CTRL_BYPASS;
	} else {
		ctrl &= ~DO_CTRL_BYPASS;
	}
	sys_write32(ctrl, base + DO_REG_CONTROL);

	/* Read back: a mode bit that silently fails to take would leave the caller
	 * believing the datapath is continuous when it is still gated -- the exact
	 * confusion this whole investigation was. */
	rb = sys_read32(base + DO_REG_CONTROL);
	if (((rb & DO_CTRL_BYPASS) != 0) != enable) {
		LOG_ERR("%s: bypass write did not take (control reads 0x%08x)", tag, rb);
		return -EIO;
	}
	return 0;
}

int axi_data_offload_bypass(bool enable)
{
	int rc_tx, rc_rx;

	/*
	 * Both directions, and both attempted even if the first fails: knowing that
	 * one core took the mode and the other did not is more useful than stopping
	 * at the first error, because the two paths fail for different reasons (the
	 * RX core is one-shot by default, the TX core cyclic).
	 */
	rc_tx = do_set_bypass(DO_TX_BASE, "TX offload", enable);
	rc_rx = do_set_bypass(DO_RX_BASE, "RX offload", enable);

	if (rc_tx || rc_rx) {
		return rc_tx ? rc_tx : rc_rx;
	}

	if (enable) {
		LOG_INF("both offload cores in bypass: the datapath streams continuously");
		LOG_INF("  instead of replaying a 1 MiB buffer (which gated TX to ~9%% and");
		LOG_INF("  truncated RX captures at 65536 beats)");
	} else {
		LOG_INF("both offload cores in store-and-replay mode");
	}
	return 0;
}

static void do_dump(uintptr_t base, const char *tag)
{
	uint32_t cfg, xfer, status, resetn, ctrl, fsm;
	uint64_t size;

	if (!do_present(base, tag)) {
		return;
	}

	cfg = sys_read32(base + DO_REG_CONFIG);
	xfer = sys_read32(base + DO_REG_XFER_LEN);
	status = sys_read32(base + DO_REG_MEM_STATUS);
	resetn = sys_read32(base + DO_REG_RESETN);
	ctrl = sys_read32(base + DO_REG_CONTROL);
	fsm = sys_read32(base + DO_REG_FSM_DEBUG);

	size = ((uint64_t)(sys_read32(base + DO_REG_SIZE_MSB) & 0x3U) << 32) |
	       sys_read32(base + DO_REG_SIZE_LSB);

	LOG_INF("%s @ 0x%08lx:", tag, (unsigned long)base);
	LOG_INF("  size %llu B = %llu beats = %llu us at %u MSPS",
		(unsigned long long)size,
		(unsigned long long)(size / DO_BEAT_BYTES),
		(unsigned long long)(size / DO_BEAT_BYTES * 1000000ULL / DO_SAMPLE_RATE),
		DO_SAMPLE_RATE / 1000000U);
	LOG_INF("  path %s, mem %s, bypass %s",
		(cfg & DO_CONFIG_TX_PATH) ? "TX" : "RX",
		(cfg & DO_CONFIG_MEM_TYPE) ? "external" : "FPGA RAM",
		(cfg & DO_CONFIG_HAS_BYPASS) ? "supported" : "NOT SUPPORTED");
	LOG_INF("  resetn %u (%s), bypass %u, oneshot %u, xfer_len 0x%08x",
		(unsigned int)(resetn & BIT(0)),
		(resetn & BIT(0)) ? "running" : "HELD IN RESET",
		(unsigned int)((ctrl & DO_CTRL_BYPASS) != 0),
		(unsigned int)((ctrl & DO_CTRL_ONESHOT) != 0), xfer);
	LOG_INF("  FSM wr 0x%02x rd 0x%01x, sync_cfg 0x%08x",
		(unsigned int)FIELD_GET(DO_FSM_WR_MASK, fsm),
		(unsigned int)FIELD_GET(DO_FSM_RD_MASK, fsm),
		sys_read32(base + DO_REG_SYNC_CONFIG));

	if (!(resetn & BIT(0))) {
		LOG_ERR("  held in reset -- this core passes nothing");
	}
	if (status & DO_STATUS_SRC_OVF) {
		LOG_WRN("  SRC_OVERFLOW latched: the producer outran this buffer");
	}
	if (status & DO_STATUS_DST_UNF) {
		LOG_WRN("  DST_UNDERFLOW latched: the consumer asked for samples this");
		LOG_WRN("  buffer did not have -- on TX that means the DAC ran dry");
	}
	if ((cfg & DO_CONFIG_MEM_TYPE) && !(status & DO_STATUS_CALIB_DONE)) {
		LOG_ERR("  external memory but calibration not done -- storage unusable");
	}
}

int axi_data_offload_status(void)
{
	LOG_INF("--- AXI data-offload cores ---");
	do_dump(DO_TX_BASE, "TX offload");
	do_dump(DO_RX_BASE, "RX offload");
	return 0;
}
