/*
 * AXI AD9081 TPL (transport-layer) cores -- configure-only.
 *
 * The TPL cores sit between the JESD204 link cores and the DMA: they map link
 * octets to/from converter samples and own the per-converter datapath (sample
 * format, data-source select, test patterns). This is the generic ADI "AXI
 * ADC/DAC" core, instantiated as adc_tpl_core (RX) and dac_tpl_core (TX).
 *
 * Unlike the serdes/link cores, the TPL runs on the device sample clock and the
 * AXI bus, so almost everything here is pure register configuration, safe at
 * boot. The one link-dependent part -- the STATUS/CLK verification, which only
 * reads valid once the lanes are running -- is split into axi_tpl_enable() and
 * driven later by the bring-up sequence.
 *
 * Register offsets, field packing and the init sequence are transcribed from
 * no-OS axi_adc_core.c / axi_dac_core.c. Base addresses come from the on-board
 * bitstream (system.hwh): RX_CORE 0x84A10000, TX_CORE 0x84B10000.
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
LOG_MODULE_REGISTER(axi_tpl, LOG_LEVEL_INF);

#include "axi_tpl.h"

/* AXI base addresses (on-board bitstream system.hwh). */
#define TPL_RX_BASE 0x84A10000UL /* rx_mxfe_tpl_core_adc_tpl_core */
#define TPL_TX_BASE 0x84B10000UL /* tx_mxfe_tpl_core_dac_tpl_core */
#define TPL_SIZE    0x10000UL

/* Number of converters on this link (M8). One datapath channel each. */
#define TPL_NUM_CHANNELS 8

/* Common core registers (same offsets on ADC and DAC cores). */
#define TPL_REG_VERSION   0x0000 /* pcore version in bits [31:16] */
#define TPL_REG_RSTN      0x0040
#define TPL_REG_CLK_FREQ  0x0054
#define TPL_REG_CLK_RATIO 0x0058
#define TPL_REG_STATUS    0x005C

#define TPL_MMCM_RSTN BIT(1)
#define TPL_RSTN      BIT(0)

/* ADC (RX) core specifics. */
#define ADC_REG_CHAN_CNTRL(c) (0x0400 + (c) * 0x40)
#define ADC_CHAN_ENABLE         BIT(0)
#define ADC_CHAN_FORMAT_ENABLE  BIT(4)
#define ADC_CHAN_FORMAT_SIGNEXT BIT(6)

/* DAC (TX) core specifics. */
#define DAC_REG_SYNC_CONTROL  0x0044
#define DAC_SYNC              BIT(0)
#define DAC_REG_DATA_SELECT(c) (0x0418 + (c) * 0x40) /* low nibble = source */
#define DAC_DATA_SEL(x)       ((x) & 0xf)

/* DAC data-source select codes (axi_dac_core.h). */
#define DAC_DATA_SEL_DDS 0 /* internal tone generator */
#define DAC_DATA_SEL_DMA 2 /* converter samples from the DMA/JESD datapath */

/*
 * DDS tone generator, one pair of registers per DDS (two DDSs per converter, so
 * the indexing is (dds >> 1) * 0x40 + (dds & 1) * 8). Same layout as no-OS
 * axi_dac_core.c. SCALE is a 1.14 fixed-point amplitude; INIT/INCR hold the phase
 * offset in the high half and the per-sample phase increment in the low half,
 * where a full 0xFFFF increment is one cycle per sample.
 */
#define DAC_REG_DDS_SCALE(d)     (0x0400 + ((d) >> 1) * 0x40 + ((d) & 1) * 0x8)
#define DAC_REG_DDS_INIT_INCR(d) (0x0404 + ((d) >> 1) * 0x40 + ((d) & 1) * 0x8)
#define DAC_DDS_SCALE_1_0        0x4000 /* 1.14 format: 0x4000 == 1.0 */
#define DAC_DDS_INIT(x)          (((uint32_t)(x) & 0xFFFFU) << 16)
#define DAC_DDS_INCR(x)          ((uint32_t)(x) & 0xFFFFU)

/* PCORE version helper (ADC core requires major >= 9). */
#define TPL_PCORE_VER_MAJOR(x) (((x) >> 16) & 0xffff)

struct axi_tpl {
	const char *name;
	uintptr_t base;
	bool tx; /* true = DAC deframer, false = ADC framer */
	uint32_t version;
};

static struct axi_tpl tpl_rx = {
	.name = "adc-tpl",
	.base = TPL_RX_BASE,
	.tx = false,
};

static struct axi_tpl tpl_tx = {
	.name = "dac-tpl",
	.base = TPL_TX_BASE,
	.tx = true,
};

static inline uint32_t tpl_read(const struct axi_tpl *t, uint32_t reg)
{
	return sys_read32(t->base + reg);
}

static inline void tpl_write(const struct axi_tpl *t, uint32_t reg,
			     uint32_t val)
{
	sys_write32(val, t->base + reg);
}

static int axi_tpl_map(void)
{
	mm_reg_t virt;

	device_map(&virt, TPL_RX_BASE, TPL_SIZE, K_MEM_CACHE_NONE);
	if (virt != TPL_RX_BASE) {
		LOG_ERR("RX TPL not identity-mapped: virt=0x%lx phys=0x%lx",
			(unsigned long)virt, TPL_RX_BASE);
		return -EIO;
	}

	device_map(&virt, TPL_TX_BASE, TPL_SIZE, K_MEM_CACHE_NONE);
	if (virt != TPL_TX_BASE) {
		LOG_ERR("TX TPL not identity-mapped: virt=0x%lx phys=0x%lx",
			(unsigned long)virt, TPL_TX_BASE);
		return -EIO;
	}
	return 0;
}

SYS_INIT(axi_tpl_map, PRE_KERNEL_1, 0);

/* Reset the core: assert, then deassert MMCM + core reset (no-OS init step 1-2). */
static void tpl_reset(const struct axi_tpl *t)
{
	tpl_write(t, TPL_REG_RSTN, 0);
	tpl_write(t, TPL_REG_RSTN, TPL_MMCM_RSTN | TPL_RSTN);
}

/*
 * Configure the RX (ADC) framer: reset, then arm each converter's datapath with
 * sign-extended, format-enabled, channel-enabled. Mirrors axi_adc_init() minus
 * the (link-dependent) status/clk read.
 */
static int tpl_configure_rx(struct axi_tpl *t)
{
	t->version = tpl_read(t, TPL_REG_VERSION);
	if (TPL_PCORE_VER_MAJOR(t->version) < 9) {
		LOG_ERR("%s: unexpected pcore version 0x%08x", t->name,
			t->version);
		return -ENOTSUP;
	}

	tpl_reset(t);

	for (uint32_t c = 0; c < TPL_NUM_CHANNELS; c++) {
		tpl_write(t, ADC_REG_CHAN_CNTRL(c),
			  ADC_CHAN_FORMAT_SIGNEXT | ADC_CHAN_FORMAT_ENABLE |
				  ADC_CHAN_ENABLE);
	}

	LOG_INF("%s @ 0x%08lx: pcore v%u, %u channels armed (signext|format|enable)",
		t->name, (unsigned long)t->base,
		TPL_PCORE_VER_MAJOR(t->version), TPL_NUM_CHANNELS);
	return 0;
}

/*
 * Configure the TX (DAC) deframer: reset, point each converter's datapath at the
 * DMA/JESD source, then latch with a SYNC pulse. Mirrors axi_dac_init() +
 * axi_dac_data_setup() minus the (link-dependent) status/clk read.
 */
static int tpl_configure_tx(struct axi_tpl *t)
{
	t->version = tpl_read(t, TPL_REG_VERSION);

	tpl_reset(t);

	for (uint32_t c = 0; c < TPL_NUM_CHANNELS; c++) {
		tpl_write(t, DAC_REG_DATA_SELECT(c),
			  DAC_DATA_SEL(DAC_DATA_SEL_DMA));
	}

	/* Latch the datapath configuration. */
	tpl_write(t, DAC_REG_SYNC_CONTROL, DAC_SYNC);

	LOG_INF("%s @ 0x%08lx: pcore v%u, %u channels -> DMA source, synced",
		t->name, (unsigned long)t->base,
		TPL_PCORE_VER_MAJOR(t->version), TPL_NUM_CHANNELS);
	return 0;
}

int axi_tpl_configure(void)
{
	int ret;

	ret = tpl_configure_rx(&tpl_rx);
	if (ret) {
		return ret;
	}
	return tpl_configure_tx(&tpl_tx);
}

/*
 * Post-link verification. Once the lanes and sample clocks are running the core
 * STATUS bit0 reflects a healthy datapath and CLK_FREQ/CLK_RATIO read real
 * values. Re-issue the DAC SYNC to align the datapath to the now-live clock.
 */
static int tpl_check(const struct axi_tpl *t)
{
	uint32_t status = tpl_read(t, TPL_REG_STATUS);
	uint32_t clk_freq = tpl_read(t, TPL_REG_CLK_FREQ);
	uint32_t clk_ratio = tpl_read(t, TPL_REG_CLK_RATIO);

	if (!(status & BIT(0))) {
		LOG_ERR("%s: datapath status not ready (0x%08x)", t->name,
			status);
		return -EIO;
	}

	LOG_INF("%s: status=0x%08x clk_freq=0x%08x clk_ratio=0x%08x", t->name,
		status, clk_freq, clk_ratio);
	return 0;
}

int axi_tpl_enable(void)
{
	int ret;

	/* Re-latch the DAC datapath against the now-running sample clock. */
	tpl_write(&tpl_tx, DAC_REG_SYNC_CONTROL, DAC_SYNC);

	ret = tpl_check(&tpl_rx);
	if (ret) {
		return ret;
	}
	return tpl_check(&tpl_tx);
}

/*
 * Point the TX transport core's converters at their internal FPGA DDS tone
 * generators instead of the DMA stream, or back again.
 *
 * This is what the no-OS example actually emits. Its app.c passes
 * `.channels = NULL` to axi_dac_init(), which sends axi_dac_data_setup() down its
 * else branch (axi_dac_core.c:1227-1238) and writes DATA_SELECT=0 -- DATA_SEL_DDS
 * -- to every converter. So the reference design's DAC output is an
 * FPGA-generated tone, and its DMA and data-offload cores are not in the
 * datapath at all: the TPL's dac_enable is only asserted for data_sel == 4'h2
 * (ad_ip_jesd204_tpl_dac_channel.v:144), so with DDS selected the transport core
 * never pulls a beat from the upack FIFO.
 *
 * That also means this path needs no DDR bandwidth whatsoever, which is why the
 * reference never has to care what the memory system can sustain.
 *
 * The DDS sits at the TPL input inside the FPGA, so its samples still cross the
 * transport core, the serial lanes and the chip's deframer, then the chip's DAC
 * datapath and DUC. Everything except DDR and the DMA engine.
 *
 * scale_micro is in micro-units of full scale (1000000 == 1.0), matching no-OS
 * axi_dac_dds_set_scale(); the register is 1.14 fixed point, so the conversion is
 * scale * 0x4000 / 1000000.
 *
 * freq_hz is quantised by the 16-bit phase increment (freq * 0xFFFF / rate), so
 * the achieved tone only equals the request when it divides the sample rate
 * evenly.
 */
int axi_tpl_tx_dds(uint32_t freq_hz, uint32_t sample_rate_hz,
		   uint32_t scale_micro, bool enable)
{
	struct axi_tpl *t = &tpl_tx;
	uint32_t incr, scale;

	if (!enable) {
		for (uint32_t c = 0; c < TPL_NUM_CHANNELS; c++) {
			tpl_write(t, DAC_REG_DATA_SELECT(c),
				  DAC_DATA_SEL(DAC_DATA_SEL_DMA));
		}
		tpl_write(t, DAC_REG_SYNC_CONTROL, DAC_SYNC);
		LOG_INF("%s: converters back on the DMA source", t->name);
		return 0;
	}

	if (sample_rate_hz == 0) {
		return -EINVAL;
	}

	incr = (uint32_t)(((uint64_t)freq_hz * 0xFFFFULL) / sample_rate_hz);
	if (incr == 0) {
		LOG_WRN("%s: %u Hz is below the DDS resolution at %u SPS",
			t->name, freq_hz, sample_rate_hz);
		return -EINVAL;
	}

	/* Clamp as no-OS does, then convert micro-units to 1.14 fixed point. */
	if (scale_micro >= 1999000U) {
		scale_micro = 1999000U;
	}
	scale = (uint32_t)(((uint64_t)scale_micro * DAC_DDS_SCALE_1_0) / 1000000U);

	/*
	 * Hold SYNC low across the whole reprogram so every DDS starts from the
	 * same phase when it is released. no-OS toggles SYNC per register write;
	 * doing it once around the batch is equivalent and leaves the converters
	 * mutually aligned rather than staggered by write order.
	 */
	tpl_write(t, DAC_REG_SYNC_CONTROL, 0);

	for (uint32_t c = 0; c < TPL_NUM_CHANNELS; c++) {
		uint32_t dds_i = c * 2;
		uint32_t dds_q = c * 2 + 1;
		/*
		 * Both DDSs of a converter get the same frequency and phase, and
		 * the phase alternates 90/0 degrees by converter index. That is
		 * no-OS's default (axi_dac_core.c:1229-1234), not a quadrature
		 * pair -- 0x4000 is 90 degrees of the 16-bit phase word.
		 */
		uint32_t phase = (c % 2) ? 0U : 0x4000U;

		tpl_write(t, DAC_REG_DDS_SCALE(dds_i), scale);
		tpl_write(t, DAC_REG_DDS_SCALE(dds_q), scale);

		/* Low bit of INCR enables the DDS, as no-OS ORs in. */
		tpl_write(t, DAC_REG_DDS_INIT_INCR(dds_i),
			  DAC_DDS_INIT(phase) | DAC_DDS_INCR(incr) | 1U);
		tpl_write(t, DAC_REG_DDS_INIT_INCR(dds_q),
			  DAC_DDS_INIT(phase) | DAC_DDS_INCR(incr) | 1U);

		tpl_write(t, DAC_REG_DATA_SELECT(c),
			  DAC_DATA_SEL(DAC_DATA_SEL_DDS));
	}

	tpl_write(t, DAC_REG_SYNC_CONTROL, DAC_SYNC);

	LOG_INF("%s: DDS tone armed, %u Hz at %u SPS (incr 0x%04x), scale 0x%04x (%u.%02u%% FS)",
		t->name, freq_hz, sample_rate_hz, incr, scale,
		scale_micro / 10000U, (scale_micro % 10000U) / 100U);
	return 0;
}
