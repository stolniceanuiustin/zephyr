/*
 * Copyright (c) 2026 Analog Devices, Inc.
 *
 * AD9680 -- 14-bit, dual-channel JESD204B ADC -- Zephyr SPI bring-up driver.
 *
 * On the DAQ2 FMC the AD9680 is the RX converter; its sample clock (ADC_CLK)
 * comes from the AD9523-1 clock generator (see clock_control_ad9523.c), which
 * initialises earlier at POST_KERNEL. This driver scope is the standalone SPI
 * bring-up only: prove the bus by reading the chip ID, soft-reset the chip,
 * apply the JESD204B link register configuration, and report the JESD PLL lock
 * status. It does NOT wire the AXI JESD204 link cores, the transceivers, or the
 * jesd204 FSM -- those are a separate later block, and until they land the
 * actual lane sync / data capture is UNVERIFIED.
 *
 * Faithful port of no-OS ad9680.c (DAQ2/AD9680, reference-hierarchy tier-3) for
 * the register sequence. The one deliberate departure from no-OS: the JESD204B
 * configuration bytes (quick-config, CSN, subclass, multiframe) are DERIVED from
 * the devicetree geometry rather than hardcoded, so a geometry that disagrees
 * with the synthesised bitstream cannot be set. Each derived byte is checked at
 * build time against the known-good no-OS constant (the tier-3 value), so a wrong
 * encoding fails the build instead of reaching hardware. See the DERIVED BYTES
 * block below.
 *
 * INIT LEVEL: POST_KERNEL, not the PRE_KERNEL_1 default for many devices,
 * because it is SPI-attached and Zephyr SPI controllers initialise at
 * POST_KERNEL/CONFIG_SPI_INIT_PRIORITY. A BUILD_ASSERT enforces the ordering.
 *
 * Property names in the binding follow ADI's Linux adi,ad9208/ad9680 binding so
 * a devicetree written for ADI Linux transfers with only syntax changes.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT adi_ad9680

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/sys/util.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ad9680, LOG_LEVEL_INF);

/* -------------------- AD9680 registers (from no-OS ad9680.h) --------------- */

#define AD9680_REG_INTERFACE_CONF_A		0x000
#define AD9680_REG_CHIP_ID_LOW			0x004
#define AD9680_REG_LINK_CONTROL			0x571
#define AD9680_REG_JESD204B_LANE_RATE_CTRL	0x56e
#define AD9680_REG_JESD204B_PLL_LOCK_STATUS	0x56f
#define AD9680_REG_JESD204B_QUICK_CONFIG	0x570
#define AD9680_REG_JESD204B_MF_CTRL		0x58d
#define AD9680_REG_JESD204B_CSN_CONFIG		0x58f
#define AD9680_REG_JESD204B_SUBCLASS_CONFIG	0x590

/* -------------------- AD9680 register values / bitfields ------------------- */

#define AD9680_CHIP_ID				0x0C5

/* INTERFACE_CONF_A: soft reset (self-clearing) + LSB-first mirror bits. */
#define AD9680_INTERFACE_CONF_A_RESET		0x81

/* SPI instruction: bit15 = read. reg_addr is 15 bits. */
#define AD9680_SPI_READ				0x80

/*
 * LINK_CONTROL (0x571). Two fixed control words from no-OS:
 *   0x15 = link disabled, ILAS test-pattern enabled (used before configuring)
 *   0x14 = link enabled
 * Bit0 is the link power-down / disable bit.
 */
#define AD9680_LINK_CTRL_DISABLE_ILAS		0x15
#define AD9680_LINK_CTRL_ENABLE			0x14

/*
 * LANE_RATE_CTRL (0x56e): the AD9680 requires "low line rate mode" enabled
 * below 6.25 Gbps and disabled at or above it. no-OS uses these two words.
 */
#define AD9680_LANE_RATE_LOW_ENABLE		0x10
#define AD9680_LANE_RATE_LOW_DISABLE		0x00
#define AD9680_LOW_LINE_RATE_THRESH_KBPS	6250000U

/* PLL_LOCK_STATUS (0x56f): bit7 set == JESD204B PLL locked. */
#define AD9680_PLL_LOCKED			0x80

/* Reset settling time, from no-OS (no_os_mdelay(250) after reset/enable). */
#define AD9680_RESET_WAIT_MS			250
#define AD9680_LINK_WAIT_MS			250

/* -------------------------- config / data --------------------------------- */

/*
 * JESD204B link geometry, from devicetree. These describe the link the DAQ2
 * bitstream was synthesised for; they are used to DERIVE the configuration
 * register bytes (see the DERIVED BYTES block) and are BUILD_ASSERT-ed against
 * the bitstream profile in AD9680_DEFINE.
 */
struct ad9680_config {
	struct spi_dt_spec spi;

	uint32_t lane_rate_kbps;	/**< Serial lane rate in kbps. */
	uint8_t num_converters;		/**< M: converters per device. */
	uint8_t num_lanes;		/**< L: lanes per device. */
	uint8_t octets_per_frame;	/**< F: octets per frame per lane. */
	uint8_t frames_per_multiframe;	/**< K: frames per multiframe. */
	uint8_t converter_resolution;	/**< N: converter resolution, bits. */
	uint8_t bits_per_sample;	/**< N': total bits per sample. */
	uint8_t subclass;		/**< JESD204B device subclass. */
};

/** @brief Runtime state for one AD9680 instance. */
struct ad9680_data {
	bool spi_ok;		/**< Chip ID read back correctly. */
	bool pll_locked;	/**< JESD204B PLL reported locked at setup. */
};

/* ------------------------------ SPI helpers ------------------------------- */

/*
 * Read one register. The AD9680 SPI frame is 3 bytes: a 16-bit instruction
 * (bit15 = read, bits[14:0] = address, MSB first) followed by one data byte.
 * Faithful port of no-OS ad9680_spi_read().
 */
static int ad9680_spi_read(const struct device *dev, uint16_t reg_addr,
			   uint8_t *reg_data)
{
	const struct ad9680_config *config = dev->config;
	uint8_t tx[3] = {
		AD9680_SPI_READ | (reg_addr >> 8),
		reg_addr & 0xFF,
		0x00,
	};
	uint8_t rx[3] = { 0 };
	const struct spi_buf txb = { .buf = tx, .len = sizeof(tx) };
	const struct spi_buf rxb = { .buf = rx, .len = sizeof(rx) };
	const struct spi_buf_set txs = { .buffers = &txb, .count = 1 };
	const struct spi_buf_set rxs = { .buffers = &rxb, .count = 1 };
	int ret;

	if (reg_data == NULL) {
		return -EINVAL;
	}

	ret = spi_transceive_dt(&config->spi, &txs, &rxs);
	if (ret < 0) {
		return ret;
	}

	*reg_data = rx[2];

	return 0;
}

/*
 * Write one register. 3-byte frame: 16-bit address (MSB first, bit15 = 0 for
 * write) then the data byte. Faithful port of no-OS ad9680_spi_write().
 */
static int ad9680_spi_write(const struct device *dev, uint16_t reg_addr,
			    uint8_t reg_data)
{
	const struct ad9680_config *config = dev->config;
	uint8_t tx[3] = {
		reg_addr >> 8,
		reg_addr & 0xFF,
		reg_data,
	};
	const struct spi_buf txb = { .buf = tx, .len = sizeof(tx) };
	const struct spi_buf_set txs = { .buffers = &txb, .count = 1 };

	return spi_write_dt(&config->spi, &txs);
}

/* ----------------------------- DERIVED BYTES ------------------------------ */

/*
 * The JESD204B configuration registers are derived from the link geometry
 * rather than hardcoded, so no geometry can be set that disagrees with the
 * bytes actually written. Each macro below is a compile-time function of the DT
 * geometry, and each is BUILD_ASSERT-ed in AD9680_DEFINE against the exact byte
 * the tier-3 no-OS reference writes for the DAQ2 profile -- the derivation is
 * the intent, the assert is the tier-3 check. A wrong encoding fails the build.
 *
 *   MF_CTRL (0x58d):   bits[4:0] = K-1               -> K=32  gives 0x1f
 *   SUBCLASS (0x590):  bits[7:5] = subclass,
 *                      bits[4:0] = N'-1              -> SC=1,N'=16 gives 0x2f
 *   CSN_CONFIG (0x58f):bits[4:0] = N-1, plus a fixed
 *                      upper-field default           -> N=14 gives 0x2d
 *   QUICK_CONFIG(0x570): datasheet preset selected by (M,L) -> M=2,L=4 = 0x88
 *
 * CSN_CONFIG upper bits: the AD9680 packs the number of control bits (CS) and
 * tail-bit config above N-1 here; the exact field split is not in this repo's
 * headers, so the low field (N-1) is derived and the upper field is carried as
 * the tier-3 constant with CS = N'-N checked separately. The BUILD_ASSERT ties
 * the whole byte to no-OS's value.
 */
#define AD9680_MF_CTRL_VAL(k)			(((k) - 1) & 0x1F)
#define AD9680_SUBCLASS_CONFIG_VAL(sc, np)	((((sc) & 0x7) << 5) | (((np) - 1) & 0x1F))

/* Upper field of CSN_CONFIG (above N-1); tier-3 constant, see note above. */
#define AD9680_CSN_CONFIG_UPPER			0x20
#define AD9680_CSN_CONFIG_VAL(n)		(AD9680_CSN_CONFIG_UPPER | (((n) - 1) & 0x1F))

/*
 * QUICK_CONFIG is a datasheet preset table, not a clean bitfield. Only the entry
 * used by the DAQ2 profile (M=2, L=4, F=1) is populated; any other geometry
 * resolves to 0x00 and is caught by the BUILD_ASSERT against no-OS's 0x88.
 */
#define AD9680_QUICK_CONFIG_VAL(m, l)		\
	(((m) == 2 && (l) == 4) ? 0x88 : 0x00)

/*
 * Serial lane rate in kbps, DERIVED from the sample rate and geometry -- it is a
 * function of the others, so it is not an independently settable property (see
 * CLAUDE.md). For 8B/10B (10/8 overhead), S=1:
 *
 *   lane_rate = fs * M * N' * (10/8) / L
 *
 * fs is carried in kHz to keep the arithmetic 32-bit; the cast to 64-bit guards
 * the intermediate product. DAQ2 RX: fs=1e6 kHz, M=2, N'=16, L=4 -> 10 Gbps.
 */
#define AD9680_LANE_RATE_KBPS(fs_khz, m, np, l)	\
	(((uint64_t)(fs_khz) * (m) * (np) * 10U) / (8U * (l)))

/* -------------------------------- setup ----------------------------------- */

/* Faithful port of no-OS ad9680_setup() (allocation and SPI-init stripped). */
static int ad9680_setup(const struct device *dev)
{
	const struct ad9680_config *config = dev->config;
	struct ad9680_data *data = dev->data;
	uint8_t chip_id;
	uint8_t pll_stat;
	uint8_t lane_rate_ctrl;
	int ret;

	/* Prove the bus: the chip ID register returns a fixed constant. */
	ret = ad9680_spi_read(dev, AD9680_REG_CHIP_ID_LOW, &chip_id);
	if (ret < 0) {
		return ret;
	}
	if (chip_id != AD9680_CHIP_ID) {
		LOG_ERR("invalid chip ID 0x%02x (expected 0x%02x)", chip_id,
			AD9680_CHIP_ID);
		LOG_ERR("check CS wiring, SPI ref clock, or 3-/4-wire mode");
		return -ENODEV;
	}
	data->spi_ok = true;
	LOG_INF("SUCCESS: AD9680 chip ID 0x%02x", chip_id);

	/* Soft reset; the reset bit is self-clearing after settling. */
	ret = ad9680_spi_write(dev, AD9680_REG_INTERFACE_CONF_A,
			       AD9680_INTERFACE_CONF_A_RESET);
	if (ret < 0) {
		return ret;
	}
	k_msleep(AD9680_RESET_WAIT_MS);
	LOG_INF("SUCCESS: AD9680 reset");

	/*
	 * JESD204B link configuration. Bring the link down with ILAS enabled,
	 * program the geometry-derived config bytes, set the lane-rate mode,
	 * then enable the link.
	 */
	ret = ad9680_spi_write(dev, AD9680_REG_LINK_CONTROL,
			       AD9680_LINK_CTRL_DISABLE_ILAS);
	if (ret < 0) {
		return ret;
	}
	ret = ad9680_spi_write(dev, AD9680_REG_JESD204B_MF_CTRL,
			       AD9680_MF_CTRL_VAL(config->frames_per_multiframe));
	if (ret < 0) {
		return ret;
	}
	ret = ad9680_spi_write(dev, AD9680_REG_JESD204B_CSN_CONFIG,
			       AD9680_CSN_CONFIG_VAL(config->converter_resolution));
	if (ret < 0) {
		return ret;
	}
	ret = ad9680_spi_write(dev, AD9680_REG_JESD204B_SUBCLASS_CONFIG,
			       AD9680_SUBCLASS_CONFIG_VAL(config->subclass,
							  config->bits_per_sample));
	if (ret < 0) {
		return ret;
	}
	ret = ad9680_spi_write(dev, AD9680_REG_JESD204B_QUICK_CONFIG,
			       AD9680_QUICK_CONFIG_VAL(config->num_converters,
						       config->num_lanes));
	if (ret < 0) {
		return ret;
	}

	/* Low line rate mode is required below 6.25 Gbps, forbidden above. */
	lane_rate_ctrl = (config->lane_rate_kbps < AD9680_LOW_LINE_RATE_THRESH_KBPS) ?
			 AD9680_LANE_RATE_LOW_ENABLE : AD9680_LANE_RATE_LOW_DISABLE;
	ret = ad9680_spi_write(dev, AD9680_REG_JESD204B_LANE_RATE_CTRL,
			       lane_rate_ctrl);
	if (ret < 0) {
		return ret;
	}

	ret = ad9680_spi_write(dev, AD9680_REG_LINK_CONTROL,
			       AD9680_LINK_CTRL_ENABLE);
	if (ret < 0) {
		return ret;
	}
	k_msleep(AD9680_LINK_WAIT_MS);
	LOG_INF("SUCCESS: AD9680 JESD204B link configured");

	/*
	 * JESD PLL lock. Non-fatal: the PLL needs the ADC clock (present, from
	 * the AD9523) but full lock also depends on the link/SYSREF that this
	 * standalone bring-up does not wire, so a not-locked report here is
	 * expected and stays UNVERIFIED until the link block lands. Report it,
	 * do not fail init on it.
	 */
	ret = ad9680_spi_read(dev, AD9680_REG_JESD204B_PLL_LOCK_STATUS,
			      &pll_stat);
	if (ret < 0) {
		return ret;
	}
	data->pll_locked = (pll_stat & AD9680_PLL_LOCKED) == AD9680_PLL_LOCKED;
	LOG_INF("AD9680 JESD PLL: %s (0x%02x)",
		data->pll_locked ? "locked" : "NOT locked", pll_stat);

	return 0;
}

/* --------------------------------- init ----------------------------------- */

static int ad9680_init(const struct device *dev)
{
	const struct ad9680_config *config = dev->config;
	int ret;

	if (!spi_is_ready_dt(&config->spi)) {
		LOG_ERR("SPI bus %s not ready", config->spi.bus->name);
		return -ENODEV;
	}

	LOG_INF("AD9680 setup over %s", config->spi.bus->name);

	ret = ad9680_setup(dev);
	if (ret) {
		LOG_ERR("ad9680_setup failed (%d)", ret);
		return ret;
	}

	LOG_INF("SUCCESS: AD9680 setup complete");

	return 0;
}

/* ------------------------------ DT plumbing ------------------------------- */

#define AD9680_DEFINE(n)                                                                    \
	/* Geometry must match the synthesised DAQ2 RX bitstream profile. */               \
	BUILD_ASSERT(DT_INST_PROP(n, adi_converters_per_device) == 2,                      \
		     "AD9680 M must be 2 (DAQ2 RX bitstream)");                            \
	BUILD_ASSERT(DT_INST_PROP(n, adi_lanes_per_device) == 4,                           \
		     "AD9680 L must be 4 (DAQ2 RX bitstream)");                            \
	BUILD_ASSERT(DT_INST_PROP(n, adi_octets_per_frame) == 1,                           \
		     "AD9680 F must be 1 (derived from M=2,L=4,N'=16,S=1)");              \
	BUILD_ASSERT(DT_INST_PROP(n, adi_bits_per_sample) ==                               \
		     DT_INST_PROP(n, adi_converter_resolution) +                          \
		     DT_INST_PROP(n, adi_control_bits_per_sample),                        \
		     "AD9680 N' must equal N + CS (no tail bits on DAQ2)");              \
	/* Derived config bytes must match the tier-3 (no-OS) constants. */               \
	BUILD_ASSERT(AD9680_MF_CTRL_VAL(DT_INST_PROP(n, adi_frames_per_multiframe)) ==     \
		     0x1f, "AD9680 MF_CTRL byte != no-OS 0x1f (check K)");                \
	BUILD_ASSERT(AD9680_SUBCLASS_CONFIG_VAL(DT_INST_PROP(n, adi_subclass),             \
						DT_INST_PROP(n, adi_bits_per_sample)) ==   \
		     0x2f, "AD9680 SUBCLASS byte != no-OS 0x2f (check subclass, N')");    \
	BUILD_ASSERT(AD9680_CSN_CONFIG_VAL(DT_INST_PROP(n, adi_converter_resolution)) ==   \
		     0x2d, "AD9680 CSN byte != no-OS 0x2d (check N)");                     \
	BUILD_ASSERT(AD9680_QUICK_CONFIG_VAL(DT_INST_PROP(n, adi_converters_per_device),   \
					     DT_INST_PROP(n, adi_lanes_per_device)) ==     \
		     0x88, "AD9680 QUICK_CONFIG byte != no-OS 0x88 (check M, L)");         \
	/* Derived lane rate must match the bitstream's synthesised 10 Gbps. */            \
	BUILD_ASSERT(AD9680_LANE_RATE_KBPS(DT_INST_PROP(n, adi_sampling_frequency_khz),    \
					   DT_INST_PROP(n, adi_converters_per_device),     \
					   DT_INST_PROP(n, adi_bits_per_sample),           \
					   DT_INST_PROP(n, adi_lanes_per_device)) ==       \
		     10000000U, "AD9680 lane rate != 10 Gbps (bitstream LANE_RATE=10)");   \
                                                                                           \
	static const struct ad9680_config ad9680_config_##n = {                            \
		.spi = SPI_DT_SPEC_INST_GET(n, SPI_WORD_SET(8) | SPI_TRANSFER_MSB |         \
						   SPI_OP_MODE_MASTER),                    \
		.lane_rate_kbps = AD9680_LANE_RATE_KBPS(                                    \
			DT_INST_PROP(n, adi_sampling_frequency_khz),                       \
			DT_INST_PROP(n, adi_converters_per_device),                       \
			DT_INST_PROP(n, adi_bits_per_sample),                             \
			DT_INST_PROP(n, adi_lanes_per_device)),                           \
		.num_converters = DT_INST_PROP(n, adi_converters_per_device),             \
		.num_lanes = DT_INST_PROP(n, adi_lanes_per_device),                       \
		.octets_per_frame = DT_INST_PROP(n, adi_octets_per_frame),                \
		.frames_per_multiframe = DT_INST_PROP(n, adi_frames_per_multiframe),      \
		.converter_resolution = DT_INST_PROP(n, adi_converter_resolution),        \
		.bits_per_sample = DT_INST_PROP(n, adi_bits_per_sample),                   \
		.subclass = DT_INST_PROP(n, adi_subclass),                                \
	};                                                                                 \
                                                                                           \
	static struct ad9680_data ad9680_data_##n;                                         \
                                                                                           \
	DEVICE_DT_INST_DEFINE(n, ad9680_init, NULL, &ad9680_data_##n,                       \
			      &ad9680_config_##n, POST_KERNEL,                             \
			      CONFIG_AD9680_INIT_PRIORITY, NULL);

BUILD_ASSERT(CONFIG_AD9680_INIT_PRIORITY > CONFIG_SPI_INIT_PRIORITY,
	     "The AD9680 is SPI-attached, so it must initialise after its SPI controller");

DT_INST_FOREACH_STATUS_OKAY(AD9680_DEFINE)
