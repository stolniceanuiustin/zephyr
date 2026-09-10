/*
 * Copyright (c) 2026 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * AD9680 JESD204B ADC -- standalone SPI bring-up. Link core/FSM bring-up and
 * the RX capture path live in samples/daq2/; DAC->ADC loopback capture
 * verified on HW 2026-09-07 (see Monday_Handoff.md).
 */

#define DT_DRV_COMPAT adi_ad9680

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/sys/util.h>

#include <zephyr/drivers/misc/ad9680.h>
#include <zephyr/jesd204/jesd204_fsm.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ad9680, LOG_LEVEL_INF);

/* -------------------------- AD9680 registers ------------------------------- */

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

/* LINK_CONTROL: 0x15 = disabled+ILAS, 0x14 = enabled. */
#define AD9680_LINK_CTRL_DISABLE_ILAS		0x15
#define AD9680_LINK_CTRL_ENABLE			0x14

/* LANE_RATE_CTRL: low line rate mode required below 6.25 Gbps. */
#define AD9680_LANE_RATE_LOW_ENABLE		0x10
#define AD9680_LANE_RATE_LOW_DISABLE		0x00
#define AD9680_LOW_LINE_RATE_THRESH_KBPS	6250000U

/* PLL_LOCK_STATUS (0x56f): bit7 set == JESD204B PLL locked. */
#define AD9680_PLL_LOCKED			0x80

/* Reset/link-enable settling time. */
#define AD9680_RESET_WAIT_MS			250
#define AD9680_LINK_WAIT_MS			250

/* -------------------------- config / data --------------------------------- */

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
 * JESD204B config bytes derived from the DT geometry (not hardcoded), each
 * BUILD_ASSERT-pinned to its known-good DAQ2 byte in AD9680_DEFINE:
 *   MF_CTRL   (0x58d): bits[4:0]=K-1                  -> K=32       0x1f
 *   SUBCLASS  (0x590): bits[7:5]=subclass, [4:0]=N'-1 -> SC=1,N'=16 0x2f
 *   CSN_CONFIG(0x58f): bits[4:0]=N-1, upper field fixed-> N=14      0x2d
 *   QUICK_CONFIG(0x570): datasheet preset by (M,L)     -> M=2,L=4   0x88
 */
#define AD9680_MF_CTRL_VAL(k)			(((k) - 1) & 0x1F)
#define AD9680_SUBCLASS_CONFIG_VAL(sc, np)	((((sc) & 0x7) << 5) | (((np) - 1) & 0x1F))

/* Upper field of CSN_CONFIG (above N-1); fixed constant, see note above. */
#define AD9680_CSN_CONFIG_UPPER			0x20
#define AD9680_CSN_CONFIG_VAL(n)		(AD9680_CSN_CONFIG_UPPER | (((n) - 1) & 0x1F))

/*
 * QUICK_CONFIG is a datasheet preset table, not a clean bitfield. Only the entry
 * used by the DAQ2 profile (M=2, L=4, F=1) is populated; any other geometry
 * resolves to 0x00 and is caught by the BUILD_ASSERT below.
 */
#define AD9680_QUICK_CONFIG_VAL(m, l)		\
	(((m) == 2 && (l) == 4) ? 0x88 : 0x00)

/*
 * Serial lane rate (kbps), derived for 8B/10B, S=1:
 *   lane_rate = fs * M * N' * (10/8) / L
 * fs in kHz keeps it 32-bit; the 64-bit cast guards the product. DAQ2 -> 10 Gbps.
 */
#define AD9680_LANE_RATE_KBPS(fs_khz, m, np, l)	\
	(((uint64_t)(fs_khz) * (m) * (np) * 10U) / (8U * (l)))

/* -------------------------------- setup ----------------------------------- */

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

	/* JESD PLL lock is non-fatal here; report it, don't fail init on it. */
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

/* Public: re-read the JESD204B PLL lock after the FPGA RX datapath is up. */
int ad9680_jesd_pll_locked(const struct device *dev, bool *locked)
{
	uint8_t pll_stat;
	int ret;

	if (dev == NULL || locked == NULL) {
		return -EINVAL;
	}
	if (!device_is_ready(dev)) {
		return -ENODEV;
	}

	ret = ad9680_spi_read(dev, AD9680_REG_JESD204B_PLL_LOCK_STATUS, &pll_stat);
	if (ret < 0) {
		return ret;
	}

	*locked = (pll_stat & AD9680_PLL_LOCKED) == AD9680_PLL_LOCKED;
	LOG_INF("AD9680 JESD PLL: %s (0x%02x)",
		*locked ? "locked" : "NOT locked", pll_stat);

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
	BUILD_ASSERT(AD9680_MF_CTRL_VAL(DT_INST_PROP(n, adi_frames_per_multiframe)) ==     \
		     0x1f, "AD9680 MF_CTRL byte != 0x1f (check K)");                       \
	BUILD_ASSERT(AD9680_SUBCLASS_CONFIG_VAL(DT_INST_PROP(n, adi_subclass),             \
						DT_INST_PROP(n, adi_bits_per_sample)) ==   \
		     0x2f, "AD9680 SUBCLASS byte != 0x2f (check subclass, N')");          \
	BUILD_ASSERT(AD9680_CSN_CONFIG_VAL(DT_INST_PROP(n, adi_converter_resolution)) ==   \
		     0x2d, "AD9680 CSN byte != 0x2d (check N)");                           \
	BUILD_ASSERT(AD9680_QUICK_CONFIG_VAL(DT_INST_PROP(n, adi_converters_per_device),   \
					     DT_INST_PROP(n, adi_lanes_per_device)) ==     \
		     0x88, "AD9680 QUICK_CONFIG byte != 0x88 (check M, L)");               \
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

/*
 * AD9680 RX ADC (RX topology top device). Re-read the JESD204B (framer
 * serializer) PLL, log, non-fatal.
 */
static int ad9680_fsm_clks_enable(struct jesd204_dev *jdev,
				  enum jesd204_state_op_reason reason,
				  struct jesd204_link *lnk)
{
	bool locked = false;

	ARG_UNUSED(lnk);

	if (reason != JESD204_STATE_OP_REASON_INIT) {
		return JESD204_STATE_CHANGE_DONE;
	}

	if (ad9680_jesd_pll_locked(jesd204_dev_priv(jdev), &locked)) {
		LOG_WRN("AD9680 JESD PLL re-read failed");
	} else {
		LOG_INF("AD9680 JESD PLL: %s", locked ? "locked" : "NOT locked");
	}

	return JESD204_STATE_CHANGE_DONE;
}

const struct jesd204_dev_data ad9680_jesd204_data = {
	.max_num_links = 1,
	.state_ops = {
		[JESD204_OP_CLOCKS_ENABLE] = {
			.per_link = ad9680_fsm_clks_enable,
		},
	},
};
