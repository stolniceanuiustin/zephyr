/*
 * AD9081/AD9082 MxFE -- Zephyr driver over the ADI API library.
 *
 * The ADI API library (src/adi_api/, copied verbatim from no-OS) is fully
 * hardware-abstracted: it reaches the chip only through the function pointers in
 * adi_ad9081_device_t.hal_info. This file implements those callbacks against
 * Zephyr SPI/GPIO and drives the library to init the device, read its ID and
 * configure the converter datapath.
 *
 * Callbacks implemented (of the AD9081 HAL set):
 *   spi_xfer       -> spi_transceive_dt
 *   delay_us       -> k_busy_wait
 *   reset_pin_ctrl -> gpio_pin_set_dt (RSTB, active low)
 *   log_write      -> LOG_*
 *   hw_open/close  -> no-op (bus is brought up by Zephyr already)
 *
 * One instance per devicetree node. hal_info.user_data carries the
 * `const struct device *`, which is how each callback finds its own instance's
 * SPI and GPIO handles -- the library passes it back unchanged.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT adi_ad9081

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <string.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ad9081, LOG_LEVEL_INF);

#include <zephyr/drivers/misc/ad9081/ad9081.h>
#include "adi_ad9081.h"
/* hal + bf: the deframer buf-protect op writes one bitfield by name. */
#include "adi_ad9081_hal.h"
#include "adi_ad9081_bf_ad9081.h"
#include <zephyr/drivers/misc/jesd204/jesd204_geometry.h>

/*
 * The SPI register page is mapped 1:1 by spi_mmio_fixup.c at PRE_KERNEL_1,
 * working around the upstream Cadence SPI driver not using DEVICE_MMIO. That is
 * a board-level wart, documented there, and deliberately not this file's
 * business.
 */

/* The AD9081 and AD9082 are the same MxFE family and share this driver / SPI
 * map; the ADI library accepts either PROD_ID.
 */
#define AD9081_CHIPID 0x9081
#define AD9082_CHIPID 0x9082

/* Devicetree configuration -- ROM, one per node. */
struct ad9081_config {
	struct spi_dt_spec spi;
	/* .port == NULL when the node has no reset-gpios. */
	struct gpio_dt_spec reset;

	/* Hz, from the kHz properties. The vendor API takes Hz. */
	uint64_t dac_freq_hz;
	uint64_t adc_freq_hz;
	uint64_t ref_freq_hz;

	uint8_t tx_main_interp;
	uint8_t tx_chan_interp;

	uint8_t rx_cddc_select;
	uint8_t rx_fddc_select;

	/*
	 * Decimation as ratios. The register codes the vendor API wants are
	 * derived from these at build time (see AD9081_PROFILE below); these
	 * are kept because the boot log reports the ratio, not the code.
	 */
	uint8_t rx_cddc_decim[4];
	uint8_t rx_fddc_decim[8];
};

/*
 * The datapath profile, in RAM because every vendor entry point takes these by
 * non-const pointer. Statically initialised from devicetree, one per node --
 * nothing here is computed at runtime.
 */
struct ad9081_profile {
	uint8_t tx_dac_chan_xbar[4];
	int64_t tx_main_shift[4];
	int64_t tx_chan_shift[8];
	uint16_t tx_chan_gain[8];
	uint8_t tx_lane_map[8];
	adi_cms_jesd_param_t tx_jesd_param;

	int64_t rx_cddc_shift[4];
	int64_t rx_fddc_shift[8];
	uint8_t rx_cddc_dcm[4]; /* register codes, not ratios */
	uint8_t rx_fddc_dcm[8];
	uint8_t rx_cc2r_en[4];
	uint8_t rx_fc2r_en[8];
	uint8_t rx_lane_map[8];
	adi_cms_jesd_param_t rx_jesd_param[2];
	adi_ad9081_jtx_conv_sel_t rx_conv_sel[2];
};

/* Per-instance state -- RAM. The ADI library's device handle, which also holds
 * the HAL binding and every register-shadow the library keeps.
 */
struct ad9081_data {
	adi_ad9081_device_t dev;
	struct ad9081_profile profile;
};

/*
 * ------------------------- ADI HAL callbacks ---------------------------------
 * All take the void *user_data we install as hal_info.user_data, which is the
 * `const struct device *` of the instance. Return API_CMS_ERROR_* codes.
 */

/*
 * Full-duplex SPI transfer. The library has already framed in_data (address
 * bytes with the R/W bit + payload); for MSB-first we transfer size_bytes as-is
 * and copy the miso bytes back into out_data. Matches no-OS ad9081_spi_xfer()
 * for the SPI_MSB_FIRST path.
 */
static int32_t hal_spi_xfer(void *user_data, uint8_t *in_data,
			    uint8_t *out_data, uint32_t size_bytes)
{
	const struct device *dev = user_data;
	const struct ad9081_config *cfg = dev->config;
	uint32_t n = size_bytes & 0xFF;
	const struct spi_buf txb = { .buf = in_data, .len = n };
	const struct spi_buf rxb = { .buf = out_data, .len = n };
	const struct spi_buf_set txs = { .buffers = &txb, .count = 1 };
	const struct spi_buf_set rxs = { .buffers = &rxb, .count = 1 };

	if (spi_transceive_dt(&cfg->spi, &txs, &rxs) != 0) {
		return API_CMS_ERROR_SPI_XFER;
	}
	return API_CMS_ERROR_OK;
}

static int32_t hal_delay_us(void *user_data, uint32_t us)
{
	ARG_UNUSED(user_data);
	k_busy_wait(us);
	return API_CMS_ERROR_OK;
}

/* enable=1 -> RESETB high (de-asserted); enable=0 -> RESETB low (asserted).
 * The GPIO is flagged GPIO_ACTIVE_LOW in DT, so logical 1 == physical low.
 * We want physical level == enable, i.e. logical value = !enable.
 */
static int32_t hal_reset_pin_ctrl(void *user_data, uint8_t enable)
{
	const struct device *dev = user_data;
	const struct ad9081_config *cfg = dev->config;

	if (cfg->reset.port == NULL) {
		return API_CMS_ERROR_OK; /* no reset line wired */
	}
	gpio_pin_set_dt(&cfg->reset, enable ? 0 : 1);
	return API_CMS_ERROR_OK;
}

static int32_t hal_log_write(void *user_data, int32_t log_type,
			     const char *message, va_list argp)
{
	ARG_UNUSED(user_data);
	char buf[128];

	vsnprintk(buf, sizeof(buf), message, argp);

	switch (log_type) {
	case ADI_CMS_LOG_ERR:
		LOG_ERR("%s", buf);
		break;
	case ADI_CMS_LOG_WARN:
		LOG_WRN("%s", buf);
		break;
	default:
		LOG_DBG("%s", buf);
		break;
	}
	return API_CMS_ERROR_OK;
}

static int32_t hal_hw_open(void *user_data)
{
	ARG_UNUSED(user_data);
	return API_CMS_ERROR_OK; /* SPI bus already up via Zephyr */
}

static int32_t hal_hw_close(void *user_data)
{
	ARG_UNUSED(user_data);
	return API_CMS_ERROR_OK;
}

/* -------------------------------------------------------------------------- */

static void ad9081_hal_bind(const struct device *dev)
{
	struct ad9081_data *data = dev->data;
	adi_ad9081_hal_t *hal = &data->dev.hal_info;

	/*
	 * What every callback above receives back from the library. This is the
	 * only channel the vendor code offers for per-instance context.
	 */
	hal->user_data = (void *)dev;

	/* SPI interface: 4-wire (SDO active), MSB first, auto address inc --
	 * matches the SPI_INTFCONFA=0x3C we set by hand before.
	 */
	hal->sdo = SPI_SDO;
	hal->msb = SPI_MSB_FIRST;
	hal->addr_inc = SPI_ADDR_INC_AUTO;

	hal->spi_xfer = hal_spi_xfer;
	hal->delay_us = hal_delay_us;
	hal->reset_pin_ctrl = hal_reset_pin_ctrl;
	hal->log_write = hal_log_write;
	hal->hw_open = hal_hw_open;
	hal->hw_close = hal_hw_close;
	hal->tx_en_pin_ctrl = NULL; /* not wired for chip-ID bring-up */
}

int ad9081_probe(const struct device *dev, uint16_t *prod_id)
{
	const struct ad9081_config *cfg;
	struct ad9081_data *data;
	adi_cms_chip_id_t chip_id = { 0 };
	int32_t err;

	if (!device_is_ready(dev)) {
		LOG_ERR("device not ready");
		return -ENODEV;
	}
	cfg = dev->config;
	data = dev->data;

	LOG_INF("AD9081 probe via ADI API lib over %s", cfg->spi.bus->name);

	if (!spi_is_ready_dt(&cfg->spi)) {
		LOG_ERR("SPI bus %s not ready", cfg->spi.bus->name);
		return -ENODEV;
	}

	if (cfg->reset.port != NULL) {
		if (!gpio_is_ready_dt(&cfg->reset)) {
			LOG_ERR("reset GPIO not ready");
			return -ENODEV;
		}
		/* Configure de-asserted; the library pulses it via
		 * reset_pin_ctrl during device_reset (HARD_RESET).
		 */
		gpio_pin_configure_dt(&cfg->reset, GPIO_OUTPUT_INACTIVE);
	}

	ad9081_hal_bind(dev);

	/*
	 * Hard reset (pulses RSTB via reset_pin_ctrl) then init. The _AND_INIT
	 * variant calls adi_ad9081_device_init() internally: SPI config +
	 * 8-bit reg access check + power-status check. No PLL boot yet.
	 */
	err = adi_ad9081_device_reset(&data->dev, AD9081_HARD_RESET_AND_INIT);
	if (err != API_CMS_ERROR_OK) {
		LOG_ERR("adi_ad9081_device_reset failed (%d)", err);
		return -EIO;
	}

	err = adi_ad9081_device_chip_id_get(&data->dev, &chip_id);
	if (err != API_CMS_ERROR_OK) {
		LOG_ERR("adi_ad9081_device_chip_id_get failed (%d)", err);
		return -EIO;
	}

	LOG_INF("chip_type=0x%02x prod_id=0x%04x grade=0x%x rev=0x%x",
		chip_id.chip_type, chip_id.prod_id,
		chip_id.prod_grade, chip_id.dev_revision);

	if (chip_id.prod_id != AD9081_CHIPID &&
	    chip_id.prod_id != AD9082_CHIPID) {
		LOG_ERR("Unexpected PROD_ID 0x%04x (expected 0x9081/0x9082)",
			chip_id.prod_id);
		return -ENODEV;
	}

	if (prod_id) {
		*prod_id = chip_id.prod_id;
	}
	return 0;
}

/*
 * ------------------------- MxFE datapath configuration ------------------------
 *
 * Every parameter now comes from devicetree (see boards/zynqmp_apu.overlay and
 * dts/bindings/adi,ad9081.yaml); the values there are the no-OS
 * zcu102_ad9081_m8_l4 profile. This function drives the ADI API in the same
 * order as no-OS ad9081_setup()/setup_tx()/setup_rx().
 */

int ad9081_setup_datapath(const struct device *dev)
{
	const struct ad9081_config *cfg;
	struct ad9081_profile *p;
	struct ad9081_data *data;
	adi_ad9081_device_t *chip;
	uint8_t pll_status;
	int32_t err;

	if (!device_is_ready(dev)) {
		LOG_ERR("device not ready");
		return -ENODEV;
	}
	cfg = dev->config;
	data = dev->data;
	p = &data->profile;
	chip = &data->dev;

	LOG_INF("AD9081 datapath setup (DAC %uG / ADC %uG, ref %uM)",
		(unsigned int)(cfg->dac_freq_hz / 1000000000ULL),
		(unsigned int)(cfg->adc_freq_hz / 1000000000ULL),
		(unsigned int)(cfg->ref_freq_hz / 1000000ULL));

	/* On-chip CLK PLL: ref (250M) != dac (12G), so the PLL is engaged. */
	err = adi_ad9081_device_clk_config_set(chip, cfg->dac_freq_hz,
					       cfg->adc_freq_hz,
					       cfg->ref_freq_hz);
	if (err != API_CMS_ERROR_OK) {
		LOG_ERR("clk_config_set failed (%d)", err);
		return -EIO;
	}

	err = adi_ad9081_device_clk_pll_lock_status_get(chip, &pll_status);
	if (err != API_CMS_ERROR_OK) {
		LOG_ERR("clk_pll_lock_status_get failed (%d)", err);
		return -EIO;
	}
	if (pll_status != 3) {
		LOG_ERR("device CLK PLL not locked (status=0x%x)", pll_status);
		return -EIO;
	}
	LOG_INF("device CLK PLL locked (status=0x%x)", pll_status);

	/* TX deframer (JRX): install chip-side lane map, start the TX datapath. */
	memcpy(chip->serdes_info.des_settings.lane_mapping[0], p->tx_lane_map,
	       sizeof(p->tx_lane_map));

	err = adi_ad9081_device_startup_tx(chip, cfg->tx_main_interp,
					   cfg->tx_chan_interp,
					   p->tx_dac_chan_xbar, p->tx_main_shift,
					   p->tx_chan_shift, &p->tx_jesd_param);
	if (err != API_CMS_ERROR_OK) {
		LOG_ERR("device_startup_tx failed (%d)", err);
		return -EIO;
	}

	err = adi_ad9081_dac_duc_nco_gains_set(chip, p->tx_chan_gain);
	if (err != API_CMS_ERROR_OK) {
		LOG_ERR("dac_duc_nco_gains_set failed (%d)", err);
		return -EIO;
	}
	LOG_INF("TX datapath up: interp %ux%u, JRX deframer mode %u (M%u L%u F%u)",
		(unsigned int)cfg->tx_main_interp,
		(unsigned int)cfg->tx_chan_interp,
		(unsigned int)p->tx_jesd_param.jesd_mode_id,
		(unsigned int)p->tx_jesd_param.jesd_m,
		(unsigned int)p->tx_jesd_param.jesd_l,
		(unsigned int)p->tx_jesd_param.jesd_f);

	/* RX framer (JTX): install chip-side lane map + converter select. */
	memcpy(chip->serdes_info.ser_settings.lane_mapping[0], p->rx_lane_map,
	       sizeof(p->rx_lane_map));

	err = adi_ad9081_device_startup_rx(chip, cfg->rx_cddc_select,
					   cfg->rx_fddc_select, p->rx_cddc_shift,
					   p->rx_fddc_shift, p->rx_cddc_dcm,
					   p->rx_fddc_dcm, p->rx_cc2r_en,
					   p->rx_fc2r_en, p->rx_jesd_param,
					   p->rx_conv_sel);
	if (err != API_CMS_ERROR_OK) {
		LOG_ERR("device_startup_rx failed (%d)", err);
		return -EIO;
	}
	LOG_INF("RX datapath up: decim %u/%u, JTX framer mode %u (M%u L%u F%u)",
		(unsigned int)cfg->rx_cddc_decim[0],
		(unsigned int)cfg->rx_fddc_decim[0],
		(unsigned int)p->rx_jesd_param[0].jesd_mode_id,
		(unsigned int)p->rx_jesd_param[0].jesd_m,
		(unsigned int)p->rx_jesd_param[0].jesd_l,
		(unsigned int)p->rx_jesd_param[0].jesd_f);

	return 0;
}

/*
 * ------------------------- bring-up ops --------------------------------------
 *
 * The chip-side half of the JESD204 bring-up, one op per thing the FSM does.
 * jesd_fsm.c used to reach adi_ad9081_* directly through an ad9081_get_device()
 * accessor; these are the same calls, moved behind dev->api so the FSM no longer
 * names this part.
 *
 * The vendor API returns API_CMS_ERROR_* (0 = OK, negative otherwise) and every
 * one of these is a register access, so the errno is uniformly -EIO. Logging
 * stays at the call site: which failure is worth a warning and which is fatal is
 * the FSM's judgement, not the driver's.
 */

/* Chip-side link select: this profile is single-link, so link 0 on both ends. */
#define AD9081_LINK AD9081_LINK_0

static int ad9081_op_sync_oneshot(const struct device *dev)
{
	struct ad9081_data *data = dev->data;

	/*
	 * Subclass comes from the link geometry rather than from a literal here:
	 * it is the same number the FPGA link cores advertise in ILAS, and it is
	 * already in the profile from adi,subclass on the TX link node.
	 */
	if (adi_ad9081_jesd_oneshot_sync(
		    &data->dev,
		    (adi_cms_jesd_subclass_e)data->profile.tx_jesd_param
			    .jesd_subclass)) {
		return -EIO;
	}
	return 0;
}

static int ad9081_op_sync_nco(const struct device *dev)
{
	struct ad9081_data *data = dev->data;

	if (adi_ad9081_device_nco_sync_post(&data->dev)) {
		return -EIO;
	}
	return 0;
}

static int ad9081_op_jesd_pll_status_get(const struct device *dev,
					 uint8_t *status)
{
	struct ad9081_data *data = dev->data;

	if (adi_ad9081_jesd_pll_lock_status_get(&data->dev, status)) {
		return -EIO;
	}
	return 0;
}

static int ad9081_op_deframer_calibrate(const struct device *dev,
					bool force_reset, uint8_t boost_mask,
					bool run_bg_cal)
{
	struct ad9081_data *data = dev->data;

	if (adi_ad9081_jesd_rx_calibrate_204c(&data->dev, force_reset,
					      boost_mask, run_bg_cal)) {
		return -EIO;
	}
	return 0;
}

static int ad9081_op_deframer_buf_protect_disable(const struct device *dev)
{
	struct ad9081_data *data = dev->data;

	/*
	 * JRX_TPL_1 (0x4A1) bit6 BUF_PROTECT_EN. A bitfield write rather than an
	 * op with a bool, because there is nothing to be gained from being able
	 * to turn it back on: it resets to 1 and this is the only thing that ever
	 * touches it.
	 */
	if (adi_ad9081_hal_bf_set(&data->dev, REG_JRX_TPL_1_ADDR,
				  BF_JRX_TPL_BUF_PROTECT_EN_INFO, 0)) {
		return -EIO;
	}
	return 0;
}

static int ad9081_op_deframer_enable(const struct device *dev, bool enable)
{
	struct ad9081_data *data = dev->data;

	if (adi_ad9081_jesd_rx_link_enable_set(&data->dev, AD9081_LINK,
					       enable ? 1 : 0)) {
		return -EIO;
	}
	return 0;
}

static int ad9081_op_framer_status_get(const struct device *dev,
				       uint16_t *status)
{
	struct ad9081_data *data = dev->data;

	if (adi_ad9081_jesd_tx_link_status_get(&data->dev, AD9081_LINK,
					       status)) {
		return -EIO;
	}
	return 0;
}

static int ad9081_op_deframer_status_get(const struct device *dev,
					 uint16_t *status)
{
	struct ad9081_data *data = dev->data;

	if (adi_ad9081_jesd_rx_link_status_get(&data->dev, AD9081_LINK,
					       status)) {
		return -EIO;
	}
	return 0;
}

static const struct ad9081_driver_api ad9081_api = {
	.sync_oneshot = ad9081_op_sync_oneshot,
	.sync_nco = ad9081_op_sync_nco,
	.jesd_pll_status_get = ad9081_op_jesd_pll_status_get,
	.deframer_calibrate = ad9081_op_deframer_calibrate,
	.deframer_buf_protect_disable = ad9081_op_deframer_buf_protect_disable,
	.deframer_enable = ad9081_op_deframer_enable,
	.framer_status_get = ad9081_op_framer_status_get,
	.deframer_status_get = ad9081_op_deframer_status_get,
};

/*
 * init() only exists so device_is_ready() means something. The real work is the
 * explicit ad9081_probe() / ad9081_setup_datapath() pair from the bring-up
 * sequence, in the order relative to the clock chip and the PL cores that it
 * needs -- init-level ordering cannot express that, and probing here would put a
 * ~1.1 s hard reset plus PLL boot inside POST_KERNEL.
 *
 * Same shape as the PL core drivers here (axi_tpl.c, axi_jesd204.c), and
 * deliberately unlike the HMC7044, which does program itself at init because
 * everything downstream needs its clocks before anything else can run.
 */
static int ad9081_init(const struct device *dev)
{
	ARG_UNUSED(dev);
	return 0;
}

/*
 * Decimation ratio -> vendor register code. The codes are not the ratios and
 * are not even monotonic in them (CDDC: DCM_2 is 0x0 but DCM_4 is 0x1, DCM_1 is
 * 0xC), so devicetree carries the ratio and this maps it. ADI's own Linux/no-OS
 * driver does exactly this -- no-OS drivers/adc/ad9081/ad9081.c:292 and :324 are
 * the same two switch statements, returning -1 for a ratio the part lacks.
 *
 * Unsupported ratios land on 0xFF, which AD9081_ASSERT_DCM below turns into a
 * build error rather than a silently wrong register write.
 */
#define AD9081_DCM_BAD 0xFF

#define AD9081_CDDC_DCM_CODE(r)                                                \
	((r) == 1  ? AD9081_CDDC_DCM_1  :                                      \
	 (r) == 2  ? AD9081_CDDC_DCM_2  :                                      \
	 (r) == 3  ? AD9081_CDDC_DCM_3  :                                      \
	 (r) == 4  ? AD9081_CDDC_DCM_4  :                                      \
	 (r) == 6  ? AD9081_CDDC_DCM_6  :                                      \
	 (r) == 8  ? AD9081_CDDC_DCM_8  :                                      \
	 (r) == 9  ? AD9081_CDDC_DCM_9  :                                      \
	 (r) == 12 ? AD9081_CDDC_DCM_12 :                                      \
	 (r) == 16 ? AD9081_CDDC_DCM_16 :                                      \
	 (r) == 18 ? AD9081_CDDC_DCM_18 :                                      \
	 (r) == 24 ? AD9081_CDDC_DCM_24 :                                      \
	 (r) == 36 ? AD9081_CDDC_DCM_36 : AD9081_DCM_BAD)

/*
 * 0 is not a ratio -- it is how the no-OS profile spells "this fine DDC is
 * disabled", for the fine DDCs the select mask leaves out. It maps to 0, which
 * is what the old hand-written rx_fddc_dcm[] carried in those slots.
 */
#define AD9081_FDDC_DCM_CODE(r)                                                \
	((r) == 0  ? 0                  :                                      \
	 (r) == 1  ? AD9081_FDDC_DCM_1  :                                      \
	 (r) == 2  ? AD9081_FDDC_DCM_2  :                                      \
	 (r) == 3  ? AD9081_FDDC_DCM_3  :                                      \
	 (r) == 4  ? AD9081_FDDC_DCM_4  :                                      \
	 (r) == 6  ? AD9081_FDDC_DCM_6  :                                      \
	 (r) == 8  ? AD9081_FDDC_DCM_8  :                                      \
	 (r) == 12 ? AD9081_FDDC_DCM_12 :                                      \
	 (r) == 16 ? AD9081_FDDC_DCM_16 :                                      \
	 (r) == 24 ? AD9081_FDDC_DCM_24 : AD9081_DCM_BAD)

#define AD9081_CDDC_CODE_ELEM(node_id, prop, idx)                              \
	AD9081_CDDC_DCM_CODE(DT_PROP_BY_IDX(node_id, prop, idx)),
#define AD9081_FDDC_CODE_ELEM(node_id, prop, idx)                              \
	AD9081_FDDC_DCM_CODE(DT_PROP_BY_IDX(node_id, prop, idx)),

#define AD9081_ASSERT_CDDC_ELEM(node_id, prop, idx)                            \
	BUILD_ASSERT(AD9081_CDDC_DCM_CODE(DT_PROP_BY_IDX(node_id, prop, idx)) !=\
			     AD9081_DCM_BAD,                                   \
		     "adi,rx-coarse-decimation: ratio not supported by the part");
#define AD9081_ASSERT_FDDC_ELEM(node_id, prop, idx)                            \
	BUILD_ASSERT(AD9081_FDDC_DCM_CODE(DT_PROP_BY_IDX(node_id, prop, idx)) !=\
			     AD9081_DCM_BAD,                                   \
		     "adi,rx-fine-decimation: ratio not supported by the part");

/*
 * One adi_cms_jesd_param_t from a link-core node.
 *
 * gnode is the node describing this direction; inode is the node carrying the
 * ILAS-only fields (N, CS, S), which only the TX core has -- the framer receives
 * the ILAS it is described in rather than announcing one, so the RX node has no
 * properties for them and axi_jesd204.c reads them from the TX node too.
 *
 * jesd_hd is derived, not stated: see JESD204_DERIVE_HD in jesd204_geometry.h.
 * That is the same expression the FPGA link core uses for its ILAS, so the two
 * ends cannot disagree the way they could when this file held a literal.
 */
#define AD9081_JESD_PARAM(gnode, inode, mode_id)                               \
	{                                                                      \
		.jesd_l = DT_PROP(gnode, adi_lanes_per_device),                 \
		.jesd_f = DT_PROP(gnode, adi_octets_per_frame),                 \
		.jesd_m = DT_PROP(gnode, adi_converters_per_device),            \
		.jesd_s = DT_PROP(inode, adi_samples_per_converter_per_frame),  \
		.jesd_hd = JESD204_DERIVE_HD(                                  \
			DT_PROP(gnode, adi_converters_per_device),              \
			DT_PROP(inode, adi_samples_per_converter_per_frame),    \
			DT_PROP(gnode, adi_bits_per_sample),                   \
			DT_PROP(gnode, adi_lanes_per_device)),                 \
		.jesd_k = DT_PROP(gnode, adi_frames_per_multiframe),            \
		.jesd_n = DT_PROP(inode, adi_converter_resolution),             \
		.jesd_np = DT_PROP(gnode, adi_bits_per_sample),                 \
		.jesd_cs = DT_PROP(inode, adi_control_bits_per_sample),         \
		.jesd_subclass = DT_PROP(gnode, adi_subclass),                  \
		.jesd_scr = JESD204_SCRAMBLING,                                 \
		.jesd_duallink = 0,                                            \
		.jesd_jesdv = JESD204_VERSION_B,                               \
		.jesd_mode_id = (mode_id),                                     \
	}

/* The 16 flat converter-select entries into the vendor's 16 named fields. */
#define AD9081_CONV_SEL(n)                                                     \
	{                                                                      \
		.virtual_converter0_index =                                    \
			DT_INST_PROP_BY_IDX(n, adi_rx_link_converter_select, 0), \
		.virtual_converter1_index =                                    \
			DT_INST_PROP_BY_IDX(n, adi_rx_link_converter_select, 1), \
		.virtual_converter2_index =                                    \
			DT_INST_PROP_BY_IDX(n, adi_rx_link_converter_select, 2), \
		.virtual_converter3_index =                                    \
			DT_INST_PROP_BY_IDX(n, adi_rx_link_converter_select, 3), \
		.virtual_converter4_index =                                    \
			DT_INST_PROP_BY_IDX(n, adi_rx_link_converter_select, 4), \
		.virtual_converter5_index =                                    \
			DT_INST_PROP_BY_IDX(n, adi_rx_link_converter_select, 5), \
		.virtual_converter6_index =                                    \
			DT_INST_PROP_BY_IDX(n, adi_rx_link_converter_select, 6), \
		.virtual_converter7_index =                                    \
			DT_INST_PROP_BY_IDX(n, adi_rx_link_converter_select, 7), \
		.virtual_converter8_index =                                    \
			DT_INST_PROP_BY_IDX(n, adi_rx_link_converter_select, 8), \
		.virtual_converter9_index =                                    \
			DT_INST_PROP_BY_IDX(n, adi_rx_link_converter_select, 9), \
		.virtual_convertera_index =                                    \
			DT_INST_PROP_BY_IDX(n, adi_rx_link_converter_select, 10),\
		.virtual_converterb_index =                                    \
			DT_INST_PROP_BY_IDX(n, adi_rx_link_converter_select, 11),\
		.virtual_converterc_index =                                    \
			DT_INST_PROP_BY_IDX(n, adi_rx_link_converter_select, 12),\
		.virtual_converterd_index =                                    \
			DT_INST_PROP_BY_IDX(n, adi_rx_link_converter_select, 13),\
		.virtual_convertere_index =                                    \
			DT_INST_PROP_BY_IDX(n, adi_rx_link_converter_select, 14),\
		.virtual_converterf_index =                                    \
			DT_INST_PROP_BY_IDX(n, adi_rx_link_converter_select, 15),\
	}

#define AD9081_TX_LINK(n) DT_INST_PHANDLE(n, adi_jesd_tx_link)
#define AD9081_RX_LINK(n) DT_INST_PHANDLE(n, adi_jesd_rx_link)

/* Every array property has one fixed length the vendor API assumes. */
#define AD9081_ASSERT_LEN(n, prop, want)                                       \
	BUILD_ASSERT(DT_INST_PROP_LEN(n, prop) == (want),                      \
		     "adi,ad9081: " #prop " must have " #want " entries");

#define AD9081_ASSERT_LENS(n)                                                  \
	AD9081_ASSERT_LEN(n, adi_tx_dac_channel_crossbar, 4)                   \
	AD9081_ASSERT_LEN(n, adi_tx_main_nco_shift_hz, 4)                      \
	AD9081_ASSERT_LEN(n, adi_tx_channel_nco_shift_hz, 8)                   \
	AD9081_ASSERT_LEN(n, adi_tx_channel_gains, 8)                          \
	AD9081_ASSERT_LEN(n, adi_tx_logical_lane_mapping, 8)                       \
	AD9081_ASSERT_LEN(n, adi_rx_coarse_decimation, 4)                      \
	AD9081_ASSERT_LEN(n, adi_rx_fine_decimation, 8)                        \
	AD9081_ASSERT_LEN(n, adi_rx_coarse_nco_shift_hz, 4)                    \
	AD9081_ASSERT_LEN(n, adi_rx_fine_nco_shift_hz, 8)                      \
	AD9081_ASSERT_LEN(n, adi_rx_logical_lane_mapping, 8)                       \
	AD9081_ASSERT_LEN(n, adi_rx_link_converter_select, 16)                 \
	DT_INST_FOREACH_PROP_ELEM(n, adi_rx_coarse_decimation,                 \
				  AD9081_ASSERT_CDDC_ELEM)                     \
	DT_INST_FOREACH_PROP_ELEM(n, adi_rx_fine_decimation,                   \
				  AD9081_ASSERT_FDDC_ELEM)

#define AD9081_DEFINE(n)                                                       \
	AD9081_ASSERT_LENS(n)                                                  \
									       \
	/* RAM: every vendor entry point below takes these by non-const	       \
	 * pointer. Statically initialised -- nothing is computed at runtime.   \
	 */								       \
	static struct ad9081_data ad9081_data_##n = {                          \
		.profile = {                                                   \
			.tx_dac_chan_xbar =                                    \
				DT_INST_PROP(n, adi_tx_dac_channel_crossbar),   \
			.tx_main_shift =                                       \
				DT_INST_PROP(n, adi_tx_main_nco_shift_hz),      \
			.tx_chan_shift =                                       \
				DT_INST_PROP(n, adi_tx_channel_nco_shift_hz),   \
			.tx_chan_gain = DT_INST_PROP(n, adi_tx_channel_gains),  \
			.tx_lane_map =                                         \
				DT_INST_PROP(n, adi_tx_logical_lane_mapping),       \
			.tx_jesd_param = AD9081_JESD_PARAM(                    \
				AD9081_TX_LINK(n), AD9081_TX_LINK(n),          \
				DT_INST_PROP(n, adi_tx_jesd_mode_id)),         \
									       \
			.rx_cddc_shift =                                       \
				DT_INST_PROP(n, adi_rx_coarse_nco_shift_hz),    \
			.rx_fddc_shift =                                       \
				DT_INST_PROP(n, adi_rx_fine_nco_shift_hz),      \
			.rx_cddc_dcm = { DT_INST_FOREACH_PROP_ELEM(            \
				n, adi_rx_coarse_decimation,                   \
				AD9081_CDDC_CODE_ELEM) },                      \
			.rx_fddc_dcm = { DT_INST_FOREACH_PROP_ELEM(            \
				n, adi_rx_fine_decimation,                     \
				AD9081_FDDC_CODE_ELEM) },                      \
			.rx_cc2r_en = DT_INST_PROP_OR(                         \
				n, adi_rx_coarse_complex_to_real, { 0 }),       \
			.rx_fc2r_en = DT_INST_PROP_OR(                         \
				n, adi_rx_fine_complex_to_real, { 0 }),         \
			.rx_lane_map =                                         \
				DT_INST_PROP(n, adi_rx_logical_lane_mapping),       \
			/* [1] stays zero: single link, jesd_duallink = 0. */   \
			.rx_jesd_param = { [0] = AD9081_JESD_PARAM(            \
				AD9081_RX_LINK(n), AD9081_TX_LINK(n),          \
				DT_INST_PROP(n, adi_rx_jesd_mode_id)) },       \
			.rx_conv_sel = { [0] = AD9081_CONV_SEL(n) },           \
		},                                                             \
	};                                                                     \
									       \
	static const struct ad9081_config ad9081_config_##n = {                \
		.spi = SPI_DT_SPEC_INST_GET(n, SPI_WORD_SET(8) |                \
						      SPI_TRANSFER_MSB |       \
						      SPI_OP_MODE_MASTER),     \
		.reset = GPIO_DT_SPEC_INST_GET_OR(n, reset_gpios, { 0 }),       \
									       \
		.dac_freq_hz =                                                 \
			DT_INST_PROP(n, adi_dac_frequency_khz) * 1000ULL,       \
		.adc_freq_hz =                                                 \
			DT_INST_PROP(n, adi_adc_frequency_khz) * 1000ULL,       \
		.ref_freq_hz =                                                 \
			DT_INST_PROP(n, adi_ref_clk_frequency_khz) * 1000ULL,   \
									       \
		.tx_main_interp = DT_INST_PROP(n, adi_tx_main_interpolation),   \
		.tx_chan_interp =                                              \
			DT_INST_PROP(n, adi_tx_channel_interpolation),          \
									       \
		.rx_cddc_select = DT_INST_PROP(n, adi_rx_coarse_ddc_select),    \
		.rx_fddc_select = DT_INST_PROP(n, adi_rx_fine_ddc_select),      \
		.rx_cddc_decim = DT_INST_PROP(n, adi_rx_coarse_decimation),     \
		.rx_fddc_decim = DT_INST_PROP(n, adi_rx_fine_decimation),       \
	};                                                                     \
									       \
	DEVICE_DT_INST_DEFINE(n, ad9081_init, NULL, &ad9081_data_##n,          \
			      &ad9081_config_##n, POST_KERNEL,                 \
			      CONFIG_AD9081_MXFE_INIT_PRIORITY, &ad9081_api);

DT_INST_FOREACH_STATUS_OKAY(AD9081_DEFINE)

/*
 * The chip is SPI-attached, so it cannot be ready before its bus controller.
 * Same constraint the HMC7044 driver asserts for itself.
 */
BUILD_ASSERT(CONFIG_AD9081_MXFE_INIT_PRIORITY > CONFIG_SPI_INIT_PRIORITY,
	     "The AD9081 is SPI-attached, so it must initialise after its SPI controller");

/*
 * adi,ref-clk-frequency-khz is a statement about a wire: the HMC7044's
 * DEV_REFCLK output drives the chip's REFCLK pin. It is a rate here rather than
 * a phandle (see the binding for why), so the two can drift -- and the failure
 * mode is the CLK PLL solving for a reference it is not being given. Check them
 * against each other at build time instead: PLL2 / that output's divider.
 */
#define AD9081_HMC7044_DEV_REFCLK DT_NODELABEL(hmc7044_c2)

BUILD_ASSERT(DT_INST_PROP(0, adi_ref_clk_frequency_khz) * 1000ULL ==
		     DT_PROP(DT_NODELABEL(hmc7044), adi_pll2_output_frequency) /
			     DT_PROP(AD9081_HMC7044_DEV_REFCLK, adi_divider),
	     "adi,ref-clk-frequency-khz disagrees with the HMC7044 DEV_REFCLK output rate");
