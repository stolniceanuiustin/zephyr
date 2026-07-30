/*
 * AD9081/AD9082 MxFE -- Zephyr HAL shim for the ADI API library.
 *
 * The ADI API library (src/adi_api/, copied verbatim from no-OS) is fully
 * hardware-abstracted: it reaches the chip only through the function pointers in
 * adi_ad9081_device_t.hal_info. This file implements those callbacks against
 * Zephyr SPI/GPIO and drives the library to init the device and read its ID.
 *
 * Callbacks implemented (of the AD9081 HAL set):
 *   spi_xfer       -> spi_transceive_dt (SPI0)
 *   delay_us       -> k_busy_wait
 *   reset_pin_ctrl -> gpio_pin_set_dt (RSTB, active low)
 *   log_write      -> LOG_*
 *   hw_open/close  -> no-op (bus is brought up by Zephyr already)
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <string.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ad9081, LOG_LEVEL_INF);

#include "ad9081.h"
#include "adi_ad9081.h"

/*
 * The SPI0 register page is mapped 1:1 by spi_mmio_fixup.c at PRE_KERNEL_1,
 * working around the upstream Cadence SPI driver not using DEVICE_MMIO. That is
 * a board-level wart, documented there, and deliberately not this file's
 * business.
 */

/* The AD9081 and AD9082 are the same MxFE family and share this driver / SPI
 * map; the ADI library accepts either PROD_ID.
 */
#define AD9081_CHIPID 0x9081
#define AD9082_CHIPID 0x9082

static const struct spi_dt_spec ad9081_spi = SPI_DT_SPEC_GET(
	DT_NODELABEL(ad9081),
	SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_OP_MODE_MASTER);

static const struct gpio_dt_spec ad9081_reset =
	GPIO_DT_SPEC_GET_OR(DT_NODELABEL(ad9081), reset_gpios, {0});

/*
 * ------------------------- ADI HAL callbacks ---------------------------------
 * All take the void *user_data we install as hal_info.user_data (unused here --
 * the SPI/GPIO handles are file-static). Return API_CMS_ERROR_* codes.
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
	ARG_UNUSED(user_data);
	uint32_t n = size_bytes & 0xFF;
	const struct spi_buf txb = { .buf = in_data, .len = n };
	const struct spi_buf rxb = { .buf = out_data, .len = n };
	const struct spi_buf_set txs = { .buffers = &txb, .count = 1 };
	const struct spi_buf_set rxs = { .buffers = &rxb, .count = 1 };

	if (spi_transceive_dt(&ad9081_spi, &txs, &rxs) != 0) {
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
	ARG_UNUSED(user_data);
	if (ad9081_reset.port == NULL) {
		return API_CMS_ERROR_OK; /* no reset line wired */
	}
	gpio_pin_set_dt(&ad9081_reset, enable ? 0 : 1);
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

static adi_ad9081_device_t ad9081_dev;

static int ad9081_hal_bind(void)
{
	adi_ad9081_hal_t *hal = &ad9081_dev.hal_info;

	hal->user_data = NULL;

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

	return 0;
}

int ad9081_probe(uint16_t *prod_id)
{
	adi_cms_chip_id_t chip_id = { 0 };
	int32_t err;

	LOG_INF("AD9081 probe via ADI API lib over %s", ad9081_spi.bus->name);

	if (!spi_is_ready_dt(&ad9081_spi)) {
		LOG_ERR("SPI bus %s not ready", ad9081_spi.bus->name);
		return -ENODEV;
	}

	if (ad9081_reset.port != NULL) {
		if (!gpio_is_ready_dt(&ad9081_reset)) {
			LOG_ERR("reset GPIO not ready");
			return -ENODEV;
		}
		/* Configure de-asserted; the library pulses it via
		 * reset_pin_ctrl during device_reset (HARD_RESET).
		 */
		gpio_pin_configure_dt(&ad9081_reset, GPIO_OUTPUT_INACTIVE);
	}

	ad9081_hal_bind();

	/*
	 * Hard reset (pulses RSTB via reset_pin_ctrl) then init. The _AND_INIT
	 * variant calls adi_ad9081_device_init() internally: SPI config +
	 * 8-bit reg access check + power-status check. No PLL boot yet.
	 */
	err = adi_ad9081_device_reset(&ad9081_dev, AD9081_HARD_RESET_AND_INIT);
	if (err != API_CMS_ERROR_OK) {
		LOG_ERR("adi_ad9081_device_reset failed (%d)", err);
		return -EIO;
	}

	err = adi_ad9081_device_chip_id_get(&ad9081_dev, &chip_id);
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

void *ad9081_get_device(void)
{
	return &ad9081_dev;
}

/*
 * ------------------------- MxFE datapath configuration ------------------------
 *
 * Parameters are the zcu102_ad9081_m8_l4 profile (no-OS
 * projects/ad9081/profiles/zcu102_ad9081_m8_l4/app_config.h), driven through the
 * ADI API exactly as no-OS ad9081_setup()/setup_tx()/setup_rx().
 *
 *   DAC 12 GHz, ADC 4 GHz, device REFCLK 250 MHz (on-chip CLK PLL used).
 *   TX: main interp 6, chan interp 8; JRX deframer mode 9, M8 L4 F4 K32 NP16.
 *   RX: coarse/fine decim 4/4; JTX framer mode 10, M8 L4 F4 K32 NP16.
 */

/* Clocks. */
#define AD9081_DAC_CLK_HZ 12000000000ULL
#define AD9081_ADC_CLK_HZ 4000000000ULL
#define AD9081_REF_CLK_HZ 250000000ULL

/* TX (JRX / deframer) datapath. */
#define AD9081_TX_MAIN_INTERP 6
#define AD9081_TX_CHAN_INTERP 8

static uint8_t tx_dac_chan_xbar[4] = { 0x1, 0x2, 0x4, 0x8 };
static int64_t tx_main_shift[4] = { 2000000000, 2000000000, 2000000000,
				    2000000000 };
static int64_t tx_chan_shift[8] = { 0 };
static uint16_t tx_chan_gain[8] = { 1024, 1024, 1024, 1024, 0, 0, 0, 0 };
static uint8_t tx_logical_lane_map[8] = { 0, 2, 7, 7, 1, 7, 7, 3 };

static adi_cms_jesd_param_t tx_jesd_param = {
	.jesd_l = 4,
	.jesd_f = 4,
	.jesd_m = 8,
	.jesd_s = 1,
	.jesd_hd = 1,
	.jesd_k = 32,
	.jesd_n = 16,
	.jesd_np = 16,
	.jesd_cs = 0,
	.jesd_subclass = 1,
	.jesd_scr = 1,
	.jesd_duallink = 0,
	.jesd_jesdv = 1,
	.jesd_mode_id = 9,
};

/* RX (JTX / framer) datapath. */
static uint8_t rx_cddc_select = 0xF;  /* all 4 coarse DDCs */
static uint8_t rx_fddc_select = 0x33; /* fine DDCs 0,1,4,5 */
static int64_t rx_cddc_shift[4] = { 2000000000, 2000000000, 2000000000,
				    2000000000 };
static int64_t rx_fddc_shift[8] = { 0 };
static uint8_t rx_cddc_dcm[4] = { AD9081_CDDC_DCM_4, AD9081_CDDC_DCM_4,
				  AD9081_CDDC_DCM_4, AD9081_CDDC_DCM_4 };
static uint8_t rx_fddc_dcm[8] = { AD9081_FDDC_DCM_4, AD9081_FDDC_DCM_4, 0, 0,
				  AD9081_FDDC_DCM_4, AD9081_FDDC_DCM_4, 0, 0 };
static uint8_t rx_cc2r_en[4] = { 0 };
static uint8_t rx_fc2r_en[8] = { 0 };
static uint8_t rx_logical_lane_map[8] = { 2, 0, 7, 7, 7, 7, 3, 1 };
static uint8_t rx_link_converter_select[16] = { 0, 1, 2, 3, 8, 9, 10, 11,
						0, 0, 0, 0, 0,  0, 0,  0 };

static adi_cms_jesd_param_t rx_jesd_param[2] = {
	[0] = {
		.jesd_l = 4,
		.jesd_f = 4,
		.jesd_m = 8,
		.jesd_s = 1,
		.jesd_hd = 1,
		.jesd_k = 32,
		.jesd_n = 16,
		.jesd_np = 16,
		.jesd_cs = 0,
		.jesd_subclass = 1,
		.jesd_scr = 1,
		.jesd_duallink = 0,
		.jesd_jesdv = 1,
		.jesd_mode_id = 10,
	},
};

static adi_ad9081_jtx_conv_sel_t rx_jesd_conv_sel[2];

/* Map the flat converter-select array into the ADI per-virtual-converter struct. */
static void ad9081_fill_conv_sel(void)
{
	adi_ad9081_jtx_conv_sel_t *s = &rx_jesd_conv_sel[0];

	s->virtual_converter0_index = rx_link_converter_select[0];
	s->virtual_converter1_index = rx_link_converter_select[1];
	s->virtual_converter2_index = rx_link_converter_select[2];
	s->virtual_converter3_index = rx_link_converter_select[3];
	s->virtual_converter4_index = rx_link_converter_select[4];
	s->virtual_converter5_index = rx_link_converter_select[5];
	s->virtual_converter6_index = rx_link_converter_select[6];
	s->virtual_converter7_index = rx_link_converter_select[7];
	s->virtual_converter8_index = rx_link_converter_select[8];
	s->virtual_converter9_index = rx_link_converter_select[9];
	s->virtual_convertera_index = rx_link_converter_select[10];
	s->virtual_converterb_index = rx_link_converter_select[11];
	s->virtual_converterc_index = rx_link_converter_select[12];
	s->virtual_converterd_index = rx_link_converter_select[13];
	s->virtual_convertere_index = rx_link_converter_select[14];
	s->virtual_converterf_index = rx_link_converter_select[15];
}

int ad9081_setup_datapath(void)
{
	adi_ad9081_device_t *dev = &ad9081_dev;
	uint8_t pll_status;
	int32_t err;

	LOG_INF("AD9081 datapath setup (DAC 12G / ADC 4G, ref 250M)");

	/* On-chip CLK PLL: ref (250M) != dac (12G), so the PLL is engaged. */
	err = adi_ad9081_device_clk_config_set(dev, AD9081_DAC_CLK_HZ,
					       AD9081_ADC_CLK_HZ,
					       AD9081_REF_CLK_HZ);
	if (err != API_CMS_ERROR_OK) {
		LOG_ERR("clk_config_set failed (%d)", err);
		return -EIO;
	}

	err = adi_ad9081_device_clk_pll_lock_status_get(dev, &pll_status);
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
	memcpy(dev->serdes_info.des_settings.lane_mapping[0],
	       tx_logical_lane_map, sizeof(tx_logical_lane_map));

	err = adi_ad9081_device_startup_tx(dev, AD9081_TX_MAIN_INTERP,
					   AD9081_TX_CHAN_INTERP,
					   tx_dac_chan_xbar, tx_main_shift,
					   tx_chan_shift, &tx_jesd_param);
	if (err != API_CMS_ERROR_OK) {
		LOG_ERR("device_startup_tx failed (%d)", err);
		return -EIO;
	}

	err = adi_ad9081_dac_duc_nco_gains_set(dev, tx_chan_gain);
	if (err != API_CMS_ERROR_OK) {
		LOG_ERR("dac_duc_nco_gains_set failed (%d)", err);
		return -EIO;
	}
	LOG_INF("TX datapath up: interp %ux%u, JRX deframer mode 9 (M8 L4 F4)",
		AD9081_TX_MAIN_INTERP, AD9081_TX_CHAN_INTERP);

	/* RX framer (JTX): install chip-side lane map + converter select. */
	memcpy(dev->serdes_info.ser_settings.lane_mapping[0],
	       rx_logical_lane_map, sizeof(rx_logical_lane_map));
	ad9081_fill_conv_sel();

	err = adi_ad9081_device_startup_rx(dev, rx_cddc_select, rx_fddc_select,
					   rx_cddc_shift, rx_fddc_shift,
					   rx_cddc_dcm, rx_fddc_dcm, rx_cc2r_en,
					   rx_fc2r_en, rx_jesd_param,
					   rx_jesd_conv_sel);
	if (err != API_CMS_ERROR_OK) {
		LOG_ERR("device_startup_rx failed (%d)", err);
		return -EIO;
	}
	LOG_INF("RX datapath up: decim 4/4, JTX framer mode 10 (M8 L4 F4)");

	return 0;
}
