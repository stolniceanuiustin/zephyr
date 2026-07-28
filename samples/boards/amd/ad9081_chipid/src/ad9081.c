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
#include <zephyr/init.h>
#include <zephyr/kernel/mm.h>
#include <zephyr/sys/device_mmio.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ad9081, LOG_LEVEL_INF);

#include "ad9081.h"
#include "adi_ad9081.h"

/*
 * The arm64 A53 SoC MMU table does not map the PS SPI register region, and the
 * Cadence SPI driver accesses its base address directly (no DEVICE_MMIO). Map
 * the SPI0 page 1:1 as non-cached device memory at PRE_KERNEL_1, i.e. before
 * the SPI driver initialises at POST_KERNEL. device_map() with
 * CONFIG_KERNEL_DIRECT_MAP=y returns virt == phys, so the driver's DT_REG_ADDR
 * base still works unchanged.
 */
#define AD9081_SPI0_BASE 0xff040000UL
#define AD9081_SPI0_SIZE 0x1000UL

static int ad9081_map_spi0(void)
{
	mm_reg_t virt;

	device_map(&virt, AD9081_SPI0_BASE, AD9081_SPI0_SIZE, K_MEM_CACHE_NONE);

	if (virt != AD9081_SPI0_BASE) {
		LOG_ERR("SPI0 not identity-mapped: virt=0x%lx phys=0x%lx",
			(unsigned long)virt, AD9081_SPI0_BASE);
		return -EIO;
	}
	return 0;
}

SYS_INIT(ad9081_map_spi0, PRE_KERNEL_1, 0);

/* The AD9081 and AD9082 are the same MxFE family and share this driver / SPI
 * map; the ADI library accepts either PROD_ID.
 */
#define AD9081_CHIPID 0x9081
#define AD9082_CHIPID 0x9082

static const struct spi_dt_spec ad9081_spi = SPI_DT_SPEC_GET(
	DT_NODELABEL(ad9081),
	SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_OP_MODE_MASTER,
	0);

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
