/*
 * HMC7044 clock/SYSREF chip -- SPI bring-up helpers.
 *
 * Proves the SPI path to the HMC7044 on the ZynqMP A53: EMIO routing, PS
 * Cadence SPI1 clocking, chip-select, and command framing.
 *
 * The HMC7044 has NO chip-ID register, so -- exactly like no-OS
 * hmc7044_read_write_check() -- we prove the bus by writing a known byte to the
 * scratchpad register (0x0008) and reading it back.
 *
 * No driver/library is used yet; the register access is replicated by hand from
 * no-OS hmc7044.c (hmc7044_read/hmc7044_write framing). Later this grows into
 * the drivers/clock_control/hmc7044.c driver (clock_control API + custom SYSREF
 * verbs).
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/init.h>
#include <zephyr/kernel/mm.h>
#include <zephyr/sys/device_mmio.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(hmc7044, LOG_LEVEL_INF);

#include "hmc7044.h"

/*
 * Same MMU story as SPI0 in ad9081.c: map the SPI1 register page 1:1 as
 * non-cached device memory at PRE_KERNEL_1, before the Cadence SPI driver
 * initialises at POST_KERNEL.
 */
#define HMC7044_SPI1_BASE 0xff050000UL
#define HMC7044_SPI1_SIZE 0x1000UL

static int hmc7044_map_spi1(void)
{
	mm_reg_t virt;

	device_map(&virt, HMC7044_SPI1_BASE, HMC7044_SPI1_SIZE, K_MEM_CACHE_NONE);

	if (virt != HMC7044_SPI1_BASE) {
		LOG_ERR("SPI1 not identity-mapped: virt=0x%lx phys=0x%lx",
			(unsigned long)virt, HMC7044_SPI1_BASE);
		return -EIO;
	}
	return 0;
}

SYS_INIT(hmc7044_map_spi1, PRE_KERNEL_1, 0);

/* HMC7044 command word (from no-OS hmc7044.c):
 *   bit 15    : 1 = read, 0 = write
 *   bits 14:13: (count - 1)      -- 1 byte here -> 0
 *   bits 11:0 : register address
 */
#define HMC7044_READ    (1U << 15)
#define HMC7044_WRITE   (0U << 15)
#define HMC7044_CNT(x)  (((x) - 1U) << 13)
#define HMC7044_ADDR(x) ((x) & 0xFFFU)

#define HMC7044_REG_SCRATCHPAD  0x0008
#define HMC7044_SCRATCH_PATTERN 0xAD

static const struct spi_dt_spec hmc7044 = SPI_DT_SPEC_GET(
	DT_NODELABEL(hmc7044),
	SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_OP_MODE_MASTER,
	0);

/* Write one 8-bit register: [ cmd_hi, cmd_lo, val ]. */
static int hmc7044_reg_write(uint16_t reg, uint8_t val)
{
	uint16_t cmd = HMC7044_WRITE | HMC7044_CNT(1) | HMC7044_ADDR(reg);
	uint8_t tx[3] = {
		cmd >> 8,
		cmd & 0xFF,
		val,
	};
	const struct spi_buf txb = { .buf = tx, .len = sizeof(tx) };
	const struct spi_buf_set txs = { .buffers = &txb, .count = 1 };

	return spi_write_dt(&hmc7044, &txs);
}

/* Read one 8-bit register. Data returns in the third byte. */
static int hmc7044_reg_read(uint16_t reg, uint8_t *val)
{
	uint16_t cmd = HMC7044_READ | HMC7044_CNT(1) | HMC7044_ADDR(reg);
	uint8_t tx[3] = {
		cmd >> 8,
		cmd & 0xFF,
		0x00,
	};
	uint8_t rx[3] = { 0 };
	const struct spi_buf txb = { .buf = tx, .len = sizeof(tx) };
	const struct spi_buf rxb = { .buf = rx, .len = sizeof(rx) };
	const struct spi_buf_set txs = { .buffers = &txb, .count = 1 };
	const struct spi_buf_set rxs = { .buffers = &rxb, .count = 1 };
	int ret;

	ret = spi_transceive_dt(&hmc7044, &txs, &rxs);
	if (ret == 0) {
		*val = rx[2];
	}
	return ret;
}

int hmc7044_probe(void)
{
	uint8_t val = 0;
	int ret;

	LOG_INF("HMC7044 scratchpad probe over %s", hmc7044.bus->name);

	if (!spi_is_ready_dt(&hmc7044)) {
		LOG_ERR("SPI bus %s not ready", hmc7044.bus->name);
		return -ENODEV;
	}

	/* Mirror no-OS hmc7044_read_write_check(): write 0xAD, read it back. */
	ret = hmc7044_reg_write(HMC7044_REG_SCRATCHPAD, HMC7044_SCRATCH_PATTERN);
	if (ret) {
		LOG_ERR("scratchpad write failed (%d)", ret);
		return ret;
	}

	ret = hmc7044_reg_read(HMC7044_REG_SCRATCHPAD, &val);
	if (ret) {
		LOG_ERR("scratchpad read failed (%d)", ret);
		return ret;
	}

	LOG_INF("scratchpad readback = 0x%02x (wrote 0x%02x)",
		val, HMC7044_SCRATCH_PATTERN);

	if (val != HMC7044_SCRATCH_PATTERN) {
		LOG_ERR("Scratchpad mismatch (got 0x%02x, expected 0x%02x)",
			val, HMC7044_SCRATCH_PATTERN);
		LOG_ERR("If 0x00/0xff: check CS wiring (CLK_CS=0 on SPI1), "
			"SPI ref clock, or 3-/4-wire mode");
		return -ENODEV;
	}

	return 0;
}
