/*
 * AD9081 SPI chip-ID probe (bring-up milestone 0).
 *
 * Proves the SPI path to the AD9081 on the ZynqMP A53: EMIO routing, PS Cadence
 * SPI clocking, chip-select, and bit order -- all confirmed by reading the
 * PROD_ID registers back as 0x9081.
 *
 * No ADI API library is used; the register access is replicated by hand from
 * no-OS adi_ad9081_hal.c (reg_get/reg_set framing) so this stays a tiny,
 * self-contained test.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/init.h>
#include <zephyr/kernel/mm.h>
#include <zephyr/sys/device_mmio.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ad9081_chipid, LOG_LEVEL_INF);

/*
 * The arm64 A53 SoC MMU table does not map the PS SPI register region, and the
 * Cadence SPI driver accesses its base address directly (no DEVICE_MMIO). Map
 * the SPI0 page 1:1 as non-cached device memory at PRE_KERNEL_1, i.e. before
 * the SPI driver initialises at POST_KERNEL. device_map() with
 * CONFIG_KERNEL_DIRECT_MAP=y returns virt == phys, so the driver's DT_REG_ADDR
 * base still works unchanged.
 *
 * This lives in the sample (not the SoC dtsi/mmu table) on purpose -- it keeps
 * the bring-up self-contained. The proper long-term fix is an mmu region entry.
 */
#define AD9081_SPI0_BASE 0xff040000UL
#define AD9081_SPI0_SIZE 0x1000UL

static int ad9081_map_spi0(void)
{
	mm_reg_t virt;

	device_map(&virt, AD9081_SPI0_BASE, AD9081_SPI0_SIZE, K_MEM_CACHE_NONE);

	if (virt != AD9081_SPI0_BASE) {
		/* Not a 1:1 map -- the Cadence driver uses the fixed DT base,
		 * so a relocated vaddr would still fault. Fail loudly.
		 */
		LOG_ERR("SPI0 not identity-mapped: virt=0x%lx phys=0x%lx",
			(unsigned long)virt, AD9081_SPI0_BASE);
		return -EIO;
	}
	return 0;
}

SYS_INIT(ad9081_map_spi0, PRE_KERNEL_1, 0);

/* AD9081 register map (from no-OS adi_ad9081_bf_ad9081.h / _bf_main.h). */
#define AD9081_REG_SPI_INTFCONFA  0x0000  /* SPI interface config A */
#define AD9081_REG_PROD_ID_LSB    0x0004
#define AD9081_REG_PROD_ID_MSB    0x0005

/* The AD9081 and AD9082 are the same MxFE family and share this driver / SPI
 * map; no-OS accepts either PROD_ID. Which one is populated is a board fit.
 */
#define AD9081_CHIPID             0x9081
#define AD9082_CHIPID             0x9082

/*
 * SPI_INTFCONFA value for 4-wire (SDO active), address auto-increment,
 * MSB-first. Mirrors adi_ad9081_device_spi_config() with sdo=SPI_SDO,
 * msb=SPI_MSB_FIRST, addr_inc=SPI_ADDR_INC_AUTO:
 *   0x18 (SDO) | 0x00 (MSB first) | 0x24 (addr auto-inc) = 0x3C
 */
#define AD9081_INTFCONFA_4WIRE    0x3C

static const struct spi_dt_spec ad9081 = SPI_DT_SPEC_GET(
	DT_NODELABEL(ad9081),
	SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_OP_MODE_MASTER,
	0);

static const struct gpio_dt_spec ad9081_reset =
	GPIO_DT_SPEC_GET_OR(DT_NODELABEL(ad9081), reset_gpios, {0});

/* Pulse the AD9081 RSTB. Timing mirrors no-OS adi_ad9081_device_reset()
 * (HARD_RESET): assert low, wait, release, then wait for the part to settle.
 */
static int ad9081_hw_reset(void)
{
	int ret;

	if (ad9081_reset.port == NULL) {
		LOG_WRN("no reset-gpios; skipping hardware reset");
		return 0;
	}

	if (!gpio_is_ready_dt(&ad9081_reset)) {
		LOG_ERR("reset GPIO %s not ready", ad9081_reset.port->name);
		return -ENODEV;
	}

	/* Start asserted (active-low RSTB driven low). */
	ret = gpio_pin_configure_dt(&ad9081_reset, GPIO_OUTPUT_ACTIVE);
	if (ret) {
		LOG_ERR("reset GPIO configure failed (%d)", ret);
		return ret;
	}

	k_msleep(10);                       /* hold in reset */
	gpio_pin_set_dt(&ad9081_reset, 0);  /* de-assert: RSTB high */
	k_msleep(10);                       /* let the part come up */

	LOG_INF("AD9081 hardware reset done (RSTB released)");
	return 0;
}

/* Write one 8-bit register: [ (reg>>8)&0x3F, reg&0xFF, val ]. Bit 7 clear = write. */
static int ad9081_reg_write(uint16_t reg, uint8_t val)
{
	uint8_t tx[3] = {
		(reg >> 8) & 0x3F,
		reg & 0xFF,
		val,
	};
	const struct spi_buf txb = { .buf = tx, .len = sizeof(tx) };
	const struct spi_buf_set txs = { .buffers = &txb, .count = 1 };

	return spi_write_dt(&ad9081, &txs);
}

/* Read one 8-bit register. Bit 7 of the first addr byte set = read. */
static int ad9081_reg_read(uint16_t reg, uint8_t *val)
{
	uint8_t tx[3] = {
		((reg >> 8) & 0x3F) | 0x80,
		reg & 0xFF,
		0x00,
	};
	uint8_t rx[3] = { 0 };
	const struct spi_buf txb = { .buf = tx, .len = sizeof(tx) };
	const struct spi_buf rxb = { .buf = rx, .len = sizeof(rx) };
	const struct spi_buf_set txs = { .buffers = &txb, .count = 1 };
	const struct spi_buf_set rxs = { .buffers = &rxb, .count = 1 };
	int ret;

	ret = spi_transceive_dt(&ad9081, &txs, &rxs);
	if (ret == 0) {
		*val = rx[2];
	}
	return ret;
}

int main(void)
{
	uint8_t lsb = 0, msb = 0;
	uint16_t prod_id;
	int ret;

	LOG_INF("AD9081 chip-ID probe over %s", ad9081.bus->name);

	if (!spi_is_ready_dt(&ad9081)) {
		LOG_ERR("SPI bus %s not ready", ad9081.bus->name);
		return -ENODEV;
	}

	ret = ad9081_hw_reset();
	if (ret) {
		return ret;
	}

	/*
	 * Put the SPI port into a known 4-wire / MSB-first / auto-increment
	 * mode before reading anything back. On a hard-reset part this is the
	 * power-up default too, but writing it explicitly makes the probe
	 * robust regardless of prior state.
	 */
	ret = ad9081_reg_write(AD9081_REG_SPI_INTFCONFA, AD9081_INTFCONFA_4WIRE);
	if (ret) {
		LOG_ERR("SPI_INTFCONFA write failed (%d)", ret);
		return ret;
	}
	k_msleep(1);

	ret = ad9081_reg_read(AD9081_REG_PROD_ID_LSB, &lsb);
	if (ret) {
		LOG_ERR("PROD_ID_LSB read failed (%d)", ret);
		return ret;
	}

	ret = ad9081_reg_read(AD9081_REG_PROD_ID_MSB, &msb);
	if (ret) {
		LOG_ERR("PROD_ID_MSB read failed (%d)", ret);
		return ret;
	}

	prod_id = ((uint16_t)msb << 8) | lsb;
	LOG_INF("PROD_ID = 0x%04x (LSB=0x%02x MSB=0x%02x)", prod_id, lsb, msb);

	if (prod_id == AD9081_CHIPID || prod_id == AD9082_CHIPID) {
		LOG_INF("SUCCESS: AD90%02x detected over SPI", prod_id & 0xFF);
	} else {
		LOG_ERR("Unexpected PROD_ID 0x%04x (expected 0x9081 or 0x9082)",
			prod_id);
		LOG_ERR("If 0x0000/0xffff: check RSTB (may need EMIO reset "
			"GPIO), CS wiring, or SPI ref clock");
	}

	return 0;
}
