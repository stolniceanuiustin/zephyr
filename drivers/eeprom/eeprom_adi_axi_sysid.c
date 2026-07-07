/*
 * Copyright (c) 2026 Analog Devices, Inc.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Read-only EEPROM driver for the Analog Devices AXI System ID core.
 *
 * The core exposes an FPGA build-info ROM behind a small MMIO register block.
 * The ROM content (board / product / git information) is presented here as a
 * read-only EEPROM: eeprom_read() returns the raw ROM bytes and eeprom_get_size()
 * the ROM size. Knowledge of the ROM *format* (header, build-info, checksum)
 * is intentionally kept out of the transport driver — a consumer layers a
 * decode helper on top of eeprom_read() (see samples/sensor/ad463x).
 *
 * Based on the no-OS reference driver by Analog Devices.
 */

#define DT_DRV_COMPAT adi_axi_sysid

#include <zephyr/device.h>
#include <zephyr/drivers/eeprom.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/device_mmio.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(eeprom_axi_sysid, CONFIG_EEPROM_LOG_LEVEL);

#define AXI_SYSID_REG_VERSION		0x0000
#define AXI_SYSID_REG_ROM_ADDR_WIDTH	0x0040

#define AXI_SYSID_VER_1_00_A		0x10061
#define AXI_SYSID_VER_1_01_A		0x10161

/* addr_width guard: max 10 keeps (1 << addr_width) words within a 4 KiB ROM. */
#define AXI_SYSID_MAX_ADDR_WIDTH	10

struct axi_sysid_config {
	DEVICE_MMIO_ROM;
};

struct axi_sysid_data {
	DEVICE_MMIO_RAM;
	size_t rom_size;	/* ROM size in bytes (== EEPROM size) */
	off_t rom_offset;	/* MMIO byte offset where ROM content begins */
};

static uint32_t axi_sysid_read_reg(const struct device *dev, uint32_t reg)
{
	return sys_read32(DEVICE_MMIO_GET(dev) + reg);
}

static int axi_sysid_read(const struct device *dev, off_t offset, void *buf,
			  size_t len)
{
	struct axi_sysid_data *data = dev->data;
	mm_reg_t rom = DEVICE_MMIO_GET(dev) + data->rom_offset;
	uint8_t *out = buf;

	if (offset < 0 || (size_t)offset + len > data->rom_size) {
		LOG_ERR("read [%ld, +%zu) past ROM size %zu",
			(long)offset, len, data->rom_size);
		return -EINVAL;
	}

	/*
	 * The ROM is 32-bit word addressable MMIO. Assemble arbitrary byte
	 * ranges from little-endian word reads so callers see a flat byte
	 * array matching the on-ROM layout.
	 */
	while (len != 0) {
		off_t word_off = offset & ~(off_t)0x3;
		size_t byte_in_word = offset & 0x3;
		size_t n = MIN(sizeof(uint32_t) - byte_in_word, len);
		uint8_t word[sizeof(uint32_t)];

		sys_put_le32(sys_read32(rom + word_off), word);
		memcpy(out, &word[byte_in_word], n);

		out += n;
		offset += n;
		len -= n;
	}

	return 0;
}

static int axi_sysid_write(const struct device *dev, off_t offset,
			   const void *buf, size_t len)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(offset);
	ARG_UNUSED(buf);
	ARG_UNUSED(len);

	/* The System ID ROM is a build-time artifact; it is read-only. */
	return -EACCES;
}

static size_t axi_sysid_size(const struct device *dev)
{
	struct axi_sysid_data *data = dev->data;

	return data->rom_size;
}

static int axi_sysid_init(const struct device *dev)
{
	struct axi_sysid_data *data = dev->data;
	uint32_t version;
	uint32_t addr_width;

	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);

	version = axi_sysid_read_reg(dev, AXI_SYSID_REG_VERSION);
	if (version != AXI_SYSID_VER_1_00_A && version != AXI_SYSID_VER_1_01_A) {
		LOG_ERR("unsupported version: 0x%08x", version);
		return -ENOTSUP;
	}

	addr_width = axi_sysid_read_reg(dev, AXI_SYSID_REG_ROM_ADDR_WIDTH);
	if (addr_width > AXI_SYSID_MAX_ADDR_WIDTH) {
		LOG_ERR("ROM addr width %u out of range (max %u)",
			addr_width, AXI_SYSID_MAX_ADDR_WIDTH);
		return -ENOTSUP;
	}

	data->rom_size = (1U << addr_width) * sizeof(uint32_t);
	/* ROM content is mapped immediately after the register/ROM window. */
	data->rom_offset = data->rom_size;

	LOG_INF("AXI SYSID v%u.%u.%c, ROM %zu bytes (read-only)",
		version >> 16, (version >> 8) & 0xff, version & 0xff,
		data->rom_size);

	return 0;
}

static DEVICE_API(eeprom, axi_sysid_driver_api) = {
	.read = axi_sysid_read,
	.write = axi_sysid_write,
	.size = axi_sysid_size,
};

#define AXI_SYSID_INIT(n)						\
	static struct axi_sysid_data axi_sysid_data_##n;		\
	static const struct axi_sysid_config axi_sysid_config_##n = {	\
		DEVICE_MMIO_ROM_INIT(DT_DRV_INST(n)),			\
	};								\
	DEVICE_DT_INST_DEFINE(n,					\
			      axi_sysid_init,				\
			      NULL,					\
			      &axi_sysid_data_##n,			\
			      &axi_sysid_config_##n,			\
			      POST_KERNEL,				\
			      CONFIG_EEPROM_INIT_PRIORITY,		\
			      &axi_sysid_driver_api);

DT_INST_FOREACH_STATUS_OKAY(AXI_SYSID_INIT)
