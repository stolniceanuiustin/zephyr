/*
 * Copyright (c) 2026 Analog Devices, Inc.
 *
 * Board-level workaround: identity-map the PS SPI register pages.
 *
 * WHY THIS EXISTS
 * ---------------
 * The upstream Cadence SPI driver stores DT_INST_REG_ADDR(n) straight into
 * cfg->base (drivers/spi/spi_cdns.c) and never maps it -- it does not use
 * DEVICE_MMIO_ROM/DEVICE_MMIO_MAP. On arm64 ZynqMP the A53 SoC MMU tables do not
 * cover the PS SPI region, so the first register access from that driver faults.
 * It also initialises at POST_KERNEL, i.e. after the point where the mapping
 * would need to exist.
 *
 * So the pages are mapped here at PRE_KERNEL_1. device_map() with
 * CONFIG_KERNEL_DIRECT_MAP=y returns virt == phys, which is what makes the SPI
 * driver's unmapped DT_REG_ADDR base work unchanged.
 *
 * WHY IT IS HERE AND NOT IN THE CHIP DRIVERS
 * ------------------------------------------
 * It used to be two copies, one inside hmc7044.c and one inside ad9081.c, each
 * mapping its own parent bus controller's register page. A converter driver
 * reaching for its bus controller's registers is not something that should
 * survive into a driver-model conversion: the driver would be unusable on any
 * other board, and the wart is not the chip's. A sample owning one documented
 * board-level workaround is fine.
 *
 * There is no board directory this could live in instead -- there is no A53
 * zcu102 board in tree (only boards/amd/zcu102_r5), and zynqmp_a53.dtsi carries
 * no spi@ nodes at all, which is why the sample's overlay defines them.
 *
 * THE REAL FIX, IF IT IS EVER WORTH DOING
 * ---------------------------------------
 * A ~10-line standalone PR against spi_cdns.c: DEVICE_MMIO_ROM in the config,
 * DEVICE_MMIO_RAM in the data, DEVICE_MMIO_MAP in spi_cdns_init(). That is a
 * separate upstream contribution with its own review cycle, fully decoupled from
 * this sample. When it lands, delete this file.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/init.h>
#include <zephyr/kernel/mm.h>
#include <zephyr/sys/device_mmio.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(spi_mmio_fixup, LOG_LEVEL_INF);

/*
 * Every sys_read32/sys_write32 in this sample -- here and in the PL core drivers
 * -- assumes the mapping is 1:1. On arm64 that is guaranteed rather than
 * configured: arch/arm64/core/Kconfig does `select KERNEL_DIRECT_MAP if MMU`.
 * Asserted anyway, because the dependency is invisible at every use site.
 */
BUILD_ASSERT(IS_ENABLED(CONFIG_KERNEL_DIRECT_MAP),
	     "This sample requires CONFIG_KERNEL_DIRECT_MAP: it relies on "
	     "device_map() returning virt == phys so that unmapped DT_REG_ADDR "
	     "bases (upstream spi_cdns.c) and raw sys_read32() of the PL cores work.");

/* Map every cdns,spi node the devicetree enables, rather than a hardcoded list. */
#define SPI_CDNS_MAP(node)                                                                 \
	{                                                                                  \
		.name = DT_NODE_FULL_NAME(node),                                           \
		.phys = DT_REG_ADDR(node),                                                 \
		.size = DT_REG_SIZE(node),                                                 \
	},

static const struct {
	const char *name;
	uintptr_t phys;
	size_t size;
} spi_pages[] = {
	DT_FOREACH_STATUS_OKAY(cdns_spi, SPI_CDNS_MAP)
};

static int spi_mmio_fixup(void)
{
	int ret = 0;

	for (size_t i = 0; i < ARRAY_SIZE(spi_pages); i++) {
		mm_reg_t virt;

		device_map(&virt, spi_pages[i].phys, spi_pages[i].size,
			   K_MEM_CACHE_NONE);

		if (virt != spi_pages[i].phys) {
			LOG_ERR("%s not identity-mapped: virt=0x%lx phys=0x%lx",
				spi_pages[i].name, (unsigned long)virt,
				(unsigned long)spi_pages[i].phys);
			ret = -EIO;
		}
	}

	return ret;
}

SYS_INIT(spi_mmio_fixup, PRE_KERNEL_1, 0);
