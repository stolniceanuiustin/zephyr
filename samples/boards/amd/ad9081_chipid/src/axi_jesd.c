/*
 * AXI JESD204 link cores (RX/TX) -- PL AXI plane probe.
 *
 * This is the fabric equivalent of the SPI scratchpad check: before bringing up
 * the JESD204B link we prove the PL AXI plane is alive and the loaded bitstream
 * is the MxFE m8-l4 design we expect. Each core exposes read-only identity
 * registers -- a MAGIC tag, a PCORE VERSION, and a SYNTH_NUM_LANES synthesis
 * parameter -- which we read back and validate.
 *
 * Register offsets and the MAGIC/version encodings are transcribed from no-OS
 * drivers/axi_core/jesd204/axi_jesd204_{rx,tx}.c (ground truth for the ADI IP).
 * The AXI base addresses come from the on-board bitstream's system.hwh.
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
LOG_MODULE_REGISTER(axi_jesd, LOG_LEVEL_INF);

#include "axi_jesd.h"

/*
 * AXI base addresses (from the on-board bitstream system.hwh, PS M_AXI_HPM0_LPD
 * plane). Like the PS SPI pages, these are NOT in the A53 SoC MMU table -- each
 * needs a 1:1 device_map(K_MEM_CACHE_NONE) before first access.
 */
#define AXI_JESD_RX_BASE 0x84A90000UL
#define AXI_JESD_TX_BASE 0x84B90000UL
#define AXI_JESD_SIZE    0x10000UL

/* Register offsets (common to axi_jesd204_rx and axi_jesd204_tx). */
#define JESD204_REG_VERSION        0x00
#define JESD204_REG_MAGIC          0x0c
#define JESD204_REG_SYNTH_NUM_LANES 0x10
#define JESD204_REG_SYNTH_DATA_PATH_WIDTH 0x14

/* MAGIC tags: '204R' for the RX core, '204T' for the TX core. */
#define JESD204_RX_MAGIC \
	(('2' << 24) | ('0' << 16) | ('4' << 8) | ('R'))
#define JESD204_TX_MAGIC \
	(('2' << 24) | ('0' << 16) | ('4' << 8) | ('T'))

/* PCORE version field extraction (major must be 1 for the current IP). */
#define PCORE_VERSION_MAJOR(x) ((x) >> 16)
#define PCORE_VERSION_MINOR(x) (((x) >> 8) & 0xff)
#define PCORE_VERSION_PATCH(x) ((x) & 0xff)

/* We expect L=4 lanes for the zcu102 m8-l4 profile. */
#define JESD204_EXPECT_NUM_LANES 4

static int axi_jesd_map(void)
{
	mm_reg_t virt;

	device_map(&virt, AXI_JESD_RX_BASE, AXI_JESD_SIZE, K_MEM_CACHE_NONE);
	if (virt != AXI_JESD_RX_BASE) {
		LOG_ERR("RX JESD not identity-mapped: virt=0x%lx phys=0x%lx",
			(unsigned long)virt, AXI_JESD_RX_BASE);
		return -EIO;
	}

	device_map(&virt, AXI_JESD_TX_BASE, AXI_JESD_SIZE, K_MEM_CACHE_NONE);
	if (virt != AXI_JESD_TX_BASE) {
		LOG_ERR("TX JESD not identity-mapped: virt=0x%lx phys=0x%lx",
			(unsigned long)virt, AXI_JESD_TX_BASE);
		return -EIO;
	}
	return 0;
}

SYS_INIT(axi_jesd_map, PRE_KERNEL_1, 0);

/*
 * Read and validate one JESD204 core's identity registers. Returns 0 if the
 * MAGIC matches (the AXI plane is alive and the bitstream is the expected IP),
 * negative errno otherwise.
 */
static int axi_jesd_check_core(const char *name, uintptr_t base, uint32_t magic)
{
	uint32_t rd_magic = sys_read32(base + JESD204_REG_MAGIC);
	uint32_t version = sys_read32(base + JESD204_REG_VERSION);
	uint32_t num_lanes = sys_read32(base + JESD204_REG_SYNTH_NUM_LANES);

	if (rd_magic != magic) {
		LOG_ERR("%s: bad MAGIC 0x%08x (expected 0x%08x) -- PL not "
			"configured or wrong bitstream?", name, rd_magic, magic);
		return -ENODEV;
	}

	LOG_INF("%s @ 0x%08lx: MAGIC ok, PCORE v%u.%u.%u, %u lanes",
		name, (unsigned long)base,
		PCORE_VERSION_MAJOR(version), PCORE_VERSION_MINOR(version),
		PCORE_VERSION_PATCH(version), num_lanes);

	if (PCORE_VERSION_MAJOR(version) != 1) {
		LOG_ERR("%s: unexpected PCORE major %u (expected 1)",
			name, PCORE_VERSION_MAJOR(version));
		return -ENOTSUP;
	}

	if (num_lanes != JESD204_EXPECT_NUM_LANES) {
		LOG_WRN("%s: SYNTH_NUM_LANES=%u (expected %u for m8-l4)",
			name, num_lanes, JESD204_EXPECT_NUM_LANES);
	}

	return 0;
}

int axi_jesd_probe(void)
{
	int ret;

	ret = axi_jesd_check_core("JESD204-RX", AXI_JESD_RX_BASE,
				  JESD204_RX_MAGIC);
	if (ret) {
		return ret;
	}

	ret = axi_jesd_check_core("JESD204-TX", AXI_JESD_TX_BASE,
				  JESD204_TX_MAGIC);
	if (ret) {
		return ret;
	}

	return 0;
}
