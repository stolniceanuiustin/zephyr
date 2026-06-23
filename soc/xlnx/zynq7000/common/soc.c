/*
 * Copyright (c) 2021 Weidmueller Interface GmbH & Co. KG
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/arch/cpu.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>

#include <cmsis_core.h>
#include <zephyr/arch/arm/mmu/arm_mmu.h>
#include "soc.h"

#ifdef CONFIG_SOC_XILINX_PL310
#include "pl310.h"
#endif

/* System Level Control Registers (SLCR) */
#define SLCR_UNLOCK     0x0008
#define SLCR_UNLOCK_KEY 0xdf0d

#ifdef CONFIG_SOC_XILINX_PL310
/*
 * L2C RAM read/write latency control (Xilinx design advisory AR#54190).
 * Must be programmed before the PL310 is enabled. Mirrors the Linux Zynq
 * mach code: regmap_update_bits(SLCR_L2C_RAM, 0x70707, 0x20202).
 */
#define SLCR_L2C_RAM      0x0A1C
#define SLCR_L2C_RAM_MASK 0x00070707
#define SLCR_L2C_RAM_VAL  0x00020202
#endif

#define AXI_GPIO_MMU_ENTRY(id)\
	MMU_REGION_FLAT_ENTRY("axigpio",\
			      DT_REG_ADDR(id),\
			      DT_REG_SIZE(id),\
			      MT_DEVICE | MATTR_SHARED | MPERM_R | MPERM_W),

static const struct arm_mmu_region mmu_regions[] = {

	MMU_REGION_FLAT_ENTRY("vectors",
			      0x00000000,
			      0x1000,
			      MT_STRONGLY_ORDERED | MPERM_R | MPERM_X),
	MMU_REGION_FLAT_ENTRY("mpcore",
			      0xF8F00000,
			      0x2000,
			      MT_STRONGLY_ORDERED | MPERM_R | MPERM_W),
	MMU_REGION_FLAT_ENTRY("ocm",
			      DT_REG_ADDR(DT_CHOSEN(zephyr_ocm)),
			      DT_REG_SIZE(DT_CHOSEN(zephyr_ocm)),
			      MT_STRONGLY_ORDERED | MPERM_R | MPERM_W),
	/* ARM Arch timer, GIC are covered by the MPCore mapping */

/* SLCR - needed by GEM driver for clock configuration */
#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(slcr))
	MMU_REGION_FLAT_ENTRY("slcr",
			      DT_REG_ADDR(DT_NODELABEL(slcr)),
			      DT_REG_SIZE(DT_NODELABEL(slcr)),
			      MT_DEVICE | MATTR_SHARED | MPERM_R | MPERM_W),
#endif

#ifdef CONFIG_SOC_XILINX_PL310
	/* PL310 (L2C-310) outer-cache controller registers */
	MMU_REGION_FLAT_ENTRY("pl310",
			      0xF8F02000,
			      0x1000,
			      MT_DEVICE | MATTR_SHARED | MPERM_R | MPERM_W),
#endif

/* GEMs */
#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(gem0))
	MMU_REGION_FLAT_ENTRY("gem0",
			      DT_REG_ADDR(DT_NODELABEL(gem0)),
			      DT_REG_SIZE(DT_NODELABEL(gem0)),
			      MT_DEVICE | MATTR_SHARED | MPERM_R | MPERM_W),
#endif
#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(gem1))
	MMU_REGION_FLAT_ENTRY("gem1",
			      DT_REG_ADDR(DT_NODELABEL(gem1)),
			      DT_REG_SIZE(DT_NODELABEL(gem1)),
			      MT_DEVICE | MATTR_SHARED | MPERM_R | MPERM_W),
#endif

DT_FOREACH_STATUS_OKAY(xlnx_xps_gpio_1_00_a, AXI_GPIO_MMU_ENTRY)

};

const struct arm_mmu_config mmu_config = {
	.num_regions = ARRAY_SIZE(mmu_regions),
	.mmu_regions = mmu_regions,
};

/* Platform-specific early initialization */

void soc_reset_hook(void)
{
	/*
	 * When coming out of u-boot rather than downloading the Zephyr binary
	 * via JTAG, a few things modified by u-boot have to be re-set to a
	 * suitable default value for Zephyr to run, namely:
	 *
	 * - u-boot places the exception vectors somewhere in RAM and then
	 *   lets the VBAR register point to them. Zephyr uses the default
	 *   vector table location at address zero (and maybe at some later
	 *   time alternatively the HIVECS position). If VBAR isn't reset
	 *   to zero, the system crashes during the first context switch when
	 *   SVC is invoked.
	 * - u-boot sets the following bits in the SCTLR register:
	 *   - [I] ICache enable
	 *   - [C] DCache enable
	 *   - [Z] Branch prediction enable
	 *   - [A] Enforce strict alignment enable
	 *   [I] and [C] will be enabled during the MMU init -> disable them
	 *   until then. [Z] is probably not harmful. [A] will cause a crash
	 *   as early as z_mem_manage_init when an unaligned access is performed
	 *   -> clear [A].
	 */

	uint32_t vbar = 0;

	__set_VBAR(vbar);

	uint32_t sctlr = __get_SCTLR();

	sctlr &= ~SCTLR_I_Msk;
	sctlr &= ~SCTLR_C_Msk;
	sctlr &= ~SCTLR_A_Msk;
	__set_SCTLR(sctlr);

	/*
	 * Cortex-A9: enable SMP / coherency participation (ACTLR.SMP, bit 6).
	 *
	 * On the A9 the load/store-exclusive monitor (LDREX/STREX) for Normal
	 * Shareable Cacheable memory -- which is how Zephyr maps its data
	 * (MATTR_SHARED) -- and the SCU cache coherency logic only function
	 * when ACTLR.SMP == 1. With SMP == 0 a STREX to such memory never
	 * succeeds, so the very first atomic op (atomic_inc in the logging
	 * init) spins forever. This is masked when the L2/outer cache is off
	 * (shareable WB then behaves as non-cacheable and exclusives resolve
	 * trivially), which is why the board only hangs once the PL310 is
	 * enabled. Set it here, with caches still off, before anything atomic
	 * runs. Harmless on a single-core boot.
	 */
	{
		uint32_t actlr = __get_ACTLR();

		actlr |= BIT(6);
		__set_ACTLR(actlr);
	}

#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(slcr))
	mm_reg_t addr = DT_REG_ADDR(DT_NODELABEL(slcr));

	/* Unlock System Level Control Registers (SLCR) */
	sys_write32(SLCR_UNLOCK_KEY, addr + SLCR_UNLOCK);

#ifdef CONFIG_SOC_XILINX_PL310
	/*
	 * Apply the L2C RAM latency advisory (AR#54190) while SLCR is unlocked
	 * and the PL310 is still off. Read-modify-write so only the latency
	 * fields are touched.
	 */
	{
		uint32_t l2c_ram = sys_read32(addr + SLCR_L2C_RAM);

		l2c_ram &= ~SLCR_L2C_RAM_MASK;
		l2c_ram |= SLCR_L2C_RAM_VAL;
		sys_write32(l2c_ram, addr + SLCR_L2C_RAM);
	}
#endif
#endif

#ifdef CONFIG_SOC_XILINX_PL310
	/*
	 * Bring up the PL310 with the MMU and L1 still off. This is the
	 * sequence the L2C-310 docs prescribe: tag/data RAM latencies and
	 * AUX_CTRL must be programmed before the controller is enabled,
	 * and the controller must be enabled before any cacheable memory
	 * is touched (otherwise L1 fills bypass L2 and we get coherence
	 * surprises later when L2 turns on with stale ways).
	 */
	soc_zynq7000_pl310_early_enable();
#endif
}
