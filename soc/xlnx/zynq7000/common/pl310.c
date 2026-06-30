/*
 * Copyright (c) 2026 Analog Devices, Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/arch/cpu.h>
#include <zephyr/sys/barrier.h>

#include "pl310.h"

/* PL310 base on Zynq-7000 (TRM Appendix B.36) */
#define PL310_BASE              0xF8F02000U
#define PL310_LINE_SIZE         32U

/* Register offsets */
#define PL310_CTRL              0x100
#define PL310_AUX_CTRL          0x104
#define PL310_TAG_RAM_CTRL      0x108
#define PL310_DATA_RAM_CTRL     0x10C
#define PL310_INT_CLEAR         0x220
#define PL310_CACHE_SYNC        0x730
#define PL310_INV_LINE_PA       0x770
#define PL310_INV_WAY           0x77C
#define PL310_CLEAN_LINE_PA     0x7B0
#define PL310_CLEAN_INV_LINE_PA 0x7F0
#define PL310_CLEAN_INV_WAY     0x7FC

/* Zynq-7000: 8 ways x 64 KB = 512 KB */
#define PL310_NUM_WAYS          8U
#define PL310_WAY_MASK          ((1U << PL310_NUM_WAYS) - 1U)

/* Tag/data RAM latencies (Xilinx defaults for Zynq-7000) */
#define PL310_TAG_RAM_LATENCY   0x00000111U
#define PL310_DATA_RAM_LATENCY  0x00000121U

/*
 * AUX_CTRL: preserve the reset value (which has correct way-size and
 * associativity straps), force only bit 22 (Shared Attribute Override).
 * Without bit 22, the PL310 treats Normal Shareable Cacheable transactions
 * as non-cacheable, causing a hang at MMU enable. Mirrors Linux mach-zynq:
 * l2c_aux_val=0x00400000, l2c_aux_mask=0xffbfffff.
 */
#define PL310_AUX_KEEP_MASK     0xFFBFFFFFU
#define PL310_AUX_FORCE_VAL     0x00400000U

static inline void pl310_sync(void)
{
	sys_write32(0, PL310_BASE + PL310_CACHE_SYNC);
}

void soc_zynq7000_pl310_early_enable(void)
{
	uint32_t aux;

	sys_write32(0, PL310_BASE + PL310_CTRL);

	sys_write32(PL310_TAG_RAM_LATENCY,  PL310_BASE + PL310_TAG_RAM_CTRL);
	sys_write32(PL310_DATA_RAM_LATENCY, PL310_BASE + PL310_DATA_RAM_CTRL);

	aux = sys_read32(PL310_BASE + PL310_AUX_CTRL);
	aux &= PL310_AUX_KEEP_MASK;
	aux |= PL310_AUX_FORCE_VAL;
	sys_write32(aux, PL310_BASE + PL310_AUX_CTRL);

	sys_write32(PL310_WAY_MASK, PL310_BASE + PL310_INV_WAY);
	while (sys_read32(PL310_BASE + PL310_INV_WAY) & PL310_WAY_MASK) {
	}
	pl310_sync();

	sys_write32(0x1FFU, PL310_BASE + PL310_INT_CLEAR);
	sys_write32(1U, PL310_BASE + PL310_CTRL);
	pl310_sync();

	barrier_dsync_fence_full();
	barrier_isync_fence_full();
}

static void pl310_range(uintptr_t reg, void *addr, size_t size)
{
	uintptr_t a = (uintptr_t)addr & ~(PL310_LINE_SIZE - 1U);
	uintptr_t end = (uintptr_t)addr + size;

	for (; a < end; a += PL310_LINE_SIZE) {
		sys_write32((uint32_t)a, PL310_BASE + reg);
	}
	pl310_sync();
}

void z_arm_outer_cache_flush_range(void *addr, size_t size)
{
	pl310_range(PL310_CLEAN_LINE_PA, addr, size);
}

void z_arm_outer_cache_invd_range(void *addr, size_t size)
{
	pl310_range(PL310_INV_LINE_PA, addr, size);
}

void z_arm_outer_cache_flush_and_invd_range(void *addr, size_t size)
{
	pl310_range(PL310_CLEAN_INV_LINE_PA, addr, size);
}
