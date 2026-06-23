/*
 * Copyright (c) 2026 Analog Devices, Inc.
 * SPDX-License-Identifier: Apache-2.0
 *
 * L2C-310 (PL310) outer-cache support for Zynq-7000.
 *
 * The controller is brought up early (from soc_reset_hook, MMU + L1 off) and
 * its line-maintenance ops are paired with the Cortex-A9 L1 ops by the arch
 * cache layer (see arch/arm/core/cortex_a_r/cache.c) so that the standard
 * sys_cache_* API keeps both cache levels coherent for DMA.
 */

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/barrier.h>

#include "pl310.h"

/* PL310 base on Zynq-7000 (TRM Appendix B.36) */
#define PL310_BASE              0xF8F02000U
#define PL310_LINE_SIZE         32U

/* Register offsets */
#define PL310_CACHE_ID          0x000
#define PL310_CACHE_TYPE        0x004
#define PL310_CTRL              0x100
#define PL310_AUX_CTRL          0x104
#define PL310_TAG_RAM_CTRL      0x108
#define PL310_DATA_RAM_CTRL     0x10C
#define PL310_INT_CLEAR         0x220
#define PL310_CACHE_SYNC        0x730
#define PL310_INV_LINE_PA       0x770
#define PL310_INV_WAY           0x77C
#define PL310_CLEAN_LINE_PA     0x7B0
#define PL310_CLEAN_WAY         0x7BC
#define PL310_CLEAN_INV_LINE_PA 0x7F0
#define PL310_CLEAN_INV_WAY     0x7FC

/* Zynq-7000 wires up 8 ways, 64 KB per way -> 512 KB total */
#define PL310_NUM_WAYS          8U
#define PL310_WAY_MASK          ((1U << PL310_NUM_WAYS) - 1U)

/*
 * Tag/Data RAM latencies. Xilinx default values for the Zynq-7000 PS PL310.
 */
#define PL310_TAG_RAM_LATENCY   0x00000111U
#define PL310_DATA_RAM_LATENCY  0x00000121U

/*
 * AUX_CTRL handling, following the Linux ARM Zynq mach code exactly:
 *
 *   arch/arm/mach-zynq/common.c:
 *     .l2c_aux_val  = 0x00400000,   (force bit 22)
 *     .l2c_aux_mask = 0xffbfffff,   (preserve every bit except 22)
 *   applied as:  aux = (aux & l2c_aux_mask) | l2c_aux_val;
 *
 * The L2C-310 AUX_CTRL carries hardware-strapped fields describing the
 * physical cache RAMs -- associativity [16] and way-size [19:17]. On the
 * Zynq-7000 these come up correct out of reset (8-way x 64 KB, reset AUX =
 * 0x02060000), so the rule is: PRESERVE the reset value, change only what we
 * must.
 *
 * Bit [22] = "Shared Attribute Override Enable". Zephyr's MMU maps normal
 * image memory (code/data/rodata) as cacheable AND shareable (MATTR_SHARED in
 * arch/arm/core/mmu/arm_mmu.c). Without bit 22 the PL310 treats shared
 * Normal-cacheable transactions as non-cacheable on the Zynq interconnect;
 * the first such access after the MMU+L1+L2 come on (the page-table walk /
 * instruction fetch) issues an AXI read the L2 never completes -> SILENT HANG
 * exactly at MMU enable. Forcing bit 22 (as Linux does) fixes that.
 *
 * KEEP_MASK clears ONLY bit 22 before we OR FORCE_VAL back in, so every other
 * reset bit (incl. the strap fields and the reset replacement-policy bit) is
 * preserved verbatim. Behavioral bits (prefetch/early-BRESP) stay at their
 * reset state for now; they can be enabled later for throughput.
 */
#define PL310_AUX_KEEP_MASK     0xFFBFFFFFU
#define PL310_AUX_FORCE_VAL     0x00400000U

static inline void pl310_sync(void)
{
	sys_write32(0, PL310_BASE + PL310_CACHE_SYNC);
}

/*
 * Called from soc_reset_hook with the MMU off and L1 cache off (Zephyr
 * disables SCTLR.[I,C] in soc_reset_hook). The L2C-310 register bank lives in
 * PS device space (0xF8F02000), reachable without MMU translation.
 */
void soc_zynq7000_pl310_early_enable(void)
{
	/* Make sure the controller is off so we can program latencies / AUX. */
	sys_write32(0, PL310_BASE + PL310_CTRL);

	/* Tag/data RAM latencies (Xilinx default for Zynq-7000). */
	sys_write32(PL310_TAG_RAM_LATENCY,  PL310_BASE + PL310_TAG_RAM_CTRL);
	sys_write32(PL310_DATA_RAM_LATENCY, PL310_BASE + PL310_DATA_RAM_CTRL);

	/*
	 * Preserve the reset AUX value, force only bit 22 (shared-attribute
	 * override) -- mirrors Linux: aux = (aux & mask) | val.
	 */
	{
		uint32_t aux = sys_read32(PL310_BASE + PL310_AUX_CTRL);

		aux &= PL310_AUX_KEEP_MASK;
		aux |= PL310_AUX_FORCE_VAL;
		sys_write32(aux, PL310_BASE + PL310_AUX_CTRL);
	}

	/* Invalidate all ways before the first enable (random reset state). */
	sys_write32(PL310_WAY_MASK, PL310_BASE + PL310_INV_WAY);
	while (sys_read32(PL310_BASE + PL310_INV_WAY) & PL310_WAY_MASK) {
	}
	pl310_sync();

	/* Clear any pending interrupts, then enable. */
	sys_write32(0x1FFU, PL310_BASE + PL310_INT_CLEAR);
	sys_write32(1U, PL310_BASE + PL310_CTRL);
	pl310_sync();

	barrier_dsync_fence_full();
	barrier_isync_fence_full();
}

/*
 * Outer-cache range maintenance ops. These provide the strong definitions of
 * the __weak z_arm_outer_cache_* hooks declared by the Cortex-A/R arch cache
 * layer (arch/arm/core/cortex_a_r/cache.c), so that the generic sys_cache_*
 * API keeps the L1 and PL310 (outer) levels coherent for non-snooping DMA.
 *
 * Loop over PL310_LINE_SIZE-aligned addresses; the register write is the
 * per-line op, the trailing CACHE_SYNC makes it visible system-wide. The PA
 * registers take physical addresses; the Zynq-7000 MMU is flat-mapped so
 * VA == PA for the DMA buffers these are used on.
 */
static ALWAYS_INLINE void pl310_range(uintptr_t reg, void *addr, size_t size)
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
