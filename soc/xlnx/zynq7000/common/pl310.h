/*
 * Copyright (c) 2026 Analog Devices, Inc.
 * SPDX-License-Identifier: Apache-2.0
 *
 * L2C-310 (PL310) outer-cache support for Xilinx Zynq-7000.
 *
 * The PL310 sits between the Cortex-A9 L1 D-cache and DDR. For DMA correctness
 * on devices that do not snoop (e.g. GEM, AXI-DMA), every L1 maintenance op
 * must be paired with the matching L2 op. That pairing is done generically by
 * the Cortex-A/R arch cache layer via the sys_cache_outer_* hooks; pl310.c
 * provides the strong definitions of those hooks (declared in <zephyr/cache.h>).
 *
 * This header only exposes the early bring-up entry point shared between
 * pl310.c and soc.c.
 */

#ifndef ZEPHYR_SOC_XLNX_ZYNQ7000_PL310_H_
#define ZEPHYR_SOC_XLNX_ZYNQ7000_PL310_H_

/*
 * Bring up the L2C-310 with the MMU and L1 still off. Called from
 * soc_reset_hook(); see pl310.c for the full sequence.
 */
void soc_zynq7000_pl310_early_enable(void);

#endif /* ZEPHYR_SOC_XLNX_ZYNQ7000_PL310_H_ */
