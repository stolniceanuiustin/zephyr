/*
 * Copyright (c) 2026 Analog Devices, Inc.
 *
 * AXI AD9081 TPL (transport-layer) cores -- ADC framer / DAC deframer datapath.
 *
 * One devicetree node per direction (adi,axi-ad9081-rx-1.0 /
 * adi,axi-ad9081-tx-1.0), so every entry point takes the device it acts on.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_MISC_JESD204_AXI_TPL_H_
#define ZEPHYR_INCLUDE_DRIVERS_MISC_JESD204_AXI_TPL_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/device.h>

/*
 * Configure one transport core: pulse core reset, arm the per-converter
 * datapath (RX sample format/enable, TX data-source select) and, on TX, latch
 * the configuration. Which of the two it does comes from the node's compatible.
 *
 * Configure-only -- the STATUS/clock verification, which only reads valid once
 * the JESD link and sample clocks are live, is deferred to axi_tpl_enable().
 * Returns 0 on success, negative errno otherwise.
 */
int axi_tpl_configure(const struct device *dev);

/*
 * Finish TPL bring-up once the link + lane clocks are running: re-issue the DAC
 * datapath SYNC pulse and verify both cores' STATUS. Driven by the JESD204
 * bring-up sequence -- not called standalone. Takes both directions because the
 * SYNC pulse and the two status reads are one ordered step.
 */
int axi_tpl_enable(const struct device *rx, const struct device *tx);

/*
 * Drive the TX transport core's converters from their internal FPGA DDS tone
 * generators (enable=true) or from the DMA stream (enable=false). dev must be an
 * adi,axi-ad9081-tx-1.0 node.
 *
 * This is the no-OS example's actual output path: its axi_dac_init() writes
 * DATA_SELECT=0 (DDS) to every converter (axi_dac_core.c:1235-1236), so the DMA
 * and data-offload cores never enter the datapath and the tone costs no DDR
 * bandwidth at all. The DDS sits at the TPL input, so its samples still cross the
 * transport core, the serial lanes, the chip's deframer and the DAC datapath.
 *
 * scale_micro is micro-units of full scale (1000000 == 1.0), as in no-OS
 * axi_dac_dds_set_scale(); values at or above 1999000 are clamped. Both DDSs of a
 * converter get the same frequency and phase, with the phase alternating by
 * converter index, matching the no-OS default.
 *
 * freq_hz is quantised to freq * 0xFFFF / sample_rate_hz by the 16-bit phase
 * accumulator, so the achieved frequency only equals the request when it divides
 * the sample rate evenly. Returns 0, or -EINVAL if the tone is below the DDS
 * resolution at that sample rate.
 */
int axi_tpl_tx_dds(const struct device *dev, uint32_t freq_hz,
		   uint32_t sample_rate_hz, uint32_t scale_micro, bool enable);

#endif /* ZEPHYR_INCLUDE_DRIVERS_MISC_JESD204_AXI_TPL_H_ */
