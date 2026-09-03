/*
 * Copyright (c) 2026 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief AXI AD9081 TPL cores -- ADC framer / DAC deframer datapath.
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_MISC_JESD204_AXI_TPL_H_
#define ZEPHYR_INCLUDE_DRIVERS_MISC_JESD204_AXI_TPL_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief AXI AD9081 transport-layer interface
 * @defgroup axi_tpl_interface AXI AD9081 transport-layer interface
 * @ingroup io_interfaces
 *
 * One devicetree node per direction (adi,axi-ad9081-rx-1.0 /
 * adi,axi-ad9081-tx-1.0), so every entry point takes the device it acts on.
 *
 * @{
 */

/**
 * @brief Configure one transport core.
 *
 * Pulses the core reset, arms the per-converter datapath (RX sample
 * format/enable, TX data-source select) and, on TX, latches the configuration.
 * Which of the two it does comes from the node's compatible.
 *
 * Configure-only: the STATUS/clock verification, which only reads valid once the
 * JESD link and sample clocks are live, is deferred to axi_tpl_enable().
 *
 * @param dev Transport-core device.
 * @retval 0 on success.
 * @retval -ENODEV if @p dev is not ready.
 * @retval -ENOTSUP on an unexpected PCORE version.
 */
int axi_tpl_configure(const struct device *dev);

/**
 * @brief Finish TPL bring-up once the link and lane clocks are running.
 *
 * Re-issues the DAC datapath SYNC pulse and verifies both cores' STATUS. Driven
 * by the JESD204 bring-up sequence -- not called standalone. Takes both
 * directions because the SYNC pulse and the two status reads are one ordered
 * step.
 *
 * @param rx RX (ADC) transport core.
 * @param tx TX (DAC) transport core.
 * @retval 0 if both report a ready datapath.
 * @retval -ENODEV if either device is not ready.
 * @retval -EIO if either datapath status is not ready.
 */
int axi_tpl_enable(const struct device *rx, const struct device *tx);

/**
 * @brief Drive the TX converters from the internal FPGA DDS tone generators.
 *
 * With @p enable false the converters go back to the DMA stream instead.
 *
 * The DDS sits at the TPL input, so its samples still cross the transport core,
 * the serial lanes, the chip's deframer and the DAC datapath -- everything except
 * the DMA engine and DDR, which the tone does not touch at all.
 *
 * Both DDSs of a converter get the same frequency and phase, with the phase
 * alternating by converter index.
 *
 * @param dev            TX transport core (adi,axi-ad9081-tx-1.0).
 * @param freq_hz        Tone frequency. Quantised to
 *                       freq_hz * 0xFFFF / @p sample_rate_hz by the 16-bit phase
 *                       accumulator, so the achieved frequency only equals the
 *                       request when it divides the sample rate evenly.
 * @param sample_rate_hz Converter sample rate.
 * @param scale_micro    Amplitude in micro-units of full scale (1000000 == 1.0).
 *                       Values at or above 1999000 are clamped.
 * @param enable         true for DDS, false to restore the DMA source.
 * @retval 0 on success.
 * @retval -ENODEV if @p dev is not ready.
 * @retval -EINVAL if @p sample_rate_hz is zero, or the tone is below the DDS
 *         resolution at that sample rate.
 */
int axi_tpl_tx_dds(const struct device *dev, uint32_t freq_hz, uint32_t sample_rate_hz,
		   uint32_t scale_micro, bool enable);

/**
 * @brief Enable or disable one RX converter's datapath channel.
 *
 * axi_tpl_configure() arms every converter, which is what the DDS/DMA bring-up
 * path wants. A buffered capture client instead selects a subset, and a disabled
 * channel contributes no samples to the DMA stream -- so the transfer size and the
 * sample layout follow which channels are enabled here.
 *
 * Read-modify-write: the sample format bits the RX core was configured with are
 * preserved, only the enable bit changes.
 *
 * RX only. The TX core's per-converter register at the same offset is the DDS
 * scale, not a channel enable; use axi_tpl_tx_dds() to switch the TX source.
 *
 * @param dev    RX transport core (adi,axi-ad9081-rx-1.0).
 * @param chan   Converter index, less than the node's adi,num-channels.
 * @param enable true to include this converter in the capture stream.
 * @retval 0 on success.
 * @retval -ENODEV if @p dev is not ready.
 * @retval -ENOTSUP if @p dev is a TX core.
 * @retval -EINVAL if @p chan is out of range.
 */
int axi_tpl_rx_chan_enable(const struct device *dev, uint32_t chan, bool enable);

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_MISC_JESD204_AXI_TPL_H_ */
