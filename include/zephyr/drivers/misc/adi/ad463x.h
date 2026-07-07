/*
 * Copyright (c) 2026 Analog Devices, Inc.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Public interface for the Analog Devices AD463x precision SAR ADC driver.
 *
 * This is a specialized AXI offload driver. The data path is:
 *
 *   PWMGEN (CNV trigger) → SPI Engine (offload program) → DMAC → DDR
 *
 * All sample acquisition happens in hardware with no per-sample CPU cost.
 * The primary API is ad463x_read_buffer(): arm one DMA transfer, block
 * until N samples arrive, return the raw binary buffer.  Call it
 * back-to-back for continuous acquisition.
 *
 * The driver also registers a Zephyr ADC API shim (adc_read / adc_channel_setup)
 * for compatibility with generic Zephyr code.  The shim runs the DMAC in
 * cyclic mode and snapshots the latest ring slot on each adc_read() call.
 * It is suitable for slow polling (DC levels, occasional reads) but discards
 * the vast majority of samples at high throughput — use ad463x_read_buffer()
 * for any application where sample integrity matters.
 *
 * Based on the no-OS reference driver and the Linux IIO driver ad4630.c.
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_MISC_ADI_AD463X_H_
#define ZEPHYR_INCLUDE_DRIVERS_MISC_ADI_AD463X_H_

#include <zephyr/device.h>
#include <stdint.h>
#include <stddef.h>
#include <sys/types.h>

/* Register addresses */
#define AD463X_REG_INTERFACE_CONFIG_A	0x00
#define AD463X_REG_INTERFACE_CONFIG_B	0x01
#define AD463X_REG_DEVICE_CONFIG	0x02
#define AD463X_REG_CHIP_TYPE		0x03
#define AD463X_REG_PRODUCT_ID_L		0x04
#define AD463X_REG_PRODUCT_ID_H		0x05
#define AD463X_REG_CHIP_GRADE		0x06
#define AD463X_REG_SCRATCH_PAD		0x0A
#define AD463X_REG_SPI_REVISION		0x0B
#define AD463X_REG_VENDOR_L		0x0C
#define AD463X_REG_VENDOR_H		0x0D
#define AD463X_REG_STREAM_MODE		0x0E
#define AD463X_REG_EXIT_CFG_MODE	0x14
#define AD463X_REG_AVG			0x15
#define AD463X_REG_OFFSET_BASE		0x16
#define AD463X_REG_GAIN_BASE		0x1C
#define AD463X_REG_MODES		0x20
#define AD463X_REG_OSCILATOR		0x21
#define AD463X_REG_IO			0x22
#define AD463X_REG_PAT0			0x23
#define AD463X_REG_PAT1			0x24
#define AD463X_REG_PAT2			0x25
#define AD463X_REG_PAT3			0x26
#define AD463X_REG_DIG_DIAG		0x34
#define AD463X_REG_DIG_ERR		0x35

#define AD463X_REG_CHAN_OFFSET(ch, pos)	(AD463X_REG_OFFSET_BASE + (3 * (ch)) + (pos))
#define AD463X_REG_CHAN_GAIN(ch, pos)	(AD463X_REG_GAIN_BASE + (2 * (ch)) + (pos))

/* INTERFACE_CONFIG_A */
#define AD463X_CFG_SW_RESET		((1 << 7) | (1 << 0))
#define AD463X_CFG_SDO_ENABLE		(1 << 4)

/* MODES bit fields */
#define AD463X_LANE_MODE_MSK		((1 << 7) | (1 << 6))
#define AD463X_CLK_MODE_MSK		((1 << 5) | (1 << 4))
#define AD463X_DDR_MODE_MSK		(1 << 3)
#define AD463X_OUT_DATA_MODE_MSK	((1 << 2) | (1 << 1) | (1 << 0))

/* MODES values */
#define AD463X_SDR_MODE			0x00
#define AD463X_DDR_MODE			(1 << 3)

#define AD463X_24_DIFF			0x00
#define AD463X_16_DIFF_8_COM		0x01
#define AD463X_24_DIFF_8_COM		0x02
#define AD463X_30_AVERAGED_DIFF		0x03
#define AD463X_32_PATTERN		0x04

#define AD463X_ONE_LANE_PER_CH		0x00
#define AD463X_TWO_LANES_PER_CH		(1 << 6)
#define AD463X_FOUR_LANES_PER_CH	(1 << 7)
#define AD463X_SHARED_TWO_CH		((1 << 6) | (1 << 7))

#define AD463X_SPI_COMPATIBLE_MODE	0x00
#define AD463X_ECHO_CLOCK_MODE		(1 << 4)
#define AD463X_CLOCK_MASTER_MODE	(1 << 5)

/* EXIT_CFG_MD */
#define AD463X_EXIT_CFG_MODE		(1 << 0)

/* DEVICE_CONFIG power modes */
#define AD463X_NORMAL_MODE		0x00
#define AD463X_LOW_POWER_MODE		((1 << 1) | (1 << 0))

/* AVG */
#define AD463X_AVG_FILTER_RESET		(1 << 7)

/* IO drive strength */
#define AD463X_DRIVER_STRENGTH_MASK	(1 << 0)
#define AD463X_NORMAL_OUTPUT_STRENGTH	0x00
#define AD463X_DOUBLE_OUTPUT_STRENGTH	(1 << 1)

/* SPI framing */
#define AD463X_REG_READ			(1 << 7)
#define AD463X_REG_WRITE		0x00
#define AD463X_REG_READ_DUMMY		0x00

/* Datasheet-specified CNV pulse width */
#define AD463X_TRIGGER_PULSE_WIDTH_NS	10

/* Magic value used by the scratchpad ID test */
#define AD463X_SCRATCH_TEST		0xAA

/**
 * AD463x part variants supported by this driver. Maps to the adi,part
 * string in the binding.
 */
enum ad463x_id {
	AD463X_ID_AD4630_24 = 0,
	AD463X_ID_AD4630_20,
	AD463X_ID_AD4630_16,
	AD463X_ID_AD4631_24,
	AD463X_ID_AD4631_20,
	AD463X_ID_AD4631_16,
	AD463X_ID_AD4632_24,
	AD463X_ID_AD4632_20,
	AD463X_ID_AD4632_16,
	AD463X_ID_COUNT,
};

/**
 * Primary acquisition entry point.
 *
 * Arms a single AXI DMAC transfer and blocks until @p len bytes of raw
 * conversion data have been written to @p buf by the hardware offload path.
 * No sample is touched by the CPU during capture. Call back-to-back for
 * continuous acquisition.
 *
 * Each frame in the buffer is @ref ad463x_get_frame_size() bytes and contains
 * both channels interleaved: [CH0 uint32][CH1 uint32]. The HDL left-aligns
 * the real bits in each uint32; sign-extend with (int32_t)word >> (32 - real_bits).
 *
 * Cannot be called while the Zephyr ADC API shim has armed the cyclic ring
 * (i.e. after the first adc_read() call). The two paths are mutually exclusive.
 *
 * @param dev  AD463x device.
 * @param buf  Destination buffer. Must be 32-byte aligned and __nocache if
 *             the platform has an L2 cache that the DMAC bypasses.
 * @param len  Capacity of @p buf in bytes.
 *
 * @return Number of bytes written on success, or a negative errno.
 */
ssize_t ad463x_read_buffer(const struct device *dev, void *buf, size_t len);

/**
 * Returns the raw bytes the HDL produces per CNV pulse (both channels combined).
 * Use this to compute sample counts from buffer sizes rather than hardcoding
 * the frame width, which varies with output mode and lane configuration.
 */
size_t ad463x_get_frame_size(const struct device *dev);

/**
 * Bring the chip out of reset, program the AXI CLKGEN, verify the chip is
 * alive via a scratchpad round-trip, and apply the capture configuration from
 * devicetree. Called automatically on the first capture; exposed here for
 * applications that need explicit control over reset and CNV timing.
 */
int ad463x_init_chip(const struct device *dev);

#endif /* ZEPHYR_INCLUDE_DRIVERS_MISC_ADI_AD463X_H_ */
