/*
 * Copyright (c) 2026 Analog Devices, Inc.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Vendor extension for the Analog Devices AXI SPI Engine.
 *
 * The SPI Engine can run in "offload" mode: a fixed command program is
 * loaded once into the core's command memory, and the hardware replays it
 * autonomously on every external trigger (e.g. a PWMGEN CNV pulse),
 * streaming the captured SDI data straight to an AXI DMAC at MS/s rates
 * with no per-sample CPU cost.
 *
 * Zephyr has no generic SPI-offload framework, so this path is exposed as
 * an ADI-specific extension on top of the standard SPI controller driver
 * (compatible "adi,axi-spi-engine"). Register-mode transfers still use the
 * normal SPI API (spi_transceive / spi_transceive_dt); only the offload
 * program setup lives here. The device handle passed to these functions is
 * the SPI controller device itself.
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_MISC_ADI_SPI_ENGINE_OFFLOAD_H_
#define ZEPHYR_INCLUDE_DRIVERS_MISC_ADI_SPI_ENGINE_OFFLOAD_H_

#include <zephyr/device.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * SPI Engine command-word builder macros for offload programs.
 * Bit layout: [14:12]=instruction, [10:8]=arg1, [7:0]=arg2.
 * arg1 MUST be 3-bit-masked to avoid SDO_LANE_CONFIG aliasing to CLK_DIV.
 */
#define SPI_ENGINE_CMD_BUILD(_inst, _arg1, _arg2) \
	((((_inst) & 0x07) << 12) | (((_arg1) & 0x07) << 8) | ((_arg2) & 0xFF))

#define SPI_ENGINE_CMD_BUILD_CONFIG_CLK_DIV(_div) \
	SPI_ENGINE_CMD_BUILD(0x02, 0x00, (_div))
#define SPI_ENGINE_CMD_BUILD_CONFIG_MODE(_mode) \
	SPI_ENGINE_CMD_BUILD(0x02, 0x01, (_mode))
#define SPI_ENGINE_CMD_BUILD_CONFIG_XFER_BITS(_bits) \
	SPI_ENGINE_CMD_BUILD(0x02, 0x02, (_bits))

#define SPI_ENGINE_CMD_BUILD_READ_N_WORDS(_n) \
	SPI_ENGINE_CMD_BUILD(0x00, 0x02, ((_n) - 1))
#define SPI_ENGINE_CMD_BUILD_WRITE_N_WORDS(_n) \
	SPI_ENGINE_CMD_BUILD(0x00, 0x01, ((_n) - 1))
#define SPI_ENGINE_CMD_BUILD_WRITE_READ_N_WORDS(_n) \
	SPI_ENGINE_CMD_BUILD(0x00, 0x03, ((_n) - 1))

#define SPI_ENGINE_CMD_BUILD_CS_LOW \
	SPI_ENGINE_CMD_BUILD(0x01, 0x03, 0x00)
#define SPI_ENGINE_CMD_BUILD_CS_HIGH \
	SPI_ENGINE_CMD_BUILD(0x01, 0x03, 0xFF)
#define SPI_ENGINE_CMD_BUILD_ASSERT(_cs_pattern) \
	SPI_ENGINE_CMD_BUILD(0x01, 0x03, (_cs_pattern))

#define SPI_ENGINE_CMD_BUILD_SYNC(_id) \
	SPI_ENGINE_CMD_BUILD(0x03, 0x00, (_id))
#define SPI_ENGINE_CMD_BUILD_SLEEP(_cycles) \
	SPI_ENGINE_CMD_BUILD(0x03, 0x01, (_cycles))

/**
 * @brief Offload program description.
 *
 * @param commands     Array of SPI Engine command words (built with the
 *                     SPI_ENGINE_CMD_BUILD_* macros) replayed on each trigger.
 * @param num_commands Number of entries in @p commands.
 * @param tx_data      Optional SDO words preloaded into offload SDO memory,
 *                     or NULL for read-only programs.
 * @param tx_len       Number of entries in @p tx_data.
 * @param rx_addr      Optional consumer-side (DMAC) capture address hint.
 * @param tx_addr      Optional producer-side address hint.
 */
struct spi_engine_offload_msg {
	uint32_t *commands;
	uint32_t num_commands;
	uint32_t *tx_data;
	uint32_t tx_len;
	uint32_t rx_addr;
	uint32_t tx_addr;
};

/**
 * @brief Load an offload command program into the SPI Engine.
 *
 * Resets the offload module and writes @p msg into the core's command
 * (and optional SDO) memory. Does not start execution; call
 * spi_engine_offload_enable() once the downstream DMAC is armed.
 *
 * @param dev SPI Engine controller device (compatible "adi,axi-spi-engine").
 * @param msg Offload program to load.
 *
 * @return 0 on success, negative errno otherwise.
 */
int spi_engine_offload_load(const struct device *dev,
			    struct spi_engine_offload_msg *msg);

/**
 * @brief Enable or disable autonomous offload execution.
 *
 * When enabled, the loaded program is replayed on every external trigger.
 * Must be enabled before the first trigger arrives, otherwise the engine
 * and the ADC drift out of frame.
 *
 * @param dev    SPI Engine controller device.
 * @param enable true to start offload replay, false to stop.
 *
 * @return 0 on success, negative errno otherwise.
 */
int spi_engine_offload_enable(const struct device *dev, bool enable);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_MISC_ADI_SPI_ENGINE_OFFLOAD_H_ */
