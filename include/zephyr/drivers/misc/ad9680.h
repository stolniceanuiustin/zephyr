/*
 * Copyright (c) 2026 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief AD9680 RX ADC public interface.
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_MISC_AD9680_H_
#define ZEPHYR_INCLUDE_DRIVERS_MISC_AD9680_H_

#include <stdbool.h>

#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief AD9680 interface
 * @defgroup ad9680_interface AD9680 RX ADC interface
 * @ingroup io_interfaces
 * @{
 */

/**
 * @brief Re-read the JESD204B (framer serializer) PLL lock status.
 *
 * The AD9680's framer PLL is derived from the sample clock and config and is
 * expected to lock without the FPGA RX path, but init() reads it once at setup,
 * before the RX transceiver and link core are armed. This entry point lets the
 * bring-up sequence re-read it after the FPGA side is up.
 *
 * @param dev    AD9680 device.
 * @param locked Out: true if the JESD204B PLL reports locked.
 * @retval 0 on success.
 * @retval -EINVAL if @p dev or @p locked is NULL.
 * @retval -ENODEV if @p dev is not ready.
 * @retval -errno on an SPI read failure.
 */
int ad9680_jesd_pll_locked(const struct device *dev, bool *locked);

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_MISC_AD9680_H_ */
