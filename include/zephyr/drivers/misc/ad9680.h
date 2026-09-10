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
#include <zephyr/jesd204/jesd204_fsm.h>

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
 * The framer PLL derives from the sample clock, so init() already reads its
 * lock once at setup. This entry point re-reads it later (e.g. after the FPGA
 * RX path is up) without re-running setup.
 *
 * @param dev    AD9680 device.
 * @param locked Out: true if the JESD204B PLL reports locked.
 * @retval 0 on success.
 * @retval -EINVAL if @p dev or @p locked is NULL.
 * @retval -ENODEV if @p dev is not ready.
 * @retval -errno on an SPI read failure.
 */
int ad9680_jesd_pll_locked(const struct device *dev, bool *locked);

/**
 * @brief JESD204 phase table for the AD9680 (RX topology top device).
 *
 * Registers a JESD204B PLL re-read as this converter's CLOCKS_ENABLE work. A
 * board points its ad9680 jesd204_dev at this table with the Zephyr device in
 * `priv`.
 */
extern const struct jesd204_dev_data ad9680_jesd204_data;

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_MISC_AD9680_H_ */
