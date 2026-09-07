/*
 * Copyright (c) 2026 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief AD9144 TX DAC public interface.
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_MISC_AD9144_H_
#define ZEPHYR_INCLUDE_DRIVERS_MISC_AD9144_H_

#include <stdbool.h>

#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief AD9144 interface
 * @defgroup ad9144_interface AD9144 TX DAC interface
 * @ingroup io_interfaces
 * @{
 */

/**
 * @brief Re-read the SERDES (deframer CDR) PLL lock status.
 *
 * The AD9144's deframer recovers its clock from the serial lanes the FPGA
 * transmits, so its SERDES PLL cannot lock until the FPGA TX transceiver and
 * JESD204 TX link core are up and sending. init() reads this once at setup,
 * before that datapath exists, and expects NOT locked; this entry point lets the
 * bring-up sequence re-read it after the FPGA side is armed.
 *
 * @param dev    AD9144 device.
 * @param locked Out: true if the SERDES PLL reports locked.
 * @retval 0 on success.
 * @retval -EINVAL if @p dev or @p locked is NULL.
 * @retval -ENODEV if @p dev is not ready.
 * @retval -errno on an SPI read failure.
 */
int ad9144_serdes_pll_locked(const struct device *dev, bool *locked);

/**
 * @brief Re-run the SERDES PLL bring-up and report the lock.
 *
 * Unlike ad9144_serdes_pll_locked(), which only re-reads the status register,
 * this re-executes the enable sequence (CDR reset + PLL enable + settle). The
 * SERDES PLL only locks once the FPGA GT lane clock is present, which is not the
 * case at chip init() -- so the init-time enable reads 0x00 and never locks. The
 * bring-up FSM calls this after the transceiver CLOCKS_ENABLE phase releases the
 * GT, mirroring no-OS's LINK_SETUP phase (no-OS ad9144.c:860).
 *
 * @param dev    AD9144 device.
 * @param locked Out: true if the SERDES PLL reports locked after re-enable.
 * @retval 0 on success.
 * @retval -EINVAL if @p dev or @p locked is NULL.
 * @retval -ENODEV if @p dev is not ready.
 * @retval -errno on an SPI failure.
 */
int ad9144_serdes_enable(const struct device *dev, bool *locked);

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_MISC_AD9144_H_ */
