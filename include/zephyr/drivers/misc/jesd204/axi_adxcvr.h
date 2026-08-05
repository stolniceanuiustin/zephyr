/*
 * Copyright (c) 2026 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief AXI ADXCVR -- GT transceiver (PHY) bring-up.
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_MISC_JESD204_AXI_ADXCVR_H_
#define ZEPHYR_INCLUDE_DRIVERS_MISC_JESD204_AXI_ADXCVR_H_

#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief AXI ADXCVR transceiver interface
 * @defgroup axi_adxcvr_interface AXI ADXCVR transceiver interface
 * @ingroup io_interfaces
 *
 * One device per JESD204 direction, from devicetree (compatible
 * "adi,axi-adxcvr-1.0").
 *
 * @{
 */

/**
 * @brief Configure one transceiver.
 *
 * Reads back SYNTH/VERSION, selects the PLL source and output-clock mux
 * (REG_CONTROL), then programs the GT dividers over DRP with the core held in
 * reset.
 *
 * Registers only: the GT ready status is not gated on here, because it only
 * becomes meaningful once the JESD204 link layer and SYSREF are up around it.
 * axi_adxcvr_enable() does that part.
 *
 * Not done from the driver's own init() -- it needs the GT reference clock rate
 * from the clock generator, which is SPI-attached and programs itself at
 * POST_KERNEL. Reading its rate earlier reads an unprogrammed chip.
 *
 * @param dev Transceiver device.
 * @retval 0 on success.
 * @retval -ENODEV if @p dev is not ready.
 * @retval -errno on a reference-rate query or divider-programming failure.
 */
int axi_adxcvr_configure(const struct device *dev);

/**
 * @brief Release the GT reset and wait for transceiver-ready.
 *
 * Clears any elastic-buffer under/overflow afterwards. Must be driven by the
 * JESD204 bring-up sequence, after link setup and SYSREF: called standalone it
 * times out, because the datapath is not clocked yet.
 *
 * Per direction rather than both at once, so the bring-up sequence can attempt
 * and report each GT independently instead of bailing on the first failure.
 *
 * @param dev Transceiver device.
 * @retval 0 when the transceiver reports ready.
 * @retval -ENODEV if @p dev is not ready.
 * @retval -EIO if it does not become ready, or reports a buffer error.
 */
int axi_adxcvr_enable(const struct device *dev);

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_MISC_JESD204_AXI_ADXCVR_H_ */
