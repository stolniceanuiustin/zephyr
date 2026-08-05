/*
 * Copyright (c) 2026 Analog Devices, Inc.
 *
 * AXI ADXCVR -- GT transceiver (PHY) bring-up.
 *
 * One device per JESD204 direction, from devicetree (compatible
 * "adi,axi-adxcvr-1.0"). Get the handles with
 * DEVICE_DT_GET(DT_NODELABEL(tx_adxcvr)) / (rx_adxcvr).
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_MISC_JESD204_AXI_ADXCVR_H_
#define ZEPHYR_INCLUDE_DRIVERS_MISC_JESD204_AXI_ADXCVR_H_

#include <zephyr/device.h>

/*
 * Configure one transceiver: read back SYNTH/VERSION, select the PLL source +
 * output-clock mux (REG_CONTROL) and program the GT dividers over DRP, holding
 * the core in reset.
 *
 * This is the DEVICE_INIT-phase step: it programs registers but does NOT gate on
 * transceiver-ready -- the GT status only becomes meaningful once the JESD204
 * link layer and SYSREF are brought up around it. Returns 0 on success.
 *
 * Not done from the driver's own init(): it needs the GT reference clock rate
 * from the HMC7044, and that chip is SPI-attached and so programs itself at
 * POST_KERNEL. Reading its rate any earlier reads an unprogrammed chip.
 */
int axi_adxcvr_configure(const struct device *dev);

/*
 * Release the GT reset and poll for transceiver-ready, clearing any
 * elastic-buffer under/overflow. This is the CLOCKS_ENABLE-phase step and must
 * be driven by the JESD204 bring-up sequence (after link setup + SYSREF), NOT
 * called standalone -- doing so times out because the datapath isn't clocked
 * yet. Returns 0 when the transceiver reports ready, negative errno otherwise.
 *
 * Per direction rather than both at once, so the bring-up sequence can attempt
 * (and report) each GT independently instead of bailing on the first failure.
 */
int axi_adxcvr_enable(const struct device *dev);

#endif /* ZEPHYR_INCLUDE_DRIVERS_MISC_JESD204_AXI_ADXCVR_H_ */
