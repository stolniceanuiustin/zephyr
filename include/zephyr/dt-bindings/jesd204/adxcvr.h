/*
 * Copyright (c) 2026 Analog Devices, Inc.
 *
 * AXI ADXCVR devicetree constants.
 *
 * Copied from ADI's Linux tree (include/dt-bindings/jesd204/adxcvr.h) so that a
 * devicetree written for ADI Linux transfers to this port unchanged. Values are
 * the GT wrapper's REG_CONTROL field encodings and must not be renumbered --
 * they are what the IP decodes.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_JESD204_ADXCVR_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_JESD204_ADXCVR_H_

/* adi,sys-clk-select -- which PLL clocks the transceiver. */
#define XCVR_CPLL  0 /* CPLL:  GTHE3, GTHE4, GTYE4, GTXE2 */
#define XCVR_QPLL1 2 /* QPLL1: GTHE3, GTHE4, GTYE4 */
#define XCVR_QPLL  3 /* QPLL0: GTHE3, GTHE4, GTYE4, GTXE2 */

/* adi,out-clk-select -- which GT output drives the fabric lane clock. */
#define XCVR_OUTCLK_PCS  1
#define XCVR_OUTCLK_PMA  2
#define XCVR_REFCLK      3
#define XCVR_REFCLK_DIV2 4
#define XCVR_PROGDIV_CLK 5 /* GTHE3, GTHE4, GTYE4 only */

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_JESD204_ADXCVR_H_ */
