/*
 * Copyright (c) 2026 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief AXI JESD204 link cores (RX framer / TX deframer).
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_MISC_JESD204_AXI_JESD204_H_
#define ZEPHYR_INCLUDE_DRIVERS_MISC_JESD204_AXI_JESD204_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief AXI JESD204 link-core interface
 * @defgroup axi_jesd204_interface AXI JESD204 link-core interface
 * @ingroup io_interfaces
 *
 * One devicetree node per direction (adi,axi-jesd204-rx-1.0 /
 * adi,axi-jesd204-tx-1.0), so every entry point takes the device it acts on.
 *
 * @{
 */

/**
 * @brief Configure one link core.
 *
 * Verifies core identity, programs the multiframe/frame geometry and, on the
 * deframer, the ILAS words, leaving the link disabled. Which of the two it does
 * comes from the node's compatible.
 *
 * Identity (MAGIC, PCORE version, synthesised lane count) is validated here,
 * which doubles as proof that the PL AXI plane is alive and carries the expected
 * bitstream.
 *
 * The geometry the two nodes must agree on is checked at build time, and HD is
 * derived from M/S/NP/L rather than being a property at all -- see
 * @ref JESD204_DERIVE_HD.
 *
 * @param dev Link-core device.
 * @retval 0 on success.
 * @retval -ENODEV if @p dev is not ready.
 * @retval -ENOTSUP on an unexpected MAGIC, core version or lane count.
 */
int axi_jesd204_configure(const struct device *dev);

/**
 * @brief Enable this core's lane clock.
 *
 * Clears LINK_DISABLE. Driven by the JESD204 bring-up sequence after the
 * transceiver and SYSREF are up -- not called standalone.
 *
 * On the deframer this first clears the sticky SYSREF status, so the status
 * afterwards describes a SYSREF seen by the now-enabled link rather than one
 * captured before it.
 *
 * @param dev Link-core device.
 * @retval 0 on success.
 * @retval -ENODEV if @p dev is not ready.
 */
int axi_jesd204_lane_clk_enable(const struct device *dev);

/**
 * @brief Inspect each lane once the link is up and restart it if any desynced.
 *
 * The aggregate LINK_STATUS can read DATA while one lane has already lost
 * alignment, so reaching DATA is not by itself proof that every lane is healthy.
 * This checks them individually and bounces LINK_DISABLE if any is not.
 *
 * Framer only: a deframer's register map has ILAS words where LANE_STATUS is, so
 * it would "pass" against link configuration instead of lane state.
 *
 * @param dev Link-core device (framer).
 * @retval 0 if every lane is healthy, or the link is disabled or not yet in
 *         DATA (nothing to check).
 * @retval -EAGAIN if a restart was issued; the link needs time to re-negotiate,
 *         so re-read status rather than treating this as a hard failure.
 * @retval -ENODEV if @p dev is not ready.
 * @retval -ENOTSUP on a deframer node.
 */
int axi_jesd204_rx_watchdog(const struct device *dev);

/**
 * @brief Read and log one core's full link status table.
 *
 * Logs the same multi-line table as no-OS's axi_jesd204_{rx,tx}_status_read():
 * link enabled/disabled, measured and reported link clock, lane rate, lane
 * rate/40, LMFC (or SYNC~ state on TX) rate, link phase (CGS / ILAS / DATA),
 * and SYSREF captured / alignment-error. Reported/lane rate and the LMFC
 * derivation come from the node's adi,lane-rate-khz property. Meaningful only
 * at the end of the bring-up sequence.
 *
 * Per core rather than both at once: the state labels differ by direction (the
 * deframer's 0x2 is ILAS, the framer's is CGS), so a shared reader would have to
 * take both devices to pick the right table anyway.
 *
 * @param dev Link-core device.
 * @retval 0 when it reports DATA.
 * @retval -EIO otherwise.
 * @retval -ENODEV if @p dev is not ready.
 */
int axi_jesd204_status_read(const struct device *dev);

/**
 * @brief Test whether this core reports DATA, without logging.
 *
 * The silent counterpart of axi_jesd204_status_read(), for polling: a caller
 * waiting for the link to negotiate would otherwise emit a log line per attempt
 * and bury whatever failure it was waiting on. Log once with
 * axi_jesd204_status_read() after the poll settles.
 *
 * @param dev Link-core device.
 * @return true if the link is enabled and in DATA.
 */
bool axi_jesd204_link_is_data(const struct device *dev);

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_MISC_JESD204_AXI_JESD204_H_ */
