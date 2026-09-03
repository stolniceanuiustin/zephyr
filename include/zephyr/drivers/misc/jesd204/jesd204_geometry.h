/*
 * Copyright (c) 2026 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief JESD204 link facts that are not configuration of this board.
 *
 * Shared between the FPGA link-core driver, which advertises them in ILAS, and
 * the MxFE driver, which configures the chip's framer/deframer against the same
 * link. Two copies of a derivation is exactly the drift this has already been
 * bitten by once.
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_MISC_JESD204_JESD204_GEOMETRY_H_
#define ZEPHYR_INCLUDE_DRIVERS_MISC_JESD204_JESD204_GEOMETRY_H_

/**
 * @brief JESD204 link geometry
 * @defgroup jesd204_geometry JESD204 link geometry
 * @ingroup io_interfaces
 * @{
 */

/**
 * @brief JESD204B (8B/10B) line coding, as advertised in ILAS.
 *
 * Not a devicetree property: it is synthesised into the PL, and load-bearing in
 * two places that look unrelated -- ILAS word 2 bits [31:29], and the lane-status
 * polarity the watchdog reads. A 204C link would be a different driver, not a
 * different property value.
 */
#define JESD204_VERSION_B 1

/** @brief Scrambling is always enabled on this link. */
#define JESD204_SCRAMBLING 1

/**
 * @brief Derive HD ("high density") from the frame geometry.
 *
 * HD is 1 only when a single converter sample is spread over more than one lane,
 * which the frame geometry decides on its own:
 *
 *     bits per lane per frame = M * S * NP / L
 *     a sample splits iff that is not a whole number of NP-bit samples
 *
 * So HD is derived, never chosen -- which is why no binding here has an
 * adi,high-density property, despite ADI's Linux binding offering one. A settable
 * property is how a wrong value gets copied in from a reference profile.
 *
 * For this link: 8 * 1 * 16 / 4 = 32 bits/lane, 32 / 16 = 2 whole samples, HD = 0.
 *
 * @param m  Converters per device (M).
 * @param s  Samples per converter per frame (S).
 * @param np Bits per sample including control (N').
 * @param l  Lanes per link (L).
 * @return 1 if a sample splits across lanes, 0 otherwise.
 */
#define JESD204_DERIVE_HD(m, s, np, l) ((((m) * (s) * (np) / (l)) % (np)) != 0)

/** @} */

#endif /* ZEPHYR_INCLUDE_DRIVERS_MISC_JESD204_JESD204_GEOMETRY_H_ */
