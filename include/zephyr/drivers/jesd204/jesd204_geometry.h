/*
 * JESD204 link facts that are not configuration of this board.
 *
 * Shared between the FPGA link-core driver (axi_jesd204.c), which advertises
 * them in ILAS, and the MxFE driver (ad9081.c), which configures the chip's
 * framer/deframer against the same link. They were private to axi_jesd204.c
 * until the chip side stopped carrying its geometry as literals; two copies of
 * a derivation is exactly the drift this port has already been bitten by once.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_JESD204_JESD204_GEOMETRY_H_
#define ZEPHYR_INCLUDE_DRIVERS_JESD204_JESD204_GEOMETRY_H_

/*
 * Not devicetree properties, because they are not configuration of this board:
 *
 *   JESD204_VERSION_B  8B/10B line coding, i.e. JESD204B. Synthesised into the
 *                      PL (make JESD_MODE=8B10B) and load-bearing in two places
 *                      that look unrelated -- ILAS word 2 bits [31:29], and the
 *                      lane-status polarity the watchdog reads. A 204C link
 *                      would be a different driver, not a different property.
 *                      ADI's Linux drivers likewise hardcode it
 *                      (axi_jesd204_tx.c:394, axi_jesd204_rx.c:602).
 *   JESD204_SCRAMBLING scrambling is always on. Also hardcoded in ADI's Linux
 *                      drivers (axi_jesd204_tx.c:392) and in no-OS.
 */
#define JESD204_VERSION_B   1
#define JESD204_SCRAMBLING  1

/*
 * HD ("high density") is 1 only when a single converter sample is spread over
 * more than one lane, which the frame geometry decides on its own:
 *
 *     bits per lane per frame = M * S * NP / L
 *     a sample splits iff that is not a whole number of NP-bit samples
 *
 * So HD is derived, never chosen -- which is why no binding here has an
 * adi,high-density property, despite ADI's Linux binding offering one. no-OS's
 * profile for this board carries HD=1 for a geometry that requires 0 (harmless
 * at F=4, since nothing splits either way); a settable property is how that
 * value would get copied back in.
 *
 * For this link: 8 * 1 * 16 / 4 = 32 bits/lane, 32 / 16 = 2 whole samples, so
 * HD = 0.
 */
#define JESD204_DERIVE_HD(m, s, np, l) ((((m) * (s) * (np) / (l)) % (np)) != 0)

#endif /* ZEPHYR_INCLUDE_DRIVERS_JESD204_JESD204_GEOMETRY_H_ */
