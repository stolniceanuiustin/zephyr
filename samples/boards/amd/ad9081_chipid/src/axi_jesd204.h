/*
 * AXI JESD204 link cores (RX framer / TX deframer).
 *
 * One devicetree node per direction (adi,axi-jesd204-rx-1.0 /
 * adi,axi-jesd204-tx-1.0), so every entry point takes the device it acts on.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef AXI_JESD204_H_
#define AXI_JESD204_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/device.h>

/*
 * Configure one link core: verify identity, program the multiframe/frame
 * geometry and (for the deframer) the ILAS words, holding the link disabled.
 * Which of the two it does comes from the node's compatible. LINK_INIT-phase,
 * configure-only -- no status gating. Returns 0 on success, negative errno
 * otherwise.
 *
 * Identity (MAGIC, PCORE version, synthesised lane count) is validated here,
 * which is also the proof that the PL AXI plane is alive and carries the
 * expected bitstream. The core register pages are 1:1 mapped by a PRE_KERNEL_1
 * SYS_INIT in axi_jesd204.c.
 *
 * The geometry the two nodes must agree on is checked at build time
 * (BUILD_ASSERTs at the bottom of axi_jesd204.c), and HD is derived from
 * M/S/NP/L rather than being a property at all -- see the derivation there.
 */
int axi_jesd204_configure(const struct device *dev);

/*
 * Enable this core's lane clock (LINK_DISABLE=0). A CLOCKS_ENABLE-phase step,
 * driven by the JESD204 bring-up sequence after the transceiver and SYSREF are
 * up -- not called standalone.
 *
 * On the deframer this also clears the sticky SYSREF status first, so the status
 * afterwards describes a SYSREF seen by the now-enabled link rather than one
 * captured before it. no-OS does that on the deframer only.
 */
int axi_jesd204_lane_clk_enable(const struct device *dev);

/*
 * Inspect each lane once the link is up and restart it if any lane desynced.
 * Framer only -- returns -ENOTSUP on a deframer node, whose register map has
 * ILAS words where LANE_STATUS is and would therefore "pass" against link
 * configuration instead of lane state.
 *
 * The aggregate LINK_STATUS can read DATA while one lane has already lost
 * alignment, so reaching DATA is not by itself proof that every lane is healthy.
 * This checks them individually and bounces LINK_DISABLE if any is not.
 *
 * No-ops (returns 0) when the link is disabled or not yet in DATA. Returns
 * -EAGAIN if a restart was issued -- the link needs time to re-negotiate, so the
 * caller should re-read status rather than treat it as a hard failure.
 *
 * Both outcomes are verified by fault injection: forcing a desynced LANE_STATUS
 * makes this bounce the link and return -EAGAIN, after which the link
 * renegotiates to DATA in well under 100 ms; with nothing forced it leaves a
 * healthy link alone. That second check is the regression test for an inverted
 * polarity bug that used to restart working links.
 */
int axi_jesd204_rx_watchdog(const struct device *dev);

/*
 * Read and log one core's link state (link enabled?, CGS/ILAS/DATA phase).
 * Returns 0 when it reports DATA, -EIO otherwise. Meaningful only at the end of
 * the bring-up sequence (LINK_RUNNING).
 *
 * Per-core rather than both-at-once: the state labels differ by direction (the
 * deframer's 0x2 is ILAS, the framer's is CGS), so a shared reader would have to
 * take both devices to pick the right table anyway. Call it TX then RX to keep
 * the boot log's order.
 */
int axi_jesd204_status_read(const struct device *dev);

/*
 * True when this core reports DATA. The silent counterpart of
 * axi_jesd204_status_read(), for polling: a caller waiting for the link to
 * negotiate would otherwise emit a log line per attempt and bury whatever
 * failure it was waiting on. Log once with status_read() after the poll settles.
 */
bool axi_jesd204_link_is_data(const struct device *dev);

#ifdef CONFIG_AD9081_FAULT_INJECTION
/*
 * Fault-injection hooks (CONFIG_AD9081_FAULT_INJECTION only). Framer only, as
 * the watchdog they exercise is.
 *
 * A lane desync cannot be caused from software: LANE_STATUS is driven by the
 * core's alignment logic and is read-only. So instead of faking a desync in the
 * hardware, force the value the watchdog *reads* -- everything after that read
 * (the healthy/desynced decision, the LINK_DISABLE bounce, the -EAGAIN return)
 * is the real code path under test.
 *
 * force_lane_status() makes every lane of `dev` read `status` until
 * clear_lane_status(). Pass a word whose low two bits are 0 to look desynced,
 * non-zero for healthy.
 */
void axi_jesd204_fi_force_lane_status(const struct device *dev,
				      uint32_t status);
void axi_jesd204_fi_clear_lane_status(const struct device *dev);

/* Unforced reads, so a test can report what the hardware actually says. */
uint32_t axi_jesd204_fi_lane_status(const struct device *dev, uint32_t lane);
uint32_t axi_jesd204_fi_num_lanes(const struct device *dev);
#endif

#endif /* AXI_JESD204_H_ */
