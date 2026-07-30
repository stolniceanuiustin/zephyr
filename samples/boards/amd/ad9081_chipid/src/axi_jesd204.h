/*
 * AXI JESD204 link cores (RX framer / TX deframer).
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef AXI_JESD204_H_
#define AXI_JESD204_H_

#include <stdbool.h>
#include <stdint.h>

/*
 * Configure the RX and TX JESD204 link cores for the AD9082 m8-l4 link: verify
 * identity, program the multiframe/frame geometry and (for TX) the ILAS words,
 * holding both links disabled. LINK_INIT-phase, configure-only -- no status
 * gating. Returns 0 on success, negative errno otherwise.
 *
 * Note: the axi_jesd core register pages are mapped by axi_jesd.c's SYS_INIT.
 */
int axi_jesd204_configure(void);

/*
 * Enable the TX/RX lane clocks (LINK_DISABLE=0). CLOCKS_ENABLE-phase steps,
 * driven by the JESD204 bring-up sequence after the transceiver and SYSREF are
 * up -- not called standalone.
 */
int axi_jesd204_tx_lane_clk_enable(void);
int axi_jesd204_rx_lane_clk_enable(void);

/*
 * Inspect each RX lane once the link is up and restart it if any lane desynced.
 *
 * The aggregate LINK_STATUS can read DATA while one lane has already lost
 * alignment, so reaching DATA is not by itself proof that every lane is healthy.
 * This checks them individually and bounces LINK_DISABLE if any is not.
 *
 * No-ops (returns 0) when the link is disabled or not yet in DATA. Returns
 * -EAGAIN if a restart was issued -- the link needs time to re-negotiate, so the
 * caller should re-read status rather than treat it as a hard failure.
 */
int axi_jesd204_rx_watchdog(void);

/*
 * Read and log the TX/RX link state (link enabled?, CGS/ILAS/DATA phase).
 * Returns 0 when both the framer and deframer report DATA, -EIO otherwise.
 * Meaningful only at the end of the bring-up sequence (LINK_RUNNING).
 */
int axi_jesd204_status_read(void);

/*
 * True when both link cores report DATA. The silent counterpart of
 * axi_jesd204_status_read(), for polling: a caller waiting for the link to
 * negotiate would otherwise emit two log lines per attempt and bury whatever
 * failure it was waiting on. Log once with status_read() after the poll settles.
 */
bool axi_jesd204_link_is_data(void);

#ifdef CONFIG_AD9081_FAULT_INJECTION
/*
 * Fault-injection hooks (CONFIG_AD9081_FAULT_INJECTION only).
 *
 * A lane desync cannot be caused from software: LANE_STATUS is driven by the
 * core's alignment logic and is read-only. So instead of faking a desync in the
 * hardware, force the value the watchdog *reads* -- everything after that read
 * (the healthy/desynced decision, the LINK_DISABLE bounce, the -EAGAIN return)
 * is the real code path under test.
 *
 * force_lane_status() makes every lane read `status` until clear_lane_status().
 * Pass a word whose low two bits are 0 to look desynced, non-zero for healthy.
 */
void axi_jesd204_fi_force_lane_status(uint32_t status);
void axi_jesd204_fi_clear_lane_status(void);

/* Unforced reads, so a test can report what the hardware actually says. */
uint32_t axi_jesd204_fi_lane_status(uint32_t lane);
uint32_t axi_jesd204_fi_num_lanes(void);
#endif

#endif /* AXI_JESD204_H_ */
