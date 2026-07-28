/*
 * AXI JESD204 link cores (RX framer / TX deframer).
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef AXI_JESD204_H_
#define AXI_JESD204_H_

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
 * Read and log the TX/RX link state (link enabled?, CGS/ILAS/DATA phase).
 * Returns 0 when both the framer and deframer report DATA, -EIO otherwise.
 * Meaningful only at the end of the bring-up sequence (LINK_RUNNING).
 */
int axi_jesd204_status_read(void);

#endif /* AXI_JESD204_H_ */
