/*
 * AXI ADXCVR -- GT transceiver (PHY) bring-up.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef AXI_ADXCVR_H_
#define AXI_ADXCVR_H_

/*
 * Configure the TX and RX GT transceivers: read back SYNTH/VERSION and select
 * the PLL source + output-clock mux (REG_CONTROL), holding the core in reset.
 * Trusts the synthesised bitstream for the GT DRP attributes (see axi_adxcvr.c).
 *
 * This is the DEVICE_INIT-phase step: it programs registers but does NOT gate on
 * transceiver-ready -- the GT status only becomes meaningful once the JESD204
 * link layer and SYSREF are brought up around it. Returns 0 on success.
 */
int axi_adxcvr_configure(void);

/*
 * Release the GT reset and poll for transceiver-ready, clearing any
 * elastic-buffer under/overflow. This is the CLOCKS_ENABLE-phase step and must
 * be driven by the JESD204 bring-up sequence (after link setup + SYSREF), NOT
 * called standalone -- doing so times out because the datapath isn't clocked
 * yet. Returns 0 when both transceivers report ready, negative errno otherwise.
 */
int axi_adxcvr_enable(void);

/*
 * Same as axi_adxcvr_enable() but per direction, so the bring-up sequence can
 * attempt (and report) each GT independently instead of bailing on the first
 * failure. Returns 0 when that transceiver reports ready, negative errno else.
 */
int axi_adxcvr_tx_enable(void);
int axi_adxcvr_rx_enable(void);

#endif /* AXI_ADXCVR_H_ */
