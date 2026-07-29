/*
 * JESD204 datapath validation -- Rung 4: DDR -> DMA -> DAC playback.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef JESD_PLAYBACK_H_
#define JESD_PLAYBACK_H_

/*
 * Rung 4: build a sine table in DDR and push it out through the TX AXI DMAC
 * (MEM_TO_DEV) -> dac_tpl_core -> JRX deframer -> DAC. Must be called only after
 * the JESD204 link has reached DATA. Runs one bounded transfer to prove the
 * engine completes, then (if the core supports it) leaves a cyclic transfer
 * running so the tone is continuously present at the DAC output for a scope.
 *
 * Returns 0 if the transfer completed and the link stayed healthy underneath it,
 * negative errno otherwise. Note this verifies the *transmit mechanism*, not the
 * analog output -- only a scope (or Rung 5's analog loopback) can do that.
 */
int jesd_playback_sine(void);

#endif /* JESD_PLAYBACK_H_ */
