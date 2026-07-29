/*
 * JESD204 datapath validation -- Rung 4: DDR -> DMA -> DAC playback.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef JESD_PLAYBACK_H_
#define JESD_PLAYBACK_H_

/*
 * Tone geometry, shared with the Rung 5 loopback check so both ends agree on
 * what is being played without duplicating the constants.
 *
 * The table is one period over 64 beats. The TX datapath runs at DAC 12 GHz /
 * (interp 6 x 8) = 250 MSPS, so the baseband tone is 250e6/64 = 3.90625 MHz. The
 * RX datapath decimates ADC 4 GHz / (4 x 4) to the same 250 MSPS, and the RX
 * coarse DDC shift (-2 GHz) is the exact negation of the TX main NCO (+2 GHz),
 * so an analog loopback returns the tone to the same baseband frequency it
 * started at. Choosing 64 to divide the 512-beat capture length means the tone
 * lands exactly on an FFT bin, making the loopback check a clean single-bin test
 * with no spectral leakage to threshold around.
 */
#define JESD_PB_PERIOD_BEATS 64U
#define JESD_PB_SAMPLE_RATE  250000000U
#define JESD_PB_TONE_HZ      (JESD_PB_SAMPLE_RATE / JESD_PB_PERIOD_BEATS)
#define JESD_PB_AMPLITUDE    24576

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
