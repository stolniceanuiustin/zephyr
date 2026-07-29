/*
 * Analog-loopback fault isolation for Rung 5.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef JESD_DIAG_H_
#define JESD_DIAG_H_

/*
 * Run when Rung 5 finds nothing at the ADC despite a cable being installed and
 * every readable status register reporting healthy. Rung 5's verdict is binary,
 * which is what a validation rung should be, but a binary answer cannot say
 * *where* the signal is lost. These checks split that one silent result into
 * distinguishable cases without needing a scope:
 *
 *   1. Digital gain readback -- is the TX fine-DUC channel gain actually the
 *      1024 we programmed? A zero gain gives a perfectly healthy link carrying
 *      silence, which is precisely the symptom.
 *   2. Frequency sweep -- retune the TX main NCO and RX coarse DDC together and
 *      re-measure at each point. Distinguishes "nothing ever gets through"
 *      (flat noise floor everywhere: a broken path) from "this frequency does
 *      not get through" (a response that varies: a band or aliasing problem).
 *   3. Chip-internal DAC test tone -- have the DAC generate a signal from its
 *      own calibration DC input, bypassing our DMA, the link and the deframer
 *      entirely. If the internal tone returns but ours does not, the fault is
 *      upstream in our transmit datapath; if neither returns, it is in the DAC
 *      output or the board's analog path.
 *
 * Purely diagnostic: it reports and restores, and never changes the verdict of
 * any rung. Returns 0 if the checks ran (regardless of what they found), or a
 * negative errno if a check could not be performed at all.
 */
int jesd_diag_loopback(void);

#endif /* JESD_DIAG_H_ */
