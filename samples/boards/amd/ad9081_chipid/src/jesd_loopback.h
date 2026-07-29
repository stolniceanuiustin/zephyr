/*
 * JESD204 datapath validation -- Rung 5: analog DAC -> ADC loopback.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef JESD_LOOPBACK_H_
#define JESD_LOOPBACK_H_

/*
 * Rung 5: with Rung 4's tone playing continuously out of the DAC and an external
 * cable from a DAC output to an ADC input, capture the returning signal and
 * verify it is the tone that was sent. This is the only rung that proves the
 * analog path, and the only one that closes the transmit direction in software
 * rather than by eye on a scope.
 *
 * Requires: cyclic playback already armed (call jesd_playback_sine() first) and
 * an SMA cable installed on the FMC between a DAC output and an ADC input.
 *
 * Returns 0 if the expected tone was found, -ENODATA if the input looks like an
 * unconnected ADC (no cable), or negative errno on a capture failure. A missing
 * cable is reported distinctly from a real datapath fault, since it is the
 * expected outcome on a board nobody has jumpered.
 */
int jesd_loopback_verify(void);

#endif /* JESD_LOOPBACK_H_ */
