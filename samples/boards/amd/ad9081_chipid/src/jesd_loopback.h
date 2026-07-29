/*
 * JESD204 datapath validation -- Rung 5: analog DAC -> ADC loopback.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef JESD_LOOPBACK_H_
#define JESD_LOOPBACK_H_

#include <stdint.h>

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

/*
 * A single loopback measurement, with no verdict attached.
 *
 * Rung 5 answers "is the tone there?" in one shot, which is the right shape for a
 * pass/fail rung but the wrong shape for hunting a fault: a sweep needs the raw
 * numbers at each point so the *shape* of the response across frequency is
 * visible. This exposes the same capture and correlator behind that verdict so a
 * diagnostic can call it repeatedly without duplicating either.
 */
struct jesd_loopback_meas {
	uint64_t rms;             /* per-sample RMS over the paired I/Q */
	uint64_t lane_rms[8];     /* per-lane RMS -- layout-agnostic raw evidence */
	uint64_t ch_rms[4];       /* interleaved reading: lanes (2n, 2n+1) */
	uint32_t concentration;   /* permille of energy in the tone bin, interleaved */
	uint64_t amplitude;       /* recovered tone amplitude, vs JESD_PB_AMPLITUDE */
	/*
	 * The same correlation under the other candidate beat layout. The
	 * virtual-converter order the chip is programmed with is I,Q,I,Q, but the
	 * order that reaches DDR after the FPGA transport core and lane mapping
	 * need not match, and pairing an I with another channel's I scores exactly
	 * the ~500 permille a real-valued signal gives -- indistinguishable from
	 * genuine quadrature imbalance if only one pairing is measured. Measuring
	 * both makes the layout an observation rather than an assumption.
	 */
	uint32_t concentration_split; /* pairing lane n with lane n+4 */
	uint64_t amplitude_split;
};

/*
 * Capture once and measure, filling *m. Returns 0 on a successful capture (even
 * if the input is silent -- silence is a measurement, not an error here), or
 * negative errno if the capture itself failed.
 */
int jesd_loopback_measure(struct jesd_loopback_meas *m);

#endif /* JESD_LOOPBACK_H_ */
