/*
 * JESD204 datapath validation -- Rung 5: analog DAC -> ADC loopback.
 *
 * This is the top of the ladder and the only rung that proves the analog path.
 * Rung 4 showed the transmit *mechanism* works, but nothing in the FPGA can see
 * what the DAC actually produced -- a deframer quietly dropping samples and a
 * perfect sine look identical from the AXI side. Rung 5 closes that loop by
 * sending the signal out through the analog world and catching it again:
 *
 *   DDR tone -> TX DMAC -> deframer -> DAC -> [SMA cable] -> ADC
 *     -> JTX framer -> link -> RX TPL -> RX DMAC -> DDR -> verify
 *
 * Everything is now under test at once: both DMA engines, both directions of the
 * JESD link, both chip datapaths, the DAC's reconstruction and the ADC's
 * digitising. If the tone comes back at the right frequency and a sane
 * amplitude, the whole signal chain works.
 *
 * Detection method
 * ----------------
 * Rather than an FFT, this correlates the capture against the exact tone that was
 * transmitted -- a single-bin DFT (the Goertzel idea, done directly). The tone
 * was deliberately chosen so one period is 64 beats and the capture is 512 beats,
 * so it sits exactly on bin 8 with no spectral leakage: the correlation phase
 * advances by 2*pi/64 per beat, which means the same 64-point cos/sin table used
 * to generate it can be indexed by (beat % 64). No FFT, no floating point, no
 * windowing.
 *
 * The verdict is the fraction of total captured energy that lands in that one
 * bin. For a clean single tone this is ~1.0; for broadband noise (an
 * unterminated ADC input) it is ~1/512. That ratio is the discriminator, and it
 * is scale-free -- it does not depend on cable loss, DAC output level or ADC
 * full-scale, which is what makes it a robust pass/fail rather than a threshold
 * that needs tuning per board.
 *
 * Why a missing cable is not a failure
 * ------------------------------------
 * Nobody has jumpered this board yet, so the overwhelmingly likely first result
 * is "no signal". That is reported as -ENODATA with an explicit instruction,
 * distinct from a real datapath fault -- an absent cable should not read like a
 * broken link.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>

#include <zephyr/kernel.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(jesd_loopback, LOG_LEVEL_INF);

#include "jesd_capture.h"
#include "jesd_loopback.h"
#include "jesd_playback.h"

/*
 * Quarter-scale cos/sin over one tone period. Amplitude 16384 = 2^14 so the
 * correlation is a plain integer multiply-accumulate with an exact power-of-two
 * scale factor to divide out.
 */
#define LB_TWIDDLE_SCALE 14
#define LB_TWIDDLE_ONE   (1 << LB_TWIDDLE_SCALE)

static const int16_t lb_cos[JESD_PB_TABLE_LEN] = {
	 16384,  16305,  16069,  15679,  15137,  14449,  13623,  12665,
	 11585,  10394,   9102,   7723,   6270,   4756,   3196,   1606,
	     0,  -1606,  -3196,  -4756,  -6270,  -7723,  -9102, -10394,
	-11585, -12665, -13623, -14449, -15137, -15679, -16069, -16305,
	-16384, -16305, -16069, -15679, -15137, -14449, -13623, -12665,
	-11585, -10394,  -9102,  -7723,  -6270,  -4756,  -3196,  -1606,
	     0,   1606,   3196,   4756,   6270,   7723,   9102,  10394,
	 11585,  12665,  13623,  14449,  15137,  15679,  16069,  16305,
};

static const int16_t lb_sin[JESD_PB_TABLE_LEN] = {
	     0,   1606,   3196,   4756,   6270,   7723,   9102,  10394,
	 11585,  12665,  13623,  14449,  15137,  15679,  16069,  16305,
	 16384,  16305,  16069,  15679,  15137,  14449,  13623,  12665,
	 11585,  10394,   9102,   7723,   6270,   4756,   3196,   1606,
	     0,  -1606,  -3196,  -4756,  -6270,  -7723,  -9102, -10394,
	-11585, -12665, -13623, -14449, -15137, -15679, -16069, -16305,
	-16384, -16305, -16069, -15679, -15137, -14449, -13623, -12665,
	-11585, -10394,  -9102,  -7723,  -6270,  -4756,  -3196,  -1606,
};

/*
 * Signal-present floor. An unconnected ADC input still digitises thermal noise
 * and its own quantisation, typically a handful of LSBs RMS. Anything below this
 * per-sample RMS is treated as "nothing is connected" rather than a bad tone.
 *
 * Kept deliberately low: this gate exists only to distinguish an absent cable
 * from a real fault, and a too-high floor would mask a weak-but-present tone as
 * "no cable" -- the concentration test below is what actually judges the signal,
 * and it works fine at low amplitude (verified at -20 dB). The RMS is logged
 * either way so a marginal level is visible rather than inferred.
 */
#define LB_SILENCE_RMS 16

/*
 * Bin-concentration threshold, in permille of total energy. Verified against
 * simulated captures: a clean tone scores 999 and stays at 999 through an
 * arbitrary phase offset (cable delay) and 20 dB of attenuation (cable loss),
 * because the measure is scale- and phase-invariant. Broadband noise scores 13
 * and a wrong-frequency tone scores 0.
 *
 * The threshold sits at 250 rather than nearer the clean-tone score to leave room
 * for an imperfectly complex return: a purely real recovered signal (severe
 * quadrature imbalance, or one half of the pair not making it back) scores 499,
 * and that should still read as "the tone came through, with a problem worth
 * seeing" rather than as a dead datapath. 250 separates any real signal from
 * noise by a wide margin in both directions.
 */
#define LB_TONE_CONCENTRATION_MIN 250

/* Integer square root (Newton), enough for 64-bit energy sums. */
static uint64_t lb_isqrt(uint64_t v)
{
	uint64_t x, prev;

	if (v == 0) {
		return 0;
	}

	x = v;
	prev = 0;
	/* Converges quickly; the equality guard also breaks the 2-cycle case. */
	while (x != prev) {
		prev = x;
		x = (x + v / x) / 2;
		if (x > prev) {
			break;
		}
	}
	return x;
}

int jesd_loopback_measure(struct jesd_loopback_meas *m)
{
	const int16_t *buf;
	size_t n, beats;
	int64_t re = 0, im = 0;
	uint64_t energy = 0;
	int ret;

	memset(m, 0, sizeof(*m));

	ret = jesd_capture_raw(&buf, &n);
	if (ret) {
		LOG_ERR("loopback capture failed (%d)", ret);
		return ret;
	}

	beats = n / JESD_CAP_LANES_PER_BEAT;
	if (beats == 0) {
		return -EINVAL;
	}

	/*
	 * Per-lane RMS first, before interpreting anything. This is the raw
	 * evidence: it says which of the eight sample slots in a beat carry
	 * energy, without assuming how those slots pair into complex channels.
	 * Which SMA on the FMC reaches which converter is also board-dependent
	 * (the connectors are dual-labelled because the card serves both the
	 * 4-ADC AD9081 and the 2-ADC AD9082 pinouts), so a signal arriving
	 * somewhere unexpected shows up here rather than looking like silence.
	 */
	for (uint32_t l = 0; l < JESD_CAP_LANES_PER_BEAT; l++) {
		uint64_t l_energy = 0;

		for (size_t b = 0; b < beats; b++) {
			int32_t v = buf[b * JESD_CAP_LANES_PER_BEAT + l];

			l_energy += (uint64_t)((int64_t)v * v);
		}
		m->lane_rms[l] = lb_isqrt(l_energy / beats);
	}

	/* Interleaved reading of the same lanes: channel n = lanes (2n, 2n+1). */
	for (uint32_t ch = 0; ch < 4; ch++) {
		uint64_t ch_energy = 0;

		for (size_t b = 0; b < beats; b++) {
			int32_t ci = buf[b * JESD_CAP_LANES_PER_BEAT + ch * 2 + 0];
			int32_t cq = buf[b * JESD_CAP_LANES_PER_BEAT + ch * 2 + 1];

			ch_energy += (uint64_t)((int64_t)ci * ci +
						(int64_t)cq * cq);
		}
		m->ch_rms[ch] = lb_isqrt(ch_energy / beats);
	}

	/*
	 * Correlate against the transmitted tone under both candidate pairings,
	 * and sum total energy alongside. Whichever layout is real produces a high
	 * concentration; the wrong one pairs an I component with an unrelated
	 * channel's I, giving a real-valued signal that scores ~500 -- so the two
	 * numbers together identify the layout instead of leaving ~500 ambiguous
	 * between "wrong pairing" and "genuine quadrature imbalance".
	 */
	int64_t re2 = 0, im2 = 0;
	uint64_t energy2 = 0;

	for (size_t b = 0; b < beats; b++) {
		const int16_t *beat = &buf[b * JESD_CAP_LANES_PER_BEAT];
		size_t t = (b * JESD_PB_STEP) % JESD_PB_TABLE_LEN;
		int32_t c = lb_cos[t];
		int32_t s = lb_sin[t];
		int32_t i_s, q_s;

		/* Multiply by e^{+j.theta} to de-rotate the transmitted e^{-j.theta}.
		 * The conjugate convention here must match pb_fill()'s negated Q --
		 * getting it backwards correlates against the mirror image and scores
		 * zero, which is exactly what an aliased return looks like. */
		i_s = beat[0];
		q_s = beat[1];
		re += ((int64_t)i_s * c - (int64_t)q_s * s) >> LB_TWIDDLE_SCALE;
		im += ((int64_t)q_s * c + (int64_t)i_s * s) >> LB_TWIDDLE_SCALE;
		energy += (uint64_t)((int64_t)i_s * i_s + (int64_t)q_s * q_s);

		i_s = beat[0];
		q_s = beat[4];
		re2 += ((int64_t)i_s * c - (int64_t)q_s * s) >> LB_TWIDDLE_SCALE;
		im2 += ((int64_t)q_s * c + (int64_t)i_s * s) >> LB_TWIDDLE_SCALE;
		energy2 += (uint64_t)((int64_t)i_s * i_s + (int64_t)q_s * q_s);
	}

	m->rms = lb_isqrt(energy / beats);
	m->amplitude = lb_isqrt((uint64_t)(re * re + im * im)) / beats;
	m->amplitude_split = lb_isqrt((uint64_t)(re2 * re2 + im2 * im2)) / beats;

	/*
	 * Energy in the tone bin vs total energy. |X|^2 / (N * total) is 1.0 for a
	 * pure tone and ~1/N for noise, independent of signal level.
	 */
	if (energy != 0) {
		m->concentration =
			(uint32_t)(((uint64_t)(re * re + im * im) * 1000U) /
				   (energy * beats));
	}
	if (energy2 != 0) {
		m->concentration_split =
			(uint32_t)(((uint64_t)(re2 * re2 + im2 * im2) * 1000U) /
				   (energy2 * beats));
	}

	return 0;
}

int jesd_loopback_verify(void)
{
	struct jesd_loopback_meas m;
	uint64_t rms;
	uint32_t concentration;
	int ret;

	LOG_INF("--- Rung 5: analog DAC -> ADC loopback ---");
	LOG_INF("expecting the Rung 4 tone back at baseband -%u MHz (RF %u MHz)",
		JESD_PB_TONE_HZ / 1000000U,
		2000U - JESD_PB_TONE_HZ / 1000000U);

	ret = jesd_loopback_measure(&m);
	if (ret) {
		return ret;
	}

	rms = m.rms;

	/*
	 * Judge on whichever I/Q pairing actually describes the buffer. The chip's
	 * virtual converters are ordered I,Q,I,Q, but what reaches DDR has been
	 * through the FPGA transport core and two lane maps, so the pairing is
	 * measured rather than assumed -- see jesd_loopback_meas. The wrong pairing
	 * scores ~500 on a perfectly good signal, which would read as permanent
	 * quadrature imbalance.
	 */
	bool split = m.concentration_split > m.concentration;

	concentration = split ? m.concentration_split : m.concentration;

	LOG_INF("per-lane RMS (which slots carry energy):");
	LOG_INF("  %llu %llu %llu %llu %llu %llu %llu %llu",
		(unsigned long long)m.lane_rms[0],
		(unsigned long long)m.lane_rms[1],
		(unsigned long long)m.lane_rms[2],
		(unsigned long long)m.lane_rms[3],
		(unsigned long long)m.lane_rms[4],
		(unsigned long long)m.lane_rms[5],
		(unsigned long long)m.lane_rms[6],
		(unsigned long long)m.lane_rms[7]);

	LOG_INF("I/Q pairing: interleaved (0,1) scores %u, split (0,4) scores %u -> %s",
		m.concentration, m.concentration_split,
		split ? "split" : "interleaved");

	LOG_INF("capture: per-sample RMS %llu", (unsigned long long)rms);

	if (rms < LB_SILENCE_RMS) {
		LOG_WRN("ADC input is silent (RMS %llu < %u)",
			(unsigned long long)rms, LB_SILENCE_RMS);
		LOG_WRN("=== Rung 5 SKIPPED: no signal at the ADC ===");
		LOG_WRN("    connect an SMA cable from any DAC output to ADC0 on the FMC,");
		LOG_WRN("    then reboot -- Rung 4 leaves the tone playing continuously.");
		LOG_WRN("    If a cable IS installed: check it is rated for ~2 GHz, and that");
		LOG_WRN("    the ADC SMA carries ADC0 (the converter this check reads).");
		return -ENODATA;
	}

	LOG_INF("tone bin: amplitude %llu (sent %d), %u/1000 of total energy",
		(unsigned long long)(split ? m.amplitude_split : m.amplitude),
		JESD_PB_AMPLITUDE, concentration);

	if (concentration < LB_TONE_CONCENTRATION_MIN) {
		LOG_ERR("signal present but it is not the transmitted tone");
		LOG_ERR("  only %u/1000 of the energy is in the expected bin (need %u)",
			concentration, LB_TONE_CONCENTRATION_MIN);
		LOG_ERR("=== Rung 5 FAIL: analog loopback carries the wrong signal ===");
		return -EIO;
	}

	/* A fully complex return concentrates ~1000; ~500 means only the real part
	 * survived. Still a pass -- the tone made the trip -- but worth naming. */
	if (concentration < 750) {
		LOG_WRN("tone recovered but only %u/1000 concentrated: likely quadrature",
			concentration);
		LOG_WRN("  imbalance or one of the I/Q halves not returning");
	}

	LOG_INF("=== Rung 5 PASS: transmitted tone recovered through the analog loopback ===");
	LOG_INF("    full chain verified: DDR -> DMA -> DAC -> cable -> ADC -> DMA -> DDR");
	return 0;
}
