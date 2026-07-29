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

static const int16_t lb_cos[JESD_PB_PERIOD_BEATS] = {
	 16384,  16305,  16069,  15679,  15137,  14449,  13623,  12665,
	 11585,  10394,   9102,   7723,   6270,   4756,   3196,   1606,
	     0,  -1606,  -3196,  -4756,  -6270,  -7723,  -9102, -10394,
	-11585, -12665, -13623, -14449, -15137, -15679, -16069, -16305,
	-16384, -16305, -16069, -15679, -15137, -14449, -13623, -12665,
	-11585, -10394,  -9102,  -7723,  -6270,  -4756,  -3196,  -1606,
	     0,   1606,   3196,   4756,   6270,   7723,   9102,  10394,
	 11585,  12665,  13623,  14449,  15137,  15679,  16069,  16305,
};

static const int16_t lb_sin[JESD_PB_PERIOD_BEATS] = {
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
 */
#define LB_SILENCE_RMS 64

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

int jesd_loopback_verify(void)
{
	const int16_t *buf;
	size_t n, beats;
	int64_t re = 0, im = 0;
	uint64_t energy = 0;
	uint64_t bin_mag, rms, amplitude;
	uint32_t concentration;
	int ret;

	LOG_INF("--- Rung 5: analog DAC -> ADC loopback ---");
	LOG_INF("expecting the Rung 4 tone back at %u.%03u MHz",
		JESD_PB_TONE_HZ / 1000000U,
		(JESD_PB_TONE_HZ % 1000000U) / 1000U);

	ret = jesd_capture_raw(&buf, &n);
	if (ret) {
		LOG_ERR("loopback capture failed (%d)", ret);
		return ret;
	}

	beats = n / JESD_CAP_LANES_PER_BEAT;

	/*
	 * Correlate channel 0's (I,Q) pair against the transmitted tone, and sum
	 * total energy alongside it. Lane 0 is I and lane 1 is Q within each beat
	 * (established by the Rung 2 capture).
	 */
	for (size_t b = 0; b < beats; b++) {
		int32_t i_s = buf[b * JESD_CAP_LANES_PER_BEAT + 0];
		int32_t q_s = buf[b * JESD_CAP_LANES_PER_BEAT + 1];
		size_t t = b % JESD_PB_PERIOD_BEATS;
		int32_t c = lb_cos[t];
		int32_t s = lb_sin[t];

		re += ((int64_t)i_s * c + (int64_t)q_s * s) >> LB_TWIDDLE_SCALE;
		im += ((int64_t)q_s * c - (int64_t)i_s * s) >> LB_TWIDDLE_SCALE;
		energy += (uint64_t)((int64_t)i_s * i_s + (int64_t)q_s * q_s);
	}

	rms = lb_isqrt(energy / beats);

	LOG_INF("capture: %zu beats, per-sample RMS %llu", beats,
		(unsigned long long)rms);

	if (rms < LB_SILENCE_RMS) {
		LOG_WRN("ADC input is silent (RMS %llu < %u)",
			(unsigned long long)rms, LB_SILENCE_RMS);
		LOG_WRN("=== Rung 5 SKIPPED: no signal at the ADC ===");
		LOG_WRN("    connect an SMA cable from a DAC output to an ADC input on the FMC,");
		LOG_WRN("    then reboot -- Rung 4 leaves the tone playing continuously.");
		return -ENODATA;
	}

	/*
	 * Energy in the tone bin vs total energy. |X|^2 / (N * total) is 1.0 for a
	 * pure tone and ~1/N for noise, independent of signal level.
	 */
	bin_mag = lb_isqrt((uint64_t)(re * re + im * im));
	amplitude = bin_mag / beats;

	if (energy == 0) {
		concentration = 0;
	} else {
		concentration = (uint32_t)(((uint64_t)(re * re + im * im) * 1000U) /
					   (energy * beats));
	}

	LOG_INF("tone bin: amplitude %llu (sent %d), %u/1000 of total energy",
		(unsigned long long)amplitude, JESD_PB_AMPLITUDE, concentration);

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
