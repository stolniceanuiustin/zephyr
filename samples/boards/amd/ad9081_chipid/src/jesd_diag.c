/*
 * Analog-loopback fault isolation for Rung 5.
 *
 * Rung 5 reports a bare ADC noise floor (RMS 8 out of +/-32768, identical on all
 * four channels) with a cable installed, while every status register that can be
 * read says the datapath is healthy: the deframer reports all four lanes locked,
 * both link cores stay in DATA under traffic, the DMA transfers complete, and
 * cyclic playback is confirmed still running. The ADC is clearly alive -- a dead
 * one gives exact zeros or a stuck constant, not thermal noise -- so something
 * between "the DMA hands samples to the DAC" and "the ADC sees a voltage" is
 * losing the signal, in a way nothing readable admits to.
 *
 * Guessing at that is expensive: each hypothesis costs a rebuild and a reflash.
 * So rather than change one setting and retry, these checks partition the
 * remaining possibilities and report what each one found.
 *
 * What the earlier rounds established, so it is not re-litigated: the balun
 * passes 500 kHz - 9 GHz, so 2 GHz is not a band problem. Every NCO frequency
 * tuning word reads back exactly as requested, so the silence at the swept
 * frequencies is not a failed retune. The TX channel gains read back as the
 * programmed 1024, so the datapath is not muted. And the chip's main-datapath DC
 * test tone returns through the cable at full amplitude, which proves the DAC
 * output stage, balun, cable, ADC, RX DDC, framer, RX link, RX DMA and the TX
 * main NCO all work.
 *
 * That leaves our samples failing somewhere in deframer -> fine DUC -> main DUC,
 * the only stretch the internal tone skips -- and failing *intermittently*: the
 * same verified frequency returned RMS 4567 on one boot and RMS 8 on the next.
 * An intermittent fault is not a wrong register value, so the last two checks
 * measure how often it works and which half of that stretch loses it.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(jesd_diag, LOG_LEVEL_INF);

#include "ad9081.h"
#include "jesd_diag.h"
#include "jesd_loopback.h"
#include "jesd_playback.h"

#include "adi_ad9081.h"
#include "adi_ad9081_hal.h"
#include "adi_ad9081_bf_ad9081.h"

/*
 * The frequency plan in use: TX main NCO up, RX coarse DDC down by the same
 * amount, so an analog loopback returns the tone to the baseband frequency it
 * left at. Both are retunable at runtime with a single call each.
 */
#define DIAG_NCO_HZ_DEFAULT 2000000000LL

/*
 * Sweep points for the TX/RX NCO pair, in MHz. Spread across the ADC's first
 * Nyquist zone (0-2000 MHz) rather than clustered, because the question is the
 * *shape* of the response, not the value at any one point:
 *
 *   - 100 MHz is low enough that essentially nothing in the analog path can be
 *     blamed: well inside the balun's range, far from Nyquist, no aliasing.
 *   - 500 and 1000 MHz fill in the middle so a rolloff is visible as a trend.
 *   - 1968.75 MHz is the configured plan, kept so the sweep always includes the
 *     frequency the rest of the app actually runs at.
 *
 * With the FTW readback confirming each point was programmed, a flat noise floor
 * across all four rules out the whole class of band and alias explanations --
 * which is what happened, so this check has already served its purpose and is
 * retained mainly to detect a regression into frequency dependence.
 */
static const uint32_t diag_sweep_mhz[] = { 100, 500, 1000, 1968 };

/* Recovered-tone concentration (permille) that counts as "the tone came back".
 * Well above the ~13 that broadband noise scores, well below the ~999 of a clean
 * return, so a weak-but-real signal still registers. */
#define DIAG_TONE_FOUND_MIN 200

/* Above this per-sample RMS there is definitely *something* at the ADC input,
 * whether or not it correlates with the tone we sent. */
#define DIAG_SIGNAL_RMS 16

/*
 * Read back the TX fine-DUC channel gain. adi_ad9081_dac_duc_nco_gains_set()
 * wrote 1024 to channels 0-3 at datapath setup, but nothing has ever confirmed
 * the value landed, and a zero gain would produce exactly the symptom under
 * investigation: a flawless link carrying silence. The register is paged per
 * channel, so select each one before reading. 12-bit field, so two bytes.
 */
static void diag_check_tx_gain(adi_ad9081_device_t *dev)
{
	LOG_INF("[1/5] TX fine-DUC channel gain readback (programmed 1024):");

	for (uint8_t ch = 0; ch < 4; ch++) {
		uint8_t raw[2] = { 0, 0 };
		int32_t err;

		err = adi_ad9081_dac_chan_select_set(dev, AD9081_DAC_CH_0 << ch);
		if (err != API_CMS_ERROR_OK) {
			LOG_WRN("  ch%u: channel select failed (%d)", ch, err);
			continue;
		}

		err = adi_ad9081_hal_bf_get(dev, REG_CHNL_GAIN0_ADDR,
					    BF_CHNL_GAIN_INFO, raw, 2);
		if (err != API_CMS_ERROR_OK) {
			LOG_WRN("  ch%u: gain read failed (%d)", ch, err);
			continue;
		}

		uint16_t gain = (uint16_t)raw[0] | ((uint16_t)raw[1] << 8);

		if (gain == 0) {
			LOG_ERR("  ch%u: gain 0 -- the datapath is muted, THIS is the fault",
				ch);
		} else if (gain != 1024) {
			LOG_WRN("  ch%u: gain %u (expected 1024) -- scaled, not muted",
				ch, gain);
		} else {
			LOG_INF("  ch%u: gain %u (as programmed)", ch, gain);
		}
	}
}

/*
 * Retune the TX main NCO and RX coarse DDC to a matched pair, then read both
 * frequency tuning words back.
 *
 * The readback is the point. A sweep that only writes cannot tell "this
 * frequency does not get through the analog path" from "this frequency was
 * never actually programmed" -- both look like silence, and the first reading
 * of this sweep (three silent points and one working one, in no pattern any
 * filter would produce) fits the second explanation at least as well as the
 * first. Comparing the readback against the FTW the requested frequency implies
 * settles it: a matching FTW means the tone really was on that carrier and the
 * silence is physical, while a stale or clamped FTW means the sweep was
 * measuring the same setting repeatedly.
 *
 * Expected FTW = 2^48 * f / f_clk, and for a negative shift the API stores the
 * two's-complement form (2^48 - x), matching adi_ad9081_hal_calc_rx_nco_ftw.
 */
static int diag_retune(adi_ad9081_device_t *dev, int64_t hz)
{
	uint64_t tx_ftw = 0, rx_ftw = 0, mod_a = 0, mod_b = 0;
	uint64_t want_tx, want_rx;
	int32_t err;

	err = adi_ad9081_dac_duc_nco_set(dev, AD9081_DAC_ALL,
					 AD9081_DAC_CH_NONE, hz);
	if (err != API_CMS_ERROR_OK) {
		LOG_WRN("  TX main NCO retune to %lld Hz failed (%d)",
			(long long)hz, err);
		return -EIO;
	}

	err = adi_ad9081_adc_ddc_coarse_nco_set(dev, AD9081_ADC_CDDC_ALL, hz);
	if (err != API_CMS_ERROR_OK) {
		LOG_WRN("  RX coarse DDC retune to %lld Hz failed (%d)",
			(long long)hz, err);
		return -EIO;
	}

	/* Let the retuned datapath settle before capturing through it. */
	k_msleep(2);

	/* What the FTWs should be if the writes landed, computed the same way the
	 * API computes them so a mismatch means the hardware, not our arithmetic. */
	(void)adi_ad9081_hal_calc_tx_nco_ftw(dev, dev->dev_info.dac_freq_hz, hz,
					     &want_tx);
	(void)adi_ad9081_hal_calc_rx_nco_ftw(dev, dev->dev_info.adc_freq_hz, hz,
					     &want_rx);

	err = adi_ad9081_dac_duc_main_nco_ftw_get(dev, AD9081_DAC_0, &tx_ftw,
						  &mod_a, &mod_b);
	if (err != API_CMS_ERROR_OK) {
		LOG_WRN("  TX FTW readback failed (%d)", err);
	}

	err = adi_ad9081_adc_ddc_coarse_nco_ftw_get(dev, AD9081_ADC_CDDC_0,
						    &rx_ftw, &mod_a, &mod_b);
	if (err != API_CMS_ERROR_OK) {
		LOG_WRN("  RX FTW readback failed (%d)", err);
	}

	LOG_INF("    FTW tx 0x%012llx (want 0x%012llx)%s, rx 0x%012llx (want 0x%012llx)%s",
		(unsigned long long)tx_ftw, (unsigned long long)want_tx,
		(tx_ftw == want_tx) ? "" : " MISMATCH",
		(unsigned long long)rx_ftw, (unsigned long long)want_rx,
		(rx_ftw == want_rx) ? "" : " MISMATCH");

	return 0;
}

/*
 * Sweep the matched NCO pair and measure the returning tone at each point. The
 * baseband tone that Rung 4 is playing does not move -- only the RF frequency it
 * is carried on does -- so the correlator stays valid at every point without
 * touching the playback buffer.
 */
static bool diag_sweep(adi_ad9081_device_t *dev)
{
	bool found_any = false;

	LOG_INF("[2/5] NCO frequency sweep (TX main + RX coarse together):");
	LOG_INF("  baseband tone stays at -%u MHz; only the RF carrier moves",
		JESD_PB_TONE_HZ / 1000000U);

	for (size_t i = 0; i < ARRAY_SIZE(diag_sweep_mhz); i++) {
		int64_t hz = (int64_t)diag_sweep_mhz[i] * 1000000LL;
		struct jesd_loopback_meas m;
		int ret;

		if (diag_retune(dev, hz) != 0) {
			continue;
		}

		ret = jesd_loopback_measure(&m);
		if (ret) {
			LOG_WRN("  %4u MHz: capture failed (%d)",
				diag_sweep_mhz[i], ret);
			continue;
		}

		LOG_INF("  %4u MHz: RMS %llu, lanes %llu/%llu/%llu/%llu/%llu/%llu/%llu/%llu",
			diag_sweep_mhz[i], (unsigned long long)m.rms,
			(unsigned long long)m.lane_rms[0],
			(unsigned long long)m.lane_rms[1],
			(unsigned long long)m.lane_rms[2],
			(unsigned long long)m.lane_rms[3],
			(unsigned long long)m.lane_rms[4],
			(unsigned long long)m.lane_rms[5],
			(unsigned long long)m.lane_rms[6],
			(unsigned long long)m.lane_rms[7]);
		LOG_INF("            tone: interleaved %u/1000 (amp %llu), split %u/1000 (amp %llu)",
			m.concentration, (unsigned long long)m.amplitude,
			m.concentration_split,
			(unsigned long long)m.amplitude_split);

		uint32_t best = MAX(m.concentration, m.concentration_split);

		if (best >= DIAG_TONE_FOUND_MIN && m.rms >= DIAG_SIGNAL_RMS) {
			LOG_INF("         ^^ the tone came back at this frequency");
			found_any = true;
		}
	}

	return found_any;
}

/*
 * Drive the DAC from its own internal calibration DC input instead of from the
 * JESD stream. The main-datapath NCO then upconverts that DC to a tone at the
 * NCO frequency, so a signal appears at the DAC output having touched none of
 * our DMA buffer, the TPL core, the serial link or the deframer.
 *
 * This is the decisive split. If the internal tone returns through the cable
 * while ours does not, everything analog works and the fault is in how we
 * deliver samples to the DAC core. If neither returns, our transmit datapath is
 * exonerated and the problem is the DAC output stage or the board's analog path
 * between the two SMAs -- which is where test equipment becomes unavoidable.
 */
static bool diag_internal_tone(adi_ad9081_device_t *dev)
{
	struct jesd_loopback_meas m;
	int32_t err;
	bool present = false;
	int ret;

	LOG_INF("[3/5] chip-internal DAC test tone (bypasses DMA + link + deframer):");

	if (diag_retune(dev, DIAG_NCO_HZ_DEFAULT) != 0) {
		LOG_WRN("  could not restore the NCO pair; skipping");
		return false;
	}

	err = adi_ad9081_dac_duc_main_dc_test_tone_offset_set(
		dev, AD9081_DAC_ALL, 0x4000);
	if (err != API_CMS_ERROR_OK) {
		LOG_WRN("  test tone offset set failed (%d)", err);
		return false;
	}

	err = adi_ad9081_dac_duc_main_dc_test_tone_en_set(dev, AD9081_DAC_ALL, 1);
	if (err != API_CMS_ERROR_OK) {
		LOG_WRN("  test tone enable failed (%d)", err);
		return false;
	}

	k_msleep(2);

	ret = jesd_loopback_measure(&m);
	if (ret == 0) {
		LOG_INF("  internal tone at %lld MHz: RMS %llu, lanes %llu/%llu/%llu/%llu/%llu/%llu/%llu/%llu",
			(long long)(DIAG_NCO_HZ_DEFAULT / 1000000LL),
			(unsigned long long)m.rms,
			(unsigned long long)m.lane_rms[0],
			(unsigned long long)m.lane_rms[1],
			(unsigned long long)m.lane_rms[2],
			(unsigned long long)m.lane_rms[3],
			(unsigned long long)m.lane_rms[4],
			(unsigned long long)m.lane_rms[5],
			(unsigned long long)m.lane_rms[6],
			(unsigned long long)m.lane_rms[7]);

		/*
		 * Judge on RMS, not on the correlator. The internal tone is a
		 * DC offset upconverted by the NCO, so it lands at the NCO
		 * frequency itself -- which the RX DDC shifts straight to
		 * baseband DC, not to the bin the correlator watches. Energy
		 * appearing at all is the signal here.
		 */
		present = (m.rms >= DIAG_SIGNAL_RMS);
		if (present) {
			LOG_INF("  the internal tone DID come back through the cable");
		} else {
			LOG_WRN("  the internal tone did NOT come back either");
		}
	} else {
		LOG_WRN("  capture failed (%d)", ret);
	}

	/* Always turn it back off -- leaving it on would corrupt every later
	 * measurement with a signal the datapath never sent. */
	err = adi_ad9081_dac_duc_main_dc_test_tone_en_set(dev, AD9081_DAC_ALL, 0);
	if (err != API_CMS_ERROR_OK) {
		LOG_WRN("  could not disable the test tone (%d)", err);
	}

	return present;
}

/*
 * Repeat one measurement of our own tone at the configured frequency.
 *
 * Across two boots the identical 1968 MHz point returned RMS 4567 once and RMS 8
 * the next time, with the frequency tuning words verified identical and correct
 * both times. That rules out every static misconfiguration -- a wrong register
 * value does not work once and then stop -- so the useful question is no longer
 * "which setting is wrong" but "how often does it work". A tone that appears in a
 * minority of captures points at something that has to line up in time rather
 * than in configuration: deframer elastic-buffer or LMFC alignment, or SYSREF
 * timing, none of which the lane-locked status bits report on.
 */
#define DIAG_REPEATS 8

static void diag_repeatability(void)
{
	uint32_t hits = 0;
	uint64_t best_rms = 0;

	LOG_INF("[4/5] repeatability of our own tone (%u captures at the same setting):",
		DIAG_REPEATS);

	for (uint32_t i = 0; i < DIAG_REPEATS; i++) {
		struct jesd_loopback_meas m;

		if (jesd_loopback_measure(&m) != 0) {
			continue;
		}

		if (m.rms > best_rms) {
			best_rms = m.rms;
		}
		if (m.rms >= DIAG_SIGNAL_RMS) {
			hits++;
		}

		LOG_INF("  #%u: RMS %llu, tone %u/1000 (lane0 %llu, lane1 %llu)",
			i, (unsigned long long)m.rms, m.concentration,
			(unsigned long long)m.lane_rms[0],
			(unsigned long long)m.lane_rms[1]);
	}

	LOG_INF("  %u/%u captures saw signal, best RMS %llu", hits,
		DIAG_REPEATS, (unsigned long long)best_rms);

	if (hits == 0) {
		LOG_INF("  never, this boot -- yet it worked on a previous boot at this");
		LOG_INF("  same verified frequency, so the fault varies per bring-up");
		LOG_INF("  (alignment/timing), not per setting.");
	} else if (hits < DIAG_REPEATS) {
		LOG_WRN("  INTERMITTENT within a single boot -- the datapath is not");
		LOG_WRN("  deterministically delivering samples to the DAC.");
	} else {
		LOG_INF("  consistently present this boot.");
	}
}

/*
 * Enable the DC test tone at the *channel* (fine DUC) datapath rather than the
 * main one. The main-datapath tone used in check 3 injects downstream of both the
 * deframer and the fine DUC, so its success narrows the fault only to "somewhere
 * before the main DUC". This injects one stage earlier -- downstream of the
 * deframer, upstream of the main DUC -- splitting that span in half:
 *
 *   returns  -> the fine DUC and everything after it are fine, so the deframer
 *               is not handing our samples on despite reporting locked lanes
 *   silent   -> the fine DUC stage itself is where the signal dies, even though
 *               its gain reads back as the programmed 1024
 */
static void diag_channel_tone(adi_ad9081_device_t *dev)
{
	struct jesd_loopback_meas m;
	int32_t err;

	LOG_INF("[5/5] fine-DUC (channel) DC test tone -- one stage earlier than [3]:");

	err = adi_ad9081_dac_dc_test_tone_offset_set(dev, AD9081_DAC_CH_0, 0x4000);
	if (err != API_CMS_ERROR_OK) {
		LOG_WRN("  channel test tone offset failed (%d)", err);
		return;
	}

	err = adi_ad9081_dac_dc_test_tone_en_set(dev, AD9081_DAC_CH_0, 1);
	if (err != API_CMS_ERROR_OK) {
		LOG_WRN("  channel test tone enable failed (%d)", err);
		return;
	}

	k_msleep(2);

	if (jesd_loopback_measure(&m) == 0) {
		LOG_INF("  RMS %llu, lanes %llu/%llu/%llu/%llu/%llu/%llu/%llu/%llu",
			(unsigned long long)m.rms,
			(unsigned long long)m.lane_rms[0],
			(unsigned long long)m.lane_rms[1],
			(unsigned long long)m.lane_rms[2],
			(unsigned long long)m.lane_rms[3],
			(unsigned long long)m.lane_rms[4],
			(unsigned long long)m.lane_rms[5],
			(unsigned long long)m.lane_rms[6],
			(unsigned long long)m.lane_rms[7]);

		if (m.rms >= DIAG_SIGNAL_RMS) {
			LOG_INF("  the fine DUC reaches the DAC: the fault is the");
			LOG_INF("  deframer -> fine DUC handoff of our JESD samples.");
		} else {
			LOG_WRN("  silent: the fine DUC stage is where the signal dies,");
			LOG_WRN("  despite its gain reading back as the programmed 1024.");
		}
	}

	err = adi_ad9081_dac_dc_test_tone_en_set(dev, AD9081_DAC_CH_0, 0);
	if (err != API_CMS_ERROR_OK) {
		LOG_WRN("  could not disable the channel test tone (%d)", err);
	}
}

int jesd_diag_loopback(void)
{
	adi_ad9081_device_t *dev = ad9081_get_device();
	bool swept_ok, internal_ok;

	if (dev == NULL) {
		LOG_ERR("AD9081 device not initialised");
		return -ENODEV;
	}

	LOG_INF("--- Rung 5 fault isolation ---");
	LOG_INF("link reports healthy but the ADC sees only its noise floor;");
	LOG_INF("splitting that into cases rather than guessing at settings.");

	diag_check_tx_gain(dev);
	swept_ok = diag_sweep(dev);
	internal_ok = diag_internal_tone(dev);
	diag_channel_tone(dev);

	/* Back to the configured plan before judging repeatability, so the repeats
	 * measure the frequency the rest of the app actually runs at. */
	(void)diag_retune(dev, DIAG_NCO_HZ_DEFAULT);
	diag_repeatability();

	/* Restore the configured frequency plan whatever happened, so the board
	 * is left in the state the rest of the app documents. */
	if (diag_retune(dev, DIAG_NCO_HZ_DEFAULT) != 0) {
		LOG_WRN("could not restore the %lld MHz NCO pair",
			(long long)(DIAG_NCO_HZ_DEFAULT / 1000000LL));
	}

	LOG_INF("--- conclusion ---");
	if (swept_ok) {
		LOG_INF("the tone returns at one or more frequencies: the full chain");
		LOG_INF("  works -- DDR -> DMA -> DAC -> cable -> ADC -> DMA -> DDR.");
		LOG_INF("  Check the FTW readbacks above before reading the silent");
		LOG_INF("  points as a band limit: a mismatch there means those");
		LOG_INF("  frequencies were never programmed, so their silence says");
		LOG_INF("  nothing about the analog path.");
	} else if (internal_ok) {
		LOG_INF("the chip's own tone reaches the ADC but ours does not:");
		LOG_INF("  the DAC output stage, balun, cable, ADC, RX DDC, framer,");
		LOG_INF("  RX link and RX DMA all work -- and so does the TX main NCO,");
		LOG_INF("  since it is what upconverts that DC offset into a tone.");
		LOG_INF("  With every FTW verified correct, the fault is confined to");
		LOG_INF("  the stretch our samples take and the internal tone skips:");
		LOG_INF("  deframer -> fine DUC -> main DUC. See [5] for which half.");
	} else {
		LOG_INF("nothing reaches the ADC at any frequency, not even a tone");
		LOG_INF("  generated inside the chip downstream of our entire");
		LOG_INF("  datapath. That exonerates the DMA, link and deframer and");
		LOG_INF("  points at the DAC output stage or the board path between");
		LOG_INF("  the two SMAs. Confirming which needs a scope or spectrum");
		LOG_INF("  analyser on the DAC output -- no register can see it.");
	}

	return 0;
}
