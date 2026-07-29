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
 * So rather than change one setting and retry, this runs three checks that
 * partition the remaining possibilities, and reports what each one found.
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
 *   - 1968.75 MHz (the current setting) sits near the Nyquist edge and is the
 *     point that is known to fail, kept so the sweep includes the failing case
 *     for comparison rather than only working ones.
 *
 * A flat noise floor across all four says the fault is not frequency-dependent
 * at all, which rules out the whole class of band/alias explanations in one boot
 * and points at the datapath or the DAC output instead.
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
	LOG_INF("[1/3] TX fine-DUC channel gain readback (programmed 1024):");

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

/* Retune the TX main NCO and RX coarse DDC to a matched pair. */
static int diag_retune(adi_ad9081_device_t *dev, int64_t hz)
{
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

	LOG_INF("[2/3] NCO frequency sweep (TX main + RX coarse together):");
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

		LOG_INF("  %4u MHz: RMS %llu (ch %llu/%llu/%llu/%llu), tone %u/1000, amp %llu",
			diag_sweep_mhz[i], (unsigned long long)m.rms,
			(unsigned long long)m.ch_rms[0],
			(unsigned long long)m.ch_rms[1],
			(unsigned long long)m.ch_rms[2],
			(unsigned long long)m.ch_rms[3],
			m.concentration, (unsigned long long)m.amplitude);

		if (m.concentration >= DIAG_TONE_FOUND_MIN &&
		    m.rms >= DIAG_SIGNAL_RMS) {
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

	LOG_INF("[3/3] chip-internal DAC test tone (bypasses DMA + link + deframer):");

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
		LOG_INF("  internal tone at %lld MHz: RMS %llu (ch %llu/%llu/%llu/%llu)",
			(long long)(DIAG_NCO_HZ_DEFAULT / 1000000LL),
			(unsigned long long)m.rms,
			(unsigned long long)m.ch_rms[0],
			(unsigned long long)m.ch_rms[1],
			(unsigned long long)m.ch_rms[2],
			(unsigned long long)m.ch_rms[3]);

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

	/* Restore the configured frequency plan whatever happened, so the board
	 * is left in the state the rest of the app documents. */
	if (diag_retune(dev, DIAG_NCO_HZ_DEFAULT) != 0) {
		LOG_WRN("could not restore the %lld MHz NCO pair",
			(long long)(DIAG_NCO_HZ_DEFAULT / 1000000LL));
	}

	LOG_INF("--- conclusion ---");
	if (swept_ok) {
		LOG_INF("the tone returns at some frequencies but not others:");
		LOG_INF("  the datapath works and the fault is frequency-dependent");
		LOG_INF("  (band limit or aliasing). Move the plan to a frequency");
		LOG_INF("  that worked above and Rung 5 should pass.");
	} else if (internal_ok) {
		LOG_INF("the chip's own tone reaches the ADC but ours never does:");
		LOG_INF("  the DAC output and the whole analog path are fine, so the");
		LOG_INF("  fault is upstream -- our samples are not reaching the DAC");
		LOG_INF("  core despite the deframer reporting locked lanes.");
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
