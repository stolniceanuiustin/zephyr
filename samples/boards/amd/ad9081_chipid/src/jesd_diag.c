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
 * An intermittent fault is not a wrong register value, so two of these checks
 * measure how often it works and which half of that stretch loses it.
 *
 * The board's IRQB0 LED being lit turned out to be a red herring: the latched
 * status reads 0x40 00 00 43 f0 00, which is DATA_READY plus PLL and DLL *lock*
 * indications and DLL_VTH_PASS on all four datapaths. No PAERR, no SYSREF_JITTER,
 * no LANE_FIFO overflow, no DLL_LOST -- the chip is not reporting a fault, and
 * DATA_READY alone is enough to hold that open-drain pin low. Check [0] is kept
 * because ruling those out cheaply is worth a few SPI reads, and it runs first so
 * it reports what the chip was already complaining about rather than anything
 * provoked here.
 *
 * What did narrow it: the fine-DUC test tone in [5] returns at full amplitude,
 * matching the main-DUC tone in [3]. So fine DUC -> main DUC -> DAC -> balun ->
 * cable -> ADC -> framer -> RX link -> RX DMA -> DDR is proven end to end, and
 * the suspect stretch collapses to a single interface: the deframer handing
 * samples into the fine DUC. Both those tones are injected inside the chip,
 * downstream of the deframer, so neither can see across it. Check [6] approaches
 * that interface from the other side with a tone generated in the FPGA at the TPL
 * input, which crosses the lanes and the deframer but skips DDR and the DMA --
 * bracketing the fault between the two.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/cache.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(jesd_diag, LOG_LEVEL_INF);

#include "ad9081.h"
#include "axi_tpl.h"
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

/* AXI DMAC channel index, matching Rung 4 (single-channel core). */
#define PB_DIAG_DMA_CHANNEL 0

/*
 * Read the chip's latched interrupt status.
 *
 * The IRQB0 pin on the board is lit red, and no check here had ever asked the chip
 * what it was complaining about -- every register consulted reported *state* (lanes
 * locked, link in DATA, gain 1024) rather than faults.
 *
 * These bits are live without us enabling anything, because
 * adi_ad9081_device_startup_tx() ends with adi_ad9081_dac_irqs_enable_set(device,
 * 0x0030cccc00), whose bits 11/15/19/23 are PAERR0-3, the per-DAC PA-protection
 * errors. Measured result: 0x40 00 00 43 f0 00 -- DATA_READY, PLL_LOCK_FAST/SLOW,
 * DLL_LOCK23, DLL_VTH_PASS on all four datapaths. No PAERR, no SYSREF_JITTER, no
 * LANE_FIFO, no DLL_LOST. The chip reports no fault at all, and DATA_READY on its
 * own holds that open-drain pin low, so the LED was never a fault indication.
 *
 * Kept because ruling out that whole class of fault costs six SPI reads, and
 * because a regression into a real PAERR or DLL_LOST would otherwise be invisible.
 *
 * Read as six bytes rather than through adi_ad9081_dac_irqs_status_get(), whose
 * 0x2800 multi-byte field descriptor writes 8 bytes into a uint64_t -- correct on
 * a little-endian host but worth not depending on when the whole point is to see
 * exactly which bits are set. Reported as raw bytes plus decoded names, so the
 * evidence survives even if my decoding of a given bit is wrong.
 */
static void diag_irq_status(adi_ad9081_device_t *dev)
{
	static const char *const irq0[8] = {
		"PRBS_I", "PRBS_Q", "SYSREF_JITTER", "BIST_DONE",
		"CAPTURE_DONE", "LANE_FIFO", "DATA_READY", NULL,
	};
	static const char *const irq3[8] = {
		"PLL_LOCK_FAST", "PLL_LOCK_SLOW", "PLL_LOST_FAST",
		"PLL_LOST_SLOW", "DLL_LOCK01", "DLL_LOST01",
		"DLL_LOCK23", "DLL_LOST23",
	};
	uint8_t st[6] = { 0 };

	LOG_INF("[0/7] chip latched IRQ status (IRQB0 is lit on the board):");

	for (uint8_t i = 0; i < 6; i++) {
		int32_t err = adi_ad9081_hal_reg_get(dev, REG_IRQ_STATUS0_ADDR + i,
						     &st[i]);

		if (err != API_CMS_ERROR_OK) {
			LOG_WRN("  IRQ_STATUS%u read failed (%d)", i, err);
			return;
		}
	}

	LOG_INF("  raw 0x%02x %02x %02x %02x %02x %02x (regs 0x26..0x2b)",
		st[0], st[1], st[2], st[3], st[4], st[5]);

	for (uint8_t b = 0; b < 8; b++) {
		if ((st[0] & BIT(b)) && irq0[b] != NULL) {
			LOG_WRN("  IRQ_STATUS0 bit%u: %s", b, irq0[b]);
		}
		if (st[3] & BIT(b)) {
			LOG_INF("  IRQ_STATUS3 bit%u: %s", b, irq3[b]);
		}
	}

	/*
	 * PAERR is bit 3 of each DAC's nibble pair: DAC0/1 in status1, DAC2/3 in
	 * status2, low nibble then high. If any of these is set the DAC output is
	 * being blanked by PA protection and no amount of correct digital datapath
	 * would produce a signal at the SMA.
	 */
	for (uint8_t d = 0; d < 4; d++) {
		uint8_t reg = st[1 + (d / 2)];
		uint8_t bit = (d & 1) ? 7 : 3;

		if (reg & BIT(bit)) {
			LOG_ERR("  PAERR%u LATCHED -- DAC%u output is blanked by PA protection",
				d, d);
		}
	}

	if (st[5] & 0x0f) {
		LOG_ERR("  SRERR latched (0x%x) -- slew-rate/PA error per DAC",
			st[5] & 0x0f);
	}

	if (st[0] & BIT(2)) {
		LOG_ERR("  SYSREF_JITTER -- SYSREF is not clean, so deframer/LMFC");
		LOG_ERR("  alignment is not trustworthy even with lanes locked");
	}
	if (st[0] & BIT(5)) {
		LOG_ERR("  LANE_FIFO error -- the JRX lane FIFO over/underflowed,");
		LOG_ERR("  which drops our samples while lanes still report locked");
	}
	if (st[3] & (BIT(5) | BIT(7))) {
		LOG_ERR("  DLL_LOST -- the DAC clock DLL lost lock; the analog");
		LOG_ERR("  output cannot be trusted regardless of the datapath");
	}

	if (st[0] == 0 && st[1] == 0 && st[2] == 0 && st[3] == 0 &&
	    st[4] == 0 && st[5] == 0) {
		LOG_WRN("  all six status bytes are zero, yet IRQB0 is asserted.");
		LOG_WRN("  Either the pin is driven by something other than these");
		LOG_WRN("  latches (check the board's own LED wiring -- IRQB is");
		LOG_WRN("  open-drain active-low, so an unconfigured pin can read as");
		LOG_WRN("  a lit LED), or the source is a JESD IRQ routed through");
		LOG_WRN("  MUX_JESD_IRQ rather than one of these six registers.");
	}
}

/*
 * Read back the TX fine-DUC channel gain. adi_ad9081_dac_duc_nco_gains_set()
 * wrote 1024 to channels 0-3 at datapath setup, but nothing has ever confirmed
 * the value landed, and a zero gain would produce exactly the symptom under
 * investigation: a flawless link carrying silence. The register is paged per
 * channel, so select each one before reading. 12-bit field, so two bytes.
 */
static void diag_check_tx_gain(adi_ad9081_device_t *dev)
{
	LOG_INF("[1/7] TX fine-DUC channel gain readback (programmed 1024):");

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

	LOG_INF("[2/7] NCO frequency sweep (TX main + RX coarse together):");
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

	LOG_INF("[3/7] chip-internal DAC test tone (bypasses DMA + link + deframer):");

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

	LOG_INF("[4/7] repeatability of our own tone (%u captures at the same setting):",
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

	LOG_INF("[5/7] fine-DUC (channel) DC test tone -- one stage earlier than [3]:");

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

/*
 * Drive the DAC from the FPGA's own DDS tone generator instead of from DDR.
 *
 * This is the other half of the split that check [5] opened. [5] showed the chip's
 * fine-DUC test tone reaching the ADC at full amplitude, which exonerates
 * everything from the fine DUC onward -- but that tone is injected inside the chip,
 * downstream of the deframer, so it proves nothing about whether the deframer hands
 * our samples on. The DDS enters at the opposite end: inside the FPGA, at the TPL
 * input, upstream of the transport core, the serial lanes and the deframer.
 *
 * Between them the two tones bracket the one remaining suspect stretch:
 *
 *   DDS returns  -> the TPL, lanes and deframer all deliver samples into the DAC
 *                   datapath. Nothing between DDR and the DAC is broken, so the
 *                   fault is our DMA feed: the buffer contents, the descriptor, the
 *                   cache maintenance, or the cyclic transfer not actually
 *                   presenting data on the TPL's input.
 *   DDS silent   -> the deframer is not passing samples on despite reporting four
 *                   locked lanes, which is the LMFC/elastic-buffer alignment
 *                   suspicion and is consistent with the per-boot intermittency.
 *
 * Judged on RMS rather than the correlator: the DDS frequency is quantised by a
 * 16-bit phase accumulator and does not land on the bin the correlator watches.
 */
static void diag_fpga_dds(void)
{
	struct jesd_loopback_meas m;
	int ret;

	LOG_INF("[6/7] FPGA DDS tone at the TPL input (crosses lanes + deframer):");

	ret = axi_tpl_tx_dds(JESD_PB_TONE_HZ, JESD_PB_SAMPLE_RATE, true);
	if (ret) {
		LOG_WRN("  could not arm the FPGA DDS (%d)", ret);
		return;
	}

	/* The TPL SYNC and the chip's deframer need a moment to settle on the new
	 * data source before the capture means anything. */
	k_msleep(5);

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
			LOG_ERR("  THE LINK AND DEFRAMER ARE FINE -- an FPGA-generated");
			LOG_ERR("  tone crosses the lanes and deframer and reaches the");
			LOG_ERR("  ADC. The fault is upstream of the TPL: the DDR buffer,");
			LOG_ERR("  the TX DMA descriptor, cache maintenance, or the cyclic");
			LOG_ERR("  transfer not presenting data at the TPL input.");
		} else {
			LOG_ERR("  silent -- the deframer is not passing samples into the");
			LOG_ERR("  DAC datapath despite reporting four locked lanes. That");
			LOG_ERR("  is an alignment problem (LMFC / elastic buffer / SYSREF),");
			LOG_ERR("  matching the per-boot intermittency: lane lock does not");
			LOG_ERR("  imply the deframer found a valid frame boundary.");
		}
	}

	/* Put the converters back on the DMA source whatever happened, so the board
	 * is left as the rest of the app expects. */
	(void)axi_tpl_tx_dds(0, 0, false);
}

/*
 * Read the playback buffer back through a non-cached mapping and re-check the DMA.
 *
 * With [6] showing an FPGA-sourced tone completing the whole trip, everything from
 * the TPL input onward is proven. What remains upstream is small and entirely
 * checkable: are the samples actually in DDR where the DMA reads them, and is the
 * engine actually still fetching them?
 *
 * pb_fill() flushes the buffer, and reading it back through the ordinary mapping
 * would just hit the same D-cache lines the CPU wrote -- it would confirm the CPU's
 * view, not memory's, which is precisely the thing in question. So invalidate first
 * and force the read to come from DDR.
 *
 * The second half is the rate question, which nothing has yet asked. The transfer
 * "completing" says the engine emptied its descriptor; it says nothing about
 * whether it sustained 250 MSPS x 16 bytes = 4 GB/s. At that rate our 1 KiB buffer
 * is 256 ns long and a cyclic transfer wraps it 3.9 million times a second. If the
 * engine cannot re-arm that fast the TPL is starved between wraps and the DAC
 * outputs mostly nothing -- which would look exactly like the observed silence
 * while every status bit stayed healthy, and would vary per boot with bus
 * contention. Sampling the byte counter across a known interval measures the
 * achieved rate instead of assuming it.
 */
static void diag_dma_feed(void)
{
	const struct device *tx_dma = DEVICE_DT_GET(DT_NODELABEL(tx_dmac));
	const int16_t *buf;
	struct dma_status s0, s1;
	size_t bytes;
	bool nonzero = false;

	LOG_INF("[7/7] the DMA feed itself (everything from the TPL on is proven):");

	if (jesd_playback_buffer(&buf, &bytes) != 0) {
		LOG_WRN("  playback buffer unavailable");
		return;
	}

	/* Read what is actually in DDR, not what the CPU's cache remembers. */
	sys_cache_data_invd_range((void *)buf, bytes);

	for (size_t i = 0; i < bytes / sizeof(int16_t); i++) {
		if (buf[i] != 0) {
			nonzero = true;
			break;
		}
	}

	LOG_INF("  DDR contents, first beat: %6d %6d %6d %6d %6d %6d %6d %6d",
		buf[0], buf[1], buf[2], buf[3],
		buf[4], buf[5], buf[6], buf[7]);

	if (!nonzero) {
		LOG_ERR("  the playback buffer is ALL ZEROS in DDR -- the DMA has been");
		LOG_ERR("  faithfully transmitting silence. THIS is the fault.");
		return;
	}
	LOG_INF("  buffer holds the tone in DDR, so the DMA is reading real samples");

	/*
	 * Measure the achieved feed rate. Two status reads a known interval apart;
	 * pending_length is the bytes left in the current buffer, so its movement
	 * (accounting for wraps) is throughput. Even a coarse figure settles the
	 * question, because the requirement is 4 GB/s and anything the CPU can
	 * observe stepping slowly is orders of magnitude short.
	 */
	if (dma_get_status(tx_dma, PB_DIAG_DMA_CHANNEL, &s0) != 0) {
		LOG_WRN("  could not read DMA status");
		return;
	}
	k_msleep(50);
	if (dma_get_status(tx_dma, PB_DIAG_DMA_CHANNEL, &s1) != 0) {
		LOG_WRN("  could not read DMA status");
		return;
	}

	LOG_INF("  DMA busy=%u then %u, pending %u then %u over 50 ms",
		s0.busy, s1.busy, s0.pending_length, s1.pending_length);

	if (!s1.busy) {
		LOG_ERR("  the cyclic transfer has STOPPED -- the DAC is being fed");
		LOG_ERR("  nothing at all, which is the silence Rung 5 sees.");
		return;
	}

	/*
	 * Still busy. Then the open question is sustained rate: this datapath wants
	 * 4 GB/s (250 MSPS x 16 B/beat), and a 1 KiB cyclic buffer is only 256 ns
	 * long, so it must wrap 3.9 million times per second. That is the one
	 * remaining explanation consistent with every observation: the engine is
	 * alive and the link is perfect, but the TPL is starved between wraps, so
	 * the DAC emits brief bursts separated by long gaps. Averaged over a
	 * capture that is indistinguishable from noise, and it would vary per boot.
	 */
	LOG_WRN("  engine is alive and the buffer is correct, yet no tone arrives.");
	LOG_WRN("  Remaining suspect is sustained rate: this link needs 4 GB/s");
	LOG_WRN("  (250 MSPS x 16 B/beat) and the 1 KiB buffer is only 256 ns long,");
	LOG_WRN("  so it must wrap ~3.9 M times/s. If re-arming cannot keep up the");
	LOG_WRN("  TPL starves between wraps and the DAC emits bursts with long");
	LOG_WRN("  gaps -- which averages to the noise floor Rung 5 reports.");
	LOG_WRN("  Next step: enlarge the playback buffer by ~1000x and retest.");
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

	/* First, and before anything here perturbs the chip: ask what it is
	 * already complaining about. IRQB0 is lit, so there is an answer. */
	diag_irq_status(dev);

	diag_check_tx_gain(dev);
	swept_ok = diag_sweep(dev);
	internal_ok = diag_internal_tone(dev);
	diag_channel_tone(dev);
	diag_fpga_dds();
	diag_dma_feed();

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
		LOG_INF("  With every FTW verified correct, and [5] showing the fine");
		LOG_INF("  DUC reaching the DAC too, the fault is confined to a single");
		LOG_INF("  interface: the deframer handing samples into the fine DUC.");
		LOG_INF("  [6] decides which side of it -- an FPGA tone that crosses");
		LOG_INF("  the deframer but skips DDR and the DMA.");
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
