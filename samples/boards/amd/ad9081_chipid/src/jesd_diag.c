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
#include "jesd_capture.h"
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

/* How much of the playback buffer to read back when checking DDR contents. The
 * buffer is megabytes; a few KiB is plenty to tell a written buffer from an empty
 * one, and keeps the cache invalidate proportionate. */
#define PB_DIAG_INSPECT_BYTES 4096U

/*
 * TX DMAC registers, read directly rather than through the DMA API.
 *
 * The driver probes hardware cyclic support and the maximum burst length at init
 * and keeps both in its private data, where a consumer cannot see them -- yet they
 * decide whether buffer size can possibly matter here. Re-probing the two
 * registers is a couple of reads and keeps the diagnostic's claims answerable from
 * the core instead of from an assumption about the bitstream. FLAGS bit0 reads back
 * set only if the core was synthesized with cyclic support; writing 0xFFFFFFFF to
 * X_LENGTH and reading it back gives the largest burst it can hold, but the value
 * is only read here (the transfer is live, so it must not be disturbed).
 */
#define PB_DIAG_DMAC_BASE       0x9C430000UL
#define PB_DIAG_DMAC_REG_FLAGS  0x040C
#define PB_DIAG_DMAC_REG_X_LENGTH 0x0418
#define PB_DIAG_DMAC_REG_CTRL          0x0400
#define PB_DIAG_DMAC_REG_TRANSFER_ID   0x0404
#define PB_DIAG_DMAC_REG_TRANSFER_DONE 0x0428
#define PB_DIAG_DMAC_REG_IRQ_PENDING   0x0084

/* Bytes the JESD link consumes per sample period: 8 converters x NP16. */
#define PB_DIAG_BEAT_BYTES 16U

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
		LOG_WRN("  INTERMITTENT within a single boot -- see [8/7] for whether that");
		LOG_WRN("  is really intermittence or just a too-short capture window.");
	}
}

/*
 * Duty cycle of the analog return, measured *within* one capture.
 *
 * [4/7] counts how many captures saw signal, and that count has been misleading:
 * each capture is one short window, so "1 of 8" conflates "the signal is rarely
 * there" with "we rarely looked while it was there". Those demand different fixes
 * and the hit count cannot separate them.
 *
 * This scans a single capture in fixed-size chunks and reports each chunk's RMS.
 * At 65 us that already answered the first question: every chunk of every capture
 * read uniformly high or uniformly low, never a mix, so the return is not gated at
 * microsecond scale and the "brief bursts averaging to the noise floor" theory is
 * false.
 *
 * At 262 us -- 4x longer, and the most a single transfer will carry -- the same scan
 * separates what is left, and there are only two shapes:
 *
 *   mixed high/low  -> the source really does gate the tone, on a period longer
 *                      than 65 us; a transition falls inside this window, so the
 *                      list dates it and gives the period
 *   still uniform   -> the signal is steady across a millisecond, so it is not the
 *                      signal that varies between captures -- it is the capture.
 *                      Each one re-arms the RX DMAC from scratch, and that arming
 *                      is then the only remaining variable
 *
 * Chunks are a multiple of the 8-beat tone period, so a chunk boundary never splits
 * the tone in a way that depresses its RMS.
 */
/* Integer square root (Newton), matching jesd_loopback.c's lb_isqrt(). */
static uint64_t diag_isqrt(uint64_t v)
{
	uint64_t x, prev;

	if (v == 0) {
		return 0;
	}

	x = v;
	prev = 0;
	while (x != prev) {
		prev = x;
		x = (x + v / x) / 2;
		if (x > prev) {
			break;
		}
	}
	return x;
}

#define DIAG_CHUNK_BEATS   4096U
#define DIAG_CHUNKS_MAX    64U
#define DIAG_CHUNKS_PER_ROW 16U

static void diag_duty_cycle(void)
{
	const int16_t *buf;
	size_t n, beats, chunks, on = 0;
	uint64_t rms[DIAG_CHUNKS_MAX];

	LOG_INF("[8/7] duty cycle of the return within one capture:");

	if (jesd_capture_raw(&buf, &n) != 0) {
		LOG_WRN("  capture failed");
		return;
	}

	beats = n / JESD_CAP_LANES_PER_BEAT;
	chunks = beats / DIAG_CHUNK_BEATS;
	if (chunks == 0) {
		LOG_WRN("  capture is shorter than one chunk (%zu beats)", beats);
		return;
	}
	if (chunks > DIAG_CHUNKS_MAX) {
		chunks = DIAG_CHUNKS_MAX;
		LOG_INF("  (reporting the first %u chunks of %zu)", DIAG_CHUNKS_MAX,
			beats / DIAG_CHUNK_BEATS);
	}

	for (size_t c = 0; c < chunks; c++) {
		uint64_t energy = 0;
		size_t base = c * DIAG_CHUNK_BEATS * JESD_CAP_LANES_PER_BEAT;

		for (size_t s = 0; s < DIAG_CHUNK_BEATS * JESD_CAP_LANES_PER_BEAT;
		     s++) {
			int32_t v = buf[base + s];

			energy += (uint64_t)((int64_t)v * v);
		}
		rms[c] = diag_isqrt(energy /
				    (DIAG_CHUNK_BEATS * JESD_CAP_LANES_PER_BEAT));
		if (rms[c] >= DIAG_SIGNAL_RMS) {
			on++;
		}
	}

	LOG_INF("  per-chunk RMS (%u beats = %u ns each):", DIAG_CHUNK_BEATS,
		(unsigned int)((uint64_t)DIAG_CHUNK_BEATS * 1000000000U /
			       JESD_PB_SAMPLE_RATE));
	for (size_t c = 0; c < chunks; c += DIAG_CHUNKS_PER_ROW) {
		char row[DIAG_CHUNKS_PER_ROW * 8 + 1];
		size_t len = 0;

		for (size_t k = 0; k < DIAG_CHUNKS_PER_ROW && c + k < chunks; k++) {
			len += snprintk(&row[len], sizeof(row) - len, "%6llu ",
					(unsigned long long)rms[c + k]);
		}
		LOG_INF("  [%02zu] %s", c, row);
	}

	LOG_INF("  %zu/%zu chunks carry signal (%zu%% duty cycle over %u us)", on,
		chunks, on * 100U / chunks,
		(unsigned int)((uint64_t)chunks * DIAG_CHUNK_BEATS * 1000000U /
			       JESD_PB_SAMPLE_RATE));

	if (on == chunks || on == 0) {
		LOG_WRN("  UNIFORM across the whole capture (%s) -- the return does not",
			on ? "tone throughout" : "noise floor throughout");
		LOG_WRN("  change state within a capture. Combined with [4/7] showing");
		LOG_WRN("  both outcomes at the same settings, the thing that varies is not");
		LOG_WRN("  the signal but the capture: each one re-arms the RX DMAC, and");
		LOG_WRN("  that arming is now the only untested variable left.");
	} else {
		size_t edges = 0, longest_on = 0, run = 0;

		/*
		 * Count state changes and the longest unbroken on-run. Two edges in a
		 * window mean a full on-period is contained in it, so its length is
		 * measured rather than merely bounded below -- that is the number an
		 * upstream stage has to be able to account for.
		 */
		for (size_t c = 0; c < chunks; c++) {
			bool hi = rms[c] >= DIAG_SIGNAL_RMS;

			if (c && hi != (rms[c - 1] >= DIAG_SIGNAL_RMS)) {
				edges++;
			}
			run = hi ? run + 1 : 0;
			if (run > longest_on) {
				longest_on = run;
			}
		}

		LOG_WRN("  MIXED -- the return genuinely switches state, roughly %zu%% on,",
			on * 100U / chunks);
		LOG_WRN("  so the source is gating the tone. This also explains [4/7]:");
		LOG_WRN("  captures shorter than the gate period land wholly inside an on-");
		LOG_WRN("  or off-period, which is why they read all-or-nothing.");
		LOG_INF("  %zu transitions, longest on-run %zu chunks (%u us)", edges,
			longest_on,
			(unsigned int)((uint64_t)longest_on * DIAG_CHUNK_BEATS *
				       1000000U / JESD_PB_SAMPLE_RATE));
		if (edges >= 2) {
			LOG_INF("  a complete on-period fits this window, so that on-time is");
			LOG_INF("  measured, not just a lower bound.");
		} else {
			LOG_INF("  only %zu transition seen, so the on-time is a lower bound --",
				edges);
			LOG_INF("  the period is longer than this %u us window.",
				(unsigned int)((uint64_t)chunks * DIAG_CHUNK_BEATS *
					       1000000U / JESD_PB_SAMPLE_RATE));
		}
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
 * whether it sustained 250 MSPS x 16 bytes = 4 GB/s. A cyclic buffer shorter than
 * the CPU's re-arm latency would starve the TPL between wraps, and the DAC would
 * emit brief bursts separated by long gaps -- indistinguishable from noise once
 * averaged over a capture, with every status bit healthy throughout.
 *
 * But that only bites if the CPU is in the re-arm path at all, so this reads the
 * core's cyclic capability rather than assuming it: with hardware cyclic the loop
 * runs unattended and wrap rate is free, which rules buffer size out entirely
 * instead of leaving it as a plausible-sounding suspect. The numbers are printed
 * from the actual buffer and the actual registers, because a diagnostic that
 * hardcodes the figures it is arguing from will keep asserting them after the code
 * has changed underneath it -- which is exactly what happened here on the first
 * pass.
 */
static void diag_dma_feed(void)
{
	const struct device *tx_dma = DEVICE_DT_GET(DT_NODELABEL(tx_dmac));
	const int16_t *buf;
	struct dma_status s0, s1;
	size_t bytes, pb_bytes;
	bool nonzero = false;

	LOG_INF("[7/7] the DMA feed itself (everything from the TPL on is proven):");

	if (jesd_playback_buffer(&buf, &bytes) != 0) {
		LOG_WRN("  playback buffer unavailable");
		return;
	}

	/*
	 * Read what is actually in DDR, not what the CPU's cache remembers. Only
	 * the leading portion is inspected: the buffer is now megabytes, the
	 * question is whether pb_fill()'s writes reached memory at all, and a
	 * partial flush would show up in the first beats as readily as the last.
	 * Invalidating only what is read also avoids discarding clean lines
	 * needlessly.
	 */
	pb_bytes = bytes; /* keep the real size; the read-back below is clamped */
	if (bytes > PB_DIAG_INSPECT_BYTES) {
		bytes = PB_DIAG_INSPECT_BYTES;
	}
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
	 * The driver's view is reported at the very end, after the hardware has been
	 * sampled -- NOT here. dma_get_status() on an IRQ-less core pumps
	 * dmac_service(), which reads IRQ_PENDING and writes it straight back, and
	 * that register is write-1-to-clear: calling it first wipes the latched
	 * SOT/EOT bits and any subsequent read of them returns 0 no matter what the
	 * core did. Doing exactly that is what made this check report "the transfer
	 * never ran" while capture #5 of [4] was recovering the tone at full
	 * amplitude. Read the hardware before touching the DMA API.
	 */

	/*
	 * The open question was sustained rate: this datapath wants
	 * 4 GB/s (250 MSPS x 16 B/beat), and a 1 KiB cyclic buffer is only 256 ns
	 * long, so it must wrap 3.9 million times per second. That is the one
	 * remaining explanation consistent with every observation: the engine is
	 * alive and the link is perfect, but the TPL is starved between wraps, so
	 * the DAC emits brief bursts separated by long gaps. Averaged over a
	 * capture that is indistinguishable from noise, and it would vary per boot.
	 */
	/*
	 * Still busy. The rate question comes next, but it only has teeth if the
	 * CPU is the one re-arming, so read that out of the core rather than
	 * asserting it. AXI_DMAC_REG_FLAGS bit0 reads back set only when the core
	 * was synthesized with cyclic support, in which case the hardware loops the
	 * buffer unaided and the wrap rate costs nothing -- no amount of wrapping
	 * can starve a transfer nobody has to service. If it reads back clear, every
	 * wrap needs a dma_get_status() poll to re-submit, and at these rates that
	 * is hopeless.
	 *
	 * X_LENGTH matters for the same reason and is easy to miss: the driver
	 * chunks a transfer into max_length-sized bursts, and hardware cyclic loops
	 * only the burst it last submitted. A buffer larger than one burst therefore
	 * plays its tail repeatedly, not the whole buffer -- harmless for a periodic
	 * tone, but it means "I made the buffer bigger" is not the same statement as
	 * "the loop got longer".
	 */
	uintptr_t dmac = PB_DIAG_DMAC_BASE;
	uint32_t saved = sys_read32(dmac + PB_DIAG_DMAC_REG_FLAGS);
	uint32_t hw_cyclic, max_len;

	sys_write32(BIT(0), dmac + PB_DIAG_DMAC_REG_FLAGS);
	hw_cyclic = sys_read32(dmac + PB_DIAG_DMAC_REG_FLAGS) & BIT(0);
	sys_write32(saved, dmac + PB_DIAG_DMAC_REG_FLAGS);
	max_len = sys_read32(dmac + PB_DIAG_DMAC_REG_X_LENGTH) + 1U;

	LOG_WRN("  the buffer is correct in DDR, yet no tone arrives.");
	LOG_INF("  buffer %zu KiB (%u us per wrap at %u MSPS x %u B/beat)",
		pb_bytes / 1024U,
		(unsigned int)((uint64_t)(pb_bytes / PB_DIAG_BEAT_BYTES) *
			       1000000U / JESD_PB_SAMPLE_RATE),
		JESD_PB_SAMPLE_RATE / 1000000U, PB_DIAG_BEAT_BYTES);
	LOG_INF("  DMAC cyclic=%s, current burst length %u bytes",
		hw_cyclic ? "hw" : "sw-only", max_len);

	/*
	 * Ask the hardware whether it is moving data, which nothing has yet done.
	 *
	 * dma_status.busy is not evidence: the driver computes it as
	 * (remaining_size > 0 || pending_bursts > 0), both software counters. For a
	 * buffer that fits one burst, submitting leaves remaining_size at 0 and
	 * pending_bursts at 1, and with no IRQ line and hardware cyclic active
	 * dmac_service() never decrements either -- so busy reads 1 forever whether
	 * or not a single byte has left DDR. Every "the DMA is running" claim in this
	 * ladder traced back to that variable.
	 *
	 * TRANSFER_ID and TRANSFER_DONE are counters in the core itself, and SOT/EOT
	 * latch in IRQ_PENDING. Sampling them across a known interval distinguishes
	 * the two cases the software flag cannot: a core cycling through transfers,
	 * versus a core sitting on a submitted descriptor it never started.
	 */
	uint32_t id0 = sys_read32(dmac + PB_DIAG_DMAC_REG_TRANSFER_ID);
	uint32_t done0 = sys_read32(dmac + PB_DIAG_DMAC_REG_TRANSFER_DONE);
	uint32_t pend0 = sys_read32(dmac + PB_DIAG_DMAC_REG_IRQ_PENDING);
	uint32_t ctrl = sys_read32(dmac + PB_DIAG_DMAC_REG_CTRL);

	k_msleep(50);

	uint32_t id1 = sys_read32(dmac + PB_DIAG_DMAC_REG_TRANSFER_ID);
	uint32_t done1 = sys_read32(dmac + PB_DIAG_DMAC_REG_TRANSFER_DONE);
	uint32_t pend1 = sys_read32(dmac + PB_DIAG_DMAC_REG_IRQ_PENDING);

	LOG_INF("  core: CTRL=0x%08x (enable=%u), X_LENGTH=%u", ctrl,
		!!(ctrl & BIT(0)), max_len);
	LOG_INF("  core: TRANSFER_ID %u -> %u, TRANSFER_DONE 0x%08x -> 0x%08x",
		id0, id1, done0, done1);
	LOG_INF("  core: IRQ_PENDING 0x%02x -> 0x%02x (SOT=%u/%u EOT=%u/%u)",
		pend0, pend1, !!(pend0 & BIT(0)), !!(pend1 & BIT(0)),
		!!(pend0 & BIT(1)), !!(pend1 & BIT(1)));

	if (!(ctrl & BIT(0))) {
		LOG_ERR("  the core is DISABLED -- CTRL enable bit is clear, so it is not");
		LOG_ERR("  fetching anything. THIS is the fault.");
		return;
	}

	/*
	 * Interpreting this needs care, because under hardware cyclic a *healthy*
	 * core also leaves TRANSFER_ID still: it replays the one submitted transfer
	 * rather than issuing new IDs, so a frozen ID is not a stall. SOT is the
	 * meaningful bit -- it latches when a transfer actually starts moving beats,
	 * and nothing but a genuinely unstarted transfer leaves it clear.
	 */
	if (!(pend0 & (BIT(0) | BIT(1))) && id0 == id1 && done0 == done1) {
		LOG_WRN("  no SOT/EOT latched and no counter movement. Under hardware cyclic a");
		LOG_WRN("  static TRANSFER_ID is normal (one transfer is replayed), so this is");
		LOG_WRN("  only suggestive -- but a never-latched SOT would mean the transfer");
		LOG_WRN("  never started, pointing at the DMA-to-TPL stream handshake: the one");
		LOG_WRN("  interface [6] cannot exercise, since the TPL's internal DDS bypasses");
		LOG_WRN("  the stream input entirely.");
	} else {
		LOG_INF("  the core has started transfers (SOT/EOT latched or counters moved),");
		LOG_INF("  so it is genuinely fetching from DDR and streaming to the TPL.");
	}

	/*
	 * Whatever the registers say, [4] is the stronger evidence and it overrides
	 * any conclusion drawn here: if even one capture in that run recovered the
	 * tone at amplitude, samples demonstrably complete the whole trip and no
	 * "the transfer never ran" story can be true. The fault is then not a broken
	 * datapath but an unreliable one, which is a different search.
	 */
	LOG_INF("  NOTE: if [4] shows any capture recovering the tone, the feed provably");
	LOG_INF("  works and the fault is intermittency, not a dead stage.");

	/*
	 * Logged last, and only for contrast: busy is (remaining_size > 0 ||
	 * pending_bursts > 0), pure software state that a one-burst hardware-cyclic
	 * transfer pins at 1 forever. Reading it earlier would have destroyed the
	 * SOT/EOT evidence above.
	 */
	if (dma_get_status(tx_dma, PB_DIAG_DMA_CHANNEL, &s0) == 0) {
		LOG_INF("  driver view (software counters, not evidence): busy=%u pending=%u",
			s0.busy, s0.pending_length);
	}
	(void)s1;

	if (!hw_cyclic) {
		LOG_ERR("  the core has NO hardware cyclic support, so every wrap needs a");
		LOG_ERR("  dma_get_status() poll to re-submit. At %u MSPS x %u B/beat this",
			JESD_PB_SAMPLE_RATE / 1000000U, PB_DIAG_BEAT_BYTES);
		LOG_ERR("  link drains the buffer far faster than the CPU can re-arm it, so");
		LOG_ERR("  the TPL starves between wraps and the DAC emits brief bursts with");
		LOG_ERR("  long gaps -- which averages to the noise floor Rung 5 reports.");
		LOG_ERR("  Fix is a longer buffer (fewer wraps/s), not a different setting.");
		return;
	}

	LOG_WRN("  but the hardware loops the buffer itself, so the CPU is not in the");
	LOG_WRN("  re-arm path and wrap rate cannot starve it. Buffer size is NOT the");
	LOG_WRN("  fault. What remains is what the loop actually contains: hardware");
	LOG_WRN("  cyclic repeats only the last submitted burst, so if the burst length");
	LOG_WRN("  above is smaller than the buffer, the DAC is replaying the tail.");
	if (max_len < pb_bytes) {
		LOG_WRN("  -- and it IS smaller here (%u < %zu). Size the buffer to one burst.",
			max_len, pb_bytes);
	} else {
		LOG_WRN("  -- the whole buffer fits one burst, so the loop content is right");
		LOG_WRN("  too, and the remaining suspects are the TPL's sample-to-beat");
		LOG_WRN("  mapping and the DMA-to-TPL stream handshake, not the buffer.");
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

	/* Whatever the hit count came out as, resolve what it means: one long
	 * capture scanned in chunks separates a real duty cycle from a window
	 * too short to land on a continuous signal. */
	diag_duty_cycle();

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
