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
 * tuning word reads back exactly as requested, so silence at a swept frequency is
 * not a failed retune. The TX channel gains read back as the programmed 1024, so
 * the datapath is not muted. The chip's main-datapath and fine-DUC DC test tones
 * both return through the cable at full amplitude, proving DAC output stage,
 * balun, cable, ADC, RX DDC, framer, RX link, RX DMA and the TX NCOs all work.
 * And the JRX elastic-buffer protection (0x4A1 bit6) has been cleared with no
 * effect on the symptom, so it is not the mechanism either.
 *
 * The symptom is now characterised rather than merely observed: the return is
 * *gated*, ~200-300 us on against ~2500-2800 us off, about 9% duty ([8] and [9]).
 * Nothing in software asks for that. It is a measurement, not an inference -- a
 * capture straddling a transition gives the same amplitude by two independent
 * routes (RMS = sqrt(f)*A and concentration = f*500 both yield A ~ 4580, matching
 * the 4576 that full-window captures read).
 *
 * That reframes every check here. Our DMA-sourced tone *does* complete the whole
 * trip -- DDR -> DMA -> TPL -> lanes -> deframer -> DUC -> DAC -> cable -> ADC ->
 * DMA -> DDR -- at full amplitude, just not continuously. So no stage is dead,
 * and any check that reads presence-or-absence from a single capture is measuring
 * the gate phase it happened to land in, not the stage it means to test. At 9%
 * duty a lone capture is a coin flip. Every check that judges presence therefore
 * samples across several gate periods and reports a duty cycle; [2] documents the
 * arithmetic behind how many samples that takes.
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
 * RESOLVED. The gate was an ADI axi_data_offload core sitting between the TX DMAC
 * and the TPL, which this port had never configured -- see axi_data_offload.h. It
 * is a store-and-replay buffer: it accumulates 1 MiB, streams it out at line rate
 * (65536 beats / 250 MSPS = 262 us), then goes silent while refilling. That is the
 * entire 9% duty and the 262 us on-time, from one number. main.c now puts both
 * cores into bypass at startup and the measured duty is 64/64.
 *
 * Nothing was faulty. The block was doing exactly what it was built to do, and the
 * port had never told it to do anything else. The reason this took four wrong
 * answers is worth recording: every hypothesis was formed from register readbacks
 * of blocks we already knew about, and no readback can reveal a block absent from
 * your model of the datapath. The reference HDL states plainly that the offload
 * gates the DMA's transfer-request line; reading it earlier would have skipped the
 * whole detour.
 *
 * The checks are kept, at their historical numbering, because they are what makes
 * that claim falsifiable and because each one still bounds something: [2] proves the
 * gate was frequency-independent, [6] places it relative to the TPL input, [10]
 * rules out the elastic-buffer protection, and [12] is now a regression check that
 * the bypass is still in effect.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/cache.h>
#include <zephyr/kernel/mm.h>
#include <zephyr/sys/device_mmio.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(jesd_diag, LOG_LEVEL_INF);

#include "ad9081.h"
#include "axi_data_offload.h"
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
 * Sweep points for the TX/RX NCO pair, in MHz, spanning the ADC's first Nyquist
 * zone. 1968 is kept last so the sweep always includes the frequency the rest of
 * the app actually runs at.
 */
static const uint32_t diag_sweep_mhz[] = {
	10,  25,  50,  75,   100,  150,  200,  300,
	400, 500, 700, 900,  1200, 1500, 1750, 1968,
};

/*
 * Probes per sweep point.
 *
 * This number is the whole correctness of the sweep, so it is derived rather than
 * picked. The return is gated at ~9% duty with a ~2.8 ms period ([8] and [9]), and
 * the earlier version of this check took *one* full measurement per point. At 9%
 * duty a single sample reads "signal" 9% of the time, so four points yielding one
 * hit is precisely what a gate with no frequency dependence predicts -- expected
 * 0.36 hits -- and reading that one hit at 100 MHz as a property *of* 100 MHz is
 * the error this check used to invite. Simply densifying the point list would have
 * made that worse rather than better: 16 single-sample points scatter ~1.4 hits
 * across the band, and scattered hits always look like a pattern.
 *
 * So each point is sampled long enough to have certainly seen both gate phases. At
 * ~340 us per probe, 64 probes span ~22 ms -- about eight gate periods, enough that
 * an on-window cannot be missed by luck. Each point then reports a duty cycle
 * instead of a coin flip, and comparing points becomes meaningful:
 *
 *   duty ~9% at every point -> the gate is frequency-independent and the 100 MHz
 *                              hit was sampling luck. The tone gets through
 *                              everywhere, gated identically.
 *   duty varies with f      -> the gating mechanism is itself frequency-dependent,
 *                              which is a far stronger clue than presence/absence
 *                              and points at the DUC/NCO path rather than the link.
 *   duty 0 at some points   -> a real band limit, and now trustworthy: zero out of
 *                              64 probes is not luck.
 */
#define DIAG_SWEEP_PROBES 64U

/* Recovered-tone concentration (permille) that counts as "the tone came back".
 * Well above the ~13 that broadband noise scores, well below the ~999 of a clean
 * return, so a weak-but-real signal still registers. */
#define DIAG_TONE_FOUND_MIN 200

/* Above this per-sample RMS there is definitely *something* at the ADC input,
 * whether or not it correlates with the tone we sent. */
#define DIAG_SIGNAL_RMS 16

/*
 * TX DMAC, addressed directly. [7] counts buffer wraps out of IRQ_PENDING, and
 * doing that through the DMA API is not possible: dma_get_status() pumps the
 * driver's state machine, which reads IRQ_PENDING and writes it straight back, and
 * the register is write-1-to-clear -- so the API call consumes the very events
 * being counted.
 */
#define DIAG_DMAC_TX_BASE          0x9C430000UL
#define DIAG_DMAC_REG_IRQ_PENDING  0x0084
#define DIAG_DMAC_REG_CTRL         0x0400
#define DIAG_DMAC_REG_TRANSFER_ID  0x0404
#define DIAG_DMAC_REG_INTF_DESC    0x0010

/*
 * Bus width, read from the core instead of assumed.
 *
 * INTF_DESC encodes bytes-per-beat as a log2 exponent per interface: source width in
 * bits 11:8, destination width in bits 3:0. Everything in this sample was written
 * around "the bus is 16 bytes wide", which sets both the beat layout and the 4 GB/s
 * figure that the whole bandwidth argument rests on -- and that number was taken from
 * the HDL handoff, never read back. If the core is actually 32 bytes wide the demand
 * is 8 GB/s, not 4, and every capacity claim made so far is off by 2x. Cheap to check,
 * so it is checked.
 */
#define DIAG_INTF_BPB_SRC_MASK  GENMASK(11, 8)
#define DIAG_INTF_BPB_DEST_MASK GENMASK(3, 0)

/*
 * Bytes the JESD link consumes per sample period: 8 converters x NP16.
 *
 * Confirmed against the reference HDL rather than inherited from the handoff. For
 * this build (JESD_MODE=8B10B, M=8, L=4, S=1, NP=16 -- the project defaults):
 *
 *   F = (M*S*NP)/(8*L) = 4,  DATAPATH_WIDTH = 4 (8B10B)
 *   SAMPLES_PER_CHANNEL = (L*8*DATAPATH_WIDTH)/(M*NP) = 1
 *   dma_data_width = NP * M * SAMPLES_PER_CHANNEL = 128 bits = 16 bytes
 *
 * So 16 is right, and the INTF_DESC readback in [7] agrees (src=dest=16). The
 * "256 bits" in the block design is do_axi_data_width -- the *memory-mapped* AXI
 * width of the data-offload block below, a different interface entirely.
 */
#define DIAG_BEAT_BYTES 16U

/*
 * The data-offload cores are handled by axi_data_offload.c, which owns their
 * register map and puts both into bypass at startup. Nothing here duplicates that
 * -- the checks below call into it rather than keeping a second copy of the map,
 * which is how the register offsets came to be defined twice in the first place.
 */

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

	LOG_INF("[0/12] chip latched IRQ status (IRQB0 is lit on the board):");

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
	LOG_INF("[1/12] TX fine-DUC channel gain readback (programmed 1024):");

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
 * never actually programmed" -- both look like silence. Comparing the readback
 * against the FTW the requested frequency implies settles it: a matching FTW
 * means the tone really was on that carrier and any silence is physical, while a
 * stale or clamped FTW means the sweep was measuring the same setting
 * repeatedly. Every point has matched for many runs, so only a mismatch is
 * logged; see the print below.
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

	/*
	 * Only a mismatch is worth a line. Every FTW has read back exactly as
	 * requested for many runs now, and the sweep has grown from 4 points to 16,
	 * so printing the agreeing case would add 16 lines that say nothing. The
	 * check still runs on every retune -- silence here means all four values
	 * matched, and a regression still announces itself.
	 */
	if (tx_ftw != want_tx || rx_ftw != want_rx) {
		LOG_ERR("    FTW MISMATCH: tx 0x%012llx (want 0x%012llx), rx 0x%012llx (want 0x%012llx)",
			(unsigned long long)tx_ftw, (unsigned long long)want_tx,
			(unsigned long long)rx_ftw, (unsigned long long)want_rx);
		LOG_ERR("    this frequency was never programmed, so any silence at it");
		LOG_ERR("    says nothing about the analog path");
	}

	return 0;
}

/*
 * Sweep the matched NCO pair and measure, at each point, what fraction of the time
 * the tone comes back. The baseband tone Rung 4 is playing does not move -- only the
 * RF carrier it rides on does -- so the correlator stays valid at every point
 * without touching the playback buffer.
 *
 * Two instruments per point, because they answer different questions and neither
 * alone is enough while the return is gated:
 *
 *   DIAG_SWEEP_PROBES fast probes give the *duty cycle* -- how often anything is
 *   there. Cheap, one bit each, and immune to the sampling luck that made the old
 *   single-measurement version unreadable.
 *
 *   Then, only where the duty is non-zero, up to DIAG_SWEEP_ID_TRIES full
 *   measurements to identify *what* is there -- the probe cannot tell the tone from
 *   any other energy. These retry because a full measurement is itself a coin flip
 *   against the gate: at ~9% duty a single one usually lands in an off-window, and
 *   giving up after one would report "gated but unidentifiable" at nearly every
 *   point that is in fact returning a clean tone.
 */
#define DIAG_SWEEP_ID_TRIES 12U

static bool diag_sweep(adi_ad9081_device_t *dev)
{
	bool found_any = false;

	LOG_INF("[2/12] NCO frequency sweep, %u probes per point (gate-aware):",
		DIAG_SWEEP_PROBES);
	LOG_INF("  baseband tone stays at -%u MHz; only the RF carrier moves",
		JESD_PB_TONE_HZ / 1000000U);
	LOG_INF("  duty is what matters: the return is gated, so presence in any one");
	LOG_INF("  capture is luck. Compare duty across points, not hit/miss.");

	for (size_t i = 0; i < ARRAY_SIZE(diag_sweep_mhz); i++) {
		int64_t hz = (int64_t)diag_sweep_mhz[i] * 1000000LL;
		struct jesd_loopback_meas m;
		size_t on = 0, taken = 0;
		int64_t t0;
		uint32_t span_us;

		if (diag_retune(dev, hz) != 0) {
			continue;
		}

		t0 = k_uptime_ticks();
		for (size_t p = 0; p < DIAG_SWEEP_PROBES; p++) {
			int r = jesd_capture_probe();

			if (r < 0) {
				LOG_WRN("  %4u MHz: probe failed (%d)",
					diag_sweep_mhz[i], r);
				break;
			}
			on += (size_t)r;
			taken++;
		}
		span_us = (uint32_t)k_ticks_to_us_floor64(k_uptime_ticks() - t0);

		if (taken == 0) {
			continue;
		}

		LOG_INF("  %4u MHz: duty %zu/%zu (%zu%%) over %u us",
			diag_sweep_mhz[i], on, taken, on * 100U / taken, span_us);

		if (on == 0) {
			/*
			 * Nothing in the whole window. Unlike the old version this is
			 * a real statement: DIAG_SWEEP_PROBES spans several gate
			 * periods, so silence here is silence, not a missed window.
			 */
			continue;
		}

		/*
		 * Something is arriving. Identify it, retrying against the gate --
		 * see DIAG_SWEEP_ID_TRIES above.
		 */
		memset(&m, 0, sizeof(m));
		for (size_t t = 0; t < DIAG_SWEEP_ID_TRIES; t++) {
			if (jesd_loopback_measure(&m) != 0) {
				m.rms = 0;
				continue;
			}
			if (m.rms >= DIAG_SIGNAL_RMS) {
				break;
			}
			m.rms = 0;
		}

		if (m.rms < DIAG_SIGNAL_RMS) {
			LOG_INF("       energy present but %u full captures all landed in",
				DIAG_SWEEP_ID_TRIES);
			LOG_INF("       off-windows, so it is gated but unidentified here");
			continue;
		}

		LOG_INF("       RMS %llu, lanes %llu/%llu/%llu/%llu/%llu/%llu/%llu/%llu",
			(unsigned long long)m.rms,
			(unsigned long long)m.lane_rms[0],
			(unsigned long long)m.lane_rms[1],
			(unsigned long long)m.lane_rms[2],
			(unsigned long long)m.lane_rms[3],
			(unsigned long long)m.lane_rms[4],
			(unsigned long long)m.lane_rms[5],
			(unsigned long long)m.lane_rms[6],
			(unsigned long long)m.lane_rms[7]);
		LOG_INF("       tone: interleaved %u/1000 (amp %llu), split %u/1000 (amp %llu)",
			m.concentration, (unsigned long long)m.amplitude,
			m.concentration_split,
			(unsigned long long)m.amplitude_split);

		uint32_t best = MAX(m.concentration, m.concentration_split);

		if (best >= DIAG_TONE_FOUND_MIN) {
			LOG_INF("       ^^ our tone, gated at the duty above");
			found_any = true;
		} else {
			LOG_INF("       ^^ energy, but not our tone (wrong bin)");
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

	LOG_INF("[3/12] chip-internal DAC test tone (bypasses DMA + link + deframer):");

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

	LOG_INF("[4/12] repeatability of our own tone (%u captures at the same setting):",
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
		LOG_WRN("  INTERMITTENT within a single boot -- see [8/12] for whether that");
		LOG_WRN("  is really intermittence or just a too-short capture window.");
	}
}

/*
 * Duty cycle of the analog return, measured *within* one capture.
 *
 * [4/12] counts how many captures saw signal, and that count has been misleading:
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

	LOG_INF("[8/12] duty cycle of the return within one capture:");

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
		LOG_INF("  UNIFORM (%s) -- this capture fell entirely inside one state,",
			on ? "tone throughout" : "noise floor throughout");
		LOG_INF("  which is the common case once the gating is known: a window");
		LOG_INF("  shorter than the period usually misses the transition. It still");
		LOG_INF("  bounds one half of the cycle at >= %u us. See [9/12] for the period.",
			(unsigned int)((uint64_t)chunks * DIAG_CHUNK_BEATS * 1000000U /
				       JESD_PB_SAMPLE_RATE));
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
		LOG_WRN("  so the source is gating the tone. This also explains [4/12]:");
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
 * Period of the gating, measured across many captures instead of within one.
 *
 * [8/12] established that the return is gated and bounded the two halves at on >=
 * 65 us and off >= 262 us, but it cannot do better: a single transfer caps at 1 MiB
 * (~262 us) on this core, so no one capture spans a full cycle.
 *
 * Run captures back to back instead and record only whether each saw the tone. That
 * turns [8/12]'s limitation into the instrument: because a capture shorter than the
 * period reads all-or-nothing, each one is a clean one-bit sample of the gate state,
 * and the run lengths in the resulting strip are the on and off times. Sampling is
 * uneven (each capture costs its own arming and cache maintenance, and the log call
 * is not free), so the strip measures the period in units of captures and the
 * elapsed wall time converts it to microseconds.
 */
#define DIAG_STRIP_SAMPLES 1024U
#define DIAG_STRIP_PER_ROW 64U
#define DIAG_EDGES_MAX     64U

static void diag_gate_period(void)
{
	static uint8_t state[DIAG_STRIP_SAMPLES];
	static int64_t stamp[DIAG_STRIP_SAMPLES];
	int64_t edge_us[DIAG_EDGES_MAX];
	size_t on = 0, edges = 0, taken = 0;
	int64_t t0, elapsed_us;

	LOG_INF("[9/12] gate period from %u fast probes:", DIAG_STRIP_SAMPLES);

	/*
	 * Sample as fast as the probe allows, timestamping each one. The previous
	 * version used the full measurement and aliased badly: at ~13.7 ms per
	 * sample against a source toggling faster than that, the strip showed a
	 * regular 4-sample beat that was pure artefact. Timestamps make the
	 * aliasing visible rather than silent -- if the sample interval is not
	 * comfortably shorter than the runs, the numbers below say so.
	 */
	t0 = k_uptime_ticks();
	for (size_t i = 0; i < DIAG_STRIP_SAMPLES; i++) {
		int p = jesd_capture_probe();

		if (p < 0) {
			LOG_WRN("  probe failed at sample %zu (%d)", i, p);
			break;
		}
		state[i] = (uint8_t)p;
		stamp[i] = k_ticks_to_us_floor64(k_uptime_ticks() - t0);
		taken++;
	}
	elapsed_us = taken ? stamp[taken - 1] : 0;

	if (taken < 2) {
		LOG_WRN("  too few samples to say anything");
		return;
	}

	for (size_t i = 0; i < taken; i++) {
		on += state[i];
		if (i && state[i] != state[i - 1]) {
			if (edges < DIAG_EDGES_MAX) {
				edge_us[edges] = stamp[i];
			}
			edges++;
		}
	}

	for (size_t i = 0; i < taken; i += DIAG_STRIP_PER_ROW) {
		char row[DIAG_STRIP_PER_ROW + 1];
		size_t k = 0;

		for (; k < DIAG_STRIP_PER_ROW && i + k < taken; k++) {
			row[k] = state[i + k] ? '#' : '.';
		}
		row[k] = '\0';
		LOG_INF("  %s", row);
	}

	LOG_INF("  %zu/%zu probes saw signal (%zu%% on), %zu transitions in %lld us",
		on, taken, on * 100U / taken, edges, (long long)elapsed_us);
	LOG_INF("  sample interval %lld us -- runs must be several of these to be real",
		(long long)(elapsed_us / (int64_t)taken));

	/*
	 * Say so when the sampling is too coarse for the runs it is reporting. The
	 * first version of this check did not, and its 38870 us "period" was pure
	 * aliasing; the second sampled at 340 us against a ~300 us on-time, which
	 * misses whole pulses and shows up as intervals at exact multiples of the
	 * true period. An instrument that cannot detect its own aliasing will report
	 * a plausible number instead of an honest failure.
	 */
	if (on && (elapsed_us / (int64_t)taken) * (int64_t)on * 4 >= elapsed_us) {
		LOG_WRN("  UNDERSAMPLED: the on-time is comparable to the sample interval,");
		LOG_WRN("  so pulses are being missed and intervals below are multiples of");
		LOG_WRN("  the true period, not the period itself. Treat the on-time as an");
		LOG_WRN("  upper bound only.");
	}

	if (edges == 0) {
		LOG_WRN("  no transition in %lld us: the gate is slower than this sweep.",
			(long long)elapsed_us);
		return;
	}

	/*
	 * Report the interval between consecutive edges rather than a single mean.
	 * A gate with unequal on and off times alternates between two values, and an
	 * average of the two is a number that describes neither -- printing the
	 * sequence shows the structure and makes a non-periodic source obvious
	 * instead of averaging it into a plausible-looking figure.
	 */
	size_t shown = MIN(edges, DIAG_EDGES_MAX);

	if (shown >= 2) {
		char row[DIAG_STRIP_PER_ROW + 1];
		size_t len = 0;

		LOG_INF("  intervals between transitions (us):");
		for (size_t e = 1; e < shown; e++) {
			len += snprintk(&row[len], sizeof(row) - len, "%lld ",
					(long long)(edge_us[e] - edge_us[e - 1]));
			if (len > sizeof(row) - 12 || e == shown - 1) {
				LOG_INF("    %s", row);
				len = 0;
			}
		}
		LOG_INF("  alternating values here are the on and off times; a steady");
		LOG_INF("  pair means a periodic gate, scattered values mean it is not");
		LOG_INF("  periodic and the cause is event-driven rather than clocked.");
	} else {
		LOG_INF("  only one transition -- period is comparable to the %lld us sweep",
			(long long)elapsed_us);
	}
}

/*
 * JRX transport-layer elastic buffer state, sampled repeatedly.
 *
 * Found by reading the reference code rather than by measuring. The JRX TPL has a
 * buffer-protection mechanism that withholds data when the LMFC/elastic-buffer phase
 * is marginal, controlled by two bits in JRX_TPL_1 (0x4A1):
 *
 *   bit7 BUF_PROTECTION   - the ADI API clears this on the 204B path
 *   bit6 BUF_PROTECT_EN   - cleared only for 204C on rev<3, in no-OS. On 204B, which
 *                           is what this link runs, NEITHER no-OS nor the vendor API
 *                           ever writes it, so it sits at its reset default
 *
 * That is a mechanism which gates sample delivery, on the exact interface the fault
 * has been narrowed to, that nothing in the port ever configures. It also explains
 * why the chip's internal test tones are continuous while DMA samples gate: those
 * tones inject downstream of the JRX TPL and never pass through this buffer.
 *
 * JRX_TPL_5 (0x4A5) holds PHASE_DIFF, the measured phase between the arriving link
 * frame and the local LMFC. If protection is asserting, that value is drifting
 * rather than parked, so sampling it repeatedly distinguishes "phase is marginal and
 * wandering" from "phase is stable and something else gates the data".
 */
#define DIAG_JRX_TPL_SAMPLES 16U

static void diag_jrx_buffer(void)
{
	adi_ad9081_device_t *dev = ad9081_get_device();
	uint8_t tpl1 = 0, tpl5 = 0;
	uint8_t pd_min = 0xFF, pd_max = 0;
	int32_t err;

	LOG_INF("[10/12] JRX TPL elastic-buffer state (0x4A1 / 0x4A5):");

	if (dev == NULL) {
		return;
	}

	err = adi_ad9081_hal_reg_get(dev, REG_JRX_TPL_1_ADDR, &tpl1);
	if (err != API_CMS_ERROR_OK) {
		LOG_WRN("  could not read 0x4A1 (%d)", err);
		return;
	}

	LOG_INF("  0x4A1 = 0x%02x: BUF_PROTECTION(b7)=%u BUF_PROTECT_EN(b6)=%u SYSREF_IGNORE_WHEN_LINKED(b2)=%u",
		tpl1, (tpl1 >> 7) & 1U, (tpl1 >> 6) & 1U, (tpl1 >> 2) & 1U);

	for (uint32_t i = 0; i < DIAG_JRX_TPL_SAMPLES; i++) {
		if (adi_ad9081_hal_reg_get(dev, REG_JRX_TPL_5_ADDR, &tpl5) !=
		    API_CMS_ERROR_OK) {
			break;
		}
		pd_min = MIN(pd_min, tpl5);
		pd_max = MAX(pd_max, tpl5);
		k_msleep(1);
	}

	LOG_INF("  PHASE_DIFF over %u samples: min %u, max %u (spread %u)",
		DIAG_JRX_TPL_SAMPLES, pd_min, pd_max,
		(unsigned int)(pd_max - pd_min));

	if ((tpl1 >> 6) & 1U) {
		LOG_WRN("  BUF_PROTECT_EN is SET, and nothing in this port or in no-OS's");
		LOG_WRN("  204B path ever writes it -- it is at its reset default. This");
		LOG_WRN("  withholds JRX samples when the elastic-buffer phase is marginal,");
		LOG_WRN("  which gates exactly the stage the fault is confined to, and");
		LOG_WRN("  spares the chip's internal tones because they inject downstream.");
	} else {
		LOG_INF("  buffer protection is disabled, so it is not withholding samples.");
	}

	if (pd_max != pd_min) {
		LOG_WRN("  PHASE_DIFF is moving: the link frame is drifting against the");
		LOG_WRN("  local LMFC rather than sitting at a fixed offset. A drifting");
		LOG_WRN("  phase crossing the protection threshold periodically is the");
		LOG_WRN("  shape that produces a periodic gate.");
	} else {
		LOG_INF("  PHASE_DIFF is stable at %u -- the link/LMFC phase is not drifting.",
			pd_min);
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

	LOG_INF("[5/12] fine-DUC (channel) DC test tone -- one stage earlier than [3]:");

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
 * What it can and cannot conclude has narrowed since it was written. A returning DDS
 * tone was originally read as "the fault is upstream of the TPL, in the DMA feed" --
 * but [2] has since recovered our own DMA-sourced tone at full amplitude, so DMA
 * samples demonstrably do cross the TPL, the lanes and the deframer. That conclusion
 * is false and is not drawn here any more.
 *
 * What remains useful is the comparison, not a verdict: the DDS is a continuous
 * source injected at the TPL input, so its duty cycle isolates which side of the TPL
 * the gate is on. A gated DDS return puts the gate at or after the TPL input; a
 * continuous one puts it upstream, in how our samples reach that input. Measured
 * with the same probe as [2] for exactly that reason -- a single capture cannot tell
 * gated from absent, which is the trap the earlier version fell into.
 *
 * Judged on RMS rather than the correlator: the DDS frequency is quantised by a
 * 16-bit phase accumulator and does not land on the bin the correlator watches.
 */
static void diag_fpga_dds(void)
{
	struct jesd_loopback_meas m;
	int ret;

	LOG_INF("[6/12] FPGA DDS tone at the TPL input (crosses lanes + deframer):");

	ret = axi_tpl_tx_dds(JESD_PB_TONE_HZ, JESD_PB_SAMPLE_RATE, true);
	if (ret) {
		LOG_WRN("  could not arm the FPGA DDS (%d)", ret);
		return;
	}

	/* The TPL SYNC and the chip's deframer need a moment to settle on the new
	 * data source before the capture means anything. */
	k_msleep(5);

	/*
	 * Duty cycle of the DDS return, measured the same way [2] measures ours. The
	 * DDS source is continuous by construction, so any gating in its return was
	 * imposed downstream of the TPL input -- which is the one thing this check can
	 * still settle.
	 */
	size_t on = 0, taken = 0;

	for (size_t p = 0; p < DIAG_SWEEP_PROBES; p++) {
		int r = jesd_capture_probe();

		if (r < 0) {
			LOG_WRN("  probe failed (%d)", r);
			break;
		}
		on += (size_t)r;
		taken++;
	}

	if (taken) {
		LOG_INF("  DDS return duty %zu/%zu (%zu%%)", on, taken,
			on * 100U / taken);
	}

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
	}

	/*
	 * Compare against our own duty from [2] to place the gate. Stated as what the
	 * reading implies rather than as a verdict: a continuous DDS return with a
	 * gated DMA return is the interesting outcome, and it points at the DMA-to-TPL
	 * stream handshake -- the one interface the DDS bypasses.
	 */
	if (taken && on == taken) {
		LOG_INF("  continuous -- the gate is NOT downstream of the TPL input, so");
		LOG_INF("  it is in how our samples reach that input (the DMA-to-TPL");
		LOG_INF("  stream handshake), not in the lanes, deframer or DUC.");
	} else if (taken && on > 0) {
		LOG_INF("  gated too, at a duty comparable to ours -- so the gate is at or");
		LOG_INF("  after the TPL input and affects any source equally. The DMA feed");
		LOG_INF("  is exonerated; suspect the link, deframer or DAC datapath.");
	} else if (taken) {
		LOG_INF("  silent throughout, while our own tone does return in [2] -- so");
		LOG_INF("  the DDS is not actually driving the converters (check the");
		LOG_INF("  DATA_SEL write and SYNC), rather than the link being at fault.");
	}

	/* Put the converters back on the DMA source whatever happened, so the board
	 * is left as the rest of the app expects. */
	(void)axi_tpl_tx_dds(0, 0, false);
}

/*
 * Achieved TX bandwidth, measured rather than assumed.
 *
 * This is the number the whole Rung 5 investigation has been missing, and every
 * theory offered so far has been an assumption standing in for it.
 *
 * The link consumes one 16-byte beat per sample period: at 250 MSPS that is a flat
 * 4.0 GB/s the DMA must sustain, forever, with no gaps. Nothing has ever checked
 * whether it does. Status registers cannot answer it -- they report that transfers
 * complete, not how fast, and under hardware cyclic a completed transfer looks
 * identical whether it took 1 ms or 10 ms.
 *
 * The core exposes no byte-progress counter, so bandwidth is counted from wraps
 * instead. Each time the cyclic buffer is fully consumed the core latches EOT in
 * IRQ_PENDING, so N wraps over a known interval means N x buffer_bytes moved:
 *
 *     achieved = wraps * PB_BUF_BYTES / elapsed
 *
 * EOT is write-1-to-clear, so the loop must clear each one to see the next, and
 * this counts them directly rather than calling dma_get_status() -- that pumps the
 * driver's state machine and consumes the very bit being counted.
 *
 * That last point cuts both ways, and the first version of this check got it wrong:
 * it counted zero wraps in 50 ms while [2] was recovering the tone at full
 * amplitude, which is impossible if the transfer is running -- and it is, so the
 * instrument was at fault, not the DMA. The reason is that this core has no
 * interrupt line wired to the PS GIC (see the overlay), so the driver runs its
 * polling path: dmac_service() is pumped from dma_get_status(), and it *also* reads
 * IRQ_PENDING and writes it straight back. Anything that polls the DMA between our
 * samples -- and the capture path does, on every probe -- silently eats the EOTs we
 * are trying to count. A zero here therefore means "nobody let us see the events",
 * not "no bytes moved".
 *
 * So the window must be quiet: no capture, no probe, no dma_get_status() on this
 * channel while counting. The playback transfer is already running cyclically and
 * needs no service to keep going (cyclic=hw), so simply not touching it for the
 * duration is enough, and this check now runs before anything else disturbs it.
 *
 * Two possible outcomes, and they point at completely different work:
 *
 *   ~4 GB/s   The DMA sustains line rate, so the samples really are arriving
 *             continuously and something downstream gates the output. The gating
 *             hunt is then still the right hunt.
 *   ~360 MB/s The DMA is simply the bottleneck -- 9% of 4 GB/s is what a 9% duty
 *             delivers on average. Then nothing is "gating" anything: the TPL emits
 *             samples while its FIFO has data and silence while it refills, and the
 *             duty cycle is just the ratio of those. That is a bandwidth problem,
 *             not a fault, and it is fixed by lowering the demand (sample rate,
 *             interpolation, lane count) or raising the supply (wider DMA path),
 *             not by finding a bit to clear.
 *
 * Sampled with a short polling window: the buffer is 4 MiB, so at 4 GB/s it wraps
 * every ~1 ms and even a few tens of ms of polling counts plenty of wraps.
 */
#define DIAG_BW_WINDOW_MS 50U

static void diag_tx_bandwidth(void)
{
	uintptr_t dmac = DIAG_DMAC_TX_BASE;
	uint32_t wraps = 0;
	int64_t t0, elapsed_us;
	uint64_t achieved, demand;
	const int16_t *buf;
	size_t buf_bytes;

	LOG_INF("[7/12] achieved TX DMA bandwidth (the link needs a sustained 4 GB/s):");

	/* The wrap count only converts to bytes if the buffer size is known, and it
	 * is read from the playback module rather than recomputed here so the two
	 * cannot drift apart. */
	if (jesd_playback_buffer(&buf, &buf_bytes) != 0 || buf_bytes == 0) {
		LOG_WRN("  playback buffer size unavailable; cannot convert wraps to bytes");
		return;
	}

	/*
	 * What the core says its own bus is, before any rate is quoted against an
	 * assumed one. See DIAG_INTF_BPB_*_MASK: this sample assumes 16 bytes
	 * everywhere, and if the hardware disagrees the beat layout is wrong too, not
	 * just the bandwidth arithmetic.
	 */
	uint32_t intf = sys_read32(dmac + DIAG_DMAC_REG_INTF_DESC);
	uint32_t w_src = 1U << FIELD_GET(DIAG_INTF_BPB_SRC_MASK, intf);
	uint32_t w_dst = 1U << FIELD_GET(DIAG_INTF_BPB_DEST_MASK, intf);

	LOG_INF("  core bus width: src %u B, dest %u B (INTF_DESC 0x%08x)", w_src,
		w_dst, intf);

	if (w_src != DIAG_BEAT_BYTES || w_dst != DIAG_BEAT_BYTES) {
		LOG_ERR("  MISMATCH: this sample assumes a %u-byte beat everywhere, so the",
			DIAG_BEAT_BYTES);
		LOG_ERR("  sample-to-lane layout in jesd_capture/jesd_playback and the link");
		LOG_ERR("  demand below are both computed from the wrong width. Fix that");
		LOG_ERR("  before drawing any conclusion from the rate.");
	}

	/* Clear anything already latched so the count starts from now. */
	sys_write32(BIT(0) | BIT(1), dmac + DIAG_DMAC_REG_IRQ_PENDING);

	t0 = k_uptime_ticks();
	do {
		uint32_t pend = sys_read32(dmac + DIAG_DMAC_REG_IRQ_PENDING);

		if (pend & BIT(1)) {
			/* Write-1-to-clear, so clear it to see the next wrap. */
			sys_write32(pend & (BIT(0) | BIT(1)),
				    dmac + DIAG_DMAC_REG_IRQ_PENDING);
			wraps++;
		}
		elapsed_us = k_ticks_to_us_floor64(k_uptime_ticks() - t0);
	} while (elapsed_us < (int64_t)DIAG_BW_WINDOW_MS * 1000);

	if (elapsed_us <= 0) {
		LOG_WRN("  timer gave no elapsed time; cannot compute a rate");
		return;
	}

	/* Demand from the width the core reports, not the one this sample assumes --
	 * the whole point of reading INTF_DESC above. */
	demand = (uint64_t)JESD_PB_SAMPLE_RATE * w_src;
	achieved = ((uint64_t)wraps * (uint64_t)buf_bytes * 1000000ULL) /
		   (uint64_t)elapsed_us;

	LOG_INF("  %u buffer wraps in %lld us", wraps, (long long)elapsed_us);
	LOG_INF("  achieved %llu MB/s, link demands %llu MB/s (%llu%%)",
		(unsigned long long)(achieved / 1000000ULL),
		(unsigned long long)(demand / 1000000ULL),
		(unsigned long long)(demand ? achieved * 100ULL / demand : 0));

	if (wraps == 0) {
		/*
		 * Do not read this as "no bytes moved" -- see the note above. Report
		 * the alternatives in the order they are likely, and prove the
		 * transfer is alive independently before blaming the bandwidth.
		 */
		LOG_WRN("  no wraps counted. This does NOT mean no data moved: with no IRQ");
		LOG_WRN("  line on this core, any dma_get_status() elsewhere clears EOT");
		LOG_WRN("  before we see it. Checking whether the transfer is alive:");

		uint32_t ctrl = sys_read32(dmac + DIAG_DMAC_REG_CTRL);
		uint32_t id0 = sys_read32(dmac + DIAG_DMAC_REG_TRANSFER_ID);

		k_msleep(1);

		uint32_t id1 = sys_read32(dmac + DIAG_DMAC_REG_TRANSFER_ID);

		LOG_WRN("  CTRL 0x%08x (enable=%u), TRANSFER_ID %u -> %u", ctrl,
			(unsigned int)((ctrl & BIT(0)) != 0), id0, id1);

		if (!(ctrl & BIT(0))) {
			LOG_ERR("  the core is DISABLED -- the transfer really is not running.");
		} else if (id0 != id1) {
			LOG_WRN("  the ID is advancing, so the engine IS moving data and the");
			LOG_WRN("  zero above is purely a lost-event artefact. Re-run with");
			LOG_WRN("  nothing else polling this channel to get a real rate.");
		} else {
			LOG_WRN("  enabled but the ID is static. Under hardware cyclic that is");
			LOG_WRN("  the normal reading (one transfer is replayed forever), so it");
			LOG_WRN("  neither confirms nor refutes movement. The tone returning in");
			LOG_WRN("  [2] is the stronger evidence: if it does, data is moving.");
		}
		return;
	}

	/*
	 * Judged against the duty cycle, because that is what makes the two
	 * candidate explanations distinguishable rather than merely plausible.
	 */
	if (achieved * 2ULL >= demand) {
		LOG_INF("  the DMA sustains most of line rate, so samples really do arrive");
		LOG_INF("  continuously and the ~9%% duty is imposed downstream of the DMA.");
		LOG_INF("  Bandwidth is NOT the explanation; keep looking for a gate.");
	} else {
		LOG_WRN("  the DMA delivers well under line rate. This alone accounts for");
		LOG_WRN("  the duty cycle -- the TPL emits while its FIFO has data and goes");
		LOG_WRN("  silent while it refills, so no gating mechanism need exist. Match");
		LOG_WRN("  this percentage against [2]'s duty: if they agree, the answer is");
		LOG_WRN("  bandwidth, and the fix is to lower the demand (sample rate,");
		LOG_WRN("  interpolation, lane count) or widen the DMA path.");
	}
}

/*
 * [12] Duty cycle with the offload in bypass -- the regression check for the one
 * mechanism this investigation actually identified.
 *
 * History, because it is the useful part: the DAC output was present only ~9% of
 * the time, and four explanations were offered and disproved before the cause was
 * found -- a JRX elastic-buffer protection bit (cleared it, no change), frequency
 * dependence in the DUC path (swept 10-1968 MHz, duty flat at 9% everywhere), and
 * raw DMA bandwidth (unmeasurable on this core, and wrong in principle: the offload
 * exists precisely to absorb that).
 *
 * The cause was an axi_data_offload core between the DMA and the TPL that this port
 * had never configured. Its 1 MiB buffer drains in 262 us at 250 MSPS and then goes
 * quiet while refilling, which is the entire 9%. Toggling bypass moved the measured
 * duty from 6/64 to 64/64 in a single run -- the only intervention in this whole
 * investigation that changed the symptom.
 *
 * main.c now sets bypass at startup, so this check no longer runs an experiment; it
 * confirms the fix is still in effect. A duty back near 9% means the bypass write
 * did not take, which is worth catching loudly rather than rediscovering.
 */
static void diag_offload_duty(void)
{
	size_t on = 0, taken = 0;

	LOG_INF("[12/12] output duty with the offload bypassed (expect ~100%%):");

	for (size_t p = 0; p < DIAG_SWEEP_PROBES; p++) {
		int r = jesd_capture_probe();

		if (r < 0) {
			LOG_WRN("  probe failed (%d)", r);
			break;
		}
		on += (size_t)r;
		taken++;
	}

	if (taken == 0) {
		LOG_WRN("  no probes completed; inconclusive.");
		return;
	}

	size_t pct = on * 100U / taken;

	LOG_INF("  duty %zu/%zu (%zu%%)", on, taken, pct);

	if (pct >= 90U) {
		LOG_INF("  continuous -- the offload bypass is in effect and the datapath");
		LOG_INF("  streams without gaps. This is the working configuration.");
	} else if (pct <= 20U) {
		LOG_ERR("  back to a gated duty. The startup bypass write did not take, or");
		LOG_ERR("  something re-enabled store-and-replay. Check [11]'s bypass bit:");
		LOG_ERR("  this exact reading is what the whole Rung 5 hunt was chasing.");
	} else {
		LOG_WRN("  partially gated, which neither configuration predicts. Check");
		LOG_WRN("  [11] for a latched SRC_OVERFLOW or DST_UNDERFLOW -- in bypass");
		LOG_WRN("  there is no buffer to absorb a rate mismatch, so a DMA that");
		LOG_WRN("  cannot keep up shows here rather than being hidden.");
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

	/*
	 * Before any capture runs. Every capture and probe calls dma_get_status(),
	 * which on this IRQ-less core clears the EOT events [7] counts -- so this has
	 * to measure while the playback transfer is the only thing touching the DMA.
	 * Running it after the sweep is what made it report 0 MB/s.
	 */
	diag_tx_bandwidth();

	/*
	 * Offload state, as the running playback left it. Includes the overflow and
	 * underflow flags, which answer the starvation question in hardware rather
	 * than by inference -- and which [7] cannot answer at all on this core, since
	 * hardware cyclic never increments the transfer ID it counts.
	 */
	LOG_INF("[11/12] AXI data-offload cores (bypassed at startup by main.c):");
	axi_data_offload_status();

	swept_ok = diag_sweep(dev);
	internal_ok = diag_internal_tone(dev);
	diag_channel_tone(dev);
	diag_fpga_dds();

	/* Back to the configured plan before judging repeatability, so the repeats
	 * measure the frequency the rest of the app actually runs at. */
	(void)diag_retune(dev, DIAG_NCO_HZ_DEFAULT);
	diag_repeatability();

	/* Whatever the hit count came out as, resolve what it means: one long
	 * capture scanned in chunks separates a real duty cycle from a window
	 * too short to land on a continuous signal. */
	diag_duty_cycle();

	/* One capture cannot span a full gate cycle (1 MiB transfer ceiling), so
	 * measure the period across many captures instead. */
	diag_gate_period();

	/* The mechanism the reference code points at: a JRX elastic-buffer protection
	 * that gates sample delivery and that the 204B path never configures. */
	diag_jrx_buffer();

	diag_offload_duty();

	/* Restore the configured frequency plan whatever happened, so the board
	 * is left in the state the rest of the app documents. */
	if (diag_retune(dev, DIAG_NCO_HZ_DEFAULT) != 0) {
		LOG_WRN("could not restore the %lld MHz NCO pair",
			(long long)(DIAG_NCO_HZ_DEFAULT / 1000000LL));
	}

	LOG_INF("--- conclusion ---");
	if (swept_ok) {
		LOG_INF("our tone returns: the full chain works -- DDR -> DMA -> DAC ->");
		LOG_INF("  cable -> ADC -> DMA -> DDR. No stage is dead, so the open");
		LOG_INF("  question is only the gating, and [2]'s per-point duty is the");
		LOG_INF("  measurement to read: a duty that is flat across the band means");
		LOG_INF("  the gate is frequency-independent, while one that varies puts");
		LOG_INF("  the mechanism in the DUC/NCO path.");
	} else if (internal_ok) {
		LOG_INF("the chip's own tone reaches the ADC but ours did not this run:");
		LOG_INF("  the DAC output stage, balun, cable, ADC, RX DDC, framer, RX");
		LOG_INF("  link and RX DMA all work, and so does the TX main NCO, since");
		LOG_INF("  it is what upconverts that DC offset into a tone.");
		LOG_INF("  Do not read this as our datapath being dead: [2] has recovered");
		LOG_INF("  the DMA-sourced tone at full amplitude before, so an all-silent");
		LOG_INF("  sweep here means the gate stayed off throughout -- check the");
		LOG_INF("  per-point duty above, and [9] for whether the gate is running.");
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
