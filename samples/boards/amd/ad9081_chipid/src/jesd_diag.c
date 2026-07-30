/*
 * Analog-loopback datapath tests, run when Rung 5 does not pass.
 *
 * Fifteen checks over the transmit and receive chain, each reporting PASS / WARN
 * / FAIL / SKIP and ending in a summary table. They were written as a fault
 * isolation walk for one specific symptom -- the DAC return being present only
 * ~9% of the time -- and that walk is now finished. The cause is that the TX DMA
 * sustains only ~403 MB/s against the link's 4000 MB/s demand:
 *
 *   1 MiB offload buffer / 4000 MB/s =  262 us drain
 *   1 MiB offload buffer /  403 MB/s = 2602 us refill
 *                            9.2% duty, 2.86 ms period
 *
 * which reproduces the independently measured 9% duty and ~2.8 ms gate period
 * exactly. The offload core is not the fault -- it was masking one, and putting
 * it in bypass does not add throughput, it only spreads the same bytes evenly and
 * so replaces a correct-but-gated output with a continuous-but-incoherent one.
 * That is why Rung 5 now returns energy at every frequency but never the tone.
 *
 * Tests 3-5 are the measurement that established this and localise what is slow:
 * TX DMA throughput in both offload modes [3], the same for RX [4], and the CPU's
 * own DDR read rate with no DMA involved [5]. Together they say whether the
 * ceiling belongs to the transmit path, to everything crossing PL-to-PS, or to
 * DDR itself.
 *
 * The rest each still bound something independently: chip fault latches [1], TX
 * gain [2], offload state [6], frequency dependence [7], injection points along
 * the DAC datapath [8][9], the TPL input [10], and the shape of the gating
 * [11]-[15].
 *
 * [16] is the one test that tries to make the throughput limit irrelevant rather
 * than measure it: the offload core replaying its own buffer with the DMA
 * stopped, which needs no DDR bandwidth at all once filled.
 *
 * Measurement notes that matter for reading the output:
 *  - Presence is never judged from a single capture. If any gating remains, one
 *    capture samples whichever phase it landed in, so every presence test takes
 *    DIAG_SWEEP_PROBES fast probes and reports a duty cycle.
 *  - Presence is NOT correctness, and several tests only measure presence.
 *    jesd_capture_probe() thresholds a mean-square, so the broadband noise the
 *    gaps produce satisfies it just as well as the tone does. That is why [15]
 *    reports a healthy 100% duty on a datapath that is not delivering samples --
 *    read it alongside [3] and the correlator result, never alone.
 *  - Already ruled out, so not re-tested: balun band (500 kHz - 9 GHz), NCO
 *    tuning words (read back exact every run), TX channel gain (1024).
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdarg.h>
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

/* ------------------------------------------------------------------------- */
/* Result collection                                                        */
/* ------------------------------------------------------------------------- */

enum diag_verdict {
	DIAG_SKIP = 0, /* could not be measured -- says nothing either way */
	DIAG_PASS,
	DIAG_WARN, /* measured, and not what a healthy datapath gives */
	DIAG_FAIL,
};

enum diag_test {
	T_IRQ = 0,
	T_GAIN,
	T_BW,
	T_BW_RX,
	T_BW_CPU,
	T_OFFLOAD,
	T_SWEEP,
	T_MAIN_TONE,
	T_CHAN_TONE,
	T_DDS,
	T_REPEAT,
	T_DUTY_IN,
	T_PERIOD,
	T_JRX,
	T_DUTY_BYPASS,
	T_CYCLIC,
	DIAG_NUM_TESTS,
};

static const char *const diag_names[DIAG_NUM_TESTS] = {
	[T_IRQ] = "chip IRQ latches",
	[T_GAIN] = "TX fine-DUC gain",
	[T_BW] = "TX DMA bandwidth",
	[T_BW_RX] = "RX DMA bandwidth",
	[T_BW_CPU] = "CPU DDR read rate",
	[T_OFFLOAD] = "data-offload state",
	[T_SWEEP] = "NCO frequency sweep",
	[T_MAIN_TONE] = "main-DUC test tone",
	[T_CHAN_TONE] = "fine-DUC test tone",
	[T_DDS] = "FPGA DDS at TPL in",
	[T_REPEAT] = "repeatability",
	[T_DUTY_IN] = "duty within capture",
	[T_PERIOD] = "gate period",
	[T_JRX] = "JRX elastic buffer",
	[T_DUTY_BYPASS] = "output duty (bypass)",
	[T_CYCLIC] = "offload cyclic replay",
};

static struct {
	enum diag_verdict verdict;
	char detail[56];
} diag_res[DIAG_NUM_TESTS];

/* Print the header for a test, so the numbering and the summary cannot drift. */
static void diag_begin(enum diag_test t)
{
	LOG_INF("[%2u/%u] %s", (unsigned int)t + 1U, (unsigned int)DIAG_NUM_TESTS,
		diag_names[t]);
}

static void diag_report(enum diag_test t, enum diag_verdict v, const char *fmt, ...)
{
	va_list ap;

	va_start(ap, fmt);
	vsnprintk(diag_res[t].detail, sizeof(diag_res[t].detail), fmt, ap);
	va_end(ap);

	diag_res[t].verdict = v;

	switch (v) {
	case DIAG_FAIL:
		LOG_ERR("       FAIL: %s", diag_res[t].detail);
		break;
	case DIAG_WARN:
		LOG_WRN("       WARN: %s", diag_res[t].detail);
		break;
	case DIAG_SKIP:
		LOG_INF("       SKIP: %s", diag_res[t].detail);
		break;
	default:
		LOG_INF("       PASS: %s", diag_res[t].detail);
		break;
	}
}

/* ------------------------------------------------------------------------- */
/* Configuration                                                            */
/* ------------------------------------------------------------------------- */

/*
 * The frequency plan in use: TX main NCO up, RX coarse DDC down by the same
 * amount, so an analog loopback returns the tone to the baseband frequency it
 * left at.
 */
#define DIAG_NCO_HZ_DEFAULT 2000000000LL

/* Sweep points in MHz, spanning the ADC's first Nyquist zone. 1968 is last so
 * the sweep ends on the frequency the rest of the app runs at. */
static const uint32_t diag_sweep_mhz[] = {
	10,  25,  50,  75,   100,  150,  200,  300,
	400, 500, 700, 900,  1200, 1500, 1750, 1968,
};

/*
 * Probes per presence measurement.
 *
 * At ~340 us per probe, 64 probes span ~22 ms. That was chosen against the ~2.8
 * ms gate period seen before the offload fix -- eight full periods, so an
 * on-window cannot be missed by luck and each point yields a duty cycle rather
 * than a coin flip. Kept at that depth so any regression is measured the same
 * way the original fault was.
 */
#define DIAG_SWEEP_PROBES 64U

/* Recovered-tone concentration (permille) counting as "the tone came back".
 * Broadband noise scores ~13, a clean return ~999. */
#define DIAG_TONE_FOUND_MIN 200

/* Per-sample RMS above which there is definitely something at the ADC input,
 * whether or not it correlates with the tone we sent. */
#define DIAG_SIGNAL_RMS 16

/*
 * TX DMAC, addressed directly for the one thing the DMA API does not expose: the
 * core's own description of its bus widths. The driver probes INTF_DESC at init
 * and keeps the result private, and test 3's arithmetic depends on it, so it is
 * read back here rather than assumed. The page is already identity-mapped by the
 * driver's DEVICE_MMIO_MAP.
 *
 * Everything else in test 3 goes through the DMA API, which is the right way
 * round: the transfers being timed are real transfers, submitted exactly as the
 * playback path submits them.
 */
#define DIAG_DMAC_TX_BASE       0x9C430000UL
#define DIAG_DMAC_REG_INTF_DESC 0x0010

/* INTF_DESC encodes bytes-per-beat as a log2 exponent per interface: source in
 * bits 11:8, destination in bits 3:0. */
#define DIAG_INTF_BPB_SRC_MASK  GENMASK(11, 8)
#define DIAG_INTF_BPB_DEST_MASK GENMASK(3, 0)

/*
 * Bytes the link consumes per sample period: 8 converters x NP16. Confirmed
 * against the reference HDL for this build (8B10B, M8 L4 S1 NP16):
 *
 *   F = (M*S*NP)/(8*L) = 4,  DATAPATH_WIDTH = 4
 *   SAMPLES_PER_CHANNEL = (L*8*DATAPATH_WIDTH)/(M*NP) = 1
 *   dma_data_width = NP * M * SAMPLES_PER_CHANNEL = 128 bits = 16 bytes
 *
 * The "256 bits" in the block design is do_axi_data_width -- the offload's
 * memory-mapped AXI port, a different interface. Test 3 reads the real width
 * back from the core rather than trusting this.
 */
#define DIAG_BEAT_BYTES 16U

/* ------------------------------------------------------------------------- */
/* [1] Chip latched interrupt status                                         */
/* ------------------------------------------------------------------------- */

/*
 * Read the chip's own fault latches. Every other register this file consults
 * reports state (lanes locked, link in DATA, gain 1024) rather than faults.
 *
 * These bits are live without enabling anything, because
 * adi_ad9081_device_startup_tx() ends with dac_irqs_enable_set(0x0030cccc00),
 * whose bits 11/15/19/23 are PAERR0-3.
 *
 * The board's lit IRQB0 LED is not a fault indication: the measured status is
 * 0x40 00 00 43 f0 00 -- DATA_READY plus PLL/DLL lock and DLL_VTH_PASS -- and
 * DATA_READY alone holds that open-drain pin low.
 *
 * Read as six raw bytes rather than through adi_ad9081_dac_irqs_status_get(),
 * whose 0x2800 field descriptor writes 8 bytes into a uint64_t: correct on a
 * little-endian host, but not worth depending on when the point is to see
 * exactly which bits are set.
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
	unsigned int faults = 0;

	diag_begin(T_IRQ);

	for (uint8_t i = 0; i < 6; i++) {
		int32_t err = adi_ad9081_hal_reg_get(dev, REG_IRQ_STATUS0_ADDR + i,
						     &st[i]);

		if (err != API_CMS_ERROR_OK) {
			diag_report(T_IRQ, DIAG_SKIP, "IRQ_STATUS%u read failed (%d)",
				    i, err);
			return;
		}
	}

	LOG_INF("       regs 0x26..0x2b = %02x %02x %02x %02x %02x %02x", st[0],
		st[1], st[2], st[3], st[4], st[5]);

	for (uint8_t b = 0; b < 8; b++) {
		if ((st[0] & BIT(b)) && irq0[b] != NULL) {
			LOG_INF("       STATUS0 b%u %s", b, irq0[b]);
		}
		if (st[3] & BIT(b)) {
			LOG_INF("       STATUS3 b%u %s", b, irq3[b]);
		}
	}

	/*
	 * PAERR is bit 3 of each DAC's nibble pair: DAC0/1 in status1, DAC2/3 in
	 * status2, low nibble then high. Any of these set means the DAC output is
	 * blanked by PA protection, and no correct datapath would show a signal.
	 */
	for (uint8_t d = 0; d < 4; d++) {
		uint8_t reg = st[1 + (d / 2)];
		uint8_t bit = (d & 1) ? 7 : 3;

		if (reg & BIT(bit)) {
			LOG_ERR("       PAERR%u: DAC%u output blanked by PA protection",
				d, d);
			faults++;
		}
	}

	if (st[5] & 0x0f) {
		LOG_ERR("       SRERR 0x%x: slew-rate/PA error", st[5] & 0x0f);
		faults++;
	}
	if (st[0] & BIT(2)) {
		LOG_ERR("       SYSREF_JITTER: deframer/LMFC alignment untrustworthy");
		faults++;
	}
	if (st[0] & BIT(5)) {
		LOG_ERR("       LANE_FIFO: JRX lane FIFO over/underflow drops samples");
		faults++;
	}
	if (st[3] & (BIT(5) | BIT(7))) {
		LOG_ERR("       DLL_LOST: DAC clock DLL lost lock");
		faults++;
	}

	if (st[0] == 0 && st[1] == 0 && st[2] == 0 && st[3] == 0 && st[4] == 0 &&
	    st[5] == 0) {
		diag_report(T_IRQ, DIAG_SKIP,
			    "all six bytes zero (IRQ routed elsewhere?)");
		return;
	}

	if (faults) {
		diag_report(T_IRQ, DIAG_FAIL, "%u fault bit(s) latched", faults);
	} else {
		diag_report(T_IRQ, DIAG_PASS, "no fault bits (lock/ready only)");
	}
}

/* ------------------------------------------------------------------------- */
/* [2] TX fine-DUC channel gain                                              */
/* ------------------------------------------------------------------------- */

/*
 * adi_ad9081_dac_duc_nco_gains_set() wrote 1024 to channels 0-3 at datapath
 * setup. A zero gain would produce exactly the symptom under investigation: a
 * flawless link carrying silence. The register is paged per channel, so select
 * each one before reading; 12-bit field, so two bytes.
 */
static void diag_check_tx_gain(adi_ad9081_device_t *dev)
{
	unsigned int muted = 0, scaled = 0, read = 0;

	diag_begin(T_GAIN);

	for (uint8_t ch = 0; ch < 4; ch++) {
		uint8_t raw[2] = { 0, 0 };
		uint16_t gain;
		int32_t err;

		err = adi_ad9081_dac_chan_select_set(dev, AD9081_DAC_CH_0 << ch);
		if (err != API_CMS_ERROR_OK) {
			LOG_WRN("       ch%u select failed (%d)", ch, err);
			continue;
		}

		err = adi_ad9081_hal_bf_get(dev, REG_CHNL_GAIN0_ADDR,
					    BF_CHNL_GAIN_INFO, raw, 2);
		if (err != API_CMS_ERROR_OK) {
			LOG_WRN("       ch%u gain read failed (%d)", ch, err);
			continue;
		}

		gain = (uint16_t)raw[0] | ((uint16_t)raw[1] << 8);
		read++;

		if (gain == 0) {
			LOG_ERR("       ch%u gain 0 (datapath muted)", ch);
			muted++;
		} else if (gain != 1024) {
			LOG_WRN("       ch%u gain %u (expected 1024)", ch, gain);
			scaled++;
		}
	}

	if (read == 0) {
		diag_report(T_GAIN, DIAG_SKIP, "no channel gain could be read");
	} else if (muted) {
		diag_report(T_GAIN, DIAG_FAIL, "%u/%u channels muted (gain 0)", muted,
			    read);
	} else if (scaled) {
		diag_report(T_GAIN, DIAG_WARN, "%u/%u channels not at 1024", scaled,
			    read);
	} else {
		diag_report(T_GAIN, DIAG_PASS, "all %u channels at 1024", read);
	}
}

/* ------------------------------------------------------------------------- */
/* NCO retune helper                                                        */
/* ------------------------------------------------------------------------- */

/*
 * Retune the TX main NCO and RX coarse DDC to a matched pair, then verify both
 * frequency tuning words.
 *
 * The readback is the point: a sweep that only writes cannot tell "this
 * frequency does not get through" from "this frequency was never programmed" --
 * both look like silence. Expected FTW = 2^48 * f / f_clk, with the API storing
 * the two's-complement form for a negative shift.
 *
 * Every point has matched for many runs, so only a mismatch is logged.
 */
static int diag_retune(adi_ad9081_device_t *dev, int64_t hz)
{
	uint64_t tx_ftw = 0, rx_ftw = 0, mod_a = 0, mod_b = 0;
	uint64_t want_tx, want_rx;
	int32_t err;

	err = adi_ad9081_dac_duc_nco_set(dev, AD9081_DAC_ALL, AD9081_DAC_CH_NONE, hz);
	if (err != API_CMS_ERROR_OK) {
		LOG_WRN("       TX NCO retune to %lld Hz failed (%d)", (long long)hz,
			err);
		return -EIO;
	}

	err = adi_ad9081_adc_ddc_coarse_nco_set(dev, AD9081_ADC_CDDC_ALL, hz);
	if (err != API_CMS_ERROR_OK) {
		LOG_WRN("       RX DDC retune to %lld Hz failed (%d)", (long long)hz,
			err);
		return -EIO;
	}

	/* Let the retuned datapath settle before capturing through it. */
	k_msleep(2);

	/* Computed the same way the API computes them, so a mismatch means the
	 * hardware rather than our arithmetic. */
	(void)adi_ad9081_hal_calc_tx_nco_ftw(dev, dev->dev_info.dac_freq_hz, hz,
					     &want_tx);
	(void)adi_ad9081_hal_calc_rx_nco_ftw(dev, dev->dev_info.adc_freq_hz, hz,
					     &want_rx);

	err = adi_ad9081_dac_duc_main_nco_ftw_get(dev, AD9081_DAC_0, &tx_ftw, &mod_a,
						  &mod_b);
	if (err != API_CMS_ERROR_OK) {
		LOG_WRN("       TX FTW readback failed (%d)", err);
	}

	err = adi_ad9081_adc_ddc_coarse_nco_ftw_get(dev, AD9081_ADC_CDDC_0, &rx_ftw,
						    &mod_a, &mod_b);
	if (err != API_CMS_ERROR_OK) {
		LOG_WRN("       RX FTW readback failed (%d)", err);
	}

	if (tx_ftw != want_tx || rx_ftw != want_rx) {
		LOG_ERR("       FTW MISMATCH tx 0x%012llx want 0x%012llx, rx 0x%012llx want 0x%012llx",
			(unsigned long long)tx_ftw, (unsigned long long)want_tx,
			(unsigned long long)rx_ftw, (unsigned long long)want_rx);
		LOG_ERR("       this frequency was never programmed");
	}

	return 0;
}

/* ------------------------------------------------------------------------- */
/* [3] Achieved TX DMA bandwidth                                             */
/* ------------------------------------------------------------------------- */

/*
 * The link consumes one 16-byte beat per sample period: a flat 4000 MB/s the DMA
 * must sustain with no gaps. Whether it can is the single unmeasured number this
 * whole investigation rests on, so this test measures it directly.
 *
 * How, and why not the obvious way
 * --------------------------------
 * The obvious way -- watch the running cyclic tone and count buffer wraps out of
 * IRQ_PENDING -- cannot work on this core, and two runs were wasted proving it:
 *
 *  - Under cyclic=hw the hardware replays *one* transfer forever. TRANSFER_ID
 *    never increments and no fresh EOT ever latches, so the wrap count is
 *    permanently zero no matter how fast the engine is going.
 *  - EOT is write-1-to-clear, and with no IRQ line wired to the PS GIC the
 *    driver's polling path reads IRQ_PENDING and writes it straight back. Any
 *    capture or probe elsewhere in the diagnostic eats the very events being
 *    counted.
 *
 * So instead: stop the cyclic tone, run *bounded* transfers of known size, and
 * time them. A bounded transfer has a defined end that the DMA API reports, which
 * makes bytes/elapsed an actual measurement. Two things then make the number
 * trustworthy rather than merely plausible:
 *
 *  - Two sizes. Each transfer carries fixed overhead (config, arming, and the
 *    poll loop's latency in noticing completion) that does not scale with length.
 *    Differencing two sizes divides it out:
 *
 *        rate = (big - small) / (t_big - t_small)
 *
 *    The single-transfer rates are reported too, and the gap between them is
 *    itself the evidence of how much overhead there was.
 *  - Two offload modes. In bypass the sink is the TPL, which backpressures at the
 *    link's 4000 MB/s, so the answer is min(DMA, line) -- what the DAC really
 *    gets. In store-and-replay the sink is the offload's own wide storage buffer,
 *    which is faster than the link, so the answer is much closer to the DMA's raw
 *    DDR read throughput. If bypass measures well under 4000 MB/s while
 *    store-and-replay measures well above it, the starved-datapath explanation of
 *    the Rung 5 result is confirmed by measurement instead of by argument.
 *
 * The bus-width readback stays, and is checked before anything else: everything
 * in this sample is written around a 16-byte beat, and if the hardware disagrees
 * then the sample-to-lane layout in jesd_capture and jesd_playback is wrong too,
 * not just this arithmetic.
 *
 * The channel is left with the cyclic tone re-armed, because every later test
 * depends on it still playing.
 */

/* Long enough that the transfer dominates the fixed overhead, short enough to
 * stay well inside the DMA poll timeout: 4 MiB at line rate is ~1 ms. */
#define DIAG_BW_BIG_BYTES   (4U * 1024U * 1024U)
#define DIAG_BW_SMALL_BYTES (1U * 1024U * 1024U)

/*
 * Rate below which tests 4 and 5 call a measurement "about the same as TX", i.e.
 * consistent with a limit shared by everything reaching DDR rather than one
 * belonging to the transmit path.
 *
 * Set at twice the 403 MB/s measured on TX. The gap between the two candidate
 * answers is expected to be large -- a shared DDR-side ceiling would land within
 * tens of percent of TX, whereas a healthy DDR path on this part should be
 * gigabytes per second -- so the exact threshold does not matter much. It is
 * deliberately generous rather than tight: the purpose is to sort the result into
 * one of two conclusions, and a number landing near the boundary means neither
 * conclusion is safe and the raw MB/s in the log is what to read.
 */
#define DIAG_BW_SHARED_LIMIT_MBPS 800U

/* Rate in MB/s from a byte count and a microsecond duration. Bytes/us is
 * MB/s exactly, so this is just a divide -- no scaling and no overflow. */
static uint32_t diag_bw_mbps(uint64_t bytes, uint32_t us)
{
	return us ? (uint32_t)(bytes / us) : 0U;
}

/*
 * Time both sizes with the offload in whichever mode it is already in, and
 * report the sustained rate. Returns MB/s, or 0 if a transfer failed.
 *
 * Two sizes rather than one, but NOT to compute a slope. An earlier version
 * differenced them -- (big-small)/(t_big-t_small) -- to divide out fixed
 * per-transfer setup cost. On this core there is no meaningful fixed cost to
 * remove: a transfer of any size up to 16 MiB is a single descriptor
 * (DMA_LENGTH_WIDTH=24), so one dma_config/dma_start pair covers the whole
 * thing and setup is a handful of register writes against milliseconds of
 * transfer. The two raw rates come out within 3% of each other, and
 * differencing two nearly-equal numbers amplifies their noise -- the old
 * "corrected" figure read *above* both raw rates, which is not physical.
 *
 * The pair is still worth timing: agreement between them is the evidence that
 * the rate is a sustained property of the path and not an artefact of one
 * size. The larger transfer's rate is reported, since it spends the greatest
 * fraction of its time actually moving data.
 */
static uint32_t diag_bw_measure(const char *mode, size_t big, size_t small)
{
	uint32_t t_big = 0, t_small = 0, rate_big, rate_small;
	int rc;

	rc = jesd_playback_timed(small, &t_small);
	if (rc) {
		LOG_WRN("       %s: %zu B transfer failed (%d)", mode, small, rc);
		return 0;
	}

	rc = jesd_playback_timed(big, &t_big);
	if (rc) {
		LOG_WRN("       %s: %zu B transfer failed (%d)", mode, big, rc);
		return 0;
	}

	LOG_INF("       %s: %zu B in %u us (%u MB/s), %zu B in %u us (%u MB/s)", mode,
		small, t_small, diag_bw_mbps(small, t_small), big, t_big,
		diag_bw_mbps(big, t_big));

	/*
	 * Report the larger transfer, and flag it if the two disagree by more than
	 * a few percent -- that would mean the rate depends on transfer size, which
	 * this path is not expected to do and which would make a single figure
	 * misleading.
	 */
	rate_big = diag_bw_mbps(big, t_big);
	rate_small = diag_bw_mbps(small, t_small);

	if (rate_small > rate_big + rate_big / 8U ||
	    rate_big > rate_small + rate_small / 8U) {
		LOG_WRN("       %s: rate varies with size (%u vs %u MB/s); not a flat ceiling",
			mode, rate_small, rate_big);
	}

	LOG_INF("       %s: sustained %u MB/s", mode, rate_big);
	return rate_big;
}

static void diag_tx_bandwidth(void)
{
	uintptr_t dmac = DIAG_DMAC_TX_BASE;
	uint32_t intf, w_src, w_dst, r_bypass, r_store;
	uint32_t demand;
	const int16_t *buf;
	size_t buf_bytes, big, small;

	diag_begin(T_BW);

	/* Read from the playback module rather than recomputed, so the two cannot
	 * drift apart. */
	if (jesd_playback_buffer(&buf, &buf_bytes) != 0 || buf_bytes == 0) {
		diag_report(T_BW, DIAG_SKIP, "playback buffer size unavailable");
		return;
	}

	intf = sys_read32(dmac + DIAG_DMAC_REG_INTF_DESC);
	w_src = 1U << FIELD_GET(DIAG_INTF_BPB_SRC_MASK, intf);
	w_dst = 1U << FIELD_GET(DIAG_INTF_BPB_DEST_MASK, intf);

	LOG_INF("       bus width src %u B dst %u B (INTF_DESC 0x%08x)", w_src, w_dst,
		intf);

	if (w_src != DIAG_BEAT_BYTES || w_dst != DIAG_BEAT_BYTES) {
		diag_report(T_BW, DIAG_FAIL, "beat is %u B, sample assumes %u B", w_src,
			    DIAG_BEAT_BYTES);
		return;
	}

	demand = (uint32_t)((uint64_t)JESD_PB_SAMPLE_RATE * w_src / 1000000ULL);

	big = MIN((size_t)DIAG_BW_BIG_BYTES, buf_bytes);
	small = MIN((size_t)DIAG_BW_SMALL_BYTES, buf_bytes / 2U);
	if (small == 0 || big <= small) {
		diag_report(T_BW, DIAG_SKIP, "playback buffer too small to time");
		return;
	}

	/* As the datapath actually runs: TPL sink, so min(DMA, line rate). */
	r_bypass = diag_bw_measure("bypass (TPL sink)", big, small);

	/*
	 * Now against the offload's storage buffer instead. That sink is wider and
	 * faster than the link, so this isolates the DMA's own DDR read rate from
	 * whatever the link is willing to accept.
	 */
	if (axi_data_offload_bypass(false) == 0) {
		uint64_t mem = 0;
		size_t s_big = big, s_small = small;

		/*
		 * Stay inside one bufferful. Store-and-replay is a fill/drain cycle:
		 * the core takes a buffer, then withholds its transfer request while
		 * it streams that out at line rate. A transfer longer than the buffer
		 * therefore spans at least one drain and measures the average of the
		 * two, which is the link rate again -- exactly the thing this mode was
		 * chosen to get away from. Both sizes must fit, and the small one must
		 * stay meaningfully smaller than the big one or the difference is noise.
		 */
		if (axi_data_offload_tx_size(&mem) == 0 && mem >= 4U * DIAG_BEAT_BYTES) {
			if ((uint64_t)s_big > mem) {
				s_big = (size_t)mem;
				s_small = s_big / 4U;
				s_small = ROUND_DOWN(s_small, DIAG_BEAT_BYTES);
				LOG_INF("       store-and-replay: capped to the %llu B buffer (%zu/%zu B)",
					(unsigned long long)mem, s_small, s_big);
			}
		} else {
			LOG_WRN("       store-and-replay: buffer size unknown; result may");
			LOG_WRN("       include a drain phase and read as the link rate");
		}

		if (s_small >= DIAG_BEAT_BYTES && s_big > s_small) {
			r_store = diag_bw_measure("store-and-replay (buffer sink)", s_big,
						  s_small);
		} else {
			r_store = 0;
			LOG_WRN("       store-and-replay: buffer too small to time");
		}
		if (axi_data_offload_bypass(true) != 0) {
			LOG_ERR("       could not restore bypass -- later tests will see");
			LOG_ERR("       a gated datapath");
		}
	} else {
		r_store = 0;
		LOG_WRN("       could not switch to store-and-replay; DMA rate not isolated");
	}

	/* Every later test needs the continuous tone back. */
	if (jesd_playback_rearm() != 0) {
		LOG_ERR("       could not re-arm cyclic playback -- the DAC is now idle");
	}

	LOG_INF("       demand %u MB/s (%u MSPS x %u B, no gaps allowed)", demand,
		JESD_PB_SAMPLE_RATE / 1000000U, w_src);

	if (r_bypass == 0) {
		diag_report(T_BW, DIAG_SKIP, "no transfer completed; rate unmeasured");
		return;
	}

	/*
	 * The verdict is about the bypass figure, because that is the rate the DAC
	 * experiences. Within 5% of demand means the datapath is fed; anything
	 * materially below it means the TPL is being starved, and the store-and-
	 * replay figure says whether the DMA or the link is the reason.
	 */
	if (r_bypass * 100U >= demand * 95U) {
		diag_report(T_BW, DIAG_PASS, "%u MB/s = %u%% of demand", r_bypass,
			    r_bypass * 100U / demand);
	} else if (r_store > demand) {
		diag_report(T_BW, DIAG_FAIL, "%u MB/s (%u%%); DMA can do %u to a buffer",
			    r_bypass, r_bypass * 100U / demand, r_store);
	} else {
		/*
		 * Deliberately not "DMA-limited". Test 4 measures the receive DMAC at
		 * essentially this same rate, and test 5 has the CPU reading DDR many
		 * times faster, so the ceiling belongs to the path the two DMACs share
		 * on their way to memory -- not to either core and not to DDR itself.
		 */
		diag_report(T_BW, DIAG_FAIL,
			    "%u MB/s = %u%% of demand; DDR-side path limit, see [4] and [5]",
			    r_bypass, r_bypass * 100U / demand);
	}
}

/* ------------------------------------------------------------------------- */
/* [4] Achieved RX DMA bandwidth                                             */
/* ------------------------------------------------------------------------- */

/*
 * The same measurement in the receive direction, and the first of two tests that
 * exist to localise the 403 MB/s test 3 measures.
 *
 * The two DMACs are separate cores with separate AXI masters, but both reach DDR
 * through the PS memory interconnect, and in the reference block design both are
 * clocked from the same sys_dma_clk. So:
 *
 *   RX also ~400 MB/s -> the limit is shared and downstream of both cores: the
 *                        PS-side port, the clock feeding it, or the coherency
 *                        routing. Nothing about the transmit path in particular,
 *                        and no amount of TX-side configuration will move it.
 *   RX much faster     -> the limit is TX-specific, and the search narrows to one
 *                        core's configuration.
 *
 * Either answer is worth having and neither requires a new bitstream, which is
 * why this runs before anything more speculative.
 *
 * Capped at the RX offload's 1 MiB one-shot buffer: above that a capture
 * truncates silently (see jesd_capture.c), which would read as a suspiciously
 * fast transfer rather than an error. Two sizes as before, to difference out the
 * fixed per-transfer overhead.
 */
#define DIAG_BW_RX_BIG_BYTES   (1024U * 1024U)
#define DIAG_BW_RX_SMALL_BYTES (256U * 1024U)

static void diag_rx_bandwidth(void)
{
	uint32_t t_big = 0, t_small = 0, corrected;
	int rc;

	diag_begin(T_BW_RX);

	rc = jesd_capture_timed(DIAG_BW_RX_SMALL_BYTES, &t_small);
	if (rc == 0) {
		rc = jesd_capture_timed(DIAG_BW_RX_BIG_BYTES, &t_big);
	}
	if (rc != 0) {
		diag_report(T_BW_RX, DIAG_SKIP, "capture failed (%d)", rc);
		return;
	}

	LOG_INF("       %u B in %u us (%u MB/s), %u B in %u us (%u MB/s)",
		DIAG_BW_RX_SMALL_BYTES, t_small,
		diag_bw_mbps(DIAG_BW_RX_SMALL_BYTES, t_small), DIAG_BW_RX_BIG_BYTES,
		t_big, diag_bw_mbps(DIAG_BW_RX_BIG_BYTES, t_big));

	/* Larger transfer, no slope -- see diag_bw_measure() for why differencing
	 * the two sizes was wrong here. */
	corrected = diag_bw_mbps(DIAG_BW_RX_BIG_BYTES, t_big);
	LOG_INF("       sustained %u MB/s", corrected);

	if (corrected == 0) {
		diag_report(T_BW_RX, DIAG_SKIP, "no measurable duration");
		return;
	}

	/*
	 * Not a pass/fail about health -- receive has no continuous-rate
	 * requirement, since its offload is one-shot and a capture is bounded by
	 * construction. The verdict reports which of the two conclusions the number
	 * supports, so the summary line carries the finding.
	 */
	if (corrected < DIAG_BW_SHARED_LIMIT_MBPS) {
		diag_report(T_BW_RX, DIAG_WARN, "%u MB/s: shared DDR-side limit, not TX",
			    corrected);
	} else {
		diag_report(T_BW_RX, DIAG_PASS, "%u MB/s: RX is fast, TX limit is local",
			    corrected);
	}
}

/* ------------------------------------------------------------------------- */
/* [5] CPU DDR read rate                                                     */
/* ------------------------------------------------------------------------- */

/*
 * The second localising measurement, and the one that needs no DMA at all: how
 * fast can the A53 itself stream DDR?
 *
 * This bounds the problem from the other end. If the CPU also gets a few hundred
 * MB/s then DDR or its controller configuration is slow for everyone, the DMACs
 * are innocent, and the fix (if there is one) is in memory-controller or
 * interconnect setup rather than anywhere in this sample. If the CPU gets
 * gigabytes per second -- which is what this part should do -- then DDR is fine
 * and the limit lives in the PL-to-PS path the DMACs use.
 *
 * Cache-line reads, not cached reads. The point is to touch DDR, so the range is
 * invalidated first and then read once with a stride of one cache line, which
 * forces a fill per line and defeats the prefetcher's ability to hide latency
 * only partially -- so this is a *lower* bound on what DDR can do, and a
 * comfortably fast result is therefore conclusive while a slow one is suggestive.
 * The playback buffer is reused as the source: it is already large, already in
 * DDR, and is not needed between transfers.
 *
 * The accumulator is volatile so the whole loop cannot be optimised away.
 */
#define DIAG_CPU_STRIDE 64U /* CONFIG_DCACHE_LINE_SIZE on this part */

static void diag_cpu_ddr_rate(void)
{
	const int16_t *buf;
	size_t bytes;
	volatile uint32_t sink = 0;
	uint64_t t0;
	uint32_t us, rate;

	diag_begin(T_BW_CPU);

	if (jesd_playback_buffer(&buf, &bytes) != 0 || bytes < DIAG_CPU_STRIDE) {
		diag_report(T_BW_CPU, DIAG_SKIP, "no buffer to read");
		return;
	}

	/* Drop the lines so every read below has to go to memory. */
	sys_cache_data_invd_range((void *)buf, bytes);

	t0 = k_cycle_get_64();
	for (size_t off = 0; off < bytes; off += DIAG_CPU_STRIDE) {
		sink += (uint32_t)*(const volatile int16_t *)((const uint8_t *)buf + off);
	}
	us = (uint32_t)k_cyc_to_us_floor64(k_cycle_get_64() - t0);

	rate = diag_bw_mbps(bytes, us);
	LOG_INF("       %zu B touched one line at a time in %u us -> %u MB/s", bytes, us,
		rate);

	if (rate == 0) {
		diag_report(T_BW_CPU, DIAG_SKIP, "no measurable duration");
	} else if (rate < DIAG_BW_SHARED_LIMIT_MBPS) {
		diag_report(T_BW_CPU, DIAG_WARN, "%u MB/s: DDR itself is slow for everyone",
			    rate);
	} else {
		diag_report(T_BW_CPU, DIAG_PASS, "%u MB/s: DDR is fine, limit is the PL path",
			    rate);
	}
}

/* ------------------------------------------------------------------------- */
/* [6] Data-offload core state                                               */
/* ------------------------------------------------------------------------- */

/*
 * The cores' own registers, as the running playback left them. Includes the
 * overflow and underflow flags, which answer the starvation question in
 * hardware rather than by inference -- and which test 3 cannot answer at all on
 * this core. axi_data_offload.c owns the register map; nothing is duplicated
 * here.
 */
static void diag_offload_state(void)
{
	diag_begin(T_OFFLOAD);

	if (axi_data_offload_status() != 0) {
		diag_report(T_OFFLOAD, DIAG_FAIL, "cores could not be read");
		return;
	}
	diag_report(T_OFFLOAD, DIAG_PASS, "both cores readable (see bypass= above)");
}

/* ------------------------------------------------------------------------- */
/* [7] NCO frequency sweep                                                   */
/* ------------------------------------------------------------------------- */

/*
 * Sweep the matched NCO pair and measure, at each point, what fraction of the
 * time the tone comes back. The baseband tone Rung 4 plays does not move -- only
 * the RF carrier it rides on -- so the correlator stays valid at every point
 * without touching the playback buffer.
 *
 * Two instruments per point, because neither alone is enough if the return is
 * gated: DIAG_SWEEP_PROBES fast probes give the duty cycle (how often anything
 * is there), then, only where the duty is non-zero, up to DIAG_SWEEP_ID_TRIES
 * full measurements identify what is there. The retries exist because a full
 * measurement is itself a coin flip against any gating.
 *
 * Result before the offload fix: duty flat at ~9% across all sixteen points,
 * which is what disproved frequency dependence.
 */
#define DIAG_SWEEP_ID_TRIES 12U

static void diag_sweep(adi_ad9081_device_t *dev)
{
	size_t points = 0, tone_points = 0, silent_points = 0;
	size_t total_on = 0, total_probes = 0;

	diag_begin(T_SWEEP);

	for (size_t i = 0; i < ARRAY_SIZE(diag_sweep_mhz); i++) {
		int64_t hz = (int64_t)diag_sweep_mhz[i] * 1000000LL;
		struct jesd_loopback_meas m;
		size_t on = 0, taken = 0;
		uint32_t best;

		if (diag_retune(dev, hz) != 0) {
			continue;
		}

		for (size_t p = 0; p < DIAG_SWEEP_PROBES; p++) {
			int r = jesd_capture_probe();

			if (r < 0) {
				LOG_WRN("       %4u MHz probe failed (%d)",
					diag_sweep_mhz[i], r);
				break;
			}
			on += (size_t)r;
			taken++;
		}

		if (taken == 0) {
			continue;
		}

		points++;
		total_on += on;
		total_probes += taken;

		if (on == 0) {
			/* Real statement, not a missed window: DIAG_SWEEP_PROBES
			 * spans several periods of any gating seen so far. */
			LOG_INF("       %4u MHz  duty %2zu/%zu (%3zu%%)  silent",
				diag_sweep_mhz[i], on, taken, on * 100U / taken);
			silent_points++;
			continue;
		}

		/* Something is arriving -- identify it, retrying against any gate. */
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
			LOG_INF("       %4u MHz  duty %2zu/%zu (%3zu%%)  energy, unidentified",
				diag_sweep_mhz[i], on, taken, on * 100U / taken);
			continue;
		}

		best = MAX(m.concentration, m.concentration_split);

		LOG_INF("       %4u MHz  duty %2zu/%zu (%3zu%%)  RMS %llu, tone %u/1000  %s",
			diag_sweep_mhz[i], on, taken, on * 100U / taken,
			(unsigned long long)m.rms, best,
			(best >= DIAG_TONE_FOUND_MIN) ? "our tone" : "wrong bin");

		if (best >= DIAG_TONE_FOUND_MIN) {
			tone_points++;
		}
	}

	if (points == 0) {
		diag_report(T_SWEEP, DIAG_SKIP, "no sweep point could be measured");
	} else if (tone_points == 0) {
		diag_report(T_SWEEP, DIAG_FAIL, "tone at 0/%zu points, %zu silent",
			    points, silent_points);
	} else if (total_on * 10U < total_probes * 9U) {
		diag_report(T_SWEEP, DIAG_WARN, "tone at %zu/%zu points, mean duty %zu%%",
			    tone_points, points, total_on * 100U / total_probes);
	} else {
		diag_report(T_SWEEP, DIAG_PASS, "tone at %zu/%zu points, mean duty %zu%%",
			    tone_points, points, total_on * 100U / total_probes);
	}
}

/* ------------------------------------------------------------------------- */
/* [8] Chip main-datapath DC test tone                                       */
/* ------------------------------------------------------------------------- */

/*
 * Drive the DAC from its own internal calibration DC input instead of from the
 * JESD stream. The main-datapath NCO upconverts that DC to a tone at the NCO
 * frequency, so a signal appears at the DAC output having touched none of our
 * DMA buffer, the TPL core, the serial link or the deframer.
 *
 * A return here exonerates the DAC output stage, balun, cable, ADC, RX DDC,
 * framer, RX link and RX DMA, plus the TX main NCO. Silence in both this and
 * test 5 points at the DAC output stage or the board path between the two SMAs,
 * which needs a scope.
 *
 * Judged on RMS, not the correlator: a DC offset upconverted by the NCO lands
 * at the NCO frequency, which the RX DDC shifts straight to baseband DC rather
 * than to the bin the correlator watches.
 */
static bool diag_internal_tone(adi_ad9081_device_t *dev)
{
	struct jesd_loopback_meas m;
	int32_t err;
	bool present = false;
	int ret;

	diag_begin(T_MAIN_TONE);

	if (diag_retune(dev, DIAG_NCO_HZ_DEFAULT) != 0) {
		diag_report(T_MAIN_TONE, DIAG_SKIP, "could not restore the NCO pair");
		return false;
	}

	err = adi_ad9081_dac_duc_main_dc_test_tone_offset_set(dev, AD9081_DAC_ALL,
							      0x4000);
	if (err != API_CMS_ERROR_OK) {
		diag_report(T_MAIN_TONE, DIAG_SKIP, "tone offset set failed (%d)", err);
		return false;
	}

	err = adi_ad9081_dac_duc_main_dc_test_tone_en_set(dev, AD9081_DAC_ALL, 1);
	if (err != API_CMS_ERROR_OK) {
		diag_report(T_MAIN_TONE, DIAG_SKIP, "tone enable failed (%d)", err);
		return false;
	}

	k_msleep(2);

	ret = jesd_loopback_measure(&m);
	if (ret == 0) {
		LOG_INF("       %lld MHz: RMS %llu, lanes %llu/%llu/%llu/%llu/%llu/%llu/%llu/%llu",
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

		present = (m.rms >= DIAG_SIGNAL_RMS);
		if (present) {
			diag_report(T_MAIN_TONE, DIAG_PASS, "returned, RMS %llu",
				    (unsigned long long)m.rms);
		} else {
			diag_report(T_MAIN_TONE, DIAG_FAIL, "silent, RMS %llu",
				    (unsigned long long)m.rms);
		}
	} else {
		diag_report(T_MAIN_TONE, DIAG_SKIP, "capture failed (%d)", ret);
	}

	/* Always off again -- leaving it on would corrupt every later measurement
	 * with a signal the datapath never sent. */
	err = adi_ad9081_dac_duc_main_dc_test_tone_en_set(dev, AD9081_DAC_ALL, 0);
	if (err != API_CMS_ERROR_OK) {
		LOG_WRN("       could not disable the test tone (%d)", err);
	}

	return present;
}

/* ------------------------------------------------------------------------- */
/* [9] Chip fine-DUC DC test tone                                            */
/* ------------------------------------------------------------------------- */

/*
 * The same trick one stage earlier. Test 6 injects downstream of both the
 * deframer and the fine DUC, so its success narrows the fault only to
 * "somewhere before the main DUC". This injects downstream of the deframer but
 * upstream of the main DUC, splitting that span:
 *
 *   returns -> the fine DUC and everything after it are fine
 *   silent  -> the fine DUC stage is where the signal dies, despite its gain
 *              reading back as the programmed 1024
 */
static void diag_channel_tone(adi_ad9081_device_t *dev)
{
	struct jesd_loopback_meas m;
	int32_t err;

	diag_begin(T_CHAN_TONE);

	err = adi_ad9081_dac_dc_test_tone_offset_set(dev, AD9081_DAC_CH_0, 0x4000);
	if (err != API_CMS_ERROR_OK) {
		diag_report(T_CHAN_TONE, DIAG_SKIP, "tone offset failed (%d)", err);
		return;
	}

	err = adi_ad9081_dac_dc_test_tone_en_set(dev, AD9081_DAC_CH_0, 1);
	if (err != API_CMS_ERROR_OK) {
		diag_report(T_CHAN_TONE, DIAG_SKIP, "tone enable failed (%d)", err);
		return;
	}

	k_msleep(2);

	if (jesd_loopback_measure(&m) == 0) {
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

		if (m.rms >= DIAG_SIGNAL_RMS) {
			diag_report(T_CHAN_TONE, DIAG_PASS, "returned, RMS %llu",
				    (unsigned long long)m.rms);
		} else {
			diag_report(T_CHAN_TONE, DIAG_FAIL,
				    "silent: signal dies at the fine DUC");
		}
	} else {
		diag_report(T_CHAN_TONE, DIAG_SKIP, "capture failed");
	}

	err = adi_ad9081_dac_dc_test_tone_en_set(dev, AD9081_DAC_CH_0, 0);
	if (err != API_CMS_ERROR_OK) {
		LOG_WRN("       could not disable the channel test tone (%d)", err);
	}
}

/* ------------------------------------------------------------------------- */
/* [10] FPGA DDS at the TPL input                                            */
/* ------------------------------------------------------------------------- */

/*
 * Drive the DAC from the FPGA's own DDS generator instead of from DDR. This is
 * the other half of the split tests 6 and 7 opened: those inject inside the
 * chip, downstream of the deframer, so they say nothing about whether the
 * deframer hands our samples on. The DDS enters at the opposite end -- inside
 * the FPGA, at the TPL input, upstream of the transport core, the lanes and the
 * deframer.
 *
 * The useful output is the comparison, not a verdict on its own. The DDS is a
 * continuous source, so its duty cycle isolates which side of the TPL any gate
 * is on: gated here means at or after the TPL input, continuous means upstream,
 * in how our samples reach that input.
 *
 * Judged on RMS rather than the correlator: the DDS frequency is quantised by a
 * 16-bit phase accumulator and does not land on the bin the correlator watches.
 */
static void diag_fpga_dds(void)
{
	struct jesd_loopback_meas m;
	size_t on = 0, taken = 0;
	int ret;

	diag_begin(T_DDS);

	ret = axi_tpl_tx_dds(JESD_PB_TONE_HZ, JESD_PB_SAMPLE_RATE, true);
	if (ret) {
		diag_report(T_DDS, DIAG_SKIP, "could not arm the FPGA DDS (%d)", ret);
		return;
	}

	/* The TPL SYNC and the chip's deframer need a moment to settle on the new
	 * data source before the capture means anything. */
	k_msleep(5);

	for (size_t p = 0; p < DIAG_SWEEP_PROBES; p++) {
		int r = jesd_capture_probe();

		if (r < 0) {
			LOG_WRN("       probe failed (%d)", r);
			break;
		}
		on += (size_t)r;
		taken++;
	}

	if (jesd_loopback_measure(&m) == 0) {
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
	}

	if (taken == 0) {
		diag_report(T_DDS, DIAG_SKIP, "no probe completed");
	} else if (on == taken) {
		diag_report(T_DDS, DIAG_PASS, "continuous, duty %zu/%zu", on, taken);
	} else if (on > 0) {
		diag_report(T_DDS, DIAG_WARN,
			    "gated at %zu%%: gate is at/after the TPL input",
			    on * 100U / taken);
	} else {
		diag_report(T_DDS, DIAG_FAIL,
			    "silent: DDS is not driving the converters");
	}

	/* Put the converters back on the DMA source whatever happened. */
	(void)axi_tpl_tx_dds(0, 0, false);
}

/* ------------------------------------------------------------------------- */
/* [11] Repeatability                                                        */
/* ------------------------------------------------------------------------- */

/*
 * Repeat one measurement of our own tone at the configured frequency.
 *
 * Across two boots the identical 1968 MHz point returned RMS 4567 once and RMS
 * 8 the next time, with the tuning words verified identical both times. That
 * rules out static misconfiguration -- a wrong register value does not work
 * once and then stop -- so this counts how often it works rather than which
 * setting is wrong. Partial hits point at something that has to line up in
 * time rather than in configuration.
 */
#define DIAG_REPEATS 8

static void diag_repeatability(void)
{
	uint32_t hits = 0, taken = 0;
	uint64_t best_rms = 0;

	diag_begin(T_REPEAT);

	for (uint32_t i = 0; i < DIAG_REPEATS; i++) {
		struct jesd_loopback_meas m;

		if (jesd_loopback_measure(&m) != 0) {
			continue;
		}
		taken++;

		if (m.rms > best_rms) {
			best_rms = m.rms;
		}
		if (m.rms >= DIAG_SIGNAL_RMS) {
			hits++;
		}

		LOG_INF("       #%u RMS %llu, tone %u/1000", i,
			(unsigned long long)m.rms, m.concentration);
	}

	if (taken == 0) {
		diag_report(T_REPEAT, DIAG_SKIP, "no capture completed");
	} else if (hits == taken) {
		diag_report(T_REPEAT, DIAG_PASS, "%u/%u saw signal, best RMS %llu", hits,
			    taken, (unsigned long long)best_rms);
	} else if (hits == 0) {
		diag_report(T_REPEAT, DIAG_FAIL, "0/%u saw signal", taken);
	} else {
		diag_report(T_REPEAT, DIAG_WARN, "intermittent: %u/%u saw signal", hits,
			    taken);
	}
}

/* ------------------------------------------------------------------------- */
/* [12] Duty cycle within one capture                                        */
/* ------------------------------------------------------------------------- */

/*
 * Scan a single capture in fixed-size chunks and report each chunk's RMS.
 *
 * Test 9 counts how many captures saw signal, and that count conflates "the
 * signal is rarely there" with "we rarely looked while it was there". This
 * separates them, since a mixed result dates a transition inside one window.
 *
 * Findings so far: at 65 us every chunk read uniformly high or uniformly low,
 * never a mix, which killed the "brief bursts averaging to the noise floor"
 * theory. At 262 us -- the most a single transfer carries on this core -- still
 * uniform, which is what forced the across-captures measurement in test 11.
 *
 * Chunks are a multiple of the 8-beat tone period, so a chunk boundary never
 * splits the tone in a way that depresses its RMS.
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

#define DIAG_CHUNK_BEATS    4096U
#define DIAG_CHUNKS_MAX     64U
#define DIAG_CHUNKS_PER_ROW 16U

static void diag_duty_cycle(void)
{
	const int16_t *buf;
	size_t n, beats, chunks, on = 0;
	uint64_t rms[DIAG_CHUNKS_MAX];
	uint32_t span_us;

	diag_begin(T_DUTY_IN);

	if (jesd_capture_raw(&buf, &n) != 0) {
		diag_report(T_DUTY_IN, DIAG_SKIP, "capture failed");
		return;
	}

	beats = n / JESD_CAP_LANES_PER_BEAT;
	chunks = beats / DIAG_CHUNK_BEATS;
	if (chunks == 0) {
		diag_report(T_DUTY_IN, DIAG_SKIP, "capture shorter than one chunk (%zu beats)",
			    beats);
		return;
	}
	if (chunks > DIAG_CHUNKS_MAX) {
		chunks = DIAG_CHUNKS_MAX;
	}

	for (size_t c = 0; c < chunks; c++) {
		uint64_t energy = 0;
		size_t base = c * DIAG_CHUNK_BEATS * JESD_CAP_LANES_PER_BEAT;

		for (size_t s = 0; s < DIAG_CHUNK_BEATS * JESD_CAP_LANES_PER_BEAT; s++) {
			int32_t v = buf[base + s];

			energy += (uint64_t)((int64_t)v * v);
		}
		rms[c] = diag_isqrt(energy /
				    (DIAG_CHUNK_BEATS * JESD_CAP_LANES_PER_BEAT));
		if (rms[c] >= DIAG_SIGNAL_RMS) {
			on++;
		}
	}

	span_us = (uint32_t)((uint64_t)chunks * DIAG_CHUNK_BEATS * 1000000U /
			     JESD_PB_SAMPLE_RATE);

	if (on == chunks) {
		diag_report(T_DUTY_IN, DIAG_PASS, "signal throughout %u us (%zu chunks)",
			    span_us, chunks);
		return;
	}
	if (on == 0) {
		diag_report(T_DUTY_IN, DIAG_FAIL, "noise floor throughout %u us", span_us);
		return;
	}

	/* Mixed: dump the per-chunk RMS, since the shape is the measurement. */
	LOG_INF("       per-chunk RMS (%u beats = %u ns each):", DIAG_CHUNK_BEATS,
		(unsigned int)((uint64_t)DIAG_CHUNK_BEATS * 1000000000U /
			       JESD_PB_SAMPLE_RATE));
	for (size_t c = 0; c < chunks; c += DIAG_CHUNKS_PER_ROW) {
		char row[DIAG_CHUNKS_PER_ROW * 8 + 1];
		size_t len = 0;

		for (size_t k = 0; k < DIAG_CHUNKS_PER_ROW && c + k < chunks; k++) {
			len += snprintk(&row[len], sizeof(row) - len, "%6llu ",
					(unsigned long long)rms[c + k]);
		}
		LOG_INF("       [%02zu] %s", c, row);
	}

	{
		size_t edges = 0, longest_on = 0, run = 0;

		/* Two edges in a window mean a full on-period is contained in it,
		 * so its length is measured rather than bounded below. */
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

		LOG_INF("       %zu transitions, longest on-run %zu chunks (%u us)%s",
			edges, longest_on,
			(unsigned int)((uint64_t)longest_on * DIAG_CHUNK_BEATS *
				       1000000U / JESD_PB_SAMPLE_RATE),
			(edges >= 2) ? ", on-time measured" : ", on-time is a lower bound");

		diag_report(T_DUTY_IN, DIAG_WARN, "gated: %zu/%zu chunks on (%zu%%)", on,
			    chunks, on * 100U / chunks);
	}
}

/* ------------------------------------------------------------------------- */
/* [13] Gate period across captures                                          */
/* ------------------------------------------------------------------------- */

/*
 * Period of any gating, measured across many captures instead of within one.
 * Test 10 cannot do this: a single transfer caps at 1 MiB (~262 us) on this
 * core, so no one capture spans a full cycle.
 *
 * Test 10's limitation becomes the instrument here. Because a capture shorter
 * than the period reads all-or-nothing, each one is a clean one-bit sample of
 * the gate state, and the run lengths in the strip are the on and off times.
 * Sampling is uneven (each capture costs its own arming and cache maintenance),
 * so the strip measures the period in units of captures and the elapsed wall
 * time converts it to microseconds.
 *
 * Timestamps are recorded because two earlier versions of this check reported
 * pure aliasing as a period: 13.7 ms per sample against a faster source gave a
 * regular 4-sample beat, and 340 us against a ~300 us on-time missed whole
 * pulses. The undersampling warning below makes that visible rather than
 * silent.
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
	int64_t t0, elapsed_us, interval_us;

	diag_begin(T_PERIOD);

	t0 = k_uptime_ticks();
	for (size_t i = 0; i < DIAG_STRIP_SAMPLES; i++) {
		int p = jesd_capture_probe();

		if (p < 0) {
			LOG_WRN("       probe failed at sample %zu (%d)", i, p);
			break;
		}
		state[i] = (uint8_t)p;
		stamp[i] = k_ticks_to_us_floor64(k_uptime_ticks() - t0);
		taken++;
	}
	elapsed_us = taken ? stamp[taken - 1] : 0;

	if (taken < 2) {
		diag_report(T_PERIOD, DIAG_SKIP, "too few samples (%zu)", taken);
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

	interval_us = elapsed_us / (int64_t)taken;

	LOG_INF("       %zu/%zu probes saw signal (%zu%%), %zu transitions in %lld us",
		on, taken, on * 100U / taken, edges, (long long)elapsed_us);
	LOG_INF("       sample interval %lld us", (long long)interval_us);

	/* Only worth dumping the strip when there is structure in it. */
	if (edges) {
		for (size_t i = 0; i < taken; i += DIAG_STRIP_PER_ROW) {
			char row[DIAG_STRIP_PER_ROW + 1];
			size_t k = 0;

			for (; k < DIAG_STRIP_PER_ROW && i + k < taken; k++) {
				row[k] = state[i + k] ? '#' : '.';
			}
			row[k] = '\0';
			LOG_INF("       %s", row);
		}
	}

	/*
	 * Say so when the sampling is too coarse for the runs being reported. An
	 * instrument that cannot detect its own aliasing reports a plausible
	 * number instead of an honest failure.
	 */
	if (on && interval_us * (int64_t)on * 4 >= elapsed_us) {
		LOG_WRN("       UNDERSAMPLED: on-time comparable to the sample interval,");
		LOG_WRN("       so intervals below are multiples of the true period");
	}

	if (edges == 0) {
		if (on == taken) {
			diag_report(T_PERIOD, DIAG_PASS,
				    "continuous over %lld us, no gating",
				    (long long)elapsed_us);
		} else {
			diag_report(T_PERIOD, DIAG_FAIL, "silent over %lld us",
				    (long long)elapsed_us);
		}
		return;
	}

	/*
	 * Report the interval between consecutive edges rather than a mean. A
	 * gate with unequal on and off times alternates between two values, and
	 * an average of the two describes neither; the sequence also makes a
	 * non-periodic source obvious.
	 */
	{
		size_t shown = MIN(edges, DIAG_EDGES_MAX);

		if (shown >= 2) {
			char row[DIAG_STRIP_PER_ROW + 1];
			size_t len = 0;

			LOG_INF("       intervals between transitions (us):");
			for (size_t e = 1; e < shown; e++) {
				len += snprintk(&row[len], sizeof(row) - len, "%lld ",
						(long long)(edge_us[e] - edge_us[e - 1]));
				if (len > sizeof(row) - 12 || e == shown - 1) {
					LOG_INF("         %s", row);
					len = 0;
				}
			}
		}
	}

	diag_report(T_PERIOD, DIAG_WARN, "gated: %zu%% on, %zu transitions in %lld us",
		    on * 100U / taken, edges, (long long)elapsed_us);
}

/* ------------------------------------------------------------------------- */
/* [14] JRX transport-layer elastic buffer                                   */
/* ------------------------------------------------------------------------- */

/*
 * The JRX TPL has a buffer-protection mechanism that withholds data when the
 * LMFC/elastic-buffer phase is marginal, controlled by two bits in JRX_TPL_1
 * (0x4A1):
 *
 *   bit7 BUF_PROTECTION - the ADI API clears this on the 204B path
 *   bit6 BUF_PROTECT_EN - cleared only for 204C on rev<3 in no-OS; on 204B,
 *                         which this link runs, neither no-OS nor the vendor
 *                         API ever writes it
 *
 * That made it a strong candidate: a mechanism gating sample delivery, on the
 * interface the fault was narrowed to, that nothing in the port configured. It
 * was disproved -- jesd_fsm.c now clears it, the readback confirms 0x41 -> 0x01,
 * and the gating did not change. Kept as a regression check on that write.
 *
 * JRX_TPL_5 (0x4A5) holds PHASE_DIFF, the phase between the arriving link frame
 * and the local LMFC. Sampling it repeatedly separates "marginal and wandering"
 * from "stable, so something else gates the data". Measured stable at 4.
 */
#define DIAG_JRX_TPL_SAMPLES 16U

static void diag_jrx_buffer(void)
{
	adi_ad9081_device_t *dev = ad9081_get_device();
	uint8_t tpl1 = 0, tpl5 = 0;
	uint8_t pd_min = 0xFF, pd_max = 0;
	int32_t err;

	diag_begin(T_JRX);

	if (dev == NULL) {
		diag_report(T_JRX, DIAG_SKIP, "device not initialised");
		return;
	}

	err = adi_ad9081_hal_reg_get(dev, REG_JRX_TPL_1_ADDR, &tpl1);
	if (err != API_CMS_ERROR_OK) {
		diag_report(T_JRX, DIAG_SKIP, "could not read 0x4A1 (%d)", err);
		return;
	}

	LOG_INF("       0x4A1 = 0x%02x: BUF_PROTECTION=%u BUF_PROTECT_EN=%u SYSREF_IGNORE=%u",
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

	LOG_INF("       PHASE_DIFF over %u samples: min %u max %u",
		DIAG_JRX_TPL_SAMPLES, pd_min, pd_max);

	if ((tpl1 >> 6) & 1U) {
		diag_report(T_JRX, DIAG_WARN,
			    "BUF_PROTECT_EN still set (fsm.c write did not take)");
	} else if (pd_max != pd_min) {
		diag_report(T_JRX, DIAG_WARN, "protection off, PHASE_DIFF drifting %u-%u",
			    pd_min, pd_max);
	} else {
		diag_report(T_JRX, DIAG_PASS, "protection off, PHASE_DIFF stable at %u",
			    pd_min);
	}
}

/* ------------------------------------------------------------------------- */
/* [15] Output duty with the offload bypassed                                */
/* ------------------------------------------------------------------------- */

/*
 * Duty AND correctness, because duty alone is not evidence of health.
 *
 * This test used to measure only how often *something* was at the ADC, and it
 * reported PASS -- "continuous, 100% duty" -- on a run where Rung 5 failed and no
 * sample was arriving correctly. That is the probe's blind spot working as
 * designed: jesd_capture_probe() thresholds a mean-square, and the broadband
 * noise produced by a starved datapath clears the threshold as easily as the tone
 * does. Presence is not correctness, and a test that cannot tell them apart must
 * not be allowed to say PASS.
 *
 * So both are measured, and the verdict needs both:
 *
 *   duty                 -- DIAG_SWEEP_PROBES fast probes, as before. Answers
 *                           "is the output continuous?", which is what the TX
 *                           offload's mode controls.
 *   tone concentration   -- one full correlation. Answers "is it the signal we
 *                           sent?", which is what throughput controls.
 *
 * The two dissociate, and the combination is what identifies the state:
 *
 *   continuous + tone    -> the datapath is genuinely healthy.
 *   continuous + no tone -> fed too slowly to be coherent: gaps everywhere
 *                           instead of gathered into one window. The current
 *                           state, and the reason for the 403 MB/s in [3].
 *   gated + tone         -> the pre-bypass state: correct samples, 9% of the
 *                           time.
 *   gated + no tone      -> the TX bypass write did not take AND the capture
 *                           landed in an off-window.
 *
 * Note this is TX-only: the RX offload stays in store-and-replay because
 * bypassing it breaks Rung 2, so the transmit path is measured through an
 * unbypassed receive path.
 */
static void diag_offload_duty(void)
{
	struct jesd_loopback_meas m;
	size_t on = 0, taken = 0, pct;
	uint32_t best;
	bool tone;

	diag_begin(T_DUTY_BYPASS);

	for (size_t p = 0; p < DIAG_SWEEP_PROBES; p++) {
		int r = jesd_capture_probe();

		if (r < 0) {
			LOG_WRN("       probe failed (%d)", r);
			break;
		}
		on += (size_t)r;
		taken++;
	}

	if (taken == 0) {
		diag_report(T_DUTY_BYPASS, DIAG_SKIP, "no probe completed");
		return;
	}

	pct = on * 100U / taken;

	if (jesd_loopback_measure(&m) != 0) {
		LOG_INF("       duty %zu/%zu (%zu%%), correlation unavailable", on, taken,
			pct);
		diag_report(T_DUTY_BYPASS, DIAG_SKIP, "duty %zu%%, tone not measured", pct);
		return;
	}

	/* Best of both candidate I/Q pairings -- the beat layout after the FPGA
	 * transport core is an observation, not an assumption (see jesd_loopback.h). */
	best = MAX(m.concentration, m.concentration_split);
	tone = best >= DIAG_TONE_FOUND_MIN;
	LOG_INF("       duty %zu/%zu (%zu%%), RMS %llu, tone %u/1000 (need %u)", on, taken,
		pct, (unsigned long long)m.rms, best, DIAG_TONE_FOUND_MIN);

	/*
	 * Ordered so the strongest statement the data supports is the one reported.
	 * Anything without the tone is a FAIL regardless of duty -- a continuously
	 * present wrong signal is not a working transmitter.
	 */
	if (pct >= 90U && tone) {
		diag_report(T_DUTY_BYPASS, DIAG_PASS, "continuous %zu%% and tone %u/1000",
			    pct, best);
	} else if (pct >= 90U) {
		diag_report(T_DUTY_BYPASS, DIAG_FAIL,
			    "continuous %zu%% but no tone: starved, see [3]", pct);
	} else if (tone) {
		diag_report(T_DUTY_BYPASS, DIAG_FAIL,
			    "tone present but gated at %zu%%: bypass not in effect", pct);
	} else {
		diag_report(T_DUTY_BYPASS, DIAG_FAIL, "gated at %zu%% and no tone", pct);
	}
}

/* ------------------------------------------------------------------------- */
/* [16] Offload cyclic replay with the DMA stopped                           */
/* ------------------------------------------------------------------------- */

/*
 * The one configuration that makes the 403 MB/s in [3] irrelevant, and the
 * mode this IP was built to be used in.
 *
 * Everything in [3]-[5] says the transmit path cannot be fed continuously from
 * DDR: 403 MB/s against 4000 MB/s of demand, with the same ceiling on receive
 * and a CPU that reads the same memory far faster. Both remaining options --
 * rebuild the bitstream, or lower the link's demand -- are outside this sample.
 * This test checks a third that is not.
 *
 * The offload core can replay its own buffer. Its read FSM latches
 * rd_cyclic_en = ~rd_oneshot on entering PRE_RD, and with sync_config =
 * AUTOMATIC a completed pass loops RD_STATE_RD straight back to itself
 * (data_offload_fsm.v:213-217). One bufferful then streams out at full line
 * rate, forever, with no DDR traffic at all after the initial fill. The DMA's
 * rate stops mattering because the DMA stops running.
 *
 * What prevents it here is one line -- data_offload_fsm.v:243:
 *
 *     if (rd_init_req_s) rd_cyclic_en <= 1'b0;
 *
 * init_req is wired to the DMA's transfer-request line
 * (mxfe_tx_data_offload/init_req <- axi_mxfe_tx_dma/m_axis_xfer_req), so a DMA
 * that keeps asking for transfers keeps clearing the cyclic latch. Cyclic
 * replay and a running DMA are mutually exclusive by construction. This sample
 * has always left the TX DMA armed in cyclic mode, which is exactly the thing
 * that cancels the core's own cyclic mode -- two mechanisms fighting, with the
 * DMA's winning and contributing nothing but its rate limit.
 *
 * So: leave store-and-replay on, fill the buffer once, stop the DMA, and look.
 *
 *   continuous + tone -> cyclic replay works. This is the answer: full duty and
 *                        correct samples together, which no configuration has
 *                        produced yet, and the DMA bottleneck is bypassed
 *                        rather than fixed. Playback would be reconfigured
 *                        around this.
 *   gated or no tone  -> it does not, and the reason is worth knowing: the FSM
 *                        state is logged either way. Then the only remaining
 *                        fixes really are in the bitstream or the link geometry.
 *
 * The replayed segment is the offload's full buffer, and the tone table's
 * period divides it evenly, so the loop point carries no phase discontinuity
 * that would smear the correlation and read as starvation.
 *
 * Restores bypass and the continuous tone before returning, whatever happens,
 * so this test does not change the state the rest of the run sees.
 */
static void diag_offload_cyclic(void)
{
	struct jesd_loopback_meas m;
	uint64_t mem = 0;
	size_t on = 0, taken = 0, pct;
	uint32_t t_fill = 0, best;
	bool tone;
	int rc;

	diag_begin(T_CYCLIC);

	/* Store-and-replay: bypass replaces the buffer with a 16-entry FIFO, and
	 * there is nothing to replay out of a FIFO. */
	rc = axi_data_offload_bypass(false);
	if (rc) {
		diag_report(T_CYCLIC, DIAG_SKIP, "cannot leave bypass (%d)", rc);
		return;
	}

	rc = axi_data_offload_tx_size(&mem);
	if (rc || mem == 0) {
		diag_report(T_CYCLIC, DIAG_SKIP, "offload size unreadable (%d)", rc);
		goto restore;
	}

	/*
	 * Fill exactly one bufferful and stop. jesd_playback_timed() ends in
	 * dma_stop(), which deasserts xfer_req -- the whole point: with init_req
	 * released, rd_cyclic_en can stay latched and the core keeps replaying what
	 * it already holds.
	 */
	rc = jesd_playback_timed((size_t)mem, &t_fill);
	if (rc) {
		diag_report(T_CYCLIC, DIAG_SKIP, "fill transfer failed (%d)", rc);
		goto restore;
	}

	LOG_INF("       filled %llu B in %u us, DMA now stopped",
		(unsigned long long)mem, t_fill);

	/* Let the core get through at least one replay pass before looking. */
	k_msleep(5);
	axi_data_offload_status();

	for (size_t p = 0; p < DIAG_SWEEP_PROBES; p++) {
		int r = jesd_capture_probe();

		if (r < 0) {
			LOG_WRN("       probe failed (%d)", r);
			break;
		}
		on += (size_t)r;
		taken++;
	}

	if (taken == 0) {
		diag_report(T_CYCLIC, DIAG_SKIP, "no probe completed");
		goto restore;
	}
	pct = on * 100U / taken;

	if (jesd_loopback_measure(&m) != 0) {
		diag_report(T_CYCLIC, DIAG_SKIP, "duty %zu%%, tone not measured", pct);
		goto restore;
	}

	best = MAX(m.concentration, m.concentration_split);
	tone = best >= DIAG_TONE_FOUND_MIN;
	LOG_INF("       duty %zu/%zu (%zu%%), RMS %llu, tone %u/1000 (need %u)", on,
		taken, pct, (unsigned long long)m.rms, best, DIAG_TONE_FOUND_MIN);

	if (pct >= 90U && tone) {
		diag_report(T_CYCLIC, DIAG_PASS,
			    "replay works: %zu%% duty, tone %u/1000, no DMA", pct, best);
	} else if (tone) {
		diag_report(T_CYCLIC, DIAG_WARN,
			    "tone %u/1000 but only %zu%% duty: replay stops early", best,
			    pct);
	} else if (pct >= 90U) {
		diag_report(T_CYCLIC, DIAG_FAIL,
			    "continuous %zu%% but no tone: not replaying the fill", pct);
	} else {
		diag_report(T_CYCLIC, DIAG_FAIL,
			    "silent at %zu%% duty: replay did not start", pct);
	}

restore:
	/* Back to the state the rest of the run expects, regardless of outcome. */
	(void)axi_data_offload_bypass(true);
	(void)jesd_playback_rearm();
}

/* ------------------------------------------------------------------------- */
/* Runner                                                                   */
/* ------------------------------------------------------------------------- */

static const char *diag_verdict_str(enum diag_verdict v)
{
	switch (v) {
	case DIAG_PASS:
		return "PASS";
	case DIAG_WARN:
		return "WARN";
	case DIAG_FAIL:
		return "FAIL";
	default:
		return "SKIP";
	}
}

int jesd_diag_loopback(void)
{
	adi_ad9081_device_t *dev = ad9081_get_device();
	unsigned int pass = 0, warn = 0, fail = 0, skip = 0;

	if (dev == NULL) {
		LOG_ERR("AD9081 device not initialised");
		return -ENODEV;
	}

	memset(diag_res, 0, sizeof(diag_res));
	for (unsigned int i = 0; i < DIAG_NUM_TESTS; i++) {
		strncpy(diag_res[i].detail, "not run", sizeof(diag_res[i].detail) - 1);
	}

	LOG_INF("=== Rung 5 datapath tests (%u) ===", (unsigned int)DIAG_NUM_TESTS);

	/* Before anything here perturbs the chip: ask what it is already
	 * complaining about. */
	diag_irq_status(dev);
	diag_check_tx_gain(dev);

	/*
	 * Before any capture runs. Every capture and probe calls
	 * dma_get_status(), which on this IRQ-less core clears the EOT events
	 * test 3 counts -- so it has to measure while the playback transfer is
	 * the only thing touching the DMA.
	 */
	diag_tx_bandwidth();
	diag_rx_bandwidth();
	diag_cpu_ddr_rate();
	diag_offload_state();

	diag_sweep(dev);
	(void)diag_internal_tone(dev);
	diag_channel_tone(dev);
	diag_fpga_dds();

	/* Back to the configured plan, so the repeats measure the frequency the
	 * rest of the app runs at. */
	(void)diag_retune(dev, DIAG_NCO_HZ_DEFAULT);
	diag_repeatability();

	diag_duty_cycle();
	diag_gate_period();
	diag_jrx_buffer();
	diag_offload_duty();
	diag_offload_cyclic();

	/* Leave the board in the state the rest of the app documents. */
	if (diag_retune(dev, DIAG_NCO_HZ_DEFAULT) != 0) {
		LOG_WRN("could not restore the %lld MHz NCO pair",
			(long long)(DIAG_NCO_HZ_DEFAULT / 1000000LL));
	}

	LOG_INF("=== results ===");
	for (unsigned int i = 0; i < DIAG_NUM_TESTS; i++) {
		LOG_INF("[%2u] %-20s %s  %s", i + 1U, diag_names[i],
			diag_verdict_str(diag_res[i].verdict), diag_res[i].detail);

		switch (diag_res[i].verdict) {
		case DIAG_PASS:
			pass++;
			break;
		case DIAG_WARN:
			warn++;
			break;
		case DIAG_FAIL:
			fail++;
			break;
		default:
			skip++;
			break;
		}
	}
	LOG_INF("%u pass, %u warn, %u fail, %u skip", pass, warn, fail, skip);

	return 0;
}
