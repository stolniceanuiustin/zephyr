/*
 * Copyright (c) 2026 Analog Devices, Inc.
 *
 * AD9081/AD9082 + HMC7044 JESD204B bring-up.
 *
 * Board profile zcu102_ad9081_m8_l4: M8/L4/F4/K32/S1/NP16 geometry, TX link-mode
 * 9, RX link-mode 10, 8B10B subclass 1.
 *
 * The bring-up sequence:
 *
 *   1. the adi,hmc7044 clock_control driver's own init()
 *   2. axi_adxcvr_configure() + axi_jesd204_configure()
 *   3. ad9081_setup_datapath()
 *   4. jesd204_fsm_start()
 *   5. axi_jesd204_rx_watchdog(), then axi_jesd204_{tx,rx}_status_read()
 *   6. axi_tpl_configure() + axi_tpl_tx_dds()
 *
 * What this delivers at the DAC
 * -----------------------------
 * An FPGA-generated tone from the TX transport core's DDS: DATA_SELECT=0
 * (DATA_SEL_DDS) on every converter, 3 MHz at 0.05 full scale.
 *
 * The DMA engines and the axi_data_offload cores are deliberately absent: with
 * DDS selected the transport core's dac_enable is never asserted
 * (ad_ip_jesd204_tpl_dac_channel.v:144), so they are not in the datapath at all.
 *
 * Note on order: the transport cores are configured *after* the link reaches
 * DATA, because the DAC core's SYNC pulse and its CLK_FREQ/CLK_RATIO readback are
 * only meaningful against a running sample clock. axi_tpl_configure() writes the
 * datapath registers, and axi_tpl_enable(), driven from inside the bring-up
 * sequence, re-latches SYNC once the clocks are live.
 *
 * IMPORTANT: JESD204 is a negotiated multi-device link -- the transceiver, link
 * cores, SYSREF and the AD9082 cannot be brought up or status-checked in
 * isolation. Each block only *configures* here; activation and the single
 * meaningful status check happen together in the bring-up FSM.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/cache.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/dma.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

#include <zephyr/drivers/misc/ad9081/ad9081.h>
#include <zephyr/drivers/clock_control/hmc7044.h>
#include <zephyr/drivers/misc/jesd204/axi_adxcvr.h>
#include <zephyr/drivers/misc/jesd204/axi_jesd204.h>
#include <zephyr/drivers/misc/jesd204/axi_tpl.h>
#include "ad9081_bringup.h"

/*
 * Every raw sys_read32/sys_write32 of a PL core in this sample and in the
 * jesd204 drivers assumes device_map() hands back virt == phys. On arm64 that is
 * guaranteed rather than configured -- arch/arm64/core/Kconfig does
 * `select KERNEL_DIRECT_MAP if MMU` -- but the dependency is invisible at every
 * use site, so it is asserted once here.
 */
BUILD_ASSERT(IS_ENABLED(CONFIG_KERNEL_DIRECT_MAP),
	     "This sample requires CONFIG_KERNEL_DIRECT_MAP: the PL core drivers "
	     "rely on device_map() returning virt == phys.");

/*
 * One-shot RX capture: enough samples to dump a few full periods of a
 * fed-in signal per channel, not a real acquisition. 8 converters
 * interleaved (M8), 16-bit signed each -- rx_tpl arms every channel with
 * ADC_CHAN_FORMAT_SIGNEXT (axi_tpl.c).
 *
 * A power of two so the capture window holds a whole number of cycles of the
 * DAC tone below, which is what the loopback check depends on.
 */
#define RX_CAPTURE_SAMPLES_PER_CHAN 64
#define RX_CAPTURE_NUM_CHAN         8

/*
 * Which DFT bin the DAC tone is expected in. 8 of 64 is a quarter of the way up
 * the band: clear of DC and any ADC offset at the bottom, and clear of the
 * decimation filter roll-off at the top.
 */
#define RX_CAPTURE_TONE_BIN 8

/*
 * Loopback pass threshold, as a percentage of captured energy that has to land
 * in RX_CAPTURE_TONE_BIN. A clean loopback puts nearly all of it there; this is
 * set low enough that cable loss, the attenuator and ADC noise cannot fail a
 * working path, and high enough that noise alone cannot pass it -- 64 bins of
 * pure noise would put about 1.5% in any one bin.
 */
#define RX_CAPTURE_TONE_MIN_PCT 25

/*
 * DAC output tone. The rate the DDS phase accumulator runs at is the transport
 * core's sample rate, 250 MSPS for this link (4 GHz ADC / 4x main / 4x channel
 * decimation on the receive side, and the matching interpolation on transmit).
 *
 * The tone is placed on an exact FFT bin of the RX capture below:
 *
 *     tone = sample_rate * RX_CAPTURE_TONE_BIN / RX_CAPTURE_SAMPLES_PER_CHAN
 *          = 250e6 * 8 / 64 = 31.25 MHz
 *
 * which is what makes a DAC-to-ADC loopback readable from the raw sample dump:
 * both NCOs shift by the same +2 GHz, so the tone returns to baseband and
 * completes exactly RX_CAPTURE_TONE_BIN cycles in the captured window. An
 * arbitrary frequency would straddle bins and smear across the dump instead.
 *
 * 3 MHz -- the frequency ADI's reference application uses -- is not usable for
 * that: one cycle is 333 ns against a 256 ns capture window, so less than a
 * full period is captured.
 */
#define DAC_DDS_SAMPLE_RATE  (250 * 1000 * 1000)
#define DAC_DDS_TONE_HZ (DAC_DDS_SAMPLE_RATE / RX_CAPTURE_SAMPLES_PER_CHAN * RX_CAPTURE_TONE_BIN)
#define DAC_DDS_SCALE_MICRO (50 * 1000) /* 0.05 full scale */

/*
 * Which HMC7044 output drives the GT reference clock. Taken from the transceiver
 * node's own `clocks` phandle rather than stated again here, so this report and
 * the rate the adxcvr driver actually solves its dividers against cannot drift
 * apart.
 */
#define GT_REFCLK_OUT DT_CLOCKS_CELL(DT_NODELABEL(tx_adxcvr), output)

/* Generous for 512 bytes; only has to bound a stall, not pace a real transfer. */
#define RX_CAPTURE_TIMEOUT_MS 100

static int16_t rx_capture_buf[RX_CAPTURE_NUM_CHAN * RX_CAPTURE_SAMPLES_PER_CHAN] __aligned(64);

/*
 * Host-generated playback tone, for the one part of the TX chain the DDS above
 * cannot reach: DDR -> tx_dmac (MEM_TO_DEV) -> the transport core's upack FIFO.
 * With DATA_SEL_DDS the core's dac_enable is never asserted, so no beat is ever
 * pulled from that FIFO and the DMA engine is out of the datapath entirely.
 *
 * Same frequency as the DDS tone -- 31.25 MHz, which is exactly
 * DAC_DDS_SAMPLE_RATE / 8 -- so this reuses the whole existing frequency plan
 * (RF = 1 GHz main NCO + 31.25 MHz) and the RX capture's bin-8 loopback check
 * without changing either. fs/8 also means the phase advances exactly 45 degrees
 * per sample, so the sample values come from an 8-entry table of exact
 * quadrant/half-quadrant values -- no libm, no soft-float, and no rounding drift
 * that would smear the tone across bins.
 *
 * A whole number of periods per buffer (TX_DMA_PERIODS of them) is what makes
 * cyclic replay seamless: the engine wraps to sample 0 with the phase continuing
 * where it left off, so there is no discontinuity to widen the spectrum.
 *
 * RF LEVEL: the amplitude below is 5% of int16 full scale, deliberately the same
 * as DAC_DDS_SCALE_MICRO, so switching to this source does not change the power
 * at the SMA. A Pluto RX is safe at that level and not much above it -- do not
 * raise TX_DMA_AMPLITUDE with a receiver cabled to the DAC.
 */
#define TX_DMA_NUM_CONV       8 /* M8: the DMA stream interleaves all converters */
#define TX_DMA_PERIOD_SAMPLES 8 /* fs/8 -> 45 degrees per sample */
#define TX_DMA_PERIODS        128
#define TX_DMA_SAMPLES_PER_CONV (TX_DMA_PERIODS * TX_DMA_PERIOD_SAMPLES)

/*
 * 0.10 * 32767 rounded, and that amplitude times cos(45 deg): the only two
 * magnitudes an fs/8 tone takes, since the other two phases are 0 and +-full.
 *
 * 0.10 rather than the DDS's DAC_DDS_SCALE_MICRO of 0.05 because the transport
 * core sums the *two* DDSs of a converter, so 0.05 per DDS puts 0.10 of full
 * scale at the converter. Matching that here keeps the power at the SMA the same
 * across a source switch, which is what makes the two measurements comparable --
 * a 6 dB step at the receiver would otherwise look like a datapath fault.
 */
#define TX_DMA_AMPLITUDE      3277
#define TX_DMA_AMPLITUDE_HALF 2317

static int16_t tx_dma_buf[TX_DMA_NUM_CONV * TX_DMA_SAMPLES_PER_CONV] __aligned(64);

/*
 * How much of channel 0's captured energy sits in RX_CAPTURE_TONE_BIN.
 *
 * A single-bin DFT (Goertzel would do the same with less arithmetic; at 64
 * points the direct form is not worth optimising and is easier to check by
 * eye). Integer throughout: the tone sits on an exact bin, so the basis
 * function only ever needs the RX_CAPTURE_SAMPLES_PER_CHAN-point cosine table
 * below, and a fixed-point table keeps this out of soft-float on a build that
 * has no FPU enabled.
 *
 * This is what turns a DAC-to-ADC loopback cable into a pass/fail: it proves
 * samples arrived, at the right frequency, with the right periodicity -- which
 * a printed sample dump can only suggest.
 *
 * Reports rather than returns: the link is already up by the time this runs, and
 * with no loopback cable fitted a failure here is the expected result, not an
 * error.
 */
/*
 * cos(2*pi*n/RX_CAPTURE_SAMPLES_PER_CHAN) * 4096, one full turn. Stated in full
 * rather than folded from a quarter table: the folding is easy to get wrong by
 * half a sample and the saving is 192 bytes of rodata.
 */
static const int32_t cos_q12[RX_CAPTURE_SAMPLES_PER_CHAN] = {
	4096,  4076,  4017,  3920,  3784,  3612,  3406,  3166,  2896,  2598,  2276,  1931,  1567,
	1189,  799,   401,   0,     -401,  -799,  -1189, -1567, -1931, -2276, -2598, -2896, -3166,
	-3406, -3612, -3784, -3920, -4017, -4076, -4096, -4076, -4017, -3920, -3784, -3612, -3406,
	-3166, -2896, -2598, -2276, -1931, -1567, -1189, -799,  -401,  0,     401,   799,   1189,
	1567,  1931,  2276,  2598,  2896,  3166,  3406,  3612,  3784,  3920,  4017,  4076,
};

static unsigned int rx_capture_bin_pct(unsigned int chan, unsigned int bin)
{
	int64_t re = 0, im = 0, energy = 0;
	int32_t mean = 0;

	/*
	 * Remove DC first. An ADC offset is a bin-0 term and does not leak into
	 * bin 8 of an exact-bin DFT, but it does inflate the total energy the
	 * fraction is taken against, which would understate a good tone.
	 */
	for (int n = 0; n < RX_CAPTURE_SAMPLES_PER_CHAN; n++) {
		mean += rx_capture_buf[n * RX_CAPTURE_NUM_CHAN + chan];
	}
	mean /= RX_CAPTURE_SAMPLES_PER_CHAN;

	for (int n = 0; n < RX_CAPTURE_SAMPLES_PER_CHAN; n++) {
		int32_t x = rx_capture_buf[n * RX_CAPTURE_NUM_CHAN + chan] - mean;
		/* sin(t) = cos(t - 90 degrees), i.e. a quarter turn back. */
		int32_t phase = (bin * n) % RX_CAPTURE_SAMPLES_PER_CHAN;
		int32_t quarter = RX_CAPTURE_SAMPLES_PER_CHAN / 4;

		re += (int64_t)x * cos_q12[phase];
		im -= (int64_t)x * cos_q12[(phase + 3 * quarter) % RX_CAPTURE_SAMPLES_PER_CHAN];
		energy += (int64_t)x * x;
	}

	if (energy == 0) {
		return 0;
	}

	/*
	 * Parseval for a real signal: the bin and its negative-frequency mirror
	 * hold 2*|X_k|^2/N of the total. Descale the two q12 factors in |X_k|^2
	 * before dividing, so the ratio is taken between like units.
	 */
	re >>= 12;
	im >>= 12;
	return (unsigned int)((200ULL * (uint64_t)(re * re + im * im)) /
			      ((uint64_t)energy * RX_CAPTURE_SAMPLES_PER_CHAN));
}

/*
 * Report the tone fraction for every captured channel.
 *
 * All of them, not just channel 0: which converter a cabled ADC input arrives on
 * depends on the coarse/fine DDC selects and the crossbar, so a single-channel
 * check reports "no tone" for a working loopback on any other channel. Scanning
 * all eight makes the log say which one it landed on.
 */
static void rx_capture_check_tone(void)
{
	unsigned int best_pct = 0, best_chan = 0;

	for (unsigned int c = 0; c < RX_CAPTURE_NUM_CHAN; c++) {
		unsigned int pct = rx_capture_bin_pct(c, RX_CAPTURE_TONE_BIN);
		unsigned int top_pct = 0, top_bin = 0;
		int16_t peak = 0;

		/*
		 * Which bin actually holds the most, and how large the samples are.
		 * Without these, a tone at the wrong frequency and no tone at all look
		 * identical -- both just report a low fraction in the expected bin.
		 */
		for (unsigned int b = 1; b < RX_CAPTURE_SAMPLES_PER_CHAN / 2; b++) {
			unsigned int p = rx_capture_bin_pct(c, b);

			if (p > top_pct) {
				top_pct = p;
				top_bin = b;
			}
		}

		for (int n = 0; n < RX_CAPTURE_SAMPLES_PER_CHAN; n++) {
			int16_t v = rx_capture_buf[n * RX_CAPTURE_NUM_CHAN + c];

			if (v > peak) {
				peak = v;
			}
		}

		/*
		 * One line per channel only when a channel is unexpected: a
		 * channel carrying the tone, or one whose energy piles into some
		 * other bin. Channels at the noise floor say nothing -- four of
		 * the eight are the unconnected ADC's, so they are the normal
		 * case, not a finding.
		 */
		if (pct >= RX_CAPTURE_TONE_MIN_PCT || top_pct >= RX_CAPTURE_TONE_MIN_PCT) {
			LOG_INF("RX capture: ch%u bin%u=%u%%, strongest bin%u=%u%% (%u MHz), peak %d",
				c, RX_CAPTURE_TONE_BIN, pct, top_bin, top_pct,
				top_bin * (DAC_DDS_SAMPLE_RATE / 1000000U) /
					RX_CAPTURE_SAMPLES_PER_CHAN,
				peak);
		}

		if (pct > best_pct) {
			best_pct = pct;
			best_chan = c;
		}
	}

	if (best_pct >= RX_CAPTURE_TONE_MIN_PCT) {
		LOG_INF("SUCCESS: loopback tone present at %u MHz on ch%u (%u%%)",
			DAC_DDS_TONE_HZ / 1000000U, best_chan, best_pct);
	} else {
		LOG_WRN("no loopback tone on any channel (best %u%% on ch%u, want >=%u%%) -- "
			"expected unless a DAC output is cabled to an ADC input",
			best_pct, best_chan, RX_CAPTURE_TONE_MIN_PCT);
	}
}

/*
 * Fill rx_capture_buf with one DMA transfer's worth of samples. Split out of
 * rx_capture_dump() so a caller can re-capture without re-logging the
 * per-channel report each time.
 *
 * Returns 0 once rx_capture_buf holds fresh, cache-invalidated samples.
 */
static int rx_capture_fetch(void)
{
	const struct device *dmac = DEVICE_DT_GET(DT_NODELABEL(rx_dmac));
	struct dma_block_config block = {
		.dest_address = (uintptr_t)rx_capture_buf,
		.block_size = sizeof(rx_capture_buf),
	};
	struct dma_config cfg = {
		.channel_direction = PERIPHERAL_TO_MEMORY,
		.block_count = 1,
		.head_block = &block,
		.dest_data_size = sizeof(int16_t),
		.dest_burst_length = sizeof(int16_t),
	};
	struct dma_status status;
	int64_t deadline;
	int ret;

	if (!device_is_ready(dmac)) {
		LOG_WRN("rx_dmac not ready, skipping RX capture dump");
		return -ENODEV;
	}

	ret = dma_config(dmac, 0, &cfg);
	if (ret) {
		LOG_WRN("rx_dmac config failed (%d), skipping RX capture dump", ret);
		return ret;
	}

	ret = dma_start(dmac, 0);
	if (ret) {
		LOG_WRN("rx_dmac start failed (%d), skipping RX capture dump", ret);
		return ret;
	}

	/*
	 * No interrupt wired to this core -- dma_get_status() self-pumps the
	 * transfer on each poll, so it must be polled to completion. Bounded:
	 * a stalled transfer must not hang a link that is already up.
	 */
	deadline = k_uptime_get() + RX_CAPTURE_TIMEOUT_MS;
	do {
		ret = dma_get_status(dmac, 0, &status);
		if (ret) {
			LOG_WRN("rx_dmac status read failed (%d)", ret);
			return ret;
		}

		if (k_uptime_get() > deadline) {
			LOG_WRN("rx_dmac transfer did not complete in %d ms",
				RX_CAPTURE_TIMEOUT_MS);
			return -ETIMEDOUT;
		}
	} while (status.busy);

	sys_cache_data_invd_range(rx_capture_buf, sizeof(rx_capture_buf));

	return 0;
}

/*
 * Kick a single DEV_TO_MEM transfer on rx_dmac and dump the captured samples, so
 * a signal fed into the ADC input can be confirmed present in the digital
 * samples without any host-side IIO tooling -- counterpart to the DAC's DDS tone
 * being visible on a scope.
 *
 * Best-effort: a failure here does not affect the link, which is already up by
 * the time this runs, so it warns rather than returning an error.
 */
static void rx_capture_dump(void)
{
	if (rx_capture_fetch()) {
		return;
	}

	rx_capture_check_tone();

#if defined(RX_SAMPLE_DUMP)
	/*
	 * The raw samples, one line each. Only useful when the per-bin figures
	 * above are themselves in doubt -- the DFT summarises the same 64 points
	 * in one line, so this is off by default rather than 64 lines per
	 * capture (128 per boot, since this runs before and after the DAC tone).
	 */
	LOG_INF("RX capture: ch0, all %u samples:", RX_CAPTURE_SAMPLES_PER_CHAN);
	for (int i = 0; i < RX_CAPTURE_SAMPLES_PER_CHAN; i++) {
		LOG_INF("  [%2d] ch0 = %6d", i,
			rx_capture_buf[i * RX_CAPTURE_NUM_CHAN]);
	}
#endif
}

/*
 * Fill tx_dma_buf with an fs/8 complex tone, interleaved across all M8
 * converters the way the transport core consumes them: sample-major, one 16-bit
 * word per converter, which is the transmit mirror of rx_capture_buf's layout.
 *
 * The converter pairs are (0,1), (2,3), (4,5), (6,7) -- even is I, odd is Q --
 * and every pair gets the same tone. Only pair 0 reaches DAC0 on this profile
 * (adi,tx-dac-channel-crossbar routes DUC n to DAC n), but filling all four costs
 * nothing and keeps the buffer independent of which SMA is cabled.
 */
static void tx_dma_fill_tone(void)
{
	/*
	 * cos and sin at 0, 45, 90 ... 315 degrees, scaled to TX_DMA_AMPLITUDE.
	 * Written out rather than computed so no float or rounding is involved:
	 * at fs/8 these eight phases are the only ones the tone ever takes.
	 */
	static const int16_t cos45[TX_DMA_PERIOD_SAMPLES] = {
		TX_DMA_AMPLITUDE,       TX_DMA_AMPLITUDE_HALF,  0,
		-TX_DMA_AMPLITUDE_HALF, -TX_DMA_AMPLITUDE,      -TX_DMA_AMPLITUDE_HALF,
		0,                      TX_DMA_AMPLITUDE_HALF,
	};
	static const int16_t sin45[TX_DMA_PERIOD_SAMPLES] = {
		0,                     TX_DMA_AMPLITUDE_HALF,  TX_DMA_AMPLITUDE,
		TX_DMA_AMPLITUDE_HALF, 0,                      -TX_DMA_AMPLITUDE_HALF,
		-TX_DMA_AMPLITUDE,     -TX_DMA_AMPLITUDE_HALF,
	};

	for (uint32_t n = 0; n < TX_DMA_SAMPLES_PER_CONV; n++) {
		uint32_t phase = n % TX_DMA_PERIOD_SAMPLES;
		int16_t *frame = &tx_dma_buf[n * TX_DMA_NUM_CONV];

		for (uint32_t c = 0; c < TX_DMA_NUM_CONV; c += 2) {
			frame[c] = cos45[phase];
			frame[c + 1] = sin45[phase];
		}
	}
}

/*
 * Replace the FPGA DDS with a tone streamed from DDR, which is the only way to
 * put tx_dmac and the transport core's upack FIFO into the datapath.
 *
 * Order matters: the DMA transfer is started while the converters are still on
 * the DDS source, so the engine primes the FIFO before anything consumes from it.
 * Switching DATA_SELECT afterwards asserts dac_enable against a FIFO that already
 * has data, rather than against an empty one.
 *
 * The transfer is cyclic, so it never completes and there is nothing to poll:
 * the engine wraps to the start of the buffer indefinitely and the DAC keeps
 * emitting. tx_dmac reports hardware cyclic support, so the wrap is done in the
 * core rather than by the driver's software resubmission.
 *
 * Best-effort like rx_capture_dump(): the link is already up, so a failure here
 * warns and leaves the DDS tone in place rather than failing the bring-up.
 */
static int tx_dma_tone_start(void)
{
	const struct device *dmac = DEVICE_DT_GET(DT_NODELABEL(tx_dmac));
	const struct device *tpl = DEVICE_DT_GET(DT_NODELABEL(tx_tpl));
	struct dma_block_config block = {
		.source_address = (uintptr_t)tx_dma_buf,
		.block_size = sizeof(tx_dma_buf),
	};
	struct dma_config cfg = {
		.channel_direction = MEMORY_TO_PERIPHERAL,
		.block_count = 1,
		.head_block = &block,
		.source_data_size = sizeof(int16_t),
		.source_burst_length = sizeof(int16_t),
		.cyclic = 1,
	};
	int ret;

	if (!device_is_ready(dmac)) {
		LOG_WRN("tx_dmac not ready, staying on the DDS tone");
		return -ENODEV;
	}

	tx_dma_fill_tone();
	/* The engine reads DDR directly; the CPU's writes are still in cache. */
	sys_cache_data_flush_range(tx_dma_buf, sizeof(tx_dma_buf));

	ret = dma_config(dmac, 0, &cfg);
	if (ret) {
		LOG_WRN("tx_dmac config failed (%d), staying on the DDS tone", ret);
		return ret;
	}

	ret = dma_start(dmac, 0);
	if (ret) {
		LOG_WRN("tx_dmac start failed (%d), staying on the DDS tone", ret);
		return ret;
	}

	/* enable=false puts every converter back on DAC_DATA_SEL_DMA and syncs. */
	ret = axi_tpl_tx_dds(tpl, 0, 0, 0, false);
	if (ret) {
		LOG_WRN("could not switch the converters to the DMA source (%d)", ret);
		return ret;
	}

	LOG_INF("SUCCESS: DAC playing a %u MHz tone from memory over tx_dmac "
		"(%u samples/converter, cyclic)",
		DAC_DDS_TONE_HZ / 1000000U, TX_DMA_SAMPLES_PER_CONV);
	return 0;
}

/*
 * Sweep the receive coarse NCO and report where the tone lands.
 *
 * Every fixed-frequency attempt so far has produced noise at the expected bin
 * while the DDC registers read back exactly as configured, which a fixed mix
 * frequency cannot tell apart from a datapath that carries nothing. Sweeping
 * separates them in one boot: a peak at some offset means the tone is real and
 * the downmix frequency is wrong by that offset, and a flat result across the
 * whole range means the fault is not frequency-related at all -- which rules out
 * the NCO, the sample rate and the Nyquist zone together.
 *
 * Reported per step is the best bin over all channels rather than bin 8 alone:
 * if the mix frequency is off, the tone is by definition not in bin 8, so
 * looking only there would report nothing at every step including the right one.
 */
__maybe_unused static void rx_nco_sweep(const struct device *mxfe)
{
	const int64_t centre_hz = 1000000000;
	const int64_t span_hz = 200000000;
	const int64_t step_hz = 25000000;

	if (!device_is_ready(mxfe)) {
		LOG_WRN("mxfe not ready, skipping RX NCO sweep");
		return;
	}

	LOG_INF("RX NCO sweep: %lld..%lld MHz in %lld MHz steps",
		(long long)((centre_hz - span_hz) / 1000000),
		(long long)((centre_hz + span_hz) / 1000000),
		(long long)(step_hz / 1000000));

	for (int64_t f = centre_hz - span_hz; f <= centre_hz + span_hz; f += step_hz) {
		unsigned int best_pct = 0, best_bin = 0, best_chan = 0;
		int ret;

		ret = ad9081_rx_coarse_nco_set(mxfe, 0xf, f);
		if (ret) {
			LOG_WRN("rx_coarse_nco_set(%lld Hz) failed (%d)",
				(long long)f, ret);
			continue;
		}

		k_msleep(2);

		if (rx_capture_fetch()) {
			return;
		}

		for (unsigned int c = 0; c < RX_CAPTURE_NUM_CHAN; c++) {
			for (unsigned int b = 1; b < RX_CAPTURE_SAMPLES_PER_CHAN / 2; b++) {
				unsigned int p = rx_capture_bin_pct(c, b);

				if (p > best_pct) {
					best_pct = p;
					best_bin = b;
					best_chan = c;
				}
			}
		}

		LOG_INF("RX NCO sweep: %4lld MHz -> best ch%u bin%u = %u%%",
			(long long)(f / 1000000), best_chan, best_bin, best_pct);
	}

	(void)ad9081_rx_coarse_nco_set(mxfe, 0xf, centre_hz);
	LOG_INF("RX NCO sweep: restored %lld MHz", (long long)(centre_hz / 1000000));
}

/*
 * The HMC7044 is a clock_control driver, so it programmes itself at POST_KERNEL
 * before main() runs -- there is no hmc7044_probe()/setup_clocks() call to make.
 * Report what it achieved instead, so the boot log keeps the two lines the
 * explicit calls used to produce.
 */
static int report_clock_tree(const struct device *clk)
{
	struct hmc7044_status status;
	uint32_t refclk_hz;
	int ret;

	if (!device_is_ready(clk)) {
		/*
		 * The driver logged the specific failure during its own init;
		 * everything downstream needs its clocks, so stop here.
		 */
		LOG_ERR("HMC7044 did not initialise -- no clocks, cannot continue");
		return -ENODEV;
	}

	ret = hmc7044_get_status(clk, &status);
	if (ret) {
		LOG_ERR("could not read HMC7044 status (%d)", ret);
		return ret;
	}

	if (!status.pll1_locked || !status.pll2_locked) {
		/*
		 * Not fatal here, deliberately: the link cannot come up without
		 * locked PLLs, but letting the bring-up proceed and fail at the
		 * phase that actually depends on the clock is more diagnosable
		 * than bailing with only a lock bit to show for it.
		 */
		LOG_WRN("HMC7044 PLLs not locked (PLL1 %s, PLL2 %s) -- "
			"the JESD204 link will not come up",
			status.pll1_fsm_state_str,
			status.pll2_locked ? "locked" : "unlocked");
	}

	/* The rate axi_adxcvr solves its GT dividers against. */
	ret = clock_control_get_rate(clk, HMC7044_CLK_OUT(GT_REFCLK_OUT),
				     &refclk_hz);
	if (ret) {
		LOG_ERR("could not read the GT refclk rate (%d)", ret);
		return ret;
	}

	/*
	 * Deliberately not another "SUCCESS: HMC7044 clock tree configured" --
	 * the driver's init() already emitted that line, byte for byte as the
	 * pre-driver-model code did, so the boot log stays diffable. This is the
	 * one genuinely new line: what the clock tree is actually running at.
	 */
	LOG_INF("HMC7044 clocks live: PLL1 %s on CLKIN%u, PLL2 %u.%03u GHz, "
		"GT refclk out%u %u.%03u MHz",
		status.pll1_fsm_state_str, status.pll1_active_clkin,
		status.pll2_freq / 1000000000U,
		(status.pll2_freq / 1000000U) % 1000U,
		GT_REFCLK_OUT, refclk_hz / 1000000U,
		(refclk_hz / 1000U) % 1000U);

	return 0;
}

int main(void)
{
	const struct device *clk = DEVICE_DT_GET(DT_NODELABEL(hmc7044));
	const struct device *mxfe = DEVICE_DT_GET(DT_NODELABEL(ad9081));
	uint16_t prod_id;
	int ret;

	LOG_INF("=== AD9081/HMC7044 bring-up ===");

	/* Clock chip first (topology order). */
	ret = report_clock_tree(clk);
	if (ret) {
		return ret;
	}

	/* MxFE. */
	ret = ad9081_probe(mxfe, &prod_id);
	if (ret) {
		LOG_ERR("AD9081 probe failed (%d)", ret);
		return ret;
	}
	LOG_INF("SUCCESS: AD90%02x detected over SPI", prod_id & 0xFF);

	/*
	 * Configure the MxFE datapath through the ADI API lib (CLK PLL, TX interp
	 * + DAC NCOs + JRX deframer, RX decim + ADC NCOs + JTX framer). The chip
	 * JESD links are configured but not enabled -- the FSM enables them with
	 * the FPGA cores.
	 */
	ret = ad9081_setup_datapath(mxfe);
	if (ret) {
		LOG_ERR("AD9081 datapath setup failed (%d)", ret);
		return ret;
	}
	LOG_INF("SUCCESS: AD9082 datapath configured (chip JESD links ready)");

	/*
	 * GT transceiver config only (clock-mux select, held in reset). The
	 * reset-release + status poll is deferred to the JESD204 bring-up FSM,
	 * because GT-ready is only meaningful once the link layer and SYSREF are
	 * up around it -- gating on it standalone times out.
	 */
	ret = axi_adxcvr_configure(DEVICE_DT_GET(DT_NODELABEL(tx_adxcvr)));
	if (ret) {
		LOG_ERR("AXI adxcvr (GT) config failed (%d)", ret);
		return ret;
	}
	ret = axi_adxcvr_configure(DEVICE_DT_GET(DT_NODELABEL(rx_adxcvr)));
	if (ret) {
		LOG_ERR("AXI adxcvr (GT) config failed (%d)", ret);
		return ret;
	}
	LOG_INF("SUCCESS: GT transceivers configured (TX QPLL0 / RX CPLL)");

	/*
	 * JESD204 link cores: program link geometry + ILAS, held disabled. TX
	 * before RX, which is the order the single-call version used.
	 */
	ret = axi_jesd204_configure(DEVICE_DT_GET(DT_NODELABEL(tx_jesd)));
	if (ret == 0) {
		ret = axi_jesd204_configure(DEVICE_DT_GET(DT_NODELABEL(rx_jesd)));
	}
	if (ret) {
		LOG_ERR("AXI jesd204 link config failed (%d)", ret);
		return ret;
	}
	LOG_INF("SUCCESS: JESD204 link cores configured (M%d/L%d/F%d/K%d)",
		DT_PROP(DT_NODELABEL(tx_jesd), adi_converters_per_device),
		DT_PROP(DT_NODELABEL(tx_jesd), adi_lanes_per_device),
		DT_PROP(DT_NODELABEL(tx_jesd), adi_octets_per_frame),
		DT_PROP(DT_NODELABEL(tx_jesd), adi_frames_per_multiframe));

	/* TPL transport cores: datapath sample-format (RX) + data-source (TX). */
	ret = axi_tpl_configure(DEVICE_DT_GET(DT_NODELABEL(rx_tpl)));
	if (ret == 0) {
		ret = axi_tpl_configure(DEVICE_DT_GET(DT_NODELABEL(tx_tpl)));
	}
	if (ret) {
		LOG_ERR("AXI TPL transport config failed (%d)", ret);
		return ret;
	}
	LOG_INF("SUCCESS: TPL transport cores configured (%d converters)",
		DT_PROP(DT_NODELABEL(rx_tpl), adi_num_channels));

	LOG_INF("=== all blocks configured, running JESD204 bring-up ===");

	/*
	 * Bring the link up: activate the transceiver, link cores and the chip's
	 * framer/deframer together, then read link status.
	 */
	ret = jesd204_bringup();
	if (ret) {
		LOG_WRN("JESD204 bring-up did not reach DATA (%d)", ret);
		return ret;
	}
	LOG_INF("SUCCESS: JESD204B link up");

	/*
	 * Per-lane check, immediately after the FSM. Reaching DATA does not
	 * guarantee every lane is aligned; if one is not, the watchdog bounces the
	 * link and returns -EAGAIN. Re-read the status afterwards rather than
	 * treating that as fatal.
	 */
	ret = axi_jesd204_rx_watchdog(DEVICE_DT_GET(DT_NODELABEL(rx_jesd)));
	if (ret == -EAGAIN) {
		/*
		 * Only a restart makes a second status read worth its 22 lines. The
		 * FSM's link_running phase already read and logged both cores and
		 * returned -EIO on anything but DATA, which jesd204_bringup() turns
		 * into the early return above -- so in the normal case reading again
		 * here printed the same two blocks a second time, 3 ms later.
		 *
		 * A watchdog restart invalidates that: the state the FSM logged is
		 * from before the bounce. Both ends are read and logged before either
		 * verdict is taken, so a failure on TX does not hide RX's state.
		 */
		LOG_WRN("link was restarted after a lane desync, re-reading status");

		ret = axi_jesd204_status_read(DEVICE_DT_GET(DT_NODELABEL(tx_jesd)));
		if (axi_jesd204_status_read(DEVICE_DT_GET(DT_NODELABEL(rx_jesd)))) {
			ret = -EIO;
		}
		if (ret) {
			LOG_ERR("=== link is not carrying DATA after the restart ===");
			return ret;
		}
	} else if (ret) {
		/* Not fatal, and not a restart: nothing to re-read, so just say so. */
		LOG_WRN("per-lane check failed (%d), link status is as logged above",
			ret);
	}

	/*
	 * Verify the transport layer, now that the link underneath it is carrying
	 * DATA -- after jesd204_bringup() returns, not inside a phase. The TPL
	 * cores are downstream of the link -- they only map JESD frames onto
	 * converters -- so they are not JESD204 topology devices.
	 *
	 * Warn but do not fail: the link is up, which is what was being brought up
	 * here. A TPL complaint is a datapath problem below the link, and returning
	 * an error would report the link as broken when it is not.
	 */
	if (axi_tpl_enable(DEVICE_DT_GET(DT_NODELABEL(rx_tpl)),
			   DEVICE_DT_GET(DT_NODELABEL(tx_tpl)))) {
		LOG_WRN("TPL post-link verify failed (link is up regardless)");
	}

	/*
	 * Point the DAC converters at the transport core's DDS, upconverted by the
	 * chip's +1 GHz main NCO. Scope
	 * the DAC output to see it.
	 *
	 * This has to happen BEFORE the RX capture below. It used to run after,
	 * which meant every capture measured a silent DAC and reported the
	 * loopback dead while the datapath was in fact working.
	 */
	ret = axi_tpl_tx_dds(DEVICE_DT_GET(DT_NODELABEL(tx_tpl)),
			     DAC_DDS_TONE_HZ, DAC_DDS_SAMPLE_RATE,
			     DAC_DDS_SCALE_MICRO, true);
	if (ret) {
		LOG_ERR("could not arm the DAC DDS tone (%d)", ret);
		return ret;
	}
	LOG_INF("SUCCESS: DAC emitting a %u MHz DDS tone at %u%% full scale",
		DAC_DDS_TONE_HZ / 1000000U, DAC_DDS_SCALE_MICRO / 10000U);

	/*
	 * RX capture: confirms captured data is reaching the CPU over rx_dmac and
	 * that the DAC tone above completes the loopback -- the receive
	 * counterpart of scoping the DAC output.
	 */
	rx_capture_dump();

	/*
	 * Hand the DAC over to a tone streamed from DDR. This runs after the RX
	 * capture above so that capture still measures the DDS -- the two tones are
	 * the same frequency, so the loopback check reads the same either way, but
	 * keeping the DDS as the thing under test there leaves the DMA path as the
	 * only variable in whatever measures the DAC afterwards.
	 */
	if (tx_dma_tone_start()) {
		LOG_WRN("TX DMA playback did not start (the DDS tone is still up)");
	}

#if defined(RX_NCO_SWEEP)
	rx_nco_sweep(DEVICE_DT_GET(DT_NODELABEL(ad9081)));
#endif

	LOG_INF("=== bring-up complete ===");

	return 0;
}
