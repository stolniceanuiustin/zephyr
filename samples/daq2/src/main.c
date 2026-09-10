/*
 * Copyright (c) 2026 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * DAQ2 JESD204 bring-up -- Phase 1b: enable the link.
 *
 * The AD9523 clock generator and the AD9144 (TX DAC) / AD9680 (RX ADC) come up
 * over SPI at POST_KERNEL init. This orchestrates the FPGA-side datapath the
 * chips' PLLs depend on: configure the transceivers (GT/QPLL/CPLL), configure
 * the JESD204 link cores, then hand off to the generic FSM to *enable* the link
 * in the correct phase order -- GT reset-release, lane clocks, chip PLL re-check,
 * link status.
 *
 * The configure/enable split matches the AD9081 sample: main() runs the configure
 * path, then daq2_jesd204_bringup() (daq2_bringup.c) walks the enable phases. The
 * transport (TPL) and DMA are later phases.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/cache.h>
#include <zephyr/drivers/dma.h>

#include <zephyr/drivers/misc/jesd204/axi_adxcvr.h>
#include <zephyr/drivers/misc/jesd204/axi_jesd204.h>
#include <zephyr/drivers/misc/jesd204/axi_tpl.h>
#include "daq2_bringup.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(daq2, LOG_LEVEL_INF);

#define TX_ADXCVR DEVICE_DT_GET(DT_NODELABEL(tx_adxcvr))
#define RX_ADXCVR DEVICE_DT_GET(DT_NODELABEL(rx_adxcvr))
#define TX_JESD   DEVICE_DT_GET(DT_NODELABEL(tx_jesd))
#define RX_JESD   DEVICE_DT_GET(DT_NODELABEL(rx_jesd))
#define TX_TPL    DEVICE_DT_GET(DT_NODELABEL(tx_tpl))
#define RX_TPL    DEVICE_DT_GET(DT_NODELABEL(rx_tpl))

/**
 * @brief Max JESD204 bring-up attempts before giving up (best-effort).
 *
 * The old DAQ2 FMC's TX link is physical-layer marginal: every software-visible
 * status (GT PLL/reset, DAC SERDES PLL, DAC LMFC sync) reads healthy on failed
 * attempts, yet the deframer intermittently misses CGS comma detection. Each
 * attempt fully resets the GT, so attempts are independent rolls; convergence is
 * probabilistic, hence a generous budget rather than the old 8.
 */
#define JESD_BRINGUP_MAX_ATTEMPTS 40

/*
 * RX capture window: M=2 converters, 16-bit signed each, a power-of-two number
 * of samples so the window holds a whole number of DAC-tone cycles (the loopback
 * check below depends on that). The tone sits on exact FFT bin 24.
 */
#define RX_CAPTURE_NUM_CHAN         2   /* M=2 */
#define RX_CAPTURE_SAMPLES_PER_CHAN 64
#define RX_CAPTURE_TONE_BIN         24

/*
 * DAC output tone. The DDS phase accumulator runs at the converter sample rate,
 * 1 GSPS on this link (lane 10G, M2/L4/NP16). Unlike the AD9081 sample there is
 * no coarse NCO on the AD9144/AD9680, so the tone appears directly at its
 * frequency -- no +1 GHz shift back to baseband.
 *
 * tone = sample_rate * bin / samples = 1e9 * 24 / 64 = 375 MHz (exact bin).
 */
#define DAC_DDS_SAMPLE_RATE (1000 * 1000 * 1000)     /* 1 GSPS */
#define DAC_DDS_TONE_HZ (DAC_DDS_SAMPLE_RATE / RX_CAPTURE_SAMPLES_PER_CHAN * RX_CAPTURE_TONE_BIN) /* 375 MHz */
#define DAC_DDS_SCALE_MICRO (500 * 1000)             /* 0.50 FS; 5% reads as noise */

/*
 * Loopback pass threshold: percentage of captured energy that must land in
 * RX_CAPTURE_TONE_BIN. Low enough that cable loss and ADC noise cannot fail a
 * working path, high enough that noise alone cannot pass -- 64 bins of pure
 * noise puts about 1.5% in any one bin.
 */
#define RX_CAPTURE_TONE_MIN_PCT 25

/* Generous for the small transfer; only has to bound a stall, not pace it. */
#define RX_CAPTURE_TIMEOUT_MS 100

static int16_t rx_capture_buf[RX_CAPTURE_NUM_CHAN * RX_CAPTURE_SAMPLES_PER_CHAN] __aligned(64);

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

/*
 * How much of a channel's captured energy sits in one DFT bin. Single-bin DFT,
 * integer throughout: the tone is on an exact bin, so the basis only needs the
 * cosine table above, keeping this off soft-float on an FPU-less build.
 */
static unsigned int rx_capture_bin_pct(unsigned int chan, unsigned int bin)
{
	int64_t re = 0, im = 0, energy = 0;
	int32_t mean = 0;

	/* Remove DC first: an ADC offset is a bin-0 term that would inflate the
	 * energy the fraction is taken against and understate a good tone.
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

	/* Parseval for a real signal: the bin and its mirror hold 2*|X_k|^2/N of
	 * the total. Descale the two q12 factors before dividing.
	 */
	re >>= 12;
	im >>= 12;
	return (unsigned int)((200ULL * (uint64_t)(re * re + im * im)) /
			      ((uint64_t)energy * RX_CAPTURE_SAMPLES_PER_CHAN));
}

/*
 * Report the tone fraction for every captured channel. Both of them, not just
 * ch0: which converter a cabled input lands on depends on the crossbar, so a
 * single-channel check reports "no tone" for a loopback on the other channel.
 */
static void rx_capture_check_tone(void)
{
	unsigned int best_pct = 0, best_chan = 0;

	for (unsigned int c = 0; c < RX_CAPTURE_NUM_CHAN; c++) {
		unsigned int pct = rx_capture_bin_pct(c, RX_CAPTURE_TONE_BIN);
		unsigned int top_pct = 0, top_bin = 0;
		int16_t peak = 0;

		/* Which bin actually holds the most, and how large the samples are:
		 * without these, a tone at the wrong frequency and no tone at all
		 * both just report a low fraction in the expected bin.
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

		/* One line per channel only when a channel is unexpected: carrying
		 * the tone, or piling energy into some other bin. Noise-floor
		 * channels say nothing.
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
 * Fill rx_capture_buf with one DMA transfer's worth of samples. Returns 0 once
 * the buffer holds fresh, cache-invalidated samples.
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

	/* No interrupt wired to this core -- dma_get_status() self-pumps the
	 * transfer on each poll, so it must be polled to completion. Bounded so a
	 * stalled transfer cannot hang a link that is already up.
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
 * Kick a single DEV_TO_MEM transfer on rx_dmac and check the captured samples
 * for the loopback tone, so a signal fed into the ADC can be confirmed present
 * in the digital samples without any host-side IIO tooling.
 *
 * Best-effort: the link is already up, and with no loopback cable fitted a "no
 * tone" result is expected, so this warns rather than failing.
 */
static void rx_capture_dump(void)
{
	if (rx_capture_fetch()) {
		return;
	}

	rx_capture_check_tone();

#if defined(RX_SAMPLE_DUMP)
	/* Raw samples, one line each. Only useful when the per-bin figures above
	 * are themselves in doubt, so off by default.
	 */
	LOG_INF("RX capture: ch0, all %u samples:", RX_CAPTURE_SAMPLES_PER_CHAN);
	for (int i = 0; i < RX_CAPTURE_SAMPLES_PER_CHAN; i++) {
		LOG_INF("  [%2d] ch0 = %6d", i,
			rx_capture_buf[i * RX_CAPTURE_NUM_CHAN]);
	}
#endif
}

int main(void)
{
	int ret;

	LOG_INF("DAQ2 JESD204 bring-up (Phase 1b: enable the link)");

	/*
	 * Step 1: transceivers. Brings up the GT and its PLL (TX QPLL0 / RX
	 * CPLL); axi_adxcvr_configure() logs each PLL's lock. TX before RX, the
	 * order the AD9081 sample uses.
	 */
	ret = axi_adxcvr_configure(TX_ADXCVR);
	if (ret == 0) {
		ret = axi_adxcvr_configure(RX_ADXCVR);
	}
	if (ret) {
		LOG_ERR("AXI adxcvr (GT) config failed (%d)", ret);
		return ret;
	}
	LOG_INF("SUCCESS: GT transceivers configured (TX QPLL0 / RX CPLL)");

	/*
	 * Step 2b: TPL transport cores -- RX sample format/enable, TX data-source
	 * select. Configured before bring-up; verified (axi_tpl_enable) after DATA,
	 * because the DAC SYNC and STATUS/clock readback are only meaningful against
	 * a running sample clock. Best-effort: the link is the deliverable here.
	 * Config-only and independent of the link cores, so done once, ahead of the
	 * bring-up retry loop below.
	 */
	if (axi_tpl_configure(RX_TPL) || axi_tpl_configure(TX_TPL)) {
		LOG_WRN("TPL transport config failed (continuing, link is unaffected)");
	} else {
		LOG_INF("SUCCESS: TPL transport cores configured (%d converters)",
			DT_PROP(DT_NODELABEL(rx_tpl), adi_num_channels));
	}

	/*
	 * Steps 2+3, retried: the TX link (FPGA framer -> AD9144 deframer) on this
	 * hot, old DAQ2 FMC is marginal -- it reaches DATA on some boots and stalls
	 * at CGS/ILAS on others (verified 2026-09-07, same binary, boot-to-boot).
	 * Re-training a stalled link needs a LINK_DISABLE 1->0 edge:
	 * axi_jesd204_configure() re-asserts disable (=1), the FSM's lane-clock
	 * enable clears it (=0), so each pass restarts CGS. Re-running bring-up
	 * alone would re-write 0 to an already-enabled link -- no edge, no retrain.
	 * Both links are re-configured/re-trained each pass; RX comes up reliably
	 * and re-training it is harmless. Best-effort is preserved:
	 * daq2_jesd204_bringup() still logs how far each link advanced every pass,
	 * and a run that never reaches DATA warns rather than aborts.
	 */
	for (int attempt = 1; attempt <= JESD_BRINGUP_MAX_ATTEMPTS; attempt++) {
		/* Step 2: JESD204 link cores -- program geometry + ILAS, held
		 * disabled. TX before RX, as in the AD9081 sample. */
		ret = axi_jesd204_configure(TX_JESD);
		if (ret == 0) {
			ret = axi_jesd204_configure(RX_JESD);
		}
		if (ret) {
			LOG_ERR("AXI jesd204 link config failed (%d)", ret);
			return ret;
		}
		if (attempt == 1) {
			LOG_INF("SUCCESS: JESD204 link cores configured (M%d/L%d/F%d/K%d)",
				DT_PROP(DT_NODELABEL(tx_jesd), adi_converters_per_device),
				DT_PROP(DT_NODELABEL(tx_jesd), adi_lanes_per_device),
				DT_PROP(DT_NODELABEL(tx_jesd), adi_octets_per_frame),
				DT_PROP(DT_NODELABEL(tx_jesd), adi_frames_per_multiframe));
		}

		/* Step 3: enable the link via the generic FSM -- releases the GT
		 * reset, enables the FPGA lane clocks, re-reads each chip PLL, and
		 * polls each link core for DATA. */
		ret = daq2_jesd204_bringup();
		if (ret == 0) {
			LOG_INF("SUCCESS: DAQ2 JESD204 links up (both ends carrying DATA)"
				" on attempt %d", attempt);
			/* Both links up: dump each core's full status table once. */
			axi_jesd204_status_print(TX_JESD);
			axi_jesd204_status_print(RX_JESD);
			break;
		}
		LOG_WRN("DAQ2 JESD204 link bring-up incomplete (%d), attempt %d/%d",
			ret, attempt, JESD_BRINGUP_MAX_ATTEMPTS);
	}
	if (ret) {
		LOG_WRN("DAQ2 JESD204 link did not reach DATA after %d attempts",
			JESD_BRINGUP_MAX_ATTEMPTS);
	}

	/*
	 * Step 4: per-lane RX watchdog snapshot. Best-effort diagnostic on top of
	 * the status the FSM's LINK_RUNNING phase already read.
	 */
	(void)axi_jesd204_rx_watchdog(RX_JESD);

	/*
	 * Step 5: verify the transport layer now that the link underneath carries
	 * DATA -- re-latches the DAC SYNC and reads both cores' STATUS. Best-effort:
	 * a TPL complaint is a datapath issue below the link, not a link failure.
	 */
	if (axi_tpl_enable(RX_TPL, TX_TPL)) {
		LOG_WRN("TPL post-link verify failed (link is up regardless)");
	}

	/*
	 * Step 6: drive the DAC converters from the transport core's internal DDS.
	 * No coarse NCO on the AD9144, so the tone appears directly at 375 MHz --
	 * scope the DAC SMA to see it. Best-effort.
	 */
	if (axi_tpl_tx_dds(TX_TPL, DAC_DDS_TONE_HZ, DAC_DDS_SAMPLE_RATE,
			   DAC_DDS_SCALE_MICRO, true)) {
		LOG_WRN("could not arm the DAC DDS tone");
	} else {
		LOG_INF("SUCCESS: DAC emitting a %u MHz DDS tone at %u%% full scale",
			DAC_DDS_TONE_HZ / 1000000U, DAC_DDS_SCALE_MICRO / 10000U);
	}

	/*
	 * Step 7: RX capture over rx_dmac + single-bin loopback check. Confirms
	 * samples reach the CPU (proves the RX datapath even with no cable, via ADC
	 * noise) and, with a DAC->ADC cable fitted, that the tone completes the
	 * loopback.
	 */
	rx_capture_dump();

	LOG_INF("SUCCESS: DAQ2 Phase 1b bring-up complete");

	return 0;
}
