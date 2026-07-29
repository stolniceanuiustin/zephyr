/*
 * JESD204 datapath validation -- Rung 2: ADC ramp -> AXI DMAC -> DDR capture.
 *
 * Rung 1 proved the receive *serial* path carries correct bits, but the FPGA
 * checked the pattern internally -- no sample ever reached memory. Rung 2 adds
 * the final hop and moves real samples into DDR:
 *
 *   chip ADC datapath (RAMP test mode)
 *     -> JTX framer -> GT -> FPGA JESD204 link -> RX TPL core
 *     -> AXI DMAC (DEV_TO_MEM / S2MM) -> DDR buffer -> CPU verify
 *
 * The chip emits a deterministic incrementing ramp; the RX DMAC captures a
 * block into a DDR buffer; the CPU reads it back and checks it looks like a
 * ramp. A clean capture proves the *whole* receive chain end to end, including
 * the transport core and DMA -- the point of the whole bring-up.
 *
 * DMA notes:
 *  - The core is synthesized without an interrupt line (see the overlay), so the
 *    Zephyr driver advances the transfer whenever dma_get_status() is polled.
 *    "Wait for completion" therefore means poll get_status() until !busy.
 *  - The DMA writes DDR directly, bypassing the A53 data cache. We flush+invalidate
 *    the buffer before the transfer (drop any dirty lines) and invalidate it after
 *    (drop stale lines) so the CPU reads what the DMA actually wrote.
 *  - The core's data bus is 16 bytes wide; the buffer is aligned/sized to match.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/cache.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(jesd_capture, LOG_LEVEL_INF);

#include "ad9081.h"
#include "jesd_capture.h"

#include "adi_ad9081.h"

/* Chip-side framer link carrying the ADC data. */
#define AD9081_JTX_LINK AD9081_LINK_0

/* AXI DMAC channel index (single-channel core). */
#define CAP_DMA_CHANNEL 0

/*
 * Capture buffer. The DMAC data bus is 16 bytes wide, so align and size to that.
 * Placed in DDR (the only RAM here); DMA-visible.
 *
 * Size is a *time window* decision. One beat is consumed per sample period, so at
 * 250 MSPS the buffer duration is beats/250e6: the original 8 KiB (512 beats) is a
 * 2 us snapshot. That is ample for Rung 2's ramp, which is deterministic and
 * identical everywhere, but far too short to characterise an intermittent analog
 * return -- eight such captures sample 16 us out of a multi-second run, so a
 * "1 in 8 captures saw signal" result says nothing about how often the signal is
 * actually there.
 *
 * 256 KiB (~65 us) settled the first question: the return is not bursty at
 * microsecond scale. A capture reads either the tone or the noise floor uniformly
 * across all of it, never a mix -- so whatever varies has a period longer than a
 * capture, and a window that cannot contain a transition cannot time one.
 *
 * 1 MiB = 65536 beats = ~262 us is the largest size that works, and the reason is
 * now known: it is the size of the RX axi_data_offload core sitting between the
 * DMAC and the TPL (see axi_data_offload.h). A 4 MiB request truncates -- the ramp
 * is perfect through beat 65535 and every beat after it reads zero, with no timeout
 * and no error anywhere -- because that core is one-shot by construction (its
 * oneshot bit resets to ~TX_OR_RXN_PATH, and RX has TX_OR_RXN_PATH=0): it hands
 * over exactly one bufferful and goes idle.
 *
 * This is why probing the burst-length register never explained it. The driver
 * reads max_length from the core and this one answers 16 MiB, which is true and
 * irrelevant -- the limit was one block upstream, in an IP this sample did not
 * know existed.
 *
 * With the offload in bypass (main.c does this at startup) the store-and-replay
 * cycle is gone, so this ceiling should no longer apply. It has not been retested
 * at a larger size; if you raise it, re-run Rung 2, which is what caught the
 * truncation in the first place. It validates every beat it is given, so a
 * silently short capture fails loudly there instead of quietly skewing an analog
 * measurement.
 *
 * DDR is 2 GB, so the cost is irrelevant; the limit is the hardware's, not memory.
 */
#define CAP_BUF_BYTES   (1024U * 1024U)
#define CAP_DMA_ALIGN   16U
#define CAP_NUM_SAMPLES (CAP_BUF_BYTES / sizeof(uint16_t))

static uint16_t cap_buf[CAP_NUM_SAMPLES] __aligned(CAP_DMA_ALIGN);

/*
 * Poll budget. At the link's 4 GB/s the block moves in about a millisecond, but the
 * transfer is advanced by dma_get_status() polls rather than an interrupt, so the
 * achieved rate depends on how often this loop runs. Allow generously -- the point
 * of the timeout is to catch a stalled path, not a slow one.
 */
#define CAP_POLL_TIMEOUT_MS 2000

/*
 * Run one capture into cap_buf: arm the DMAC, poll it to completion, and do the
 * cache maintenance around it. Shared by Rung 2 (with a test mode enabled) and
 * Rung 5 (capturing whatever the ADC is really digitising).
 */
static int cap_transfer_n(const struct device *rx_dma, size_t bytes);

static int cap_transfer(const struct device *rx_dma)
{
	return cap_transfer_n(rx_dma, sizeof(cap_buf));
}

/*
 * Capture `bytes` rather than the whole buffer.
 *
 * The cost of a capture is not dominated by the transfer. Every capture memsets the
 * buffer, cleans it out of the D-cache, and invalidates it again afterwards -- all
 * proportional to the length, and at 1 MiB that is milliseconds of CPU work around
 * ~262 us of DMA. A caller that only needs a yes/no answer about the signal can ask
 * for a short block and get a far higher sampling rate: at 8 KiB the whole thing
 * costs tens of microseconds, which is what makes it possible to sample a periodic
 * gate faster than it toggles.
 */
static int cap_transfer_n(const struct device *rx_dma, size_t bytes)
{
	struct dma_block_config block = {0};
	struct dma_config cfg = {0};
	struct dma_status status;
	int64_t deadline;
	int ret;

	if (bytes == 0 || bytes > sizeof(cap_buf)) {
		return -EINVAL;
	}
	bytes = ROUND_DOWN(bytes, CAP_DMA_ALIGN);

	/* Drop any dirty lines so the DMA's writes can't be overwritten later. */
	memset(cap_buf, 0, bytes);
	sys_cache_data_flush_and_invd_range(cap_buf, bytes);

	block.source_address = 0; /* device FIFO -- no memory address */
	block.dest_address = (uintptr_t)cap_buf;
	block.block_size = bytes;

	cfg.channel_direction = PERIPHERAL_TO_MEMORY;
	cfg.block_count = 1;
	cfg.head_block = &block;
	cfg.source_data_size = 2; /* NP16: 16-bit samples */
	cfg.dest_data_size = 2;

	ret = dma_config(rx_dma, CAP_DMA_CHANNEL, &cfg);
	if (ret) {
		LOG_ERR("dma_config failed (%d)", ret);
		return ret;
	}

	ret = dma_start(rx_dma, CAP_DMA_CHANNEL);
	if (ret) {
		LOG_ERR("dma_start failed (%d)", ret);
		return ret;
	}

	/* Poll to completion -- get_status() also pumps the (IRQ-less) transfer. */
	deadline = k_uptime_get() + CAP_POLL_TIMEOUT_MS;
	do {
		ret = dma_get_status(rx_dma, CAP_DMA_CHANNEL, &status);
		if (ret) {
			LOG_ERR("dma_get_status failed (%d)", ret);
			goto stop;
		}
		if (!status.busy) {
			break;
		}
		if (k_uptime_get() > deadline) {
			LOG_ERR("capture timed out (pending=%u bytes)",
				status.pending_length);
			ret = -ETIMEDOUT;
			goto stop;
		}
	} while (true);

	/* CPU must invalidate before reading DMA-written DDR. */
	sys_cache_data_invd_range(cap_buf, bytes);
	ret = 0;
stop:
	dma_stop(rx_dma, CAP_DMA_CHANNEL);
	return ret;
}

/*
 * Fast one-bit probe: is a signal present at the ADC right now?
 *
 * Built for sampling something that changes over time, where the cost of a probe
 * sets the resolution. It captures JESD_CAP_PROBE_BYTES rather than the whole
 * buffer, and computes only a mean-square over lane 0 instead of the full
 * correlation -- enough to separate the ~4576 tone from the ~7 noise floor by three
 * orders of magnitude, which needs no precision at all.
 *
 * Returns 1 if signal is present, 0 if not, or a negative errno if the capture
 * failed. Overwrites the shared capture buffer like any other capture.
 */
int jesd_capture_probe(void)
{
	const struct device *rx_dma = DEVICE_DT_GET(DT_NODELABEL(rx_dmac));
	const int16_t *s = (const int16_t *)cap_buf;
	size_t n = JESD_CAP_PROBE_BYTES / sizeof(uint16_t);
	uint64_t energy = 0;
	int ret;

	if (!device_is_ready(rx_dma)) {
		return -ENODEV;
	}

	ret = cap_transfer_n(rx_dma, JESD_CAP_PROBE_BYTES);
	if (ret) {
		return ret;
	}

	for (size_t i = 0; i < n; i += JESD_CAP_LANES_PER_BEAT) {
		int32_t v = s[i];

		energy += (uint64_t)((int64_t)v * v);
	}

	/* Compare mean-square against the squared threshold to avoid a sqrt. */
	energy /= (n / JESD_CAP_LANES_PER_BEAT);
	return energy >= (uint64_t)JESD_CAP_PROBE_RMS_MIN * JESD_CAP_PROBE_RMS_MIN;
}

int jesd_capture_raw(const int16_t **buf, size_t *n)
{
	const struct device *rx_dma = DEVICE_DT_GET(DT_NODELABEL(rx_dmac));
	int ret;

	if (!device_is_ready(rx_dma)) {
		LOG_ERR("RX DMAC not ready");
		return -ENODEV;
	}

	ret = cap_transfer(rx_dma);
	if (ret) {
		return ret;
	}

	*buf = (const int16_t *)cap_buf;
	*n = CAP_NUM_SAMPLES;
	return 0;
}

int jesd_capture_ramp(void)
{
	const struct device *rx_dma = DEVICE_DT_GET(DT_NODELABEL(rx_dmac));
	adi_ad9081_device_t *dev = ad9081_get_device();
	int32_t err;
	int ret;

	if (dev == NULL) {
		LOG_ERR("AD9081 device not initialised");
		return -ENODEV;
	}
	if (!device_is_ready(rx_dma)) {
		LOG_ERR("RX DMAC not ready");
		return -ENODEV;
	}

	LOG_INF("--- Rung 2: ADC ramp -> DMA capture into DDR ---");

	/* Chip: drive both I and Q of the ADC datapath with an incrementing ramp. */
	err = adi_ad9081_adc_test_mode_config_set(dev, AD9081_TMODE_RAMP,
						  AD9081_TMODE_RAMP,
						  AD9081_JTX_LINK);
	if (err != API_CMS_ERROR_OK) {
		LOG_ERR("chip RAMP test-mode set failed (%d)", err);
		return -EIO;
	}
	k_msleep(2); /* let the pattern propagate through the link */

	ret = cap_transfer(rx_dma);
	if (ret == 0) {
		ret = jesd_capture_analyze(cap_buf, CAP_NUM_SAMPLES);
	}

	err = adi_ad9081_adc_test_mode_config_set(dev, AD9081_TMODE_OFF,
						  AD9081_TMODE_OFF,
						  AD9081_JTX_LINK);
	if (err != API_CMS_ERROR_OK) {
		LOG_WRN("failed to restore ADC normal mode (%d)", err);
	}

	return ret;
}

/*
 * Interleave confirmed from the first raw HW capture (2026-07-28):
 *   [00] 31d1 1919 31d1 1919 31d1 1919 31d1 1919
 *   [08] 31d2 191a 31d2 191a 31d2 191a 31d2 191a
 * Each 16-byte DMA beat = 8 x 16-bit samples. Within a beat the even lanes
 * carry one ramp and the odd lanes another (the I/Q of the M8 converters -- in
 * RAMP test mode every converter emits the identical counter, so all four even
 * lanes match and all four odd lanes match). Both ramps advance by exactly 1
 * per beat. This checker validates that model strictly, deriving the two ramp
 * start values from beat 0 (they're just wherever the counters happened to be
 * when capture began) rather than hard-coding them.
 */
#define CAP_LANES_PER_BEAT JESD_CAP_LANES_PER_BEAT
#define CAP_MISMATCH_DUMP  8 /* cap the per-mismatch log spam */

int jesd_capture_analyze(const uint16_t *buf, size_t n)
{
	uint16_t base_even, base_odd;
	size_t beats, bad = 0;

	LOG_INF("first 16 samples:");
	for (size_t i = 0; i < 16 && i < n; i += 8) {
		LOG_INF("  [%02zu] %04x %04x %04x %04x %04x %04x %04x %04x",
			i, buf[i + 0], buf[i + 1], buf[i + 2], buf[i + 3],
			buf[i + 4], buf[i + 5], buf[i + 6], buf[i + 7]);
	}

	if (n < CAP_LANES_PER_BEAT * 2) {
		LOG_ERR("capture too short to validate (%zu samples)", n);
		return -EINVAL;
	}

	base_even = buf[0];
	base_odd = buf[1];

	/* All-zero / all-constant fall out of the strict check, but call them out
	 * explicitly -- they're the two classic DMA/datapath failures. */
	if (base_even == 0 && base_odd == 0) {
		bool all_zero = true;

		for (size_t i = 0; i < n; i++) {
			if (buf[i] != 0) {
				all_zero = false;
				break;
			}
		}
		if (all_zero) {
			LOG_ERR("capture is all-zero: DMA didn't write, or no samples arrived");
			return -EIO;
		}
	}

	beats = n / CAP_LANES_PER_BEAT;
	for (size_t b = 0; b < beats; b++) {
		uint16_t exp_even = (uint16_t)(base_even + b);
		uint16_t exp_odd = (uint16_t)(base_odd + b);

		for (size_t l = 0; l < CAP_LANES_PER_BEAT; l++) {
			uint16_t v = buf[b * CAP_LANES_PER_BEAT + l];
			uint16_t exp = (l & 1) ? exp_odd : exp_even;

			if (v != exp) {
				if (bad < CAP_MISMATCH_DUMP) {
					LOG_WRN("  mismatch beat %zu lane %zu: got %04x want %04x",
						b, l, v, exp);
				}
				bad++;
			}
		}
	}

	LOG_INF("ramp check: base even=0x%04x odd=0x%04x, %zu beats, %zu mismatches",
		base_even, base_odd, beats, bad);

	if (bad) {
		LOG_ERR("=== Rung 2 FAIL: %zu/%zu samples off the ramp ===", bad, n);
		return -EIO;
	}

	LOG_INF("=== Rung 2 PASS: clean ramp captured to DDR (all %zu samples) ===",
		n);
	return 0;
}
