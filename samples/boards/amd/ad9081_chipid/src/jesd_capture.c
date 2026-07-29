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
 * 8 KiB = 4096 x 16-bit samples -- enough to see the ramp wrap several times
 * while staying tiny. Placed in DDR (the only RAM here); DMA-visible.
 */
#define CAP_BUF_BYTES   8192U
#define CAP_DMA_ALIGN   16U
#define CAP_NUM_SAMPLES (CAP_BUF_BYTES / sizeof(uint16_t))

static uint16_t cap_buf[CAP_NUM_SAMPLES] __aligned(CAP_DMA_ALIGN);

/* Poll budget: the whole 8 KiB block should move in well under this. */
#define CAP_POLL_TIMEOUT_MS 200

int jesd_capture_ramp(void)
{
	const struct device *rx_dma = DEVICE_DT_GET(DT_NODELABEL(rx_dmac));
	adi_ad9081_device_t *dev = ad9081_get_device();
	struct dma_block_config block = {0};
	struct dma_config cfg = {0};
	struct dma_status status;
	int64_t deadline;
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

	/* Prepare the buffer + descriptor. */
	memset(cap_buf, 0, sizeof(cap_buf));
	sys_cache_data_flush_and_invd_range(cap_buf, sizeof(cap_buf));

	block.source_address = 0; /* device FIFO -- no memory address */
	block.dest_address = (uintptr_t)cap_buf;
	block.block_size = sizeof(cap_buf);

	cfg.channel_direction = PERIPHERAL_TO_MEMORY;
	cfg.block_count = 1;
	cfg.head_block = &block;
	cfg.source_data_size = 2; /* NP16: 16-bit samples */
	cfg.dest_data_size = 2;

	ret = dma_config(rx_dma, CAP_DMA_CHANNEL, &cfg);
	if (ret) {
		LOG_ERR("dma_config failed (%d)", ret);
		goto restore;
	}

	ret = dma_start(rx_dma, CAP_DMA_CHANNEL);
	if (ret) {
		LOG_ERR("dma_start failed (%d)", ret);
		goto restore;
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
	sys_cache_data_invd_range(cap_buf, sizeof(cap_buf));

	ret = jesd_capture_analyze(cap_buf, CAP_NUM_SAMPLES);

stop:
	dma_stop(rx_dma, CAP_DMA_CHANNEL);
restore:
	err = adi_ad9081_adc_test_mode_config_set(dev, AD9081_TMODE_OFF,
						  AD9081_TMODE_OFF,
						  AD9081_JTX_LINK);
	if (err != API_CMS_ERROR_OK) {
		LOG_WRN("failed to restore ADC normal mode (%d)", err);
	}

	return ret;
}

/*
 * Inspect a captured block. We don't yet assume the exact converter interleave
 * across the 16-byte bus, so this is deliberately lenient: dump the first
 * samples so the ramp is visible in the log, then sanity-check that the buffer
 * (a) isn't all-zero (DMA never wrote / cache stale) and (b) isn't stuck at one
 * value (link/datapath dead). A strict per-lane ramp check comes once the
 * interleave is confirmed from this raw dump.
 */
int jesd_capture_analyze(const uint16_t *buf, size_t n)
{
	uint16_t first = buf[0];
	bool all_zero = true;
	bool all_same = true;
	size_t ramp_steps = 0;

	LOG_INF("first 16 samples:");
	for (size_t i = 0; i < 16 && i < n; i += 8) {
		LOG_INF("  [%02zu] %04x %04x %04x %04x %04x %04x %04x %04x",
			i, buf[i + 0], buf[i + 1], buf[i + 2], buf[i + 3],
			buf[i + 4], buf[i + 5], buf[i + 6], buf[i + 7]);
	}

	for (size_t i = 0; i < n; i++) {
		if (buf[i] != 0) {
			all_zero = false;
		}
		if (buf[i] != first) {
			all_same = false;
		}
	}

	/*
	 * Count adjacent +1 steps as a rough "does it ramp?" signal. With M8
	 * interleave the stride may not be 1 sample, but a ramp still produces
	 * many incrementing neighbours; a dead/garbage buffer produces few.
	 */
	for (size_t i = 1; i < n; i++) {
		if ((uint16_t)(buf[i] - buf[i - 1]) == 1U) {
			ramp_steps++;
		}
	}

	LOG_INF("analysis: all_zero=%d all_same=%d adjacent_+1_steps=%zu/%zu",
		all_zero, all_same, ramp_steps, n - 1);

	if (all_zero) {
		LOG_ERR("capture is all-zero: DMA didn't write, or samples never arrived");
		return -EIO;
	}
	if (all_same) {
		LOG_ERR("capture is a constant 0x%04x: datapath stuck, no live samples",
			first);
		return -EIO;
	}

	LOG_INF("=== Rung 2 PASS: live samples captured to DDR ===");
	LOG_INF("    (ramp interleave to be confirmed from the dump above)");
	return 0;
}
