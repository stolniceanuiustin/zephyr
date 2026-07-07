/*
 * Copyright (c) 2026 Analog Devices, Inc.
 * SPDX-License-Identifier: Apache-2.0
 *
 * AD463x sample: capture conversion data from an AD4630/AD4631/AD4632
 * precision SAR ADC over the AXI offload data path
 * (PWMGEN CNV -> SPI Engine -> DMAC -> DDR) and print decoded codes for
 * both channels. First a one-shot batch, then continuous streaming.
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/misc/adi/ad463x.h>

#include "sysid_decode.h"

#define BATCH_SAMPLES	64
#define STREAM_SAMPLES	8

/*
 * Real-bit precision of the configured output mode. The HDL left-aligns the
 * real bits in each 32-bit word, so a code is recovered with
 * (int32_t)word >> (32 - real_bits). This must match adi,part / adi,output-mode
 * in the devicetree overlay: the default (ad4630-20, 24-diff-8-com) is 20 bits;
 * use 30 for 30-avg-diff.
 */
#define AD463X_REAL_BITS	20
#define AD463X_SIGN_SHIFT	(32 - AD463X_REAL_BITS)

static uint32_t __nocache __aligned(32) batch_buf[BATCH_SAMPLES * 2];
static uint32_t __nocache __aligned(32) stream_buf[STREAM_SAMPLES * 2];

static inline int32_t ad463x_code(uint32_t word)
{
	return (int32_t)word >> AD463X_SIGN_SHIFT;
}

/* Print FPGA build information from the AXI System ID ROM, if one is present. */
static void print_board_info(void)
{
#if DT_NODE_EXISTS(DT_NODELABEL(axi_sysid))
	const struct device *sysid = DEVICE_DT_GET(DT_NODELABEL(axi_sysid));
	struct sysid_board_info info;

	if (!device_is_ready(sysid) || sysid_get_board_info(sysid, &info) != 0) {
		return;
	}

	printf("FPGA design: %s / %s (git %.12s%s)\n",
	       info.board[0] ? info.board : "unknown",
	       info.product[0] ? info.product : "unknown",
	       info.git_hash[0] ? info.git_hash : "unknown",
	       info.git_clean ? "" : "-dirty");
#endif
}

/* One-shot batch capture: fill batch_buf, then decode and print every sample. */
static int batch_capture(const struct device *dev)
{
	size_t frame = ad463x_get_frame_size(dev);
	ssize_t got;
	int n;

	printf("\n=== Batch capture (%d samples) ===\n", BATCH_SAMPLES);

	got = ad463x_read_buffer(dev, batch_buf, sizeof(batch_buf));
	if (got < 0) {
		printf("read_buffer failed: %d\n", (int)got);
		return (int)got;
	}

	n = (int)((size_t)got / frame);
	printf("%-12s %s\n", "channel 0", "channel 1");
	for (int i = 0; i < n; i++) {
		printf("%-12d %d\n",
		       ad463x_code(batch_buf[2 * i]),
		       ad463x_code(batch_buf[2 * i + 1]));
	}

	return 0;
}

/* Continuous streaming: capture back-to-back bursts and print each one. */
static void stream_loop(const struct device *dev)
{
	size_t frame = ad463x_get_frame_size(dev);

	printf("\n=== Continuous stream ===\n");
	printf("%-12s %s\n", "channel 0", "channel 1");

	while (1) {
		ssize_t got = ad463x_read_buffer(dev, stream_buf,
						 sizeof(stream_buf));

		if (got < 0) {
			printf("read_buffer failed: %d\n", (int)got);
			k_msleep(500);
			continue;
		}

		int n = (int)((size_t)got / frame);

		for (int i = 0; i < n; i++) {
			printf("%-12d %d\n",
			       ad463x_code(stream_buf[2 * i]),
			       ad463x_code(stream_buf[2 * i + 1]));
		}

		k_msleep(500);
	}
}

int main(void)
{
	const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(ad463x));

	if (!device_is_ready(dev)) {
		printf("AD463x device not ready\n");
		return 0;
	}

	print_board_info();

	if (batch_capture(dev) == 0) {
		stream_loop(dev);
	}

	return 0;
}
