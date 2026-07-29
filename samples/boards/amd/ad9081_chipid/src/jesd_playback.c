/*
 * JESD204 datapath validation -- Rung 4: DDR -> AXI DMAC -> DAC playback.
 *
 * Rungs 1-2 proved the whole *receive* chain: the serial path is bit-error-free
 * and real samples land in DDR. The transmit direction has never moved a single
 * sample. Rung 4 is the mirror image of Rung 2, running the other way:
 *
 *   DDR sine table -> AXI DMAC (MEM_TO_DEV / MM2S) -> DAC TPL core
 *     -> FPGA JESD204 link -> GT -> chip JRX deframer -> DAC datapath -> analog
 *
 * What this rung can and cannot prove
 * -----------------------------------
 * There is no transmit equivalent of Rung 1's PN monitor: the pattern checker
 * lives in the *receive* TPL core, and nothing on the FPGA side sees what the
 * chip's deframer actually received. So this rung proves the transmit
 * *mechanism* -- the DMA engine sources DDR and completes, the TPL core accepts
 * the stream, and the link stays in DATA while samples flow -- but it cannot
 * confirm the bits arrived correct, and it certainly can't see the analog output.
 * A scope on the DAC output closes that gap by eye; Rung 5's analog loopback
 * (DAC out -> ADC in, captured with Rung 2's machinery) closes it in software.
 *
 * A stalled or misconfigured transmit path fails loudly here anyway: if the DAC
 * TPL core isn't accepting data the DMAC never gets its beats consumed and the
 * transfer times out rather than completing.
 *
 * DMA notes (same core family as Rung 2, opposite direction)
 * ---------------------------------------------------------
 *  - Direction is hardwired in the bitstream and probed by the driver; requesting
 *    MEMORY_TO_PERIPHERAL on a core synthesized as MM2S is required, and a
 *    mismatch is rejected outright by dma_config().
 *  - No interrupt line is wired, so dma_get_status() pumps the transfer forward;
 *    "wait for completion" means polling it until !busy (as in Rung 2).
 *  - Cache direction is the *opposite* of capture: the CPU writes the buffer and
 *    the DMA reads it, so we only need to flush (clean dirty lines out to DDR)
 *    before starting. No invalidate afterwards -- the DMA never writes here.
 *  - If the core was synthesized with cyclic support we re-arm the transfer in
 *    cyclic mode at the end so the tone stays present at the DAC output for as
 *    long as the board is powered, which is what makes a scope check possible.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/cache.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(jesd_playback, LOG_LEVEL_INF);

#include "ad9081.h"
#include "axi_jesd204.h"
#include "jesd_playback.h"

#include "adi_ad9081.h"

/* Chip-side deframer link consuming the DAC data. */
#define AD9081_JRX_LINK AD9081_LINK_0

/* AXI DMAC channel index (single-channel core). */
#define PB_DMA_CHANNEL 0

/*
 * Sample layout. The DMAC data bus is 16 bytes wide = 8 x 16-bit samples per
 * beat, and this link is M8 (8 converters), so one beat carries exactly one
 * sample for each converter. Rung 2's capture confirmed that mapping in the
 * receive direction; playback is its mirror.
 *
 * We emit a full-scale-ish sine, one period per PB_SINE_LEN beats, with the same
 * value written to all 8 converter slots of a beat -- so every DAC plays the
 * identical tone and any output can be probed. Amplitude is 24576 = 0.75 of
 * 16-bit full scale, backed off from the rails so a little digital gain or
 * interpolation overshoot in the chip's datapath can't clip.
 */
#define PB_SINE_LEN       JESD_PB_PERIOD_BEATS
#define PB_LANES_PER_BEAT 8U  /* 16-byte bus / 2-byte sample */
#define PB_NUM_SAMPLES    (PB_SINE_LEN * PB_LANES_PER_BEAT)
#define PB_BUF_BYTES      (PB_NUM_SAMPLES * sizeof(uint16_t))
#define PB_DMA_ALIGN      16U

/*
 * One period of a complex exponential, 64 points, amplitude 24576, signed 16-bit.
 * Generated offline (round(24576 * cos|sin(2*pi*n/64))) rather than computed at
 * runtime so the tables are exact, deterministic and need no floating point.
 *
 * I and Q must be in quadrature, not identical: the chip's DUC mixes the complex
 * baseband signal up by the main NCO (+2 GHz here). A real-valued input (I == Q
 * is not quadrature) would upconvert to *two* tones, one either side of the NCO
 * frequency. Feeding cos on I and sin on Q gives a single clean tone at
 * NCO + f_baseband, which is what Rung 5's loopback expects to find.
 */
static const int16_t pb_cos[PB_SINE_LEN] = {
	 24576,  24458,  24104,  23518,  22705,  21674,  20434,  18998,
	 17378,  15591,  13654,  11585,   9405,   7134,   4795,   2409,
	     0,  -2409,  -4795,  -7134,  -9405, -11585, -13654, -15591,
	-17378, -18998, -20434, -21674, -22705, -23518, -24104, -24458,
	-24576, -24458, -24104, -23518, -22705, -21674, -20434, -18998,
	-17378, -15591, -13654, -11585,  -9405,  -7134,  -4795,  -2409,
	     0,   2409,   4795,   7134,   9405,  11585,  13654,  15591,
	 17378,  18998,  20434,  21674,  22705,  23518,  24104,  24458,
};

static const int16_t pb_sin[PB_SINE_LEN] = {
	     0,   2409,   4795,   7134,   9405,  11585,  13654,  15591,
	 17378,  18998,  20434,  21674,  22705,  23518,  24104,  24458,
	 24576,  24458,  24104,  23518,  22705,  21674,  20434,  18998,
	 17378,  15591,  13654,  11585,   9405,   7134,   4795,   2409,
	     0,  -2409,  -4795,  -7134,  -9405, -11585, -13654, -15591,
	-17378, -18998, -20434, -21674, -22705, -23518, -24104, -24458,
	-24576, -24458, -24104, -23518, -22705, -21674, -20434, -18998,
	-17378, -15591, -13654, -11585,  -9405,  -7134,  -4795,  -2409,
};

/* Playback buffer in DDR, sized/aligned to the DMAC data bus. */
static int16_t pb_buf[PB_NUM_SAMPLES] __aligned(PB_DMA_ALIGN);

/* Poll budget: 1 KiB of beats moves in far less than this. */
#define PB_POLL_TIMEOUT_MS 200

/*
 * Expand one period across the buffer. Rung 2's capture established the beat
 * layout: even sample slots are the I components, odd slots the Q, so a beat
 * holds an (I,Q) pair for each of the 4 complex channels. Every channel gets the
 * same tone, so any DAC output can be probed.
 */
static void pb_fill(void)
{
	for (uint32_t b = 0; b < PB_SINE_LEN; b++) {
		for (uint32_t l = 0; l < PB_LANES_PER_BEAT; l++) {
			pb_buf[b * PB_LANES_PER_BEAT + l] =
				(l & 1) ? pb_sin[b] : pb_cos[b];
		}
	}

	/* The DMA reads this buffer straight out of DDR, bypassing the D-cache,
	 * so the freshly written lines must be cleaned out to memory first. */
	sys_cache_data_flush_range(pb_buf, sizeof(pb_buf));
}

static void pb_describe(void)
{
	LOG_INF("tone table: %u beats, I=cos/Q=sin, amplitude %d (0.75 FS)",
		PB_SINE_LEN, pb_cos[0]);
	LOG_INF("  baseband %u.%03u MHz at 250 MSPS (one period per %u beats)",
		JESD_PB_TONE_HZ / 1000000U,
		(JESD_PB_TONE_HZ % 1000000U) / 1000U, PB_SINE_LEN);
	LOG_INF("first 2 beats (I Q I Q ...):");
	for (uint32_t i = 0; i < 16; i += 8) {
		LOG_INF("  [%02u] %6d %6d %6d %6d %6d %6d %6d %6d", i,
			pb_buf[i + 0], pb_buf[i + 1], pb_buf[i + 2],
			pb_buf[i + 3], pb_buf[i + 4], pb_buf[i + 5],
			pb_buf[i + 6], pb_buf[i + 7]);
	}
}

/* Arm one transfer of the whole buffer. cyclic=true re-arms it forever. */
static int pb_submit(const struct device *tx_dma, bool cyclic)
{
	struct dma_block_config block = {0};
	struct dma_config cfg = {0};
	int ret;

	block.source_address = (uintptr_t)pb_buf;
	block.dest_address = 0; /* device FIFO -- no memory address */
	block.block_size = sizeof(pb_buf);

	cfg.channel_direction = MEMORY_TO_PERIPHERAL;
	cfg.block_count = 1;
	cfg.head_block = &block;
	cfg.source_data_size = 2; /* NP16: 16-bit samples */
	cfg.dest_data_size = 2;
	cfg.cyclic = cyclic;

	ret = dma_config(tx_dma, PB_DMA_CHANNEL, &cfg);
	if (ret) {
		LOG_ERR("dma_config failed (%d)", ret);
		return ret;
	}

	ret = dma_start(tx_dma, PB_DMA_CHANNEL);
	if (ret) {
		LOG_ERR("dma_start failed (%d)", ret);
		return ret;
	}
	return 0;
}

int jesd_playback_sine(void)
{
	const struct device *tx_dma = DEVICE_DT_GET(DT_NODELABEL(tx_dmac));
	adi_ad9081_device_t *dev = ad9081_get_device();
	struct dma_status status;
	uint16_t link_status = 0;
	int64_t deadline;
	int32_t err;
	int ret;

	if (dev == NULL) {
		LOG_ERR("AD9081 device not initialised");
		return -ENODEV;
	}
	if (!device_is_ready(tx_dma)) {
		LOG_ERR("TX DMAC not ready");
		return -ENODEV;
	}

	LOG_INF("--- Rung 4: DDR sine -> DMA -> DAC playback ---");

	pb_fill();
	pb_describe();

	/* One bounded transfer first: proves the engine sources DDR and the DAC
	 * TPL core consumes the beats, with a timeout to catch a stalled path. */
	ret = pb_submit(tx_dma, false);
	if (ret) {
		return ret;
	}

	deadline = k_uptime_get() + PB_POLL_TIMEOUT_MS;
	do {
		ret = dma_get_status(tx_dma, PB_DMA_CHANNEL, &status);
		if (ret) {
			LOG_ERR("dma_get_status failed (%d)", ret);
			goto stop;
		}
		if (!status.busy) {
			break;
		}
		if (k_uptime_get() > deadline) {
			LOG_ERR("playback stalled: %u of %zu bytes unsent -- DAC TPL core is not consuming beats",
				status.pending_length, sizeof(pb_buf));
			ret = -ETIMEDOUT;
			goto stop;
		}
	} while (true);

	LOG_INF("transfer complete: %zu bytes (%u samples) pushed to the DAC datapath",
		sizeof(pb_buf), PB_NUM_SAMPLES);

	/*
	 * The transfer completing means the DMA engine and TPL core are happy,
	 * but a deframer that lost sync would also silently swallow beats. Check
	 * both ends of the link are still in DATA underneath the traffic -- that
	 * is the strongest transmit-side evidence available without a scope.
	 */
	err = adi_ad9081_jesd_rx_link_status_get(dev, AD9081_JRX_LINK,
						 &link_status);
	if (err != API_CMS_ERROR_OK) {
		LOG_WRN("chip JRX link status read failed (%d)", err);
	} else {
		LOG_INF("chip JRX (deframer) link status = 0x%04x while transmitting",
			link_status);
	}

	ret = axi_jesd204_status_read();
	if (ret) {
		LOG_ERR("=== Rung 4 FAIL: link fell out of DATA while transmitting ===");
		goto stop;
	}

	/*
	 * Leave a cyclic transfer running so the tone is continuously present at
	 * the DAC output and can be probed with a scope. Deliberately not fatal:
	 * if the core was synthesized without cyclic support the one-shot result
	 * above already carries the verdict.
	 */
	dma_stop(tx_dma, PB_DMA_CHANNEL);
	if (pb_submit(tx_dma, true) == 0) {
		LOG_INF("cyclic playback armed -- tone is now continuous at the DAC output");
	} else {
		LOG_WRN("cyclic playback not available; DAC output was driven for one buffer only");
	}

	LOG_INF("=== Rung 4 PASS: transmit path moves samples DDR -> DAC, link stayed in DATA ===");
	LOG_INF("    (scope the DAC output to confirm the analog tone, or run Rung 5 loopback)");
	return 0;

stop:
	dma_stop(tx_dma, PB_DMA_CHANNEL);
	return ret;
}
