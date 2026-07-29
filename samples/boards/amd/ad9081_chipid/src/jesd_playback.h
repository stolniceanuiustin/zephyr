/*
 * JESD204 datapath validation -- Rung 4: DDR -> DMA -> DAC playback.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef JESD_PLAYBACK_H_
#define JESD_PLAYBACK_H_

/*
 * Tone geometry, shared with the Rung 5 loopback check so both ends agree on
 * what is being played without duplicating the constants.
 *
 * Sample rates: the TX datapath runs at DAC 12 GHz / (interp 6 x 8) = 250 MSPS
 * and the RX datapath decimates ADC 4 GHz / (4 x 4) to the same 250 MSPS. The RX
 * coarse DDC shift (-2 GHz) is the exact negation of the TX main NCO (+2 GHz), so
 * an analog loopback returns the tone to the baseband frequency it started at.
 *
 * The tone must be placed on the LOW side of the NCO, not the high side. The ADC
 * samples at 4 GHz, so its first Nyquist zone ends at 2000 MHz -- exactly where
 * the +2 GHz NCO sits. A positive baseband frequency would put the RF tone above
 * 2000 MHz, where it aliases back as its own mirror image (returning as -f, which
 * the correlator rejects) and is additionally buried by the ADC and input balun
 * rolloff at the Nyquist edge. So the baseband tone is negative: the table is
 * stepped as a negative-frequency exponential (I = cos, Q = -sin), putting the RF
 * tone at 2000 - 31.25 = 1968.75 MHz, comfortably inside Nyquist.
 *
 * Frequency is set by how far the 64-entry table is stepped per beat:
 * f = -rate * STEP / 64. STEP=8 gives -31.25 MHz. Because 512 beats x 8 steps is
 * a whole multiple of 64, the tone still lands exactly on a bin, keeping the
 * loopback check a clean single-bin test with no spectral leakage.
 */
#define JESD_PB_TABLE_LEN    64U
#define JESD_PB_STEP         8U
#define JESD_PB_SAMPLE_RATE  250000000U
#define JESD_PB_TONE_HZ      (JESD_PB_SAMPLE_RATE * JESD_PB_STEP / JESD_PB_TABLE_LEN)
#define JESD_PB_AMPLITUDE    24576

/*
 * Rung 4: build a sine table in DDR and push it out through the TX AXI DMAC
 * (MEM_TO_DEV) -> dac_tpl_core -> JRX deframer -> DAC. Must be called only after
 * the JESD204 link has reached DATA. Runs one bounded transfer to prove the
 * engine completes, then (if the core supports it) leaves a cyclic transfer
 * running so the tone is continuously present at the DAC output for a scope.
 *
 * Returns 0 if the transfer completed and the link stayed healthy underneath it,
 * negative errno otherwise. Note this verifies the *transmit mechanism*, not the
 * analog output -- only a scope (or Rung 5's analog loopback) can do that.
 */
int jesd_playback_sine(void);

/*
 * Expose the playback buffer so a diagnostic can inspect what the DMA is actually
 * sourcing. Returns 0 and fills *buf / *bytes, or -EINVAL on a null argument.
 *
 * The caller is reading a buffer the DMA reads from DDR, so to see memory's view
 * rather than the CPU's it must invalidate the range first -- pb_fill() wrote it
 * through the D-cache and flushed, and a plain read would be answered by those same
 * lines, confirming nothing.
 */
int jesd_playback_buffer(const int16_t **buf, size_t *bytes);

/*
 * Time one bounded (non-cyclic) transfer of `bytes` from the playback buffer and
 * return how long it took in microseconds.
 *
 * This is the only way to get a throughput number out of this core. The wrap
 * counting the diagnostic used to attempt cannot work: under cyclic=hw the
 * hardware replays one transfer forever, so no fresh EOT ever latches and the
 * wrap count is permanently zero. A *bounded* transfer, by contrast, has a
 * defined beginning and end, and the DMA API reports exactly when it finishes --
 * so bytes/elapsed is a real measurement rather than an inference.
 *
 * What the number means depends on what is downstream, which is the point of
 * being able to call this twice:
 *
 *  - with the TX offload in bypass the sink is the TPL itself, backpressuring at
 *    the link's 250 MSPS x 16 B = 4000 MB/s, so the result is
 *    min(DMA rate, line rate) -- what the DAC actually receives.
 *  - with the offload in store-and-replay the sink is its storage buffer, which
 *    is wider and faster than the link, so the result is much closer to the DMA's
 *    own DDR read throughput.
 *
 * Two sizes give a third useful thing: the difference between them divides out
 * the fixed per-transfer overhead (config, cache flush, arming, the poll loop's
 * latency in noticing completion), which at these durations is not negligible.
 *
 * Leaves the channel stopped -- call jesd_playback_rearm() to put the continuous
 * tone back. Returns 0 on success, -EINVAL for a size outside the buffer,
 * -ETIMEDOUT if the transfer never completed, or another negative errno from the
 * DMA API.
 */
int jesd_playback_timed(size_t bytes, uint32_t *elapsed_us);

/*
 * Re-arm the cyclic whole-buffer playback, restoring the continuous tone after
 * jesd_playback_timed() has borrowed the channel. Returns 0 on success.
 */
int jesd_playback_rearm(void);

#endif /* JESD_PLAYBACK_H_ */
