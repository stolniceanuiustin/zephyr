/*
 * ADI AXI data-offload cores -- the store-and-replay buffers between each AXI
 * DMAC and its JESD204 transport core.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef AXI_DATA_OFFLOAD_H_
#define AXI_DATA_OFFLOAD_H_

#include <stdbool.h>
#include <stdint.h>

/*
 * What this block is, and why the port needs to know about it.
 *
 * The reference HDL (hdl/projects/ad9081_fmca_ebz) does not wire the DMACs
 * straight to the transport cores. An axi_data_offload sits in between on both
 * directions, and it drives the DMA's transfer-request line:
 *
 *   mxfe_tx_data_offload/init_req -> axi_mxfe_tx_dma/m_axis_xfer_req
 *   mxfe_rx_data_offload/init_req -> axi_mxfe_rx_dma/s_axis_xfer_req
 *
 * So it is not passive plumbing that can be left alone -- it decides when the
 * DMA runs at all. In its default (store-and-replay) mode it accumulates a full
 * buffer, streams it out at line rate, then goes quiet while it refills. That
 * is deliberate: it exists so a burst can leave DDR at a rate the memory system
 * could not sustain continuously.
 *
 * For this build (M8 L4 S1 NP16, 8B10B) the buffer is 1 MiB = 65536 beats of
 * 16 bytes = 262 us at 250 MSPS, and that single number was the cause of both
 * long-standing symptoms in this sample:
 *
 *   - TX output present only ~9% of the time: 262 us of drain per ~2.9 ms cycle.
 *     Measured 9% -> 100% duty the moment bypass was enabled. Fixed.
 *   - RX captures truncating at exactly beat 65535 with no error: the RX core is
 *     one-shot by construction (oneshot resets to ~TX_OR_RXN_PATH, and RX has
 *     TX_OR_RXN_PATH=0), so it hands over 1 MiB and stops. NOT fixed, and not
 *     worth fixing -- see the note on axi_data_offload_bypass() below.
 *
 * Neither was a fault. The block was doing exactly what it was built to do, and
 * this port had never told it to do anything else.
 */

/*
 * Put the *TX* offload core into bypass, so it behaves as a plain streaming
 * FIFO: samples pass through continuously with no accumulate/replay cycle.
 *
 * That is what continuous transmit needs. Store-and-replay exists to replay a
 * finite buffer at a rate DDR cannot sustain; a transmitter driving a DAC
 * forever wants the opposite, an uninterrupted stream with the DMA keeping up
 * in real time. Bypass is the correct mode for the TX path here, not a
 * workaround, and it is what moved the measured duty from 9% to 100%.
 *
 * The RX core is deliberately NOT bypassed, and this is load-bearing: doing so
 * breaks Rung 2's ramp capture completely (all odd lanes read 0x0000, even
 * lanes read +/-1 instead of a ramp). Two reasons, both in
 * hdl/library/data_offload/data_offload.v:
 *
 *   - In bypass the storage buffer is replaced by a 16-entry FIFO
 *     (SRC_ADDR_WIDTH_BYPASS = 4), streaming continuously. A capture DMA arming
 *     against that grabs whatever is mid-flight, with no beat alignment.
 *   - m_axis_last is forced to 1'b0 in bypass, so the block framing the capture
 *     path relies on is gone.
 *
 * One-shot store-and-replay is the right mode for capture: the DMA arms, and
 * the core hands over exactly one coherent bufferful from a defined start. The
 * 1 MiB / 65536-beat capture ceiling noted below is the cost of that, and it is
 * a fair trade -- a bounded capture that is correct beats an unbounded one that
 * is misaligned.
 *
 * So the RX truncation and the TX duty had the same root cause but do NOT have
 * the same fix. Only the TX symptom was one worth fixing.
 *
 * Returns 0 if the TX core is in the requested mode, -ENODEV if it is not
 * mapped where expected, or -ENOTSUP if the bitstream was built without bypass
 * support (HAS_BYPASS=0), in which case the bit is tied off.
 */
int axi_data_offload_bypass(bool enable);

/*
 * Log both cores' configuration and latched status: buffer size, direction,
 * reset state, mode bits, FSM state, and the overflow/underflow flags.
 *
 * The two flags are the ones worth having. SRC_OVERFLOW means the producer
 * outran the buffer; DST_UNDERFLOW means the consumer asked for samples the
 * buffer did not have. On the TX side that second one is direct hardware
 * evidence that the DAC ran dry -- which is otherwise only inferable from a
 * duty-cycle measurement.
 *
 * Returns 0 if the cores were read, negative errno if they could not be mapped.
 */
int axi_data_offload_status(void);

/*
 * The TX core's storage size in bytes, read from its own configuration register.
 *
 * Needed to time the DMA honestly in store-and-replay mode. That mode is a
 * fill/drain cycle -- the core accepts one bufferful, then withholds its transfer
 * request while it streams that out at line rate -- so timing a transfer *larger*
 * than the buffer measures the average of fill and drain, not the DMA's own read
 * rate. Staying under this number keeps the whole transfer inside a single fill,
 * which is the only way to see what the DMA can do with the link out of the way.
 *
 * Returns 0 and fills *bytes, or -ENODEV if the core is not where expected.
 */
int axi_data_offload_tx_size(uint64_t *bytes);

#endif /* AXI_DATA_OFFLOAD_H_ */
