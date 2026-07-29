/*
 * ADI AXI data-offload cores -- the store-and-replay buffers between each AXI
 * DMAC and its JESD204 transport core.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef AXI_DATA_OFFLOAD_H_
#define AXI_DATA_OFFLOAD_H_

#include <stdbool.h>

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
 *     Measured 9% -> 100% duty the moment bypass was enabled.
 *   - RX captures truncating at exactly beat 65535 with no error: the RX core is
 *     one-shot by construction (oneshot resets to ~TX_OR_RXN_PATH, and RX has
 *     TX_OR_RXN_PATH=0), so it hands over 1 MiB and stops.
 *
 * Neither was a fault. The block was doing exactly what it was built to do, and
 * this port had never told it to do anything else.
 */

/*
 * Put both offload cores into bypass, so each behaves as a plain streaming FIFO:
 * samples pass through continuously with no accumulate/replay cycle.
 *
 * This is what a transceiver needs. The store-and-replay mode is for replaying a
 * finite buffer at a rate DDR cannot sustain; continuous transmit and receive
 * want the opposite -- an uninterrupted stream, with the DMA keeping up in real
 * time. Bypass is therefore the correct mode for this sample, not a workaround.
 *
 * Returns 0 if both cores are in bypass, -ENODEV if no core is mapped where
 * expected, or -ENOTSUP if the bitstream was built without bypass support
 * (HAS_BYPASS=0), in which case the bit is tied off and cannot be set.
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

#endif /* AXI_DATA_OFFLOAD_H_ */
