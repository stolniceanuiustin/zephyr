/*
 * JESD204 datapath validation -- Rung 2: DMA capture into DDR.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef JESD_CAPTURE_H_
#define JESD_CAPTURE_H_

#include <stddef.h>
#include <stdint.h>

/*
 * Rung 2: put the chip's ADC datapath into RAMP test mode, capture a block via
 * the RX AXI DMAC into a DDR buffer, and verify live samples arrived. Must be
 * called only after the JESD204 link has reached DATA. Returns 0 on a good
 * capture, negative errno otherwise. Restores normal ADC mode before returning.
 */
int jesd_capture_ramp(void);

/*
 * Inspect a captured sample block: dump the head, and flag an all-zero (DMA
 * never wrote / stale cache) or constant (dead datapath) buffer. Exposed for
 * unit-style reuse. Returns 0 if the block looks like live ramp data.
 */
int jesd_capture_analyze(const uint16_t *buf, size_t n);

/*
 * Capture a block of live samples through the RX AXI DMAC into the shared DDR
 * buffer, with no test mode involved -- whatever the ADC is actually digitising.
 * Handles the cache maintenance and polls the (IRQ-less) transfer to completion.
 *
 * On success returns 0 and points *buf at the captured samples, storing the count
 * in *n. The buffer stays valid until the next capture. Used by Rung 5's analog
 * loopback; Rung 2 drives the same machinery with the ramp test mode enabled.
 */
int jesd_capture_raw(const int16_t **buf, size_t *n);

/* Beat width of the capture stream: 16-byte DMA bus / 2-byte sample. */
#define JESD_CAP_LANES_PER_BEAT 8U

#endif /* JESD_CAPTURE_H_ */
