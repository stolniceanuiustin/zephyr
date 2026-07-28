/*
 * JESD204 datapath validation tests (post link bring-up).
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef JESD_TEST_H_
#define JESD_TEST_H_

/*
 * Rung 1: receive-path PN link-integrity test. Drives the chip's ADC datapath
 * through a set of PN test patterns and verifies each with the FPGA RX transport
 * core's PN monitor. Proves the receive serial path (chip framer -> GT -> FPGA
 * link -> TPL) is bit-error-free, with no DMA or analog. Must be called only
 * after the JESD204 link has reached DATA. Returns 0 if all patterns pass,
 * negative errno otherwise.
 */
int jesd_test_rx_pn(void);

#endif /* JESD_TEST_H_ */
