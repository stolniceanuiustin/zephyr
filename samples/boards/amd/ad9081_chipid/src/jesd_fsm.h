/*
 * JESD204B bring-up sequence -- coordinates the FPGA cores (adxcvr, jesd204 link,
 * TPL) and the AD9082 chip through the link-up phases.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef JESD_FSM_H_
#define JESD_FSM_H_

/*
 * Run the JESD204B bring-up sequence to completion. Assumes every block has
 * already been *configured* (adxcvr, jesd204 link cores, TPL, and the AD9082
 * datapath). Steps all devices through the phases together -- SYNC, clocks
 * enable (GT reset-release + 204C calibrate), link enable, then reads the final
 * link status on both ends. Returns 0 if the link reaches running state,
 * negative errno otherwise.
 */
int jesd204_bringup(void);

#endif /* JESD_FSM_H_ */
