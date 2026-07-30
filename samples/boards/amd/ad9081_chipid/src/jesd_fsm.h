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
 *
 * The phase order is table-driven; see the state_ops tables in jesd_fsm.c and
 * the framework in jesd204_fsm.c.
 */
int jesd204_bringup(void);

/*
 * Tear the link down: the same phase tables walked in reverse with
 * REASON_UNINIT, as no-OS jesd204_fsm_stop() does.
 *
 * Currently that means one thing only -- disabling the chip's JRX deframer. The
 * GT transceivers, the FPGA lane clocks and the HMC7044 clock tree all stay
 * running, because none of the blocks here has a disable entry point yet
 * (no-OS has adxcvr_clk_disable() / axi_jesd204_*_lane_clk_disable(); this port
 * has not needed them). So this is a link stop, not a full unwind.
 *
 * Untested: nothing in the sample calls it, and it has never run on hardware.
 * Whether jesd204_bringup() succeeds after it is unverified.
 *
 * Returns the number of failed steps (0 on a clean walk), or -ENODEV if the chip
 * was never initialised.
 */
int jesd204_teardown(void);

#endif /* JESD_FSM_H_ */
