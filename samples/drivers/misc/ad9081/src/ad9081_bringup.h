/*
 * Copyright (c) 2026 Analog Devices, Inc.
 *
 * JESD204B bring-up sequence -- coordinates the FPGA cores (adxcvr, jesd204 link,
 * TPL) and the AD9082 chip through the link-up phases.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef AD9081_BRINGUP_H_
#define AD9081_BRINGUP_H_

/*
 * Run the JESD204B bring-up sequence to completion. Assumes every block has
 * already been *configured* (adxcvr, jesd204 link cores, TPL, and the AD9082
 * datapath). Steps all devices through the phases together -- SYNC, clocks
 * enable (GT reset-release + 204C calibrate), link enable, then reads the final
 * link status on both ends. Returns 0 if the link reaches running state,
 * negative errno otherwise.
 *
 * The phase order is table-driven; see the state_ops tables in
 * ad9081_bringup.c and the generic phase walker in jesd204_fsm.c.
 */
int jesd204_bringup(void);

/*
 * Tear the link down: the same phase tables walked in reverse with
 * REASON_UNINIT.
 *
 * Currently that means one thing only -- disabling the chip's JRX deframer. The
 * GT transceivers, the FPGA lane clocks and the HMC7044 clock tree all stay
 * running, because none of the blocks here has a disable entry point yet -- this
 * port has not needed one. So this is a link stop, not a full unwind.
 *
 * That partial unwind is nevertheless enough to re-bring-up from: tearing the
 * link down and calling jesd204_bringup() again reaches DATA the second time
 * (verified on hardware by the fault-injection suite that used to live here,
 * boot_log_fi.golden). Disabling the deframer does drop the link out of DATA, and
 * re-running CLK_SYNC and LINK_ENABLE over the still-running GT, lane clocks and
 * HMC7044 tree renegotiates it. Only this one cycle is tested -- nothing says
 * repeated cycles stay clean, and the blocks left running are why it works, so a
 * fuller unwind would need its own verification.
 *
 * Returns the number of failed steps (0 on a clean walk), or -ENODEV if the chip
 * was never initialised.
 */
int jesd204_teardown(void);

#endif /* AD9081_BRINGUP_H_ */
