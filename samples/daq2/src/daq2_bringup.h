/*
 * Copyright (c) 2026 Analog Devices, Inc.
 *
 * DAQ2 JESD204B bring-up -- enable path over the reused generic FSM.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef DAQ2_BRINGUP_H_
#define DAQ2_BRINGUP_H_

/**
 * @brief Enable both DAQ2 JESD204 links and re-check the converter PLLs.
 *
 * Assumes the FPGA blocks have already been *configured* (adxcvr GT/PLL and the
 * JESD204 link cores), the same configure-then-enable split the AD9081 sample
 * uses. Both converters (AD9144 TX DAC, AD9680 RX ADC) already enable their own
 * framer/deframer at init(), so this only drives the FPGA-side enable and a chip
 * PLL re-read, in the order the enable path requires: GT reset-release, then the
 * link cores' lane clocks, then (top device, visited last) the chip PLL check.
 *
 * DAQ2 has two independent converters, so each is the top device of its own
 * single-link topology. Two independent FSM walks run, TX then RX; the two links
 * are physically separate so the order between them should not matter.
 *
 * The phase order is table-driven; see the state_ops tables in daq2_bringup.c
 * and the generic phase walker in jesd204_fsm.c.
 *
 * @retval 0 if both links reach running state.
 * @retval -errno if a walk could not be started, or -EIO if either link failed
 *         to come fully up.
 */
int daq2_jesd204_bringup(void);

#endif /* DAQ2_BRINGUP_H_ */
