/*
 * JESD204 fault-injection tests -- exercise the failure and recovery paths that
 * a successful bring-up never reaches.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef FAULT_INJECTION_H_
#define FAULT_INJECTION_H_

/*
 * Run the fault-injection suite. Call it after a bring-up that reached DATA:
 * every test starts from a known-good link, breaks one specific thing, and
 * checks that the code reports the break and can rebuild the link afterwards.
 *
 * Returns the number of tests whose observed behaviour did not match what the
 * code claims it does (0 = every path behaved as documented). The return value
 * is about the *tests*, not the hardware: an injected fault being correctly
 * detected and reported is a pass.
 *
 * These tests deliberately take a working link down. The suite restores the link
 * and reports whether it came back, but it is not something to leave enabled in
 * a normal build.
 */
int jesd204_fault_injection_run(void);

#endif /* FAULT_INJECTION_H_ */
