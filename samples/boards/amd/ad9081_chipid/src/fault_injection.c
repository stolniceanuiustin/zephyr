/*
 * JESD204 fault-injection tests.
 *
 * Every failure path in this sample is unreachable on a board that works. The
 * happy path is verified on hardware; the retry loops, the desync watchdog, the
 * timeout reports and the teardown have never executed. Two bugs have already
 * been found in exactly that code -- an inverted lane-status test that made the
 * watchdog bounce a healthy link, and a TPL warning that was escalated into a
 * link failure -- and both were found by reading, not by running. These tests
 * run it.
 *
 * Five faults, in increasing order of how much they disturb the link:
 *
 *   1. FSM failure accounting. A synthetic topology whose callbacks fail on
 *      purpose, to check the framework counts and attributes failures correctly
 *      and does not abort the walk. Touches no hardware.
 *  1b. Topology rank validation. Malformed topologies -- ranks decreasing, and a
 *      device with no rank -- must be refused with -EINVAL before any callback
 *      runs, while the real board topology still passes. Touches no hardware.
 *   2. Lane desync + watchdog restart. Force the watchdog to read a desynced
 *      LANE_STATUS and confirm it bounces the link, returns -EAGAIN, and that
 *      the link renegotiates to DATA afterwards.
 *   3. Teardown, then bring-up again. jesd204_teardown() has never run. Its
 *      documentation says it stops the chip's deframer and claims nothing about
 *      whether a second bring-up works. Find out.
 *   4. GT reference clock lost. Point the RX transceiver at an undriven PLL so
 *      its PLL genuinely cannot lock, then run the real enable path and check it
 *      reports -ETIMEDOUT rather than hanging. Restores and rebuilds the link.
 *
 * What a "pass" means here: the injected fault was detected and reported the way
 * the code's own documentation says it would. A test failing means the code did
 * something other than what it claims -- which is the point.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(fault_injection, LOG_LEVEL_INF);

#include "axi_adxcvr.h"
#include "axi_jesd204.h"
#include "fault_injection.h"
#include "jesd204_fsm.h"
#include "jesd_fsm.h"

#define JESD204_STATE_CHANGE_DONE 1

/* Test bookkeeping: one line per test, and a tally of mismatches. */
static int fi_failures;

static void fi_pass(const char *name, const char *detail)
{
	LOG_INF("FI PASS  %-24s %s", name, detail);
}

static void fi_fail(const char *name, const char *detail)
{
	LOG_ERR("FI FAIL  %-24s %s", name, detail);
	fi_failures++;
}

/*
 * Restore the link to DATA after a test has disturbed it, and say whether it
 * came back. Used as both a cleanup step and an assertion: a recovery path that
 * reports success but leaves the link down has not recovered.
 */
static bool fi_link_recovered(const char *name)
{
	/*
	 * A full second, deliberately longer than the FSM's 80 ms poll. That
	 * budget is sized for a link whose ends were just enabled in order; here
	 * the link is re-running CGS/ILAS after being interrupted mid-DATA, and
	 * timing out early would report a recovery failure that was really just
	 * an impatient test.
	 */
	for (int attempt = 0; attempt < 250; attempt++) {
		k_msleep(4);
		if (axi_jesd204_link_is_data()) {
			return true;
		}
	}

	fi_fail(name, "link did not return to DATA within 1 s");
	return false;
}

/* ------------------------------------------- 1. FSM failure accounting --- */

/*
 * A synthetic device whose callbacks fail on demand. This exercises the
 * framework (jesd204_fsm.c) rather than the board: it checks that a negative
 * return is counted, attributed to the right phase, does not abort the walk, and
 * reaches jesd204_bringup()'s -EIO. Doing it on a fake topology means the real
 * device tables stay untouched and the link stays up.
 */
static int fi_cb_fail(struct jesd204_dev *jdev,
		      enum jesd204_state_op_reason reason)
{
	ARG_UNUSED(jdev);

	if (reason != JESD204_STATE_OP_REASON_INIT) {
		return JESD204_STATE_CHANGE_DONE;
	}
	return -EIO;
}

static int fi_cb_fail_link(struct jesd204_dev *jdev,
			   enum jesd204_state_op_reason reason,
			   struct jesd204_link *lnk)
{
	ARG_UNUSED(jdev);
	ARG_UNUSED(lnk);

	if (reason != JESD204_STATE_OP_REASON_INIT) {
		return JESD204_STATE_CHANGE_DONE;
	}
	return -EIO;
}

static int fi_cb_ok(struct jesd204_dev *jdev,
		    enum jesd204_state_op_reason reason)
{
	ARG_UNUSED(jdev);
	ARG_UNUSED(reason);
	return JESD204_STATE_CHANGE_DONE;
}

/* Records that a phase after the failing one still ran. */
static bool fi_late_phase_ran;

static int fi_cb_late(struct jesd204_dev *jdev,
		      enum jesd204_state_op_reason reason,
		      struct jesd204_link *lnk)
{
	ARG_UNUSED(jdev);
	ARG_UNUSED(lnk);

	if (reason == JESD204_STATE_OP_REASON_INIT) {
		fi_late_phase_ran = true;
	}
	return JESD204_STATE_CHANGE_DONE;
}

static const struct jesd204_dev_data fi_bad_data = {
	.state_ops = {
		/* Fails in an early phase... */
		[JESD204_OP_LINK_INIT] = {
			.per_device = fi_cb_fail,
		},
		/* ...and both callbacks of a later one fail, to check the count. */
		[JESD204_OP_LINK_SETUP] = {
			.per_device = fi_cb_fail,
			.per_link = fi_cb_fail_link,
		},
		/* This must still run despite the failures above. */
		[JESD204_OP_LINK_RUNNING] = {
			.per_link = fi_cb_late,
		},
	},
};

static const struct jesd204_dev_data fi_good_data = {
	.state_ops = {
		[JESD204_OP_LINK_INIT] = {
			.per_device = fi_cb_ok,
		},
	},
};

/*
 * Both synthetic devices share JESD204_RANK_TEST. Equal ranks are legal and mean
 * the order between them is not load-bearing, which is true here: neither touches
 * hardware, so what this topology exercises is the walker's failure accounting,
 * not a dependency.
 */
static struct jesd204_dev fi_bad_dev = {
	.name = "fi-bad",
	.rank = JESD204_RANK_TEST,
	.dev_data = &fi_bad_data,
};

static struct jesd204_dev fi_good_dev = {
	.name = "fi-good",
	.rank = JESD204_RANK_TEST,
	.dev_data = &fi_good_data,
};

static struct jesd204_topology fi_topology = {
	.devs = { &fi_good_dev, &fi_bad_dev },
	.devs_number = 2,
	.link = { .link_id = 99, .is_transmit = false },
};

static void fi_test_fsm_accounting(void)
{
	const char *name = "fsm-accounting";
	int failures;

	LOG_INF("--- FI 1: FSM failure accounting (synthetic topology) ---");

	fi_late_phase_ran = false;
	failures = jesd204_fsm_start(&fi_topology);

	/*
	 * Three failing callbacks: LINK_INIT per-device, LINK_SETUP per-device,
	 * LINK_SETUP per-link. The good device contributes none.
	 */
	if (failures != 3) {
		LOG_ERR("expected 3 failures, got %d", failures);
		fi_fail(name, "wrong failure count");
	} else if (!fi_late_phase_ran) {
		fi_fail(name, "walk aborted early -- LINK_RUNNING never ran");
	} else {
		fi_pass(name, "3 failures counted, walk continued to the end");
	}

	/*
	 * Teardown of the same topology: UNINIT returns DONE everywhere, so a
	 * clean reverse walk is the expected result. This is the only check that
	 * the reverse walk runs at all.
	 */
	failures = jesd204_fsm_stop(&fi_topology);
	if (failures != 0) {
		LOG_ERR("reverse walk reported %d failure(s)", failures);
		fi_fail("fsm-reverse-walk", "UNINIT should have been clean");
	} else {
		fi_pass("fsm-reverse-walk", "reverse walk clean");
	}
}

/*
 * The rank check is itself a failure path, so it gets injected like any other.
 * Two malformed topologies, both of which used to be silently walkable:
 *
 *   - ranks decreasing, which is the bug that actually happened once (the chip
 *     listed ahead of the GT, moving its JESD PLL check before the reset-release)
 *   - a device with no .rank at all, the omission JESD204_RANK_UNSET exists for
 *
 * Both must be refused with -EINVAL before any callback runs. That "before" is
 * the part worth testing: a walk that started and then bailed would leave
 * hardware half-configured, which is exactly what refusing up front avoids. It is
 * checked via fi_late_phase_ran, so these devices register fi_cb_late rather than
 * fi_good_data's fi_cb_ok -- fi_cb_ok sets no flag, which would make the
 * assertion vacuously true whether the walk ran or not.
 */
static const struct jesd204_dev_data fi_ranked_data = {
	.state_ops = {
		[JESD204_OP_LINK_INIT] = {
			.per_link = fi_cb_late,
		},
	},
};

static struct jesd204_dev fi_ranked_low = {
	.name = "fi-rank-low",
	.rank = JESD204_RANK_PHY,
	.dev_data = &fi_ranked_data,
};

static struct jesd204_dev fi_ranked_high = {
	.name = "fi-rank-high",
	.rank = JESD204_RANK_LINK,
	.dev_data = &fi_ranked_data,
};

/* .rank deliberately omitted -- defaults to JESD204_RANK_UNSET (0). */
static struct jesd204_dev fi_unranked = {
	.name = "fi-rank-unset",
	.dev_data = &fi_ranked_data,
};

static struct jesd204_topology fi_misordered_topology = {
	/* LINK before PHY: rank decreases across the array. */
	.devs = { &fi_ranked_high, &fi_ranked_low },
	.devs_number = 2,
	.link = { .link_id = 98, .is_transmit = false },
};

static struct jesd204_topology fi_unranked_topology = {
	.devs = { &fi_ranked_low, &fi_unranked },
	.devs_number = 2,
	.link = { .link_id = 97, .is_transmit = false },
};

static void fi_test_topology_ranks(void)
{
	int ret;

	LOG_INF("--- FI 1b: topology rank validation ---");

	fi_late_phase_ran = false;
	ret = jesd204_fsm_start(&fi_misordered_topology);
	if (ret != -EINVAL) {
		LOG_ERR("misordered topology returned %d, expected -EINVAL", ret);
		fi_fail("rank-misordered", "misordering was not refused");
	} else if (fi_late_phase_ran) {
		fi_fail("rank-misordered", "refused, but only after walking");
	} else {
		fi_pass("rank-misordered", "-EINVAL before any callback ran");
	}

	fi_late_phase_ran = false;
	ret = jesd204_fsm_start(&fi_unranked_topology);
	if (ret != -EINVAL) {
		LOG_ERR("unranked device returned %d, expected -EINVAL", ret);
		fi_fail("rank-unset", "missing rank was not refused");
	} else if (fi_late_phase_ran) {
		fi_fail("rank-unset", "refused, but only after walking");
	} else {
		fi_pass("rank-unset", "-EINVAL before any callback ran");
	}

	/*
	 * The reverse walk validates too, so teardown of a misordered topology is
	 * refused the same way. Checked because an unwind in the wrong order is as
	 * damaging as a bring-up in the wrong order.
	 */
	ret = jesd204_fsm_stop(&fi_misordered_topology);
	if (ret != -EINVAL) {
		LOG_ERR("misordered teardown returned %d, expected -EINVAL", ret);
		fi_fail("rank-misordered-stop", "teardown did not validate ranks");
	} else {
		fi_pass("rank-misordered-stop", "teardown refused with -EINVAL");
	}

	/* The real topology must of course still pass. */
	ret = jesd204_bringup_topology_is_valid();
	if (ret < 0) {
		LOG_ERR("real topology rejected (%d)", ret);
		fi_fail("rank-real-topology", "the working topology failed the check");
	} else {
		fi_pass("rank-real-topology", "board topology ranks ascend");
	}
}

/* ------------------------------------------------ 2. Lane desync watchdog --- */

static void fi_test_lane_desync(void)
{
	const char *name = "lane-desync";
	uint32_t lanes = axi_jesd204_fi_num_lanes();
	int ret;

	LOG_INF("--- FI 2: RX lane desync -> watchdog restart ---");

	/* What the hardware really reports, for the record. */
	for (uint32_t lane = 0; lane < lanes; lane++) {
		LOG_INF("lane %u actual status = 0x%08x", lane,
			axi_jesd204_fi_lane_status(lane));
	}

	/*
	 * A healthy 8B/10B lane has non-zero low two bits (this board reads
	 * 0x32). 0x30 keeps the upper bits plausible while clearing the state
	 * field -- what a lane that lost alignment would look like.
	 */
	axi_jesd204_fi_force_lane_status(0x30);
	ret = axi_jesd204_rx_watchdog();
	axi_jesd204_fi_clear_lane_status();

	if (ret != -EAGAIN) {
		LOG_ERR("watchdog returned %d, expected -EAGAIN", ret);
		fi_fail(name, "desync not detected");
	} else {
		fi_pass(name, "watchdog detected desync and bounced the link");
	}

	/* The bounce is a real LINK_DISABLE cycle, so the link must renegotiate. */
	if (!fi_link_recovered("lane-desync-recovery")) {
		/*
		 * Skip the false-positive check rather than run it against a link
		 * that is down: the watchdog returns 0 early when the link is not
		 * in DATA, so it would report a pass without testing anything.
		 */
		LOG_WRN("skipping the false-positive check -- link is not in DATA");
		return;
	}
	fi_pass("lane-desync-recovery", "link renegotiated to DATA");

	/*
	 * And with nothing forced, the watchdog must be quiet. This is the
	 * regression test for the inverted-polarity bug: that version reported
	 * every healthy lane as desynced and bounced a working link.
	 */
	ret = axi_jesd204_rx_watchdog();
	if (ret != 0) {
		LOG_ERR("watchdog returned %d on a healthy link", ret);
		fi_fail("watchdog-no-false-positive", "spurious restart");
	} else {
		fi_pass("watchdog-no-false-positive", "healthy link left alone");
	}
}

/* --------------------------------------------- 3. Teardown and re-bring-up --- */

static void fi_test_teardown_rebringup(void)
{
	const char *name = "teardown";
	int ret;

	LOG_INF("--- FI 3: teardown, then bring-up again ---");

	ret = jesd204_teardown();
	if (ret != 0) {
		LOG_ERR("teardown reported %d failed step(s)", ret);
		fi_fail(name, "teardown walk had failures");
	} else {
		fi_pass(name, "teardown walk clean");
	}

	/*
	 * Teardown disables the chip's JRX deframer and nothing else, so the link
	 * should drop out of DATA. If it does not, the teardown did not actually
	 * do anything -- worth knowing either way, since the header only claims
	 * it stops the deframer.
	 */
	k_msleep(20);
	if (axi_jesd204_link_is_data()) {
		fi_fail("teardown-effect",
			"link still in DATA -- teardown had no effect");
	} else {
		fi_pass("teardown-effect", "link left DATA after teardown");
	}

	/*
	 * The open question: is the link re-bringable? jesd_fsm.h says this is
	 * unverified. The GT, the FPGA lane clocks and the HMC7044 tree were all
	 * left running, so bring-up is re-running CLK_SYNC and LINK_ENABLE over
	 * live clocks rather than starting cold.
	 */
	ret = jesd204_bringup();
	if (ret != 0) {
		LOG_ERR("second bring-up returned %d", ret);
		fi_fail("teardown-rebringup", "link did not come back up");
	} else {
		fi_pass("teardown-rebringup", "second bring-up reached DATA");
	}
}

/* -------------------------------------------------- 4. GT refclk lost --- */

static void fi_test_gt_refclk_lost(void)
{
	const char *name = "gt-refclk-lost";
	int ret;

	LOG_INF("--- FI 4: RX GT reference clock lost ---");

	axi_adxcvr_fi_rx_break_refclk();

	/*
	 * The real enable path, with a GT that genuinely cannot lock. This takes
	 * ~200 ms: adxcvr_reset() makes two attempts, each polling RESET_DONE for
	 * 100 ms. Reaching the -ETIMEDOUT below means the retry loop terminated
	 * instead of hanging, which is the thing being tested.
	 */
	ret = axi_adxcvr_rx_enable();
	LOG_INF("rx_adxcvr STATUS after failed lock = 0x%08x",
		axi_adxcvr_fi_rx_status());

	if (ret != -ETIMEDOUT) {
		LOG_ERR("rx_enable returned %d, expected -ETIMEDOUT", ret);
		fi_fail(name, "unlocked GT was not reported as a timeout");
	} else {
		fi_pass(name, "-ETIMEDOUT after 2 bounded reset attempts");
	}

	/* Put the reference clock back and rebuild the link from the GT up. */
	ret = axi_adxcvr_fi_rx_restore_refclk();
	if (ret) {
		LOG_ERR("restoring the RX GT failed (%d) -- link is now down",
			ret);
		fi_fail("gt-refclk-restore", "could not restore the GT");
		return;
	}

	ret = jesd204_bringup();
	if (ret != 0) {
		LOG_ERR("bring-up after GT restore returned %d", ret);
		fi_fail("gt-refclk-restore", "link did not come back up");
	} else {
		fi_pass("gt-refclk-restore", "link rebuilt after refclk restored");
	}
}

/* ------------------------------------------------------------- driver --- */

int jesd204_fault_injection_run(void)
{
	fi_failures = 0;

	LOG_INF("=== JESD204 fault injection: 5 faults ===");

	/*
	 * Order matters: least disruptive first. Tests 1 and 1b touch no hardware,
	 * test 2 bounces the link core, test 3 stops the chip's deframer, test 4
	 * takes the GT down. Each restores the link before the next, so a failure
	 * that leaves the link down is attributable to the test that caused it
	 * rather than to whatever ran before.
	 */
	fi_test_fsm_accounting();
	fi_test_topology_ranks();
	fi_test_lane_desync();
	fi_test_teardown_rebringup();
	fi_test_gt_refclk_lost();

	if (fi_failures) {
		LOG_ERR("=== fault injection: %d check(s) did not behave as documented ===",
			fi_failures);
	} else {
		LOG_INF("=== fault injection: all checks behaved as documented ===");
	}

	return fi_failures;
}
