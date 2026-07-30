/*
 * JESD204 bring-up framework -- the state machine walk.
 *
 * Port of no-OS jesd204/jesd204-fsm.c. The whole framework is two loops: for
 * each phase, for each device, call whatever that device registered for that
 * phase. Everything that makes a link come up in the right order is expressed
 * in the tables, not here.
 *
 * no-OS's version carries a third loop over links and a fourth matching link
 * IDs per device (jesd204-fsm.c:25-28). This board has one link, so both
 * collapse and the per-link callback is invoked once per device per phase.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(jesd204_fsm, LOG_LEVEL_INF);

#include "jesd204_fsm.h"

/* Phase names, indexed by enum jesd204_dev_op, for the per-phase log line. */
static const char *const op_names[__JESD204_MAX_OPS] = {
	[JESD204_OP_DEVICE_INIT] = "device_init",
	[JESD204_OP_LINK_INIT] = "link_init",
	[JESD204_OP_LINK_SUPPORTED] = "link_supported",
	[JESD204_OP_LINK_PRE_SETUP] = "link_pre_setup",
	[JESD204_OP_CLK_SYNC_STAGE1] = "clk_sync_stage1",
	[JESD204_OP_CLK_SYNC_STAGE2] = "clk_sync_stage2",
	[JESD204_OP_CLK_SYNC_STAGE3] = "clk_sync_stage3",
	[JESD204_OP_LINK_SETUP] = "link_setup",
	[JESD204_OP_OPT_SETUP_STAGE1] = "opt_setup_stage1",
	[JESD204_OP_OPT_SETUP_STAGE2] = "opt_setup_stage2",
	[JESD204_OP_OPT_SETUP_STAGE3] = "opt_setup_stage3",
	[JESD204_OP_OPT_SETUP_STAGE4] = "opt_setup_stage4",
	[JESD204_OP_OPT_SETUP_STAGE5] = "opt_setup_stage5",
	[JESD204_OP_CLOCKS_ENABLE] = "clocks_enable",
	[JESD204_OP_LINK_ENABLE] = "link_enable",
	[JESD204_OP_LINK_RUNNING] = "link_running",
	[JESD204_OP_OPT_POST_RUNNING_STAGE] = "opt_post_running",
};

int jesd204_sysref_async(struct jesd204_topology *topology)
{
	for (unsigned int i = 0; i < topology->devs_number; i++) {
		struct jesd204_dev *jdev = topology->devs[i];

		if (jdev->dev_data->sysref_cb) {
			return jdev->dev_data->sysref_cb(jdev);
		}
	}

	/*
	 * No device claims SYSREF ownership. On this board that is expected: the
	 * HMC7044 emits DEV_SYSREF/FPGA_SYSREF continuously, so subclass-1
	 * alignment happens against the free-running pulse train rather than an
	 * on-demand one. Not an error, but worth saying once.
	 */
	LOG_DBG("no SYSREF callback in topology (continuous SYSREF assumed)");
	return 0;
}

/*
 * Run one device's ops for one phase. Returns the number of failures (0, 1 or
 * 2 -- a phase may populate both a per_device and a per_link callback).
 *
 * A callback returning JESD204_STATE_CHANGE_DONE (1) or DEFER (0) is a success;
 * no-OS uses those as its "state advanced" signal and discards them entirely
 * (jesd204-fsm.c:31-43). Negative is a failure. That means a callback here can
 * report a real error where in no-OS it would vanish -- the FSM still does not
 * abort on it, but it is counted and logged rather than lost.
 */
static int run_dev_op(struct jesd204_dev *jdev, enum jesd204_dev_op op,
		      enum jesd204_state_op_reason reason,
		      struct jesd204_link *lnk)
{
	const struct jesd204_state_op *sop = &jdev->dev_data->state_ops[op];
	int failures = 0;
	int ret;

	if (sop->per_device) {
		ret = sop->per_device(jdev, reason);
		if (ret < 0) {
			LOG_WRN("%s: %s (per-device) failed (%d)", jdev->name,
				op_names[op], ret);
			failures++;
		}
	}

	if (sop->per_link) {
		ret = sop->per_link(jdev, reason, lnk);
		if (ret < 0) {
			LOG_WRN("%s: %s (per-link) failed (%d)", jdev->name,
				op_names[op], ret);
			failures++;
		}
	}

	return failures;
}

/* True if any device has work registered for this phase. */
static bool op_is_populated(struct jesd204_topology *topology,
			    enum jesd204_dev_op op)
{
	for (unsigned int i = 0; i < topology->devs_number; i++) {
		const struct jesd204_state_op *sop =
			&topology->devs[i]->dev_data->state_ops[op];

		if (sop->per_device || sop->per_link) {
			return true;
		}
	}
	return false;
}

/* True if any device asked for a SYSREF after this phase. */
static bool op_wants_sysref(struct jesd204_topology *topology,
			    enum jesd204_dev_op op)
{
	for (unsigned int i = 0; i < topology->devs_number; i++) {
		if (topology->devs[i]->dev_data->state_ops[op].post_state_sysref) {
			return true;
		}
	}
	return false;
}

int jesd204_fsm_start(struct jesd204_topology *topology)
{
	int total_failures = 0;

	LOG_INF("--- JESD204 FSM: bring-up (%u devices, link %u) ---",
		topology->devs_number, topology->link.link_id);

	for (enum jesd204_dev_op op = 0; op < __JESD204_MAX_OPS; op++) {
		int failures = 0;

		if (!op_is_populated(topology, op)) {
			continue;
		}

		for (unsigned int i = 0; i < topology->devs_number; i++) {
			failures += run_dev_op(topology->devs[i], op,
					       JESD204_STATE_OP_REASON_INIT,
					       &topology->link);
		}

		if (op_wants_sysref(topology, op)) {
			jesd204_sysref_async(topology);
		}

		if (failures) {
			LOG_WRN("phase %-16s : %d failure(s)", op_names[op],
				failures);
		} else {
			LOG_INF("phase %-16s : ok", op_names[op]);
		}
		total_failures += failures;
	}

	return total_failures;
}

int jesd204_fsm_stop(struct jesd204_topology *topology)
{
	int total_failures = 0;

	LOG_INF("--- JESD204 FSM: teardown ---");

	for (int op = __JESD204_MAX_OPS - 1; op >= 0; op--) {
		int failures = 0;

		if (!op_is_populated(topology, op)) {
			continue;
		}

		/* Reverse device order, as no-OS does (jesd204-fsm.c:81-95). */
		for (int i = topology->devs_number - 1; i >= 0; i--) {
			failures += run_dev_op(topology->devs[i], op,
					       JESD204_STATE_OP_REASON_UNINIT,
					       &topology->link);
		}

		if (failures) {
			LOG_WRN("phase %-16s : %d failure(s) (uninit)",
				op_names[op], failures);
		}
		total_failures += failures;
	}

	return total_failures;
}
