/*
 * JESD204 bring-up framework -- topology construction and the state machine
 * walk.
 *
 * Port of no-OS jesd204/jesd204-core.c (jesd204_topology_init) and
 * jesd204/jesd204-fsm.c (jesd204_fsm_start / jesd204_fsm_stop). The walk is four
 * nested loops -- phase, link, device, and the device's own link list -- and
 * everything that makes a link come up in the right order is expressed in the
 * device tables and in the topology, not here.
 *
 * The visit order within a phase is the part worth being careful about, because
 * it is the part hardware notices. Forward (jesd204-fsm.c:21-53):
 *
 *     for each link:
 *         for each non-top device that serves this link, in array order:
 *             per_device (once per phase only), then per_link
 *         top device's per_link
 *     top device's per_device
 *
 * Reverse (jesd204-fsm.c:71-97) mirrors it: the top device's per_device first,
 * then per link in reverse, the top device's per_link before the non-top devices
 * in reverse array order.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <zephyr/kernel.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(jesd204_fsm, LOG_LEVEL_INF);

#include <zephyr/jesd204/jesd204.h>

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

/* ------------------------------------------------------------ topology --- */

/* True if `id` appears in the top device's declared link set. */
static bool top_declares_link(const struct jesd204_dev_top *top,
			      unsigned int id)
{
	for (unsigned int i = 0; i < top->num_links; i++) {
		if (top->link_ids[i] == id) {
			return true;
		}
	}
	return false;
}

/*
 * Shape checks on one row, applied to top and non-top rows alike. See the
 * jesd204_topology_init() comment in the header for why these exist when no-OS
 * has none of them.
 */
static int check_row(const struct jesd204_topology_dev *row, unsigned int idx)
{
	if (row->jdev == NULL || row->jdev->dev_data == NULL) {
		LOG_ERR("topology row %u has no device or no phase table", idx);
		return -EINVAL;
	}

	if (row->links_number == 0) {
		LOG_ERR("%s: topology row %u serves no link", row->jdev->name,
			idx);
		return -EINVAL;
	}

	if (row->links_number > JESD204_MAX_TOPOLOGY_LINKS) {
		LOG_ERR("%s: %u links (max %d)", row->jdev->name,
			row->links_number, JESD204_MAX_TOPOLOGY_LINKS);
		return -EINVAL;
	}

	/*
	 * max_num_links is what the driver says it can serve. no-OS carries the
	 * field and never checks it; a device handed more links than it declares
	 * would run its per-link callback for a link it has no state for.
	 */
	if (row->jdev->dev_data->max_num_links &&
	    row->links_number > row->jdev->dev_data->max_num_links) {
		LOG_ERR("%s: given %u links, driver declares max %u",
			row->jdev->name, row->links_number,
			row->jdev->dev_data->max_num_links);
		return -EINVAL;
	}

	for (unsigned int i = 0; i < row->links_number; i++) {
		for (unsigned int j = i + 1; j < row->links_number; j++) {
			if (row->link_ids[i] == row->link_ids[j]) {
				LOG_ERR("%s: link id %u listed twice",
					row->jdev->name, row->link_ids[i]);
				return -EINVAL;
			}
		}
	}

	if (row->is_sysref_provider &&
	    row->jdev->dev_data->sysref_cb == NULL) {
		LOG_ERR("%s: marked SYSREF provider but registers no sysref_cb",
			row->jdev->name);
		return -EINVAL;
	}

	return 0;
}

int jesd204_topology_init(struct jesd204_topology *topology,
			  const struct jesd204_topology_dev *devs,
			  unsigned int devs_number)
{
	const struct jesd204_topology_dev *top_row = NULL;
	unsigned int sysref_providers = 0;
	unsigned int d = 0;
	int ret;

	if (topology == NULL || devs == NULL || devs_number == 0) {
		return -EINVAL;
	}

	memset(topology, 0, sizeof(*topology));

	/*
	 * First pass: find the top device and validate every row. The top row has
	 * to be known before the non-top rows can be checked against its link
	 * set, and it may appear anywhere in the array (no-OS's ad9081 project
	 * puts it last, app.c:434-439).
	 */
	for (unsigned int i = 0; i < devs_number; i++) {
		ret = check_row(&devs[i], i);
		if (ret) {
			return ret;
		}

		if (devs[i].is_sysref_provider) {
			sysref_providers++;
		}

		if (!devs[i].is_top_device) {
			continue;
		}

		if (top_row != NULL) {
			LOG_ERR("topology has more than one top device (%s and %s)",
				top_row->jdev->name, devs[i].jdev->name);
			return -EINVAL;
		}
		top_row = &devs[i];
	}

	if (top_row == NULL) {
		LOG_ERR("topology has no is_top_device row");
		return -EINVAL;
	}

	if (sysref_providers > 1) {
		LOG_ERR("topology has %u SYSREF providers (max 1)",
			sysref_providers);
		return -EINVAL;
	}

	if (devs_number - 1 > JESD204_MAX_DEVS) {
		LOG_ERR("topology has %u non-top devices (max %d)",
			devs_number - 1, JESD204_MAX_DEVS);
		return -EINVAL;
	}

	/*
	 * The top device declares the topology's links (jesd204-core.c:104-109).
	 * active_links[] is what per-link callbacks are handed, so the direction
	 * is stamped in here, once, from the row that owns the link set.
	 */
	topology->dev_top.jdev = top_row->jdev;
	topology->dev_top.jdev->is_top = true;
	topology->dev_top.num_links = top_row->links_number;

	for (unsigned int l = 0; l < top_row->links_number; l++) {
		topology->dev_top.link_ids[l] = top_row->link_ids[l];
		topology->dev_top.active_links[l].link_id = top_row->link_ids[l];
		topology->dev_top.active_links[l].is_transmit =
			top_row->is_transmit[l];
	}

	/* Second pass: copy the non-top rows, order preserved. */
	for (unsigned int i = 0; i < devs_number; i++) {
		if (devs[i].is_top_device) {
			continue;
		}

		for (unsigned int l = 0; l < devs[i].links_number; l++) {
			if (!top_declares_link(&topology->dev_top,
					       devs[i].link_ids[l])) {
				LOG_ERR("%s: link id %u is not declared by top device %s",
					devs[i].jdev->name, devs[i].link_ids[l],
					top_row->jdev->name);
				return -EINVAL;
			}
		}

		/*
		 * post_state_sysref is only honoured on the top device, as in
		 * no-OS (jesd204-fsm.c:44-52 tests jdev_top's ops and no
		 * others). Silently ignoring it on a non-top device would leave
		 * a board author waiting for a SYSREF that never comes.
		 */
		for (enum jesd204_dev_op op = 0; op < __JESD204_MAX_OPS; op++) {
			if (devs[i].jdev->dev_data->state_ops[op].post_state_sysref) {
				LOG_ERR("%s: post_state_sysref on a non-top device (phase %s) is never honoured",
					devs[i].jdev->name, op_names[op]);
				return -EINVAL;
			}
		}

		topology->devs[d++] = devs[i];
	}
	topology->devs_number = d;

	/*
	 * Resolved after the copy, so the provider may be any row -- including
	 * the top device, which is not in devs[].
	 */
	for (unsigned int i = 0; i < devs_number; i++) {
		if (devs[i].is_sysref_provider) {
			topology->dev_top.jdev_sysref = devs[i].jdev;
		}
	}

	topology->initialised = true;
	return 0;
}

int jesd204_sysref_async(struct jesd204_topology *topology)
{
	const struct jesd204_dev_data *dev_data;

	if (topology == NULL || !topology->initialised) {
		return -EINVAL;
	}

	/*
	 * No provider is not an error (no-OS jesd204-core.c:358-360). On this
	 * board it is the expected case: the HMC7044 emits DEV_SYSREF/FPGA_SYSREF
	 * continuously, so subclass-1 alignment happens against the free-running
	 * pulse train rather than an on-demand one.
	 */
	if (topology->dev_top.jdev_sysref == NULL) {
		LOG_DBG("no SYSREF provider in topology (continuous SYSREF assumed)");
		return 0;
	}

	dev_data = topology->dev_top.jdev_sysref->dev_data;
	return dev_data->sysref_cb(topology->dev_top.jdev_sysref);
}

/* ---------------------------------------------------------------- walk --- */

/*
 * Run one callback and count it. A callback returning JESD204_STATE_CHANGE_DONE
 * (1) or DEFER (0) is a success; no-OS uses those as its "state advanced" signal
 * and discards them entirely (jesd204-fsm.c:31-43). Negative is a failure. That
 * means a callback here can report a real error where in no-OS it would vanish
 * -- the FSM still does not abort on it, but it is counted and logged rather
 * than lost.
 */
static int run_dev_cb(struct jesd204_dev *jdev, enum jesd204_dev_op op,
		      enum jesd204_state_op_reason reason)
{
	int ret = jdev->dev_data->state_ops[op].per_device(jdev, reason);

	if (ret < 0) {
		LOG_WRN("%s: %s (per-device) failed (%d)", jdev->name,
			op_names[op], ret);
		return 1;
	}
	return 0;
}

static int run_link_cb(struct jesd204_dev *jdev, enum jesd204_dev_op op,
		       enum jesd204_state_op_reason reason,
		       struct jesd204_link *lnk)
{
	int ret = jdev->dev_data->state_ops[op].per_link(jdev, reason, lnk);

	if (ret < 0) {
		LOG_WRN("%s: %s (per-link, link %u) failed (%d)", jdev->name,
			op_names[op], lnk->link_id, ret);
		return 1;
	}
	return 0;
}

/* True if this device serves `link_id`. */
static bool dev_serves_link(const struct jesd204_topology_dev *row,
			    unsigned int link_id)
{
	for (unsigned int i = 0; i < row->links_number; i++) {
		if (row->link_ids[i] == link_id) {
			return true;
		}
	}
	return false;
}

/* True if any device in the topology has work registered for this phase. */
static bool op_is_populated(const struct jesd204_topology *topology,
			    enum jesd204_dev_op op)
{
	const struct jesd204_state_op *sop =
		&topology->dev_top.jdev->dev_data->state_ops[op];

	if (sop->per_device || sop->per_link) {
		return true;
	}

	for (unsigned int i = 0; i < topology->devs_number; i++) {
		sop = &topology->devs[i].jdev->dev_data->state_ops[op];
		if (sop->per_device || sop->per_link) {
			return true;
		}
	}
	return false;
}

/*
 * Resolve the link_idx argument into a half-open range over the top device's
 * link_ids[]. JESD204_LINKS_ALL means every link. Returns -EINVAL for an index
 * past the end.
 */
static int link_range(const struct jesd204_topology *topology,
		      unsigned int link_idx, unsigned int *first,
		      unsigned int *last)
{
	if (link_idx == JESD204_LINKS_ALL) {
		*first = 0;
		*last = topology->dev_top.num_links;
		return 0;
	}

	if (link_idx >= topology->dev_top.num_links) {
		LOG_ERR("link_idx %u out of range (%u link(s))", link_idx,
			topology->dev_top.num_links);
		return -EINVAL;
	}

	*first = link_idx;
	*last = link_idx + 1;
	return 0;
}

int jesd204_fsm_start(struct jesd204_topology *topology, unsigned int link_idx)
{
	struct jesd204_dev_top *top;
	unsigned int first, last;
	int total_failures = 0;
	int ret;

	if (topology == NULL || !topology->initialised) {
		LOG_ERR("topology not initialised (jesd204_topology_init failed?)");
		return -EINVAL;
	}

	ret = link_range(topology, link_idx, &first, &last);
	if (ret) {
		return ret;
	}

	top = &topology->dev_top;

	LOG_INF("--- JESD204 FSM: bring-up (%u+1 devices, %u link(s)) ---",
		topology->devs_number, last - first);

	for (enum jesd204_dev_op op = 0; op < __JESD204_MAX_OPS; op++) {
		/*
		 * One flag per non-top device per phase: a device serving two
		 * links must not run its per_device callback twice
		 * (jesd204-fsm.c:13-14, :19-24).
		 */
		bool per_device_done[JESD204_MAX_DEVS] = { false };
		int failures = 0;

		if (!op_is_populated(topology, op)) {
			continue;
		}

		for (unsigned int l = first; l < last; l++) {
			struct jesd204_link *lnk = &top->active_links[l];

			for (unsigned int d = 0; d < topology->devs_number; d++) {
				struct jesd204_topology_dev *row =
					&topology->devs[d];
				const struct jesd204_state_op *sop =
					&row->jdev->dev_data->state_ops[op];

				if (!dev_serves_link(row, lnk->link_id)) {
					continue;
				}

				if (sop->per_device && !per_device_done[d]) {
					failures += run_dev_cb(row->jdev, op,
						JESD204_STATE_OP_REASON_INIT);
					per_device_done[d] = true;
				}

				if (sop->per_link) {
					failures += run_link_cb(row->jdev, op,
						JESD204_STATE_OP_REASON_INIT,
						lnk);
				}
			}

			/* Top device last within the link (jesd204-fsm.c:42-48). */
			if (top->jdev->dev_data->state_ops[op].per_link) {
				failures += run_link_cb(top->jdev, op,
					JESD204_STATE_OP_REASON_INIT, lnk);

				if (top->jdev->dev_data->state_ops[op].post_state_sysref) {
					jesd204_sysref_async(topology);
				}
			}
		}

		/* Then once for the phase (jesd204-fsm.c:49-53). */
		if (top->jdev->dev_data->state_ops[op].per_device) {
			failures += run_dev_cb(top->jdev, op,
					       JESD204_STATE_OP_REASON_INIT);

			if (top->jdev->dev_data->state_ops[op].post_state_sysref) {
				jesd204_sysref_async(topology);
			}
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

int jesd204_fsm_stop(struct jesd204_topology *topology, unsigned int link_idx)
{
	struct jesd204_dev_top *top;
	unsigned int first, last;
	int total_failures = 0;
	int ret;

	if (topology == NULL || !topology->initialised) {
		LOG_ERR("topology not initialised (jesd204_topology_init failed?)");
		return -EINVAL;
	}

	ret = link_range(topology, link_idx, &first, &last);
	if (ret) {
		return ret;
	}

	top = &topology->dev_top;

	LOG_INF("--- JESD204 FSM: teardown ---");

	for (int op = __JESD204_MAX_OPS - 1; op >= 0; op--) {
		bool per_device_done[JESD204_MAX_DEVS] = { false };
		int failures = 0;

		if (!op_is_populated(topology, op)) {
			continue;
		}

		/*
		 * Mirror image of the forward walk: the top device's per_device
		 * comes first here, then each link in reverse with the top
		 * device's per_link ahead of the non-top devices
		 * (jesd204-fsm.c:74-96).
		 */
		if (top->jdev->dev_data->state_ops[op].per_device) {
			failures += run_dev_cb(top->jdev, op,
					       JESD204_STATE_OP_REASON_UNINIT);
		}

		for (unsigned int l = last; l-- > first; ) {
			struct jesd204_link *lnk = &top->active_links[l];

			if (top->jdev->dev_data->state_ops[op].per_link) {
				failures += run_link_cb(top->jdev, op,
					JESD204_STATE_OP_REASON_UNINIT, lnk);
			}

			for (unsigned int d = topology->devs_number; d-- > 0; ) {
				struct jesd204_topology_dev *row =
					&topology->devs[d];
				const struct jesd204_state_op *sop =
					&row->jdev->dev_data->state_ops[op];

				if (!dev_serves_link(row, lnk->link_id)) {
					continue;
				}

				if (sop->per_device && !per_device_done[d]) {
					failures += run_dev_cb(row->jdev, op,
						JESD204_STATE_OP_REASON_UNINIT);
					per_device_done[d] = true;
				}

				if (sop->per_link) {
					failures += run_link_cb(row->jdev, op,
						JESD204_STATE_OP_REASON_UNINIT,
						lnk);
				}
			}
		}

		if (failures) {
			LOG_WRN("phase %-16s : %d failure(s) (uninit)",
				op_names[op], failures);
		}
		total_failures += failures;
	}

	return total_failures;
}
