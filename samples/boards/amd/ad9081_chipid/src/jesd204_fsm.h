/*
 * JESD204 bring-up framework -- state machine types.
 *
 * A port of the no-OS/Linux jesd204 framework (no-OS include/jesd204.h and
 * jesd204/jesd204-fsm.c), scoped to what this sample needs: one topology, one
 * link, a handful of devices.
 *
 * The point of the framework is *ordering*. A JESD204 link only comes up when
 * every device -- clock chip, converter, FPGA transceiver, FPGA link cores --
 * passes through the same phases together. Rather than hand-writing that order
 * as a script, each device registers a table of callbacks indexed by phase
 * (struct jesd204_dev_data::state_ops), and the FSM walks the phases op-major:
 * for each phase, visit every device. So all devices finish LINK_PRE_SETUP
 * before any device begins LINK_SETUP, and correct ordering follows from the
 * table rather than from the order statements happen to be written in.
 *
 * Deliberately not ported from no-OS:
 *   - multi-link topologies (jdev_top->num_links); this board has one link, so
 *     the link loop collapses to a single iteration.
 *   - jesd204_dev_priv() / sizeof_priv dynamic allocation. Devices here are
 *     static singletons and reach their own state directly.
 *   - num_retries. The field exists in no-OS's struct but its FSM never reads
 *     it, so porting it would only add dead code.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef JESD204_FSM_H_
#define JESD204_FSM_H_

#include <stdbool.h>
#include <stdint.h>

/*
 * Why a state op ran. Every callback must handle both: INIT on the way up,
 * UNINIT on the way down (jesd204_fsm_stop() walks the phases in reverse).
 * no-OS callbacks all switch on this -- see hmc7044.c:1213-1214.
 */
enum jesd204_state_op_reason {
	JESD204_STATE_OP_REASON_INIT,
	JESD204_STATE_OP_REASON_UNINIT,
};

/*
 * Whether a phase's work is per-device or per-link (no-OS jesd204.h:174-176).
 *
 * Declarative only: which callback runs is decided by which function pointer is
 * populated, not by this field, and the walker never reads it. no-OS is the same
 * -- its FSM tests the pointers too (jesd204-fsm.c:29-40). Kept because the
 * device tables read as documentation and this records the author's intent.
 */
enum jesd204_state_op_mode {
	JESD204_STATE_OP_MODE_PER_LINK,
	JESD204_STATE_OP_MODE_PER_DEVICE,
};

/*
 * The phase list, in order, from no-OS jesd204.h:197-215. Not every phase is
 * used on this board; unpopulated table slots are simply skipped, which is how
 * the framework lets a device opt into only the phases it cares about.
 */
enum jesd204_dev_op {
	JESD204_OP_DEVICE_INIT,
	JESD204_OP_LINK_INIT,
	JESD204_OP_LINK_SUPPORTED,
	JESD204_OP_LINK_PRE_SETUP,
	JESD204_OP_CLK_SYNC_STAGE1,
	JESD204_OP_CLK_SYNC_STAGE2,
	JESD204_OP_CLK_SYNC_STAGE3,
	JESD204_OP_LINK_SETUP,
	JESD204_OP_OPT_SETUP_STAGE1,
	JESD204_OP_OPT_SETUP_STAGE2,
	JESD204_OP_OPT_SETUP_STAGE3,
	JESD204_OP_OPT_SETUP_STAGE4,
	JESD204_OP_OPT_SETUP_STAGE5,
	JESD204_OP_CLOCKS_ENABLE,
	JESD204_OP_LINK_ENABLE,
	JESD204_OP_LINK_RUNNING,
	JESD204_OP_OPT_POST_RUNNING_STAGE,

	__JESD204_MAX_OPS,
};

/*
 * Link parameters handed to per-link callbacks. no-OS carries the full
 * negotiated JESD204 parameter set here; this board programs those into the
 * cores at configure time, so only the identity of the link is needed.
 */
struct jesd204_link {
	unsigned int link_id;
	bool is_transmit;
};

struct jesd204_dev;

typedef int (*jesd204_dev_cb)(struct jesd204_dev *jdev,
			      enum jesd204_state_op_reason reason);
typedef int (*jesd204_link_cb)(struct jesd204_dev *jdev,
			       enum jesd204_state_op_reason reason,
			       struct jesd204_link *lnk);
typedef int (*jesd204_sysref_cb)(struct jesd204_dev *jdev);

/*
 * One device's work for one phase (no-OS jesd204.h:189-194).
 *
 * post_state_sysref asks the framework to issue a SYSREF once the phase
 * completes, via whichever device in the topology owns sysref_cb.
 */
struct jesd204_state_op {
	enum jesd204_state_op_mode mode;
	jesd204_dev_cb per_device;
	jesd204_link_cb per_link;
	bool post_state_sysref;
};

/* A device's registration with the framework (no-OS jesd204.h:227-233). */
struct jesd204_dev_data {
	jesd204_sysref_cb sysref_cb;
	struct jesd204_state_op state_ops[__JESD204_MAX_OPS];
};

/*
 * A participating device. `name` is for logging; `dev_data` is its phase table.
 * Devices are static here, so there is no separate registration call -- the
 * topology just points at them.
 */
struct jesd204_dev {
	const char *name;
	const struct jesd204_dev_data *dev_data;
};

/*
 * The set of devices taking part, in visit order, plus the link they serve.
 *
 * Visit order matters: within a single phase, devices run in this order. It is
 * the one thing the table format does not make self-evident, so it is chosen to
 * match the order no-OS drives these blocks in.
 */
#define JESD204_MAX_DEVS 8

struct jesd204_topology {
	struct jesd204_dev *devs[JESD204_MAX_DEVS];
	unsigned int devs_number;
	struct jesd204_link link;
};

/*
 * Walk every phase from DEVICE_INIT to OPT_POST_RUNNING_STAGE with
 * REASON_INIT, visiting each device in topology order within a phase.
 *
 * Like no-OS's jesd204_fsm_start(), this is best-effort: a callback's return
 * value does not abort the walk, because a JESD204 link stalls at the *first*
 * broken phase and aborting there would hide the state of everything
 * downstream. Failures are counted and logged per phase; the return value is
 * the number of failed callbacks, so 0 means a clean walk.
 */
int jesd204_fsm_start(struct jesd204_topology *topology);

/*
 * Walk every phase in reverse with REASON_UNINIT, visiting devices in reverse
 * topology order (no-OS jesd204-fsm.c:59-101).
 *
 * How much actually gets undone is up to the device tables: a phase whose
 * callbacks return DONE on UNINIT without doing anything is a no-op on the way
 * down. This provides the walk, not the guarantee -- see jesd204_teardown() in
 * jesd_fsm.h for what the current tables really unwind.
 */
int jesd204_fsm_stop(struct jesd204_topology *topology);

/*
 * Issue a SYSREF through the topology's SYSREF-owning device. Called by the
 * framework for phases marked post_state_sysref; exposed because a caller may
 * legitimately want to re-align an already-running link.
 */
int jesd204_sysref_async(struct jesd204_topology *topology);

#endif /* JESD204_FSM_H_ */
