/*
 * Copyright (c) 2026 Analog Devices, Inc.
 *
 * JESD204 bring-up framework -- state machine types.
 *
 * The point of the framework is *ordering*. A JESD204 link only comes up when
 * every device -- clock chip, converter, FPGA transceiver, FPGA link cores --
 * passes through the same phases together. Rather than hand-writing that order
 * as a script, each device registers a table of callbacks indexed by phase
 * (struct jesd204_dev_data::state_ops), and the FSM walks the phases op-major:
 * for each phase, for each link, visit every device. So all devices finish
 * LINK_PRE_SETUP before any device begins LINK_SETUP, and correct ordering
 * follows from the tables rather than from the order statements happen to be
 * written in.
 *
 * Ordering rules, all load-bearing:
 *
 *   - Within a phase, non-top devices are visited in topology array order, and
 *     the device marked is_top_device is visited *last*. That is what gets the
 *     GT out of reset before the converter checks its JESD PLL -- the converter
 *     is the top device, so it is always last.
 *   - A device registered against several links runs its per_link callback once
 *     per link, but its per_device callback only once per phase.
 *   - jesd204_fsm_stop() walks phases in reverse, and devices in reverse, with
 *     the top device *first*.
 *
 * Everything here is statically allocated: the caller owns one
 * struct jesd204_topology by value, so a bring-up cannot fail for want of heap,
 * and a driver points `priv` straight at state it already owns. There is
 * deliberately only one SYSREF provider -- a fallback second one would be a code
 * path with no way to reach it.
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
 */
enum jesd204_state_op_reason {
	JESD204_STATE_OP_REASON_INIT,
	JESD204_STATE_OP_REASON_UNINIT,
};

/*
 * Whether a phase's work is per-device or per-link.
 *
 * Declarative only: which callback runs is decided by which function pointer is
 * populated, not by this field, and the walker never reads it. Kept because the
 * device tables read as documentation and this records the author's intent.
 */
enum jesd204_state_op_mode {
	JESD204_STATE_OP_MODE_PER_LINK,
	JESD204_STATE_OP_MODE_PER_DEVICE,
};

/*
 * The phase list, in order. Not every phase is used on this board; unpopulated
 * table slots are simply skipped, which is how the framework lets a device opt
 * into only the phases it cares about.
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
 * Link parameters handed to per-link callbacks. Only the identity and direction
 * of the link is needed -- the full JESD204 parameter set is programmed into the
 * cores at configure time, from devicetree.
 *
 * is_transmit is from the *converter's* point of view: true means the chip
 * receives JESD data on its JRX deframer and transmits analogue -- the TX
 * datapath. The framework never sets it; the topology declares it per link,
 * because the link-id-to-direction convention belongs to the converter and not
 * to the walker.
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
 * One device's work for one phase.
 *
 * post_state_sysref asks the framework to issue a SYSREF once the phase
 * completes, via whichever device in the topology was marked
 * is_sysref_provider.
 */
struct jesd204_state_op {
	enum jesd204_state_op_mode mode;
	jesd204_dev_cb per_device;
	jesd204_link_cb per_link;
	bool post_state_sysref;
};

/* A device's registration with the framework. */
struct jesd204_dev_data {
	jesd204_sysref_cb sysref_cb;
	unsigned int max_num_links;
	struct jesd204_state_op state_ops[__JESD204_MAX_OPS];
};

/*
 * A participating device.
 *
 * `name` is for logging; `dev_data` is its phase table. Devices here are static,
 * so there is no register() call -- a driver defines one of these next to its
 * phase table and the topology points at it. `is_top` is set by
 * jesd204_topology_init(), not by the driver.
 *
 * `priv` is how one phase table serves several instances: the RX and TX link
 * cores run identical callbacks against different registers, so each gets its own
 * jesd204_dev with the same dev_data and a different priv -- usually the Zephyr
 * struct device *.
 */
struct jesd204_dev {
	const char *name;
	const struct jesd204_dev_data *dev_data;
	void *priv;
	bool is_top;
};

static inline void *jesd204_dev_priv(struct jesd204_dev *jdev)
{
	return jdev->priv;
}

/* Bound on the links one topology can declare. */
#define JESD204_MAX_TOPOLOGY_LINKS 16

/* Bound on one topology's non-top devices. */
#define JESD204_MAX_DEVS 8

/*
 * One row of the client-written topology array.
 *
 * This is the structure a board port fills in, and the only one it needs to
 * understand:
 *
 *   is_top_device      exactly one row must set this. The top device is the
 *                      converter (ADC/DAC/transceiver); it is visited last in
 *                      every forward phase and first in every reverse phase. It
 *                      also declares the topology's link set -- num_links and
 *                      the link IDs come from this row -- so its link_ids[] must
 *                      be the union of every other row's.
 *   is_sysref_provider at most one row. Its device's dev_data->sysref_cb is what
 *                      jesd204_sysref_async() calls.
 *   link_ids           which links this device takes part in. A device is only
 *                      visited for a link it lists.
 *   is_transmit        direction of each link, positionally matching link_ids[].
 *                      Only read from the top device's row, since that row is
 *                      what defines the topology's links. Declared here rather
 *                      than in the converter driver because the converter does
 *                      not own the link struct in this design.
 */
struct jesd204_topology_dev {
	struct jesd204_dev *jdev;
	bool is_top_device;
	bool is_sysref_provider;
	unsigned int link_ids[JESD204_MAX_TOPOLOGY_LINKS];
	bool is_transmit[JESD204_MAX_TOPOLOGY_LINKS];
	unsigned int links_number;
};

/*
 * The top device, split out of the topology array by jesd204_topology_init().
 * Owns the link set every other device is matched against.
 */
struct jesd204_dev_top {
	struct jesd204_dev *jdev;
	struct jesd204_dev *jdev_sysref;
	unsigned int link_ids[JESD204_MAX_TOPOLOGY_LINKS];
	unsigned int num_links;
	struct jesd204_link active_links[JESD204_MAX_TOPOLOGY_LINKS];
};

/*
 * A prepared topology. Produced by jesd204_topology_init() from the client's
 * array; `devs` holds the non-top rows only, in the order the client wrote them.
 *
 * Held by value rather than allocated, so a board declares one at file scope and
 * bring-up cannot fail for want of heap.
 */
struct jesd204_topology {
	struct jesd204_dev_top dev_top;
	struct jesd204_topology_dev devs[JESD204_MAX_DEVS];
	unsigned int devs_number;
	bool initialised;
};

/* Walk every link in the topology. */
#define JESD204_LINKS_ALL ((unsigned int)(-1))

/*
 * Prepare `topology` from a client-written device array.
 *
 * Splits out the is_top_device row, adopts its link set as the topology's, and
 * records the SYSREF provider. Returns 0, or -EINVAL if the array is malformed:
 * no top device, more than one, more than one SYSREF provider, a row with no
 * links, a link count over JESD204_MAX_TOPOLOGY_LINKS, a duplicate link ID, a
 * non-top row referencing a link the top device does not declare, a device whose
 * dev_data->max_num_links is smaller than the number of links it is given, or
 * more non-top rows than JESD204_MAX_DEVS.
 *
 * Validated rather than trusted because a missing or misplaced is_top_device is
 * the one mistake this API makes easy to make, and its symptom -- the converter
 * visited in the wrong position within a phase -- is a link that comes up looking
 * fine on a good board and intermittently on a marginal one. It was a real bug in
 * this port's own history: the converter was listed first, which moved its JESD
 * PLL check ahead of the GT reset-release.
 */
int jesd204_topology_init(struct jesd204_topology *topology,
			  const struct jesd204_topology_dev *devs,
			  unsigned int devs_number);

/*
 * Walk every phase from DEVICE_INIT to OPT_POST_RUNNING_STAGE with REASON_INIT.
 * Pass JESD204_LINKS_ALL for link_idx to bring up every link, or an index into
 * the top device's link_ids[] to bring up one.
 *
 * Within a phase: for each link, every non-top device that lists that link, in
 * topology array order, then the top device. per_device callbacks fire once per
 * phase regardless of link count; per_link callbacks fire once per link.
 *
 * Best-effort by design: a callback's return value does not abort the walk,
 * because a JESD204 link stalls at the *first*
 * broken phase and aborting there would hide the state of everything
 * downstream. Failures are counted and logged per phase; the return value is
 * the number of failed callbacks, so 0 means a clean walk.
 *
 * The one thing that does abort is a topology that was never successfully
 * initialised, or a link_idx out of range: returns -EINVAL without walking.
 * That is a programming error, not a hardware failure, so there is no
 * downstream state worth exposing.
 */
int jesd204_fsm_start(struct jesd204_topology *topology, unsigned int link_idx);

/*
 * Walk every phase in reverse with REASON_UNINIT: the top device first, then the
 * non-top devices in reverse array order.
 *
 * How much actually gets undone is up to the device tables: a phase whose
 * callbacks return DONE on UNINIT without doing anything is a no-op on the way
 * down. This provides the walk, not the guarantee -- see jesd204_teardown() in
 * jesd_fsm.h for what the current tables really unwind.
 */
int jesd204_fsm_stop(struct jesd204_topology *topology, unsigned int link_idx);

/*
 * Issue a SYSREF through the topology's SYSREF provider. Called by the framework
 * after any phase in which a device set post_state_sysref; exposed because a
 * caller may legitimately want to re-align an already-running link.
 */
int jesd204_sysref_async(struct jesd204_topology *topology);

#endif /* JESD204_FSM_H_ */
