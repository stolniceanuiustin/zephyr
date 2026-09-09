/*
 * Copyright (c) 2026 Analog Devices, Inc.
 *
 * DAQ2 JESD204B bring-up -- this board's device state tables and topology.
 *
 * The other half is jesd204_fsm.c, the generic phase walker (copied verbatim
 * from the AD9081 sample). It knows about phases, links and visit order and
 * nothing about any converter, transceiver or FPGA core. Everything board- and
 * chip-specific is here; the walker calls back into this file, never the reverse.
 *
 * DAQ2 differs from the AD9081 board in a way that shapes this file: it has two
 * *separate* converters, an AD9144 TX DAC and an AD9680 RX ADC, each on its own
 * single-direction JESD204 link, rather than one MxFE serving both a framer and a
 * deframer link. So instead of one topology with a single top device owning two
 * links, there are TWO independent single-link topologies, each with its own
 * converter as top device, walked in turn (TX then RX). The two links are
 * physically independent, so the order between the walks should not matter
 * (UNVERIFIED).
 *
 * What each side must do to come up (the enable path Phase 1's linear configure
 * path was missing):
 *
 *   CLOCKS_ENABLE  GT reset-release (adxcvr), then FPGA lane clocks (jesd core),
 *                  then -- because the converter is the top device and top
 *                  devices are visited last -- the chip's PLL lock re-read. The
 *                  adxcvr must precede the link core (it needs a running GT
 *                  clock), and the PLL check is only meaningful once the GT is
 *                  up, so the converter goes last. Getting this order wrong is
 *                  exactly what Phase 1's linear sequence did: it re-read the
 *                  chip PLLs with the GT still in reset and the link disabled.
 *   LINK_RUNNING   poll the link's FPGA core for DATA, then read its status word.
 *
 * No LINK_ENABLE phase: both DAQ2 converters already enable their own
 * framer/deframer at init() (ad9144.c, ad9680.c), so unlike the AD9081 sample
 * there is no chip-side deframer enable to drive here.
 *
 * The chip PLL re-read is non-fatal: whether the AD9144 SERDES PLL and AD9680
 * JESD PLL lock once the FPGA datapath is armed is the empirical unknown this
 * phase exists to resolve, so it is logged, not gated.
 *
 * SYSREF: the AD9523 emits DEV/FPGA SYSREF continuously, so no device registers
 * a sysref_cb and no phase requests a pulse -- as in the AD9081 sample.
 * Subclass-1 alignment against that free-running SYSREF is UNVERIFIED for DAQ2.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(daq2_fsm, LOG_LEVEL_INF);

#include <zephyr/drivers/misc/jesd204/axi_adxcvr.h>
#include <zephyr/drivers/misc/jesd204/axi_jesd204.h>
#include <zephyr/drivers/misc/ad9144.h>
#include <zephyr/drivers/misc/ad9680.h>
#include <zephyr/jesd204/jesd204_fsm.h>
#include "daq2_bringup.h"

/*
 * Single link per topology. DAQ2's two converters are independent, each the top
 * device of its own topology, so both use link id 0 -- the id only has to be
 * unique within a topology, and each topology has just the one link.
 */
#define DAQ2_LINK0 0

/*
 * Every callback returns JESD204_STATE_CHANGE_DONE on success and a negative
 * errno on failure. On UNINIT the enable phases have nothing to undo and say so
 * by returning DONE.
 */
#define JESD204_STATE_CHANGE_DONE 1

/* ------------------------------------------------------- GT transceivers --- */

/*
 * GT reset-release for one direction's transceiver. One jesd204_dev per adxcvr
 * instance, each carrying its Zephyr device in `priv`, so the phase table is
 * shared and the topology says which instance serves which link.
 */
static int adxcvr_fsm_clks_enable(struct jesd204_dev *jdev,
				  enum jesd204_state_op_reason reason,
				  struct jesd204_link *lnk)
{
	int ret;

	ARG_UNUSED(lnk);

	if (reason != JESD204_STATE_OP_REASON_INIT) {
		return JESD204_STATE_CHANGE_DONE;
	}

	ret = axi_adxcvr_enable(jesd204_dev_priv(jdev));
	if (ret != 0) {
		return ret;
	}

	return JESD204_STATE_CHANGE_DONE;
}

static const struct jesd204_dev_data adxcvr_jesd204_data = {
	.max_num_links = 1,
	.state_ops = {
		[JESD204_OP_CLOCKS_ENABLE] = {
			.per_link = adxcvr_fsm_clks_enable,
		},
	},
};

static struct jesd204_dev tx_adxcvr_jdev = {
	.name = "tx_adxcvr",
	.dev_data = &adxcvr_jesd204_data,
	.priv = (void *)DEVICE_DT_GET(DT_NODELABEL(tx_adxcvr)),
};

static struct jesd204_dev rx_adxcvr_jdev = {
	.name = "rx_adxcvr",
	.dev_data = &adxcvr_jesd204_data,
	.priv = (void *)DEVICE_DT_GET(DT_NODELABEL(rx_adxcvr)),
};

/* ---------------------------------------------------- FPGA JESD204 cores --- */

static int axi_jesd204_fsm_clks_enable(struct jesd204_dev *jdev,
				       enum jesd204_state_op_reason reason,
				       struct jesd204_link *lnk)
{
	int ret;

	ARG_UNUSED(lnk);

	if (reason != JESD204_STATE_OP_REASON_INIT) {
		return JESD204_STATE_CHANGE_DONE;
	}

	ret = axi_jesd204_lane_clk_enable(jesd204_dev_priv(jdev));
	if (ret != 0) {
		return ret;
	}

	return JESD204_STATE_CHANGE_DONE;
}

/*
 * The authoritative link check: this link's FPGA core reporting DATA. Polled
 * rather than assumed -- 20 attempts at 4 ms, then the status word is logged
 * whatever the outcome. Same budget the AD9081 sample uses.
 */
static int axi_jesd204_fsm_link_running(struct jesd204_dev *jdev,
					enum jesd204_state_op_reason reason,
					struct jesd204_link *lnk)
{
	const struct device *core = jesd204_dev_priv(jdev);
	bool is_data = false;

	ARG_UNUSED(lnk);

	if (reason != JESD204_STATE_OP_REASON_INIT) {
		return JESD204_STATE_CHANGE_DONE;
	}

	for (int attempt = 0; attempt < 20 && !is_data; attempt++) {
		k_msleep(4);
		is_data = axi_jesd204_link_is_data(core);
	}

	if (axi_jesd204_status_read(core)) {
		return -EIO;
	}

	return JESD204_STATE_CHANGE_DONE;
}

static const struct jesd204_dev_data axi_jesd204_jesd204_data = {
	.max_num_links = 1,
	.state_ops = {
		[JESD204_OP_CLOCKS_ENABLE] = {
			.per_link = axi_jesd204_fsm_clks_enable,
		},
		[JESD204_OP_LINK_RUNNING] = {
			.per_link = axi_jesd204_fsm_link_running,
		},
	},
};

static struct jesd204_dev tx_jesd_jdev = {
	.name = "tx_jesd",
	.dev_data = &axi_jesd204_jesd204_data,
	.priv = (void *)DEVICE_DT_GET(DT_NODELABEL(tx_jesd)),
};

static struct jesd204_dev rx_jesd_jdev = {
	.name = "rx_jesd",
	.dev_data = &axi_jesd204_jesd204_data,
	.priv = (void *)DEVICE_DT_GET(DT_NODELABEL(rx_jesd)),
};

/* ------------------------------------------------------------ converters --- */

/*
 * AD9144 TX DAC (TX topology top device). Re-run the SERDES PLL bring-up now
 * that the FPGA TX datapath is armed. The AD9144 is visited last in this phase
 * (topology order {tx_adxcvr, tx_jesd, ad9144}), so the GT reset-release and
 * FPGA lane clock are already up -- the clocking environment the SERDES PLL
 * needs, which did NOT exist at chip init(). Re-enabling here (not just
 * re-reading) mirrors no-OS driving its LINK_SETUP phase after CLOCKS_ENABLE
 * (no-OS ad9144.c:860). Non-fatal: whether it locks is logged, not gated.
 */
static int ad9144_fsm_clks_enable(struct jesd204_dev *jdev,
				  enum jesd204_state_op_reason reason,
				  struct jesd204_link *lnk)
{
	bool locked = false;

	ARG_UNUSED(lnk);

	if (reason != JESD204_STATE_OP_REASON_INIT) {
		return JESD204_STATE_CHANGE_DONE;
	}

	if (ad9144_serdes_enable(jesd204_dev_priv(jdev), &locked)) {
		LOG_WRN("AD9144 SERDES PLL re-enable failed");
	} else {
		LOG_INF("AD9144 SERDES PLL: %s", locked ? "locked" : "NOT locked");
	}

	return JESD204_STATE_CHANGE_DONE;
}

static const struct jesd204_dev_data ad9144_jesd204_data = {
	.max_num_links = 1,
	.state_ops = {
		[JESD204_OP_CLOCKS_ENABLE] = {
			.per_link = ad9144_fsm_clks_enable,
		},
	},
};

static struct jesd204_dev ad9144_jdev = {
	.name = "ad9144",
	.dev_data = &ad9144_jesd204_data,
	.priv = (void *)DEVICE_DT_GET(DT_NODELABEL(ad9144)),
};

/*
 * AD9680 RX ADC (RX topology top device). Same shape: re-read the JESD204B
 * (framer serializer) PLL, log, non-fatal.
 */
static int ad9680_fsm_clks_enable(struct jesd204_dev *jdev,
				  enum jesd204_state_op_reason reason,
				  struct jesd204_link *lnk)
{
	bool locked = false;

	ARG_UNUSED(lnk);

	if (reason != JESD204_STATE_OP_REASON_INIT) {
		return JESD204_STATE_CHANGE_DONE;
	}

	if (ad9680_jesd_pll_locked(jesd204_dev_priv(jdev), &locked)) {
		LOG_WRN("AD9680 JESD PLL re-read failed");
	} else {
		LOG_INF("AD9680 JESD PLL: %s", locked ? "locked" : "NOT locked");
	}

	return JESD204_STATE_CHANGE_DONE;
}

static const struct jesd204_dev_data ad9680_jesd204_data = {
	.max_num_links = 1,
	.state_ops = {
		[JESD204_OP_CLOCKS_ENABLE] = {
			.per_link = ad9680_fsm_clks_enable,
		},
	},
};

static struct jesd204_dev ad9680_jdev = {
	.name = "ad9680",
	.dev_data = &ad9680_jesd204_data,
	.priv = (void *)DEVICE_DT_GET(DT_NODELABEL(ad9680)),
};

/* ------------------------------------------------------------ topologies --- */

/*
 * TX topology: FPGA transceiver out of reset, then the link core's lane clock,
 * then (last, by is_top_device) the AD9144. is_transmit = true throughout: from
 * the converter's point of view the TX DAC receives JESD and drives analogue out.
 */
static const struct jesd204_topology_dev tx_topology_devs[] = {
	{
		.jdev = &tx_adxcvr_jdev,
		.link_ids = { DAQ2_LINK0 },
		.is_transmit = { true },
		.links_number = 1,
	},
	{
		.jdev = &tx_jesd_jdev,
		.link_ids = { DAQ2_LINK0 },
		.is_transmit = { true },
		.links_number = 1,
	},
	{
		.jdev = &ad9144_jdev,
		.link_ids = { DAQ2_LINK0 },
		.is_transmit = { true },
		.links_number = 1,
		.is_top_device = true,
	},
};

/*
 * RX topology: same shape, is_transmit = false -- the RX ADC samples analogue in
 * and transmits JESD.
 */
static const struct jesd204_topology_dev rx_topology_devs[] = {
	{
		.jdev = &rx_adxcvr_jdev,
		.link_ids = { DAQ2_LINK0 },
		.is_transmit = { false },
		.links_number = 1,
	},
	{
		.jdev = &rx_jesd_jdev,
		.link_ids = { DAQ2_LINK0 },
		.is_transmit = { false },
		.links_number = 1,
	},
	{
		.jdev = &ad9680_jdev,
		.link_ids = { DAQ2_LINK0 },
		.is_transmit = { false },
		.links_number = 1,
		.is_top_device = true,
	},
};

static struct jesd204_topology tx_topology;
static struct jesd204_topology rx_topology;

/*
 * Every device in a topology must be ready before its walk is attempted.
 * device_is_ready(), not a NULL check: DEVICE_DT_GET() resolves at build time and
 * can never be NULL, so it fails only if a node's init() returned an error. A
 * node given status = "disabled" fails the BUILD_ASSERT below at build time
 * rather than producing a device missing at run time.
 */
#define DAQ2_TX_PARTICIPANTS(fn)		\
	fn(DT_NODELABEL(tx_adxcvr))		\
	fn(DT_NODELABEL(tx_jesd))		\
	fn(DT_NODELABEL(ad9144))

#define DAQ2_RX_PARTICIPANTS(fn)		\
	fn(DT_NODELABEL(rx_adxcvr))		\
	fn(DT_NODELABEL(rx_jesd))		\
	fn(DT_NODELABEL(ad9680))

#define DAQ2_ASSERT_ENABLED(node_id)						\
	BUILD_ASSERT(DT_NODE_HAS_STATUS_OKAY(node_id),				\
		     "DAQ2 JESD204 topology names a disabled node: "		\
		     DT_NODE_FULL_NAME(node_id));

DAQ2_TX_PARTICIPANTS(DAQ2_ASSERT_ENABLED)
DAQ2_RX_PARTICIPANTS(DAQ2_ASSERT_ENABLED)

#define DAQ2_CHECK_READY(node_id)						\
	if (!device_is_ready(DEVICE_DT_GET(node_id))) {				\
		LOG_ERR("DAQ2 JESD204 device not initialised: %s",		\
			DEVICE_DT_GET(node_id)->name);				\
		return -ENODEV;							\
	}

static int tx_participants_ready(void)
{
	DAQ2_TX_PARTICIPANTS(DAQ2_CHECK_READY)
	return 0;
}

static int rx_participants_ready(void)
{
	DAQ2_RX_PARTICIPANTS(DAQ2_CHECK_READY)
	return 0;
}

/*
 * Walk one topology to completion. Returns 0 if the link reaches running state,
 * -errno if the walk could not be started, or -EIO if it failed to come fully up.
 */
static int daq2_link_bringup(const char *tag, struct jesd204_topology *topology,
			     const struct jesd204_topology_dev *devs, size_t n)
{
	int failures;
	int ret;

	ret = jesd204_topology_init(topology, devs, n);
	if (ret) {
		LOG_ERR("%s topology is malformed (%d)", tag, ret);
		return ret;
	}

	failures = jesd204_fsm_start(topology, JESD204_LINKS_ALL);
	if (failures < 0) {
		return failures;
	}

	if (failures) {
		LOG_WRN("=== %s JESD204B link NOT fully up (%d failed step(s)) ===",
			tag, failures);
		return -EIO;
	}

	LOG_INF("=== %s JESD204B LINK UP (carrying DATA) ===", tag);
	return 0;
}

int daq2_jesd204_bringup(void)
{
	int ret;
	int first_err = 0;

	ret = tx_participants_ready();
	if (ret) {
		return ret;
	}
	ret = rx_participants_ready();
	if (ret) {
		return ret;
	}

	/*
	 * Two independent walks, TX then RX. Both are attempted even if the first
	 * fails -- the links are separate converters, so an RX result is still
	 * worth having when TX did not come up. The first error is returned.
	 */
	ret = daq2_link_bringup("TX", &tx_topology, tx_topology_devs,
				ARRAY_SIZE(tx_topology_devs));
	if (ret && !first_err) {
		first_err = ret;
	}

	ret = daq2_link_bringup("RX", &rx_topology, rx_topology_devs,
				ARRAY_SIZE(rx_topology_devs));
	if (ret && !first_err) {
		first_err = ret;
	}

	return first_err;
}
