/*
 * JESD204B bring-up -- this board's device state tables and topology.
 *
 * Every block (adxcvr, jesd204 link cores, TPL, AD9082 datapath) is *configured*
 * separately at boot, each holding its link/reset dark. A JESD204 link only
 * comes alive when the FPGA transceiver, the FPGA link cores and the AD9082's
 * framer/deframer are activated together in a fixed phase order, with SYSREF
 * crossing both ends.
 *
 * That order lives in the tables below rather than in a hand-written sequence.
 * Each participating device registers callbacks against the phases it cares
 * about, and jesd204_fsm_start() (jesd204_fsm.c) walks the phases op-major:
 * every device completes a phase before any device starts the next. This is the
 * structure of the no-OS/Linux jesd204 framework -- see
 * no-OS drivers/frequency/hmc7044/hmc7044.c:1441 and
 * drivers/axi_core/jesd204/axi_jesd204_rx.c:840 for the reference tables.
 *
 * TWO LINKS, as in no-OS. The AD9082's JTX framer (chip transmits JESD, analogue
 * in -- the RX datapath) and its JRX deframer (chip receives JESD, analogue out
 * -- the TX datapath) are separate JESD204 links with separate IDs, and the
 * framework visits each in turn within every phase. The IDs and the naming are
 * the converter's, from no-OS drivers/adc/ad9081/ad9081.h:239-241.
 *
 * `is_transmit` throughout is from the converter's point of view (no-OS
 * ad9081.c:923, `is_transmit = !jtx`): true on DEFRAMER_LINK0_TX, false on
 * FRAMER_LINK0_RX. It is the switch every per-link callback below uses to decide
 * which of its two instances the phase applies to.
 *
 * Phase assignment here, and where it comes from in no-OS:
 *
 *   CLK_SYNC_STAGE1  chip one-shot SYNC (subclass 1). The clock-sync phases are
 *                    where no-OS's HMC7044 driver does its tree sync
 *                    (hmc7044.c:1447-1458); this chip's SYNC belongs in the same
 *                    window.
 *   CLK_SYNC_STAGE2  chip NCO sync.
 *   CLOCKS_ENABLE    GT reset-release, then FPGA lane clocks, then -- because the
 *                    converter is the topology's top device and top devices are
 *                    visited last -- the chip's JESD PLL lock check and 204C
 *                    calibration. That is exactly no-OS's order: adxcvr before
 *                    the link cores (jesd204_clk.c:44-66), converter last
 *                    (jesd204-fsm.c:42-53), and the PLL check needs a running GT
 *                    to be meaningful.
 *   LINK_ENABLE      chip JRX deframer enable, after clearing the JRX
 *                    transport-layer buffer protection. TX link only -- the JTX
 *                    framer runs once its digital reset is released and SYSREF
 *                    arrives.
 *   LINK_RUNNING     poll for DATA on the link's FPGA core, read the chip's
 *                    matching status word, then verify the TPL datapath.
 *
 * Ordering *within* a phase is: non-top devices in the order they appear in
 * board_topology_devs[] below, then the top device. Nothing else. The rank
 * mechanism a previous version of this file used is gone -- `is_top_device` is
 * what keeps the converter last, and it is what no-OS uses.
 *
 * SYSREF: the HMC7044 emits DEV_SYSREF / FPGA_SYSREF continuously at 1.953 MHz
 * (hmc7044.c), so no device registers a sysref_cb and no phase requests a pulse.
 * Subclass-1 alignment happens against that free-running SYSREF as each end is
 * enabled. The framework hook exists for a board that gates SYSREF instead.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(jesd_fsm, LOG_LEVEL_INF);

#include "ad9081.h"
#include "axi_adxcvr.h"
#include "axi_jesd204.h"
#include "axi_tpl.h"
#include "jesd204_fsm.h"

/*
 * Link IDs, from no-OS drivers/adc/ad9081/ad9081.h:239-241. The values matter
 * only in that the two links differ; they are the converter's convention and are
 * kept identical to it so a topology copied from a no-OS project ports without
 * renumbering.
 */
#define DEFRAMER_LINK0_TX 0
#define FRAMER_LINK0_RX   2

/*
 * Every callback returns JESD204_STATE_CHANGE_DONE on success and a negative
 * errno on failure, as in no-OS. On UNINIT most of them have nothing to undo:
 * the phase is a no-op on the way down and says so by returning DONE.
 */
#define JESD204_STATE_CHANGE_DONE 1

/* ---------------------------------------------------------------- AD9082 --- */

/*
 * The converter, reached only through the ad9081_*() ops in ad9081.h. This file
 * used to hold the vendor handle and call adi_ad9081_* on it directly; nothing
 * here names the part's registers or vendor enums any more, so the phase
 * callbacks below describe what a converter does at each phase rather than how
 * this one does it.
 */
static const struct device *chip(void)
{
	return DEVICE_DT_GET(DT_NODELABEL(ad9081));
}

static int ad9081_fsm_oneshot_sync(struct jesd204_dev *jdev,
				   enum jesd204_state_op_reason reason)
{
	ARG_UNUSED(jdev);

	if (reason != JESD204_STATE_OP_REASON_INIT) {
		return JESD204_STATE_CHANGE_DONE;
	}

	/* Subclass is not passed: the op takes it from the link geometry. */
	if (ad9081_sync_oneshot(chip())) {
		return -EIO;
	}
	return JESD204_STATE_CHANGE_DONE;
}

static int ad9081_fsm_nco_sync(struct jesd204_dev *jdev,
			       enum jesd204_state_op_reason reason)
{
	ARG_UNUSED(jdev);

	if (reason != JESD204_STATE_OP_REASON_INIT) {
		return JESD204_STATE_CHANGE_DONE;
	}

	if (ad9081_sync_nco(chip())) {
		return -EIO;
	}
	return JESD204_STATE_CHANGE_DONE;
}

/*
 * JESD PLL lock check, plus the deframer's serdes calibration on the TX link.
 *
 * no-OS splits it the same way (ad9081.c:1047-1100): the PLL status read is
 * unconditional, and the calibration is guarded by lnk->is_transmit because it
 * touches the JRX deframer, which only the TX link has. no-OS additionally
 * guards the calibration on JESD204_VERSION_C; this port is 204B-only, and the
 * calibration is run anyway because it was measured to be needed on this
 * silicon.
 *
 * Being a per-link callback on the top device, the PLL status read happens once
 * per link -- two identical reads and two identical log lines. That is what
 * no-OS does too, and it is left that way rather than hoisted to a per_device
 * callback: per_device would run before any link's per_link work in this phase,
 * which is a different position in the sequence, not just a different log.
 */
static int ad9081_fsm_clks_enable(struct jesd204_dev *jdev,
				  enum jesd204_state_op_reason reason,
				  struct jesd204_link *lnk)
{
	uint8_t pll_status = 0;

	ARG_UNUSED(jdev);

	if (reason != JESD204_STATE_OP_REASON_INIT) {
		return JESD204_STATE_CHANGE_DONE;
	}

	if (ad9081_jesd_pll_status_get(chip(), &pll_status)) {
		LOG_ERR("chip JESD PLL status read failed");
		return -EIO;
	}
	LOG_INF("chip JESD PLL status = 0x%x", pll_status);
	if (!pll_status) {
		return -EIO;
	}

	if (!lnk->is_transmit) {
		return JESD204_STATE_CHANGE_DONE;
	}

	/* Deframer serdes calibration: force reset, no lane boost, run background. */
	if (ad9081_deframer_calibrate(chip(), true, 0, true)) {
		LOG_WRN("chip JRX 204C calibration failed");
		return -EIO;
	}

	return JESD204_STATE_CHANGE_DONE;
}

/*
 * Enable the chip's JRX deframer. TX link only, as in no-OS
 * (ad9081.c:1125-1132, guarded on lnk->is_transmit): the JTX framer has no
 * equivalent enable here -- it runs once its digital reset is released, which
 * startup_rx already did, and SYSREF arrives.
 */
static int ad9081_fsm_link_enable(struct jesd204_dev *jdev,
				  enum jesd204_state_op_reason reason,
				  struct jesd204_link *lnk)
{
	ARG_UNUSED(jdev);

	if (!lnk->is_transmit) {
		return JESD204_STATE_CHANGE_DONE;
	}

	if (reason == JESD204_STATE_OP_REASON_UNINIT) {
		if (ad9081_deframer_enable(chip(), false)) {
			return -EIO;
		}
		return JESD204_STATE_CHANGE_DONE;
	}

	/*
	 * Disable the JRX transport-layer elastic-buffer protection before
	 * enabling the link.
	 *
	 * JRX_TPL_1 (0x4A1) bit6 BUF_PROTECT_EN withholds samples from the
	 * deframer output when the elastic-buffer phase is judged marginal. It
	 * resets to 1 -- measured 0x4A1 = 0x41 on this board -- and nothing
	 * clears it on a 204B link: the vendor API clears only bit7
	 * BUF_PROTECTION, and no-OS clears bit6 only for 204C on rev<3 silicon.
	 * A 204B port therefore inherits it enabled, so this write brings the
	 * link in line with what the reference code does for 204C.
	 *
	 * It was found while chasing the ~2.7 ms gating of the DAC output, and it
	 * is NOT the cause: clearing it here (confirmed 0x41 -> 0x01 by readback)
	 * left the gating unchanged. PHASE_DIFF (0x4A5) also reads a stable 4, so
	 * the protection had no marginal phase to act on in the first place. Kept
	 * because inheriting a reset default the reference code clears is worth
	 * not doing, not because it fixes anything.
	 */
	if (ad9081_deframer_buf_protect_disable(chip())) {
		LOG_WRN("chip JRX buf-protect clear failed");
		return -EIO;
	}

	if (ad9081_deframer_enable(chip(), true)) {
		return -EIO;
	}

	/* Let the link negotiate CGS -> ILAS -> DATA before anyone reads status. */
	k_msleep(10);

	return JESD204_STATE_CHANGE_DONE;
}

/*
 * Read the chip-side status word for this link: the JRX deframer's on the TX
 * link, the JTX framer's on the RX link. no-OS splits it the same way
 * (ad9081.c:1157-1165).
 *
 * Reported, not gated. The chip status words are diagnostic here -- the
 * authoritative DATA check is the FPGA link core's own, in
 * axi_jesd204_fsm_link_running(). no-OS treats them the same way (app.c:450-451
 * prints them and acts on neither).
 */
static int ad9081_fsm_link_running(struct jesd204_dev *jdev,
				   enum jesd204_state_op_reason reason,
				   struct jesd204_link *lnk)
{
	uint16_t status = 0;

	ARG_UNUSED(jdev);

	if (reason != JESD204_STATE_OP_REASON_INIT) {
		return JESD204_STATE_CHANGE_DONE;
	}

	if (lnk->is_transmit) {
		if (ad9081_deframer_status_get(chip(), &status)) {
			LOG_WRN("chip deframer status read failed");
		} else {
			LOG_INF("chip JRX (deframer) link status = 0x%04x",
				status);
		}
	} else {
		if (ad9081_framer_status_get(chip(), &status)) {
			LOG_WRN("chip framer status read failed");
		} else {
			LOG_INF("chip JTX (framer)   link status = 0x%04x",
				status);
		}
	}

	return JESD204_STATE_CHANGE_DONE;
}

/*
 * max_num_links = 2: this chip serves both its framer and its deframer link.
 * no-OS declares 4 (ad9081.c:1425) because the part supports dual-link in each
 * direction; this board's bitstream does not, so 2 is the honest bound and the
 * framework rejects a topology that hands it more.
 */
static const struct jesd204_dev_data ad9081_jesd204_data = {
	.max_num_links = 2,
	.state_ops = {
		[JESD204_OP_CLK_SYNC_STAGE1] = {
			.per_device = ad9081_fsm_oneshot_sync,
			.mode = JESD204_STATE_OP_MODE_PER_DEVICE,
		},
		[JESD204_OP_CLK_SYNC_STAGE2] = {
			.per_device = ad9081_fsm_nco_sync,
			.mode = JESD204_STATE_OP_MODE_PER_DEVICE,
		},
		[JESD204_OP_CLOCKS_ENABLE] = {
			.per_link = ad9081_fsm_clks_enable,
		},
		[JESD204_OP_LINK_ENABLE] = {
			.per_link = ad9081_fsm_link_enable,
		},
		[JESD204_OP_LINK_RUNNING] = {
			.per_link = ad9081_fsm_link_running,
		},
	},
};

static struct jesd204_dev ad9081_jdev = {
	.name = "ad9082",
	.dev_data = &ad9081_jesd204_data,
};

/* ------------------------------------------------------- GT transceivers --- */

/*
 * GT reset-release for one direction's transceiver.
 *
 * One jesd204_dev per adxcvr instance, each carrying its Zephyr device in
 * `priv`, so the phase table is shared and the topology says which instance
 * serves which link. That is how no-OS's link cores work
 * (axi_jesd204_rx.c:735-736 recovers its instance from jesd204_dev_priv()); the
 * GT itself is not a topology device in no-OS, where it is reached through the
 * lane-clock abstraction instead (jesd204_clk.c:48-52). Making it one here is
 * the same sequence expressed as a device: adxcvr before the link cores, both
 * before the converter.
 */
static int adxcvr_fsm_clks_enable(struct jesd204_dev *jdev,
				  enum jesd204_state_op_reason reason,
				  struct jesd204_link *lnk)
{
	ARG_UNUSED(lnk);

	if (reason != JESD204_STATE_OP_REASON_INIT) {
		return JESD204_STATE_CHANGE_DONE;
	}

	return axi_adxcvr_enable(jesd204_dev_priv(jdev)) ?:
	       JESD204_STATE_CHANGE_DONE;
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
	ARG_UNUSED(lnk);

	if (reason != JESD204_STATE_OP_REASON_INIT) {
		return JESD204_STATE_CHANGE_DONE;
	}

	return axi_jesd204_lane_clk_enable(jesd204_dev_priv(jdev)) ?:
	       JESD204_STATE_CHANGE_DONE;
}

/*
 * The authoritative link check: this link's FPGA core reporting DATA.
 *
 * no-OS polls here rather than assuming -- axi_jesd204_rx.c:818-823 retries the
 * status read 20 times at 4 ms, per core, which is what this does now that each
 * core is its own device on its own link.
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

	/*
	 * Poll quietly, then log the outcome once. 20 attempts at 4 ms is no-OS's
	 * budget (axi_jesd204_rx.c:813, :819-823); logging inside the loop would
	 * emit a line per attempt and bury the failure it is waiting on.
	 */
	for (int attempt = 0; attempt < 20 && !is_data; attempt++) {
		k_msleep(4);
		is_data = axi_jesd204_link_is_data(core);
	}

	/* Log this core's state whatever the outcome, then take the verdict. */
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

static struct jesd204_dev rx_jesd_jdev = {
	.name = "rx_jesd",
	.dev_data = &axi_jesd204_jesd204_data,
	.priv = (void *)DEVICE_DT_GET(DT_NODELABEL(rx_jesd)),
};

static struct jesd204_dev tx_jesd_jdev = {
	.name = "tx_jesd",
	.dev_data = &axi_jesd204_jesd204_data,
	.priv = (void *)DEVICE_DT_GET(DT_NODELABEL(tx_jesd)),
};

/* --------------------------------------------------------- TPL datapath --- */

/*
 * TPL verify + DAC re-sync, once both links are running.
 *
 * A per_device callback so it runs once, not once per link: axi_tpl_enable()
 * takes both cores together and there is nothing per-link about it. Being
 * per_device on a non-top device, it runs before any link's per_link work in
 * LINK_RUNNING -- which would be too early -- so it sits in
 * OPT_POST_RUNNING_STAGE instead, the phase after. no-OS has no TPL device in
 * its topology at all (it calls axi_dac_init()/axi_adc_init() after the FSM
 * returns, app.c:454-455); this is the same "after everything" position
 * expressed inside the walk.
 *
 * Warn but do not fail: the links are up and carrying DATA, which is what
 * LINK_RUNNING decided. A TPL complaint is a datapath problem downstream of the
 * link, and failing the phase would report the wrong thing.
 */
static int axi_tpl_fsm_post_running(struct jesd204_dev *jdev,
				    enum jesd204_state_op_reason reason)
{
	ARG_UNUSED(jdev);

	if (reason != JESD204_STATE_OP_REASON_INIT) {
		return JESD204_STATE_CHANGE_DONE;
	}

	if (axi_tpl_enable(DEVICE_DT_GET(DT_NODELABEL(rx_tpl)),
			   DEVICE_DT_GET(DT_NODELABEL(tx_tpl)))) {
		LOG_WRN("TPL post-link verify failed (link is up regardless)");
	}

	return JESD204_STATE_CHANGE_DONE;
}

static const struct jesd204_dev_data axi_tpl_jesd204_data = {
	.max_num_links = 2,
	.state_ops = {
		[JESD204_OP_OPT_POST_RUNNING_STAGE] = {
			.per_device = axi_tpl_fsm_post_running,
			.mode = JESD204_STATE_OP_MODE_PER_DEVICE,
		},
	},
};

static struct jesd204_dev axi_tpl_jdev = {
	.name = "axi-tpl",
	.dev_data = &axi_tpl_jesd204_data,
};

/* ------------------------------------------------------------ topology --- */

/*
 * THE BOARD TOPOLOGY. This array is what a port to another board edits, and in
 * most cases the only thing.
 *
 * Its shape is no-OS's (projects/ad9081/src/app.c:387-441) so that a topology
 * from a no-OS project ports by substituting the device handle:
 *
 *     no-OS:   .jdev = rx_jesd->jdev
 *     here:    .jdev = &rx_jesd_jdev
 *
 * Two rules govern what this array can say, both enforced by
 * jesd204_topology_init():
 *
 *   1. Exactly one row is the top device -- the converter. It is visited LAST in
 *      every forward phase, which is what puts the chip's JESD PLL check after
 *      the GT reset-release. Getting this wrong was a real bug in this port: the
 *      converter was once first, and the PLL was checked before the GT it
 *      depends on had a clock. The link still came up, so no boot log caught it.
 *   2. The top device declares the link set. Its link_ids[] must be the union of
 *      every other row's, and its is_transmit[] gives each link's direction.
 *
 * Row order below is otherwise the hardware dependency order: transceivers out
 * of reset, then the link cores' lane clocks, then the TPL datapath, then (last,
 * by is_top_device) the converter. Within a phase that is the visit order.
 *
 * Not in this array: the HMC7044. It is a Zephyr clock_control driver that has
 * the tree locked and SYSREF running before any of this, and it registers no
 * phase callbacks -- so a row for it would be visited and do nothing. no-OS
 * lists it (app.c:388-393) because there its clock-tree sync happens inside the
 * CLK_SYNC phases and it is the SYSREF provider. If a board ever needs gated
 * SYSREF, that is the row to add, with .is_sysref_provider = true.
 */
static const struct jesd204_topology_dev board_topology_devs[] = {
	{
		.jdev = &tx_adxcvr_jdev,
		.link_ids = { DEFRAMER_LINK0_TX },
		.is_transmit = { true },
		.links_number = 1,
	},
	{
		.jdev = &rx_adxcvr_jdev,
		.link_ids = { FRAMER_LINK0_RX },
		.is_transmit = { false },
		.links_number = 1,
	},
	{
		.jdev = &rx_jesd_jdev,
		.link_ids = { FRAMER_LINK0_RX },
		.is_transmit = { false },
		.links_number = 1,
	},
	{
		.jdev = &tx_jesd_jdev,
		.link_ids = { DEFRAMER_LINK0_TX },
		.is_transmit = { true },
		.links_number = 1,
	},
	{
		.jdev = &axi_tpl_jdev,
		.link_ids = { DEFRAMER_LINK0_TX, FRAMER_LINK0_RX },
		.is_transmit = { true, false },
		.links_number = 2,
	},
	{
		.jdev = &ad9081_jdev,
		.link_ids = { DEFRAMER_LINK0_TX, FRAMER_LINK0_RX },
		.is_transmit = { true, false },
		.links_number = 2,
		.is_top_device = true,
	},
};

static struct jesd204_topology topology;

/*
 * Built once, on first use, from board_topology_devs[]. no-OS calls
 * jesd204_topology_init() from application code before the FSM (app.c:443-444);
 * doing it lazily here keeps jesd204_bringup() a single call for the sample and
 * for the fault-injection suite, which brings the link up more than once.
 */
static int topology_ready(void)
{
	static bool built;
	int ret;

	if (built) {
		return 0;
	}

	ret = jesd204_topology_init(&topology, board_topology_devs,
				    ARRAY_SIZE(board_topology_devs));
	if (ret) {
		LOG_ERR("board topology is malformed (%d)", ret);
		return ret;
	}

	built = true;
	return 0;
}

/*
 * Every device in the topology must be ready before the walk is attempted.
 *
 * device_is_ready(), not a NULL check: DEVICE_DT_GET() resolves at build time
 * and can never be NULL, so the old `chip() == NULL` guard could not fire. This
 * can -- it fails if a node's init() returned an error, which is what the
 * message has always claimed to mean.
 *
 * The list is the devicetree half of the topology. A node given
 * status = "disabled" fails DT_NODE_HAS_STATUS_OKAY() at build time rather than
 * producing a device that is missing at run time.
 */
#define JESD204_PARTICIPANTS(fn)		\
	fn(DT_NODELABEL(ad9081))		\
	fn(DT_NODELABEL(tx_adxcvr))		\
	fn(DT_NODELABEL(rx_adxcvr))		\
	fn(DT_NODELABEL(tx_jesd))		\
	fn(DT_NODELABEL(rx_jesd))		\
	fn(DT_NODELABEL(rx_tpl))		\
	fn(DT_NODELABEL(tx_tpl))

#define JESD204_ASSERT_ENABLED(node_id)						\
	BUILD_ASSERT(DT_NODE_HAS_STATUS_OKAY(node_id),				\
		     "JESD204 topology names a disabled node: "			\
		     DT_NODE_FULL_NAME(node_id));

JESD204_PARTICIPANTS(JESD204_ASSERT_ENABLED)

static int participants_ready(void)
{
#define JESD204_CHECK_READY(node_id)						\
	if (!device_is_ready(DEVICE_DT_GET(node_id))) {				\
		LOG_ERR("JESD204 device not initialised: %s",			\
			DEVICE_DT_GET(node_id)->name);				\
		return -ENODEV;							\
	}

	JESD204_PARTICIPANTS(JESD204_CHECK_READY)
#undef JESD204_CHECK_READY

	return 0;
}

int jesd204_bringup(void)
{
	int failures;
	int ret;

	ret = participants_ready();
	if (ret) {
		return ret;
	}

	ret = topology_ready();
	if (ret) {
		return ret;
	}

	failures = jesd204_fsm_start(&topology, JESD204_LINKS_ALL);
	if (failures < 0) {
		return failures;
	}

	if (failures) {
		LOG_WRN("=== JESD204B link NOT fully up (%d failed step(s)) ===",
			failures);
		return -EIO;
	}

	LOG_INF("=== JESD204B LINK UP (both ends carrying DATA) ===");
	return 0;
}

int jesd204_teardown(void)
{
	int ret;

	ret = participants_ready();
	if (ret) {
		return ret;
	}

	ret = topology_ready();
	if (ret) {
		return ret;
	}

	return jesd204_fsm_stop(&topology, JESD204_LINKS_ALL);
}

int jesd204_bringup_topology_is_valid(void)
{
	struct jesd204_topology scratch;

	/*
	 * Built into a scratch copy rather than reusing the live one: this is a
	 * verdict on the array, and it is called while a link is up.
	 */
	return jesd204_topology_init(&scratch, board_topology_devs,
				     ARRAY_SIZE(board_topology_devs));
}
