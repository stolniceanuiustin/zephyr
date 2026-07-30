/*
 * JESD204B bring-up -- device state tables and topology.
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
 * Phase assignment here, and where it comes from in no-OS:
 *
 *   CLK_SYNC_STAGE1  chip one-shot SYNC (subclass 1). The clock-sync phases are
 *                    where no-OS's HMC7044 driver does its tree sync
 *                    (hmc7044.c:1447-1458); this chip's SYNC belongs in the same
 *                    window.
 *   CLK_SYNC_STAGE2  chip NCO sync.
 *   CLOCKS_ENABLE    GT reset-release, then FPGA lane clocks -- in that order,
 *                    matching no-OS jesd204_clk_enable() (jesd204_clk.c:44-66),
 *                    which drives adxcvr before the link cores. Chip JESD PLL
 *                    lock and 204C calibration also sit here.
 *   LINK_ENABLE      chip JRX deframer enable, after clearing the JRX
 *                    transport-layer buffer protection.
 *   LINK_RUNNING     poll for DATA on both FPGA cores, read chip link status,
 *                    then verify the TPL datapath.
 *
 * Ordering *within* a phase is the device order in jesd204_topology below.
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

#include "adi_ad9081.h"
#include "adi_ad9081_hal.h"
#include "adi_ad9081_bf_ad9081.h"

/* Chip-side link select: the deframer (JRX) link 0. */
#define AD9081_JRX_LINK AD9081_LINK_0

/*
 * Every callback returns JESD204_STATE_CHANGE_DONE on success and a negative
 * errno on failure, as in no-OS. On UNINIT most of them have nothing to undo:
 * the phase is a no-op on the way down and says so by returning DONE.
 */
#define JESD204_STATE_CHANGE_DONE 1

/* ---------------------------------------------------------------- AD9082 --- */

static adi_ad9081_device_t *chip(void)
{
	return ad9081_get_device();
}

static int ad9081_fsm_oneshot_sync(struct jesd204_dev *jdev,
				   enum jesd204_state_op_reason reason)
{
	ARG_UNUSED(jdev);

	if (reason != JESD204_STATE_OP_REASON_INIT) {
		return JESD204_STATE_CHANGE_DONE;
	}

	if (adi_ad9081_jesd_oneshot_sync(chip(), JESD_SUBCLASS_1)) {
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

	if (adi_ad9081_device_nco_sync_post(chip())) {
		return -EIO;
	}
	return JESD204_STATE_CHANGE_DONE;
}

static int ad9081_fsm_clks_enable(struct jesd204_dev *jdev,
				  enum jesd204_state_op_reason reason,
				  struct jesd204_link *lnk)
{
	uint8_t pll_status = 0;

	ARG_UNUSED(jdev);
	ARG_UNUSED(lnk);

	if (reason != JESD204_STATE_OP_REASON_INIT) {
		return JESD204_STATE_CHANGE_DONE;
	}

	if (adi_ad9081_jesd_pll_lock_status_get(chip(), &pll_status)) {
		LOG_ERR("chip JESD PLL status read failed");
		return -EIO;
	}
	LOG_INF("chip JESD PLL status = 0x%x", pll_status);
	if (!pll_status) {
		return -EIO;
	}

	/* 204C background calibration on the deframer (force reset, no boost). */
	if (adi_ad9081_jesd_rx_calibrate_204c(chip(), 1, 0, 1)) {
		LOG_WRN("chip JRX 204C calibration failed");
		return -EIO;
	}

	return JESD204_STATE_CHANGE_DONE;
}

static int ad9081_fsm_link_enable(struct jesd204_dev *jdev,
				  enum jesd204_state_op_reason reason,
				  struct jesd204_link *lnk)
{
	ARG_UNUSED(jdev);
	ARG_UNUSED(lnk);

	if (reason == JESD204_STATE_OP_REASON_UNINIT) {
		if (adi_ad9081_jesd_rx_link_enable_set(chip(), AD9081_JRX_LINK,
						       0)) {
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
	if (adi_ad9081_hal_bf_set(chip(), REG_JRX_TPL_1_ADDR,
				  BF_JRX_TPL_BUF_PROTECT_EN_INFO, 0)) {
		LOG_WRN("chip JRX buf-protect clear failed");
		return -EIO;
	}

	/*
	 * Enable the chip's JRX deframer. The JTX framer runs once its digital
	 * reset is released (done inside startup_rx) and SYSREF arrives.
	 */
	if (adi_ad9081_jesd_rx_link_enable_set(chip(), AD9081_JRX_LINK, 1)) {
		return -EIO;
	}

	/* Let the link negotiate CGS -> ILAS -> DATA before anyone reads status. */
	k_msleep(10);

	return JESD204_STATE_CHANGE_DONE;
}

static int ad9081_fsm_link_running(struct jesd204_dev *jdev,
				   enum jesd204_state_op_reason reason,
				   struct jesd204_link *lnk)
{
	uint16_t tx_status = 0;
	uint16_t rx_status = 0;

	ARG_UNUSED(jdev);
	ARG_UNUSED(lnk);

	if (reason != JESD204_STATE_OP_REASON_INIT) {
		return JESD204_STATE_CHANGE_DONE;
	}

	if (adi_ad9081_jesd_tx_link_status_get(chip(), AD9081_LINK_0,
					       &tx_status)) {
		LOG_WRN("chip jesd_tx_link_status_get failed");
	} else {
		LOG_INF("chip JTX (framer)   link status = 0x%04x", tx_status);
	}

	if (adi_ad9081_jesd_rx_link_status_get(chip(), AD9081_JRX_LINK,
					       &rx_status)) {
		LOG_WRN("chip jesd_rx_link_status_get failed");
	} else {
		LOG_INF("chip JRX (deframer) link status = 0x%04x", rx_status);
	}

	/*
	 * Reported, not gated. The chip status words are diagnostic here -- the
	 * authoritative DATA check is the FPGA link cores' own, in
	 * axi_jesd204_fsm_link_running(). no-OS treats them the same way
	 * (app.c:450-451 prints them and acts on neither).
	 */
	return JESD204_STATE_CHANGE_DONE;
}

static const struct jesd204_dev_data ad9081_jesd204_data = {
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
 * GT reset-release. Each direction is released independently so a TX failure
 * does not hide the state of RX.
 *
 * This runs before the link cores' lane-clock enable, which is the order no-OS
 * uses in jesd204_clk_enable() (jesd204_clk.c:48-64): adxcvr first, then
 * jesd204_rx, then jesd204_tx.
 */
static int adxcvr_fsm_clks_enable(struct jesd204_dev *jdev,
				  enum jesd204_state_op_reason reason,
				  struct jesd204_link *lnk)
{
	int ret;

	ARG_UNUSED(jdev);
	ARG_UNUSED(lnk);

	if (reason != JESD204_STATE_OP_REASON_INIT) {
		return JESD204_STATE_CHANGE_DONE;
	}

	ret = axi_adxcvr_tx_enable();
	if (ret) {
		return ret;
	}

	ret = axi_adxcvr_rx_enable();
	if (ret) {
		return ret;
	}

	return JESD204_STATE_CHANGE_DONE;
}

static const struct jesd204_dev_data adxcvr_jesd204_data = {
	.state_ops = {
		[JESD204_OP_CLOCKS_ENABLE] = {
			.per_link = adxcvr_fsm_clks_enable,
		},
	},
};

static struct jesd204_dev adxcvr_jdev = {
	.name = "adxcvr",
	.dev_data = &adxcvr_jesd204_data,
};

/* ---------------------------------------------------- FPGA JESD204 cores --- */

static int axi_jesd204_fsm_clks_enable(struct jesd204_dev *jdev,
				       enum jesd204_state_op_reason reason,
				       struct jesd204_link *lnk)
{
	int ret;

	ARG_UNUSED(jdev);
	ARG_UNUSED(lnk);

	if (reason != JESD204_STATE_OP_REASON_INIT) {
		return JESD204_STATE_CHANGE_DONE;
	}

	ret = axi_jesd204_rx_lane_clk_enable();
	if (ret) {
		return ret;
	}

	ret = axi_jesd204_tx_lane_clk_enable();
	if (ret) {
		return ret;
	}

	return JESD204_STATE_CHANGE_DONE;
}

/*
 * The authoritative link check: both FPGA cores reporting DATA.
 *
 * no-OS polls here rather than assuming -- axi_jesd204_rx.c:818-823 retries the
 * status read 20 times at 4 ms. axi_jesd204_status_read() logs and evaluates
 * both ends in one call, so this retries around it on the same budget.
 */
static int axi_jesd204_fsm_link_running(struct jesd204_dev *jdev,
					enum jesd204_state_op_reason reason,
					struct jesd204_link *lnk)
{
	bool is_data = false;

	ARG_UNUSED(jdev);
	ARG_UNUSED(lnk);

	if (reason != JESD204_STATE_OP_REASON_INIT) {
		return JESD204_STATE_CHANGE_DONE;
	}

	/*
	 * Poll quietly, then log the outcome once. 20 attempts at 4 ms is no-OS's
	 * budget (axi_jesd204_rx.c:813, :819-823); logging inside the loop would
	 * emit two lines per attempt and bury the failure it is waiting on.
	 */
	for (int attempt = 0; attempt < 20 && !is_data; attempt++) {
		k_msleep(4);
		is_data = axi_jesd204_link_is_data();
	}

	/* Log both ends' state regardless of outcome, and take its verdict. */
	if (axi_jesd204_status_read()) {
		return -EIO;
	}

	/*
	 * TPL datapath verify + DAC re-sync, now that the link clocks run.
	 *
	 * Warn but do not fail the phase: the link itself is up and carrying
	 * DATA, which is what this phase decides. A TPL status complaint is a
	 * datapath problem downstream of the link, and treating it as a link
	 * failure would report the wrong thing. This matches the behaviour before
	 * the FSM was table-driven.
	 */
	if (axi_tpl_enable()) {
		LOG_WRN("TPL post-link verify failed (link is up regardless)");
	}

	return JESD204_STATE_CHANGE_DONE;
}

static const struct jesd204_dev_data axi_jesd204_jesd204_data = {
	.state_ops = {
		[JESD204_OP_CLOCKS_ENABLE] = {
			.per_link = axi_jesd204_fsm_clks_enable,
		},
		[JESD204_OP_LINK_RUNNING] = {
			.per_link = axi_jesd204_fsm_link_running,
		},
	},
};

static struct jesd204_dev axi_jesd204_jdev = {
	.name = "axi-jesd204",
	.dev_data = &axi_jesd204_jesd204_data,
};

/* ---------------------------------------------------------------- driver --- */

/*
 * Device visit order within each phase.
 *
 * This order is chosen to reproduce exactly the step order that was verified
 * working on this board, which the phase tables alone do not determine. Inside
 * CLOCKS_ENABLE it yields: GT TX/RX reset-release, then the chip's JESD PLL
 * check and 204C calibration, then the FPGA lane clocks. adxcvr before
 * axi-jesd204 is additionally what no-OS does in jesd204_clk_enable()
 * (jesd204_clk.c:48-64).
 *
 * The CLK_SYNC phases are unaffected -- ad9082 is the only device registered
 * for them. In LINK_RUNNING, ad9082 precedes axi-jesd204, which only affects
 * log order: the chip's status read is diagnostic and the FPGA core's is the
 * one that gates.
 */
static struct jesd204_topology topology = {
	.devs = {
		&adxcvr_jdev,
		&ad9081_jdev,
		&axi_jesd204_jdev,
	},
	.devs_number = 3,
	.link = {
		.link_id = 0,
		.is_transmit = false,
	},
};

int jesd204_bringup(void)
{
	int failures;

	if (ad9081_get_device() == NULL) {
		LOG_ERR("AD9081 device not initialised");
		return -ENODEV;
	}

	failures = jesd204_fsm_start(&topology);

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
	if (ad9081_get_device() == NULL) {
		return -ENODEV;
	}
	return jesd204_fsm_stop(&topology);
}
