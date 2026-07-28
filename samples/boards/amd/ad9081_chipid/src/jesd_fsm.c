/*
 * JESD204B bring-up sequence.
 *
 * Every block was *configured* separately (adxcvr, jesd204 link cores, TPL, and
 * the AD9082 datapath), each holding its link/reset dark. A JESD204 link only
 * comes alive when the FPGA transceiver + link cores and the AD9082's framer/
 * deframer are activated together, in a fixed phase order, with SYSREF crossing
 * both ends. no-OS/Linux express this as a jesd204 state machine that steps all
 * devices through each phase in lock-step. We don't pull in that framework; this
 * file reproduces the same phase order by hand for our single chip + single FPGA
 * link pair.
 *
 * Phase order (mirrors the no-OS FSM ops for the ad9081 + axi_jesd devices):
 *   LINK_INIT      - link params already programmed at configure time.
 *   SETUP/SYNC     - chip one-shot SYNC + NCO sync (subclass 1 uses SYSREF).
 *   CLOCKS_ENABLE  - GT reset-release (both dirs); chip JESD PLL lock check;
 *                    204C background calibration on the chip's JRX; enable the
 *                    FPGA lane clocks (link cores).
 *   LINK_ENABLE    - chip JRX deframer enable; SYSREF is free-running from the
 *                    HMC7044 (continuous, ch3/ch13), so both ends see it.
 *   LINK_RUNNING   - read link status on both the FPGA cores and the chip.
 *
 * SYSREF note: the HMC7044 emits DEV_SYSREF / FPGA_SYSREF continuously at
 * 1.953 MHz (see hmc7044.c), so there is no explicit "pulse SYSREF" step here --
 * subclass-1 alignment happens against that free-running SYSREF as each end is
 * enabled.
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

#include "adi_ad9081.h"

/* Chip-side link select: the deframer (JRX) link 0. */
#define AD9081_JRX_LINK AD9081_LINK_0

/*
 * Bring-up is deliberately best-effort: a JESD204 link stalls at the *first*
 * broken stage, but which stage that is, is exactly what we're trying to learn.
 * So instead of aborting on the first failure (which blinds us to everything
 * downstream), each step records its result and we press on. The chip SPI ops
 * and the FPGA AXI status reads are all safe to attempt regardless of GT state.
 * A one-line-per-step summary at the end shows the whole chain in one boot.
 */
struct step_result {
	const char *name;
	int rc;
};

#define MAX_STEPS 12

static void record(struct step_result *steps, int *n, const char *name, int rc)
{
	if (*n < MAX_STEPS) {
		steps[*n].name = name;
		steps[*n].rc = rc;
		(*n)++;
	}
	if (rc) {
		LOG_WRN("step %-22s : FAIL (%d)", name, rc);
	} else {
		LOG_INF("step %-22s : ok", name);
	}
}

int jesd204_bringup(void)
{
	adi_ad9081_device_t *dev = ad9081_get_device();
	struct step_result steps[MAX_STEPS];
	uint8_t jesd_pll_status = 0;
	uint16_t rx_link_status = 0;
	uint16_t tx_link_status = 0;
	int nsteps = 0;
	int fpga_ok;
	int i;
	int32_t err;

	if (dev == NULL) {
		LOG_ERR("AD9081 device not initialised");
		return -ENODEV;
	}

	LOG_INF("--- JESD204B bring-up sequence (best-effort, full chain) ---");

	/*
	 * SETUP / SYNC. Subclass 1: one-shot SYNC then NCO sync. The chip aligns
	 * to the continuous SYSREF from the HMC7044.
	 */
	err = adi_ad9081_jesd_oneshot_sync(dev, JESD_SUBCLASS_1);
	record(steps, &nsteps, "chip oneshot_sync", err ? -EIO : 0);

	err = adi_ad9081_device_nco_sync_post(dev);
	record(steps, &nsteps, "chip nco_sync_post", err ? -EIO : 0);

	/*
	 * CLOCKS_ENABLE. Release the GT resets (each direction independently so a
	 * TX failure doesn't hide RX), check the chip's JESD PLL and run 204C
	 * calibration on the deframer, then enable the FPGA link-core lane clocks.
	 */
	record(steps, &nsteps, "GT TX reset-release", axi_adxcvr_tx_enable());
	record(steps, &nsteps, "GT RX reset-release", axi_adxcvr_rx_enable());

	err = adi_ad9081_jesd_pll_lock_status_get(dev, &jesd_pll_status);
	if (err != API_CMS_ERROR_OK) {
		record(steps, &nsteps, "chip JESD PLL read", -EIO);
	} else {
		LOG_INF("chip JESD PLL status = 0x%x", jesd_pll_status);
		record(steps, &nsteps, "chip JESD PLL lock",
		       jesd_pll_status ? 0 : -EIO);
	}

	/* 204C background calibration on the deframer (force reset, no boost). */
	err = adi_ad9081_jesd_rx_calibrate_204c(dev, 1, 0, 1);
	record(steps, &nsteps, "chip JRX 204C cal", err ? -EIO : 0);

	/* FPGA link cores: release the framer/deframer lane clocks. */
	record(steps, &nsteps, "FPGA rx lane clk", axi_jesd204_rx_lane_clk_enable());
	record(steps, &nsteps, "FPGA tx lane clk", axi_jesd204_tx_lane_clk_enable());

	/*
	 * LINK_ENABLE. Enable the chip's JRX deframer; the JTX framer runs once
	 * its digital reset is released (done inside startup_rx) and SYSREF
	 * arrives. Give the link a moment to negotiate CGS -> ILAS -> DATA.
	 */
	err = adi_ad9081_jesd_rx_link_enable_set(dev, AD9081_JRX_LINK, 1);
	record(steps, &nsteps, "chip JRX enable", err ? -EIO : 0);

	k_msleep(10);

	/*
	 * LINK_RUNNING. Read link status on both ends. This is the meaningful
	 * status check -- everything before was configuration/activation.
	 */
	LOG_INF("--- link status ---");

	err = adi_ad9081_jesd_tx_link_status_get(dev, AD9081_LINK_0,
						 &tx_link_status);
	if (err == API_CMS_ERROR_OK) {
		LOG_INF("chip JTX (framer)   link status = 0x%04x", tx_link_status);
	} else {
		LOG_WRN("chip jesd_tx_link_status_get failed (%d)", err);
	}

	err = adi_ad9081_jesd_rx_link_status_get(dev, AD9081_JRX_LINK,
						 &rx_link_status);
	if (err == API_CMS_ERROR_OK) {
		LOG_INF("chip JRX (deframer) link status = 0x%04x", rx_link_status);
	} else {
		LOG_WRN("chip jesd_rx_link_status_get failed (%d)", err);
	}

	/* FPGA-side link state (CGS/ILAS/DATA). */
	fpga_ok = axi_jesd204_status_read();
	record(steps, &nsteps, "FPGA link DATA", fpga_ok);

	/* TPL datapath verify + DAC re-sync now that the link clocks run. */
	if (fpga_ok == 0) {
		int tpl = axi_tpl_enable();

		if (tpl) {
			LOG_WRN("TPL post-link verify failed (%d)", tpl);
		}
	}

	/* One-shot chain summary: the first FAIL is where the link stalls. */
	LOG_INF("=== JESD204 bring-up summary ===");
	for (i = 0; i < nsteps; i++) {
		LOG_INF("  [%s] %s", steps[i].rc ? "FAIL" : " ok ", steps[i].name);
	}

	if (fpga_ok == 0) {
		LOG_INF("=== JESD204B LINK UP (both ends carrying DATA) ===");
	} else {
		LOG_WRN("=== JESD204B link NOT fully up (see summary above) ===");
	}
	return fpga_ok;
}
