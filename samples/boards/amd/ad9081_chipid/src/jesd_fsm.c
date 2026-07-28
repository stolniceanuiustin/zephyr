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

int jesd204_bringup(void)
{
	adi_ad9081_device_t *dev = ad9081_get_device();
	uint8_t jesd_pll_status;
	uint16_t rx_link_status;
	uint16_t tx_link_status;
	int32_t err;
	int ret;

	if (dev == NULL) {
		LOG_ERR("AD9081 device not initialised");
		return -ENODEV;
	}

	LOG_INF("--- JESD204B bring-up sequence ---");

	/*
	 * SETUP / SYNC. Subclass 1: one-shot SYNC then NCO sync. The chip aligns
	 * to the continuous SYSREF from the HMC7044.
	 */
	err = adi_ad9081_jesd_oneshot_sync(dev, JESD_SUBCLASS_1);
	if (err != API_CMS_ERROR_OK) {
		LOG_ERR("chip oneshot_sync failed (%d)", err);
		return -EIO;
	}

	err = adi_ad9081_device_nco_sync_post(dev);
	if (err != API_CMS_ERROR_OK) {
		LOG_ERR("chip nco_sync_post failed (%d)", err);
		return -EIO;
	}
	LOG_INF("phase SYNC: chip one-shot SYNC + NCO sync done");

	/*
	 * CLOCKS_ENABLE. Release the GT resets (both directions) so the lanes are
	 * clocked, then check the chip's JESD PLL and run 204C calibration on the
	 * deframer, and finally enable the FPGA link-core lane clocks.
	 */
	ret = axi_adxcvr_enable();
	if (ret) {
		LOG_ERR("GT transceiver enable failed (%d)", ret);
		return ret;
	}
	LOG_INF("phase CLOCKS_ENABLE: GT transceivers ready");

	err = adi_ad9081_jesd_pll_lock_status_get(dev, &jesd_pll_status);
	if (err != API_CMS_ERROR_OK) {
		LOG_ERR("chip jesd_pll_lock_status_get failed (%d)", err);
		return -EIO;
	}
	if (!jesd_pll_status) {
		LOG_ERR("chip JESD PLL not locked (status=0x%x)",
			jesd_pll_status);
		return -EIO;
	}
	LOG_INF("phase CLOCKS_ENABLE: chip JESD PLL locked (0x%x)",
		jesd_pll_status);

	/* 204C background calibration on the deframer (force reset, no boost). */
	err = adi_ad9081_jesd_rx_calibrate_204c(dev, 1, 0, 1);
	if (err != API_CMS_ERROR_OK) {
		LOG_ERR("chip jesd_rx_calibrate_204c failed (%d)", err);
		return -EIO;
	}
	LOG_INF("phase CLOCKS_ENABLE: chip JRX 204C calibration done");

	/* FPGA link cores: release the framer/deframer lane clocks. */
	ret = axi_jesd204_rx_lane_clk_enable();
	if (ret) {
		LOG_ERR("FPGA jesd204-rx lane clk enable failed (%d)", ret);
		return ret;
	}
	ret = axi_jesd204_tx_lane_clk_enable();
	if (ret) {
		LOG_ERR("FPGA jesd204-tx lane clk enable failed (%d)", ret);
		return ret;
	}
	LOG_INF("phase CLOCKS_ENABLE: FPGA link cores enabled");

	/*
	 * LINK_ENABLE. Enable the chip's JRX deframer; the JTX framer runs once
	 * its digital reset is released (done inside startup_rx) and SYSREF
	 * arrives. Give the link a moment to negotiate CGS -> ILAS -> DATA.
	 */
	err = adi_ad9081_jesd_rx_link_enable_set(dev, AD9081_JRX_LINK, 1);
	if (err != API_CMS_ERROR_OK) {
		LOG_ERR("chip jesd_rx_link_enable_set failed (%d)", err);
		return -EIO;
	}
	LOG_INF("phase LINK_ENABLE: chip JRX deframer enabled");

	k_msleep(10);

	/*
	 * LINK_RUNNING. Read link status on both ends. This is the single
	 * meaningful status check -- everything before was configuration.
	 */
	LOG_INF("phase LINK_RUNNING: reading link status");

	err = adi_ad9081_jesd_tx_link_status_get(dev, AD9081_LINK_0,
						 &tx_link_status);
	if (err == API_CMS_ERROR_OK) {
		LOG_INF("chip JTX (framer) link status = 0x%04x", tx_link_status);
	} else {
		LOG_WRN("chip jesd_tx_link_status_get failed (%d)", err);
	}

	err = adi_ad9081_jesd_rx_link_status_get(dev, AD9081_JRX_LINK,
						 &rx_link_status);
	if (err == API_CMS_ERROR_OK) {
		LOG_INF("chip JRX (deframer) link status = 0x%04x",
			rx_link_status);
	} else {
		LOG_WRN("chip jesd_rx_link_status_get failed (%d)", err);
	}

	/* FPGA-side link state (CGS/ILAS/DATA). */
	ret = axi_jesd204_status_read();

	/* TPL datapath verify + DAC re-sync now that the link clocks run. */
	if (ret == 0) {
		int tpl = axi_tpl_enable();

		if (tpl) {
			LOG_WRN("TPL post-link verify failed (%d)", tpl);
		}
	}

	if (ret == 0) {
		LOG_INF("=== JESD204B LINK UP (both ends carrying DATA) ===");
	} else {
		LOG_WRN("=== JESD204B link NOT fully up (see status above) ===");
	}
	return ret;
}
