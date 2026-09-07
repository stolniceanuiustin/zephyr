/*
 * Copyright (c) 2026 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * DAQ2 JESD204 bring-up -- Phase 1b: enable the link.
 *
 * The AD9523 clock generator and the AD9144 (TX DAC) / AD9680 (RX ADC) come up
 * over SPI at POST_KERNEL init. This orchestrates the FPGA-side datapath the
 * chips' PLLs depend on: configure the transceivers (GT/QPLL/CPLL), configure
 * the JESD204 link cores, then hand off to the generic FSM to *enable* the link
 * in the correct phase order -- GT reset-release, lane clocks, chip PLL re-check,
 * link status.
 *
 * The configure/enable split matches the AD9081 sample: main() runs the configure
 * path, then daq2_jesd204_bringup() (daq2_bringup.c) walks the enable phases. The
 * transport (TPL) and DMA are later phases.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>

#include <zephyr/drivers/misc/jesd204/axi_adxcvr.h>
#include <zephyr/drivers/misc/jesd204/axi_jesd204.h>
#include "daq2_bringup.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(daq2, LOG_LEVEL_INF);

#define TX_ADXCVR DEVICE_DT_GET(DT_NODELABEL(tx_adxcvr))
#define RX_ADXCVR DEVICE_DT_GET(DT_NODELABEL(rx_adxcvr))
#define TX_JESD   DEVICE_DT_GET(DT_NODELABEL(tx_jesd))
#define RX_JESD   DEVICE_DT_GET(DT_NODELABEL(rx_jesd))

int main(void)
{
	int ret;

	LOG_INF("DAQ2 JESD204 bring-up (Phase 1b: enable the link)");

	/*
	 * Step 1: transceivers. Brings up the GT and its PLL (TX QPLL0 / RX
	 * CPLL); axi_adxcvr_configure() logs each PLL's lock. TX before RX, the
	 * order the AD9081 sample uses.
	 */
	ret = axi_adxcvr_configure(TX_ADXCVR);
	if (ret == 0) {
		ret = axi_adxcvr_configure(RX_ADXCVR);
	}
	if (ret) {
		LOG_ERR("AXI adxcvr (GT) config failed (%d)", ret);
		return ret;
	}
	LOG_INF("SUCCESS: GT transceivers configured (TX QPLL0 / RX CPLL)");

	/*
	 * Step 2: JESD204 link cores. Programs link geometry + ILAS, held
	 * disabled. TX before RX, as in the AD9081 sample.
	 */
	ret = axi_jesd204_configure(TX_JESD);
	if (ret == 0) {
		ret = axi_jesd204_configure(RX_JESD);
	}
	if (ret) {
		LOG_ERR("AXI jesd204 link config failed (%d)", ret);
		return ret;
	}
	LOG_INF("SUCCESS: JESD204 link cores configured (M%d/L%d/F%d/K%d)",
		DT_PROP(DT_NODELABEL(tx_jesd), adi_converters_per_device),
		DT_PROP(DT_NODELABEL(tx_jesd), adi_lanes_per_device),
		DT_PROP(DT_NODELABEL(tx_jesd), adi_octets_per_frame),
		DT_PROP(DT_NODELABEL(tx_jesd), adi_frames_per_multiframe));

	/*
	 * Step 3: enable the link via the generic FSM. This releases the GT
	 * reset, enables the FPGA lane clocks, re-reads each chip PLL (now that
	 * the datapath is armed), and polls each link core for DATA -- in the
	 * phase order the Phase 1 linear sequence got wrong. Non-fatal:
	 * daq2_jesd204_bringup() logs how far each link advanced regardless.
	 */
	ret = daq2_jesd204_bringup();
	if (ret) {
		LOG_WRN("DAQ2 JESD204 link bring-up incomplete (%d)", ret);
	} else {
		LOG_INF("SUCCESS: DAQ2 JESD204 links up (both ends carrying DATA)");
	}

	/*
	 * Step 4: per-lane RX watchdog snapshot. Best-effort diagnostic on top of
	 * the status the FSM's LINK_RUNNING phase already read.
	 */
	(void)axi_jesd204_rx_watchdog(RX_JESD);

	LOG_INF("SUCCESS: DAQ2 Phase 1b bring-up complete");

	return 0;
}
