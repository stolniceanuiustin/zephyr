/*
 * AD9081/AD9082 + HMC7044 JESD204B bring-up (application / FSM).
 *
 * This app grows bottom-up into the full JESD204B link, mirroring how no-OS
 * keeps the project FSM in application source. Current milestones:
 *
 *   [x] SPI0 -> AD9081/AD9082: read PROD_ID (0x9081/0x9082)
 *   [x] SPI1 -> HMC7044: scratchpad read/write check
 *   [x] HMC7044 clock + SYSREF configuration (PLL1/PLL2 lock)
 *   [x] AXI plane alive: JESD204 RX/TX core identity (MAGIC/version/lanes)
 *   [x] AXI adxcvr: GT transceiver clock-mux config (DEVICE_INIT phase)
 *   [x] AXI jesd204 rx/tx link cores config (LINK_INIT phase)
 *   [x] AXI TPL transport cores config (datapath format / data-source select)
 *   [x] AXI DMAC engines bound (RX S2MM capture / TX MM2S playback)
 *   [x] AD9082 datapath config (CLK PLL, TX/RX NCOs, on-chip JESD framer/deframer)
 *   [x] JESD204 bring-up FSM: reset GT + link enable + SYSREF, then status
 *
 * IMPORTANT: JESD204 is a negotiated multi-device link -- the transceiver, link
 * cores, SYSREF and the AD9082 cannot be brought up or status-checked in
 * isolation. Each block only *configures* here; the actual activation and the
 * single meaningful status check happen together in the bring-up FSM at the end
 * (mirroring the no-OS/Linux jesd204 state machine). So blocks are added as
 * "configure-only" until enough exist to run the FSM.
 *   [ ] link bring-up: CGS -> ILAS -> Data
 *   [ ] DMA capture (axi_dmac)
 *
 * Bring-up order is topology order: clock -> PHY -> link -> transport. For now
 * we just prove both SPI control planes are alive before building on them.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

#include "ad9081.h"
#include "hmc7044.h"
#include "axi_jesd.h"
#include "axi_adxcvr.h"
#include "axi_jesd204.h"
#include "axi_tpl.h"
#include "jesd_fsm.h"
#include "jesd_test.h"

int main(void)
{
	uint16_t prod_id;
	int ret;

	LOG_INF("=== AD9081/HMC7044 bring-up ===");

	/* Clock chip first (topology order). */
	ret = hmc7044_probe();
	if (ret) {
		LOG_ERR("HMC7044 probe failed (%d)", ret);
		return ret;
	}
	LOG_INF("SUCCESS: HMC7044 scratchpad read/write confirmed");

	/* Program the HMC7044 clock tree + SYSREF (PLL1/PLL2 lock). */
	ret = hmc7044_setup_clocks();
	if (ret) {
		LOG_ERR("HMC7044 clock setup failed (%d)", ret);
		return ret;
	}
	LOG_INF("SUCCESS: HMC7044 clock tree configured");

	/* MxFE. */
	ret = ad9081_probe(&prod_id);
	if (ret) {
		LOG_ERR("AD9081 probe failed (%d)", ret);
		return ret;
	}
	LOG_INF("SUCCESS: AD90%02x detected over SPI", prod_id & 0xFF);

	/*
	 * Configure the MxFE datapath through the ADI API lib (CLK PLL, TX interp
	 * + DAC NCOs + JRX deframer, RX decim + ADC NCOs + JTX framer). The chip
	 * JESD links are configured but not enabled -- the FSM enables them with
	 * the FPGA cores.
	 */
	ret = ad9081_setup_datapath();
	if (ret) {
		LOG_ERR("AD9081 datapath setup failed (%d)", ret);
		return ret;
	}
	LOG_INF("SUCCESS: AD9082 datapath configured (chip JESD links ready)");

	/* Prove the PL AXI plane is alive before link bring-up. */
	ret = axi_jesd_probe();
	if (ret) {
		LOG_ERR("AXI JESD204 probe failed (%d)", ret);
		return ret;
	}
	LOG_INF("SUCCESS: PL AXI plane alive, JESD204 cores identified");

	/*
	 * GT transceiver config only (clock-mux select, held in reset). The
	 * reset-release + status poll is deferred to the JESD204 bring-up FSM,
	 * because GT-ready is only meaningful once the link layer and SYSREF are
	 * up around it -- gating on it standalone times out.
	 */
	ret = axi_adxcvr_configure();
	if (ret) {
		LOG_ERR("AXI adxcvr (GT) config failed (%d)", ret);
		return ret;
	}
	LOG_INF("SUCCESS: GT transceivers configured (TX QPLL0 / RX CPLL)");

	/* JESD204 link cores: program link geometry + ILAS, held disabled. */
	ret = axi_jesd204_configure();
	if (ret) {
		LOG_ERR("AXI jesd204 link config failed (%d)", ret);
		return ret;
	}
	LOG_INF("SUCCESS: JESD204 link cores configured (M8/L4/F4/K32)");

	/* TPL transport cores: datapath sample-format (RX) + data-source (TX). */
	ret = axi_tpl_configure();
	if (ret) {
		LOG_ERR("AXI TPL transport config failed (%d)", ret);
		return ret;
	}
	LOG_INF("SUCCESS: TPL transport cores configured (8 converters)");

	/*
	 * Datapath DMA engines. The ADI AXI DMAC driver binds from the overlay
	 * and auto-probes each core (direction/width/version) at POST_KERNEL --
	 * we just confirm both are ready. Actual capture/playback (dma_config +
	 * dma_start) is driven later, once samples flow through the live link.
	 */
	{
		const struct device *rx_dma = DEVICE_DT_GET(DT_NODELABEL(rx_dmac));
		const struct device *tx_dma = DEVICE_DT_GET(DT_NODELABEL(tx_dmac));

		if (!device_is_ready(rx_dma) || !device_is_ready(tx_dma)) {
			LOG_ERR("AXI DMAC engines not ready (rx=%d tx=%d)",
				device_is_ready(rx_dma), device_is_ready(tx_dma));
			return -ENODEV;
		}
		LOG_INF("SUCCESS: AXI DMAC engines ready (%s, %s)",
			rx_dma->name, tx_dma->name);
	}

	LOG_INF("=== all blocks configured, running JESD204 bring-up ===");

	/*
	 * Bring the link up: activate the transceiver, link cores and the chip's
	 * framer/deframer together, then read link status. A failure here is
	 * expected until every knob is right -- the value is seeing *where* the
	 * link stalls (CGS/ILAS/DATA), not a clean pass.
	 */
	ret = jesd204_bringup();
	if (ret) {
		LOG_WRN("JESD204 bring-up did not reach DATA (%d)", ret);
		return ret;
	}
	LOG_INF("SUCCESS: JESD204B link up");

	/*
	 * Rung 1 datapath validation: now that the link carries DATA, prove the
	 * receive serial path is actually bit-error-free with a PN test (chip
	 * ADC PN test mode -> FPGA RX TPL PN monitor). No DMA/analog yet.
	 */
	ret = jesd_test_rx_pn();
	if (ret) {
		LOG_WRN("Rung 1 receive-path PN test failed (%d)", ret);
		return ret;
	}
	LOG_INF("SUCCESS: receive datapath verified bit-error-free (PN)");
	return 0;
}
