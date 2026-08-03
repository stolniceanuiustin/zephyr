/*
 * AD9081/AD9082 + HMC7044 JESD204B bring-up.
 *
 * A Zephyr counterpart to the no-OS ad9081 example
 * (no-OS/projects/ad9081/src/app.c), profile zcu102_ad9081_m8_l4: same board,
 * same M8/L4/F4/K32/S1/NP16 geometry, TX link-mode 9, RX link-mode 10, 8B10B
 * subclass 1.
 *
 * The sequence below is app.c's, step for step:
 *
 *   app_clock_init()            -> the adi,hmc7044 clock_control
 *                                  driver's own init()         (app.c:343)
 *   app_jesd_init()             -> axi_adxcvr_configure() +
 *                                  axi_jesd204_configure()     (app.c:347)
 *   ad9081_init()               -> ad9081_setup_datapath()     (app.c:367)
 *   jesd204_fsm_start()         -> jesd204_bringup()           (app.c:446)
 *   axi_jesd204_rx_watchdog()   -> axi_jesd204_rx_watchdog()   (app.c:448)
 *   axi_jesd204_{tx,rx}_status_read() -> axi_jesd204_status_read()
 *                                                              (app.c:450-451)
 *   axi_dac_init()/axi_adc_init() -> axi_tpl_configure() +
 *                                    axi_tpl_tx_dds()          (app.c:453-454)
 *
 * What this delivers at the DAC
 * -----------------------------
 * The same thing no-OS delivers: an FPGA-generated tone from the TX transport
 * core's DDS. no-OS passes `.channels = NULL` to axi_dac_init(), which makes
 * axi_dac_data_setup() take its else branch (axi_dac_core.c:1227-1238) and write
 * DATA_SELECT=0 (DATA_SEL_DDS) to every converter at 3 MHz and 0.05 full scale.
 *
 * So the DMA engines and the axi_data_offload cores are deliberately absent here:
 * with DDS selected the transport core's dac_enable is never asserted
 * (ad_ip_jesd204_tpl_dac_channel.v:144), so they are not in the datapath. The
 * reference design does not stream from DDR -- in its default build (IIOD=n) it
 * starts no DMA transfer in either direction, and axi_dmac_init() only detects
 * capabilities (axi_dmac.c:326-350). See COMPARE_noos_vs_zephyr.md.
 *
 * Note on order: no-OS configures the transport cores *after* the link reaches
 * DATA, because the DAC core's SYNC pulse and its CLK_FREQ/CLK_RATIO readback are
 * only meaningful against a running sample clock. That order is preserved --
 * axi_tpl_configure() writes the datapath registers and axi_tpl_enable(), driven
 * from inside the bring-up sequence, re-latches SYNC once the clocks are live.
 *
 * IMPORTANT: JESD204 is a negotiated multi-device link -- the transceiver, link
 * cores, SYSREF and the AD9082 cannot be brought up or status-checked in
 * isolation. Each block only *configures* here; activation and the single
 * meaningful status check happen together in the bring-up FSM, mirroring the
 * no-OS/Linux jesd204 state machine.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

#include <zephyr/drivers/misc/ad9081/ad9081.h>
#include <zephyr/drivers/clock_control/hmc7044.h>
#include <zephyr/drivers/misc/jesd204/axi_adxcvr.h>
#include <zephyr/drivers/misc/jesd204/axi_jesd204.h>
#include <zephyr/drivers/misc/jesd204/axi_tpl.h>
#include "jesd_fsm.h"

#ifdef CONFIG_AD9081_FAULT_INJECTION
#include "fault_injection.h"
#endif

/*
 * DAC output tone, matching no-OS axi_dac_data_setup()'s defaults:
 * 3 MHz at 0.05 of full scale (axi_dac_core.c:1229-1234).
 *
 * The rate the DDS phase accumulator runs at is the transport core's sample rate,
 * 250 MSPS for this link (4 GHz ADC / 4x main / 4x channel decimation on the
 * receive side, and the matching interpolation on transmit).
 */
#define DAC_DDS_TONE_HZ      (3 * 1000 * 1000)
#define DAC_DDS_SAMPLE_RATE  (250 * 1000 * 1000)
#define DAC_DDS_SCALE_MICRO  (50 * 1000) /* 0.05 of full scale */

/*
 * Which HMC7044 output drives the GT reference clock. Taken from the transceiver
 * node's own `clocks` phandle rather than stated again here, so this report and
 * the rate the adxcvr driver actually solves its dividers against cannot drift
 * apart.
 */
#define GT_REFCLK_OUT DT_CLOCKS_CELL(DT_NODELABEL(tx_adxcvr), output)

/*
 * The HMC7044 is a clock_control driver, so it programmes itself at POST_KERNEL
 * before main() runs -- there is no hmc7044_probe()/setup_clocks() call to make.
 * Report what it achieved instead, so the boot log keeps the two lines the
 * explicit calls used to produce.
 */
static int report_clock_tree(const struct device *clk)
{
	struct hmc7044_status status;
	uint32_t refclk_hz;
	int ret;

	if (!device_is_ready(clk)) {
		/*
		 * The driver logged the specific failure during its own init;
		 * everything downstream needs its clocks, so stop here.
		 */
		LOG_ERR("HMC7044 did not initialise -- no clocks, cannot continue");
		return -ENODEV;
	}

	ret = hmc7044_get_status(clk, &status);
	if (ret) {
		LOG_ERR("could not read HMC7044 status (%d)", ret);
		return ret;
	}

	if (!status.pll1_locked || !status.pll2_locked) {
		/*
		 * Not fatal here, deliberately: the link cannot come up without
		 * locked PLLs, but letting the bring-up proceed and fail at the
		 * phase that actually depends on the clock is more diagnosable
		 * than bailing with only a lock bit to show for it.
		 */
		LOG_WRN("HMC7044 PLLs not locked (PLL1 %s, PLL2 %s) -- "
			"the JESD204 link will not come up",
			status.pll1_fsm_state_str,
			status.pll2_locked ? "locked" : "unlocked");
	}

	/* The rate axi_adxcvr solves its GT dividers against. */
	ret = clock_control_get_rate(clk, HMC7044_CLK_OUT(GT_REFCLK_OUT),
				     &refclk_hz);
	if (ret) {
		LOG_ERR("could not read the GT refclk rate (%d)", ret);
		return ret;
	}

	/*
	 * Deliberately not another "SUCCESS: HMC7044 clock tree configured" --
	 * the driver's init() already emitted that line, byte for byte as the
	 * pre-driver-model code did, so the boot log stays diffable. This is the
	 * one genuinely new line: what the clock tree is actually running at.
	 */
	LOG_INF("HMC7044 clocks live: PLL1 %s on CLKIN%u, PLL2 %u.%03u GHz, "
		"GT refclk out%u %u.%03u MHz",
		status.pll1_fsm_state_str, status.pll1_active_clkin,
		status.pll2_freq / 1000000000U,
		(status.pll2_freq / 1000000U) % 1000U,
		GT_REFCLK_OUT, refclk_hz / 1000000U,
		(refclk_hz / 1000U) % 1000U);

	return 0;
}

int main(void)
{
	const struct device *clk = DEVICE_DT_GET(DT_NODELABEL(hmc7044));
	const struct device *mxfe = DEVICE_DT_GET(DT_NODELABEL(ad9081));
	uint16_t prod_id;
	int ret;

	LOG_INF("=== AD9081/HMC7044 bring-up ===");

	/* Clock chip first (topology order). */
	ret = report_clock_tree(clk);
	if (ret) {
		return ret;
	}

	/* MxFE. */
	ret = ad9081_probe(mxfe, &prod_id);
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
	ret = ad9081_setup_datapath(mxfe);
	if (ret) {
		LOG_ERR("AD9081 datapath setup failed (%d)", ret);
		return ret;
	}
	LOG_INF("SUCCESS: AD9082 datapath configured (chip JESD links ready)");

	/*
	 * GT transceiver config only (clock-mux select, held in reset). The
	 * reset-release + status poll is deferred to the JESD204 bring-up FSM,
	 * because GT-ready is only meaningful once the link layer and SYSREF are
	 * up around it -- gating on it standalone times out.
	 */
	ret = axi_adxcvr_configure(DEVICE_DT_GET(DT_NODELABEL(tx_adxcvr)));
	if (ret) {
		LOG_ERR("AXI adxcvr (GT) config failed (%d)", ret);
		return ret;
	}
	ret = axi_adxcvr_configure(DEVICE_DT_GET(DT_NODELABEL(rx_adxcvr)));
	if (ret) {
		LOG_ERR("AXI adxcvr (GT) config failed (%d)", ret);
		return ret;
	}
	LOG_INF("SUCCESS: GT transceivers configured (TX QPLL0 / RX CPLL)");

	/*
	 * JESD204 link cores: program link geometry + ILAS, held disabled. TX
	 * before RX, which is the order the single-call version used.
	 */
	ret = axi_jesd204_configure(DEVICE_DT_GET(DT_NODELABEL(tx_jesd)));
	if (ret == 0) {
		ret = axi_jesd204_configure(DEVICE_DT_GET(DT_NODELABEL(rx_jesd)));
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

	/* TPL transport cores: datapath sample-format (RX) + data-source (TX). */
	ret = axi_tpl_configure(DEVICE_DT_GET(DT_NODELABEL(rx_tpl)));
	if (ret == 0) {
		ret = axi_tpl_configure(DEVICE_DT_GET(DT_NODELABEL(tx_tpl)));
	}
	if (ret) {
		LOG_ERR("AXI TPL transport config failed (%d)", ret);
		return ret;
	}
	LOG_INF("SUCCESS: TPL transport cores configured (%d converters)",
		DT_PROP(DT_NODELABEL(rx_tpl), adi_num_channels));

	LOG_INF("=== all blocks configured, running JESD204 bring-up ===");

	/*
	 * Bring the link up: activate the transceiver, link cores and the chip's
	 * framer/deframer together, then read link status. This is the counterpart
	 * of no-OS jesd204_fsm_start() (app.c:446).
	 */
	ret = jesd204_bringup();
	if (ret) {
		LOG_WRN("JESD204 bring-up did not reach DATA (%d)", ret);
		return ret;
	}
	LOG_INF("SUCCESS: JESD204B link up");

	/*
	 * Per-lane check, as no-OS does immediately after its FSM (app.c:448).
	 * Reaching DATA does not guarantee every lane is aligned; if one is not,
	 * the watchdog bounces the link and returns -EAGAIN. Re-read the status
	 * afterwards rather than treating that as fatal.
	 */
	ret = axi_jesd204_rx_watchdog(DEVICE_DT_GET(DT_NODELABEL(rx_jesd)));
	if (ret == -EAGAIN) {
		LOG_WRN("link was restarted after a lane desync, re-reading status");
	}

	/*
	 * no-OS app.c:450-451 -- the one meaningful link status check. Both ends
	 * are read and logged before either verdict is taken, so a failure on TX
	 * does not hide RX's state.
	 */
	ret = axi_jesd204_status_read(DEVICE_DT_GET(DT_NODELABEL(tx_jesd)));
	if (axi_jesd204_status_read(DEVICE_DT_GET(DT_NODELABEL(rx_jesd)))) {
		ret = -EIO;
	}
	if (ret) {
		LOG_ERR("=== link is not carrying DATA ===");
		return ret;
	}

	/*
	 * Verify the transport layer, now that the link underneath it is carrying
	 * DATA. This is no-OS's axi_dac_init()/axi_adc_init() position: after
	 * jesd204_fsm_start() returns, not inside a phase (app.c:454-455). The TPL
	 * cores are downstream of the link -- they only map JESD frames onto
	 * converters -- so they are not JESD204 topology devices.
	 *
	 * Warn but do not fail: the link is up, which is what was being brought up
	 * here. A TPL complaint is a datapath problem below the link, and returning
	 * an error would report the link as broken when it is not.
	 */
	if (axi_tpl_enable(DEVICE_DT_GET(DT_NODELABEL(rx_tpl)),
			   DEVICE_DT_GET(DT_NODELABEL(tx_tpl)))) {
		LOG_WRN("TPL post-link verify failed (link is up regardless)");
	}

	/*
	 * Point the DAC converters at the transport core's DDS, which is what the
	 * no-OS example emits: 3 MHz at 0.05 full scale, upconverted by the chip's
	 * +2 GHz main NCO. Scope the DAC output to see it.
	 */
	ret = axi_tpl_tx_dds(DEVICE_DT_GET(DT_NODELABEL(tx_tpl)),
			     DAC_DDS_TONE_HZ, DAC_DDS_SAMPLE_RATE,
			     DAC_DDS_SCALE_MICRO, true);
	if (ret) {
		LOG_ERR("could not arm the DAC DDS tone (%d)", ret);
		return ret;
	}
	LOG_INF("SUCCESS: DAC emitting a %u MHz DDS tone at %u%% full scale",
		DAC_DDS_TONE_HZ / 1000000U, DAC_DDS_SCALE_MICRO / 10000U);

	LOG_INF("=== bring-up complete ===");

#ifdef CONFIG_AD9081_FAULT_INJECTION
	/*
	 * Last, deliberately. The tests need a link that is known to be up before
	 * they break it, so that a test failure means the injected fault was
	 * mishandled rather than that the link was never working. Anything the
	 * bring-up above achieved has already been reported by this point.
	 */
	if (jesd204_fault_injection_run()) {
		return -EIO;
	}
#endif

	return 0;
}
