/*
 * Copyright (c) 2026 Analog Devices, Inc.
 *
 * AD9081/AD9082 MxFE.
 *
 * One devicetree node per chip (compatible "adi,ad9081"), so every entry point
 * takes the device it acts on.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_MISC_AD9081_AD9081_H_
#define ZEPHYR_INCLUDE_DRIVERS_MISC_AD9081_AD9081_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/device.h>

/*
 * Probe the chip over its SPI bus: pulse RSTB, set 4-wire mode, read PROD_ID.
 * Accepts 0x9081 and 0x9082 -- same family, same register map. Stores the
 * 16-bit PROD_ID read back in *prod_id when non-NULL. Returns 0 on success,
 * negative errno otherwise.
 *
 * The bus controller's register page is 1:1 mapped by a PRE_KERNEL_1 SYS_INIT
 * in spi_mmio_fixup.c, working around the upstream Cadence SPI driver not using
 * DEVICE_MMIO -- see the comment there.
 */
int ad9081_probe(const struct device *dev, uint16_t *prod_id);

/*
 * Configure the MxFE datapath (configure-only): on-chip CLK PLL, TX interp +
 * DAC NCOs + JRX deframer link params, RX decim + ADC NCOs + JTX framer link
 * params and converter/lane mapping. Mirrors no-OS ad9081_setup(); the device's
 * JESD links are configured but NOT enabled -- enabling is done by the bring-up
 * sequence together with the FPGA cores. Must run after ad9081_probe().
 * Returns 0 on success, negative errno otherwise.
 */
int ad9081_setup_datapath(const struct device *dev);

/*
 * ------------------------- bring-up ops --------------------------------------
 *
 * What the JESD204 bring-up sequence needs from the chip, as driver ops rather
 * than as the raw vendor handle. Before this, ad9081_bringup.c reached the
 * handle through an ad9081_get_device() accessor and called adi_ad9081_*
 * directly,
 * which meant the FSM included the vendor headers and named this part's register
 * bits -- so a second converter, or a different MxFE, could not be dropped in
 * behind the same FSM.
 *
 * Everything here is a *chip-side link* operation: the FPGA transceiver and link
 * cores have their own drivers. The ops are deliberately narrow -- one per thing
 * the FSM actually does, no pass-through of vendor enums -- so this interface
 * says what a converter has to be able to do, not which library implements it.
 *
 * All return 0 on success and a negative errno on failure. Valid after
 * ad9081_probe(); the link ops additionally need ad9081_setup_datapath(),
 * because the vendor API requires its startup_tx()/startup_rx() first.
 */
struct ad9081_driver_api {
	/*
	 * One-shot SYNC. Subclass comes from the link node's adi,subclass, so
	 * the caller does not restate it.
	 */
	int (*sync_oneshot)(const struct device *dev);

	/* NCO sync, after SYNC. */
	int (*sync_nco)(const struct device *dev);

	/*
	 * JESD PLL lock status. Non-zero means locked; the raw value is passed
	 * out because it is logged.
	 */
	int (*jesd_pll_status_get)(const struct device *dev, uint8_t *status);

	/*
	 * Deframer serdes calibration. force_reset must be true for the first
	 * calibration after boot; boost_mask is one bit per lane for high-boost
	 * mode (insertion loss above ~10 dB).
	 */
	int (*deframer_calibrate)(const struct device *dev, bool force_reset,
				  uint8_t boost_mask, bool run_bg_cal);

	/*
	 * Clear the deframer's transport-layer elastic-buffer protection. Resets
	 * to enabled and nothing else clears it on a 204B link -- see the call
	 * site in ad9081_bringup.c for why this is done and what it did not fix.
	 */
	int (*deframer_buf_protect_disable)(const struct device *dev);

	/* Enable or disable the chip's deframer link. */
	int (*deframer_enable)(const struct device *dev, bool enable);

	/* Link status words, diagnostic only -- the FPGA cores gate on theirs. */
	int (*framer_status_get)(const struct device *dev, uint16_t *status);
	int (*deframer_status_get)(const struct device *dev, uint16_t *status);
};

/*
 * Plain `const struct`, not DEVICE_API(): that macro puts the API in an iterable
 * linker section whose start/end symbols are generated from the __subsystem tags
 * parse_syscalls.py finds, and it only scans include/, drivers/ and subsys/net.
 * A sample-local API class would get no section and fail to link. dev->api works
 * either way -- the section is only what DEVICE_API_IS() needs, and nothing here
 * type-checks a device at runtime. This becomes DEVICE_API() when the driver
 * moves in-tree and this header moves to include/zephyr/drivers/.
 */
#define AD9081_API(dev) ((const struct ad9081_driver_api *)(dev)->api)

/*
 * Wrappers, so call sites read as ad9081_*() rather than as api dereferences.
 * Every op is mandatory, so none of them is NULL-checked -- a driver that omits
 * one is a build-time hole, not a runtime one.
 */
static inline int ad9081_sync_oneshot(const struct device *dev)
{
	return AD9081_API(dev)->sync_oneshot(dev);
}

static inline int ad9081_sync_nco(const struct device *dev)
{
	return AD9081_API(dev)->sync_nco(dev);
}

static inline int ad9081_jesd_pll_status_get(const struct device *dev,
					     uint8_t *status)
{
	return AD9081_API(dev)->jesd_pll_status_get(dev, status);
}

static inline int ad9081_deframer_calibrate(const struct device *dev,
					    bool force_reset, uint8_t boost_mask,
					    bool run_bg_cal)
{
	return AD9081_API(dev)->deframer_calibrate(dev, force_reset, boost_mask,
						   run_bg_cal);
}

static inline int ad9081_deframer_buf_protect_disable(const struct device *dev)
{
	return AD9081_API(dev)->deframer_buf_protect_disable(dev);
}

static inline int ad9081_deframer_enable(const struct device *dev, bool enable)
{
	return AD9081_API(dev)->deframer_enable(dev, enable);
}

static inline int ad9081_framer_status_get(const struct device *dev,
					   uint16_t *status)
{
	return AD9081_API(dev)->framer_status_get(dev, status);
}

static inline int ad9081_deframer_status_get(const struct device *dev,
					     uint16_t *status)
{
	return AD9081_API(dev)->deframer_status_get(dev, status);
}

#endif /* ZEPHYR_INCLUDE_DRIVERS_MISC_AD9081_AD9081_H_ */
