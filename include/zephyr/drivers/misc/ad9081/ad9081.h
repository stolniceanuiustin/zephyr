/*
 * Copyright (c) 2026 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief AD9081/AD9082 MxFE.
 *
 * One devicetree node per chip (compatible "adi,ad9081"), so every entry point
 * takes the device it acts on.
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_MISC_AD9081_AD9081_H_
#define ZEPHYR_INCLUDE_DRIVERS_MISC_AD9081_AD9081_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief AD9081/AD9082 MxFE interface
 * @defgroup ad9081_interface AD9081/AD9082 MxFE interface
 * @ingroup io_interfaces
 * @{
 */

/**
 * @brief Probe the chip over its SPI bus.
 *
 * Pulses RSTB, selects 4-wire mode and reads PROD_ID. Accepts 0x9081 and 0x9082
 * -- same family, same register map.
 *
 * @param dev     MxFE device.
 * @param prod_id Destination for the 16-bit PROD_ID read back.
 * @retval 0 on success.
 * @retval -EINVAL if @p prod_id is NULL.
 * @retval -ENODEV if @p dev is not ready, or PROD_ID is not a supported part.
 * @retval -EIO on an SPI transfer failure.
 */
int ad9081_probe(const struct device *dev, uint16_t *prod_id);

/**
 * @brief Configure the MxFE datapath.
 *
 * Sets up the on-chip CLK PLL, the TX interpolators, DAC NCOs and JRX deframer
 * link parameters, and the RX decimators, ADC NCOs, JTX framer link parameters
 * and converter/lane mapping.
 *
 * Configure-only: the device's JESD links are configured but not enabled, since
 * enabling is done by the bring-up sequence together with the FPGA cores. Must
 * run after ad9081_probe().
 *
 * @param dev MxFE device.
 * @retval 0 on success.
 * @retval -ENODEV if @p dev is not ready.
 * @retval -EIO on a vendor-library or SPI failure.
 */
int ad9081_setup_datapath(const struct device *dev);

/**
 * @brief Chip-side JESD204 bring-up operations.
 *
 * What the JESD204 bring-up sequence needs from the converter, as driver ops
 * rather than as a raw vendor handle -- so a second converter, or a different
 * MxFE, can be dropped in behind the same sequence.
 *
 * Everything here is a chip-side link operation; the FPGA transceiver and link
 * cores have their own drivers. The ops are deliberately narrow -- one per thing
 * the sequence actually does, no pass-through of vendor enums -- so this
 * interface says what a converter has to be able to do, not which library
 * implements it.
 *
 * All return 0 on success and a negative errno on failure. Valid after
 * ad9081_probe(); the link ops additionally need ad9081_setup_datapath().
 */
__subsystem struct ad9081_driver_api {
	/**
	 * One-shot SYNC. Subclass comes from the link node's adi,subclass, so
	 * the caller does not restate it.
	 */
	int (*sync_oneshot)(const struct device *dev);

	/** NCO sync, after SYNC. */
	int (*sync_nco)(const struct device *dev);

	/**
	 * JESD PLL lock status. Non-zero means locked; the raw value is passed
	 * out because it is logged.
	 */
	int (*jesd_pll_status_get)(const struct device *dev, uint8_t *status);

	/**
	 * Deframer serdes calibration. force_reset must be true for the first
	 * calibration after boot; boost_mask is one bit per lane for high-boost
	 * mode (insertion loss above ~10 dB).
	 */
	int (*deframer_calibrate)(const struct device *dev, bool force_reset, uint8_t boost_mask,
				  bool run_bg_cal);

	/**
	 * Clear the deframer's transport-layer elastic-buffer protection. Resets
	 * to enabled and nothing else clears it on a 204B link.
	 */
	int (*deframer_buf_protect_disable)(const struct device *dev);

	/** Enable or disable the chip's deframer link. */
	int (*deframer_enable)(const struct device *dev, bool enable);

	/** Framer status word, diagnostic only -- the FPGA cores gate on theirs. */
	int (*framer_status_get)(const struct device *dev, uint16_t *status);

	/** Deframer status word, diagnostic only. */
	int (*deframer_status_get)(const struct device *dev, uint16_t *status);
};

/**
 * @brief Issue a one-shot SYNC.
 *
 * @param dev MxFE device.
 * @return 0 on success, negative errno otherwise.
 */
static inline int ad9081_sync_oneshot(const struct device *dev)
{
	return DEVICE_API_GET(ad9081, dev)->sync_oneshot(dev);
}

/**
 * @brief Synchronise the NCOs. Call after ad9081_sync_oneshot().
 *
 * @param dev MxFE device.
 * @return 0 on success, negative errno otherwise.
 */
static inline int ad9081_sync_nco(const struct device *dev)
{
	return DEVICE_API_GET(ad9081, dev)->sync_nco(dev);
}

/**
 * @brief Read the chip's JESD PLL lock status.
 *
 * @param dev    MxFE device.
 * @param status Destination for the raw status byte; non-zero means locked.
 * @return 0 on success, negative errno otherwise.
 */
static inline int ad9081_jesd_pll_status_get(const struct device *dev, uint8_t *status)
{
	return DEVICE_API_GET(ad9081, dev)->jesd_pll_status_get(dev, status);
}

/**
 * @brief Run the deframer serdes calibration.
 *
 * @param dev         MxFE device.
 * @param force_reset true for the first calibration after boot.
 * @param boost_mask  One bit per lane needing high-boost mode.
 * @param run_bg_cal  Leave background calibration running afterwards.
 * @return 0 on success, negative errno otherwise.
 */
static inline int ad9081_deframer_calibrate(const struct device *dev, bool force_reset,
					    uint8_t boost_mask, bool run_bg_cal)
{
	return DEVICE_API_GET(ad9081, dev)
		->deframer_calibrate(dev, force_reset, boost_mask, run_bg_cal);
}

/**
 * @brief Clear the deframer's elastic-buffer protection.
 *
 * @param dev MxFE device.
 * @return 0 on success, negative errno otherwise.
 */
static inline int ad9081_deframer_buf_protect_disable(const struct device *dev)
{
	return DEVICE_API_GET(ad9081, dev)->deframer_buf_protect_disable(dev);
}

/**
 * @brief Enable or disable the chip's deframer link.
 *
 * @param dev    MxFE device.
 * @param enable true to enable the link.
 * @return 0 on success, negative errno otherwise.
 */
static inline int ad9081_deframer_enable(const struct device *dev, bool enable)
{
	return DEVICE_API_GET(ad9081, dev)->deframer_enable(dev, enable);
}

/**
 * @brief Read the chip's framer status word.
 *
 * @param dev    MxFE device.
 * @param status Destination for the status word.
 * @return 0 on success, negative errno otherwise.
 */
static inline int ad9081_framer_status_get(const struct device *dev, uint16_t *status)
{
	return DEVICE_API_GET(ad9081, dev)->framer_status_get(dev, status);
}

/**
 * @brief Read the chip's deframer status word.
 *
 * @param dev    MxFE device.
 * @param status Destination for the status word.
 * @return 0 on success, negative errno otherwise.
 */
static inline int ad9081_deframer_status_get(const struct device *dev, uint16_t *status)
{
	return DEVICE_API_GET(ad9081, dev)->deframer_status_get(dev, status);
}

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_MISC_AD9081_AD9081_H_ */
