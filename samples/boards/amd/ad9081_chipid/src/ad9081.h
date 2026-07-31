/*
 * AD9081/AD9082 MxFE.
 *
 * One devicetree node per chip (compatible "adi,ad9081"), so every entry point
 * takes the device it acts on.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef AD9081_H_
#define AD9081_H_

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
 * Accessor for the ADI device handle (adi_ad9081_device_t *), so the JESD204
 * bring-up sequence can drive the chip-side link enable / status APIs. Returned
 * as void * to keep the heavy ADI headers out of this interface; the FSM casts
 * it back. NULL if dev is NULL. Valid after ad9081_probe().
 *
 * A leak, and going away: it welds jesd_fsm.c to this one chip. Replaced by
 * driver ops in a following commit -- do not add call sites.
 */
void *ad9081_get_device(const struct device *dev);

#endif /* AD9081_H_ */
