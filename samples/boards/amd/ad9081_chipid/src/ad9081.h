/*
 * AD9081/AD9082 MxFE -- SPI bring-up helpers.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef AD9081_H_
#define AD9081_H_

#include <stdint.h>

/*
 * Probe the AD9081/AD9082 over SPI0: pulse RSTB, set 4-wire mode, read PROD_ID.
 * Returns 0 and stores the 16-bit PROD_ID (0x9081 or 0x9082) in *prod_id on
 * success, negative errno otherwise.
 */
int ad9081_probe(uint16_t *prod_id);

/*
 * Configure the MxFE datapath (configure-only): on-chip CLK PLL, TX interp +
 * DAC NCOs + JRX deframer link params, RX decim + ADC NCOs + JTX framer link
 * params and converter/lane mapping. Mirrors no-OS ad9081_setup(); the device's
 * JESD links are configured but NOT enabled -- enabling is done by the bring-up
 * sequence together with the FPGA cores. Must run after ad9081_probe().
 * Returns 0 on success, negative errno otherwise.
 */
int ad9081_setup_datapath(void);

/*
 * Accessor for the ADI device handle (adi_ad9081_device_t *), so the JESD204
 * bring-up sequence can drive the chip-side link enable / status APIs. Returned
 * as void * to keep the heavy ADI headers out of this interface; the FSM casts
 * it back. Valid after ad9081_probe().
 */
void *ad9081_get_device(void);

#endif /* AD9081_H_ */
