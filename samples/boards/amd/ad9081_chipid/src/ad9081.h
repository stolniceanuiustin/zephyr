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

#endif /* AD9081_H_ */
