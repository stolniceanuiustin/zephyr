/*
 * HMC7044 clock/SYSREF chip -- SPI bring-up helpers.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef HMC7044_H_
#define HMC7044_H_

/*
 * Prove the SPI path to the HMC7044 over SPI1. The HMC7044 has no chip-ID
 * register, so -- like no-OS hmc7044_read_write_check() -- this writes a known
 * byte to the scratchpad register and reads it back.
 *
 * Returns 0 on a confirmed read/write, negative errno otherwise.
 */
int hmc7044_probe(void);

#endif /* HMC7044_H_ */
