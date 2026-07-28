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

/*
 * Program the HMC7044 clock tree (PLL1/PLL2/VCO, output dividers and SYSREF)
 * for the zcu102 AD9082-FMC-EBZ-A2 profile, then report PLL1/PLL2 lock status.
 * Faithful port of no-OS hmc7044_setup().
 *
 * Returns 0 on success, negative errno otherwise.
 */
int hmc7044_setup_clocks(void);

#endif /* HMC7044_H_ */
