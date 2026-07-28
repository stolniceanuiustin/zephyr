/*
 * AXI JESD204 link cores (RX/TX) -- PL AXI plane probe.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef AXI_JESD_H_
#define AXI_JESD_H_

/*
 * Prove the PL AXI plane is alive and the loaded bitstream is the expected MxFE
 * link IP: read the RX and TX JESD204 cores' identity registers (MAGIC tag,
 * PCORE version, synthesised lane count) and validate them.
 *
 * Returns 0 if both cores respond with the correct MAGIC, negative errno
 * otherwise (e.g. -ENODEV if the PL is unconfigured / the wrong bitstream).
 */
int axi_jesd_probe(void);

#endif /* AXI_JESD_H_ */
