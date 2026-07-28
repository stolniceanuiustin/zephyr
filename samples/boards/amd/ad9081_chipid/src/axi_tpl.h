/*
 * AXI AD9081 TPL (transport-layer) cores -- ADC framer / DAC deframer datapath.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef AXI_TPL_H_
#define AXI_TPL_H_

/*
 * Configure the RX (ADC) and TX (DAC) transport cores: pulse core reset, arm the
 * per-converter datapath (RX sample format/enable, TX data-source select) and
 * latch the TX config. Configure-only -- the STATUS/clock verification (which
 * only reads valid once the JESD link and sample clocks are live) is deferred to
 * axi_tpl_enable(). Returns 0 on success, negative errno otherwise.
 */
int axi_tpl_configure(void);

/*
 * Finish TPL bring-up once the link + lane clocks are running: re-issue the DAC
 * datapath SYNC pulse and verify the RX/TX core STATUS. Driven by the JESD204
 * bring-up sequence -- not called standalone.
 */
int axi_tpl_enable(void);

#endif /* AXI_TPL_H_ */
