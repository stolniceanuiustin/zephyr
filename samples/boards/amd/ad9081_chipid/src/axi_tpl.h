/*
 * AXI AD9081 TPL (transport-layer) cores -- ADC framer / DAC deframer datapath.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef AXI_TPL_H_
#define AXI_TPL_H_

#include <stdbool.h>
#include <stdint.h>

/*
 * AXI ADC PN sequence codes for the RX TPL core's PN checker (axi_adc_core.h).
 * These must match the polynomial the chip's ADC test mode is emitting.
 */
enum axi_tpl_pn_sel {
	AXI_TPL_PN9   = 0,
	AXI_TPL_PN23A = 1,
	AXI_TPL_PN7   = 4,
	AXI_TPL_PN15  = 5,
	AXI_TPL_PN23  = 6,
	AXI_TPL_PN31  = 7,
};

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

/*
 * Drive the TX transport core's converters from their internal FPGA DDS tone
 * generators (enable=true) or from the DMA stream (enable=false).
 *
 * A diagnostic aid for isolating where transmit samples are lost. The DDS sits at
 * the TPL input inside the FPGA, so its samples traverse the transport core, the
 * serial lanes and the chip's deframer -- everything the DMA path crosses except
 * DDR and the DMA engine. Comparing it against the chip's own test tone (which
 * enters downstream of the deframer) splits the transmit path in two.
 *
 * freq_hz is quantised to freq * 0xFFFF / sample_rate_hz by the 16-bit phase
 * accumulator, so the achieved frequency only equals the request when it divides
 * the sample rate evenly. Returns 0, or -EINVAL if the tone is below the DDS
 * resolution at that sample rate.
 */
int axi_tpl_tx_dds(uint32_t freq_hz, uint32_t sample_rate_hz, bool enable);

/*
 * Run a PN link-integrity test on the receive path. The caller first puts the
 * chip's ADC datapath into the matching PN test mode; this programs each RX TPL
 * converter's PN checker to the same polynomial, clears the sticky status, waits
 * delay_ms and reports per-converter OOS/ERR. Returns 0 if all converters are
 * clean, -EIO if any shows an out-of-sync or bit error. No DMA/analog involved.
 */
int axi_tpl_adc_pn_mon(uint32_t pn_sel, uint32_t delay_ms);

#endif /* AXI_TPL_H_ */
