/*
 * JESD204 datapath validation -- Rung 1: SERDES / receive-path PN test.
 *
 * The link reaching DATA proves the framed serial pipe negotiated (CGS -> ILAS
 * -> DATA); it does NOT prove the bits arriving are correct. This test closes
 * that gap for the receive direction with a deterministic pattern and zero DMA
 * or analog:
 *
 *   chip ADC datapath (PN test mode)
 *     -> JTX framer -> GT -> FPGA JESD204 link -> RX TPL core (PN checker)
 *
 * The chip is put into an ADC PN test mode; the FPGA RX transport core runs its
 * built-in PN monitor expecting the same polynomial and latches any out-of-sync
 * or bit error per converter. A clean pass = the whole receive serial path is
 * bit-error-free, i.e. the [DATA] milestone is real. Same shape as no-OS
 * fmcdaq2 (chip PN test mode + axi_adc_pn_mon()).
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(jesd_test, LOG_LEVEL_INF);

#include "ad9081.h"
#include "axi_tpl.h"
#include "jesd_test.h"

#include "adi_ad9081.h"

/* Chip-side framer link carrying the ADC data. */
#define AD9081_JTX_LINK AD9081_LINK_0

/*
 * One PN polynomial paired across both ends. PN9 and PN23 are the classic ADC
 * link-check patterns (short enough to sync fast, long enough to catch stuck
 * lanes). The chip test-mode enum and the AXI TPL PN-select enum use different
 * numeric codes for the same polynomial, so each entry carries both.
 */
struct pn_case {
	const char *name;
	adi_ad9081_test_mode_e chip_mode; /* chip ADC datapath emits this */
	enum axi_tpl_pn_sel tpl_sel;      /* FPGA TPL checker expects this */
};

static const struct pn_case pn_cases[] = {
	{ "PN9",   AD9081_TMODE_PN9,  AXI_TPL_PN9   },
	/*
	 * The AXI ADC core distinguishes two PN23 conventions; converters that
	 * generate PN on-chip (like the AD9082 ADC datapath) use the "A" variant
	 * -- pairing chip PN23 with AXI_TPL_PN23 (6) leaves the checker unable to
	 * sync (PN_OOS on every lane), while AXI_TPL_PN23A (1) locks. This mirrors
	 * no-OS fmcdaq2, which monitors AD9680 PN23 as AXI_ADC_PN23A.
	 */
	{ "PN23A", AD9081_TMODE_PN23, AXI_TPL_PN23A },
};

int jesd_test_rx_pn(void)
{
	adi_ad9081_device_t *dev = ad9081_get_device();
	int failures = 0;
	int32_t err;

	if (dev == NULL) {
		LOG_ERR("AD9081 device not initialised");
		return -ENODEV;
	}

	LOG_INF("--- Rung 1: receive-path PN link-integrity test ---");

	for (size_t i = 0; i < ARRAY_SIZE(pn_cases); i++) {
		const struct pn_case *c = &pn_cases[i];
		int rc;

		/* Chip: drive both I and Q of the ADC datapath with this PN. */
		err = adi_ad9081_adc_test_mode_config_set(dev, c->chip_mode,
							  c->chip_mode,
							  AD9081_JTX_LINK);
		if (err != API_CMS_ERROR_OK) {
			LOG_WRN("%s: chip test-mode set failed (%d)", c->name,
				err);
			failures++;
			continue;
		}

		/* Let the pattern propagate before the checker clears status. */
		k_msleep(2);

		/* FPGA: check every converter for OOS/bit errors over 10 ms. */
		rc = axi_tpl_adc_pn_mon(c->tpl_sel, 10);
		if (rc == 0) {
			LOG_INF("%s: PASS (all converters bit-error-free)",
				c->name);
		} else {
			LOG_WRN("%s: FAIL (see per-channel status above)",
				c->name);
			failures++;
		}
	}

	/* Restore normal ADC operation regardless of outcome. */
	err = adi_ad9081_adc_test_mode_config_set(dev, AD9081_TMODE_OFF,
						  AD9081_TMODE_OFF,
						  AD9081_JTX_LINK);
	if (err != API_CMS_ERROR_OK) {
		LOG_WRN("failed to restore ADC normal mode (%d)", err);
	}

	if (failures == 0) {
		LOG_INF("=== Rung 1 PASS: receive serial path is bit-error-free ===");
		return 0;
	}

	LOG_WRN("=== Rung 1: %d PN case(s) failed ===", failures);
	return -EIO;
}
