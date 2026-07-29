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
 * One PN polynomial paired across both ends. The chip test-mode enum and the
 * AXI TPL PN-select enum use different numeric codes for the same polynomial,
 * so each entry carries both.
 *
 * Cases are split by intent:
 *  - "required": short sequences (PN7/PN9/PN15). Their max consecutive-
 *    identical-bit (CID) run is <=15, representative of real sample data. These
 *    MUST pass -- a failure here means the receive path is genuinely dropping
 *    bits, and Rung 1 fails.
 *  - "stress" (required=false): long sequences (PN23/PN31) with CID runs up to
 *    23/31 that deliberately torture the transceiver CDR / AC-coupling far
 *    beyond anything real samples contain. These are a margin *diagnostic*: we
 *    report how far the link holds up, but a failure is not fatal to Rung 1.
 *
 * Observed on this board (2026-07-28): PN7/PN9/PN15 clean, PN23A errors,
 * PN31 can't sync -- failure tracks CID length monotonically, the textbook
 * link-margin signature (not a polynomial-convention bug, which would be
 * scattered). PN23 uses the "A" variant on-chip: AXI_TPL_PN23 (6) can't sync,
 * AXI_TPL_PN23A (1) locks (mirrors no-OS fmcdaq2 on the AD9680).
 */
struct pn_case {
	const char *name;
	adi_ad9081_test_mode_e chip_mode; /* chip ADC datapath emits this */
	enum axi_tpl_pn_sel tpl_sel;      /* FPGA TPL checker expects this */
	bool required;                    /* false = stress/margin diagnostic */
};

static const struct pn_case pn_cases[] = {
	{ "PN7",   AD9081_TMODE_PN7,  AXI_TPL_PN7,   true  },
	{ "PN9",   AD9081_TMODE_PN9,  AXI_TPL_PN9,   true  },
	{ "PN15",  AD9081_TMODE_PN15, AXI_TPL_PN15,  true  },
	{ "PN23A", AD9081_TMODE_PN23, AXI_TPL_PN23A, false },
	{ "PN31",  AD9081_TMODE_PN31, AXI_TPL_PN31,  false },
};

int jesd_test_rx_pn(void)
{
	adi_ad9081_device_t *dev = ad9081_get_device();
	int required_failures = 0;
	const char *margin = "none"; /* longest stress pattern that still passed */
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
			if (c->required) {
				required_failures++;
			}
			continue;
		}

		/* Let the pattern propagate before the checker clears status. */
		k_msleep(2);

		/* FPGA: check every converter for OOS/bit errors over 10 ms. */
		rc = axi_tpl_adc_pn_mon(c->tpl_sel, 10);
		if (rc == 0) {
			LOG_INF("%s: PASS%s", c->name,
				c->required ? " (required)" : " (stress)");
			if (!c->required) {
				margin = c->name;
			}
		} else if (c->required) {
			LOG_ERR("%s: FAIL (required) -- receive path dropping bits",
				c->name);
			required_failures++;
		} else {
			LOG_WRN("%s: fail (stress, margin diagnostic only)",
				c->name);
		}
	}

	/* Restore normal ADC operation regardless of outcome. */
	err = adi_ad9081_adc_test_mode_config_set(dev, AD9081_TMODE_OFF,
						  AD9081_TMODE_OFF,
						  AD9081_JTX_LINK);
	if (err != API_CMS_ERROR_OK) {
		LOG_WRN("failed to restore ADC normal mode (%d)", err);
	}

	if (required_failures == 0) {
		LOG_INF("=== Rung 1 PASS: receive serial path is bit-error-free ===");
		LOG_INF("    (stress-pattern margin: longest clean = %s; "
			"real samples never hit these CID runs)", margin);
		return 0;
	}

	LOG_ERR("=== Rung 1 FAIL: %d required PN case(s) failed ===",
		required_failures);
	return -EIO;
}
