/*
 * AD9081/AD9082 + HMC7044 JESD204B bring-up (application / FSM).
 *
 * This app grows bottom-up into the full JESD204B link, mirroring how no-OS
 * keeps the project FSM in application source. Current milestones:
 *
 *   [x] SPI0 -> AD9081/AD9082: read PROD_ID (0x9081/0x9082)
 *   [x] SPI1 -> HMC7044: scratchpad read/write check
 *   [ ] HMC7044 clock + SYSREF configuration
 *   [ ] AXI adxcvr / jesd204 rx-tx / transport cores
 *   [ ] link bring-up: CGS -> ILAS -> Data
 *   [ ] DMA capture (axi_dmac)
 *
 * Bring-up order is topology order: clock -> PHY -> link -> transport. For now
 * we just prove both SPI control planes are alive before building on them.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

#include "ad9081.h"
#include "hmc7044.h"

int main(void)
{
	uint16_t prod_id;
	int ret;

	LOG_INF("=== AD9081/HMC7044 bring-up ===");

	/* Clock chip first (topology order). */
	ret = hmc7044_probe();
	if (ret) {
		LOG_ERR("HMC7044 probe failed (%d)", ret);
		return ret;
	}
	LOG_INF("SUCCESS: HMC7044 scratchpad read/write confirmed");

	/* MxFE. */
	ret = ad9081_probe(&prod_id);
	if (ret) {
		LOG_ERR("AD9081 probe failed (%d)", ret);
		return ret;
	}
	LOG_INF("SUCCESS: AD90%02x detected over SPI", prod_id & 0xFF);

	LOG_INF("=== bring-up milestone: both SPI control planes alive ===");
	return 0;
}
