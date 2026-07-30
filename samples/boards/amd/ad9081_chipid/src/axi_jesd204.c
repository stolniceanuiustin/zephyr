/*
 * AXI JESD204 link cores (RX framer / TX deframer) -- configure-only.
 *
 * Programs the link-layer parameters (M/L/F/K/N/NP..., scrambling, ILAS) into
 * the RX and TX axi_jesd204 cores for the AD9082 m8-l4 link. This is the
 * LINK_INIT-phase step of the JESD204 bring-up: it writes configuration
 * registers only. The link is held disabled here; the actual lane-clock enable
 * + SYSREF + status check happen later, driven by the bring-up sequence, since
 * a JESD204 link only comes alive when the transceiver, link cores, SYSREF and
 * the AD9082 are activated together (see project notes / no-OS jesd204 FSM).
 *
 * Register offsets, field packing and the ILAS layout are transcribed from
 * no-OS axi_jesd204_{rx,tx}.c. Link parameters come from the ADI Linux DTS for
 * hdl_project ad9081_fmca_ebz/zcu102 (TX link-mode 9, RX link-mode 10, both
 * subclass 1, M8 L4 F4 K32 N16 NP16 S1 CS0 HD0).
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/kernel/mm.h>
#include <zephyr/sys/device_mmio.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(axi_jesd204, LOG_LEVEL_INF);

#include "axi_jesd204.h"

/* AXI base addresses (on-board bitstream system.hwh). Mapped by axi_jesd.c. */
#define JESD204_RX_BASE 0x84A90000UL
#define JESD204_TX_BASE 0x84B90000UL

/* Register offsets (common to axi_jesd204_rx and _tx unless noted). */
#define JESD204_REG_VERSION           0x00
#define JESD204_REG_MAGIC             0x0c
#define JESD204_REG_SYNTH_NUM_LANES   0x10
#define JESD204_REG_SYNTH_DATA_PATH_WIDTH 0x14
#define JESD204_REG_LINK_DISABLE      0xc0
#define JESD204_REG_SYSREF_CONF       0x100
#define JESD204_REG_SYSREF_LMFC_OFFSET 0x104
#define JESD204_REG_SYSREF_STATUS     0x108

/* TX uses CONF0 at 0x210; RX uses LINK_CONF0 at 0x210 (same offset). */
#define JESD204_REG_CONF0             0x210
#define JESD204_REG_LINK_CONF4        0x21C
#define JESD204_RG_LINK_STATE         0xc4
#define JESD204_REG_LINK_STATUS       0x280
#define JESD204_RX_REG_LINK_CONF2     0x240
#define JESD204_TX_REG_ILAS(x, y)     (((x) * 32 + (y) * 4) + 0x310)

/* Per-lane RX status/error counters (axi_jesd204_rx only). */
#define JESD204_RX_REG_LANE_STATUS(x) (((x) * 32) + 0x300)
#define JESD204_RX_REG_LANE_ERRORS(x) (((x) * 32) + 0x308)

/* LINK_STATUS value meaning "carrying data". */
#define JESD204_LINK_STATUS_DATA 3

/* Field bits. */
#define JESD204_SYSREF_CONF_SYSREF_DISABLE   BIT(0)
#define JESD204_RX_LINK_CONF2_BUFFER_EARLY_RELEASE BIT(16)

/* SYNTH_DATA_PATH_WIDTH packing. */
#define JESD204_SYNTH_DATA_PATH_WIDTH_GET(x) ((x) & 0xff)
#define JESD204_TPL_DATA_PATH_WIDTH_GET(x)   (((x) >> 8) & 0xff)

#define JESD204_TX_MAGIC (('2' << 24) | ('0' << 16) | ('4' << 8) | ('T'))
#define JESD204_RX_MAGIC (('2' << 24) | ('0' << 16) | ('4' << 8) | ('R'))

#define PCORE_VER(major, minor, patch) \
	(((major) << 16) | ((minor) << 8) | (patch))
#define PCORE_VER_MAJOR(x) (((x) >> 16) & 0xff)
#define PCORE_VER_MINOR(x) (((x) >> 8) & 0xff)

/*
 * JESD204B link parameters (AD9082 m8-l4, from the zcu102 DTS). Same numbers
 * for the RX framer and TX deframer on this design.
 */
#define JESD_L  4  /* lanes per device */
#define JESD_M  8  /* converters per device */
#define JESD_F  4  /* octets per frame */
#define JESD_K  32 /* frames per multiframe */
#define JESD_N  16 /* converter resolution */
#define JESD_NP 16 /* bits per sample */
#define JESD_S  1  /* samples per converter per frame */
#define JESD_CS 0  /* control bits per sample */
#define JESD_HD 0  /* high density */
#define JESD_SUBCLASS 1
#define JESD_VERSION  1 /* 1 = 204B */
#define JESD_SCRAMBLING 1

/* One JESD204 link core (RX or TX). */
struct axi_jesd204 {
	const char *name;
	uintptr_t base;
	bool tx; /* true = TX deframer, false = RX framer */
	uint32_t magic;
	/* read back from the core */
	uint32_t version;
	uint32_t num_lanes;
	uint32_t data_path_width;
	uint32_t tpl_data_path_width;
};

static struct axi_jesd204 jesd_tx = {
	.name = "jesd204-tx",
	.base = JESD204_TX_BASE,
	.tx = true,
	.magic = JESD204_TX_MAGIC,
};

static struct axi_jesd204 jesd_rx = {
	.name = "jesd204-rx",
	.base = JESD204_RX_BASE,
	.tx = false,
	.magic = JESD204_RX_MAGIC,
};

static inline uint32_t jesd_read(const struct axi_jesd204 *j, uint32_t reg)
{
	return sys_read32(j->base + reg);
}

static inline void jesd_write(const struct axi_jesd204 *j, uint32_t reg,
			      uint32_t val)
{
	sys_write32(val, j->base + reg);
}

/* ILAS checksum (sum of link fields, low 8 bits) -- no-OS calc_ilas_chksum. */
static uint8_t jesd_ilas_chksum(uint32_t lane_id)
{
	uint32_t sum = 0;

	sum += 0;                 /* device_id */
	sum += 0;                 /* bank_id */
	sum += lane_id;
	sum += JESD_L - 1;
	sum += JESD_SCRAMBLING;
	sum += JESD_F - 1;
	sum += JESD_K - 1;
	sum += JESD_M - 1;
	sum += JESD_CS;
	sum += JESD_N - 1;
	sum += JESD_NP - 1;
	sum += JESD_SUBCLASS;
	sum += JESD_S - 1;
	sum += JESD_VERSION;
	sum += JESD_HD;

	return sum & 0xff;
}

/*
 * Program the TX deframer's ILAS (Initial Lane Alignment Sequence) words for one
 * lane. These are the link parameters the TX will announce during ILAS. Layout
 * from no-OS axi_jesd204_tx_set_lane_ilas_legacy().
 */
static void jesd_tx_set_lane_ilas(const struct axi_jesd204 *j, uint32_t lane)
{
	uint32_t val;

	/* word 0: device_id<<8 | bank_id<<24 (both 0 here) */
	jesd_write(j, JESD204_TX_REG_ILAS(lane, 0), 0);

	/* word 1: lane id, L-1, scrambling, F-1, K-1 */
	val = lane;
	val |= (JESD_L - 1) << 8;
	val |= JESD_SCRAMBLING << 15;
	val |= (JESD_F - 1) << 16;
	val |= (JESD_K - 1) << 24;
	jesd_write(j, JESD204_TX_REG_ILAS(lane, 1), val);

	/* word 2: M-1, N-1, CS, NP-1, subclass, S-1, jesd_version */
	val = (JESD_M - 1);
	val |= (JESD_N - 1) << 8;
	val |= JESD_CS << 14;
	val |= (JESD_NP - 1) << 16;
	val |= JESD_SUBCLASS << 21;
	val |= (JESD_S - 1) << 24;
	val |= JESD_VERSION << 29;
	jesd_write(j, JESD204_TX_REG_ILAS(lane, 2), val);

	/* word 3: HD, checksum */
	val = JESD_HD << 7;
	val |= (uint32_t)jesd_ilas_chksum(lane) << 24;
	jesd_write(j, JESD204_TX_REG_ILAS(lane, 3), val);
}

/*
 * Configure one link core: verify identity, hold the link disabled, and program
 * the multiframe/frame geometry (+ ILAS for TX). Mirrors no-OS
 * axi_jesd204_{tx,rx}_init + apply_config_legacy, configure-phase only.
 */
static int jesd_configure(struct axi_jesd204 *j)
{
	uint32_t magic = jesd_read(j, JESD204_REG_MAGIC);
	uint32_t dpw;
	uint32_t octets_per_multiframe;
	uint32_t val;

	if (magic != j->magic) {
		LOG_ERR("%s: bad MAGIC 0x%08x", j->name, magic);
		return -ENODEV;
	}

	j->version = jesd_read(j, JESD204_REG_VERSION);
	if (PCORE_VER_MAJOR(j->version) != 1) {
		LOG_ERR("%s: unexpected PCORE major %u", j->name,
			PCORE_VER_MAJOR(j->version));
		return -ENOTSUP;
	}
	j->num_lanes = jesd_read(j, JESD204_REG_SYNTH_NUM_LANES);
	dpw = jesd_read(j, JESD204_REG_SYNTH_DATA_PATH_WIDTH);
	j->data_path_width = 1 << JESD204_SYNTH_DATA_PATH_WIDTH_GET(dpw);
	j->tpl_data_path_width = JESD204_TPL_DATA_PATH_WIDTH_GET(dpw);

	LOG_INF("%s @ 0x%08lx: PCORE v%u.%u, %u lanes, dpw=%u tpl_dpw=%u",
		j->name, (unsigned long)j->base, PCORE_VER_MAJOR(j->version),
		PCORE_VER_MINOR(j->version), j->num_lanes, j->data_path_width,
		j->tpl_data_path_width);

	/* Hold the link disabled while we configure it. */
	jesd_write(j, JESD204_REG_LINK_DISABLE, 0x1);

	octets_per_multiframe = JESD_K * JESD_F; /* 128 */
	if (octets_per_multiframe % j->data_path_width != 0) {
		LOG_ERR("%s: octets/multiframe %u not a multiple of dpw %u",
			j->name, octets_per_multiframe, j->data_path_width);
		return -EINVAL;
	}

	/* CONF0 / LINK_CONF0 (same offset): (octets/mf - 1) | (F-1)<<16 */
	val = (octets_per_multiframe - 1) | ((JESD_F - 1) << 16);
	jesd_write(j, JESD204_REG_CONF0, val);

	/* beats per multiframe (LINK_CONF4): TX >= 1.6a, RX >= 1.7a */
	if ((j->tx && j->version >= PCORE_VER(1, 6, 'a')) ||
	    (!j->tx && j->version >= PCORE_VER(1, 7, 'a'))) {
		val = (octets_per_multiframe / j->tpl_data_path_width) - 1;
		jesd_write(j, JESD204_REG_LINK_CONF4, val);
	}

	/*
	 * subclass 1 -> SYSREF is used, so we do NOT set SYSREF_DISABLE. (no-OS
	 * only disables SYSREF / sets buffer-early-release for subclass 0.)
	 */

	if (j->tx) {
		for (uint32_t lane = 0; lane < j->num_lanes; lane++) {
			jesd_tx_set_lane_ilas(j, lane);
		}
	}

	LOG_INF("%s: configured (M%u L%u F%u K%u NP%u subclass%u), link held disabled",
		j->name, JESD_M, JESD_L, JESD_F, JESD_K, JESD_NP,
		JESD_SUBCLASS);
	return 0;
}

int axi_jesd204_configure(void)
{
	int ret;

	ret = jesd_configure(&jesd_tx);
	if (ret) {
		return ret;
	}
	return jesd_configure(&jesd_rx);
}

int axi_jesd204_tx_lane_clk_enable(void)
{
	/* SYSREF status clear (write 0x3) + enable link (LINK_DISABLE=0). */
	jesd_write(&jesd_tx, JESD204_REG_SYSREF_STATUS, 0x3);
	jesd_write(&jesd_tx, JESD204_REG_LINK_DISABLE, 0x0);
	return 0;
}

int axi_jesd204_rx_lane_clk_enable(void)
{
	jesd_write(&jesd_rx, JESD204_REG_LINK_DISABLE, 0x0);
	return 0;
}

/*
 * Per-lane desync check -- port of no-OS axi_jesd204_rx_check_lane_status().
 *
 * Returns true if this lane needs the link restarted. For 8B/10B a *non-zero*
 * low two bits means the lane is fine (CGS/frame sync achieved); all-zero means
 * it has lost alignment. The error counter is only present from PCORE minor 2
 * onward, so it is read for the log line but never gates the decision.
 */
static bool jesd_rx_check_lane_status(const struct axi_jesd204 *j, uint32_t lane)
{
	uint32_t status = jesd_read(j, JESD204_RX_REG_LANE_STATUS(lane));
	uint32_t errors = 0;

	/* This link is 8B/10B (JESD204B); the 64B/66B EMB path does not apply. */
	if ((status & 0x3) != 0x0) {
		return false;
	}

	if (PCORE_VER_MINOR(j->version) >= 2) {
		errors = jesd_read(j, JESD204_RX_REG_LANE_ERRORS(lane));
	}

	LOG_WRN("%s: lane %u desynced (status 0x%08x, %u errors), restarting link",
		j->name, lane, status, errors);
	return true;
}

/*
 * Link watchdog -- port of no-OS axi_jesd204_rx_watchdog(), called once after
 * bring-up (no-OS app.c:448).
 *
 * A link can reach DATA with one lane already unhappy: the aggregate status only
 * reports the link state, so a lane that lost alignment during ILAS stays
 * invisible there. This inspects each lane and, if any is desynced, bounces
 * LINK_DISABLE to force a fresh CGS/ILAS pass.
 *
 * Only meaningful while the link is enabled and already reporting DATA -- a link
 * still negotiating has lanes legitimately mid-alignment, and disabling it then
 * would interrupt a bring-up that was going to succeed.
 *
 * One deliberate deviation: no-OS compares the whole status word against 3,
 * while this masks the state field. On this core the RX word reads exactly 0x3,
 * so both agree; masking only makes it tolerant of status bits above the state
 * field rather than silently skipping the check.
 */
int axi_jesd204_rx_watchdog(void)
{
	const struct axi_jesd204 *j = &jesd_rx;
	bool restart = false;

	if (jesd_read(j, JESD204_RG_LINK_STATE) & 0x1) {
		return 0; /* link disabled -- nothing to police */
	}

	if ((jesd_read(j, JESD204_REG_LINK_STATUS) & 0x3) !=
	    JESD204_LINK_STATUS_DATA) {
		return 0;
	}

	for (uint32_t lane = 0; lane < j->num_lanes; lane++) {
		restart |= jesd_rx_check_lane_status(j, lane);
	}

	if (restart) {
		jesd_write(j, JESD204_REG_LINK_DISABLE, 0x1);
		k_msleep(100);
		jesd_write(j, JESD204_REG_LINK_DISABLE, 0x0);
		return -EAGAIN;
	}

	LOG_INF("%s: all %u lanes in sync", j->name, j->num_lanes);
	return 0;
}

/* TX deframer link-state label (8B/10B: WAIT/CGS/ILAS/DATA). */
static const char *const tx_status_label[] = { "WAIT", "CGS", "ILAS", "DATA" };
/* RX framer link-state label (8B/10B: RESET/WAIT-PHY/CGS/DATA). */
static const char *const rx_status_label[] = { "RESET", "WAIT_PHY", "CGS",
					       "DATA" };

int axi_jesd204_status_read(void)
{
	uint32_t tx_state = jesd_read(&jesd_tx, JESD204_RG_LINK_STATE) & 0x1;
	uint32_t rx_state = jesd_read(&jesd_rx, JESD204_RG_LINK_STATE) & 0x1;
	uint32_t tx_status = jesd_read(&jesd_tx, JESD204_REG_LINK_STATUS);
	uint32_t rx_status = jesd_read(&jesd_rx, JESD204_REG_LINK_STATUS);

	LOG_INF("jesd204-tx: link %s, status=0x%08x [%s]",
		tx_state ? "disabled" : "enabled", tx_status,
		tx_status_label[tx_status & 0x3]);
	LOG_INF("jesd204-rx: link %s, status=0x%08x [%s]",
		rx_state ? "disabled" : "enabled", rx_status,
		rx_status_label[rx_status & 0x3]);

	/* Both ends carrying DATA (0x3) is the healthy end state. */
	if ((tx_status & 0x3) == 0x3 && (rx_status & 0x3) == 0x3) {
		return 0;
	}
	return -EIO;
}
