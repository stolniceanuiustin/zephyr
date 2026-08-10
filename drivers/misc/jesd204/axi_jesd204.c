/*
 * Copyright (c) 2026 Analog Devices, Inc.
 *
 * AXI JESD204 link cores (RX framer / TX deframer) -- configure-only.
 *
 * Programs the link-layer parameters (M/L/F/K/N/NP..., scrambling, ILAS) into
 * the RX and TX axi_jesd204 cores. This is the LINK_INIT-phase step of the
 * JESD204 bring-up: it writes configuration registers only. The link is held
 * disabled here; the actual lane-clock enable + SYSREF + status check happen
 * later, driven by the bring-up sequence, since a JESD204 link only comes alive
 * when the transceiver, link cores, SYSREF and the AD9082 are activated
 * together.
 *
 * Two compatibles rather than one node type with a direction flag, following
 * ADI's Linux drivers: the framer and the deframer share the
 * VERSION/MAGIC/LINK_DISABLE/STATUS block but differ in what they own -- the
 * deframer announces the link (ILAS words, and therefore N/CS/S/HD), the framer
 * reports per-lane sync status the deframer has no equivalent of. ADI gives
 * them separate compatibles, separate bindings and separate drivers; the
 * direction is which IP core this is, not configuration of a common one.
 *
 * Link geometry comes from the devicetree nodes, named as ADI's Linux binding
 * names them
 * (Documentation/devicetree/bindings/iio/jesd204/adi,jesd204-{rx,tx}.txt).
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/init.h>
#include <zephyr/kernel/mm.h>
#include <zephyr/sys/device_mmio.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(axi_jesd204, LOG_LEVEL_INF);

#include <zephyr/drivers/misc/jesd204/axi_jesd204.h>
#include <zephyr/drivers/misc/jesd204/jesd204_geometry.h>

/*
 * Every sys_read32/sys_write32 below uses the physical address straight out of
 * the devicetree, which is only correct because the PL pages are identity
 * mapped. Same assumption as the other PL drivers here.
 */
BUILD_ASSERT(IS_ENABLED(CONFIG_KERNEL_DIRECT_MAP),
	     "PL register access assumes virt == phys (CONFIG_KERNEL_DIRECT_MAP)");

/* Register offsets (common to axi_jesd204_rx and _tx unless noted). */
#define JESD204_REG_VERSION           0x00
#define JESD204_REG_MAGIC             0x0c
#define JESD204_REG_SYNTH_NUM_LANES   0x10
#define JESD204_REG_SYNTH_DATA_PATH_WIDTH 0x14
#define JESD204_REG_LINK_DISABLE      0xc0
#define JESD204_REG_LINK_CLK_RATIO    0xc8
#define JESD204_REG_SYSREF_CONF       0x100
#define JESD204_REG_SYSREF_STATUS     0x108

/* TX uses CONF0 at 0x210; RX uses LINK_CONF0 at 0x210 (same offset). */
#define JESD204_REG_CONF0             0x210
#define JESD204_REG_LINK_CONF4        0x21C
#define JESD204_RG_LINK_STATE         0xc4
#define JESD204_REG_LINK_STATUS       0x280
#define JESD204_TX_REG_ILAS(x, y)     (((x) * 32 + (y) * 4) + 0x310)

/* Per-lane RX status/error counters (axi_jesd204_rx only). */
#define JESD204_RX_REG_LANE_STATUS(x) (((x) * 32) + 0x300)
#define JESD204_RX_REG_LANE_ERRORS(x) (((x) * 32) + 0x308)

/* LINK_STATUS value meaning "carrying data". */
#define JESD204_LINK_STATUS_DATA 3

/* Field bits. */
#define JESD204_SYSREF_CONF_SYSREF_DISABLE   BIT(0)

/* SYNTH_DATA_PATH_WIDTH packing. */
#define JESD204_SYNTH_DATA_PATH_WIDTH_GET(x) ((x) & 0xff)
#define JESD204_TPL_DATA_PATH_WIDTH_GET(x)   (((x) >> 8) & 0xff)

#define JESD204_TX_MAGIC (('2' << 24) | ('0' << 16) | ('4' << 8) | ('T'))
#define JESD204_RX_MAGIC (('2' << 24) | ('0' << 16) | ('4' << 8) | ('R'))

#define PCORE_VER(major, minor, patch) \
	(((major) << 16) | ((minor) << 8) | (patch))
#define PCORE_VER_MAJOR(x) (((x) >> 16) & 0xff)
#define PCORE_VER_MINOR(x) (((x) >> 8) & 0xff)

/* Devicetree configuration -- ROM, one per node. */
struct axi_jesd204_config {
	uintptr_t base;
	uint32_t magic; /* from the compatible: which core this must be */
	bool tx;        /* true = TX deframer, false = RX framer */

	/* Link geometry. N/CS/S are TX-only -- see the header. */
	uint32_t l;
	uint32_t m;
	uint32_t f;
	uint32_t k;
	uint32_t n;
	uint32_t np;
	uint32_t s;
	uint32_t cs;
	uint32_t hd;
	uint32_t subclass;

	/*
	 * Raw serial lane rate in kHz, same value and property name as the
	 * sibling adxcvr node's adi,lane-rate-khz (axi_adxcvr.c). Only used to
	 * compute the "Reported Link Clock" / lane-rate / LMFC lines of
	 * axi_jesd204_status_read()'s table. Nothing is configured from it, so
	 * this stays a plain integer rather than a clock phandle dependency.
	 */
	uint32_t lane_rate_khz;
};

/* Per-instance state -- RAM. Read back from the core at configure time. */
struct axi_jesd204_data {
	uint32_t version;
	uint32_t num_lanes;
	uint32_t data_path_width;
	uint32_t tpl_data_path_width;
};

static inline uint32_t jesd_read(const struct device *dev, uint32_t reg)
{
	const struct axi_jesd204_config *cfg = dev->config;

	return sys_read32(cfg->base + reg);
}

static inline void jesd_write(const struct device *dev, uint32_t reg,
			      uint32_t val)
{
	const struct axi_jesd204_config *cfg = dev->config;

	sys_write32(val, cfg->base + reg);
}

/*
 * 1:1 map every link core's register page before anything touches it. A SYS_INIT
 * rather than DEVICE_MMIO_MAP because the mapping must land at PRE_KERNEL_1,
 * ahead of any driver init that might read a PL register.
 */
#define JESD204_MAP_ONE(node)                                                                      \
	do {                                                                                       \
		mm_reg_t virt;                                                                     \
		uintptr_t phys = DT_REG_ADDR(node);                                                \
                                                                                                   \
		device_map(&virt, phys, DT_REG_SIZE(node), K_MEM_CACHE_NONE);                      \
		if (virt != phys) {                                                                \
			LOG_ERR("%s not identity-mapped: virt=0x%lx phys=0x%lx",                   \
				DT_NODE_FULL_NAME(node), (unsigned long)virt,                      \
				(unsigned long)phys);                                              \
			return -EIO;                                                               \
		}                                                                                  \
	} while (0);

static int axi_jesd204_map(void)
{
	DT_FOREACH_STATUS_OKAY(adi_axi_jesd204_rx_1_0, JESD204_MAP_ONE)
	DT_FOREACH_STATUS_OKAY(adi_axi_jesd204_tx_1_0, JESD204_MAP_ONE)
	return 0;
}

SYS_INIT(axi_jesd204_map, PRE_KERNEL_1, 0);

/* ILAS checksum: sum of the announced link fields, low 8 bits. */
static uint8_t jesd_ilas_chksum(const struct device *dev, uint32_t lane_id)
{
	const struct axi_jesd204_config *cfg = dev->config;
	uint32_t sum = 0;

	sum += 0;                 /* device_id */
	sum += 0;                 /* bank_id */
	sum += lane_id;
	sum += cfg->l - 1;
	sum += JESD204_SCRAMBLING;
	sum += cfg->f - 1;
	sum += cfg->k - 1;
	sum += cfg->m - 1;
	sum += cfg->cs;
	sum += cfg->n - 1;
	sum += cfg->np - 1;
	sum += cfg->subclass;
	sum += cfg->s - 1;
	sum += JESD204_VERSION_B;
	sum += cfg->hd;

	return sum & 0xff;
}

/*
 * Program the TX deframer's ILAS (Initial Lane Alignment Sequence) words for one
 * lane. These are the link parameters the TX will announce during ILAS.
 */
static void jesd_tx_set_lane_ilas(const struct device *dev, uint32_t lane)
{
	const struct axi_jesd204_config *cfg = dev->config;
	uint32_t val;

	/* word 0: device_id<<8 | bank_id<<24 (both 0 here) */
	jesd_write(dev, JESD204_TX_REG_ILAS(lane, 0), 0);

	/* word 1: lane id, L-1, scrambling, F-1, K-1 */
	val = lane;
	val |= (cfg->l - 1) << 8;
	val |= JESD204_SCRAMBLING << 15;
	val |= (cfg->f - 1) << 16;
	val |= (cfg->k - 1) << 24;
	jesd_write(dev, JESD204_TX_REG_ILAS(lane, 1), val);

	/* word 2: M-1, N-1, CS, NP-1, subclass, S-1, jesd_version */
	val = (cfg->m - 1);
	val |= (cfg->n - 1) << 8;
	val |= cfg->cs << 14;
	val |= (cfg->np - 1) << 16;
	val |= cfg->subclass << 21;
	val |= (cfg->s - 1) << 24;
	val |= JESD204_VERSION_B << 29;
	jesd_write(dev, JESD204_TX_REG_ILAS(lane, 2), val);

	/* word 3: HD, checksum */
	val = cfg->hd << 7;
	val |= (uint32_t)jesd_ilas_chksum(dev, lane) << 24;
	jesd_write(dev, JESD204_TX_REG_ILAS(lane, 3), val);
}

/*
 * Configure one link core: verify identity, hold the link disabled, and program
 * the multiframe/frame geometry (+ ILAS for TX). Configure phase only -- the
 * lane-clock enable and status check happen later, from the bring-up sequence.
 */
int axi_jesd204_configure(const struct device *dev)
{
	const struct axi_jesd204_config *cfg = dev->config;
	struct axi_jesd204_data *data = dev->data;
	uint32_t magic;
	uint32_t dpw;
	uint32_t octets_per_multiframe;
	uint32_t val;

	if (!device_is_ready(dev)) {
		LOG_ERR("%s is not ready", dev->name);
		return -ENODEV;
	}

	magic = jesd_read(dev, JESD204_REG_MAGIC);
	if (magic != cfg->magic) {
		LOG_ERR("%s: bad MAGIC 0x%08x", dev->name, magic);
		return -ENODEV;
	}

	data->version = jesd_read(dev, JESD204_REG_VERSION);
	if (PCORE_VER_MAJOR(data->version) != 1) {
		LOG_ERR("%s: unexpected PCORE major %u", dev->name,
			PCORE_VER_MAJOR(data->version));
		return -ENOTSUP;
	}
	data->num_lanes = jesd_read(dev, JESD204_REG_SYNTH_NUM_LANES);
	dpw = jesd_read(dev, JESD204_REG_SYNTH_DATA_PATH_WIDTH);
	data->data_path_width = 1 << JESD204_SYNTH_DATA_PATH_WIDTH_GET(dpw);
	data->tpl_data_path_width = JESD204_TPL_DATA_PATH_WIDTH_GET(dpw);

	LOG_DBG("%s @ 0x%08lx: PCORE v%u.%u, %u lanes, dpw=%u tpl_dpw=%u",
		dev->name, (unsigned long)cfg->base,
		PCORE_VER_MAJOR(data->version), PCORE_VER_MINOR(data->version),
		data->num_lanes, data->data_path_width,
		data->tpl_data_path_width);

	/*
	 * The core's synthesised lane count must match the geometry we are about
	 * to program. A mismatch means the loaded bitstream is not the design
	 * this node's properties describe -- warn rather than fail, since the
	 * link may still be diagnosable.
	 */
	if (data->num_lanes != cfg->l) {
		LOG_WRN("%s: SYNTH_NUM_LANES=%u (expected %u for m%u-l%u)",
			dev->name, data->num_lanes, cfg->l, cfg->m, cfg->l);
	}

	/* Hold the link disabled while we configure it. */
	jesd_write(dev, JESD204_REG_LINK_DISABLE, 0x1);

	octets_per_multiframe = cfg->k * cfg->f;
	if (octets_per_multiframe % data->data_path_width != 0) {
		LOG_ERR("%s: octets/multiframe %u not a multiple of dpw %u",
			dev->name, octets_per_multiframe,
			data->data_path_width);
		return -EINVAL;
	}

	/* CONF0 / LINK_CONF0 (same offset): (octets/mf - 1) | (F-1)<<16 */
	val = (octets_per_multiframe - 1) | ((cfg->f - 1) << 16);
	jesd_write(dev, JESD204_REG_CONF0, val);

	/* beats per multiframe (LINK_CONF4): TX >= 1.6a, RX >= 1.7a */
	if ((cfg->tx && data->version >= PCORE_VER(1, 6, 'a')) ||
	    (!cfg->tx && data->version >= PCORE_VER(1, 7, 'a'))) {
		val = (octets_per_multiframe / data->tpl_data_path_width) - 1;
		jesd_write(dev, JESD204_REG_LINK_CONF4, val);
	}

	/*
	 * subclass 1 -> SYSREF is used, so SYSREF_DISABLE stays clear. The
	 * subclass-0 registers (SYSREF_LMFC_OFFSET, RX LINK_CONF2's
	 * buffer-early-release) are deliberately not touched, and not defined
	 * above, since nothing on this board runs subclass 0.
	 */

	if (cfg->tx) {
		/* cfg->l, not the read-back count: every declared lane must
		 * get its ILAS words even if SYNTH_NUM_LANES disagrees.
		 */
		for (uint32_t lane = 0; lane < cfg->l; lane++) {
			jesd_tx_set_lane_ilas(dev, lane);
		}
	}

	LOG_DBG("%s: configured (M%u L%u F%u K%u NP%u subclass%u), link held disabled",
		dev->name, cfg->m, cfg->l, cfg->f, cfg->k, cfg->np,
		cfg->subclass);
	return 0;
}

int axi_jesd204_lane_clk_enable(const struct device *dev)
{
	const struct axi_jesd204_config *cfg;

	if (!device_is_ready(dev)) {
		return -ENODEV;
	}
	cfg = dev->config;

	/*
	 * TX additionally clears the SYSREF status (write 0x3 to the sticky
	 * captured/alignment-error bits) so the first SYSREF after the link is
	 * enabled is what the status reports. Deframer only -- the framer's
	 * equivalent bits are not consulted anywhere in this driver.
	 */
	if (cfg->tx) {
		jesd_write(dev, JESD204_REG_SYSREF_STATUS, 0x3);
	}

	jesd_write(dev, JESD204_REG_LINK_DISABLE, 0x0);
	return 0;
}

/*
 * Per-lane desync check.
 *
 * Returns true if this lane needs the link restarted. For 8B/10B a *non-zero*
 * low two bits means the lane is fine (CGS/frame sync achieved); all-zero means
 * it has lost alignment. The error counter is only present from PCORE minor 2
 * onward, so it is read for the log line but never gates the decision.
 */
static bool jesd_rx_check_lane_status(const struct device *dev, uint32_t lane)
{
	struct axi_jesd204_data *data = dev->data;
	uint32_t status = jesd_read(dev, JESD204_RX_REG_LANE_STATUS(lane));
	uint32_t errors = 0;

	/* This link is 8B/10B (JESD204B); the 64B/66B EMB path does not apply. */
	if ((status & 0x3) != 0x0) {
		return false;
	}

	if (PCORE_VER_MINOR(data->version) >= 2) {
		errors = jesd_read(dev, JESD204_RX_REG_LANE_ERRORS(lane));
	}

	LOG_WRN("%s: lane %u desynced (status 0x%08x, %u errors), restarting link",
		dev->name, lane, status, errors);
	return true;
}

/*
 * Link watchdog, called once after bring-up.
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
 * The DATA test masks the state field rather than comparing the whole status
 * word, so a core that sets status bits above the state field still gets
 * checked instead of silently skipped.
 */
int axi_jesd204_rx_watchdog(const struct device *dev)
{
	const struct axi_jesd204_config *cfg;
	struct axi_jesd204_data *data;
	bool restart = false;

	if (!device_is_ready(dev)) {
		return -ENODEV;
	}
	cfg = dev->config;
	data = dev->data;

	/*
	 * Framer-only: LANE_STATUS is an axi_jesd204_rx register. On the
	 * deframer's map that offset is an ILAS word, so reading it there would
	 * "pass" against link configuration rather than lane state.
	 */
	if (cfg->tx) {
		LOG_ERR("%s: lane watchdog is RX-only", dev->name);
		return -ENOTSUP;
	}

	if (jesd_read(dev, JESD204_RG_LINK_STATE) & 0x1) {
		return 0; /* link disabled -- nothing to police */
	}

	if ((jesd_read(dev, JESD204_REG_LINK_STATUS) & 0x3) !=
	    JESD204_LINK_STATUS_DATA) {
		return 0;
	}

	/*
	 * num_lanes, not cfg->l -- the opposite choice from the ILAS loop in
	 * configure(). LANE_STATUS is a real register per *synthesised* lane, so
	 * iterating a larger declared L would read past the core's map. ILAS is
	 * the reverse case: those words are what we announce, so every declared
	 * lane must get them even if the bitstream has fewer.
	 */
	for (uint32_t lane = 0; lane < data->num_lanes; lane++) {
		restart |= jesd_rx_check_lane_status(dev, lane);
	}

	if (restart) {
		jesd_write(dev, JESD204_REG_LINK_DISABLE, 0x1);
		k_msleep(100);
		jesd_write(dev, JESD204_REG_LINK_DISABLE, 0x0);
		return -EAGAIN;
	}

	LOG_DBG("%s: all %u lanes in sync", dev->name, data->num_lanes);
	return 0;
}

/* TX deframer link-state label (8B/10B: WAIT/CGS/ILAS/DATA). */
static const char *const tx_status_label[] = { "WAIT", "CGS", "ILAS", "DATA" };
/* RX framer link-state label (8B/10B: RESET/WAIT-PHY/CGS/DATA). */
static const char *const rx_status_label[] = { "RESET", "WAIT_PHY", "CGS",
					       "DATA" };

bool axi_jesd204_link_is_data(const struct device *dev)
{
	uint32_t status;

	/* No errno to return: an unusable core is not carrying DATA. */
	if (!device_is_ready(dev)) {
		return false;
	}

	status = jesd_read(dev, JESD204_REG_LINK_STATUS);

	return (status & 0x3) == JESD204_LINK_STATUS_DATA;
}

/*
 * Print the multi-line link status table. MHz values are fixed-point
 * (whole.thousandths) because this target has no libc float printf support wired
 * up for %f.
 *
 * "Reported Link Clock" is computed as lane_rate_khz / 40 rather than read from a
 * clock provider: this driver has no clock phandle, and for 8B/10B the link clock
 * is the lane rate over 40 by definition.
 */
static void jesd_status_table(const struct device *dev, uint32_t state,
			       uint32_t status)
{
	const struct axi_jesd204_config *cfg = dev->config;
	uint32_t sysref_status = jesd_read(dev, JESD204_REG_SYSREF_STATUS);
	uint32_t clk_ratio = jesd_read(dev, JESD204_REG_LINK_CLK_RATIO);
	uint32_t sysref_conf = jesd_read(dev, JESD204_REG_SYSREF_CONF);
	uint32_t link_conf0 = jesd_read(dev, JESD204_REG_CONF0);
	uint32_t clk_rate, link_rate, lmfc_rate;

	LOG_INF("%s status:", dev->name);
	LOG_INF("  Link is %s", (state & 0x1) ? "disabled" : "enabled");

	if (clk_ratio == 0) {
		LOG_INF("  Measured Link Clock: off");
	} else {
		uint64_t r = (100000ULL * clk_ratio + (1ULL << 15)) >> 16;

		LOG_INF("  Measured Link Clock: %u.%03u MHz",
			(uint32_t)(r / 1000), (uint32_t)(r % 1000));
	}

	clk_rate = cfg->lane_rate_khz / 40;
	LOG_INF("  Reported Link Clock: %u.%03u MHz", clk_rate / 1000,
		clk_rate % 1000);

	if (state & 0x1) {
		LOG_INF("  External reset is %s",
			(state & 0x2) ? "asserted" : "deasserted");
		return;
	}

	clk_rate = cfg->lane_rate_khz;
	link_rate = (clk_rate + 20) / 40;
	lmfc_rate = clk_rate / (10 * ((link_conf0 & 0xff) + 1));

	LOG_INF("  Lane rate: %u.%03u MHz", clk_rate / 1000, clk_rate % 1000);
	LOG_INF("  Lane rate / 40: %u.%03u MHz", link_rate / 1000,
		link_rate % 1000);
	LOG_INF("  LMFC rate: %u.%03u MHz", lmfc_rate / 1000,
		lmfc_rate % 1000);

	if (cfg->tx) {
		LOG_INF("  SYNC~: %s", (status & 0x10) ? "deasserted" : "asserted");
	}

	LOG_INF("  Link status: %s",
		cfg->tx ? tx_status_label[status & 0x3]
			: rx_status_label[status & 0x3]);
	LOG_INF("  SYSREF captured: %s",
		(sysref_conf & JESD204_SYSREF_CONF_SYSREF_DISABLE) ? "disabled"
		: (sysref_status & 1) ? "Yes" : "No");
	LOG_INF("  SYSREF alignment error: %s",
		(sysref_conf & JESD204_SYSREF_CONF_SYSREF_DISABLE) ? "disabled"
		: (sysref_status & 2) ? "Yes" : "No");
}

int axi_jesd204_status_read(const struct device *dev)
{
	uint32_t state, status;

	if (!device_is_ready(dev)) {
		return -ENODEV;
	}

	state = jesd_read(dev, JESD204_RG_LINK_STATE) & 0x3;
	status = jesd_read(dev, JESD204_REG_LINK_STATUS);

	jesd_status_table(dev, state, status);

	/* Carrying DATA (0x3) is the healthy end state. */
	if ((status & 0x3) == JESD204_LINK_STATUS_DATA) {
		return 0;
	}
	return -EIO;
}

/*
 * init() only exists so device_is_ready() means something. The real work is an
 * explicit axi_jesd204_configure() call from the bring-up sequence, in the order
 * relative to the transceiver and transport cores that it needs -- init-level
 * ordering cannot express that.
 */
static int axi_jesd204_init(const struct device *dev)
{
	ARG_UNUSED(dev);
	return 0;
}

/*
 * N, CS and S are TX-only: they reach a register solely through the ILAS words
 * and their checksum, which only the deframer emits. The framer receives the
 * ILAS it is described in, so its node does not carry them and they are passed
 * as 0 here. HD derives from the same three, so it is 0 on RX for the same
 * reason.
 */
#define JESD204_DEFINE(node, is_tx, magic_val, n_val, cs_val, s_val)                               \
	static struct axi_jesd204_data axi_jesd204_data_##node;                                    \
                                                                                                   \
	static const struct axi_jesd204_config axi_jesd204_config_##node = {                       \
		.base = DT_REG_ADDR(node),                                                         \
		.magic = (magic_val),                                                              \
		.tx = (is_tx),                                                                     \
		.l = DT_PROP(node, adi_lanes_per_device),                                          \
		.m = DT_PROP(node, adi_converters_per_device),                                     \
		.f = DT_PROP(node, adi_octets_per_frame),                                          \
		.k = DT_PROP(node, adi_frames_per_multiframe),                                     \
		.n = (n_val),                                                                      \
		.np = DT_PROP(node, adi_bits_per_sample),                                          \
		.s = (s_val),                                                                      \
		.cs = (cs_val),                                                                    \
		.hd = JESD204_DERIVE_HD(DT_PROP(node, adi_converters_per_device), (s_val),         \
					DT_PROP(node, adi_bits_per_sample),                        \
					DT_PROP(node, adi_lanes_per_device)),                      \
		.subclass = DT_PROP(node, adi_subclass),                                           \
		.lane_rate_khz = DT_PROP(node, adi_lane_rate_khz),                                 \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_DEFINE(node, axi_jesd204_init, NULL, &axi_jesd204_data_##node,                   \
			 &axi_jesd204_config_##node, POST_KERNEL,                                  \
			 CONFIG_JESD204_AXI_LINK_INIT_PRIORITY, NULL);

#define JESD204_DEFINE_TX(node)                                                \
	JESD204_DEFINE(node, true, JESD204_TX_MAGIC,                           \
		       DT_PROP(node, adi_converter_resolution),                 \
		       DT_PROP(node, adi_control_bits_per_sample),              \
		       DT_PROP(node, adi_samples_per_converter_per_frame))

#define JESD204_DEFINE_RX(node) JESD204_DEFINE(node, false, JESD204_RX_MAGIC, 0, 0, 0)

DT_FOREACH_STATUS_OKAY(adi_axi_jesd204_rx_1_0, JESD204_DEFINE_RX)
DT_FOREACH_STATUS_OKAY(adi_axi_jesd204_tx_1_0, JESD204_DEFINE_TX)

/*
 * The framer and the deframer are two ends of ONE physical link, so every
 * parameter both nodes carry has to agree: the deframer announces it in ILAS and
 * the framer is configured against it. Split properties are what makes that
 * checkable at all -- and also what makes it possible to get wrong, so it is
 * checked rather than trusted.
 *
 * Driven off adi,jesd204-peer rather than fixed node labels, so a board with
 * differently-named nodes, or a second link, still gets checked.
 *
 * tools/check_profile.py does the rest: each side against the bitstream's make
 * parameters, and HD against the derivation.
 */
#define JESD204_ASSERT_SAME(node, peer, prop)                                                      \
	BUILD_ASSERT(DT_PROP(node, prop) == DT_PROP(peer, prop),                                   \
		     "RX and TX link cores describe one link: " #prop " must match");

/*
 * F is not free either: it is the per-lane octet count the rest of the geometry
 * implies. A node stating something else would program CONF0 and ILAS word 1
 * with a frame length the converters do not produce.
 *
 * And HD must come out 0 for this profile. Asserted separately from the
 * derivation macro so a future geometry that genuinely needs HD=1 has to be
 * acknowledged here: the chip-side jesd_param structs in ad9081.c carry HD as a
 * literal and would otherwise silently disagree with what the core advertises.
 */
#define JESD204_ASSERT_LINK(node)                                                                  \
	JESD204_ASSERT_SAME(node, DT_PHANDLE(node, adi_jesd204_peer), adi_lanes_per_device)        \
	JESD204_ASSERT_SAME(node, DT_PHANDLE(node, adi_jesd204_peer), adi_converters_per_device)   \
	JESD204_ASSERT_SAME(node, DT_PHANDLE(node, adi_jesd204_peer), adi_octets_per_frame)        \
	JESD204_ASSERT_SAME(node, DT_PHANDLE(node, adi_jesd204_peer), adi_frames_per_multiframe)   \
	JESD204_ASSERT_SAME(node, DT_PHANDLE(node, adi_jesd204_peer), adi_bits_per_sample)         \
	JESD204_ASSERT_SAME(node, DT_PHANDLE(node, adi_jesd204_peer), adi_subclass)                \
	BUILD_ASSERT(DT_PROP(node, adi_octets_per_frame) ==                                        \
			     DT_PROP(node, adi_converters_per_device) *                            \
				     DT_PROP(node, adi_samples_per_converter_per_frame) *          \
				     DT_PROP(node, adi_bits_per_sample) / 8 /                      \
				     DT_PROP(node, adi_lanes_per_device),                          \
		     "F must equal M*S*NP/8/L");                                                   \
	BUILD_ASSERT(JESD204_DERIVE_HD(DT_PROP(node, adi_converters_per_device),                   \
				       DT_PROP(node, adi_samples_per_converter_per_frame),         \
				       DT_PROP(node, adi_bits_per_sample),                         \
				       DT_PROP(node, adi_lanes_per_device)) == 0,                  \
		     "HD derives to 1: both ends take it from JESD204_DERIVE_HD, so this "         \
		     "assert and its two users have to be re-checked together");

/* Deframer side only: it is the end that carries N, CS and S. */
DT_FOREACH_STATUS_OKAY(adi_axi_jesd204_tx_1_0, JESD204_ASSERT_LINK)
