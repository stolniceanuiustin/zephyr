/*
 * Copyright (c) 2026 Analog Devices, Inc.
 *
 * AD9144 -- 16-bit, quad-channel JESD204B DAC -- Zephyr SPI bring-up driver.
 *
 * On the DAQ2 FMC the AD9144 is the TX converter; its sample clock (DAC_CLK)
 * comes from the AD9523-1 clock generator (see clock_control_ad9523.c), which
 * initialises earlier at POST_KERNEL. This driver scope is the standalone SPI
 * bring-up only: prove the bus by reading the product ID, soft-reset the chip,
 * apply the ad9144_setup() register sequence, and report the SERDES PLL lock
 * status. It does NOT wire the AXI JESD204 link cores, the transceivers, or the
 * jesd204 FSM -- those are a separate later block, and until they land the
 * actual DAC output / lane sync is UNVERIFIED.
 *
 * Faithful port of no-OS ad9144.c (DAQ2/AD9144, reference-hierarchy tier-3) for
 * the register sequence, scoped to the DAQ2-exercised path only: CHIPID_AD9144,
 * pll_enable=false (external DAC clock, on-chip PLL off), interpolation=1. The
 * AD9152/9135/9136 branches, the DAC-PLL setup, and the iio/sample-rate
 * machinery are dropped -- mirrors how the AD9680 port was scoped.
 *
 * The one deliberate departure from no-OS: the JESD204B ILAS configuration
 * bytes are DERIVED from the devicetree geometry rather than hardcoded, so a
 * geometry that disagrees with the synthesised bitstream cannot be set. Each
 * derived byte is checked at build time against the known-good no-OS constant,
 * so a wrong encoding fails the build instead of reaching hardware. See the
 * DERIVED BYTES block below.
 *
 * HD note (CLAUDE.md records that no-OS's AD9081 HD was wrong): that case was a
 * different geometry. For the AD9144 DAQ2 profile HD = (F*8 < N') = (8 < 16) = 1,
 * which AGREES with no-OS mode-4 hd=1. There is no HD mismatch here.
 *
 * INIT LEVEL: POST_KERNEL, not the PRE_KERNEL_1 default, because it is
 * SPI-attached and Zephyr SPI controllers initialise at
 * POST_KERNEL/CONFIG_SPI_INIT_PRIORITY. A BUILD_ASSERT enforces the ordering.
 *
 * Property names in the binding follow ADI's Linux adi,ad9144 binding so a
 * devicetree written for ADI Linux transfers with only syntax changes.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT adi_ad9144

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/sys/util.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ad9144, LOG_LEVEL_INF);

/* -------------------- AD9144 registers (from no-OS ad9144.h) --------------- */

/* SPI / identity. */
#define AD9144_REG_SPI_INTFCONFA	0x000
#define AD9144_REG_SPI_PRODIDL		0x004
#define AD9144_REG_SPI_PRODIDH		0x005
#define AD9144_REG_SPI_CHIPGRADE	0x006
#define AD9144_REG_SPI_SCRATCHPAD	0x00A

/* Power / clocks / sysref. */
#define AD9144_REG_PWRCNTRL0		0x011
#define AD9144_REG_CLKCFG0		0x080
#define AD9144_REG_SYSREF_ACTRL0	0x081

/* Datapath. */
#define AD9144_REG_DATA_FORMAT		0x110
#define AD9144_REG_DATAPATH_CTRL	0x111
#define AD9144_REG_INTERP_MODE		0x112

/* DAC calibration (datasheet Rev. B table 86). */
#define AD9144_REG_CAL_CLKDIV		0x0E7
#define AD9144_REG_CAL_INDX		0x0E8
#define AD9144_REG_CAL_CTRL		0x0E9
#define AD9144_REG_CAL_INIT		0x0ED

/* SERDES / CDR / receive PLL. */
#define AD9144_REG_CDR_RESET		0x206	/* unnamed in no-OS header */
#define AD9144_REG_CDR_OPERATING_MODE	0x230
#define AD9144_REG_SYNTH_ENABLE_CNTRL	0x280
#define AD9144_REG_PLL_STATUS		0x281
#define AD9144_REG_REF_CLK_DIVIDER_LDO	0x289
#define AD9144_REG_TERM_BLK1_CTRLREG0	0x2A7
#define AD9144_REG_TERM_BLK2_CTRLREG0	0x2AE

/* JESD input termination (values from datasheet, unnamed in no-OS header). */
#define AD9144_REG_JESD_TERM_CTRL_A	0x2AA
#define AD9144_REG_JESD_TERM_CTRL_B	0x2AB
#define AD9144_REG_JESD_TERM_CTRL_C	0x2B1
#define AD9144_REG_JESD_TERM_CTRL_D	0x2B2

/* Link / transport / sync. */
#define AD9144_REG_SYNC_CTRL		0x03A
#define AD9144_REG_MASTER_PD		0x200
#define AD9144_REG_PHY_PD		0x201
#define AD9144_REG_EQ_BIAS		0x268
#define AD9144_REG_GENERAL_JRX_CTRL_0	0x300
#define AD9144_REG_GENERAL_JRX_CTRL_1	0x301
#define AD9144_REG_SYNCB_GEN_1		0x312
#define AD9144_REG_PCLK_CTRL		0x314	/* unnamed in no-OS header */

/* LMFC / receive-buffer delay. */
#define AD9144_REG_LMFC_DELAY_0		0x304
#define AD9144_REG_LMFC_DELAY_1		0x305
#define AD9144_REG_LMFC_VAR_0		0x306
#define AD9144_REG_LMFC_VAR_1		0x307

/* Lane crossbar: REG_XBAR(i) holds the two lanes routed to PHY pair i. */
#define AD9144_REG_XBAR(x)		(0x308 + (x))

/* ILAS / link config (REG_ILS_*). */
#define AD9144_REG_ILS_DID		0x450
#define AD9144_REG_ILS_BID		0x451
#define AD9144_REG_ILS_SCR_L		0x453
#define AD9144_REG_ILS_F		0x454
#define AD9144_REG_ILS_K		0x455
#define AD9144_REG_ILS_M		0x456
#define AD9144_REG_ILS_CS_N		0x457
#define AD9144_REG_ILS_NP		0x458
#define AD9144_REG_ILS_S		0x459
#define AD9144_REG_ILS_HD_CF		0x45A
#define AD9144_REG_LANEDESKEW		0x46C
#define AD9144_REG_CTRLREG1		0x476
#define AD9144_REG_KVAL			0x478
#define AD9144_REG_LANEENABLE		0x47D

/* -------------------- AD9144 register values / bitfields ------------------- */

/* SPI instruction: bit15 = read, bits[14:0] = address (MSB first). */
#define AD9144_SPI_READ			0x80

/* INTERFACE_CONF_A: soft reset (self-clearing) + its LSB-first mirror bit. */
#define AD9144_SOFTRESET		BIT(0)
#define AD9144_SOFTRESET_M		BIT(7)
/* Post-reset config: 0x18 selects 4-wire SPI, 0x00 keeps 3-wire (DAQ2 board). */
#define AD9144_INTFCONFA_3WIRE		0x00
#define AD9144_INTFCONFA_4WIRE		0x18

/* Bus-proof: product ID 0x9144, chip grade 0 (see AD9144_PROD_ID below). */
#define AD9144_PROD_IDH			0x91
#define AD9144_PROD_IDL			0x44
#define AD9144_CHIP_GRADE		0x00
#define AD9144_SCRATCH_TEST		0xAD

/* Datapath. */
#define AD9144_MOD_TYPE_MASK		(0x3 << 2)
#define AD9144_MOD_TYPE_NONE		(0x0 << 2)	/* NCO mixing off */
#define AD9144_DATA_FORMAT_2S_COMP	0x00
#define AD9144_INTERP_MODE_1X		0x00

/* SYSREF_ACTRL0: BIT4 set when NOT subclass-1; BIT2 set for rising-edge capture. */
#define AD9144_SYSREF_ACTRL0_NO_SUBCLASS	BIT(4)
#define AD9144_SYSREF_ACTRL0_RISING_EDGE	BIT(2)
#define AD9144_SYSREF_ACTRL0_PWRUP		0x00	/* power up, falling edge */

/* SYNC_CTRL arm sequence: continuous mode, then enable, then arm. */
#define AD9144_SYNC_MODE_ONESHOT	0x01
#define AD9144_SYNC_MODE_CONTINUOUS	0x02
#define AD9144_SYNCENABLE		BIT(7)
#define AD9144_SYNCARM			BIT(6)

/* DAC calibration (table 86). */
#define AD9144_CAL_FIN			BIT(7)	/* calibration finished */
#define AD9144_CAL_ACTIVE		BIT(6)	/* calibration in progress */
#define AD9144_CAL_ERRHI		BIT(5)	/* SAR data error: too high */
#define AD9144_CAL_ERRLO		BIT(4)	/* SAR data error: too low */
#define AD9144_CAL_CLKDIV_ON		0x38	/* calibration clock enabled */
#define AD9144_CAL_CLKDIV_OFF		0x30	/* calibration clock off */
#define AD9144_CAL_INIT_VAL		0xA2	/* initial SAR value */
#define AD9144_CAL_CTRL_ENABLE		0x01
#define AD9144_CAL_CTRL_START		0x03
#define AD9144_CAL_POLL_TRIES		30
#define AD9144_CAL_SETTLE_MS		10

/* Receive PLL. */
#define AD9144_PLL_STATUS_LOCKED	BIT(0)
#define AD9144_SYNTH_PLL_DISABLE	0x00
#define AD9144_SYNTH_PLL_ENABLE		0x01
#define AD9144_PLL_SETTLE_MS		20

/* Input termination calibration trigger. */
#define AD9144_TERM_CAL_TRIGGER		0x01

/* CDR reset is level-triggered: assert 0 then release to 1. */
#define AD9144_CDR_RESET_ASSERT		0x00
#define AD9144_CDR_RESET_RELEASE	0x01

/*
 * SERDES CDR divider and receive-PLL divider, selected by lane rate from
 * datasheet Rev. B table 4. Only the >= 5.75 Gbps entry is exercised by DAQ2
 * (10 Gbps); the lower-rate entries are carried for completeness.
 */
#define AD9144_SERDES_RATE_HALF_KBPS	2880000U
#define AD9144_SERDES_RATE_FULL_KBPS	5750000U
#define AD9144_CDR_DIV_QUARTER		0x0A
#define AD9144_CDR_DIV_HALF		0x08
#define AD9144_CDR_DIV_FULL		0x28
#define AD9144_PLLDIV_QUARTER		0x06
#define AD9144_PLLDIV_HALF		0x05
#define AD9144_PLLDIV_FULL		0x04

/* Fixed control words from no-OS (datasheet-derived, single-use). */
#define AD9144_JESD_TERM_A_VAL		0xB7
#define AD9144_JESD_TERM_B_VAL		0x87
#define AD9144_EQ_BIAS_VAL		0x62
#define AD9144_PCLK_MASTER		0x01	/* pclk == QBD master clock */
#define AD9144_KVAL_STATIC		0x01
#define AD9144_LMFC_DELAY_VAL		0x00
#define AD9144_LMFC_VAR_VAL		0x0A	/* receive-buffer delay */
#define AD9144_LINK_ENABLE		0x01
#define AD9144_LINK_SINGLE		0x00	/* single link, link 0 */

/*
 * Lane crossbar: DAQ2/ZCU102 uses the IDENTITY map (PHY lane i carries logical
 * lane i). This is board wiring, confirmed from ADI's Linux DAQ2 devicetree
 * (adi-daq2.dtsi / zynqmp-zcu102-rev10-fmcdaq2.dts): no adi,logic-lanes /
 * xbar override is present, so the driver default (identity) stands. It is a
 * hardcoded board fact here, not a DT knob. Counter-example, for the record:
 * the VC707 FMCDAQ2 carrier remaps lanes {2,3,0,1}, which is why this must be
 * per-board and cannot be assumed universally.
 */
static const uint8_t ad9144_xbar_identity[8] = { 0, 1, 2, 3, 4, 5, 6, 7 };

/* -------------------------- config / data --------------------------------- */

/*
 * JESD204B link geometry, from devicetree. These describe the link the DAQ2 TX
 * bitstream was synthesised for; they are used to DERIVE the ILAS config bytes
 * (see the DERIVED BYTES block) and are BUILD_ASSERT-ed against the bitstream
 * profile in AD9144_DEFINE.
 */
struct ad9144_config {
	struct spi_dt_spec spi;

	uint32_t lane_rate_kbps;	/**< Serial lane rate in kbps. */
	uint8_t num_converters;		/**< M: converters per device. */
	uint8_t num_lanes;		/**< L: lanes per device. */
	uint8_t octets_per_frame;	/**< F: octets per frame per lane. */
	uint8_t frames_per_multiframe;	/**< K: frames per multiframe. */
	uint8_t converter_resolution;	/**< N: converter resolution, bits. */
	uint8_t bits_per_sample;	/**< N': total bits per sample. */
	uint8_t subclass;		/**< JESD204B device subclass. */
	uint8_t interpolation;		/**< Datapath interpolation factor. */
};

/** @brief Runtime state for one AD9144 instance. */
struct ad9144_data {
	bool spi_ok;			/**< Product ID read back correctly. */
	bool serdes_pll_locked;		/**< SERDES PLL reported locked at setup. */
};

/* ------------------------------ SPI helpers ------------------------------- */

/*
 * Read one register. The AD9144 SPI frame is 3 bytes: a 16-bit instruction
 * (bit15 = read, bits[14:0] = address, MSB first) followed by one data byte.
 * Faithful port of no-OS ad9144_read().
 */
static int ad9144_spi_read(const struct device *dev, uint16_t reg_addr,
			   uint8_t *reg_data)
{
	const struct ad9144_config *config = dev->config;
	uint8_t tx[3] = {
		AD9144_SPI_READ | (reg_addr >> 8),
		reg_addr & 0xFF,
		0x00,
	};
	uint8_t rx[3] = { 0 };
	const struct spi_buf txb = { .buf = tx, .len = sizeof(tx) };
	const struct spi_buf rxb = { .buf = rx, .len = sizeof(rx) };
	const struct spi_buf_set txs = { .buffers = &txb, .count = 1 };
	const struct spi_buf_set rxs = { .buffers = &rxb, .count = 1 };
	int ret;

	if (reg_data == NULL) {
		return -EINVAL;
	}

	ret = spi_transceive_dt(&config->spi, &txs, &rxs);
	if (ret < 0) {
		return ret;
	}

	*reg_data = rx[2];

	return 0;
}

/*
 * Write one register. 3-byte frame: 16-bit address (MSB first, bit15 = 0 for
 * write) then the data byte. Faithful port of no-OS ad9144_write().
 */
static int ad9144_spi_write(const struct device *dev, uint16_t reg_addr,
			    uint8_t reg_data)
{
	const struct ad9144_config *config = dev->config;
	uint8_t tx[3] = {
		reg_addr >> 8,
		reg_addr & 0xFF,
		reg_data,
	};
	const struct spi_buf txb = { .buf = tx, .len = sizeof(tx) };
	const struct spi_buf_set txs = { .buffers = &txb, .count = 1 };

	return spi_write_dt(&config->spi, &txs);
}

/* Read-modify-write helper: clear the bits in @mask, set the bits in @val. */
static int ad9144_spi_update(const struct device *dev, uint16_t reg_addr,
			     uint8_t mask, uint8_t val)
{
	uint8_t reg;
	int ret;

	ret = ad9144_spi_read(dev, reg_addr, &reg);
	if (ret < 0) {
		return ret;
	}

	reg = (reg & ~mask) | (val & mask);

	return ad9144_spi_write(dev, reg_addr, reg);
}

/* ----------------------------- DERIVED BYTES ------------------------------ */

/*
 * The JESD204B ILAS configuration registers are derived from the link geometry
 * rather than hardcoded, so no geometry can be set that disagrees with the bytes
 * actually written. Each macro below is a compile-time function of the DT
 * geometry, and each is BUILD_ASSERT-ed in AD9144_DEFINE against the exact byte
 * the tier-3 no-OS reference writes for the DAQ2 profile -- the derivation is the
 * intent, the assert is the tier-3 check. A wrong encoding fails the build.
 *
 *   SCR_L  (0x453): bit7 = scrambling, bits[4:0] = L-1        -> L=4,scr -> 0x83
 *   ILS_F  (0x454): F-1                                       -> F=1     -> 0x00
 *   ILS_K  (0x455): K-1                                       -> K=32    -> 0x1f
 *   ILS_M  (0x456): M-1                                       -> M=2     -> 0x01
 *   ILS_CS_N (0x457): bits[7:6]=CS, bits[4:0]=N-1        -> CS=0,N=16 -> 0x0f
 *   ILS_NP (0x458): bits[7:5]=subclass, bits[4:0]=N'-1   -> SC=1,N'=16-> 0x2f
 *   ILS_S  (0x459): bit5=JESDVER(=B), bits[4:0]=S-1      -> S=1       -> 0x20
 *   ILS_HD_CF (0x45A): bit7 = high-density               -> HD=1      -> 0x80
 *
 * The AD9144 is 16-bit, so CS = N'-N = 0 and N-1 = 15 -- this differs from the
 * AD9680 (N=14, CS=2). HD is derived, not copied: HD = (F*8 < N') = (8 < 16) = 1,
 * which agrees with no-OS mode-4. S is not an independent property either: it is
 * S = F*8*L/(M*N'), and the derivation is BUILD_ASSERT-ed to 1.
 */
#define AD9144_SCR_L_VAL(l, scr)	((((l) - 1) & 0x1F) | ((scr) ? BIT(7) : 0))
#define AD9144_ILS_F_VAL(f)		(((f) - 1) & 0xFF)
#define AD9144_ILS_K_VAL(k)		(((k) - 1) & 0x1F)
#define AD9144_ILS_M_VAL(m)		(((m) - 1) & 0xFF)
#define AD9144_ILS_CS_N_VAL(cs, n)	((((cs) & 0x3) << 6) | (((n) - 1) & 0x1F))
#define AD9144_ILS_NP_VAL(sc, np)	((((sc) & 0x7) << 5) | (((np) - 1) & 0x1F))
#define AD9144_ILS_S_VAL(s)		(BIT(5) | (((s) - 1) & 0x1F))
#define AD9144_ILS_HD_CF_VAL(hd)	((hd) ? BIT(7) : 0)

/* S and HD are functions of the geometry, not free parameters. */
#define AD9144_S_DERIVED(f, l, m, np)	(((uint32_t)(f) * 8U * (l)) / ((uint32_t)(m) * (np)))
#define AD9144_HD_DERIVED(f, np)	(((uint32_t)(f) * 8U < (np)) ? 1 : 0)

/*
 * Serial lane rate in kbps, DERIVED from the sample rate and geometry -- it is a
 * function of the others, so it is not an independently settable property (see
 * CLAUDE.md). For 8B/10B (10/8 overhead):
 *
 *   lane_rate = fs * M * N' * (10/8) / (L * interpolation)
 *
 * fs is carried in kHz to keep the arithmetic 32-bit; the cast to 64-bit guards
 * the intermediate product. DAQ2 TX: fs=1e6 kHz, M=2, N'=16, L=4, interp=1 -> 10 Gbps.
 */
#define AD9144_LANE_RATE_KBPS(fs_khz, m, np, l, interp)	\
	(((uint64_t)(fs_khz) * (m) * (np) * 10U) / (8U * (l) * (interp)))

/* ----------------------- datasheet register tables ------------------------ */

/** @brief One {register, value} pair for a datasheet write sequence. */
struct ad9144_reg_seq {
	uint16_t reg;	/**< Register address. */
	uint8_t val;	/**< Value to write. */
};

/*
 * Required device configuration, AD9144 datasheet Rev. B table 16. Opaque
 * factory settings with no per-register public semantics; carried verbatim from
 * no-OS ad9144_required_device_config[] so a reviewer can diff against tier-3.
 */
static const struct ad9144_reg_seq ad9144_required_device_config[] = {
	{ 0x12D, 0x8B }, { 0x146, 0x01 }, { 0x2A4, 0xFF },
	{ 0x232, 0xFF }, { 0x333, 0x01 },
};

/*
 * SERDES optimization, AD9144 datasheet Rev. B table 39 (9144-specific pair).
 * Carried verbatim from the no-OS 0x9144 branch of ad9144_setup().
 */
static const struct ad9144_reg_seq ad9144_serdes_opt[] = {
	{ 0x296, 0x03 }, { 0x28A, 0x7B },
};

/*
 * Optimal SERDES PLL settings, AD9144 datasheet Rev. B table 39. Opaque PLL
 * tuning; carried verbatim from no-OS ad9144_optimal_serdes_settings[].
 */
static const struct ad9144_reg_seq ad9144_optimal_serdes_settings[] = {
	{ 0x284, 0x62 }, { 0x285, 0xC9 }, { 0x286, 0x0E }, { 0x287, 0x12 },
	{ 0x28B, 0x00 }, { 0x290, 0x89 }, { 0x294, 0x24 }, { 0x297, 0x0D },
	{ 0x299, 0x02 }, { 0x29A, 0x8E }, { 0x29C, 0x2A }, { 0x29F, 0x78 },
	{ 0x2A0, 0x06 },
};

static int ad9144_write_seq(const struct device *dev,
			    const struct ad9144_reg_seq *seq, size_t len)
{
	int ret;

	for (size_t i = 0; i < len; i++) {
		ret = ad9144_spi_write(dev, seq[i].reg, seq[i].val);
		if (ret < 0) {
			return ret;
		}
	}

	return 0;
}

/* ---------------------------- setup sub-blocks ---------------------------- */

/*
 * Block 1: soft-reset then prove the bus. Reset is done first (as no-OS does)
 * so the ID read happens from a known state. A wrong ID or scratchpad mismatch
 * is FATAL: it almost always means CS, SPI mode, 1 MHz or 3-/4-wire is wrong.
 */
static int ad9144_reset_and_probe(const struct device *dev)
{
	struct ad9144_data *data = dev->data;
	uint8_t idl, idh, grade, scratch;
	int ret;

	k_msleep(5);
	ret = ad9144_spi_write(dev, AD9144_REG_SPI_INTFCONFA,
			       AD9144_SOFTRESET_M | AD9144_SOFTRESET);
	if (ret < 0) {
		return ret;
	}
	/* DAQ2 board is 3-wire (bidirectional SDIO), like the AD9523/AD9680. */
	ret = ad9144_spi_write(dev, AD9144_REG_SPI_INTFCONFA,
			       AD9144_INTFCONFA_3WIRE);
	if (ret < 0) {
		return ret;
	}
	k_msleep(4);

	ret = ad9144_spi_read(dev, AD9144_REG_SPI_PRODIDL, &idl);
	if (ret < 0) {
		return ret;
	}
	ret = ad9144_spi_read(dev, AD9144_REG_SPI_PRODIDH, &idh);
	if (ret < 0) {
		return ret;
	}
	ret = ad9144_spi_read(dev, AD9144_REG_SPI_CHIPGRADE, &grade);
	if (ret < 0) {
		return ret;
	}
	grade >>= 4;	/* chip grade is in the 4 MSBs */

	if (idh != AD9144_PROD_IDH || idl != AD9144_PROD_IDL ||
	    grade != AD9144_CHIP_GRADE) {
		LOG_ERR("invalid ID 0x%02x%02x grade 0x%x (expected 0x9144 grade 0)",
			idh, idl, grade);
		LOG_ERR("check CS1 / SPI mode 0 / 1 MHz / 3-vs-4-wire");
		return -ENODEV;
	}

	/* Scratchpad round-trips to confirm writes land, not just reads. */
	ret = ad9144_spi_write(dev, AD9144_REG_SPI_SCRATCHPAD,
			       AD9144_SCRATCH_TEST);
	if (ret < 0) {
		return ret;
	}
	ret = ad9144_spi_read(dev, AD9144_REG_SPI_SCRATCHPAD, &scratch);
	if (ret < 0) {
		return ret;
	}
	if (scratch != AD9144_SCRATCH_TEST) {
		LOG_ERR("scratchpad readback 0x%02x != 0x%02x", scratch,
			AD9144_SCRATCH_TEST);
		return -EIO;
	}

	data->spi_ok = true;
	LOG_INF("SUCCESS: AD9144 chip ID 0x%02x%02x", idh, idl);

	return 0;
}

/*
 * Block 2: power up the DACs/clocks and apply the datasheet device-config and
 * SERDES-optimisation tables. pd_dac/pd_clk are functions of M (derived, as in
 * no-OS): for M=2 they resolve to PWRCNTRL0=0x18, CLKCFG0=0x40.
 */
static int ad9144_power_up(const struct device *dev)
{
	const struct ad9144_config *config = dev->config;
	uint8_t pd_dac = GENMASK(6 - config->num_converters, 3);
	uint8_t pd_clk = GENMASK(7 - (uint8_t)DIV_ROUND_UP(config->num_converters, 2), 6);
	int ret;

	ret = ad9144_spi_write(dev, AD9144_REG_GENERAL_JRX_CTRL_0,
			       AD9144_LINK_SINGLE);
	if (ret < 0) {
		return ret;
	}
	ret = ad9144_spi_write(dev, AD9144_REG_PWRCNTRL0, pd_dac);
	if (ret < 0) {
		return ret;
	}
	ret = ad9144_spi_write(dev, AD9144_REG_CLKCFG0, pd_clk);
	if (ret < 0) {
		return ret;
	}
	ret = ad9144_spi_write(dev, AD9144_REG_SYSREF_ACTRL0,
			       AD9144_SYSREF_ACTRL0_PWRUP);
	if (ret < 0) {
		return ret;
	}

	ret = ad9144_spi_write(dev, AD9144_REG_JESD_TERM_CTRL_A,
			       AD9144_JESD_TERM_A_VAL);
	if (ret < 0) {
		return ret;
	}
	ret = ad9144_spi_write(dev, AD9144_REG_JESD_TERM_CTRL_B,
			       AD9144_JESD_TERM_B_VAL);
	if (ret < 0) {
		return ret;
	}
	ret = ad9144_spi_write(dev, AD9144_REG_PCLK_CTRL, AD9144_PCLK_MASTER);
	if (ret < 0) {
		return ret;
	}

	ret = ad9144_write_seq(dev, ad9144_required_device_config,
			       ARRAY_SIZE(ad9144_required_device_config));
	if (ret < 0) {
		return ret;
	}
	ret = ad9144_write_seq(dev, ad9144_serdes_opt,
			       ARRAY_SIZE(ad9144_serdes_opt));
	if (ret < 0) {
		return ret;
	}
	/* 9144 SERDES-opt termination pair (table 39). */
	ret = ad9144_spi_write(dev, AD9144_REG_JESD_TERM_CTRL_C,
			       AD9144_JESD_TERM_A_VAL);
	if (ret < 0) {
		return ret;
	}
	ret = ad9144_spi_write(dev, AD9144_REG_JESD_TERM_CTRL_D,
			       AD9144_JESD_TERM_B_VAL);
	if (ret < 0) {
		return ret;
	}
	ret = ad9144_write_seq(dev, ad9144_optimal_serdes_settings,
			       ARRAY_SIZE(ad9144_optimal_serdes_settings));
	if (ret < 0) {
		return ret;
	}

	LOG_INF("SUCCESS: AD9144 device config");

	return 0;
}

/*
 * Block 3: digital datapath. interpolation=1 -> INTERP_MODE 1x; data format is
 * 2's complement. NCO mixing is off: the DAQ2 frequency-center-shift is 0, so
 * mod_type = NONE. (no-OS applies the NCO from setup_samplerate(); the registers
 * are independent, so it is grouped with the datapath here.)
 */
static int ad9144_setup_datapath(const struct device *dev)
{
	int ret;

	ret = ad9144_spi_write(dev, AD9144_REG_INTERP_MODE, AD9144_INTERP_MODE_1X);
	if (ret < 0) {
		return ret;
	}
	ret = ad9144_spi_write(dev, AD9144_REG_DATA_FORMAT,
			       AD9144_DATA_FORMAT_2S_COMP);
	if (ret < 0) {
		return ret;
	}
	ret = ad9144_spi_update(dev, AD9144_REG_DATAPATH_CTRL,
				AD9144_MOD_TYPE_MASK, AD9144_MOD_TYPE_NONE);
	if (ret < 0) {
		return ret;
	}

	LOG_INF("SUCCESS: AD9144 datapath");

	return 0;
}

/*
 * Block 4: transport + JESD204B link configuration. Powers up the master and
 * the used PHYs, programs the identity crossbar, and writes the geometry-derived
 * ILAS bytes. Faithful port of no-OS ad9144_setup_link() plus the equaliser.
 */
static int ad9144_setup_link(const struct device *dev)
{
	const struct ad9144_config *config = dev->config;
	uint8_t lane_mask = BIT(config->num_lanes) - 1;	/* L=4 -> 0x0f */
	uint8_t phy_mask = 0xFF;
	uint8_t sysref_actrl0 = 0;
	int ret;

	/* Power up only the PHYs carrying the used (identity-mapped) lanes. */
	for (uint8_t i = 0; i < config->num_lanes; i++) {
		phy_mask &= ~BIT(ad9144_xbar_identity[i]);
	}

	ret = ad9144_spi_write(dev, AD9144_REG_MASTER_PD, 0x00);
	if (ret < 0) {
		return ret;
	}
	ret = ad9144_spi_write(dev, AD9144_REG_PHY_PD, phy_mask);
	if (ret < 0) {
		return ret;
	}

	/* Identity crossbar: PHY pair i carries lanes 2i and 2i+1. */
	for (uint8_t i = 0; i < 4; i++) {
		uint8_t val = ad9144_xbar_identity[2 * i] |
			      (ad9144_xbar_identity[2 * i + 1] << 3);

		ret = ad9144_spi_write(dev, AD9144_REG_XBAR(i), val);
		if (ret < 0) {
			return ret;
		}
	}

	/* Subclass 1, SYSREF falling-edge capture (capture_falling_edge=0). */
	if (config->subclass == 0) {
		sysref_actrl0 |= AD9144_SYSREF_ACTRL0_NO_SUBCLASS;
	}
	sysref_actrl0 |= AD9144_SYSREF_ACTRL0_RISING_EDGE;
	ret = ad9144_spi_write(dev, AD9144_REG_SYSREF_ACTRL0, sysref_actrl0);
	if (ret < 0) {
		return ret;
	}
	ret = ad9144_spi_write(dev, AD9144_REG_GENERAL_JRX_CTRL_1,
			       config->subclass);
	if (ret < 0) {
		return ret;
	}

	/* DID/BID are 0 for the single-link DAQ2 profile. */
	ret = ad9144_spi_write(dev, AD9144_REG_ILS_DID, 0x00);
	if (ret < 0) {
		return ret;
	}
	ret = ad9144_spi_write(dev, AD9144_REG_ILS_BID, 0x00);
	if (ret < 0) {
		return ret;
	}

	/* Geometry-derived ILAS bytes (scrambling on, matching no-OS). */
	ret = ad9144_spi_write(dev, AD9144_REG_ILS_SCR_L,
			       AD9144_SCR_L_VAL(config->num_lanes, 1));
	if (ret < 0) {
		return ret;
	}
	ret = ad9144_spi_write(dev, AD9144_REG_ILS_F,
			       AD9144_ILS_F_VAL(config->octets_per_frame));
	if (ret < 0) {
		return ret;
	}
	ret = ad9144_spi_write(dev, AD9144_REG_ILS_K,
			       AD9144_ILS_K_VAL(config->frames_per_multiframe));
	if (ret < 0) {
		return ret;
	}
	ret = ad9144_spi_write(dev, AD9144_REG_ILS_M,
			       AD9144_ILS_M_VAL(config->num_converters));
	if (ret < 0) {
		return ret;
	}
	ret = ad9144_spi_write(dev, AD9144_REG_ILS_CS_N,
			       AD9144_ILS_CS_N_VAL(config->bits_per_sample -
						   config->converter_resolution,
						   config->converter_resolution));
	if (ret < 0) {
		return ret;
	}
	ret = ad9144_spi_write(dev, AD9144_REG_ILS_NP,
			       AD9144_ILS_NP_VAL(config->subclass,
						 config->bits_per_sample));
	if (ret < 0) {
		return ret;
	}
	ret = ad9144_spi_write(dev, AD9144_REG_ILS_S,
			       AD9144_ILS_S_VAL(AD9144_S_DERIVED(
				       config->octets_per_frame, config->num_lanes,
				       config->num_converters, config->bits_per_sample)));
	if (ret < 0) {
		return ret;
	}
	ret = ad9144_spi_write(dev, AD9144_REG_ILS_HD_CF,
			       AD9144_ILS_HD_CF_VAL(AD9144_HD_DERIVED(
				       config->octets_per_frame,
				       config->bits_per_sample)));
	if (ret < 0) {
		return ret;
	}

	ret = ad9144_spi_write(dev, AD9144_REG_KVAL, AD9144_KVAL_STATIC);
	if (ret < 0) {
		return ret;
	}
	ret = ad9144_spi_write(dev, AD9144_REG_LANEDESKEW, lane_mask);
	if (ret < 0) {
		return ret;
	}
	ret = ad9144_spi_write(dev, AD9144_REG_CTRLREG1, config->octets_per_frame);
	if (ret < 0) {
		return ret;
	}
	ret = ad9144_spi_write(dev, AD9144_REG_LANEENABLE, lane_mask);
	if (ret < 0) {
		return ret;
	}

	/*
	 * SYNC~ error-pulse length in PCLK cycles: 2 frame cycles = 2*F/4 PCLK.
	 * F=1 -> 0.5 PCLK -> field 0; the field lives in bits[7:4].
	 */
	ret = ad9144_spi_write(dev, AD9144_REG_SYNCB_GEN_1,
			       (config->octets_per_frame == 1 ? 0 :
				config->octets_per_frame == 2 ? 1 : 2) << 4);
	if (ret < 0) {
		return ret;
	}

	ret = ad9144_spi_write(dev, AD9144_REG_EQ_BIAS, AD9144_EQ_BIAS_VAL);
	if (ret < 0) {
		return ret;
	}

	LOG_INF("SUCCESS: AD9144 JESD204B link configured");

	return 0;
}

/*
 * Block 5: data-link LMFC delays and the SYNC~ arm sequence. DAQ2 uses
 * continuous SYSREF (not one-shot), so sync_mode = 0x02, then enable, then arm
 * (0x02, 0x82, 0xc2).
 */
static int ad9144_setup_datalink_sync(const struct device *dev)
{
	uint8_t sync_mode = AD9144_SYNC_MODE_CONTINUOUS;
	int ret;

	ret = ad9144_spi_write(dev, AD9144_REG_LMFC_DELAY_0, AD9144_LMFC_DELAY_VAL);
	if (ret < 0) {
		return ret;
	}
	ret = ad9144_spi_write(dev, AD9144_REG_LMFC_VAR_0, AD9144_LMFC_VAR_VAL);
	if (ret < 0) {
		return ret;
	}
	/* 9144 also programs link-1 LMFC registers. */
	ret = ad9144_spi_write(dev, AD9144_REG_LMFC_DELAY_1, AD9144_LMFC_DELAY_VAL);
	if (ret < 0) {
		return ret;
	}
	ret = ad9144_spi_write(dev, AD9144_REG_LMFC_VAR_1, AD9144_LMFC_VAR_VAL);
	if (ret < 0) {
		return ret;
	}

	ret = ad9144_spi_write(dev, AD9144_REG_SYNC_CTRL, sync_mode);
	if (ret < 0) {
		return ret;
	}
	ret = ad9144_spi_write(dev, AD9144_REG_SYNC_CTRL,
			       sync_mode | AD9144_SYNCENABLE);
	if (ret < 0) {
		return ret;
	}
	ret = ad9144_spi_write(dev, AD9144_REG_SYNC_CTRL,
			       sync_mode | AD9144_SYNCENABLE | AD9144_SYNCARM);
	if (ret < 0) {
		return ret;
	}

	LOG_INF("SUCCESS: AD9144 SYNC armed");

	return 0;
}

/*
 * Block 6: SERDES sample-rate config. The CDR and receive-PLL dividers are
 * selected by lane rate (datasheet table 4); DAQ2 at 10 Gbps takes the full-rate
 * entry (CDR 0x28, plldiv 0x04). The PLL lock read is NON-FATAL: full lock also
 * needs the lane clock from the link block that this bring-up does not wire, so
 * a not-locked report here is expected and stays UNVERIFIED until then.
 */
static int ad9144_setup_serdes(const struct device *dev)
{
	const struct ad9144_config *config = dev->config;
	struct ad9144_data *data = dev->data;
	uint8_t serdes_cdr, serdes_plldiv, pll_stat;
	int ret;

	if (config->lane_rate_kbps < AD9144_SERDES_RATE_HALF_KBPS) {
		serdes_cdr = AD9144_CDR_DIV_QUARTER;
		serdes_plldiv = AD9144_PLLDIV_QUARTER;
	} else if (config->lane_rate_kbps < AD9144_SERDES_RATE_FULL_KBPS) {
		serdes_cdr = AD9144_CDR_DIV_HALF;
		serdes_plldiv = AD9144_PLLDIV_HALF;
	} else {
		serdes_cdr = AD9144_CDR_DIV_FULL;
		serdes_plldiv = AD9144_PLLDIV_FULL;
	}

	ret = ad9144_spi_write(dev, AD9144_REG_SYNTH_ENABLE_CNTRL,
			       AD9144_SYNTH_PLL_DISABLE);
	if (ret < 0) {
		return ret;
	}
	ret = ad9144_spi_write(dev, AD9144_REG_TERM_BLK1_CTRLREG0,
			       AD9144_TERM_CAL_TRIGGER);
	if (ret < 0) {
		return ret;
	}
	ret = ad9144_spi_write(dev, AD9144_REG_TERM_BLK2_CTRLREG0,
			       AD9144_TERM_CAL_TRIGGER);
	if (ret < 0) {
		return ret;
	}
	ret = ad9144_spi_write(dev, AD9144_REG_CDR_OPERATING_MODE, serdes_cdr);
	if (ret < 0) {
		return ret;
	}
	ret = ad9144_spi_write(dev, AD9144_REG_CDR_RESET, AD9144_CDR_RESET_ASSERT);
	if (ret < 0) {
		return ret;
	}
	ret = ad9144_spi_write(dev, AD9144_REG_CDR_RESET, AD9144_CDR_RESET_RELEASE);
	if (ret < 0) {
		return ret;
	}
	ret = ad9144_spi_write(dev, AD9144_REG_REF_CLK_DIVIDER_LDO, serdes_plldiv);
	if (ret < 0) {
		return ret;
	}
	ret = ad9144_spi_write(dev, AD9144_REG_SYNTH_ENABLE_CNTRL,
			       AD9144_SYNTH_PLL_ENABLE);
	if (ret < 0) {
		return ret;
	}
	k_msleep(AD9144_PLL_SETTLE_MS);

	ret = ad9144_spi_read(dev, AD9144_REG_PLL_STATUS, &pll_stat);
	if (ret < 0) {
		return ret;
	}
	data->serdes_pll_locked = (pll_stat & AD9144_PLL_STATUS_LOCKED) != 0;
	LOG_INF("AD9144 SERDES PLL: %s (0x%02x)",
		data->serdes_pll_locked ? "locked" : "NOT locked", pll_stat);

	LOG_INF("SUCCESS: AD9144 SERDES configured");

	return 0;
}

/*
 * Block 7: DAC calibration, AD9144 datasheet Rev. B table 86. Calibrates all
 * DACs together, then polls each in turn. Per-DAC failure is NON-FATAL: it is
 * reported and setup continues (mirrors no-OS, which only logs).
 */
static int ad9144_dac_calibrate(const struct device *dev)
{
	const struct ad9144_config *config = dev->config;
	uint8_t dac_mask = BIT(config->num_converters) - 1;	/* all DACs */
	int ret;

	ret = ad9144_spi_write(dev, AD9144_REG_CAL_CLKDIV, AD9144_CAL_CLKDIV_ON);
	if (ret < 0) {
		return ret;
	}
	ret = ad9144_spi_write(dev, AD9144_REG_CAL_INIT, AD9144_CAL_INIT_VAL);
	if (ret < 0) {
		return ret;
	}
	ret = ad9144_spi_write(dev, AD9144_REG_CAL_INDX, dac_mask);
	if (ret < 0) {
		return ret;
	}
	ret = ad9144_spi_write(dev, AD9144_REG_CAL_CTRL, AD9144_CAL_CTRL_ENABLE);
	if (ret < 0) {
		return ret;
	}
	ret = ad9144_spi_write(dev, AD9144_REG_CAL_CTRL, AD9144_CAL_CTRL_START);
	if (ret < 0) {
		return ret;
	}
	k_msleep(AD9144_CAL_SETTLE_MS);

	for (uint8_t i = 0; i < config->num_converters; i++) {
		uint8_t val = 0;
		int tries = AD9144_CAL_POLL_TRIES;

		ret = ad9144_spi_write(dev, AD9144_REG_CAL_INDX, BIT(i));
		if (ret < 0) {
			return ret;
		}

		do {
			k_msleep(1);
			ret = ad9144_spi_read(dev, AD9144_REG_CAL_CTRL, &val);
			if (ret < 0) {
				return ret;
			}
		} while ((val & AD9144_CAL_ACTIVE) && tries--);

		if ((val & (AD9144_CAL_FIN | AD9144_CAL_ERRHI | AD9144_CAL_ERRLO)) !=
		    AD9144_CAL_FIN) {
			LOG_WRN("DAC-%d calibration failed (0x%02x)", i, val);
		} else {
			LOG_INF("DAC-%d calibration ok", i);
		}
	}

	ret = ad9144_spi_write(dev, AD9144_REG_CAL_CLKDIV, AD9144_CAL_CLKDIV_OFF);
	if (ret < 0) {
		return ret;
	}

	LOG_INF("SUCCESS: AD9144 DAC calibration");

	return 0;
}

/* -------------------------------- setup ----------------------------------- */

/* Faithful port of no-OS ad9144_setup() (DAQ2 path), grouped into blocks. */
static int ad9144_setup(const struct device *dev)
{
	int ret;

	ret = ad9144_reset_and_probe(dev);
	if (ret < 0) {
		return ret;
	}
	ret = ad9144_power_up(dev);
	if (ret < 0) {
		return ret;
	}
	ret = ad9144_setup_datapath(dev);
	if (ret < 0) {
		return ret;
	}
	ret = ad9144_setup_link(dev);
	if (ret < 0) {
		return ret;
	}
	ret = ad9144_setup_datalink_sync(dev);
	if (ret < 0) {
		return ret;
	}
	ret = ad9144_setup_serdes(dev);
	if (ret < 0) {
		return ret;
	}
	ret = ad9144_dac_calibrate(dev);
	if (ret < 0) {
		return ret;
	}

	/* Enable link 0. */
	ret = ad9144_spi_write(dev, AD9144_REG_GENERAL_JRX_CTRL_0,
			       AD9144_LINK_ENABLE);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

/* --------------------------------- init ----------------------------------- */

static int ad9144_init(const struct device *dev)
{
	const struct ad9144_config *config = dev->config;
	int ret;

	if (!spi_is_ready_dt(&config->spi)) {
		LOG_ERR("SPI bus %s not ready", config->spi.bus->name);
		return -ENODEV;
	}

	LOG_INF("AD9144 setup over %s", config->spi.bus->name);

	ret = ad9144_setup(dev);
	if (ret) {
		LOG_ERR("ad9144_setup failed (%d)", ret);
		return ret;
	}

	LOG_INF("SUCCESS: AD9144 setup complete");

	return 0;
}

/* ------------------------------ DT plumbing ------------------------------- */

#define AD9144_DEFINE(n)                                                                    \
	/* Geometry must match the synthesised DAQ2 TX bitstream profile. */               \
	BUILD_ASSERT(DT_INST_PROP(n, adi_converters_per_device) == 2,                      \
		     "AD9144 M must be 2 (DAQ2 TX bitstream)");                            \
	BUILD_ASSERT(DT_INST_PROP(n, adi_lanes_per_device) == 4,                           \
		     "AD9144 L must be 4 (DAQ2 TX bitstream)");                            \
	BUILD_ASSERT(DT_INST_PROP(n, adi_octets_per_frame) == 1,                           \
		     "AD9144 F must be 1 (derived from M=2,L=4,N'=16,S=1)");              \
	BUILD_ASSERT(DT_INST_PROP(n, adi_bits_per_sample) ==                               \
		     DT_INST_PROP(n, adi_converter_resolution) +                          \
		     DT_INST_PROP(n, adi_control_bits_per_sample),                        \
		     "AD9144 N' must equal N + CS");                                       \
	BUILD_ASSERT(DT_INST_PROP(n, adi_converter_resolution) == 16 &&                    \
		     DT_INST_PROP(n, adi_control_bits_per_sample) == 0,                    \
		     "AD9144 is 16-bit: N=16, CS=0 (differs from AD9680 N=14/CS=2)");     \
	/* S and HD are derived, not free; pin them to the profile values. */              \
	BUILD_ASSERT(AD9144_S_DERIVED(DT_INST_PROP(n, adi_octets_per_frame),               \
				      DT_INST_PROP(n, adi_lanes_per_device),               \
				      DT_INST_PROP(n, adi_converters_per_device),          \
				      DT_INST_PROP(n, adi_bits_per_sample)) == 1,          \
		     "AD9144 S must derive to 1 (F*8*L/(M*N'))");                          \
	BUILD_ASSERT(AD9144_HD_DERIVED(DT_INST_PROP(n, adi_octets_per_frame),              \
				       DT_INST_PROP(n, adi_bits_per_sample)) == 1,         \
		     "AD9144 HD must derive to 1 (F*8 < N' => 8 < 16)");                  \
	/* Derived ILAS bytes must match the tier-3 (no-OS) constants. */                  \
	BUILD_ASSERT(AD9144_SCR_L_VAL(DT_INST_PROP(n, adi_lanes_per_device), 1) == 0x83,   \
		     "AD9144 SCR_L byte != no-OS 0x83 (check L)");                        \
	BUILD_ASSERT(AD9144_ILS_F_VAL(DT_INST_PROP(n, adi_octets_per_frame)) == 0x00,      \
		     "AD9144 ILS_F byte != no-OS 0x00 (check F)");                        \
	BUILD_ASSERT(AD9144_ILS_K_VAL(DT_INST_PROP(n, adi_frames_per_multiframe)) == 0x1f, \
		     "AD9144 ILS_K byte != no-OS 0x1f (check K)");                        \
	BUILD_ASSERT(AD9144_ILS_M_VAL(DT_INST_PROP(n, adi_converters_per_device)) == 0x01, \
		     "AD9144 ILS_M byte != no-OS 0x01 (check M)");                        \
	BUILD_ASSERT(AD9144_ILS_CS_N_VAL(DT_INST_PROP(n, adi_control_bits_per_sample),     \
					 DT_INST_PROP(n, adi_converter_resolution)) == 0x0f,\
		     "AD9144 ILS_CS_N byte != no-OS 0x0f (check N, CS)");                 \
	BUILD_ASSERT(AD9144_ILS_NP_VAL(DT_INST_PROP(n, adi_subclass),                      \
				       DT_INST_PROP(n, adi_bits_per_sample)) == 0x2f,      \
		     "AD9144 ILS_NP byte != no-OS 0x2f (check subclass, N')");            \
	BUILD_ASSERT(AD9144_ILS_S_VAL(AD9144_S_DERIVED(                                    \
			     DT_INST_PROP(n, adi_octets_per_frame),                       \
			     DT_INST_PROP(n, adi_lanes_per_device),                       \
			     DT_INST_PROP(n, adi_converters_per_device),                  \
			     DT_INST_PROP(n, adi_bits_per_sample))) == 0x20,              \
		     "AD9144 ILS_S byte != no-OS 0x20 (check S)");                        \
	BUILD_ASSERT(AD9144_ILS_HD_CF_VAL(AD9144_HD_DERIVED(                               \
			     DT_INST_PROP(n, adi_octets_per_frame),                       \
			     DT_INST_PROP(n, adi_bits_per_sample))) == 0x80,              \
		     "AD9144 ILS_HD_CF byte != no-OS 0x80 (check HD)");                   \
	/* Derived lane rate must match the bitstream's synthesised 10 Gbps. */            \
	BUILD_ASSERT(AD9144_LANE_RATE_KBPS(DT_INST_PROP(n, adi_sampling_frequency_khz),    \
					   DT_INST_PROP(n, adi_converters_per_device),     \
					   DT_INST_PROP(n, adi_bits_per_sample),           \
					   DT_INST_PROP(n, adi_lanes_per_device),          \
					   DT_INST_PROP(n, adi_interpolation)) ==          \
		     10000000U, "AD9144 lane rate != 10 Gbps (bitstream LANE_RATE=10)");   \
                                                                                           \
	static const struct ad9144_config ad9144_config_##n = {                            \
		.spi = SPI_DT_SPEC_INST_GET(n, SPI_WORD_SET(8) | SPI_TRANSFER_MSB |         \
						   SPI_OP_MODE_MASTER),                    \
		.lane_rate_kbps = AD9144_LANE_RATE_KBPS(                                    \
			DT_INST_PROP(n, adi_sampling_frequency_khz),                       \
			DT_INST_PROP(n, adi_converters_per_device),                       \
			DT_INST_PROP(n, adi_bits_per_sample),                             \
			DT_INST_PROP(n, adi_lanes_per_device),                            \
			DT_INST_PROP(n, adi_interpolation)),                              \
		.num_converters = DT_INST_PROP(n, adi_converters_per_device),             \
		.num_lanes = DT_INST_PROP(n, adi_lanes_per_device),                       \
		.octets_per_frame = DT_INST_PROP(n, adi_octets_per_frame),                \
		.frames_per_multiframe = DT_INST_PROP(n, adi_frames_per_multiframe),      \
		.converter_resolution = DT_INST_PROP(n, adi_converter_resolution),        \
		.bits_per_sample = DT_INST_PROP(n, adi_bits_per_sample),                   \
		.subclass = DT_INST_PROP(n, adi_subclass),                                \
		.interpolation = DT_INST_PROP(n, adi_interpolation),                      \
	};                                                                                 \
                                                                                           \
	static struct ad9144_data ad9144_data_##n;                                         \
                                                                                           \
	DEVICE_DT_INST_DEFINE(n, ad9144_init, NULL, &ad9144_data_##n,                       \
			      &ad9144_config_##n, POST_KERNEL,                             \
			      CONFIG_AD9144_INIT_PRIORITY, NULL);

BUILD_ASSERT(CONFIG_AD9144_INIT_PRIORITY > CONFIG_SPI_INIT_PRIORITY,
	     "The AD9144 is SPI-attached, so it must initialise after its SPI controller");

DT_INST_FOREACH_STATUS_OKAY(AD9144_DEFINE)
