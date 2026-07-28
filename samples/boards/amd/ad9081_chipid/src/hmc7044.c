/*
 * HMC7044 clock/SYSREF chip -- setup + SPI bring-up.
 *
 * This is a faithful port of the no-OS hmc7044 driver
 * (drivers/frequency/hmc7044/hmc7044.c) reduced to the single-chip HMC7044
 * (non-HMC7043) PLL1/PLL2/VCO/output-divider/SYSREF programming path, using the
 * zcu102 PLATFORM_ZYNQMP profile from projects/ad9081/src/app_clock.c as the
 * ground-truth configuration.
 *
 * The no-OS driver itself is not copied verbatim (it is saturated with the
 * no_os_* clock framework, calloc and the JESD204 FSM); instead the register
 * recipe is reimplemented thin against Zephyr SPI, keeping every register write,
 * ordering and the PLL divider math identical to no-OS. Later this grows into
 * drivers/clock_control/hmc7044.c with a custom SYSREF API.
 *
 * The HMC7044 has NO chip-ID register, so -- exactly like no-OS
 * hmc7044_read_write_check() -- we prove the bus by writing a known byte to the
 * scratchpad register (0x0008) and reading it back.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/init.h>
#include <zephyr/kernel/mm.h>
#include <zephyr/sys/device_mmio.h>
#include <zephyr/sys/util.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(hmc7044, LOG_LEVEL_INF);

#include "hmc7044.h"

/*
 * Same MMU story as SPI0 in ad9081.c: map the SPI1 register page 1:1 as
 * non-cached device memory at PRE_KERNEL_1, before the Cadence SPI driver
 * initialises at POST_KERNEL.
 */
#define HMC7044_SPI1_BASE 0xff050000UL
#define HMC7044_SPI1_SIZE 0x1000UL

static int hmc7044_map_spi1(void)
{
	mm_reg_t virt;

	device_map(&virt, HMC7044_SPI1_BASE, HMC7044_SPI1_SIZE, K_MEM_CACHE_NONE);

	if (virt != HMC7044_SPI1_BASE) {
		LOG_ERR("SPI1 not identity-mapped: virt=0x%lx phys=0x%lx",
			(unsigned long)virt, HMC7044_SPI1_BASE);
		return -EIO;
	}
	return 0;
}

SYS_INIT(hmc7044_map_spi1, PRE_KERNEL_1, 0);

/* HMC7044 command word (from no-OS hmc7044.c):
 *   bit 15    : 1 = read, 0 = write
 *   bits 14:13: (count - 1)      -- 1 byte here -> 0
 *   bits 11:0 : register address
 */
#define HMC7044_READ    (1U << 15)
#define HMC7044_WRITE   (0U << 15)
#define HMC7044_CNT(x)  (((x) - 1U) << 13)
#define HMC7044_ADDR(x) ((x) & 0xFFFU)

/* -------------------- HMC7044 registers / bitfields (no-OS) ---------------- */

/* Global Control */
#define HMC7044_REG_SOFT_RESET		0x0000
#define HMC7044_SOFT_RESET		BIT(0)

#define HMC7044_REG_REQ_MODE_0		0x0001
#define HMC7044_RESEED_REQ		BIT(7)
#define HMC7044_HIGH_PERF_DISTRIB_PATH	BIT(6)
#define HMC7044_HIGH_PERF_PLL_VCO	BIT(5)
#define HMC7044_FORCE_HOLDOVER		BIT(4)
#define HMC7044_MUTE_OUT_DIV		BIT(3)
#define HMC7044_PULSE_GEN_REQ		BIT(2)
#define HMC7044_RESTART_DIV_FSM		BIT(1)
#define HMC7044_SLEEP_MODE		BIT(0)

#define HMC7044_REG_EN_CTRL_0		0x0003
#define HMC7044_RF_RESEEDER_EN		BIT(5)
#define HMC7044_VCO_SEL(x)		(((x) & 0x3) << 3)
#define HMC7044_VCO_EXT			0
#define HMC7044_VCO_HIGH		1
#define HMC7044_VCO_LOW			2
#define HMC7044_SYSREF_TIMER_EN		BIT(2)
#define HMC7044_PLL2_EN			BIT(1)
#define HMC7044_PLL1_EN			BIT(0)

#define HMC7044_REG_GLOB_MODE		0x0005
#define HMC7044_REF_PATH_EN(x)		((x) & 0xf)
#define HMC7044_RFSYNC_EN		BIT(4)
#define HMC7044_VCOIN_MODE_EN		BIT(5)
#define HMC7044_SYNC_PIN_MODE(x)	(((x) & 0x3) << 6)

#define HMC7044_REG_SCRATCHPAD		0x0008
#define HMC7044_SCRATCH_PATTERN		0xAD

/* PLL1 */
#define HMC7044_REG_CLKIN0_BUF_CTRL	0x000A
#define HMC7044_REG_CLKIN1_BUF_CTRL	0x000B
#define HMC7044_REG_CLKIN2_BUF_CTRL	0x000C
#define HMC7044_REG_CLKIN3_BUF_CTRL	0x000D
#define HMC7044_REG_OSCIN_BUF_CTRL	0x000E

#define HMC7044_REG_PLL1_REF_PRIO_CTRL	0x0014

#define HMC7044_REG_PLL1_CP_CTRL	0x001A
#define HMC7044_PLL1_CP_CURRENT(x)	((x) & 0xf)

#define HMC7044_REG_CLKIN_PRESCALER(x)	(0x001C + (x))
#define HMC7044_REG_OSCIN_PRESCALER	0x0020

#define HMC7044_REG_PLL1_R_LSB		0x0021
#define HMC7044_REG_PLL1_R_MSB		0x0022
#define HMC7044_REG_PLL1_N_LSB		0x0026
#define HMC7044_REG_PLL1_N_MSB		0x0027

#define HMC7044_REG_PLL1_LOCK_DETECT	0x0028
#define HMC7044_LOCK_DETECT_TIMER(x)	((x) & 0x1f)

#define HMC7044_REG_PLL1_REF_SWITCH	0x0029
#define HMC7044_HOLDOVER_DAC		BIT(2)
#define HMC7044_AUTO_REVERT_SWITCH	BIT(1)
#define HMC7044_AUTO_MODE_SWITCH	BIT(0)

/* PLL2 */
#define HMC7044_REG_PLL2_FREQ_DOUBLER	0x0032
#define HMC7044_PLL2_FREQ_DOUBLER_DIS	BIT(0)

#define HMC7044_REG_PLL2_R_LSB		0x0033
#define HMC7044_R2_LSB(x)		((x) & 0xff)
#define HMC7044_REG_PLL2_R_MSB		0x0034
#define HMC7044_R2_MSB(x)		(((x) & 0xf00) >> 8)
#define HMC7044_REG_PLL2_N_LSB		0x0035
#define HMC7044_N2_LSB(x)		((x) & 0xff)
#define HMC7044_REG_PLL2_N_MSB		0x0036
#define HMC7044_N2_MSB(x)		(((x) & 0xff00) >> 8)

/* GPIO/SDATA Control */
#define HMC7044_REG_GPI_CTRL(x)		(0x0046 + (x))
#define HMC7044_REG_GPO_CTRL(x)		(0x0050 + (x))

/* SYSREF/SYNC Control */
#define HMC7044_REG_PULSE_GEN		0x005A
#define HMC7044_PULSE_GEN_MODE(x)	((x) & 0x7)

#define HMC7044_REG_SYNC		0x005B
#define HMC7044_SYNC_RETIME		BIT(2)

#define HMC7044_REG_SYSREF_TIMER_LSB	0x005C
#define HMC7044_SYSREF_TIMER_LSB(x)	((x) & 0xff)
#define HMC7044_REG_SYSREF_TIMER_MSB	0x005D
#define HMC7044_SYSREF_TIMER_MSB(x)	(((x) & 0xf00) >> 8)

#define HMC7044_CLK_INPUT_CTRL		0x0064
#define HMC7044_LOW_FREQ_INPUT_MODE	BIT(0)

/* Status and Alarm readback */
#define HMC7044_REG_ALARM_READBACK	0x007D
#define HMC7044_REG_PLL1_STATUS		0x0082

#define HMC7044_PLL1_FSM_STATE(x)	((x) & 0x7)
#define HMC7044_PLL1_ACTIVE_CLKIN(x)	(((x) >> 3) & 0x3)
#define HMC7044_PLL2_LOCK_DETECT(x)	((x) & 0x1)

/* Other Controls */
#define HMC7044_REG_CLK_OUT_DRV_LOW_PW	0x009F
#define HMC7044_REG_CLK_OUT_DRV_HIGH_PW	0x00A0
#define HMC7044_REG_PLL1_DELAY		0x00A5
#define HMC7044_REG_PLL1_HOLDOVER	0x00A8
#define HMC7044_REG_VTUNE_PRESET	0x00B0

/* Clock Distribution */
#define HMC7044_REG_CH_OUT_CRTL_0(ch)	(0x00C8 + 0xA * (ch))
#define HMC7044_HI_PERF_MODE		BIT(7)
#define HMC7044_SYNC_EN			BIT(6)
#define HMC7044_CH_EN			BIT(0)
#define HMC7044_START_UP_MODE_DYN_EN	(BIT(3) | BIT(2))

#define HMC7044_REG_CH_OUT_CRTL_1(ch)	(0x00C9 + 0xA * (ch))
#define HMC7044_DIV_LSB(x)		((x) & 0xFF)
#define HMC7044_REG_CH_OUT_CRTL_2(ch)	(0x00CA + 0xA * (ch))
#define HMC7044_DIV_MSB(x)		(((x) >> 8) & 0xFF)
#define HMC7044_REG_CH_OUT_CRTL_3(ch)	(0x00CB + 0xA * (ch))
#define HMC7044_REG_CH_OUT_CRTL_4(ch)	(0x00CC + 0xA * (ch))
#define HMC7044_REG_CH_OUT_CRTL_7(ch)	(0x00CF + 0xA * (ch))
#define HMC7044_REG_CH_OUT_CRTL_8(ch)	(0x00D0 + 0xA * (ch))
#define HMC7044_DRIVER_MODE(x)		(((x) & 0x3) << 3)
#define HMC7044_DRIVER_Z_MODE(x)	(((x) & 0x3) << 0)
#define HMC7044_DYN_DRIVER_EN		BIT(5)
#define HMC7044_FORCE_MUTE_EN		BIT(7)

#define HMC7044_NUM_CHAN	14

#define HMC7044_LOW_VCO_MIN	2150000
#define HMC7044_LOW_VCO_MAX	2880000
#define HMC7044_HIGH_VCO_MIN	2650000
#define HMC7044_HIGH_VCO_MAX	3200000

#define HMC7044_RECOMM_LCM_MIN	30000
#define HMC7044_RECOMM_LCM_MAX	70000
#define HMC7044_RECOMM_PFD1	10000
#define HMC7044_MIN_PFD1	1
#define HMC7044_MAX_PFD1	50000

#define HMC7044_CP_CURRENT_STEP	120
#define HMC7044_CP_CURRENT_MIN	120
#define HMC7044_CP_CURRENT_MAX	1920
#define HMC7044_CP_CURRENT_DEF	1080

#define HMC7044_R1_MAX		65535
#define HMC7044_N1_MAX		65535

#define HMC7044_R2_MIN		1
#define HMC7044_R2_MAX		4095
#define HMC7044_N2_MIN		8
#define HMC7044_N2_MAX		65535

#define HMC7044_OUT_DIV_MIN	1
#define HMC7044_OUT_DIV_MAX	4094

static const char *const pll1_fsm_states[] = {
	"Reset",
	"Acquisition",
	"Locked",
	"Invalid",
	"Holdover",
	"DAC assisted holdover exit",
	"Invalid",
};

/* -------------------------- Output channel spec --------------------------- */

struct hmc7044_chan_spec {
	unsigned int num;
	bool disable;
	bool high_performance_mode_dis;
	bool start_up_mode_dynamic_enable;
	bool dynamic_driver_enable;
	bool force_mute_enable;
	bool is_sysref;
	unsigned int divider;
	unsigned int driver_mode;
	unsigned int driver_impedance;
	unsigned int coarse_delay;
	unsigned int fine_delay;
	unsigned int out_mux_mode;
};

/*
 * zcu102 (PLATFORM_ZYNQMP) channel map, from no-OS app_clock.c. pll2_freq is
 * 3 GHz, so divider N gives 3000/N MHz.
 */
static struct hmc7044_chan_spec hmc7044_channels[] = {
	{ .num = 0,  .divider = 12,   .driver_mode = 2 },                  /* CORE_CLK_RX   250 MHz */
	{ .num = 2,  .divider = 12,   .driver_mode = 2 },                  /* DEV_REFCLK    250 MHz */
	{ .num = 3,  .divider = 1536, .driver_mode = 2, .is_sysref = true }, /* DEV_SYSREF  1.953 MHz */
	{ .num = 6,  .divider = 12,   .driver_mode = 2 },                  /* CORE_CLK_TX   250 MHz */
	{ .num = 8,  .divider = 6,    .driver_mode = 2 },                  /* CORE_CLK_RX   500 MHz */
	{ .num = 10, .divider = 12,   .driver_mode = 2 },                  /* CORE_CLK_RX_ALT 250 MHz */
	{ .num = 12, .divider = 6,    .driver_mode = 2 },                  /* FPGA_REFCLK   500 MHz */
	{ .num = 13, .divider = 1536, .driver_mode = 2, .is_sysref = true }, /* FPGA_SYSREF 1.953 MHz */
};

/*
 * Device context. Mirrors the subset of no-OS struct hmc7044_dev that the
 * single-chip HMC7044 setup path uses. Populated from the zcu102 init profile
 * in hmc7044_apply_init_defaults().
 */
struct hmc7044 {
	uint32_t clkin_freq[4];
	uint32_t vcxo_freq;
	uint32_t pll1_pfd;
	uint32_t pfd1_limit;
	uint32_t pll1_cp_current;
	uint32_t pll2_freq;
	uint32_t pll1_loop_bw;
	uint32_t sysref_timer_div;
	unsigned int pll1_ref_prio_ctrl;
	bool pll1_ref_autorevert_en;
	bool clkin0_rfsync_en;
	bool clkin1_vcoin_en;
	bool high_performance_mode_clock_dist_en;
	bool rf_reseeder_en;
	unsigned int sync_pin_mode;
	uint32_t pulse_gen_mode;
	uint32_t in_buf_mode[5];
	uint32_t gpi_ctrl[4];
	uint32_t gpo_ctrl[4];
	uint32_t num_channels;
	struct hmc7044_chan_spec *channels;
	bool read_write_confirmed;
};

static struct hmc7044 hmc7044_dev;

/* --------------------------- inlined no-OS math --------------------------- */

/* no_os_greatest_common_divisor() */
static uint32_t hmc7044_gcd(uint32_t a, uint32_t b)
{
	uint32_t div;

	if ((a == 0) || (b == 0)) {
		return MAX(a, b);
	}

	while (b != 0) {
		div = a % b;
		a = b;
		b = div;
	}

	return a;
}

/* no_os_rational_best_approximation() */
static void hmc7044_rational_best_approximation(uint32_t num, uint32_t den,
						uint32_t max_num, uint32_t max_den,
						uint32_t *best_num, uint32_t *best_den)
{
	uint32_t gcd = hmc7044_gcd(num, den);

	*best_num = num / gcd;
	*best_den = den / gcd;

	if ((*best_num > max_num) || (*best_den > max_den)) {
		*best_num = 0;
		*best_den = 0;
	}
}

/* no_os_find_last_set_bit() == no_os_log_base_2() */
static uint32_t hmc7044_log_base_2(uint32_t word)
{
	uint32_t bit = 0;
	uint32_t last_set_bit = 32;

	while (word) {
		if (word & 0x1) {
			last_set_bit = bit;
		}
		word >>= 1;
		bit++;
	}

	return last_set_bit;
}

/* ------------------------------ SPI helpers ------------------------------- */

static const struct spi_dt_spec hmc7044_spi = SPI_DT_SPEC_GET(
	DT_NODELABEL(hmc7044),
	SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_OP_MODE_MASTER,
	0);

/* Write one 8-bit register: [ cmd_hi, cmd_lo, val ]. */
static int hmc7044_write(uint16_t reg, uint8_t val)
{
	uint16_t cmd = HMC7044_WRITE | HMC7044_CNT(1) | HMC7044_ADDR(reg);
	uint8_t tx[3] = {
		cmd >> 8,
		cmd & 0xFF,
		val,
	};
	const struct spi_buf txb = { .buf = tx, .len = sizeof(tx) };
	const struct spi_buf_set txs = { .buffers = &txb, .count = 1 };

	return spi_write_dt(&hmc7044_spi, &txs);
}

/* Read one 8-bit register. Data returns in the third byte. */
static int hmc7044_read(uint16_t reg, uint8_t *val)
{
	uint16_t cmd = HMC7044_READ | HMC7044_CNT(1) | HMC7044_ADDR(reg);
	uint8_t tx[3] = {
		cmd >> 8,
		cmd & 0xFF,
		0x00,
	};
	uint8_t rx[3] = { 0 };
	const struct spi_buf txb = { .buf = tx, .len = sizeof(tx) };
	const struct spi_buf rxb = { .buf = rx, .len = sizeof(rx) };
	const struct spi_buf_set txs = { .buffers = &txb, .count = 1 };
	const struct spi_buf_set rxs = { .buffers = &rxb, .count = 1 };
	int ret;

	ret = spi_transceive_dt(&hmc7044_spi, &txs, &rxs);
	if (ret == 0) {
		*val = rx[2];
	}
	return ret;
}

/* no-OS hmc7044_read_write_check() */
static void hmc7044_read_write_check(struct hmc7044 *dev)
{
	uint8_t val = 0;

	hmc7044_write(HMC7044_REG_SCRATCHPAD, HMC7044_SCRATCH_PATTERN);
	hmc7044_read(HMC7044_REG_SCRATCHPAD, &val);

	dev->read_write_confirmed = (val == HMC7044_SCRATCH_PATTERN);

	if (!dev->read_write_confirmed) {
		LOG_WRN("Read/Write check failed (0x%X)", val);
	}
}

/* no-OS hmc7044_toggle_bit() */
static int hmc7044_toggle_bit(struct hmc7044 *dev, uint16_t reg,
			      uint8_t mask, uint32_t us_delay)
{
	uint8_t val;
	int ret;

	if (dev->read_write_confirmed) {
		ret = hmc7044_read(reg, &val);
		if (ret < 0) {
			return ret;
		}
	} else {
		val = 0;
	}

	ret = hmc7044_write(reg, val | mask);
	if (ret < 0) {
		return ret;
	}

	val &= ~mask;

	ret = hmc7044_write(reg, val);
	if (ret < 0) {
		return ret;
	}

	if (us_delay) {
		k_busy_wait(us_delay);
	}

	return 0;
}

/* -------------------------------- info ------------------------------------ */

/* no-OS hmc7044_info() -- HMC7044, non-vcoin path. */
static int hmc7044_info(struct hmc7044 *dev)
{
	uint32_t clkin_freq, active;
	uint8_t alarm_stat = 0, pll1_stat = 0;
	int ret;

	if (!dev->read_write_confirmed) {
		LOG_INF("Probed, SPI read support failed");
		return 0;
	}

	ret = hmc7044_read(HMC7044_REG_PLL1_STATUS, &pll1_stat);
	if (ret < 0) {
		return ret;
	}

	if (HMC7044_PLL1_FSM_STATE(pll1_stat) != 2) { /* Lock */
		k_msleep(DIV_ROUND_UP(5000, dev->pll1_loop_bw));
		ret = hmc7044_read(HMC7044_REG_PLL1_STATUS, &pll1_stat);
		if (ret < 0) {
			return ret;
		}
	}

	ret = hmc7044_read(HMC7044_REG_ALARM_READBACK, &alarm_stat);
	if (ret < 0) {
		return ret;
	}

	active = HMC7044_PLL1_ACTIVE_CLKIN(pll1_stat);
	clkin_freq = dev->clkin_freq[active];

	LOG_INF("PLL1: %s, CLKIN%u @ %u Hz, PFD: %u kHz - PLL2: %s @ %u.%06u MHz",
		pll1_fsm_states[HMC7044_PLL1_FSM_STATE(pll1_stat)],
		active, clkin_freq, dev->pll1_pfd,
		HMC7044_PLL2_LOCK_DETECT(alarm_stat) ? "Locked" : "Unlocked",
		dev->pll2_freq / 1000000, dev->pll2_freq % 1000000);

	return 0;
}

/* ------------------------------- setup ------------------------------------ */

/* Faithful port of no-OS hmc7044_setup() (single-chip HMC7044 path). */
static int hmc7044_setup(struct hmc7044 *dev)
{
	struct hmc7044_chan_spec *chan;
	bool high_vco_en;
	bool pll2_freq_doubler_en;
	uint32_t vcxo_freq, pll2_freq;
	uint32_t clkin_freq[4];
	uint32_t lcm_freq;
	uint32_t in_prescaler[5];
	uint32_t pll1_lock_detect;
	uint32_t n1, r1;
	uint32_t n, r;
	uint32_t pfd1_freq;
	uint32_t vco_limit;
	uint32_t n2[2], r2[2];
	uint32_t i, ref_en = 0;
	int ret;

	vcxo_freq = dev->vcxo_freq / 1000;
	pll2_freq = dev->pll2_freq / 1000;

	lcm_freq = vcxo_freq;
	for (i = 0; i < ARRAY_SIZE(clkin_freq); i++) {
		clkin_freq[i] = dev->clkin_freq[i] / 1000;

		if (clkin_freq[i]) {
			lcm_freq = hmc7044_gcd(clkin_freq[i], lcm_freq);
			ref_en |= BIT(i);
		}
	}

	while (lcm_freq > HMC7044_RECOMM_LCM_MAX) {
		lcm_freq /= 2;
	}

	for (i = 0; i < ARRAY_SIZE(clkin_freq); i++) {
		if (clkin_freq[i]) {
			in_prescaler[i] = clkin_freq[i] / lcm_freq;
		} else {
			in_prescaler[i] = 1;
		}
	}
	in_prescaler[4] = vcxo_freq / lcm_freq;

	pll1_lock_detect = hmc7044_log_base_2((lcm_freq * 4000) / dev->pll1_loop_bw);

	/* fVCXO / N1 = fLCM / R1 */
	hmc7044_rational_best_approximation(vcxo_freq, lcm_freq,
					    HMC7044_N1_MAX, HMC7044_R1_MAX,
					    &n1, &r1);

	pfd1_freq = vcxo_freq / n1;

	n = n1;
	r = r1;
	while (pfd1_freq > dev->pfd1_limit) {
		do {
			n++;
		} while (((vcxo_freq % n) || (lcm_freq * n % vcxo_freq)) &&
			 (n <= HMC7044_N1_MAX));
		r = lcm_freq * n / vcxo_freq;

		if ((n > HMC7044_N1_MAX) || (r > HMC7044_R1_MAX)) {
			break;
		}

		n1 = n;
		r1 = r;
		pfd1_freq = vcxo_freq / n1;
	}

	dev->pll1_pfd = pfd1_freq;

	if (pll2_freq < HMC7044_LOW_VCO_MIN || pll2_freq > HMC7044_HIGH_VCO_MAX) {
		return -EINVAL;
	}

	vco_limit = (HMC7044_LOW_VCO_MAX + HMC7044_HIGH_VCO_MIN) / 2;
	high_vco_en = (pll2_freq >= vco_limit);

	/* fVCO / N2 = fVCXO * doubler / R2 */
	pll2_freq_doubler_en = true;
	hmc7044_rational_best_approximation(pll2_freq, vcxo_freq * 2,
					    HMC7044_N2_MAX, HMC7044_R2_MAX,
					    &n2[0], &r2[0]);

	if (pll2_freq != vcxo_freq * n2[0] / r2[0]) {
		hmc7044_rational_best_approximation(pll2_freq, vcxo_freq,
						    HMC7044_N2_MAX, HMC7044_R2_MAX,
						    &n2[1], &r2[1]);

		if (abs((int)pll2_freq - (int)(vcxo_freq * 2 * n2[0] / r2[0])) >
		    abs((int)pll2_freq - (int)(vcxo_freq * n2[1] / r2[1]))) {
			n2[0] = n2[1];
			r2[0] = r2[1];
			pll2_freq_doubler_en = false;
		}
	}

	while ((n2[0] < HMC7044_N2_MIN) && (r2[0] <= HMC7044_R2_MAX / 2)) {
		n2[0] *= 2;
		r2[0] *= 2;
	}
	if (n2[0] < HMC7044_N2_MIN) {
		return -EINVAL;
	}

	LOG_INF("PLL1 N1=%u R1=%u PFD1=%u kHz; PLL2 N2=%u R2=%u doubler=%d VCO=%s",
		n1, r1, pfd1_freq, n2[0], r2[0], pll2_freq_doubler_en,
		high_vco_en ? "high" : "low");

	/* Resets all registers to default values */
	ret = hmc7044_toggle_bit(dev, HMC7044_REG_SOFT_RESET,
				 HMC7044_SOFT_RESET, 100);
	if (ret) {
		return ret;
	}

	hmc7044_read_write_check(dev);

	/* Disable all channels */
	for (i = 0; i < HMC7044_NUM_CHAN; i++) {
		ret = hmc7044_write(HMC7044_REG_CH_OUT_CRTL_0(i), 0);
		if (ret) {
			return ret;
		}
	}

	/* Load the configuration updates (provided by Analog Devices) */
	ret = hmc7044_write(HMC7044_REG_CLK_OUT_DRV_LOW_PW, 0x4d);
	if (ret) {
		return ret;
	}
	ret = hmc7044_write(HMC7044_REG_CLK_OUT_DRV_HIGH_PW, 0xdf);
	if (ret) {
		return ret;
	}
	ret = hmc7044_write(HMC7044_REG_PLL1_DELAY, 0x06);
	if (ret) {
		return ret;
	}
	ret = hmc7044_write(HMC7044_REG_PLL1_HOLDOVER, 0x06);
	if (ret) {
		return ret;
	}
	ret = hmc7044_write(HMC7044_REG_VTUNE_PRESET, 0x04);
	if (ret) {
		return ret;
	}

	ret = hmc7044_write(HMC7044_REG_GLOB_MODE,
			    HMC7044_SYNC_PIN_MODE(dev->sync_pin_mode) |
			    (dev->clkin0_rfsync_en ? HMC7044_RFSYNC_EN : 0) |
			    (dev->clkin1_vcoin_en ? HMC7044_VCOIN_MODE_EN : 0) |
			    HMC7044_REF_PATH_EN(ref_en));
	if (ret) {
		return ret;
	}

	/* Program PLL2 -- select the VCO range */
	ret = hmc7044_write(HMC7044_REG_EN_CTRL_0,
			    (dev->rf_reseeder_en ? HMC7044_RF_RESEEDER_EN : 0) |
			    HMC7044_VCO_SEL(high_vco_en ? HMC7044_VCO_HIGH :
					    HMC7044_VCO_LOW) |
			    HMC7044_SYSREF_TIMER_EN | HMC7044_PLL2_EN |
			    HMC7044_PLL1_EN);
	if (ret) {
		return ret;
	}

	/* Program the PLL2 dividers */
	ret = hmc7044_write(HMC7044_REG_PLL2_R_LSB, HMC7044_R2_LSB(r2[0]));
	if (ret) {
		return ret;
	}
	ret = hmc7044_write(HMC7044_REG_PLL2_R_MSB, HMC7044_R2_MSB(r2[0]));
	if (ret) {
		return ret;
	}
	ret = hmc7044_write(HMC7044_REG_PLL2_N_LSB, HMC7044_N2_LSB(n2[0]));
	if (ret) {
		return ret;
	}
	ret = hmc7044_write(HMC7044_REG_PLL2_N_MSB, HMC7044_N2_MSB(n2[0]));
	if (ret) {
		return ret;
	}

	/* Program the reference doubler */
	ret = hmc7044_write(HMC7044_REG_PLL2_FREQ_DOUBLER,
			    pll2_freq_doubler_en ? 0 : HMC7044_PLL2_FREQ_DOUBLER_DIS);
	if (ret) {
		return ret;
	}

	/* Program PLL1 */
	ret = hmc7044_write(HMC7044_REG_PLL1_CP_CTRL,
			    HMC7044_PLL1_CP_CURRENT(dev->pll1_cp_current /
						    HMC7044_CP_CURRENT_STEP - 1));
	if (ret) {
		return ret;
	}
	/* Set the lock detect timer threshold */
	ret = hmc7044_write(HMC7044_REG_PLL1_LOCK_DETECT,
			    HMC7044_LOCK_DETECT_TIMER(pll1_lock_detect));
	if (ret) {
		return ret;
	}

	/* Set the LCM */
	for (i = 0; i < ARRAY_SIZE(clkin_freq); i++) {
		ret = hmc7044_write(HMC7044_REG_CLKIN_PRESCALER(i),
				    in_prescaler[i]);
		if (ret) {
			return ret;
		}
	}
	ret = hmc7044_write(HMC7044_REG_OSCIN_PRESCALER, in_prescaler[4]);
	if (ret) {
		return ret;
	}

	/*
	 * Program the PLL1 dividers. Note: no-OS deliberately reuses the PLL2
	 * R2_/N2_ field macros here (same LSB/MSB split), so keep that.
	 */
	ret = hmc7044_write(HMC7044_REG_PLL1_R_LSB, HMC7044_R2_LSB(r1));
	if (ret) {
		return ret;
	}
	ret = hmc7044_write(HMC7044_REG_PLL1_R_MSB, HMC7044_R2_MSB(r1));
	if (ret) {
		return ret;
	}
	ret = hmc7044_write(HMC7044_REG_PLL1_N_LSB, HMC7044_N2_LSB(n1));
	if (ret) {
		return ret;
	}
	ret = hmc7044_write(HMC7044_REG_PLL1_N_MSB, HMC7044_N2_MSB(n1));
	if (ret) {
		return ret;
	}
	ret = hmc7044_write(HMC7044_REG_PLL1_REF_PRIO_CTRL,
			    dev->pll1_ref_prio_ctrl);
	if (ret) {
		return ret;
	}
	ret = hmc7044_write(HMC7044_REG_PLL1_REF_SWITCH,
			    HMC7044_HOLDOVER_DAC |
			    (dev->pll1_ref_autorevert_en ?
			     HMC7044_AUTO_REVERT_SWITCH : 0) |
			    HMC7044_AUTO_MODE_SWITCH);
	if (ret) {
		return ret;
	}

	/* Program the SYSREF timer divide ratio */
	ret = hmc7044_write(HMC7044_REG_SYSREF_TIMER_LSB,
			    HMC7044_SYSREF_TIMER_LSB(dev->sysref_timer_div));
	if (ret) {
		return ret;
	}
	ret = hmc7044_write(HMC7044_REG_SYSREF_TIMER_MSB,
			    HMC7044_SYSREF_TIMER_MSB(dev->sysref_timer_div));
	if (ret) {
		return ret;
	}

	/* Set the pulse generator mode configuration */
	ret = hmc7044_write(HMC7044_REG_PULSE_GEN,
			    HMC7044_PULSE_GEN_MODE(dev->pulse_gen_mode));
	if (ret) {
		return ret;
	}

	/* Enable the input buffers */
	ret = hmc7044_write(HMC7044_REG_CLKIN0_BUF_CTRL, dev->in_buf_mode[0]);
	if (ret) {
		return ret;
	}
	ret = hmc7044_write(HMC7044_REG_CLKIN1_BUF_CTRL, dev->in_buf_mode[1]);
	if (ret) {
		return ret;
	}
	ret = hmc7044_write(HMC7044_REG_CLKIN2_BUF_CTRL, dev->in_buf_mode[2]);
	if (ret) {
		return ret;
	}
	ret = hmc7044_write(HMC7044_REG_CLKIN3_BUF_CTRL, dev->in_buf_mode[3]);
	if (ret) {
		return ret;
	}
	ret = hmc7044_write(HMC7044_REG_OSCIN_BUF_CTRL, dev->in_buf_mode[4]);
	if (ret) {
		return ret;
	}

	/* Set GPIOs */
	for (i = 0; i < ARRAY_SIZE(dev->gpi_ctrl); i++) {
		ret = hmc7044_write(HMC7044_REG_GPI_CTRL(i), dev->gpi_ctrl[i]);
		if (ret) {
			return ret;
		}
	}
	for (i = 0; i < ARRAY_SIZE(dev->gpo_ctrl); i++) {
		ret = hmc7044_write(HMC7044_REG_GPO_CTRL(i), dev->gpo_ctrl[i]);
		if (ret) {
			return ret;
		}
	}

	k_msleep(10);

	/* Program the output channels */
	for (i = 0; i < dev->num_channels; i++) {
		chan = &dev->channels[i];

		if (chan->num >= HMC7044_NUM_CHAN || chan->disable) {
			continue;
		}

		ret = hmc7044_write(HMC7044_REG_CH_OUT_CRTL_1(chan->num),
				    HMC7044_DIV_LSB(chan->divider));
		if (ret) {
			return ret;
		}
		ret = hmc7044_write(HMC7044_REG_CH_OUT_CRTL_2(chan->num),
				    HMC7044_DIV_MSB(chan->divider));
		if (ret) {
			return ret;
		}
		ret = hmc7044_write(HMC7044_REG_CH_OUT_CRTL_8(chan->num),
				    HMC7044_DRIVER_MODE(chan->driver_mode) |
				    HMC7044_DRIVER_Z_MODE(chan->driver_impedance) |
				    (chan->dynamic_driver_enable ?
				     HMC7044_DYN_DRIVER_EN : 0) |
				    (chan->force_mute_enable ?
				     HMC7044_FORCE_MUTE_EN : 0));
		if (ret) {
			return ret;
		}
		ret = hmc7044_write(HMC7044_REG_CH_OUT_CRTL_3(chan->num),
				    chan->fine_delay & 0x1F);
		if (ret) {
			return ret;
		}
		ret = hmc7044_write(HMC7044_REG_CH_OUT_CRTL_4(chan->num),
				    chan->coarse_delay & 0x1F);
		if (ret) {
			return ret;
		}
		ret = hmc7044_write(HMC7044_REG_CH_OUT_CRTL_7(chan->num),
				    chan->out_mux_mode & 0x3);
		if (ret) {
			return ret;
		}
		ret = hmc7044_write(HMC7044_REG_CH_OUT_CRTL_0(chan->num),
				    (chan->start_up_mode_dynamic_enable ?
				     HMC7044_START_UP_MODE_DYN_EN : 0) | BIT(4) |
				    (chan->high_performance_mode_dis ?
				     0 : HMC7044_HI_PERF_MODE) | HMC7044_SYNC_EN |
				    HMC7044_CH_EN);
		if (ret) {
			return ret;
		}
	}
	k_msleep(10);

	/* Do a restart to reset the system and initiate calibration */
	ret = hmc7044_toggle_bit(dev, HMC7044_REG_REQ_MODE_0,
				 HMC7044_RESTART_DIV_FSM, 10000);
	if (ret) {
		return ret;
	}

	ret = hmc7044_toggle_bit(dev, HMC7044_REG_REQ_MODE_0,
				 HMC7044_RESEED_REQ, 1000);
	if (ret) {
		return ret;
	}

	ret = hmc7044_write(HMC7044_REG_REQ_MODE_0,
			    (dev->high_performance_mode_clock_dist_en ?
			     HMC7044_HIGH_PERF_DISTRIB_PATH : 0));
	if (ret) {
		return ret;
	}

	return hmc7044_info(dev);
}

/*
 * Populate the device context from the zcu102 (PLATFORM_ZYNQMP, non-MCS)
 * profile, applying the same defaults no-OS hmc7044_init() does.
 * Board is the AD9082-FMC-EBZ-A2 -> 100 MHz VCXO/XO variant.
 */
static void hmc7044_apply_init_defaults(struct hmc7044 *dev)
{
	dev->clkin_freq[0] = 100000000;
	dev->clkin_freq[1] = 10000000;
	dev->clkin_freq[2] = 0;
	dev->clkin_freq[3] = 0;
	dev->vcxo_freq = 100000000;

	/* pfd1_limit == 0 -> recommended PFD1 (in kHz internally). */
	dev->pfd1_limit = HMC7044_RECOMM_PFD1;

	/* pll1_cp_current == 0 -> default. */
	dev->pll1_cp_current = HMC7044_CP_CURRENT_DEF;

	dev->pll2_freq = 3000000000U;
	dev->pll1_loop_bw = 200;
	dev->sysref_timer_div = 1024;   /* non-MCS */
	dev->pll1_ref_prio_ctrl = 0xe4; /* non-MCS */
	dev->pll1_ref_autorevert_en = false;
	dev->clkin0_rfsync_en = false;
	dev->clkin1_vcoin_en = false;
	dev->high_performance_mode_clock_dist_en = false;
	dev->rf_reseeder_en = true;     /* !rf_reseeder_disable */
	dev->sync_pin_mode = 0x1;
	dev->pulse_gen_mode = 0x0;      /* non-MCS */

	dev->in_buf_mode[0] = 0x07;
	dev->in_buf_mode[1] = 0x07;
	dev->in_buf_mode[2] = 0x00;
	dev->in_buf_mode[3] = 0x00;
	dev->in_buf_mode[4] = 0x15;

	dev->gpi_ctrl[0] = 0x00;
	dev->gpi_ctrl[1] = 0x00;
	dev->gpi_ctrl[2] = 0x00;
	dev->gpi_ctrl[3] = 0x00;

	dev->gpo_ctrl[0] = 0x37;
	dev->gpo_ctrl[1] = 0x33;
	dev->gpo_ctrl[2] = 0x00;
	dev->gpo_ctrl[3] = 0x00;

	dev->channels = hmc7044_channels;
	dev->num_channels = ARRAY_SIZE(hmc7044_channels);
}

int hmc7044_probe(void)
{
	uint8_t val = 0;
	int ret;

	LOG_INF("HMC7044 scratchpad probe over %s", hmc7044_spi.bus->name);

	if (!spi_is_ready_dt(&hmc7044_spi)) {
		LOG_ERR("SPI bus %s not ready", hmc7044_spi.bus->name);
		return -ENODEV;
	}

	/* Mirror no-OS hmc7044_read_write_check(): write 0xAD, read it back. */
	ret = hmc7044_write(HMC7044_REG_SCRATCHPAD, HMC7044_SCRATCH_PATTERN);
	if (ret) {
		LOG_ERR("scratchpad write failed (%d)", ret);
		return ret;
	}

	ret = hmc7044_read(HMC7044_REG_SCRATCHPAD, &val);
	if (ret) {
		LOG_ERR("scratchpad read failed (%d)", ret);
		return ret;
	}

	LOG_INF("scratchpad readback = 0x%02x (wrote 0x%02x)",
		val, HMC7044_SCRATCH_PATTERN);

	if (val != HMC7044_SCRATCH_PATTERN) {
		LOG_ERR("Scratchpad mismatch (got 0x%02x, expected 0x%02x)",
			val, HMC7044_SCRATCH_PATTERN);
		LOG_ERR("If 0x00/0xff: check CS wiring (CLK_CS=0 on SPI1), "
			"SPI ref clock, or 3-/4-wire mode");
		return -ENODEV;
	}

	return 0;
}

int hmc7044_setup_clocks(void)
{
	int ret;

	LOG_INF("HMC7044 clock setup (zcu102 profile) over %s",
		hmc7044_spi.bus->name);

	if (!spi_is_ready_dt(&hmc7044_spi)) {
		LOG_ERR("SPI bus %s not ready", hmc7044_spi.bus->name);
		return -ENODEV;
	}

	hmc7044_apply_init_defaults(&hmc7044_dev);

	ret = hmc7044_setup(&hmc7044_dev);
	if (ret) {
		LOG_ERR("hmc7044_setup failed (%d)", ret);
		return ret;
	}

	return 0;
}
