# AD9081 sample → Zephyr drivers: implementation plan

Supersedes the analysis in `HANDOFF_driver_model.md`, which surveyed the options.
This picks among them and sequences the work. Where it departs from the handoff,
it says so and why.

Baseline: the sample works. Bring-up in ~2.57 s, all four fault-injection faults
pass, 10/10 checks. (Earlier revisions of this plan said 8/8; that predates the
`teardown-effect` and `watchdog-no-false-positive` checks.) Both baselines are
now committed as `boot_log.golden` and `boot_log_fi.golden`. Steps 0, 1 and 6 are
**done and hardware-verified**.

---

## 0. What this converts, and what "driver model" buys

The sample is a flat application: nine hardware blocks, each a file-scope
singleton with `void`-argument entry points, called in sequence from `main.c`.
Base addresses, link geometry and chip profile are `#define`s.

Zephyr's model instead describes hardware in devicetree and creates driver
*instances* from it: a `compatible` string matches a node to a driver, and
`DEVICE_DT_INST_DEFINE` generates one `struct device` per node with a `const`
ROM `config` (base address, geometry, phandles) and a mutable RAM `data`
(probed `num_lanes`, `version`, `data_path_width`). Every function takes
`const struct device *dev` first.

What that actually buys:

| | Now | As drivers |
|---|---|---|
| Two link cores | two hardcoded singletons | two DT nodes, one driver |
| A second link | not expressible | add a node |
| Disable RX only | edit code | `status = "disabled"` |
| Init ordering | statement order in `main.c` | DT dependencies + init levels |
| Different bitstream | rebuild the app | new overlay |
| Upstreamable | no | yes |

The last row is the point. Nine hardcoded base addresses is a demo; drivers plus
an overlay is something ADI or AMD could merge.

---

## 1. Decisions taken

### 1.1 Build steps 3–7 as real drivers that live in the sample directory

The expensive, hard-to-reverse work is the **API shape**: `dev`-first
parameters, const-config vs mutable-data, geometry from DT, own compatibles and
bindings. Where the `.c` file finally sits is a `git mv` plus a Kconfig line.

So do the expensive part now, against a boot log that can be diffed after every
step, and defer the directory decision until step 8 reveals whether ADXCVR /
link / TPL become a subsystem or stay `misc` (§1.2.1). This decouples steps 2–6 from
subsystem placement entirely.

**Amended for `hmc7044` only.** Its placement was never in doubt — it is a
`clock_control` driver and that subsystem already exists, so there was nothing
for step 8 to reveal. It has therefore been moved in-tree already, ahead of the
rest:

| | |
|---|---|
| driver | `drivers/clock_control/clock_control_hmc7044.c` |
| extension API | `include/zephyr/drivers/clock_control/hmc7044.h` |
| binding | `dts/bindings/clock/adi,hmc7044.yaml` |
| Kconfig | `drivers/clock_control/Kconfig.hmc7044` |

`CLOCK_CONTROL_HMC7044` defaults to `y` on `DT_HAS_ADI_HMC7044_ENABLED`, so the
sample enables it by having the devicetree node plus `CONFIG_CLOCK_CONTROL=y`;
it no longer names the driver in `target_sources()`. The reasoning above still
holds for steps 2–5 and 7: those blocks have no obvious subsystem, so they stay
sample-local until step 8 decides.

### 1.2 Placement, per block

| Block | Lines | Placement | Why |
|---|---|---|---|
| `xilinx_transceiver.c` | 2169 | moves verbatim | vendor divider math; only 2 functions touch hardware |
| `hmc7044.c` | 1001 | `clock_control` — **moved in-tree**, see §1.3 outcome | it *is* a clock controller — §1.3 |
| `axi_adxcvr.c` | 685 | sample-local, `misc` **provisionally** | placeholder for a JESD204 subsystem — §1.2.1 |
| `axi_jesd204.c` | 440 | sample-local, split RX/TX, `misc` **provisionally** | two different cores, two register maps |
| `ad9081.c` | 421 | `misc/ad9081/` + custom API header | the in-tree ADI precedent — §1.4 |
| `axi_tpl.c` | 408 | sample-local, `misc` **provisionally** | same as adxcvr |
| `axi_jesd.c` | 133 | **deleted** | done; §2 step 0 |

Rejected, with the reason:

- **`adc` / `dac` for AD9081.** `dac.h` is `channel_setup` + `write_value` for
  one sample; `adc.h` is a blocking `adc_read`. Both model the CPU touching
  samples. In JESD204 samples flow over serial lanes and the CPU never sees
  them. This is the case where joining a subsystem actively *misleads*: an
  `adc_read()` that returns `-ENOSYS` on a device which emphatically does
  convert is worse than not claiming the API.
- **`mfd` for AD9081.** `mfd` is for a parent owning bus access on behalf of
  devicetree *children* (a PMIC with regulator/GPIO sub-nodes). AD9081 has no
  children. It is multi-function in the English sense, not the `mfd` sense.
- **`phy` for ADXCVR.** Zephyr's `phy` is Ethernet-specific.

Verified: `grep -ril jesd204 drivers/ include/ dts/bindings/` returns nothing.
There is no JESD204 anything in Zephyr. For ADXCVR, the link cores and the TPL
there is no subsystem to join — only one to found.

#### 1.2.1 `misc` is provisional for the three PL blocks, not an answer

For ADXCVR, the link cores and the TPL, `misc` is a placeholder meaning "the
JESD204 subsystem that does not exist yet." Recorded explicitly because
"`misc` because it is convenient" is how a subsystem gap becomes permanent: each
driver that routes around the gap removes the pressure to close it, and the next
JESD204 device in Zephyr inherits the same non-answer.

The accommodating thing is eventually to create the subsystem (the handoff's
option (b): `include/zephyr/drivers/jesd204.h`, a binding, topology resolution).
That stays a later, separate effort — landing new infrastructure and the driver
conversion in the same commits would make their bugs indistinguishable. But the
plan should not pretend `misc` is where these belong.

### 1.3 HMC7044 → `clock_control`. Settled, and the objection to it was wrong.

**Correction.** Earlier drafts of this plan (and the handoff's §4.1) framed this
as "semantic fit vs. a mostly-`-ENOTSUP` API," and treated the stub cost as a
real reason to hesitate. That cost does not exist.

Every field in `clock_control_driver_api` is tagged `@driver_ops_optional`
(`include/zephyr/drivers/clock_control.h:147-162`), and every public wrapper
NULL-checks and returns `-ENOSYS` on the caller's behalf:

```c
if (api->get_rate == NULL) {
	return -ENOSYS;
}
```

So unimplemented ops are simply **left NULL** — no stubs are written at all.
That is the documented design, not a driver shirking a contract.

`clock_control_fixed_rate.c` is the in-tree proof: a complete upstream driver
implementing four ops, two of which are `return 0;`, with `async_on`, `set_rate`
and `configure` absent entirely. Nobody treats it as a bad citizen.

The HMC7044 would implement *more* than that:

| Op | HMC7044 |
|---|---|
| `get_rate` | real — per-output divider math |
| `get_status` | real — PLL1/PLL2 lock, currently invisible outside the driver |
| `on` / `off` | real — per-output enable |
| `set_rate`, `async_on`, `configure` | left NULL → `-ENOSYS` |

**Why it is the right home on the merits.** A 14-output, SPI-programmed clock
generator with two PLLs is a clock controller by any reasonable reading. And one
op is load-bearing today: `xilinx_transceiver.c` computes GT dividers from
`ref_rate_khz`, hardcoded to 500 MHz at `axi_adxcvr.c:157`. As a clock provider
that becomes `clock_control_get_rate()` against the output which actually drives
the GT — an unstated assumption becomes a queried fact, and if the clock tree is
ever reprogrammed the divider math follows instead of silently going wrong.

`clock_control` currently contains no discrete off-SoC clock chips — no
`drivers/clock_control/*.c` references `spi_dt_spec` or `i2c_dt_spec`. That is
an argument for being first, not against: putting the HMC7044 in `misc` because
the subsystem happens to hold only SoC-internal drivers today would perpetuate
the gap and leave its rates unqueryable by anything that is not our own code.

One genuine constraint, which is a documentation matter rather than a design one:

- **It cannot be `PRE_KERNEL_1`.** Clock providers conventionally are — note
  `fixed_rate` is, since it needs no bus — but an SPI-attached one cannot:
  `spi_cdns` is `POST_KERNEL, CONFIG_SPI_INIT_PRIORITY`
  (`drivers/spi/spi_cdns.c:824`). This works here only because every consumer is
  also POST_KERNEL. Put it in a comment at the `DEVICE_DT_INST_DEFINE` and in
  the binding description, since a future consumer is what would trip over it.
- **SYSREF is not a clock.** It is a subclass-1 alignment pulse whose *phase*
  relative to the link matters, and it does not fit `clock_control_subsys_t`.
  Put `hmc7044_sysref_request()` in an extension header beside the standard API,
  which is how the subsystem already handles vendor-specific ops.

Note: no `drivers/clock_control/*.c` currently contains `spi_dt_spec` or
`i2c_dt_spec` — every driver there is an SoC-internal register block. An
SPI-attached clock generator would be the first of its kind.

**Outcome (step 6, hardware-verified; now in-tree — paths in §1.1).** Implemented as designed above: `on`,
`off`, `get_rate`, `get_status` real; `set_rate`, `async_on`, `configure` left
NULL. The chip's configuration moved out of C arrays into the `adi,hmc7044`
binding using ADI's Linux property names, so a DTS written for ADI Linux ports
with syntax changes only. Both POST_KERNEL constraints are documented where this
section asked for them, plus a `BUILD_ASSERT` that the init priority exceeds
`CONFIG_SPI_INIT_PRIORITY`. `hmc7044_sysref_request()` and `hmc7044_get_status()`
live in the extension header.

The load-bearing op paid off exactly as argued: `axi_adxcvr.c` no longer defines
`ADXCVR_REF_CLK_KHZ 500000`. It queries the output that drives the GT and solves
the dividers against the answer — `GT refclk from hmc7044@0 out12: 500000 kHz`,
then `GT dividers programmed for 10000000 kHz lane @ 500000 kHz ref` on both
transceivers. 500 MHz survives only as a sanity check that warns when the tree
disagrees with what the bitstream was synthesised for.

Two implementation details worth recording, neither anticipated here:

- **Output 0 collides with `CLOCK_CONTROL_SUBSYS_ALL`**, which is `NULL`. The
  subsystem handle is therefore the output number biased by one —
  `HMC7044_CLK_OUT(n)`. `SUBSYS_ALL` is given the one defensible whole-chip
  meaning: `get_rate` returns the PLL2 VCO frequency every output divides down
  from.
- **`get_status` gates `ON` on PLL lock, not just the output enable bit.** A
  divider running off an unlocked PLL2 is producing *something*, but not the rate
  `get_rate()` reports — returning `ON` there would lie in precisely the case a
  caller is checking for. Unlocked-but-transient maps to `STATUS_STARTING`; an
  unverifiable read/write path maps to `STATUS_UNKNOWN`.

### 1.4 AD9081 → `misc/` with a custom API, and drop the vendor-handle leak

ADI already set this precedent in-tree: `max2221x` ships a custom API at
`include/zephyr/drivers/misc/max2221x/max2221x.h` with the driver in
`drivers/misc/max2221x/`. That is the in-tree answer for "ADI chip that no
existing subsystem models."

The real obstacle is coupling, not placement. `ad9081_get_device()` returns the
raw `adi_ad9081_device_t *`, and the FSM reaches through it — the `chip()` helper
at `jesd_fsm.c:71-74` — to call `adi_ad9081_*` directly:

```c
adi_ad9081_jesd_oneshot_sync(chip(), JESD_SUBCLASS_1);
```

`jesd_fsm.c` is therefore welded to this one chip. It must become driver ops
(`->oneshot_sync(dev)`, `->link_enable(dev, on)`, ...) or nothing else can ever
participate in the FSM. Same coupling as §1.6; solve it once, in steps 7–8.

### 1.5 The two SPI mapping hooks stay — they are upstream's problem

**Reversed from the handoff.** It proposed patching the Cadence SPI driver as a
prerequisite. `drivers/spi/spi_cdns.c` is upstream code authored by Ryan
McClelland (Meta) and maintained by Srikanth Boyapally, with no local commits.
Modifying it is a separate contribution with its own review cycle, not a
prerequisite for this work.

The wart is real: `spi_cdns.c:816` stores `DT_INST_REG_ADDR(n)` straight into
`cfg->base` and never maps it, and the driver inits at `POST_KERNEL` — after the
mapping is needed. So `ad9081.c:45-59` and `hmc7044.c:44-58` each map their own
parent bus controller's register page.

Handling: **keep the mapping, but move it out of the two chip drivers** into one
sample-owned `SYS_INIT` that maps both SPI pages, with a comment naming the
upstream cause. A converter driver reaching for its bus controller's registers
should not survive; the sample owning a documented board-level workaround is
fine. (The handoff's "board-level SYS_INIT" has nowhere to live: there is no A53
zcu102 board in tree — only `boards/amd/zcu102_r5` — and `zynqmp_a53.dtsi`
carries no `spi@` or `gpio@` nodes at all, which is why the overlay defines
them.)

**This became a prerequisite, not a nicety.** Once `hmc7044` moved to
`drivers/clock_control/`, an in-tree driver could not have kept a mapping hook
for a ZynqMP SPI controller inside it — it would have been unshippable. Doing
§1.5 first is what made the driver's own move possible. `spi_mmio_fixup.c` stays
sample-local permanently and disappears when the upstream fix below lands.

If it is ever worth fixing properly, it is a ~10-line standalone PR against
`spi_cdns.c` (`DEVICE_MMIO_ROM`/`DEVICE_MMIO_RAM` + `DEVICE_MMIO_MAP` in
`spi_cdns_init()`), fully decoupled from this plan.

For the **PL cores**, the idiomatic answer does apply: `DEVICE_MMIO_ROM` in
config, `DEVICE_MMIO_MAP` in each driver's own `init()`, each mapping only its
own `reg`. Those `SYS_INIT` hooks all disappear as steps 2–4 land.

`CONFIG_KERNEL_DIRECT_MAP` is not a decision: `arch/arm64/core/Kconfig:10` does
`select KERNEL_DIRECT_MAP if MMU`. A `BUILD_ASSERT` is still worth adding as
documentation of the `virt == phys` dependency, since every `sys_read32` here
relies on it.

### 1.6 The FSM: sample-local now, module later; topology stays in C

Two independent questions the handoff ran together.

**Where the code lives.** The FSM is 371 lines with no hardware access —
ordering logic over callback tables. It does not need `drivers/`. It stays
sample-local while the block drivers are still moving, then becomes a **Zephyr
module** (own repo, `zephyr/module.yml`, west manifest) once there is a second
consumer. A module is a good fit precisely because there is no in-tree home for
JESD204, and it allows reuse without waiting on upstream review — ADI ship
things this way. Doing it before a second consumer exists is speculative
packaging; the move is cheap (a self-contained file pair with no hardware
dependency).

**Where the topology comes from.** It stays a C array. The ordering *risk* is
real and does not require devicetree to fix:

```c
.devs = { &adxcvr_jdev, &ad9081_jdev, &axi_jesd204_jdev },   /* jesd_fsm.c:416 */
```

That order is load-bearing and was already a bug once — an earlier version put
`ad9081_jdev` first, silently moving the chip's JESD PLL check ahead of the GT
reset-release. The 15-line comment at `:401-415` exists because the phase tables
do not determine the order.

A C topology is also the better fit if the FSM heads for a module, since a module
cannot easily add bindings that in-tree drivers depend on. The DT-topology and
full-DAG designs (handoff §5.3 (b)/(c)) stay available as a later, separate
effort — their bugs would be indistinguishable from conversion bugs if landed
now.

**The ordering check, concretely.** Earlier drafts said "fix it with an explicit
ordering assertion" and left it there. That does not cash out: `BUILD_ASSERT`
cannot express semantic ordering of a pointer array — nothing at compile time
knows that `adxcvr` must precede `axi_jesd204`. It needs a declared rank:

```c
/* Lower rank is visited first within a phase. Ranks encode the hardware
 * dependency, not the array position: the GT must leave reset before the
 * link cores' lane clocks are enabled (no-OS jesd204_clk.c:48-64).
 */
enum jesd204_dev_rank {
	JESD204_RANK_CLOCK   = 0,   /* hmc7044, if it ever registers */
	JESD204_RANK_PHY     = 10,  /* adxcvr — GT reset-release */
	JESD204_RANK_CHIP    = 20,  /* ad9082 — JESD PLL check, 204C cal */
	JESD204_RANK_LINK    = 30,  /* axi-jesd204 — lane clocks, DATA poll */
};

struct jesd204_dev {
	const char *name;
	enum jesd204_dev_rank rank;
	const struct jesd204_dev_data *dev_data;
};
```

`jesd204_fsm_start()` then validates once, before walking, that ranks ascend
across `topology.devs[]` and returns `-EINVAL` if not. The gap-of-10 numbering
leaves room to insert a device without renumbering.

This converts the bug that already happened — `ad9081_jdev` ahead of
`adxcvr_jdev` — from silent misordering into a startup refusal, and it makes the
15-line explanatory comment at `jesd_fsm.c:401-415` redundant because the
constraint is in the data. Cheaper than a devicetree phandle array and it
survives the move to a module.

A runtime check is the only option here: the topology is a file-scope array of
pointers to file-scope structs, so the ranks are not integer constant
expressions available to `BUILD_ASSERT` at the point the array is declared.

### 1.7 204C is out of scope

`JESD_VERSION 1` (204B) is load-bearing in the ILAS checksum
(`jesd_ilas_chksum()`) and the lane-status polarity, and the 64B/66B path is
explicitly not handled (`axi_jesd204.c:301`). This is a feature addition, not a
devicetree property. Out of scope; note it in the bindings.

### 1.8 Open: how far to parameterise (handoff §2.3/§2.4)

**Not yet decided.** Three positions:

- **Link geometry only** — the 11 `JESD_*` defines to DT (both `axi_jesd204`
  and `axi_tpl` consume them), following ADI's `adi,jesd204-*` names. AD9081's
  clock rates, interpolation factors and per-channel NCO/gain arrays stay
  hardcoded. ~12 properties.
- **Full** — also AD9081's profile, following `adi,ad9081.yaml`. ~30
  properties.
- **Neither** — only the base addresses to DT. Smallest diff; the drivers serve
  exactly this bitstream.

Caveat: this is true of steps 3, 4, 6 and 7, but **step 5 *is* the geometry
step** — starting it requires the answer. So either decide before step 5, or
accept that step 5 goes last among 3–7.

---

## 2. Sequencing

### 2.0 Baseline: captured — `boot_log.golden`, `boot_log_fi.golden`

Every step below is verified by diffing a boot log, and originally there was no
committed reference in the repo, so "boot log identical" resolved to whatever was
last in a terminal scrollback on a branch about to accumulate eight refactors.

**Resolved.** `boot_log.golden` holds a hardware-captured default-configuration
boot from the ZCU102, with the provenance, the how-to-recapture recipe, the
known-good PLL/link values, and the three lines a future step is most likely to
perturb spelled out so they are not mistaken for noise.

One caveat recorded in the file itself: the baseline was captured from
`90a9cb34e85` *plus* the then-uncommitted steps 1 and 6, not from a pristine
commit. It is the post-step-6 reference, which is what steps 2–5 and 7–8 need.
There is deliberately no pre-conversion baseline — steps 1 and 6 were verified by
line-by-line comparison against the terminal output of the previous boot, and
their expected three-line diff is documented in `boot_log.golden`.

**Also resolved:** `boot_log_fi.golden` is the equivalent reference for the
fault-injection build, hardware-captured with all 10 checks passing. It was taken
*after* `hmc7044` moved to `drivers/clock_control/`, and its bring-up prefix
matches `boot_log.golden` line for line — which is the hardware confirmation that
the in-tree move changed no behaviour. It also documents what varies legitimately
between runs (the TPL `clk_freq` LSB) and why the `<wrn>`/`<err>` lines under
FI 1 and FI 4 are the point of those tests rather than a failure.

§2.0 is closed. Both baselines exist.

### 2.1 Steps

| # | Step | Verify by |
|---|---|---|
| **0** | **Delete `axi_jesd.c`** — done, not yet booted | boot log identical minus one `SUCCESS:` line |
| **A** | **Capture + commit the golden boot log and FI output (§2.0)** — **done**: `boot_log.golden` + `boot_log_fi.golden`, both hardware-captured | — |
| 1 | Consolidate the two SPI mapping hooks (§1.5); add `BUILD_ASSERT` for `virt == phys` — **done in `src/spi_mmio_fixup.c`, hardware-verified** | boot log identical |
| 2 | Move `xilinx_transceiver.c` unchanged; add `dev` to the two `xcvr_shim.h` functions | boot log identical |
| 3 | Convert `axi_adxcvr` — config already separated, easiest real conversion | GT lines identical; **FI test 4** |
| 4 | Convert `axi_tpl`; delete dead `axi_tpl_adc_pn_mon()` | TPL status + DDS lines identical |
| 5 | Convert `axi_jesd204`, split RX/TX compatibles (needs §1.8 settled) | link status identical; **FI test 2** |
| 6 | Convert `hmc7044` → `clock_control` (§1.3) — **done, hardware-verified: PLL1/PLL2 lock lines identical, GT refclk now queried not hardcoded** | PLL lock lines identical |
| 7 | Convert `ad9081` → `misc` (§1.4), remove `get_device()` leak | chip status identical |
| 8 | FSM: driver ops + rank check (§1.6) | **FI test 1**; full suite 10/10 |

Two deletions to fold in while passing through: `axi_adxcvr_enable()` and
`axi_tpl_adc_pn_mon()` both have zero call sites.

### 2.2 `jesd_fsm.c` is touched at every block step — not only at step 8

**Correction to an earlier claim in this plan** ("steps 2–7 are independent, only
step 8 needs the FSM"). That is wrong. `jesd_fsm.c` calls directly into four of
the blocks being converted:

| Call site | Belongs to | Broken by step |
|---|---|---|
| `axi_adxcvr_tx_enable()` / `rx_enable()` — `:278,283` | adxcvr | 3 |
| `axi_tpl_enable()` — `:376` | TPL | 4 |
| `axi_jesd204_rx_lane_clk_enable()` / `tx_...` / `status_read()` / `link_is_data()` — `:319,324,359,363` | link cores | 5 |
| `chip()` → `adi_ad9081_*` — `:71-226` | ad9081 | 7 |

So each of steps 3, 5, 6 and 7 must thread a `dev` handle through
`jesd_fsm.c`. The steps remain independent **of each other**, but `jesd_fsm.c`
gets edited five times, and step 8 is "the last FSM step," not "the FSM step."

Two consequences:

- Parallelising steps 3–7 across people would conflict in exactly this one file.
  Serialise them, or agree the split inside `jesd_fsm.c` up front.
- The ranked-topology change (§1.6) can land early and independently — it touches
  only `jesd204_fsm.{c,h}` and the topology array, not the per-block call sites.
  Doing it before step 3 means the ordering constraint is protected *while* the
  call sites churn, which is when misordering is most likely to be introduced.

### 2.3 Departure from the handoff on fault injection

The handoff says "keep the FI suite runnable at every step" but also defers the
FI rewrite to a final step 8. Those contradict. The faults are per-block — test
4 is adxcvr, test 2 is the lane-status polarity — so **each fault's port belongs
in the step that breaks it**, as the table above does. That leaves only test 1,
which is FSM-level, for the final step.

One test to guard specifically: the lane-status polarity at
`axi_jesd204.c:296-313`. Non-zero low two bits means *healthy* for 8B/10B. This
was inverted once (fixed in `d0b385f3f8d`) and the inverted version bounced
healthy links. Keep that regression test alive through the conversion.

---

## 3. Risks

### 3.1 Losing the readable boot log

`main.c` drops from 223 lines to ~40:

```c
const struct device *link = DEVICE_DT_GET(DT_NODELABEL(jesd_link0));

jesd204_bringup(link);
axi_jesd204_rx_watchdog(rx_core);
axi_tpl_tx_dds(dac_tpl, 3000000, 250000000, 50000, true);
```

The nine explicit calls it replaces currently produce nine `SUCCESS:` log lines
in a fixed, readable order — the thing that made the boot log diffable and let
both prior bug hunts succeed. Init-level ordering is implicit and much harder to
reason about when it goes wrong.

Two mitigations, both mandatory: **keep the per-block log line inside each
driver's `init()`**, and **diff each step's boot log against the golden capture
(§2.0) line by line** before trusting it.

### 3.2 Hardware access is the real constraint on pace

Steps 3–7 are independent as code, but every one of them is only worth anything
once booted on the board — a passing build proves nothing about a link that comes
up. The FI suite matters for the same reason: failure paths are precisely what a
passing boot cannot verify.

So the sequencing above is not a schedule. It is an order in which changes can be
*validated*, and the throughput limit is board access, not editing.

---

## 4. Behaviour to preserve

### 4.1 The FSM reports failures without aborting — this is better than no-OS

Landed in `5b1f31a208f`. `jesd204_fsm.c:74-101` and `:130-164` do three things
the reference does not:

- `run_dev_op()` returns a per-device-phase failure count (0–2; a phase may
  populate both a `per_device` and a `per_link` callback)
- each failure logs device name, phase name and errno
- per-phase `ok` / `N failure(s)` lines, plus a total returned from
  `jesd204_fsm_start()`

no-OS discards return values entirely (`jesd204-fsm.c:31-43` returns 0
unconditionally), so a callback error there simply vanishes. The distinction:
**non-abort is the design choice** — a link stalls at the *first* broken phase,
and bailing there hides all downstream state — **and full attribution is the
improvement.** Verified by FI test 1. Do not "fix" this into fail-fast, and do
not lose the attribution.

### 4.2 Three more

- **`state_op::mode` is never read** — not by this walker, not by no-OS's. It is
  declarative documentation (`jesd204_fsm.h:52-55`). Keep it documented as such
  or drop it; do not start branching on it.
- **Teardown is a partial unwind, and that is why it works.** It disables the
  chip's JRX deframer only; GT, lane clocks and HMC7044 stay running, which is
  exactly why the second bring-up succeeds. If drivers gain real `disable` entry
  points (no-OS has `adxcvr_clk_disable()`,
  `axi_jesd204_*_lane_clk_disable()`), teardown becomes a fuller unwind and
  re-bring-up needs re-verifying. Do not add them casually.
- **The SYSREF callback path has never run.** No device registers a
  `sysref_cb` — this board's HMC7044 emits SYSREF continuously at 1.953 MHz — so
  `jesd204_fsm.c:44-62` has only ever taken its no-callback path. Design for it;
  know it is untested.

---

## 5. Baseline to diff against

Committed as `boot_log.golden` (default) and `boot_log_fi.golden`
(fault-injection). Diff against those files, not against this prose — they carry
the per-line diff traps and the values that vary legitimately between runs.

**Verified on hardware:** full bring-up ~2.57 s to `=== bring-up complete ===`;
all four FI faults, 10/10 checks; link reaches DATA three times in one boot
(normal, post-teardown, post-GT-restore); watchdog detects a forced desync,
bounces, recovers in ~120 ms, and leaves a healthy link alone; GT with no refclk
reports `-ETIMEDOUT` after ~229 ms with `STATUS=0x10`.

**Not covered — do not let the conversion imply otherwise:**

- A *real* lane desync. `LANE_STATUS` is read-only and driven by the core's
  alignment logic, so FI test 2 substitutes the value the watchdog reads. The
  decision logic and recovery are proven; whether this board ever genuinely
  reports a desync is unknown.
- Repeated teardown/bring-up cycles. One cycle is verified.
- The SYSREF callback path (§4.2).
- The elastic-buffer clear loop, `axi_adxcvr.c:568-587`, 100 retries. Not
  injectable in software — needs a real buffer under/overflow.
- Any analog claim. The 2.003 GHz tone (3 MHz DDS + 2 GHz NCO) at ~−26 dBFS has
  never been observed on a scope; an ADALM2000 cannot see it.

---

## 6. Step 0 record

Deleted `src/axi_jesd.c` (133 lines) and `src/axi_jesd.h` (20). It read
MAGIC/VERSION/SYNTH_NUM_LANES from both link cores and validated them;
`jesd_configure()` in `axi_jesd204.c` reads the same three registers from the
same two cores and validates them more thoroughly. The bases were duplicated
verbatim. It existed because it was written first, before the link driver.

Three things had to move rather than vanish:

1. **The page mapping.** `axi_jesd.c` owned the `SYS_INIT` that 1:1 mapped
   *both* link-core pages, which `axi_jesd204.c` depended on. Moved into
   `axi_jesd204.c` as `axi_jesd204_map()`, the driver that owns those pages.
2. **The lane-count check.** `axi_jesd204.c` read `num_lanes` but never
   validated it; only `axi_jesd.c:108-111` warned on a mismatch. Added at
   `axi_jesd204.c:250-259`, checked against `JESD_L`.
3. **The stale comment** in `axi_jesd204.h` pointing at `axi_jesd.c`'s
   `SYS_INIT`.

`main.c` loses its `axi_jesd_probe()` call and one `SUCCESS:` line. Net −116
lines. Both configurations build clean (default and
`-DEXTRA_CONF_FILE=fault_injection.conf`); remaining warnings are pre-existing
(vendor `adi_api`, `xcvr_shim.h` format strings, upstream DMA driver, SPI DT
macro deprecation).

**Not yet run on hardware.** The expected diff is exactly one fewer `SUCCESS:`
line, plus the new `SYNTH_NUM_LANES` warning if the bitstream is not m8-l4.
