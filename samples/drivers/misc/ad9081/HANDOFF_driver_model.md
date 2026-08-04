# Translating the AD9081 sample into Zephyr drivers

Status: the sample works. Happy path verified on hardware, and as of
`ef2d1611dd8` all four failure paths are exercised by fault injection and pass
(8/8 checks). This document maps what exists onto the driver model.

**This is a mapping, not a subsystem decision.** For each block it states what
the code currently is, what shape it has to take, and what the real obstacles
are. Where a subsystem choice is open, it lays out the candidates and the
consequences of each rather than picking. Section 8 lists the decisions that are
yours.

---

## 1. Where the work actually is

Line counts, sample source only (`adi_api/` is 32k lines of vendor library,
untouched by any of this):

| File | Lines | What it becomes | Difficulty |
|---|---|---|---|
| `xilinx_transceiver.c` | 2169 | GT divider math, moves verbatim | trivial — no signatures change |
| `hmc7044.c` | 1001 | clock generator driver | low |
| `axi_adxcvr.c` | 685 | GT transceiver driver | medium |
| `jesd_fsm.c` | 456 | the topology + per-device tables | **high — see §5** |
| `axi_jesd204.c` | 440 | JESD204 link-layer driver | medium |
| `ad9081.c` | 421 | MxFE converter driver | low-medium |
| `axi_tpl.c` | 408 | transport-layer driver | medium |
| `fault_injection.c` | 400 | tests, rewritten against new signatures | low but wasted |
| `main.c` | 223 | shrinks to ~40 lines | trivial |
| `jesd204_fsm.c` + `.h` | 371 | the framework — subsystem or library | **decision, §5** |
| `axi_jesd.c` | 133 | **delete** — see §3 | trivial |

Total ~7.5k lines. The honest split: roughly 5k lines is mechanical
transcription, ~1.5k needs real design thought, and ~1k (the FSM) is the actual
hard problem.

---

## 2. The five obstacles, in order of severity

Everything else in this document is detail. These are the things that make the
conversion more than mechanical.

### 2.1 Nothing is instantiable (affects every block)

Every hardware block is a file-scope singleton with `void`-argument entry points:

```c
static struct axi_jesd204 jesd_tx = { .base = JESD204_TX_BASE, ... };  /* :107 */
static struct axi_jesd204 jesd_rx = { .base = JESD204_RX_BASE, ... };  /* :114 */

int axi_jesd204_tx_lane_clk_enable(void);   /* which instance? the linker decides */
```

Same pattern in `axi_adxcvr.c:155,165` and `axi_tpl.c:102,108`. There are exactly
two of each block and the code hardcodes both. A second link, or a board with one
direction only, cannot be expressed.

This is the single most pervasive change: every public function grows a
`const struct device *dev` first parameter, every `static struct` becomes
`DEVICE_DT_INST_DEFINE` config/data pairs. It touches ~40 function signatures but
each individual change is trivial and the compiler finds them all.

The split matters: `config` is `const` and lives in ROM (base address, geometry,
phandles); `data` is mutable RAM (probed `num_lanes`, `version`,
`data_path_width`). Currently both are mixed in the same mutable struct.

### 2.2 Eight hardcoded base addresses (affects 5 files)

```
axi_jesd204.c:33-34   0x84A90000  0x84B90000   JESD204 RX/TX link cores
axi_tpl.c:35-36       0x84A10000  0x84B10000   TPL RX/TX transport cores
axi_adxcvr.c:42-43    0x84A60000  0x84B60000   GT transceivers RX/TX
axi_jesd.c:34-35      0x84A90000  0x84B90000   (duplicates the link cores)
ad9081.c:42           0xff040000               PS SPI0
hmc7044.c:41          0xff050000               PS SPI1
```

All become `reg` in devicetree. The DMACs are already done this way in
`boards/zynqmp_apu.overlay` and are the pattern to copy.

**The MMU wrinkle, and it is not cosmetic.** These PL pages are not in the A53
SoC MMU table, so each is mapped by a hand-written `SYS_INIT(..., PRE_KERNEL_1)`
that calls `device_map()` and asserts `virt == phys` (`axi_jesd.c:58-78`). The
two SPI pages are mapped the same way from `ad9081.c:45-59` and
`hmc7044.c:44-58` — a *device driver mapping its own parent bus controller's
register page* because the Cadence SPI driver initialises at `POST_KERNEL`,
after the mapping is needed.

That is backwards and cannot survive into drivers. Options:

- **`DEVICE_MMIO_ROM`/`DEVICE_MMIO_MAP`** in each PL driver's own `init()` — the
  idiomatic Zephyr answer for the PL cores, and it removes all the `SYS_INIT`
  hooks. Works because each driver maps only its own `reg`.
- **The two SPI pages are a different problem.** They belong to `cdns,spi`, not
  to us. The correct fix is upstream: `zynqmp_a53.dtsi` should carry the SPI
  controllers, or the Cadence driver should map its own page. Until then a
  board-level `SYS_INIT` is the least-wrong holding position — but it should live
  in board support, not inside the AD9081 and HMC7044 drivers.

Depends on `CONFIG_KERNEL_DIRECT_MAP=y` for `virt == phys`. Worth an explicit
`BUILD_ASSERT` rather than an unstated assumption, since every `sys_read32` in
the tree assumes it.

### 2.3 Link geometry is compile-time (affects `axi_jesd204.c`, `axi_tpl.c`)

```c
#define JESD_L  4   /* lanes */          #define JESD_M  8   /* converters */
#define JESD_F  4   /* octets/frame */   #define JESD_K  32  /* frames/multiframe */
#define JESD_N  16  #define JESD_NP 16  #define JESD_S 1  #define JESD_CS 0
#define JESD_HD 0   #define JESD_SUBCLASS 1  #define JESD_VERSION 1
#define JESD_SCRAMBLING 1
```

`axi_jesd204.c:81-92`. Only `num_lanes` is probed from the core
(`axi_jesd204.c:215`); everything else is a build-time constant of one profile.
A driver serving any other link mode has to read these from devicetree. Linux's
`adi,jesd204-*` properties are the naming precedent.

Note `JESD_VERSION 1` means 204B. The `#define` is load-bearing for the ILAS
checksum (`jesd_ilas_chksum()`) and for the lane-status polarity — the 64B/66B
path is explicitly *not* handled (`axi_jesd204.c:301`). A 204C link is a real
feature addition, not a devicetree property.

The `struct jesd204_link` in `jesd204_fsm.h:89-92` currently carries only
`link_id` and `is_transmit` because the geometry is compile-time. Once geometry
is per-instance, this struct grows to carry the negotiated parameter set — which
is what no-OS's equivalent already does, and is a prerequisite for §5.

### 2.4 Chip profile is compile-time (affects `ad9081.c`)

```c
#define AD9081_DAC_CLK_HZ 12000000000ULL   /* :263 */
#define AD9081_ADC_CLK_HZ  4000000000ULL
#define AD9081_REF_CLK_HZ   250000000ULL
#define AD9081_TX_MAIN_INTERP 6            /* :268 */
#define AD9081_TX_CHAN_INTERP 8
```

Plus per-channel NCO shifts and gains as static arrays, and the +2 GHz main NCO
shift at `ad9081.c:272`. Same treatment: devicetree properties. This is
straightforward — ADI's Linux binding
(`Documentation/devicetree/bindings/iio/adc/adi,ad9081.yaml`) already names every
one of these and should be followed rather than invented.

### 2.5 The FSM has no natural device (the real problem — §5)

---

## 3. Delete `axi_jesd.c` first

`axi_jesd.c` (133 lines) reads MAGIC/VERSION/SYNTH_NUM_LANES from both link cores
and validates them. `axi_jesd204.c:199-218` reads *the same three registers from
the same two cores* and validates them again, more thoroughly. The bases are
duplicated verbatim (`axi_jesd.c:34-35` == `axi_jesd204.c:33-34`).

It exists because it was written first, as a standalone "is the PL alive" probe
before the link driver existed. It is now redundant: identity validation is
exactly what a driver's `init()` does, and `jesd_configure()` already does it.

Do this before anything else — it removes one of the five duplicate base-address
sites and one `SYS_INIT` mapping hook for free. `main.c:122-127` loses its
`axi_jesd_probe()` call; the driver's own init covers it.

---

## 4. Block-by-block mapping

For each: what it is now, what it becomes, and what is genuinely hard.

### 4.1 HMC7044 — clock generator

**Now:** `hmc7044_probe()` (scratchpad check), `hmc7044_setup_clocks()`
(PLL1/PLL2 + output dividers + SYSREF, all hardcoded to the zcu102 profile). SPI
device, already in devicetree as `adi,hmc7044-probe`.

**Becomes:** an SPI device driver. Probe and clock setup both fold into
`init()` — that is what init is for, and nothing else can run before the clock
tree is locked.

**Subsystem candidates:**

- **`clock_control`** — the semantic fit. Its API is
  `on/off/get_rate/set_rate/get_status` keyed by a `clock_control_subsys_t`,
  which maps cleanly onto "14 outputs, each with a divider." Consumers
  (`ad9081`, `axi_adxcvr`) reference it by phandle and ask for rates instead of
  assuming them. Cost: the sample never changes a rate at runtime, so most of
  the API would be `-ENOTSUP` — you would be implementing an interface for one
  static configuration.
- **`misc`** — honest about what it does (configure once at boot, never
  touched again) but gives consumers no way to query rates, so the 500 MHz
  refclk stays a hardcoded assumption in `axi_adxcvr.c:157`.

**The SYSREF wrinkle, whichever you pick.** SYSREF is not a clock in the
`clock_control` sense — it is a subclass-1 alignment pulse whose *phase* relative
to the link matters. On this board the HMC7044 emits it continuously at 1.953 MHz
so nothing has to gate it, which is why `jesd_fsm.c:36-39` registers no
`sysref_cb` at all. A board that gates SYSREF needs the framework hook that
already exists (`jesd204_sysref_async()`), driven from the clock driver. Design
for that even though this board does not exercise it — and note the hook is
**untested**: no device on this board registers a callback, so
`jesd204_fsm.c:44-62` has only ever taken its no-callback path.

### 4.2 AD9081/AD9082 — MxFE converter

**Now:** `ad9081_probe(&prod_id)`, `ad9081_setup_datapath()`,
`ad9081_get_device()` returning the raw `adi_ad9081_device_t *` that
`jesd_fsm.c:71-74` reaches through for every chip call. Six HAL callbacks bridge
the vendor library to Zephyr SPI.

**Becomes:** an SPI driver. Probe → `init()`. Datapath setup → `init()` or an
explicit call, depending on whether you want the chip configured before the FSM
runs (it currently is, and the FSM depends on it).

**Subsystem candidates:**

- **`adc` + `dac`** — Zephyr has both, and an MxFE is literally both. But
  `dac.h`'s API is `channel_setup` + `write_value` for a single sample, and
  `adc`'s is a blocking `adc_read` of a sequence. Neither models a
  JESD204-streamed converter where samples flow over lanes and the CPU never
  touches them. You would implement the ops as `-ENOTSUP` and put the real API
  elsewhere, which is worse than not claiming the subsystem.
- **`mfd`** — defensible: the chip *is* multi-function (DAC, ADC, two JESD
  links, NCOs, PLL) and `mfd` exists for exactly the "one device, several
  unrelated capabilities" case.
- **A custom API under `drivers/misc/` or a new class** — most honest. There is
  no JESD204 converter subsystem in Zephyr today (`grep -rl jesd204 drivers/
  dts/bindings/ include/` returns nothing). Creating one is a bigger claim but
  matches what the hardware is.

**Real obstacle regardless of choice:** `ad9081_get_device()` leaks the vendor
handle so the FSM can call `adi_ad9081_*` directly. That has to become driver
ops (`->oneshot_sync()`, `->link_enable()`, ...) or the FSM stays welded to this
one chip. This is the same coupling as §5 and should be solved once.

### 4.3 AXI ADXCVR — GT transceiver PHY

**Now:** two singletons with per-instance config already cleanly separated
(`sys_clk_sel`, `out_clk_sel`, `lpm_enable`, `lane_rate_khz`, `ref_rate_khz` —
`axi_adxcvr.c:155-173`). `configure()` / `tx_enable()` / `rx_enable()`.

**Becomes:** the easiest real conversion in the set. That struct is already a
devicetree `config` in all but name — TX is QPLL0/no-LPM, RX is CPLL/LPM, both
10 Gbps off 500 MHz refclk. Move it to `reg` + properties and the driver falls
out.

`xilinx_transceiver.c` (2169 lines) reaches the hardware through exactly two
functions, `adxcvr_drp_read/write` declared in `xcvr_shim.h`. Give those a
`const struct device *` and the whole file moves verbatim — as it should, it is
vendor divider math.

**Subsystem candidates:** `phy` is the closest conceptually but Zephyr's `phy` is
Ethernet-specific. Realistically `misc`, or grouped with whatever the JESD204
answer turns out to be. It has no meaning outside a JESD204 link.

**Fault-injection note:** `axi_adxcvr_fi_rx_break_refclk()` works by re-pointing
`sys_clk_sel` at an undriven QPLL1. When that becomes a devicetree property the
test can set it per-instance instead of poking a register — a genuine
improvement, and one of the few places the conversion makes testing *easier*.

Also fold in: `axi_adxcvr_enable()` is dead code (0 call sites — the FSM calls
the per-direction variants). Delete during the move.

### 4.4 AXI JESD204 — link layer

**Now:** two singletons, `configure()` / `{tx,rx}_lane_clk_enable()` /
`rx_watchdog()` / `status_read()` / `link_is_data()`. Reads `num_lanes`,
`version`, `data_path_width` from the core.

**Becomes:** the block where §2.1 and §2.3 both bite. Geometry to devicetree,
singletons to instances, and the RX/TX asymmetry made explicit — RX has the
watchdog and per-lane status, TX has ILAS words. Currently one struct with a
`bool tx` and branches (`axi_jesd204.c:98`); as drivers, either two compatibles
(`adi,axi-jesd204-rx` / `-tx`) or one with a direction property. Two compatibles
matches the IP, which really is two different cores with different register maps.

**Preserve exactly:** the lane-status polarity at `axi_jesd204.c:296-313`.
Non-zero low two bits means *healthy* for 8B/10B. This was inverted once
(fixed in `d0b385f3f8d`) and the inverted version bounced healthy links. The
fault-injection suite has a regression test for it; keep that test alive through
the conversion.

### 4.5 AXI TPL — transport layer

**Now:** `configure()` / `enable()` / `tx_dds()` / `adc_pn_mon()`. Two
singletons. `TPL_NUM_CHANNELS 8` hardcoded.

**Becomes:** structurally the same conversion as the link cores. The DDS is the
interesting part — `axi_tpl_tx_dds(freq, sample_rate, scale, enable)` is the one
place in the sample that produces observable analog output, and it is a
reasonable `dac`-subsystem citizen if you want one: "set a channel's output."

`axi_tpl_adc_pn_mon()` is dead code (0 call sites). Delete during the move.

The `DATA_SELECT` mux (0 = DDS, 2 = DMA) is where a future DMA streaming path
attaches. Worth keeping as an explicit driver op rather than a side effect of
`tx_dds()`, since that is the seam any real datapath work goes through.

---

## 5. The FSM — the actual hard problem

Everything above is transcription. This is design.

### 5.1 Why it does not fit

`jesd204_fsm.c` + `.h` (371 lines) is a **coordinator across devices**, not a
device. It walks 17 phases; within each phase it visits every device in topology
order, so all devices finish `CLK_SYNC_STAGE1` before any starts
`CLK_SYNC_STAGE2`. That op-major ordering is the entire reason the framework
exists — a JESD204 link only comes up when the clock chip, converter, GT and link
cores are stepped through the phases *together*.

The devicetree has no natural way to say that. It expresses parent/child and
phandle references, not "these N devices participate in a phased sequence, in
this order, and here is the order."

Concretely, the ordering is currently carried by an array index:

```c
static struct jesd204_topology topology = {
	.devs = { &adxcvr_jdev, &ad9081_jdev, &axi_jesd204_jdev },
	.devs_number = 3,
	.link = { .link_id = 0, .is_transmit = false },
};
```

`jesd_fsm.c:416-427`. That order is load-bearing and was already a bug once: an
earlier version put `ad9081_jdev` first, which silently moved the chip's JESD PLL
check ahead of the GT reset-release. The comment at `:401-415` exists because the
phase tables alone do not determine it.

### 5.2 What Linux does

ADI's Linux JESD204 framework solves this with `jesd204-device` nodes and
`jesd204-inputs` phandles forming a DAG, and the framework derives the walk order
from the topology. That is the proven design and the reference to follow, but it
is a substantial piece of infrastructure — a devicetree binding, a registration
mechanism, topology resolution, and a link-parameter negotiation pass that this
port does not have (§2.3: `struct jesd204_link` currently carries two fields
where Linux's carries the full parameter set).

### 5.3 Three options

**(a) Keep it as a library, topology in C.** The FSM stays roughly as-is;
`state_ops` tables move into each driver and are reached via a driver op. The
topology stays a C array in board or app code. *Pro:* smallest change, preserves
verified behaviour, ships. *Con:* device order stays implicit in an array index —
the thing that caused a bug. Mitigate with a `BUILD_ASSERT` or a runtime
ordering check.

**(b) A real subsystem, topology in devicetree.** `jesd204-device` +
`jesd204-inputs` phandles as in Linux; a new `include/zephyr/drivers/jesd204.h`
API; the framework derives order from the DAG. *Pro:* correct, upstreamable, and
the ordering bug becomes structurally impossible. *Con:* the largest piece of
work in the entire conversion — likely bigger than all the block drivers
combined.

**(c) Hybrid: devicetree lists the participants, C keeps the order.** Devices are
found by DT (so `status = "disabled"` works and a second link is expressible),
but the phase order stays the enum and device order stays explicit. *Pro:* most
of the benefit, a fraction of the cost. *Con:* half-measure; a future upstream
attempt would revisit it.

If the goal is "prove Zephyr can do this, then upstream," (a) then (b) as a
separate effort is the lower-risk path — (a) is a refactor of verified code,
whereas (b) is new infrastructure whose bugs would be indistinguishable from
conversion bugs.

### 5.4 Framework details worth not losing

- **The FSM is best-effort by design.** `jesd204_fsm_start()` does not abort on
  failure; it counts, logs per phase, and returns the total. A link stalls at the
  *first* broken phase and aborting there hides downstream state. no-OS goes
  further and discards return values entirely (`jesd204-fsm.c:31-43`, returns 0
  unconditionally); this port counts them, which is strictly more informative.
  Verified by fault-injection test 1. Do not "fix" this into fail-fast.
- **`state_op::mode` is never read** — by this walker or no-OS's. It is
  declarative documentation (`jesd204_fsm.h:52-55`). Either keep it documented as
  such or drop it; do not start branching on it.
- **Teardown is a partial unwind and that is why it works.** It disables the
  chip's JRX deframer only; GT, lane clocks and HMC7044 stay running, which is
  exactly why the second bring-up succeeds (verified). If drivers gain real
  `disable` entry points (`adxcvr_clk_disable()`,
  `axi_jesd204_*_lane_clk_disable()` exist in no-OS), teardown becomes a *fuller*
  unwind and re-bring-up needs re-verifying. Do not add them casually.
- **`post_state_sysref` / `sysref_cb` is untested** — no device registers one, so
  only the no-callback path has ever run.

---

## 6. What the sample becomes

`main.c` drops from 223 lines to roughly 40. Everything from `hmc7044_probe()`
through `axi_tpl_configure()` — nine sequential calls with error handling —
becomes driver `init()`, ordered by devicetree dependencies and init levels
rather than by statement order. What survives:

```c
const struct device *link = DEVICE_DT_GET(DT_NODELABEL(jesd_link0));

jesd204_bringup(link);              /* or whatever §5 settles on */
axi_jesd204_rx_watchdog(rx_core);
axi_tpl_tx_dds(dac_tpl, 3000000, 250000000, 50000, true);
```

**This is the main risk of the whole conversion.** The nine explicit calls
currently produce nine `SUCCESS:` log lines in a fixed, readable order — the
thing that made the boot log diffable and let both prior bug hunts succeed. Init
level ordering is implicit and much harder to reason about when it goes wrong.
Two mitigations: keep the per-block log lines in each driver's `init()`, and diff
the new boot log against the current one line by line before trusting it.

---

## 7. Sequencing

Ordered so each step is independently verifiable against a boot log and nothing
depends on §5 until the end.

| # | Step | Verify by |
|---|---|---|
| 0 | Delete `axi_jesd.c` (§3) | boot log identical minus one line |
| 1 | Move `xilinx_transceiver.c` unchanged; add `dev` to the two shim functions | boot log identical |
| 2 | Convert `axi_adxcvr` (easiest, config already separated) | GT lines identical; FI test 4 passes |
| 3 | Convert `axi_tpl` (self-contained, delete `adc_pn_mon`) | TPL status + DDS lines identical |
| 4 | Convert `axi_jesd204` (§2.3 geometry to DT) | link status identical; FI test 2 passes |
| 5 | Convert `hmc7044` (§4.1) | PLL lock lines identical |
| 6 | Convert `ad9081` (§4.2, incl. removing `get_device()` leak) | chip status identical |
| 7 | The FSM (§5) — whichever option | full FI suite, 8/8 |
| 8 | Rewrite fault injection against new signatures | 8/8 again |

Steps 0–6 are independent of each other and of the subsystem decisions; only 7
needs those settled. Steps 2–6 can proceed in any order or in parallel.

**Keep the fault-injection suite runnable at every step.** It is the only
mechanical check that a refactor did not break a failure path — and failure paths
are precisely what a passing boot cannot verify. Expect step 8's rewrite to be
mostly signature churn; the four faults themselves and what they assert do not
change.

---

## 8. Decisions that are yours

Subsystem placement, per block. The candidates and their consequences are in §4;
the summary:

| Block | Candidates | The tension |
|---|---|---|
| HMC7044 | `clock_control` / `misc` | semantic fit vs. mostly-`-ENOTSUP` API for one static config |
| AD9081 | `adc`+`dac` / `mfd` / custom | Zephyr's `adc`/`dac` model CPU-read samples, not JESD-streamed ones |
| ADXCVR | `misc` / with JESD204 | no standalone meaning; `phy` is Ethernet-only |
| JESD204 link | `misc` / new subsystem | nothing in-tree to join (`grep jesd204 drivers/` → nothing) |
| TPL | `misc` / `dac` | the DDS is a plausible `dac`; the rest is not |
| FSM | §5 (a) / (b) / (c) | ship-now vs. upstreamable-later |

Three more that are not subsystem choices but are yours:

1. **How far to go on §2.3/§2.4** — full devicetree parameterisation, or leave
   this profile hardcoded and parameterise only when a second board appears.
   Hardcoded is defensible for a proof-of-concept and cuts real work; it just
   means the drivers serve one bitstream.
2. **Whether the two SPI `SYS_INIT` mapping hooks get fixed properly** (§2.2) —
   upstream `zynqmp_a53.dtsi`/`cdns,spi` change, or a board-level holding
   position. The current arrangement, where converter drivers map their own bus
   controller's page, should not survive either way.
3. **Whether 204C is in scope.** `JESD_VERSION 1` is load-bearing in the ILAS
   checksum and the lane-status polarity; the 64B/66B path is explicitly not
   handled. This is a feature, not a property.

## 9. What is verified, and what that does not cover

Carrying this into the conversion as the baseline to diff against:

**Verified on hardware:** the full bring-up (~2.83 s to
`=== bring-up complete ===`); all four FI faults, 8/8 checks; link reaches DATA
three times in one boot (normal, post-teardown, post-GT-restore); watchdog
detects a forced desync, bounces, recovers in ~120 ms, and leaves a healthy link
alone; GT with no refclk reports `-ETIMEDOUT` after ~229 ms with `STATUS=0x10`.

**Not covered:**
- A *real* lane desync. `LANE_STATUS` is read-only and driven by the core's
  alignment logic, so FI test 2 substitutes the value the watchdog reads.
  The decision logic and recovery are proven; whether this board ever genuinely
  reports a desync is unknown.
- Repeated teardown/bring-up cycles. One cycle is verified.
- The SYSREF callback path (§5.4).
- Elastic-buffer clear loop, `axi_adxcvr.c:568-587`, 100 retries. Not
  injectable in software — needs a real buffer under/overflow.
- Any analog claim. The 2.003 GHz tone (3 MHz DDS + 2 GHz NCO) at ~−26 dBFS has
  never been observed on a scope; an ADALM2000 cannot see it.
