# Handoff: step 7 (FSM to devicetree) and step 8 (fault injection rewrite)

Branch `drivers/ad9801`, head `b9ad406d313`. Steps 1-6 are done and
hardware-verified. This document covers what is left.

## Start here

```bash
cd /home/istolnic/ZephyrOpensource
python3 tools/check_profile.py                    # expect: 116 compared, 0 mismatches
.venv/bin/west build -p always -b zynqmp_apu --build-dir build \
    zephyr/samples/boards/amd/ad9081_chipid
./flash_ad9081.sh && ./serial.sh /dev/ttyUSB0     # capture a boot log BEFORE touching anything
```

Verified this session (2026-08-03), build only, no hardware run:

- `check_profile.py`: **116 compared, 0 mismatches, 0 skipped**
- default build: links
- FI build (`-DEXTRA_CONF_FILE=fault_injection.conf`): **links** — so step 8 is a
  rewrite for correctness, not a build fix. The compiler warnings
  (`xcvr_shim.h:38` format specifiers, `dma_adi_axi_dmac.c:197` unused,
  `adi_ad9081_jesd.c:2111` maybe-uninitialized) are pre-existing and not yours.

**Unverified:** the FI suite has not *run* on hardware since commit `adf13bc0747`.
Three behaviour commits have landed since (`66878fc01a8`, `c4474063b2b`,
`b9ad406d313`). It builds; whether it still passes 10/10 is unknown.

## Blocker before step 8: two golden-log problems

Both need you at the board. Do not let an agent write either file.

1. **`boot_log_fi.golden` is stale.** Line 168 says `=== JESD204 fault
   injection: 4 faults ===`; `src/fault_injection.c:506` says `5 faults`. Stale
   since `adf13bc0747`, which added FI 1b (rank validation) as a fifth fault.
   The log's own header (line 7) says "Four injected faults produce ten FI
   checks" and line 11 already flags that CLAUDE.md's "8 checks" is a third,
   different number. **Reconcile all three on re-capture.** Actual current count
   from the code: **5 faults, 13 `fi_pass` call sites** (`fault_injection.c`
   lines 226, 239, 311, 322, 335, 344, 378, 391, 403, 421, 435, 449, 479, 496 —
   minus the two mutually-exclusive branches).

2. **`flash_ad9081.sh:43` hardcodes `BUILD=build/zephyr`.** So a separate
   `build_fi/` directory is invisible to the flash script and you must rebuild
   in place to run FI. Three-line fix:

   ```sh
   BUILD=${BUILD:-build/zephyr}
   ```

   Then `BUILD=build_fi/zephyr ./flash_ad9081.sh`. Do this first — it saves a
   full rebuild on every FI/normal switch for the rest of step 8.

## Step 7 — the FSM

**Estimated: half a day if the golden diff is clean; a full day if the topology
validation fights the DT ordering.**

### What exists now

`src/jesd_fsm.c` (478 lines) holds three things:

1. **Phase callbacks** — `ad9081_fsm_*`, `adxcvr_fsm_*`, `axi_jesd204_fsm_*`.
   These are already clean: after 6c they call only driver ops
   (`ad9081_sync_oneshot()`, `axi_adxcvr_enable()`, `axi_jesd204_lane_clk_enable()`,
   `axi_tpl_enable()`). No vendor handle, no `adi_ad9081_*`. **Leave them alone.**
2. **`struct jesd204_dev` tables** — `adxcvr_jdev`, `ad9081_jdev`,
   `axi_jesd204_jdev`, each with a `state_ops[]` array and a `rank`.
3. **`static struct jesd204_topology topology`** at `jesd_fsm.c:427-438` —
   a hardcoded `.devs = {&adxcvr_jdev, &ad9081_jdev, &axi_jesd204_jdev}`,
   `.devs_number = 3`.

Device handles are fetched inline via `DEVICE_DT_GET(DT_NODELABEL(...))` at eight
call sites: `jesd_fsm.c:73` (ad9081), `:277` (tx_adxcvr), `:282` (rx_adxcvr),
`:320` (rx_jesd), `:325` (tx_jesd), `:344-345` (tx/rx_jesd), `:389-390`
(rx/tx_tpl).

### What hybrid option (c) means

DT lists the *participating devices*; **phase order and device order stay in C.**
Rationale in CLAUDE.md: `status = "disabled"` should remove a device from the
walk, and a second link should be expressible, without either being a C edit.

Sketch — add to `boards/zynqmp_apu.overlay`:

```dts
jesd204_link0: jesd204-link-0 {
    compatible = "adi,jesd204-link";
    status = "okay";
    link-id = <0>;
    is-transmit;                 /* absent = receive; matches .is_transmit = false today */
    devices = <&rx_adxcvr &tx_adxcvr>, <&ad9081>, <&rx_jesd &tx_jesd>;
};
```

The grouping matters: **the `devs[]` array is one entry per `jesd204_dev`, not
per Zephyr device.** `adxcvr_jdev` covers both adxcvr instances; `axi_jesd204_jdev`
covers both link cores. A flat `devices = <&a &b &c &d &e>` list does not map onto
`devs[3]`. Either keep the grouping in the DT phandle-array shape (above), or keep
`devs[]` in C and use DT only for the `status` check.

**Recommended, and simpler: the second option.** Keep the `topology` literal in C
exactly as it is, and add a DT-driven readiness gate:

```c
/* Every device in the walk must be ready, or the walk is not attempted. */
#define CHECK_READY(node_id, ...) \
    if (!device_is_ready(DEVICE_DT_GET(node_id))) { return -ENODEV; }
```

That gets you `status = "disabled"` working with no restructuring of `devs[]`,
and the device order stays where CLAUDE.md says it must stay.

### The three tripwires

Each was a real bug or is verified-correct code that reads as wrong:

1. **Device order is load-bearing** (`jesd_fsm.c:416-427`, comment block). An
   earlier version put `ad9081_jdev` first and silently moved the chip's JESD
   PLL check ahead of the GT reset-release. `jesd204_fsm_start()` now *refuses*
   to walk if the array contradicts the per-device ranks — so the guard exists.
   **Do not remove it when the array moves.** FI 1b tests it.
2. **The FSM is best-effort by design.** `jesd204_fsm_start()` counts failures
   per phase and does not abort. FI test 1 verifies this. **Do not make it
   fail-fast.**
3. **`state_op::mode` is never read** by this walker or no-OS's. Declarative
   documentation only. Do not start branching on it.

### Verification for step 7

One commit, structural only. **The boot log must be byte-identical** apart from
the build-version string. If it is not, the change was not structural.

```bash
# after building: diff with timestamps stripped
sed 's/^\[[0-9:.,]*\] //' <your_captured_log> > /tmp/new.txt
sed 's/^\[[0-9:.,]*\] //' boot_log.golden  > /tmp/gold.txt
diff /tmp/gold.txt /tmp/new.txt
```

Two hunks are expected and benign, both already documented: the build string,
and `dac-tpl clk_freq` LSB noise (`0x00027ffe` vs `0x00027fff`,
`boot_log.golden:60-62`). The `1,79c1,2` hunk is the golden's prose header plus
a U-Boot line-1 gluing artefact.

Lines that must match exactly: `chip JTX (framer) 0x007d`, `chip JRX (deframer)
0x000f`, `chip JESD PLL status = 0x1`, `jesd204-tx 0x00000013 [DATA]`,
`jesd204-rx 0x00000003 [DATA]`, all 4 RX lanes in sync, all 5 FSM phases `ok`.

## Step 8 — rewrite fault injection

**Estimated: 1-2 hours after step 7, plus one hardware run.**

`src/fault_injection.c` (529 lines) builds today, so the signatures already
match. The work is:

1. Fix the `flash_ad9081.sh` `BUILD=` line (above) so you can run FI without a
   rebuild.
2. Run it as-is on hardware **before** step 7. That gives you a real pre-step-7
   baseline instead of an AI-written golden. Capture it.
3. Reconcile the count: code says 5 faults; golden says 4; CLAUDE.md says
   "8 checks". Pick the number the code actually produces and update CLAUDE.md
   and the FI log header to match. **You capture the log; do not have it
   written.**
4. Re-run after step 7. The topology-validation tests (FI 1b, four checks at
   `fault_injection.c:301-344`) are the ones step 7 can plausibly break.

`fi_both_ends_in_data()` at `fault_injection.c:54-58` already uses
`DEVICE_DT_GET` on both link cores, so it survives step 7 either way.

## After 7 and 8

Move in-tree, per CLAUDE.md's settled table:

| Block | Destination |
|---|---|
| adxcvr, axi_jesd204 rx/tx, axi_tpl | `drivers/misc/jesd204/` |
| AD9081/AD9082 | `drivers/misc/` + custom API header |
| vendor `adi_api/` (32k lines) | a west module — **ask the `hal_adi` maintainers** |
| `xilinx_transceiver.c` (2169 lines) | Xilinx GT math, not ADI code — placement unresolved |

`hal_adi` exists: `zephyr/west.yml:149-153`, path `modules/hal/adi`. You have ADI
colleagues who maintain Zephyr — ask them where the no-OS MxFE API belongs and
whether ADI wants the AD9081 driver upstream at all before doing the move.

Two things NOT to move: `src/adi_api/` is byte-identical to no-OS (24/24 files,
`cmp`-verified) and `src/xilinx_transceiver.c` differs from no-OS by the include
block only. Keep both that way.

## Small corrections owed

- CLAUDE.md's "dead code to delete (0 call sites)" names `axi_adxcvr_enable()`,
  which has **a live call site** — since the `jesd_fsm.c` split it is
  `src/ad9081_bringup.c:364`, the GT reset-release in the `CLOCKS_ENABLE` phase.
  Only `axi_tpl_adc_pn_mon()` was actually dead, and it is gone
  (`0cd63fb060c`). CLAUDE.md was corrected on 2026-08-10.
- `src/spi_mmio_fixup.c` (98 lines) becomes deletable once the upstream spi_cdns
  fix lands. That fix is committed on branch `spi_cdns_bugfix` as `b11365f32b0`
  and not yet submitted. The file's own header comment (lines 30-35) already
  says "when it lands, delete this file".

## The rules that do not change

1. **Never write or edit a golden log.** They are hardware captures. A log an
   agent wrote is not evidence.
2. **Report diffs, not descriptions.**
3. **Never assert hardware behaviour not observed in the current session.** Cite
   a log file and line, or say "unverified".
4. **One block per commit.** Structural commits produce identical boot logs.
5. **`check_profile.py` must stay at 0 mismatches** — run before and after any
   parameter touch.
