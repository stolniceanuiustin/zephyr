# Handout: how to review the JESD204 FSM rewrite and the in-tree move

For an agent teaching the user to audit this work. Branch `drivers/ad9801`,
5 commits, `be131423f81..cc0a337288e`. 29 files, +342/-92.

**Your job is to teach the reading, not to re-do the work.** The user has said
plainly they cannot audit thousands of lines of driver C unaided. Everything
below is ordered so each step is checkable in a few minutes and the check is
mechanical, not a matter of trusting a description.

## The five commits, and which are risky

| commit | what | risk |
|---|---|---|
| `be131423f81` | TPL out of an FSM phase, into `main.c` | **behaviour** — read it |
| `183e79dd5f3` | FSM to `subsys/jesd204/` | superseded, see below |
| `f6dc0b02248` | AXI drivers to `drivers/misc/jesd204/` | structural |
| `962196048fc` | reverts `183e79dd5f3` | structural |
| `cc0a337288e` | headers under `include/zephyr/drivers/misc/` | structural |

Two of those five cancel out. `183e79dd5f3` moved the FSM framework into
`subsys/jesd204/` as a new top-level Zephyr subsystem; the user pushed back
("i dont think you can make a subsys just like that") and `962196048fc`
reverted it. **They are both in history on purpose** — the rule in CLAUDE.md is
one block per commit, and rewriting history to hide a reverted decision would
make the remaining commits unreadable. Net effect of the pair is zero. Confirm
that rather than reading either:

```bash
git diff 183e79dd5f3~1 962196048fc -- subsys/ include/zephyr/jesd204/
# expect: empty
```

If that is empty, you only have to review three commits.

## Read in this order

### 1. `be131423f81` — the one behaviour change (10 min)

The only commit that changes what the hardware does. Small on purpose.

```bash
git show be131423f81 --stat
git show be131423f81 -- samples/boards/amd/ad9081_chipid/src/main.c
```

What to check: `axi_tpl_enable()` used to run inside the FSM as an
`OPT_POST_RUNNING_STAGE` phase callback. It now runs in `main.c` after
`jesd204_bringup()` returns. The claim is that this matches no-OS:

```bash
sed -n '450,460p' ../../../../../ADI_Temp/AD9081/no-OS-reference/projects/ad9081/src/app.c
```

no-OS calls `axi_dac_init()`/`axi_adc_init()` there, after
`jesd204_fsm_start()`. **Verify that line range says what the commit message
claims it says.** If it does not, the justification for the change is wrong even
if the code works.

Consequence to notice: the topology went from 5+1 devices to 4+1, and the TPL
nodes left the `JESD204_PARTICIPANTS` readiness gate. A disabled TPL core is now
a datapath problem, not something that blocks the link. That is intended; make
sure the user sees it, because it is a real change in failure behaviour.

### 2. The FSM rewrite — `e67450c19b0`, the commit *before* this range (1 h)

Not in the 5 commits above, but it is the substance. 1058 insertions. Do not
read it as a diff — read the three files as they now stand, in this order:

1. **`src/jesd204_fsm.h`** (325 lines, mostly comments). The types. Read the
   file header first: it lists what was deliberately *not* ported from no-OS and
   why. Then `struct jesd204_topology_dev` — that is the only structure a board
   port fills in.

2. **`src/jesd204_fsm.c`** (543 lines). Two halves.
   `jesd204_topology_init()` validates the client's array;
   `jesd204_fsm_start()`/`_stop()` walk it. The walk order is stated as a
   4-line pseudocode block at the top of the file. **Check the code against
   that block, and the block against no-OS `jesd204-fsm.c:21-53`.** This is the
   single highest-value review in the whole port: if the visit order is wrong,
   the link still comes up on a good board and fails intermittently on a
   marginal one, and no boot log catches it.

3. **`src/jesd_fsm.c`** (650 lines). This board's tables. Jump straight to
   `board_topology_devs[]` (~line 528). Six rows, one marked
   `.is_top_device = true`. That array is what a customer edits and in most
   cases the only thing.

The claim worth testing hardest: *"a no-OS topology ports by substituting the
device handle and nothing else."* Test it by putting the two arrays side by
side:

```bash
sed -n '387,441p' ../../../../../ADI_Temp/AD9081/no-OS-reference/projects/ad9081/src/app.c
grep -n -A 45 "board_topology_devs\[\] = {" src/jesd_fsm.c
```

If a field exists in one and not the other, that is a divergence and it should
be commented in `jesd_fsm.c`. There are three known ones (adxcvr is a topology
device here, HMC7044 is not, and the TPL now runs outside) — all documented in
the file. **A fourth, undocumented divergence is a finding.**

### 3. `rank` is gone — what replaced it (20 min)

The previous design ordered devices by a `rank` field that no-OS does not have.
It existed to stop one specific bug: the converter being visited before the GT
came out of reset, which moved the chip's JESD PLL check too early.

`rank` is deleted. Two things replaced it:

- **`is_top_device`** — the converter is visited last in every forward phase,
  first in reverse. Same guarantee, expressed the way no-OS expresses it.
- **9 validation checks** in `jesd204_topology_init()` — no top device, two top
  devices, a link the top device does not declare, and so on. no-OS has none of
  these and walks a malformed topology silently.

Prove the ordering holds on hardware. In `boot_log.golden`:

```bash
grep -n "lane clock up\|JESD PLL status" boot_log.golden
```

Expect: `tx_adxcvr lane clock up` → `chip JESD PLL 0x1` → `rx_adxcvr lane clock
up` → `chip JESD PLL 0x1`. GT before the converter's PLL check, **per link**.
That interleave is the evidence `is_top_device` works. If the two GT lines were
adjacent, the ordering would have regressed.

### 4. The in-tree move — `f6dc0b02248` + `cc0a337288e` (20 min)

Structural. Do not read the moved code; verify it did not change.

```bash
git show f6dc0b02248 --stat
git log --follow --oneline -3 -- drivers/misc/jesd204/xilinx_transceiver.c
```

Three checks that matter more than reading diffs:

1. **`xilinx_transceiver.c` is still byte-identical to no-OS bar its include
   block.** It is 2169 lines of vendor GT divider math and must stay untouched:
   ```bash
   diff <(sed '/^#include/d' drivers/misc/jesd204/xilinx_transceiver.c) \
        <(sed '/^#include/d' ../../../../../ADI_Temp/AD9081/no-OS-reference/drivers/axi_core/jesd204/xilinx_transceiver.c)
   # expect: empty
   ```
   Adjust the relative paths for where you run it.

2. **`src/adi_api/` is still byte-identical to no-OS, 24/24 files.** It did not
   move and must not have changed.

3. **The drivers auto-enable from devicetree, not from the sample's prj.conf:**
   ```bash
   grep JESD204_AXI build/zephyr/.config
   ```
   Expect `_ADXCVR`, `_LINK`, `_TPL` all `=y`, each because its compatible is
   present. Watch for the `-1.0` suffix in the compatibles — the first attempt at
   this Kconfig used the wrong `DT_HAS_*` names and silently compiled nothing
   in, which showed up as undefined references at link time, not as a config
   error.

## What "verified" means here, and what it does not

Verified on hardware, from a log the user captured:

- the link reaches DATA on both ends, all phases `ok`
- `chip JTX 0x007d`, `chip JRX 0x000f`, `jesd204-tx 0x00000013 [DATA]`,
  `jesd204-rx 0x00000003 [DATA]`, 4 lanes in sync
- `check_profile.py`: 116 compared, 0 mismatches

**Not verified:**

- **The four structural commits have never been run on hardware.** They build
  and link; no boot log has been captured since `be131423f81`. The expected diff
  against `boot_log.golden` is the build-version string alone. Until someone
  captures that log, the moves are build-verified only. **This is the top open
  item.**
- **The fault-injection suite has never run on hardware.** `build_fi/` links.
  FI 1 was rewritten for the two-link topology and now expects **4** failures,
  not 3 (LINK_INIT per_device ×1, LINK_SETUP per_device ×1, LINK_SETUP per_link
  ×2). A count of 6 would mean `per_device` fired per link; 3 would mean the
  second link was skipped. Nobody has seen it produce 4.
- **`flash_ad9081.sh:43` hardcodes `BUILD=build/zephyr`**, so `build_fi/` is
  invisible to it. One-line fix: `BUILD=${BUILD:-build/zephyr}`.
- **`post_state_sysref` / `sysref_cb` has never executed.** No device on this
  board registers one; only the no-callback path has ever run.

## Rules to hold the user to

From CLAUDE.md, and they were followed here:

1. **Never write or edit a golden log.** The user captures it from hardware. If
   you are asked to write one, refuse and explain why.
2. **Report diffs, not descriptions.** "It looks correct" is not a result.
3. **Never assert hardware behaviour not observed this session.** Cite a log
   file and line, or say "unverified".
4. **One block per commit.** Structural commits produce identical boot logs.
5. **`check_profile.py` must stay at 0.** Run it before and after any parameter
   touch.

<!-- SPDX-License-Identifier: Apache-2.0 -->
