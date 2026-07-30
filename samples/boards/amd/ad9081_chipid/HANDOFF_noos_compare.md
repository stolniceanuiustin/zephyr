# Handoff: compare the no-OS ad9081 sample against this Zephyr port

## Your job

Read the no-OS ad9081 example and tell us **what the sample actually does** — the
sequence of operations it performs on the device, and what it delivers at the
DAC/ADC when it runs. Then compare that against what this Zephyr sample does,
and report the deltas.

You are **not** porting the DMA path, and you are **not** being asked to make
anything faster. The requester was explicit: *"i dont care about the DMA path, i
only care about what the sample does."*

You cannot run the no-OS example (no hardware access for it). It is known-working
and well documented, so treat its source and README as ground truth.

## Why this task exists (read this — it changes how you should work)

The Zephyr port's TX path measures **403 MB/s** against a link demanding
**4000 MB/s** (250 MSPS x 16 B, no gaps). Measurements are solid and reproducible:

| measurement | result |
|---|---|
| TX DMA sustained | 403 MB/s |
| RX DMA sustained | 400 MB/s |
| CPU reading same DDR | 2592 MB/s |

The arithmetic closes on the observed symptom: 1 MiB drains at line rate in
262 us and refills at 403 MB/s in 2602 us, giving a 2864 us cycle and 9.15% duty
— matching a measured ~2.8 ms period and 9% duty. Related: RMS 1372 vs the
4576 expected (`4576 x sqrt(0.09) = 1373`), and tone 0/1000 at all 16 sweep
frequencies.

**The previous agent proposed three explanations for the 403 MB/s and all three
were wrong**, each time from reasoning about Vivado/Verilog internals rather than
against a known-good reference:

1. *Driver burst queueing* — misread `MAX_BYTES_PER_BURST` (an internal AXI burst
   parameter) as a software-visible descriptor limit. A transfer up to 16 MiB is
   a **single** descriptor (`DMA_LENGTH_WIDTH=24`). The "fix" was a no-op; the
   hardware timings came back byte-identical.
2. *CCI/coherency misconfiguration* — the HPC1-vs-HP2 port choice is
   `if {$CACHE_COHERENCY}` **wiring in the block design**, not settable from
   software at all.
3. *Offload cyclic replay* — implemented as test [16]; it failed
   (`FAIL: silent at 0% duty`, RMS 7 = noise floor). The cyclic latch line cited
   (`data_offload_fsm.v:243`) is real, but it was not the blocker: the offload's
   **write** FSM never completes (`wr=0x08` = `WR_STATE_WR`, `rd=0x1` =
   `RD_STATE_IDLE`), and the read side waits on `rd_ml_valid`, which is only
   produced from `wr_response_eot` (`data_offload.v:365`); the read side also
   learns its replay length from the write's reported length (`:386`).

Two things were also **ruled out** and must not be re-litigated:

- `xfer_len=0x3fff` is **correct** — `data_offload_regmap.v:133` resets it to
  all-ones and `:222` reads it back shifted right by 6, so `0xFFFFF >> 6 =
  0x3FFF` means "the whole 1 MiB", in 64-byte units.
- `dma_stop()` did **not** break the write handshake —
  `dest_axi_stream.v:104` is `assign xfer_req = active`, so `xfer_req` drops on
  its own at transfer completion.

**So: do not theorise from HDL. Diff against the working reference.** That is the
entire point of this handoff.

## THE question to answer first

**Does the no-OS sample actually stream continuously from DDR at line rate, or
does it do something bounded?**

This reframes everything. If no-OS's TX demo is a cyclic/one-shot buffer replay,
or an IIO-driven on-demand transfer, then **4000 MB/s of sustained DDR streaming
was never what this reference design does**, the "10x short" framing is the wrong
yardstick, and the Zephyr port's pre-bypass state (9% duty with a *recoverable*
tone) may already be at parity. That would be a good outcome and it is cheap to
establish from the source.

Answer this before anything else, and state the evidence (file:line) for it.

A strong early signal: `grep -n offload ~/no-OS/no-OS/projects/ad9081/src/app.c`
returns **nothing**. The no-OS app appears not to touch the `axi_data_offload`
cores at all, whereas this Zephyr sample explicitly configures the TX one. Work
out what that means — including whether no-OS leaves them at reset values, and
what those reset values are for TX vs RX.

## Where everything is

**no-OS (read-only reference):** `~/no-OS/no-OS/projects/ad9081/`

| file | lines | why you care |
|---|---|---|
| `src/app.c` | 580 | main bring-up sequence; the primary artifact |
| `src/app_jesd.c` | 179 | JESD204 clock/link setup |
| `src/app_clock.c` | 411 | HMC7044 clock tree |
| `src/parameters.h` | 148 | base addresses, IRQ numbers |
| `profiles/zcu102_ad9081_m8_l4/app_config.h` | 100 | **our exact target and geometry** |
| `README.rst` | 309 | documented behaviour — what it claims to deliver |

Use the `zcu102_ad9081_m8_l4` profile. It is the same board and the same
M8/L4/S1/NP16 geometry as this port. Ignore the vcu118/zc706 profiles.

Supporting library code lives under `~/no-OS/no-OS/drivers/` and
`~/no-OS/no-OS/libraries/` (the ADI vendor API is shared with our port).

**Zephyr port (the thing being compared):**
`/home/istolnic/ZephyrOpensource/zephyr/samples/boards/amd/ad9081_chipid/`

| file | role |
|---|---|
| `src/main.c` | bring-up order — the direct counterpart to no-OS `app.c` |
| `src/jesd_playback.c` | TX: DDR sine table -> DMA -> DAC (Rung 4) |
| `src/jesd_capture.c` | RX: ADC -> DMA -> DDR (Rung 2) |
| `src/axi_data_offload.c/.h` | the offload cores; headers carry the findings |
| `src/jesd_diag.c` | 16 diagnostics; header comment summarises the state |
| `src/jesd_loopback.c/.h` | Rung 5 analog loopback + correlator |

**HDL reference (read-only):** `~/HDL/hdl/` — `library/data_offload/`,
`library/axi_dmac/`, `projects/ad9081_fmca_ebz/`. Consult only to confirm a
finding, never as the basis for a theory.

## What to compare, concretely

For each item: what no-OS does (file:line), what Zephyr does (file:line), and
whether the difference could matter.

1. **Bring-up order.** Clocks, chip init, link enable, DMA arm, offload config.
   Specifically: does no-OS start any DMA *before* or *after* the JESD204 link
   reaches DATA? This port configures the offload before link-up deliberately.
2. **The offload cores.** Does no-OS write them at all? If not, what mode do they
   default to for TX vs RX? (Relevant HDL fact already established:
   `up_oneshot <= ~TX_OR_RXN_PATH`, so TX resets to cyclic and RX to one-shot.)
3. **TX transfer shape.** Cyclic or one-shot? What size? Does it re-arm forever,
   or transfer on demand (IIO)? This port leaves the TX DMA armed cyclic
   permanently.
4. **What TX actually emits.** A test tone from a DDR table, a chip-internal test
   tone, the FPGA DDS, or IIO-supplied buffers? How does the README describe the
   expected output?
5. **RX capture shape.** Size, one-shot vs continuous, and any documented
   ceiling. This port caps at 1 MiB because the RX offload is one-shot.
6. **Anything this port omits entirely.** Any device, register block, or
   init step in no-OS with no Zephyr counterpart. This is the highest-value
   category — an entire missing step would explain more than any parameter
   tweak.

## Deliverable

A written comparison, no code changes. Include:

- **The answer to THE question above**, with evidence.
- A table of deltas, most-likely-to-matter first, each with file:line on both
  sides.
- For each delta, whether it is fixable in Zephyr software, needs a bitstream
  change, or is merely cosmetic.
- An explicit statement of whether the Zephyr port is at functional parity with
  no-OS for what the sample does — and if not, precisely what is missing.
- Anything you could not determine from source alone, listed as such.

## Ground rules

- **Do not change code.** This is a read-and-report task. Findings first.
- **Cite file:line for every claim.** The failure mode this task exists to
  correct is confident unsourced reasoning.
- **Distinguish "no-OS does X" from "no-OS does not do X".** An absence (like the
  missing `offload` references) is evidence, but confirm it is a real absence and
  not just a different call path or a name you did not grep for.
- **Say so when the source does not settle a question.** "Cannot determine
  without running it" is a valid and useful answer. Do not fill gaps with
  plausible mechanism — that is exactly what produced the three wrong theories.
- Do not add `Co-Authored-By` or any Claude/Anthropic attribution to any commit.
- Python work needs the project `.venv` activated; never `pip install` globally.

## State of the tree

Branch `drivers/ad9801`, three commits ahead, working tree otherwise clean:

```
09bee0f2a26 samples: ad9081: measure DMA bandwidth and localise the ceiling
0f6ee86ad65 samples: ad9081: time a bounded DMA transfer in each direction
87ce75fe118 samples: ad9081: bypass only the TX data-offload core
```

Test [16] (cyclic replay) is implemented and committed but **fails** — leave it
in place; its FSM dump is diagnostic.

Build: `cd /home/istolnic/ZephyrOpensource && west build -p always -b zynqmp_apu
zephyr/samples/boards/amd/ad9081_chipid/`. Flash `build/zephyr/zephyr.bin`, boot
with `go` (not `bootelf`), console on COM13. Hardware runs are operator-driven —
you will need to ask for a boot log rather than running it yourself.

## Open decision (context, not your call)

TX is currently in offload **bypass**: 100% duty but **no recoverable tone**.
Pre-bypass it was 9% duty **with** a recoverable tone. For a transmitter the
latter is arguably better, and reverting is on the table pending this comparison.
Your findings should inform that choice; do not make it unilaterally.
