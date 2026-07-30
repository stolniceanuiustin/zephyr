# no-OS ad9081 example vs. this Zephyr port — what the sample actually does

Read-only comparison. No code changed. Profile: `zcu102_ad9081_m8_l4`
(M8/L4/F4/K32/S1/NP16, TX mode 9, RX mode 10, 8B10B subclass 1) — same board and
geometry as this port, confirmed at
`profiles/zcu102_ad9081_m8_l4/app_config.h:50-72` and `:79-95`.

---

## THE question: does no-OS stream continuously from DDR at line rate?

**No. In its default build it does not stream from DDR at all — it never starts a
single DMA transfer in either direction.**

Evidence, in the default configuration (`Makefile:1`, `IIOD ?= n`):

- `src/app.c:456-457` calls `axi_dmac_init()` for both TX and RX. That function
  (`drivers/axi_core/axi_dmac/axi_dmac.c:326-350`) only mallocs a struct, stores
  the base address, and calls `axi_dmac_detect_caps()`. **It touches no transfer
  register and starts nothing.**
- The only calls to `axi_dmac_transfer_start()` in the whole project are inside
  the IIO glue: `drivers/axi_core/iio_axi_dac/iio_axi_dac.c:513` and
  `drivers/axi_core/iio_axi_adc/iio_axi_adc.c:360`. Both are reached only from
  `#ifdef IIO_SUPPORT` code (`app.c:459-558`).
- With `IIOD=n`, `app.c` falls into the `#else` at `:565`, prints `"Bye"`,
  disables the caches and **returns 0** (`:566-577`). The program ends.

So the DDR→DAC streaming path that this Zephyr port has been benchmarked against
**is not what the default no-OS sample exercises.** There is no sustained
4000 MB/s DDR read anywhere in it.

### What the default no-OS sample emits instead: the FPGA DDS

`app.c:258-262` initialises the TX DAC core with `.channels = NULL`. That makes
`axi_dac_data_setup()` take its else branch
(`drivers/axi_core/axi_dac_core/axi_dac_core.c:1227-1238`), which for every
converter:

- sets both DDS tones to **3 MHz** (`:1229-1230`),
- sets phases to 0°/90° alternating by channel index (`:1231-1232`),
- sets scale to `50 * 1000` micro-units = **0.05 of full scale** (`:1233-1234`) —
  `axi_dac_dds_set_scale():723-744` converts micro-units by `*0x4000/1000000`,
- writes **`AXI_DAC_REG_DATA_SELECT = 0`** to every converter (`:1235-1236`).

`0` is `AXI_DAC_DATA_SEL_DDS` — it is the first enumerator at
`axi_dac_core.h:84`. And in the HDL, `dac_data_sel` selects between the DMA
stream and the internal DDS *inside the TPL core*, downstream of the offload:
`library/jesd204/ad_ip_jesd204_tpl_dac/ad_ip_jesd204_tpl_dac_channel.v:144-155`
— `dac_enable <= (dac_data_sel == 4'h2)` and `default: dac_data <=
dac_dds_data_s`.

**Consequence:** in the default no-OS build the DAC output is a 3 MHz, 5%-full-scale
FPGA-generated tone, upconverted by the chip's +2 GHz main NCO
(`app_config.h:66`). The DMA and the data-offload core are **not in the datapath
at all** — `dac_enable` is 0, so the TPL does not even pull beats from the upack
FIFO. That is why `grep -n offload app.c` returns nothing and why it does not
need to: nothing downstream of the offload is consuming its output.

The README supports this reading. It describes the example as *"a simple
application that initializes the AD9081 device"* (`README.rst:292-294`) and
documents no streaming, no bandwidth figure, and no expected DDR throughput
anywhere in its 309 lines. Its documented success criterion is a build log
(`:137-144`).

### And when IIO *is* enabled, TX is a bounded cyclic buffer replay

With `IIOD=y` the transfer is on-demand and finite:

- `iio_axi_dac.c:485-513`: `iio_axi_dac_write_data()` computes
  `bytes = nb_samples * active_channels * 2` from whatever the host pushed, then
  starts a transfer with `.cyclic = CYCLIC` (`:505`).
- The buffer ceiling is `MAX_DAC_BUF_SAMPLES` = 10000000 int16 (`parameters.h:143`,
  commented `//1MB`, actually ~19 MiB), a static array at `app.c:66`.
- RX is explicitly **one-shot**: `iio_axi_adc.c:354` sets `.cyclic = NO` and then
  blocks on `axi_dmac_transfer_wait_completion(..., 500)` (`:364`).
- `iio_axi_dac_prepare_transfer():457-473` only switches a converter to
  `AXI_DAC_DATA_SEL_DMA` if its bit is in the host's channel mask, and puts every
  unmasked converter **back to DDS** (`:467-469`).

So even the IIO path is host-paced, cyclic, finite-buffer replay — a signal
generator, not a sustained DDR pipe.

### Verdict on the framing

**The "403 MB/s against 4000 MB/s demanded" yardstick is not measuring anything
the reference design does.** The reference never asks DDR for 4000 MB/s. When it
wants a continuous tone at the DAC it uses the FPGA DDS, which needs zero DDR
bandwidth, and when it wants arbitrary samples it uses a hardware-cyclic replay
of a bounded buffer — precisely the mode the offload's store-and-replay design
exists to serve.

This does **not** mean 403 MB/s is a good number in the abstract. It means the
403-vs-4000 gap is not evidence of a defect relative to the reference, and the
pre-bypass "9% duty with a recoverable tone" is closer to reference behaviour
than the framing implied.

---

## Delta table — most likely to matter first

| # | Delta | no-OS | Zephyr | Fixable in SW? | Matters? |
|---|---|---|---|---|---|
| 1 | **TX data source** | DDS, `DATA_SELECT=0` on every converter (`axi_dac_core.c:1235-1236`) | DMA, `DATA_SEL=2` on every converter (`axi_tpl.c:194-195`) | **Yes** | **Yes — root of the whole comparison.** Different sample sources, different bandwidth demand. no-OS's DAC never touches DDR. |
| 2 | **Does TX DMA ever run?** | Never in default build (`app.c:565-577` returns after `axi_dmac_init`) | Permanently armed cyclic (`jesd_playback.c:394`, `jesd_playback_rearm():293-302`) | Yes | **Yes.** This port asks the memory system for something the reference never asks for. |
| 3 | **Data-offload cores** | Not written at all (`grep -n offload` over `projects/ad9081/` → no hits, confirmed). Left at reset: TX cyclic, RX one-shot (`data_offload_regmap.v:129`, `up_oneshot <= ~TX_OR_RXN_PATH`; `:130` `up_bypass <= 'd0`) | TX explicitly bypassed (`main.c:170`), RX left at reset | Yes | **Moderate.** Harmless in no-OS because nothing downstream consumes the offload output (delta #1). Once TX is DMA-sourced the mode becomes load-bearing — which is why this port had to confront it and no-OS never did. |
| 4 | **RX capture** | One-shot, host-sized, `.cyclic = NO` + 500 ms wait (`iio_axi_adc.c:354,364`); IIO-only | One-shot, capped 1 MiB by the RX offload buffer (`jesd_capture.c:65-86`) | n/a | **No — at parity.** Both are bounded one-shot captures. no-OS documents no ceiling because the RX offload buffer *is* its ceiling too; nothing in no-OS ever requests more. |
| 5 | **`axi_jesd204_rx_watchdog()`** | Called once at `app.c:448`. Re-checks per-lane status when `link_status == 3` and bounces `LINK_DISABLE` if any lane needs a restart (`axi_jesd204_rx.c:502-526`) | **No counterpart** (`grep -rn watchdog src/` → no hits) | **Yes, cheaply** | **Low-moderate.** A one-shot recovery bounce for a link that came up with a bad lane. Not a throughput factor, but a real omission and the only genuinely missing init step found. |
| 6 | **TX DDS available but unused for output** | Primary output path | Implemented (`axi_tpl.c:332-392`) but only called from diagnostics (`jesd_diag.c:1183`, `:1231`) | Yes | **Yes for the open decision.** This port already has no-OS's actual TX mechanism; it is just not the default. See recommendation. |
| 7 | Bring-up order: DMA/offload vs link-up | JESD FSM (`app.c:446`) → status (`:450-451`) → then `axi_dac_init`/`axi_adc_init` (`:453-454`) → then `axi_dmac_init` (`:456-457`). **All datapath config is after DATA.** | TPL (`main.c:127`) and offload (`:170`) configured **before** `jesd204_bringup()` (`:187`) | Yes | **Low.** Deliberate here and defensible. But note no-OS re-latches the DAC datapath *after* the link is live via `axi_dac_init`'s `SYNC` (`axi_dac_core.c:1186`); this port does the equivalent in `axi_tpl_enable()` (`axi_tpl.c:245`, called from `jesd_fsm.c:192`). Covered. |
| 8 | DDS amplitude / frequency | 3 MHz, 0.05 FS (`axi_dac_core.c:1229,1233`) | 0.75 FS from the DDR table (`jesd_playback.c:76-77`); DDS path uses `DAC_DDS_SCALE_1_0` = full scale (`axi_tpl.c:370-371`) | Yes | **Cosmetic**, but note no-OS backs off 20x. Full-scale DDS risks clipping through interpolation. |
| 9 | ADC core init writes `CHAN_CNTRL` per channel | `axi_adc_init()` writes `AXI_ADC_REG_CHAN_CNTRL(ch)` in a loop before the 100 ms settle (`axi_adc_core.c:654-658`) | `axi_tpl.c:171` writes `ADC_REG_CHAN_CNTRL(c)` | No change needed | **No — at parity.** |
| 10 | `Xil_DCacheEnable()` / cache mgmt | Explicit enable at `app.c:293-295`, flush/invalidate via IIO callbacks (`app.c:513-514`, `:531`) | Zephyr `sys_cache_data_flush_range()` (`jesd_playback.c:171`) | n/a | **Cosmetic** — equivalent, idiomatic per-OS. |

### Nothing else is missing

Every no-OS init step has a Zephyr counterpart:
`app_clock_init` → `hmc7044_setup_clocks()`; `app_jesd_init`
(`app_jesd.c:69-179`: adxcvr + jesd204 rx/tx) → `axi_adxcvr_configure()` +
`axi_jesd204_configure()`; `ad9081_init` → `ad9081_setup_datapath()`;
`jesd204_fsm_start` → `jesd204_bringup()`; `axi_dac_init`/`axi_adc_init` →
`axi_tpl_configure()`/`axi_tpl_enable()`; `axi_dmac_init` → the DT device-ready
check at `main.c:140-151`. The **only** absent item is the RX watchdog (delta #5).

Conversely this port does strictly more: PN monitor (Rung 1), ramp capture
(Rung 2), DDR playback (Rung 4), analog loopback + correlator (Rung 5), and 16
diagnostics. None of that exists in no-OS.

---

## Functional parity statement

**For what the sample does, this Zephyr port is at or beyond parity with no-OS —
and the one respect in which it falls short is a consequence of it attempting
something no-OS never attempts.**

Precisely:

- Everything no-OS does to bring the device up, this port also does, in
  equivalent order, with the single exception of `axi_jesd204_rx_watchdog()`.
- no-OS's *delivered output* — a continuous DAC tone — this port can produce by
  the same mechanism (the TPL DDS, `axi_tpl.c:332`), and currently produces by a
  different and strictly harder mechanism (DDR→DMA→DAC), which no-OS only
  attempts under host control with a bounded cyclic buffer.
- The 403 MB/s TX figure has **no counterpart in no-OS to be short of.** It
  cannot be called a parity gap, because the reference never exercises that path.

What is genuinely unresolved is narrower than the handoff assumed: *this port's
DDR→DMA→DAC path cannot sustain line rate, and store-and-replay is the mode built
to paper over exactly that.* That is a real limitation of an ambition no-OS does
not share.

---

## Bearing on the open TX bypass decision

I was asked to inform this, not decide it. The comparison points one way:

Reverting to store-and-replay (9% duty, **recoverable tone**) is closer to
reference behaviour than bypass (100% duty, **no recoverable tone**). The offload
in store-and-replay is the ADI-intended mode for replaying a finite buffer faster
than DDR can sustain — the handoff's own note (`axi_data_offload.h:26-28`) states
this correctly. no-OS leaves TX at its cyclic reset default and never overrides
it.

A transmitter that emits a correct-but-gated tone is more useful than one that
emits continuous garbage, and "correct tone, gated" is what the reference's own
buffering mode is designed to produce.

Worth noting for whoever decides: if the goal is *a continuous, correct tone at
the DAC*, the reference's answer is neither bypass nor store-and-replay — it is
**the TPL DDS**, which sidesteps DDR bandwidth entirely and is already
implemented here at `axi_tpl.c:332-392`. That is a third option the framing did
not include.

---

## Could not determine from source alone

1. **Whether the ZCU102 bitstream on this board matches `~/HDL` reference
   parameters.** `MAX_BYTES_PER_BURST` is 4096 in the Xilinx BD
   (`ad9081_fmca_ebz_bd.tcl:311,381`) but 2048 in the Intel/Qsys flow
   (`ad9081_fmca_ebz_qsys.tcl:198,214`). Which was built is not readable from
   source; requires a register dump.
2. **Whether `CACHE_COHERENCY` was set at build time**
   (`ad9081_fmca_ebz_bd.tcl:319,388` → HPC0 vs HP1 at `:497,500`). Not
   software-visible, as the handoff already established.
3. **Whether the no-OS default build actually produces a visible 3 MHz tone on
   this specific board.** The code path is unambiguous, but 0.05 FS through
   6x·8x interpolation and a +2 GHz NCO is a claim about analog output I cannot
   verify without running it. The *code* claim (DDS selected, DMA not in path) is
   solid; the *analog* claim is inference.
4. **Whether adding the RX watchdog would change anything observed here.** It is
   a recovery mechanism for a specific fault (a lane needing restart after
   `link_status == 3`). Whether this board ever hits that state is unknown
   without a boot log showing per-lane status.
5. **Whether no-OS's IIO path sustains its cyclic TX without underflow on this
   hardware.** It uses the same offload in the same reset mode, so it would face
   the same fill/drain gating — but nobody appears to have measured duty cycle on
   the no-OS side, and the README makes no claim about it.
