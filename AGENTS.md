# AGENTS.md — WHAM-XREX-PFMG474 Firmware

Orientation file for any AI agent or LLM working in this repository.
Read this before touching code.

## What this project is

Firmware for a **closed-loop PID controller for the Transrex ISR-2126
magnet power supplies** (four independent units, see
`docs/Transrex/Transrex_Controls_Upgrade (1).pdf`) -- **on the exact
same PCB, same STM32G474QET6, same pin mapping** as the switching-supply
project this repo was reseeded from (see "Provenance" below), not a new
board. The application is different, not the hardware: instead of one
phase-locked table-driven PWM output (WHAM-PFMG474-V4's job), this
project drives up to 4 *independent* PFM demand channels (`DEMAND`, one
per Transrex, no relationship between their frequencies) against 4
independent PFM feedback channels (`UNIT_CURRENT`, V-to-F encoded
current sense), closing a live PID loop between them entirely on-MCU --
see `docs/Transrex/hrtim_divergent_period_timing.pdf`'s "Possibility 3"
for the core mechanic (per-channel HRTIM shadow-register update on that
channel's own roll-over, `ResetTrigger=NONE`/`UpdateTrigger=NONE`/
`ResetUpdate=ENABLED`, decoupled from any shared Master-anchored
interleave) and the architecture discussion in this project's own early
history (below, and see the design conversation referenced in the first
`docs/changelog.txt` entry) for how that combines with a *fixed-rate*
Master-tick heartbeat driving the PID compute step itself (a real,
deliberate design point -- fixed control-loop sample time, decoupled
from each channel's own instantaneous carrier frequency; DO NOT
"simplify" this back to a self-clocked per-channel update without
re-reading that discussion first). Implemented 2026-09-09 (same day as
the fork) and CONFIRMED converging on real hardware -- see
"Current functionality" below and docs/changelog.txt's own entry for
the full writeup, including a real anti-windup bug found and fixed via
real-hardware bench testing.

**Provenance**: this repo was reseeded (2026-09-09) from
[`WHAM-PFMG474-V4`](https://github.com/everettpenne/WHAM-PFMG474-V4) at
its commit `de83324`, as a genuinely **independent git repository, not a
GitHub fork** -- no shared git history, by deliberate choice, matching
this exact project's own long-standing convention for sibling projects
(see below): the two are expected to diverge substantially (a different
HRTIM usage model entirely, no shared table engine), so there's nothing
for GitHub's fork/upstream-sync machinery to usefully do once that
happens. A plain second git remote is enough for the rare case a fix
genuinely wants cherry-picking between the two (`git remote add wham-v4
git@github.com:everettpenne/WHAM-PFMG474-V4.git`). **WHAM-PFMG474-V4
itself was NOT modified to create this repo** -- it continues
unchanged, as its own separate, working switching-supply project.
Everything in this file and this codebase up through the fork point is
inherited context, copied as-is; treat historical entries in
`docs/changelog.txt` and inline comments describing past debugging
(the HRTIM SET/RESET collision, the NVIC priority-inversion fix, etc.)
as genuine history of code that's still sitting in this repo, not as
something that happened to raise "PFM controller" in general terms.

**Sibling project**: `../../PFM-STM32G474` (referred to below as "V3")
and `../../PFM-STM32G474-V4` ("V4") are separate git repositories for
the earlier hardware revision(s) that `WHAM-PFMG474-V4` (this repo's own
own base) was itself built from -- further along and much larger in
scope (PWM generation, closed-loop feedback, fault handling, a GUI).
None of these three share git history with this repo or each other.
Code ported over is called out explicitly below and in each file's own
comments -- check there before assuming a mechanism is unique to this
project or before "fixing" something that was a deliberate port.

## Current functionality (as of 2026-09-09)

### Closed-loop PID control -- this project's actual point

Implemented 2026-09-09, CONFIRMED converging on real hardware the same
day. `Core/Inc/pid.h`/`Core/Src/pid.c`: `HRTIM_NUM_CHANNELS` (4)
independent channels, each `{setpoint, Kp/Ki/Kd, integral, derivative
history}`, no shared table, no relationship to any other channel.
`PID_Update()` runs once per Master heartbeat (`PID_LOOP_RATE_HZ`,
`ctrlr_config.h`, currently 1 kHz -- NOT yet tuned against real
Transrex/magnet dynamics), reading each channel's latest measured
period (`pfm_input.c`'s new continuous/free-running capture --
`PfmInput_StartContinuous()`/`GetLatestPeriod()`, alongside the
original bounded bench-capture path, unchanged), converting to Hz,
running that channel's PID with a fixed Delta-t, clamping to
`[PID_OUTPUT_MIN_HZ, PID_OUTPUT_MAX_HZ]` (3000-150000 Hz -- the floor
is a real HRTIM 16-bit-`PER` hardware limit, not a tuning choice), and
writing the result via `hrtim.c`'s `HRTIM1_SetChannelPeriod()` (each
channel's own shadow registers, promoted at that channel's own
roll-over -- Possibility 3, see "What this project is" above).
Wire interface: `PID:START`/`STOP`/`SETPOINT <ch> <hz>`/
`GAINS <ch> <kp> <ki> <kd>`/`STATus? <ch>` (`commands.c`, ERR 11/12).

CONFIRMED on real hardware (channel 1, the one channel with a real
loopback wired on this bench -- see docs/changelog.txt for the full
numbers and the real anti-windup bug this testing found and fixed):
measured/output climbed smoothly from ~4.1 kHz to within ~7% of a
20 kHz setpoint over 3 seconds, tracking each other almost exactly
throughout -- the whole chain (capture -> Hz conversion -> PID ->
per-channel HRTIM write -> real output -> real feedback) genuinely
closes. Channels 2-4 build and report status correctly but are
functionally UNTESTED -- nothing physically wired to their feedback
pins yet. Gains used were arbitrary bench values against a wire
loopback, not a real plant.

### Inherited from WHAM-PFMG474-V4 (still present, no longer this
### project's real output path)

Configurable-channel-count PWM generation (HRTIM1, phase-locked to the
Master timer) plus the serial command layer, `TABLE:*`/`FIRE` --
WHAM-PFMG474-V4's own switching-supply mechanism, carried over at the
fork point. Still links and does something sane (a thin compatibility
shim, `HRTIM1_ApplyPfmStep()` in `hrtim.c`, now calls
`HRTIM1_SetChannelPeriod()` per channel with no phase/interleave), but
is NOT how this project actually drives hardware anymore -- see
`pid.c` above for that. Kept as a bench-testing fallback, not because
this project needs a table-playback mode. There is still **no state
machine** -- no ARM precondition, no interlock, no fault gating; `FIRE`
takes effect immediately whenever sent, per the original project
decision (see `docs/command_reference.md`'s `FIRE` entry).

- **Serial command architecture** (`uart.c`, `cmd_parser.c`) -- ported
  from the sibling project, ported *architecture-only* (no command
  handlers came with it). Interrupt-driven single-byte USART2 RX with
  line-buffer accumulation (`uart.c`), polled from the main loop.
  Dispatch is a **flat, SCPI-style hierarchical command table**
  (`cmd_parser.c`) -- see `docs/command_reference.md` for the full
  mnemonic-matching rules (short/long form, `:`-separated levels, `?`
  query suffix). This is a genuine departure from the sibling project's
  case-sensitive exact-match dispatch; see that file's header comment
  for the reasoning.
- **`*IDN?`** (`commands.c`, constants in `ctrlr_config.h`) -- reports
  board name, hardware revision, and firmware version. `HW_BOARD_REV`
  in `ctrlr_config.h` is a placeholder; keep it in sync with the
  actual PCB silkscreen. (`version.h` no longer exists -- condensed
  into `ctrlr_config.h`, 2026-09-08; see `docs/changelog.txt`.)
- **`BOOT`** / serial-bootloader entry (`boot_jump.c`) -- software jump
  into the STM32 ROM bootloader over USART2, no physical BOOT0/NRST
  access needed. Ported from the sibling project's `boot_jump.c`, with
  two additions made here:
  1. **Removable by design**: `BOOT_JUMP_FEATURE_ENABLED` in
     `boot_jump.h` is the single point of control -- see that header's
     doc comment for exactly what disabling it does and does not remove.
  2. **A real bugfix, found and confirmed on hardware in this project**
     (not present in the sibling project's copy as of this writing --
     see "Known gaps" below): the ROM bootloader's `Go` command (what
     `stm32flash -g` issues after writing a new image) does **not**
     perform a real chip reset, so `SCB->VTOR` and `SYSCFG->MEMRMP` can
     be left pointing at the bootloader's own vector table. Without a
     fix, the newly-flashed app runs but is deaf to every
     interrupt-driven peripheral (including the USART2 RX interrupt
     everything here depends on) until something -- a manual reset --
     clears it. `BootJump_CheckAndEnter()` now restores both
     unconditionally on the normal-boot path. See that function's doc
     comment for the full mechanism, and `docs/changelog.txt` for the
     debugging history (this was chased down empirically, on real
     hardware, over several flash/reset cycles).
- **Host-side tooling** (`python/`):
  - `wham_build.py` -- **the canonical way to build this project**,
    added 2026-09-11 (see "Build & verify" above for the full reasoning):
    `gen_git_version.py` then `make` then the `.bin` regeneration, as
    one command instead of three separate steps to forget.
  - `gen_git_version.py` -- regenerates `Core/Inc/git_version.h` from
    the current git state (commit hash + dirty-tree flag), so `*IDN?`
    reports exactly which commit a running firmware was built from.
    Gitignored output; run automatically by `wham_build.py`, or
    directly if driving `make` yourself for some other reason.
  - `wham_serial_flash.py` -- one-command serial reflash (`BOOT` +
    `stm32flash`). Ported from the sibling project's
    `pfm_serial_flash.py`, corrected for this project's actual baud and
    build artifact name. **Verified working end-to-end on real hardware**,
    including the VTOR/MEMRMP fix above (confirmed: flash a new version,
    query `*IDN?` immediately, no manual reset, get the new version back
    -- done twice, back to back). Flashes whatever `Debug/*.bin` already
    exists -- doesn't rebuild; run `wham_build.py` first.
  - `wham_console.py` -- **the maintained interactive operator
    console**, added 2026-09-10, the intended day-to-day front end for
    a human working with a real board (raw SCPI passthrough, a guided
    shot-profile wizard, live status, automatic diagnostic plotting --
    see its own header comment for the full command list). Extend this
    as new firmware commands are added, rather than leaving operators
    to fall back on raw SCPI for everything new -- see its own "Adding
    a new console command" section before adding one (console
    meta-command names must never collide with a real SCPI mnemonic's
    leading token -- that section explains exactly why and how).
    `diag` (added 2026-09-14) is a one-action debugging snapshot
    (state/fault/GDS/QSPI/PFMIN + every channel's config and live
    status); `report [ch|all]` (same date, extended 2026-09-14) is the
    shot-PERFORMANCE tool -- does everything `plot` does plus a written
    .md summary under `shots/` with THREE separate error comparisons:
    output-vs-commanded (did the controller drive what the profile
    said, independent of any feedback), feedback-vs-commanded (closed-
    loop convergence), and feedback-vs-output (today, a bench-wiring
    self-check only -- feedback is currently looped back from this
    controller's own output, not an independent Transrex supply) -- see
    `compute_log_stats()`'s own doc comment for the full reasoning and
    `format_channel_report_md()` for what's written. `timing
    [<rampS> <flatS>]` and `demand <ch> [<amps>]` (added 2026-09-14)
    wrap PID:PROFile:TIMing/CURRent -- the only two PID:* settings that
    had no wrapper before this, which a real LLM-console session
    exploited by guessing wrong syntax twice in a row (see
    docs/changelog.txt's matching entry).
  - `wham_llm_console.py` -- added 2026-09-14, SIDE PROJECT (lower
    priority than the main firmware work, not yet exercised against
    real hardware or a real LLM server as of its first commit --
    SINCE tested end-to-end against both, repeatedly, same day: health
    checks and `diag` work reliably and accurately; the safety gate
    itself has held correctly every time it was actually tested
    (confirmation asked before every dangerous action). A REAL
    unconfirmed shot DID fire during one verification pass anyway --
    not a gate bug, a testing-methodology mistake (piped blind "y"
    answers into a non-interactive test run, one of which landed on the
    real PID:PROFile:STARt confirm instead of a human reading it) --
    see docs/changelog.txt's matching entry for the full, transparent
    account and the resulting rule: interactive-only, single-stepped,
    real state checks between dangerous actions, for any future testing
    of this file. `report`'s natural-language tool-selection has a
    demonstrated, not-fully-fixed reliability gap (see docs/
    changelog.txt's 2026-09-14 entries for the full record, including a
    ~3x real-world speedup via Qwen3's "/no_think" convention + a
    longer Ollama keep-alive -- both root-caused on real hardware, not
    guessed -- and full raw-LLM-call logging, elapsed time/token usage/
    reasoning text, added the same day specifically because the
    existing summarized log couldn't show WHY a real session's turn
    failed).
    An alternate front end to `wham_console.py` where the operator
    talks in plain English to a small local LLM (Ollama/LM Studio),
    which decides which `wham_console.py` command lines to run; drives
    a real `WhamConsole` instance unmodified, so it adds no new wire
    behavior of its own -- see its own header comment for the full
    design (system prompt, JSON action contract, safety gate, speed).
    Its own safety gate and `wham_console.py`'s `_is_dangerous()` share
    one source of truth as of the 2026-09-14 fix below -- keep it that
    way; don't reintroduce a separate, driftable dangerous-command list
    in this file. Ollama's own default config (4096 context, ~5min
    keep-alive) is too tight for this script -- see its header comment
    for the actual working fix (launchctl setenv, both a context-length
    AND a keep-alive parameter silently do NOT take effect when passed
    per-request through Ollama's OpenAI-compatible endpoint, confirmed
    on this exact setup -- don't assume a per-request override works
    without checking `ollama ps` before trusting it).
  - `scpi.py` -- a minimal interactive serial terminal (not written by
    an agent; predates this documentation pass, and predates
    `wham_console.py` above). Superseded by `wham_console.py` for
    routine use; kept as a bare-bones fallback.
  - `pfm_table_upload.py` -- builds one of 5 fixed PFM shot profiles
    (see below) and uploads it via `TABLE:*`.
  - `dsl_viewer.py` -- interactive viewer for saved DSLogic/DSView
    `.dsl` capture files (not this project's firmware/protocol tooling
    -- a bench-instrumentation aid, generalized from several one-off
    hand-decode scripts used during the cold-start phase-lock
    investigation, 2026-09-04). Stacked digital-trace plot
    (matplotlib), scroll-to-zoom on the time axis, and a two-cursor
    click-to-measure readout (time delta + implied frequency) --
    requires `pip install matplotlib numpy`. See its own header
    comment for the `.dsl` format details (reverse-engineered from real
    captures, not from any DSView spec) and full usage.
  - `dslogic_shot_capture.py` -- added 2026-09-13, LIVE (not saved-file)
    DSLogic cross-check for `shot` (see `wham_console.py` above): if a
    DSLogic is connected, arms a free-running capture right before a
    shot fires (Phase U/V/W/X <-> DSLogic Ch0/1/2/3, the user's own
    fixed wiring -- corrected same-day from an initial Ch2/Ch3 mixup,
    see docs/changelog.txt) and saves a frequency-vs-firmware-ground-
    truth comparison PNG (one panel per WHAM channel, including
    disabled/idle ones) to `shots/` once the shot completes. Depends on
    `~/dslogic-tool` (kept outside this repo -- vendors ~10MB of
    DreamSourceLab GPL source unrelated to WHAM firmware; see that
    project's own README). Every public function degrades to a clean
    no-op if no DSLogic is connected, `~/dslogic-tool` isn't present, or
    numpy/matplotlib
    aren't installed -- a shot behaves identically either way. Verified
    end-to-end on real hardware through the actual `shot` command (not
    just called directly) -- see `docs/changelog.txt`'s 2026-09-13 entry.
  - `pfm_input_plot.py` -- pulls whatever `PFMIN:DATA?` capture is
    currently sitting in the controller and plots it per channel
    (period + frequency, outlier-robust y-scaling). Read-only: doesn't
    arm a capture or `FIRE` -- run those yourself first.
  - `memory_report.py` -- flash/RAM footprint snapshot and lifetime
    history, added 2026-09-09. **Standing instruction: run this after
    any commit that changes flash/RAM usage** (same "update proactively,
    not just when asked" convention as `docs/changelog.txt`) --
    `python3 memory_report.py history` (appends one row to
    `docs/memory_history.csv` for the current `HEAD`) then
    `python3 memory_report.py report` (regenerates
    `docs/memory_report.html` from that CSV), and commit both files
    alongside the code change. `python3 memory_report.py history
    --backfill` rebuilds the whole CSV from every past commit that has
    a `Debug/WHAM-XREX-PFMG474.elf` checked in -- only needed once, or if
    the CSV is ever lost/corrupted. Per-module attribution is read off
    the **linked ELF's own retained symbol table**
    (`arm-none-eabi-nm --print-size -l`), not summed `.o` files -- with
    `-ffunction-sections`/`-fdata-sections`/`--gc-sections` (this
    project's Makefile), a raw per-`.o` `size` counts code the linker
    later discards as unreachable, wildly overcounting HAL driver files
    where only a handful of functions per file actually ship. See the
    script's own header comment for the full method (including the
    Default_Handler weak-alias dedup, and why `size`'s own totals --
    not the module sum -- are always the authoritative numbers, with
    any gap rolled into an explicit "Padding / unattributed" bucket).
- **PWM/HRTIM engine** (`hrtim.c`, `pfm.c`) -- originally ported from
  the sibling project's hardcoded 3-channel design and wired into
  `main.c` (see `docs/changelog.txt`, 2026-09-04 entries, for the full
  CubeMX-vs-hand-code saga this took to get right -- read that before
  touching `hrtim.c`'s `HAL_HRTIM_MspInit()` or the `.ioc`), then
  generalized (2026-09-08) to a compile-time channel count,
  `HRTIM_NUM_CHANNELS` in `ctrlr_config.h`, 1-5. Channels 0..N-1 are
  phase-locked to the Master timer (channel 0 via `MASTER_PER`,
  channels 1..N-1 via `MASTER_CMP1..MASTER_CMP(N-1)`, evenly spaced --
  only 4 Master compare units exist, capping this scheme at 5 synced
  channels, not 6); channels N..5 (up through Timer F) stay
  pin/dead-time-reserved but unlocked, exactly as D/E/F behaved before
  this generalization, and are never started by `HRTIM1_PWM_Start()`.
  See `ctrlr_config.h`'s own comment for why a 6th phase-locked channel
  isn't a simple extension (real HAL support exists for it --
  cross-timer `HRTIM_TIMRESETTRIGGER_OTHERx_CMPy` reset triggers -- but
  needs new phase math with no existing pattern here, deliberately left
  as a follow-up).
- **`TABLE:BEGIN`/`STEP`/`END`/`?`, `CONFig:CHANnels?`** (`commands.c`,
  primitives in `pfm.c`) -- uploads a complete PFM table (`PFM_Step_t`
  entries, each now `per` + one compare value per channel) built
  entirely off-controller. Firmware does zero construction/validation
  beyond "does it fit" -- see `pfm.h`'s "ADDED" header note.
  `CONFig:CHANnels?` (`OK <N>`) reports `HRTIM_NUM_CHANNELS` so host
  tooling can confirm a board's actual channel count rather than
  assuming it. `python/pfm_table_upload.py` is the reference builder: 5
  fixed profiles today (constant 50% duty; 25%->75% ramp; 90%->10%
  ramp; 10%->90% ramp; 90%->25% ramp), selected by editing a constant
  before running -- it queries `CONFig:CHANnels?` itself at startup and
  refuses to upload on a mismatch against its own `HRTIM_NUM_CHANNELS`
  constant.
  `FIRE` plays an uploaded table back -- see "Current functionality"
  above.
- **`FAULT?`/`FAULT:CLEAR`, two independent fault sources unified at
  the command layer** (`hrtim.c`/`hrtim.h`, `gate_driver.c`/
  `gate_driver.h`, gated into `pfm.c`/`commands.c`) -- `FAULT?`/
  `FAULT:CLEAR`/`FIRE`'s `ERR 6` check ALWAYS mean "either source,"
  via `commands.c`'s `AnyFaultLatched()`. The two sources themselves
  stay structurally separate, two different mechanisms:
  1. **PC10/HRTIM1_FLT6, native HRTIM hardware fault input** --
     PC10 is wired directly to HRTIM1's own FLT6 fault input on this
     board (`docs/pin_mapping_v4.csv`), active-low. Configured as a
     genuine silicon-level protection
     (`HAL_HRTIM_FaultConfig()`/`FaultModeCtl()`,
     `pOutputCfg.FaultLevel = HRTIM_OUTPUTFAULTLEVEL_INACTIVE`) --
     fault-enabled outputs are forced safe autonomously, in hardware,
     the instant the pin trips, no CPU/interrupt/polling involved.
     Deliberately **not** the sibling PFM-STM32G474 project's
     software-polled `HRTIM1_EmergencyStop()` approach. **Verified on
     hardware so far**: idle reads as healthy (PC10 idle-high isn't a
     spurious trip) and normal `FIRE` still works. **Not yet
     verified**: an actual PC10-low trip on scope/DSLogic, or its
     specific contribution to the recovery sequence -- needs the
     bench, see `docs/changelog.txt`.
  2. **GateDriverStatus_01..12 (PE0-PE11), software/EXTI-driven fault
     interrupt** -- added 2026-09-08 while investigating why a fault
     wasn't being registered (these pins weren't even configured as
     GPIO inputs before that day). `MX_GPIO_Init()` configures
     PE0-PE11 `GPIO_MODE_IT_RISING_FALLING`/`GPIO_NOPULL`, backing 7
     EXTI IRQ handlers (`stm32g4xx_it.c`) that all call
     `GateDriver_CheckFault()` (`gate_driver.c`): re-reads all 12 pins
     bitwise and evaluates them against the compile-time
     `GDS_FAULT_POLARITY` (`ctrlr_config.h`, `NORMALLY_HIGH` or
     `NORMALLY_LOW`) -- if any pin is in its fault state, immediately
     `PFM_ForceStop()`s HRTIM output and latches. One boot-time
     explicit call (`main.c`) also runs this check once, closing a
     real gap: EXTI is edge-triggered, so a pin already in its fault
     state before the interrupt is even configured produces no edge
     and would otherwise go undetected until something happened to
     toggle it. `GateDriver_FaultClear()` re-validates immediately
     after clearing (calls `GateDriver_CheckFault()` again) for the
     same reason -- unlike PC10's hardware latch, which re-trips on
     its own if still physically tripped, a still-bad GateDriverStatus
     pin needs this explicit recheck or a clear would silently "fix"
     a fault that's still there. **Verified on real hardware,
     end-to-end**: `GDS_FAULT_POLARITY = GDS_NORMALLY_LOW` was set
     after confirming this board's actual gate driver wiring; the
     board's GateDriverStatus_03 (PE2) was already HIGH (a fault under
     that polarity) at boot -- `FAULT?` correctly read `OK 1`
     immediately after flashing, `FIRE` was correctly rejected with
     `ERR 6`, and `FAULT:CLEAR` correctly re-latched (`FAULT?` back to
     `OK 1`) rather than clearing a fault that was still physically
     present.

  Both sources: latched-until-cleared by design (matching the sibling
  project's Lockout/`CLEARFAULTS` convention); `FAULT:CLEAR` never
  itself reconnects outputs -- that's the next explicit `FIRE`'s job.
  `docs/command_reference.md` documents `GDS?` and `FAULT?`/
  `FAULT:CLEAR`/`ERR 6` -- the GateDriverStatus source's real-hardware
  verification (immediately above) was thorough enough on its own that
  the writeup no longer waits on the PC10-low bench trip, which stays
  flagged as the one still-open item there instead.
- **`GDS?`** (`gate_driver.c`/`gate_driver.h`, `commands.c`) -- raw
  HIGH/LOW snapshot of the same 12 GateDriverStatus_01..12 pins.
  Direct `GPIOE->IDR` read, no debounce, no polarity interpretation,
  no fault-latching of its own -- unaffected by and independent of the
  fault interrupt described above (same pins, separate mechanism). One
  `OK 01=HIGH 02=LOW ...` line, PE0 first. **Verified on hardware**:
  stable, repeatable across back-to-back queries.
- **`QSPI:ID?`** (`qspi_test.c`/`qspi_test.h`, `commands.c`) -- QUADSPI
  connectivity test against a W25Q128JVS NOR flash on
  PE12-PE15/PB10-PB11 (all AF10, `docs/pin_mapping_v4.csv`), added
  2026-09-08. Deliberately narrow: the chip's standard JEDEC Read ID
  instruction (`0x9F`, plain 1-line mode) and nothing else -- no
  program/erase, no memory-mapped access. **Easily removable by
  design**, following `boot_jump.h`'s exact pattern:
  `QSPI_TEST_FEATURE_ENABLED` (`qspi_test.h`) is the single point of
  control -- confirmed genuinely removable, not just structured to
  look that way: building with it forced to 0 compiles clean and drops
  2308 bytes of `.text`, with `QSPI:ID?` disappearing from the command
  table entirely (falls through to `ERR 1` like any unknown mnemonic).
  Required copying `stm32g4xx_hal_qspi.c`/`.h` into `Drivers/` (never
  present in this project before -- QUADSPI was never enabled via
  CubeMX here) from `STM32Cube_FW_G4_V1.6.3`, chosen specifically
  because it bundles HAL Driver V1.2.7, an exact version match against
  every other driver file already in this project (confirmed via each
  file's own version banner, not assumed).
  **Verified on real hardware, with a real bug found and fixed in the
  process**: the first attempt returned a stable "DE 80 30" instead of
  Winbond's real "EF 40 18" -- confirmed this was a precise **1-bit
  sample-timing error**, not noise (`EF4018 << 1 == DE8030` exactly, as
  one continuous 24-bit stream). Root cause: `SampleShifting` was
  `QSPI_SAMPLE_SHIFTING_HALFCYCLE` (the usually-recommended default);
  switching to `QSPI_SAMPLE_SHIFTING_NONE` fixed it immediately and
  repeatably (`OK EF 40 18`, correct Winbond manufacturer ID + W25Q128
  memory type/capacity, 3x confirmed) -- see `qspi_test.c`'s own
  bugfix comment and `docs/changelog.txt` for the fuller writeup. This
  confirms the new QUADSPI hardware itself (pins, traces, the
  W25Q128JVS) is correctly wired and responding, not just that the
  firmware compiles.
- **`PFMIN:CAPTURE`/`STATus?`/`DATA?`** (`pfm_input.c`/`pfm_input.h`,
  hooked into `pfm.c`, `commands.c`) -- per-period **period-only**
  measurement (rising-edge-to-rising-edge; duty/time-high was dropped,
  see below) on the 6 PFM_Input_01..06 pins
  (PA15/PD4/PB2/PC12/PB4/PD12, TIM2/TIM3/TIM4/TIM5), added 2026-09-08.
  **Capture is synchronized to PFM shot lifetime, not a standalone
  command** -- `PFMIN:CAPTURE <M>` only arms a target period count per
  channel (`PfmInput_Arm()`); the actual `HAL_TIM_IC_Start_IT()` calls
  happen inside `PFM_Restart()` (`pfm.c`), the same function that
  starts `HRTIM1_PWM_Start()`, via a `PfmInput_OnShotStart()` call
  right alongside it -- so capture begins at the literal same call as
  the shot, not a separately-timed command. `PfmInput_OnShotEnd()`,
  called from everywhere `pfm.c` stops a shot, bounds a capture's
  runtime to the shot that started it -- no separate, arbitrarily-
  chosen timeout exists or was needed. Each of the 6 channels captures
  fully independently, plain `TIM_ICPOLARITY_RISING`, armed once at
  init and never changed again. **Easily removable by design**
  (`PFM_INPUT_FEATURE_ENABLED`, `pfm_input.h`), same pattern as
  `boot_jump.h`/`qspi_test.h` -- confirmed genuinely removable, and
  this check caught a real linkage gap before it shipped: `extern
  TIM_HandleTypeDef htim2..htim5` and `stm32g4xx_it.c`'s 4
  `IRQHandler()`s referencing them must be guarded identically to
  `pfm_input.c`'s own definitions, or disabling the feature fails to
  link -- confirmed by actually building the disabled configuration,
  not just re-reading the fix. `stm32g4xx_hal_tim.c`/`.h`/`_ex` copied
  into `Drivers/` from `STM32Cube_FW_G4_V1.6.3` (same HAL-V1.2.7
  exact-version-match reasoning as the QSPI driver copy).
  **Command-layer and shot-synchronized start/stop plumbing verified
  on real hardware.**
  **Duty-cycle measurement was attempted and dropped, NOT
  root-caused.** Two real-hardware designs were tried for measuring
  time-high (both rise and fall on every channel):
  `TIM_ICPOLARITY_BOTHEDGE` + live-GPIO edge disambiguation (a genuine
  race -- HAL clears `CCxIF` before the callback runs, so a later edge
  can silently overwrite `CCR` without tripping the overcapture flag),
  then an alternating-single-polarity redesign (flip
  `TIM_ICPOLARITY_RISING`/`_FALLING` directly via `CCER` inside the
  ISR -- the standard, architecturally-correct STM32 technique). Both
  produced real signals wired to `PFM_Input_01` reading back
  *consistently one full real period late* (measured 20/20 periods:
  reported high == real period + real high; reported period == 2x real
  period) -- see `docs/changelog.txt` for the full diagnostic history,
  including a disassembly-confirmed CCxE-toggle fix (the documented
  STM32 requirement for a clean polarity change) that compiled exactly
  as intended and still made zero measured difference. A follow-up
  isolation test (STM32's native PWM Input Mode, cross-connecting one
  timer's CH1+CH2) caused two real hangs requiring physical power-
  cycles and was abandoned before yielding data. **Decision**: rather
  than continue chasing an unexplained bug whose one common factor was
  an in-ISR polarity change, this module now measures period only,
  armed `RISING` once and never touched again -- no polarity flip
  exists anywhere in this file any more. **Confirmed fixed on real
  hardware, same day**: repeated the phase-U-into-`PFM_Input_01` test
  (identical 100-entry/100 kHz/40%-duty table) -- `PFMIN:DATA?`
  reported the real period (1699 ticks) 20/20 times, `OVERCAP=0`,
  cross-checked against a fresh DSLogic capture of the same shot
  (48 real periods averaging 1697.1 ticks at 170 MHz, within ordinary
  cross-clock quantization noise of 1699 -- nothing like the old bug's
  exact 2x signature). Root cause of the original bug is still not
  explained -- sidestepped, not fixed -- but the measured data is now
  correct. `docs/command_reference.md` does not document these
  commands yet. Restricted to `PFM_Input_01` only, via
  `PFM_INPUT_ACTIVE_CHANNEL_MASK` (`pfm_input.h`) -- the other 5
  channels aren't wired to anything on this bench, and leaving them
  fully initialized was adding CPU/interrupt load for no benefit while
  the investigation below was open. A timer with no active channel is
  left completely untouched (no clock, no NVIC, no GPIO), not merely
  armed with `M=0`.
- **RESOLVED 2026-09-09: real, DSLogic-confirmed missing PWM pulses on
  V/W during a fast frequency ramp -- an HRTIM SET/RESET-event
  collision, not a `pfm_input.c` bug.** A frequency-ramp test (`python/
  pfm_table_upload.py` `PROFILE 6`/`7`, added this session specifically
  to verify period capture tracks a *changing* signal) found real,
  DSLogic-confirmed missing output pulses -- not a capture artifact --
  on V and W starting as low as ~68-69 kHz, well below where U alone
  had seemed safe. Root cause, found via ST's own community
  engineering forum (search "HRTIM PWM transients greater than a
  period"): when an HRTIM timer's output SET and RESET events fall
  within ~3 HRTIM clocks of each other, RESET always wins and SET is
  silently dropped -- V/W's SET (`TIMPER`, meaning their own
  `MASTER_CMPk` reset-trigger event, which itself shifts every table
  entry) could land within a few clocks of the *previous* cycle's own
  duty compare (`CMP1`) during a fast ramp. **Channel 0 (U) was NOT
  actually immune** -- a first fix (V/W only) confirmed clean on V/W
  but then unmasked the SAME failure on U at the same per-value
  transitions, just rarer. **Fix** (`HRTIM1_FullInit()`, `hrtim.c`):
  every active channel's output SET moved off the un-prioritized
  `TIMPER` onto a dedicated `CMP3` compare (HRTIM favors the
  higher-numbered compare on a collision, so SET now wins instead of
  losing) -- required also updating `HRTIM1_PWM_Start()`'s cold-start
  settling logic, since channel 0's SET source no longer coincides
  with its own reset event. **Confirmed fixed on real hardware, all 6
  channels** (U/UN/V/VN/W/WN), 60-100 kHz ramp, DSLogic: zero anomalies
  anywhere, cold start unaffected. Generalizes automatically to any
  future `HRTIM_NUM_CHANNELS` (1-5, `ctrlr_config.h`) -- every place
  this fix lives is keyed off that constant, not a hardcoded count;
  re-verify on real hardware when raising it, the same way this fix
  itself was verified, not assumed. Also added, same session: a hard
  `PFM_MAX_CARRIER_FREQ_HZ` (100 kHz, `ctrlr_config.h`) ceiling on
  every `TABLE:STEP`, rejected with `ERR 10` -- predates and is
  independent of the SET/RESET fix, kept as a documented, tested
  boundary rather than removed now that the fix holds. Full diagnostic
  history (DSLogic captures, exact failure signatures, dead ends) in
  `docs/changelog.txt`'s 2026-09-09 entries.

## Tech stack & layout

- C11, STM32Cube HAL (G4), bare metal -- no RTOS, no heap.
- Built with STM32CubeIDE's generated makefile; toolchain
  arm-none-eabi-gcc 13.3.
- `Core/Src|Inc/` -- all project code. Currently: `main.c`, `uart.c`,
  `cmd_parser.c`, `commands.c`, `boot_jump.c`, `hrtim.c`, `pfm.c`,
  `gate_driver.c`, `qspi_test.c`, `pfm_input.c`, `pid.c`,
  `state_machine.c` (added 2026-09-13 -- the top-level IDLE/ARMED/
  FIRING/FAULT operating-state machine, see `docs/command_reference.md`'s
  `ARM`/`DISARM`/`STATE?` section and that file's own header comment),
  plus CubeMX-generated `stm32g4xx_hal_msp.c`/`stm32g4xx_it.c`/
  `system_stm32g4xx.c`/`syscalls.c`/`sysmem.c`.
- `python/` -- host-side tooling (see above).
- `docs/` -- this documentation set.

## Build & verify

**Canonical way to build, as of 2026-09-11:**

```bash
python3 python/wham_build.py
```

One command instead of three separate steps to remember. It runs, in
order: (1) `python/gen_git_version.py`, regenerating
`Core/Inc/git_version.h` from the current git state -- so `*IDN?`'s
reply always reports exactly which commit this build is from (see
`docs/command_reference.md`'s own `*IDN?` section); (2) `make -j4 all`
in `Debug/`; (3) `arm-none-eabi-objcopy -O binary` to regenerate
`Debug/WHAM-XREX-PFMG474.bin` -- this project's `.cproject` does **not**
have CubeIDE's "Convert to binary file (.bin)" post-build step enabled
(confirmed; unlike the sibling project, which does), so a bare `make`
alone never touches the `.bin` at all. Skipping this step is a REAL,
confirmed-on-hardware gotcha: `wham_serial_flash.py` against a stale
`.bin` flashes it "successfully" -- verified even -- while silently NOT
containing your latest changes (see `docs/changelog.txt`'s 2026-09-10
entry for exactly this happening). `wham_build.py` exists specifically
so there's no gap in the sequence left to forget.

If you need the two lower-level steps directly (debugging the build
itself, say):

```bash
cd Debug && make all -j4
arm-none-eabi-objcopy -O binary WHAM-XREX-PFMG474.elf WHAM-XREX-PFMG474.bin
```

-- but prefer `wham_build.py` for anything you intend to flash, for
`*IDN?` accuracy. `Core/Inc/git_version.h` is gitignored (see that
file's own generated header comment, and `.gitignore`'s comment on
why) -- a fresh checkout has no `git_version.h` until
`gen_git_version.py` (or `wham_build.py`) runs at least once;
`commands.c` `#include`s it unconditionally, so a bare `make` with
neither ever run fails to compile `commands.c` with a plain
missing-header error, not a mysterious one.

**Adding a new source file from outside CubeIDE (e.g. this agent
writing a `.c`/`.h` pair directly)**: CubeIDE's managed build
auto-discovers new files in a source directory the *first* time you
build a fresh project (no `Debug/` yet) -- but once `Debug/` exists,
its generated per-directory `subdir.mk` and the top-level
`Debug/objects.list` are **not** automatically refreshed by a plain
`make`. Every file added this way in this project (`Core/Src`:
`boot_jump.c`, `cmd_parser.c`, `commands.c`, `uart.c`, `gate_driver.c`,
`qspi_test.c`, `pfm_input.c`; `Drivers/STM32G4xx_HAL_Driver/Src`:
`stm32g4xx_hal_qspi.c`, `stm32g4xx_hal_tim.c`, `stm32g4xx_hal_tim_ex.c`
-- all copied in from an external HAL package rather than hand-written,
but the exact same gotcha applies) needed its
directory's own `subdir.mk` plus `Debug/objects.list` hand-patched to
add the new `.c`/`.o`/`.d` entries before `make` would pick it up. In
CubeIDE itself, `F5` (Refresh) + a normal Build regenerates these
correctly, no hand-patching needed -- the manual patching is only
necessary when
building from the command line without going through the IDE first.

**`%f`/`%g` in a `snprintf()` reply silently prints nothing** (not a
crash, not a build warning -- just empty output where the number
should be) unless the link includes `-Wl,-u,_printf_float`. This
project links against newlib-nano (`--specs=nano.specs`), which strips
floating-point support out of `printf`/`snprintf` by default to save
flash. Confirmed on real hardware, 2026-09-10: the first version of
`PID:GAINS?`/`PID:PROFILE:CURRENT?`/`PID:PROFILE:TIMING?` (see
`docs/changelog.txt`) built and linked cleanly, then replied `OK   `
(spaces where the numbers should be) on real hardware -- traced to
this exact missing flag. Fixed by hand-adding `-Wl,-u,_printf_float`
to `Debug/makefile`'s link line (adds ~6 KB flash -- worth tracking,
see "Host-side tooling" below). Like the `subdir.mk`/`objects.list`
gotcha above, this is a project (`.cproject`) setting CubeIDE would
normally manage -- in the IDE itself it's Project Properties -> C/C++
Build -> Settings -> MCU Settings -> "Use float with printf from
newlib-nano (`-u _printf_float`)" -- and a full CubeIDE-driven
regeneration of `Debug/makefile` could silently drop the hand-added
flag again; re-check `PID:GAINS?` (or any other `%f`/`%g` reply) after
any such regeneration.

- No on-host test suite. Verification so far = clean build (zero
  warnings) + real hardware round trips over the serial link (see
  `docs/changelog.txt` for what's actually been confirmed on hardware
  vs. only compiled/linked).
- Serial console: **115200 8N1** (raised from 9600 on 2026-09-04, see
  "Hard-won invariants" below and `docs/changelog.txt`), SCPI-style
  mnemonics (case-insensitive, short/long form), `OK ...` /
  `ERR <code> <msg>` responses. Full protocol: `docs/command_reference.md`.

## Coding conventions

Same conventions as the sibling project (module-prefixed public
functions, `stdint.h` exact-width types, minimal non-blocking ISRs,
`volatile` on ISR-shared state, bugfix comments that state root cause +
symptom + why the fix works) -- there's no separate style doc here yet
since the codebase is still small; read `boot_jump.c` and `cmd_parser.c`
for the current standard to match.

## Hard-won invariants

1. **Baud rate is 115200 everywhere on purpose** (raised from 9600 on
   2026-09-04, real-hardware experiment -- see `docs/changelog.txt`;
   `python/wham_serial_flash.py`, `python/pfm_table_upload.py`, and
   `python/scpi.py` were all updated to match). `main.c`'s
   `huart2.Init.BaudRate = 115200` line lives in CubeMX-generated code
   *outside* any `USER CODE` marker. A `.ioc` "Generate Code" would
   reset it back to HAL's own default -- which, as it happens, IS
   115200 right now, so a regen wouldn't silently break this today.
   That's a coincidence, not a guarantee: if this value ever needs to
   change again, re-apply it explicitly rather than trusting a regen to
   land on the right number by chance. This is a WHAM-PFMG474-V4-only
   divergence -- the sibling PFM-STM32G474 project still uses 9600, and
   nothing here implies porting this change there.
   - **921600 does not work on this hardware** -- tried directly after
     115200 (skipping the standard steps in between) as part of the
     same experiment: consistently garbled, same-length-but-wrong-content
     replies (a real baud mismatch/reliability failure, not a fluke --
     retried 3x). Left unexplored which of 230400/460800 is the actual
     practical ceiling; 115200 was chosen as "clearly safe and already a
     large win," not as "the fastest verified-working rate."
   - **If a bad baud change ever strands the board** (as 921600 did,
     mid-experiment): the serial `BOOT` command needs a working link to
     even request the ROM bootloader, so it can't recover a board stuck
     at a wrong baud. An ST-Link + `st-flash write <bin> 0x08000000`
     (or `st-info --probe` first to confirm it's detected) recovers
     over SWD, completely bypassing the broken serial link -- confirmed
     working for exactly this scenario the same day.
2. **`BootJump_CheckAndEnter()` must stay the literal first statement in
   `main()`**, before `HAL_Init()`. This is what lets the ROM bootloader
   jump-in path (entering, not the `Go` command discussed above) work
   reliably -- see `boot_jump.c`'s header comment.
3. **USART2's NVIC priority/enable call lives inside `MX_USART2_UART_Init()`'s
   `USER CODE BEGIN USART2_Init 2` block**, not in `USER CODE BEGIN 2`
   with everything else -- without it, `HAL_UART_Receive_IT()` arms but
   the interrupt never actually reaches the NVIC, and nothing over
   serial ever gets a reply. Easy to lose track of on a CubeMX regen if
   you're not looking in the right `USER CODE` block.

## Known gaps / in-flight work (as of 2026-08-31)

- **The VTOR/MEMRMP bugfix (see above) has not been ported to the
  sibling V3/V4 projects yet**, which carry the identical unfixed
  `boot_jump.c` this one was forked from. Deliberately deferred, not
  forgotten -- ask before assuming it should happen automatically.
- **RESOLVED 2026-09-04: `FIRE` now exists.** `HRTIM1_EnableMasterInterrupt()`
  (`hrtim.c`) and `HRTIM1_Master_IRQHandler()` (`stm32g4xx_it.c`) were
  ported from the sibling project (state-machine/fault-pin calls
  stripped, since neither exists here), enabled once at boot from
  `main.c` (after `FixSysTickPriority()`, also newly ported, has raised
  SysTick off the HAL default -- see that function's own doc comment
  for the priority scheme: SysTick=0, HRTIM1_Master=1, USART2=2).
  `cmd_fire()` (`commands.c`) wraps `PFM_Restart()`, rejecting with
  `ERR 5` against an empty table. No ARM/state-machine interlock was
  added -- `FIRE` always takes effect immediately; adding an interlock
  is future work, not implied by this change. See
  `docs/test_protocol.md`'s T3 for the (now largely superseded, but
  still useful for `hrtim.c`-only bring-up) developer-only direct-call
  path this bypassed. **Verified end-to-end on real hardware the same
  day** (flash -> upload a real 500-entry table -> `FIRE` -> `OK`) --
  command-layer round trip only, no scope available in that pass, so
  the actual waveform (T4.5-T4.8) is still unobserved. That same pass
  also caught and fixed an unrelated ~25-50x slowdown in
  `pfm_table_upload.py`'s own reply-reading loop -- see
  `docs/changelog.txt`, not a firmware issue.
- **RESOLVED 2026-09-04: cold-start phase-lock glitch on every `FIRE`,
  fully confirmed on real hardware.** The waveform T4.5-T4.8 left
  unobserved above WAS observed, via DSLogic captures -- and went
  through three iterations before it was actually right (full story in
  `docs/changelog.txt`'s several 2026-09-04 entries; this is the final
  state only). Root cause: `HRTIM1_PWM_Start()`'s force-ACTIVE step
  (needed to avoid an undefined complementary-pair state, per ST's own
  guidance) puts U/V/W's main outputs HIGH at the same instant with no
  regard for their intended 120°/240° stagger, and Timers B/C
  (`ResetTrigger = MASTER_CMP1`/`MASTER_CMP2`) don't become genuinely
  phase-locked until they've received that first reset -- which, unlike
  Timer A's (`MASTER_PER`, coincides with its own natural rollover), an
  external reset does NOT itself regenerate the output's SET state.
  Fixed in `HRTIM1_PWM_Start()`: each phase now waits for and connects
  to its own pins SEPARATELY, at its own reset-trigger event (V at
  `MASTER_CMP1`, W at `MASTER_CMP2`, U at `MASTER_PER`/`MREP`, via the
  new static helper `HRTIM1_WaitForPhaseAndConnect()`), with V/W's
  output re-forced ACTIVE at that exact moment (a direct `SETx1R`/
  `RSTx2R` write, same reasoning as the direct `OENR` write) so their
  first visible pulse is a genuine fresh start, not a snapshot of
  whatever their comparator had been doing since an earlier, still-
  hidden reset. Global interrupts are masked for the whole sequence
  (`HRTIM_MASTER_IT_MREP`/`MCMP1`/`MCMP2`, all armed at boot by
  `HRTIM1_EnableMasterInterrupt()`, would otherwise fire mid-sequence
  and let `PFM_CycleBoundaryHandler()` silently advance the table
  before step 0 was ever visible) -- bounded by a spin-count ceiling,
  not `HAL_GetTick()`, since that can't advance while masked.

  **Confirmed on real hardware**: a ~5 ms DSLogic capture spanning 499
  of profile 3's 500 periods shows phase spacing holding at
  ~3300 ns/~6600-6700 ns (target 3333/6667 ns) consistently from the
  cold-start edge through the last period, and U's duty tracking the
  90% -> 10% ramp cleanly throughout, landing exactly on 10.0% at the
  final period -- the fix holds for the entire shot, not just the
  startup instant.
- **RESOLVED 2026-09-08: channel count is a real, working, compile-time
  setting.** `version.h`'s `PWM_NUM_CHANNELS` was documentation-only
  ("NOT read by hrtim.c today") since it was first added -- generalized
  the whole PWM/HRTIM path to a real `HRTIM_NUM_CHANNELS` (1-5) in a
  new `ctrlr_config.h`, per a direct request to plan this out first
  (see the plan file this was implemented from, and the design
  discussion earlier in that session for the full "why 1-5, why
  compile-time, why not channel F" reasoning). Touched: `PFM_Step_t`
  (fixed `cmpA/cmpB/cmpC` -> `cmp[HRTIM_NUM_CHANNELS]`, sized exactly
  per build, no waste), `PFM_Phase120()/PFM_Phase240()` ->
  `PFM_PhaseForChannel()`, `PFM_Phase_t`/`PFM_SetPhaseEnabled()`/
  `PFM_GetPhaseEnabled()` -> a plain 0..N-1 channel index (durable
  per-channel enable array, same contract as before), every hardcoded
  A/B/C block in `HRTIM1_FullInit()`/`HRTIM1_PWM_Start()`/
  `HRTIM1_PWM_Stop()`/`HRTIM1_SoftwareUpdate()`/`HRTIM1_ApplyPfmStep()`
  -> a loop over 0..N-1 (the per-phase cold-start settling mechanism
  from the entry above it generalized directly, unchanged in mechanism
  -- channel k waits on `MASTER_CMPk`, ascending, channel 0 waits on
  `MASTER_PER`/`MREP` last), the per-channel register-address getters
  -> single indexed getters backed by lookup tables. `TABLE:STEP`'s
  wire format is now `per` + N compare values, not a fixed 4 -- new
  `CONFig:CHANnels?` (`OK <N>`) lets host tooling confirm N instead of
  assuming it; `python/pfm_table_upload.py` queries it at startup and
  refuses to upload on a mismatch.

  A real bug was found and fixed during this work, not just designed
  around: the first attempt stringified `TABLE:STEP`'s expected token
  count via the standard `#x` preprocessor idiom, which only works on
  plain literals -- `1 + HRTIM_NUM_CHANNELS` being an *expression*
  meant the `ERR 4` message printed the literal unevaluated text
  `"(1U + (4U))"` instead of `"5"`. Fixed with a runtime `snprintf()`
  instead of compile-time stringification.

  **Verified on real hardware**: full clean rebuild at the default N=3
  (zero warnings, `.bss` unchanged -- `PFM_Step_t` stays exactly 8
  bytes/entry at N=3, confirming no regression), reflashed, `*IDN?` and
  the new `CONFig:CHANnels?` both correct, a real table uploaded and
  `FIRE`d twice with the board staying responsive throughout. Then
  temporarily rebuilt at N=4 specifically to prove the generalization
  works at a *different* channel count, not just N=3 restated: `.bss`
  grew by exactly the predicted +10,000 B (2 B/entry x 5000 entries),
  `CONFig:CHANnels?` correctly reported 4, a 4-channel-wide table
  uploaded and fired cleanly, and the wrong-width-table rejection
  (`ERR 4`) was confirmed too. Also confirmed the `#error` compile-time
  guard actually fires for an out-of-range N=6. Reverted to the
  project's real N=3 default before the final commit -- N=4/5 are
  proven to work, not shipped as the active setting.

  **Not yet done**: a DSLogic capture confirming N=4/5's actual
  waveform (even spacing at the new angle, correct dead time on the
  added channel) -- everything above verifies the command layer and
  the build/RAM math, not the electrical output, and this project's
  physical probes have only ever been wired to 3 channels. The N=3
  regression pass also has no fresh DSLogic capture of its own in this
  entry (though extensive command-layer verification found nothing
  wrong, and the underlying settling mechanism is provably unchanged
  for channel 0..2 -- see the diff) -- a scope/DSLogic pass confirming
  N=3's waveform is still bit-for-bit the same as the baseline above
  would close this out completely.
- **`HRTIM1` *is* represented in the `.ioc` now** (as of 2026-09-04,
  hand-constructed, not GUI-generated -- see `docs/changelog.txt`).
  Do NOT reopen HRTIM1's Mode/Configuration panel in CubeIDE's Pinout &
  Configuration GUI -- doing so once already triggered a full
  regeneration that silently rewrote the clock tree and produced
  duplicate-symbol link errors against `hrtim.c`'s hand-written
  `HAL_HRTIM_MspInit()`/`hhrtim1`. Other peripherals' pinout remains
  safe to configure through the GUI normally.
- **`_Min_Stack_Size` in `STM32G474QETX_FLASH.ld` is still the CubeMX
  nominal minimum (0x400 = 1 KB)**, not a value chosen for this
  firmware's actual worst-case call depth. Deliberately not changed
  yet (2026-08-31) -- revisit once `PFM_TABLE_SIZE` is actually raised
  from its current default (5000); the two decisions are linked (a
  larger table leaves less RAM headroom for a larger stack reservation
  to mean anything). Don't raise `PFM_TABLE_SIZE` far past 5000 without
  also revisiting this.
- **`HW_BOARD_REV` in `ctrlr_config.h`** (`"REVA"`) **is a placeholder**
  -- not yet confirmed against the actual board silkscreen.

## Documentation map

| Doc | Contents |
|---|---|
| `docs/command_reference.md` | Every serial command, SCPI matching rules, error codes |
| `docs/serial_reflash_guide.md` | How to use `wham_serial_flash.py`, the BOOT mechanism, the VTOR/MEMRMP fix, troubleshooting |
| `docs/changelog.txt` | Running change log, dated entries, root causes for fixes |
| `docs/memory_history.csv` + `docs/memory_report.html` | Flash/RAM footprint, tracked per commit since project start -- see `python/memory_report.py`'s entry above for how/when to regenerate |
| `docs/test_protocol.md` | Numbered bench test protocol (T0-T3) -- markedly shorter than the sibling project's, since it only covers what actually exists here (serial identity, BOOT/reflash incl. the Go-jump regression check, and a developer-only HRTIM smoke test). Update it as functionality is added rather than assuming it tracks the sibling's group numbering. |
| `docs/sop/wham_pfmg474_v4_sop.tex` | Formal Standard Operating Procedure: build, bootstrap flash, serial reflash, verification, troubleshooting. Same LaTeX house style as the sibling project (`mathpazo`, 5in×8.25in `geometry`, plain `\hline` tables) -- compile with `pdflatex` (twice, for cross-references), then render to PNG and visually inspect before calling any edit done; this document's own `\tabcolsep` note explains a real overfull-table pitfall worth reading before adding another table. |
