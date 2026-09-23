# WHAM-XREX-PFMG474 — Serial Command Reference

USART2, **115200 8N1** (raised from 9600 on 2026-09-04 — see `docs/changelog.txt`; WHAM-XREX-PFMG474-only, the sibling PFM-STM32G474 project still uses 9600). Commands are terminated by `\r`, `\n`, or `\r\n`.

## Response conventions

Ported from the sibling PFM-STM32G474 project, per project decision:

| Form | Meaning |
|---|---|
| `OK\r\n` | Accepted, no data |
| `OK <value>\r\n` | Accepted, with a return value |
| `ERR <n> <msg>\r\n` | Rejected; error codes are stable across firmware versions |

### Error codes

| Code | Meaning |
|---|---|
| 1 | Unknown command |
| 2 | Not currently uploading a table — send `TABLE:BEGIN` first |
| 3 | Table full (`PFM_TABLE_SIZE` entries already appended) |
| 4 | Invalid `TABLE:STEP` arguments (wrong count — must be `1 + HRTIM_NUM_CHANNELS`, see `CONFig:CHANnels?` — or a value outside uint16 range 0-65535) |
| 5 | Table is empty — `FIRE` has nothing to play back |
| 6 | Fault latched — either PC10/HRTIM1_FLT6 (native HRTIM hardware fault input) or the GateDriverStatus_01..12 EXTI interrupt (`PE0`-`PE11`) — `FAULT:CLEAR` required before `FIRE` will work again |
| 7 | QUADSPI command failed or timed out (see `QSPI:ID?`) |
| 8 | Invalid `PFM_Input` channel (1-6) (also reused by `PFMIN:DEBUG:RAW?`/`PFMIN:DEBUG:REG?`) |
| 9 | `M` out of range for `PFMIN:CAPTURE` (1-`PFM_INPUT_MAX_PERIODS`) |
| 10 | `TABLE:STEP` `per` value implies a carrier frequency above `PFM_MAX_CARRIER_FREQ_HZ` |
| 11 | Invalid channel (also reused by `OCP:TEST:FAULT`, `XREX:CHANnel:STATus?`, same channel-range check) |
| 12 | Invalid command arguments -- see the specific command's own usage (also reused by `OCP:TEST:FAULT`, `PFMIN:DEBUG:RAW?`/`PFMIN:DEBUG:REG?`, `XREX:CHANnel:STATus?`) |
| 13 | Invalid state-machine transition for the current state (`ARM`/`DISARM`/`SHOT:STARt`, see that section) |
| 14 | Invalid `CHANnel:NICKname` -- name must be 1-`PID_CHANNEL_NICKNAME_MAX_LEN` chars, no spaces, and not the reserved value `-` |
| 15 | `ARM`/`SOURce:RUN`/`SHOT:STARt` refused -- `EXTernal:ENAble` is on and the external-enable pin (`PF13` as of 2026-09-17, was `PF15`) does not currently read `HIGH` (see `EXTernal:ENAble`). `ARM` reporting this distinctly is new 2026-09-22 -- previously collapsed into `ARM`'s own generic `ERR 13` |
| 16 | **RETIRED 2026-09-17** -- previously `EXTernal:TRIGger 1` refused unless `EXTernal:ENAble` was already on, back when enable and trigger shared one pin (`PF15`). No longer generated: enable moved to `PF13`, trigger stayed on `PF15`, and the two are no longer coupled (see `EXTernal:TRIGger`). Kept here, not reassigned, per this table's own "never renumbered or reused" convention |

Codes are never renumbered or reused once assigned, matching the
sibling project's convention.

## Mnemonic syntax (SCPI-style)

Commands are matched by `cmd_parser.c`'s `scpi_match()` against a flat
table of patterns — see that file's header comment for the full
specification. Summary:

- **Hierarchical**: pattern levels are separated by `:`, e.g.
  `SOURce:VOLTage:LIMit`. A leading `:` on the input is tolerated but
  never required (there's no "current path" concept — every command is
  matched from the root).
- **Short/long form, per level**: in a pattern token, the leading
  UPPERCASE run is the mandatory short form; any lowercase letters
  after it are an optional long-form suffix that must be matched *in
  full* if used at all — no partial-long-form matches. Example:
  `VERSion` accepts `VERS`, `VERSION`, `version` — not `VERSI`.
- **Case-insensitive** throughout.
- **Query suffix `?` must match exactly**: a pattern ending in `?` only
  matches input also ending in `?`, and vice versa.
- **Common (IEEE 488.2) commands** like `*IDN?` are just zero-colon
  patterns — no special-casing needed.

Compound commands (`;`-separated, multiple mnemonics on one line) are
**not** supported yet.

## Commands

### `*IDN?`

Board and firmware identification.

```
> *IDN?
< OK WHAM-XREX-PFMG474 REVA v0.1 74994292
  (or, built from an uncommitted working tree:)
< OK WHAM-XREX-PFMG474 REVA v0.1 74994292-dirty
```

Reports, space-separated: `HW_BOARD_NAME`, `HW_BOARD_REV`,
`FW_VERSION_STRING` (compile-time constants, `Core/Inc/ctrlr_config.h`),
then `FW_GIT_COMMIT` (added 2026-09-11, per direct request) — the
short git commit hash this exact firmware build was made from, an
`-dirty` suffix if the working tree had uncommitted changes at build
time, or `unknown` if `Core/Inc/git_version.h` was never generated
(git unavailable, or built some other way entirely). This is how to
answer "which commit is actually running on this board right now" with
certainty, independent of what you think you last flashed.

`HW_BOARD_REV` is currently a placeholder (`REVA`); update it to match
the actual PCB silkscreen revision.

**Build with `python3 python/wham_build.py`** (not a bare `make`) to
get an accurate `FW_GIT_COMMIT` — it runs `python/gen_git_version.py`
(regenerates `Core/Inc/git_version.h` from the current git state) before
`make`, then regenerates `Debug/WHAM-XREX-PFMG474.bin` (this project's
`.cproject` has no "Convert to binary" post-build step -- `make` alone
never touches the `.bin` at all, a real gotcha this same script exists
to close, see `docs/changelog.txt`'s 2026-09-10 entry). One command
instead of three separate steps to remember; see that script's own
header comment. `git_version.h` is gitignored -- see `.gitignore`'s own
comment on why it's deliberately not tracked.

### `BOOT`

Resets the MCU into the STM32 ROM serial bootloader (System memory), so
a new `.bin` can be written over the same USART2 link with no physical
BOOT0/NRST access. See `docs/serial_reflash_guide.md` for the full
mechanism and usage via `python/wham_serial_flash.py`.

```
> BOOT
< OK ENTERING BOOTLOADER
  (link drops — MCU has reset into the ROM bootloader)
```

Present only when `BOOT_JUMP_FEATURE_ENABLED` (`Core/Inc/boot_jump.h`)
is nonzero (the default). When disabled, `BOOT` is simply unrecognized
(`ERR 1 Unknown command`), like any other unknown mnemonic — see that
header for exactly what disabling the module does.

No state-machine/Firing concept exists in this firmware yet, so `BOOT`
has no rejection conditions today. When application logic that can be
mid-operation is added, gate this command the same way the sibling
project's `cmd_boot()` does (reject with an `ERR` code while active —
resetting under load would drop outputs uncontrolled).

### `TABLE:BEGIN`, `TABLE:STEP`, `TABLE:END`, `TABLE?`

Uploads a complete PFM shot profile (a sequence of `(per, cmp0, ...,
cmp(N-1))` steps, N = `HRTIM_NUM_CHANNELS` — see `Core/Inc/pfm.h`'s
`PFM_Step_t` and `Core/Inc/ctrlr_config.h`), built entirely
off-controller. This firmware has no on-device table *construction*
logic (no frequency/duty math, no sweep builder) — see `pfm.h`'s
"ADDED" header note for why that's a deliberate project decision, not
a gap waiting to be filled in here. `python/pfm_table_upload.py` is
the reference host-side builder — it queries `CONFig:CHANnels?` itself
before uploading, rather than assuming N.

```
> TABLE:BEGIN
< OK
> TABLE:STEP 1699 850 850 850
< OK
> TABLE:STEP 1699 850 850 850
< OK
  ... (repeat for every entry -- example above is N=3) ...
> TABLE:END
< OK 500
```

- **`TABLE:BEGIN`** — clears the table (`PFM_TableReset()`) and opens
  an upload session. Safe to call again mid-upload to start over.
- **`TABLE:STEP <per> <cmp0> ... <cmp(N-1)>`** — appends exactly one
  entry: `per` plus exactly one compare value per channel (N =
  `HRTIM_NUM_CHANNELS`, a compile-time constant — query
  `CONFig:CHANnels?` rather than assuming it). All values are
  `uint16_t` (0-65535); no other validation happens here (see
  `PFM_AppendStep()`'s doc comment — the eventual apply-time
  `HRTIM1_ClampCompare()` in `hrtim.c` is a backstop, not a substitute
  for the host script sending sane values). Rejects with `ERR 2` if no
  `TABLE:BEGIN` is open, `ERR 3` if the table is already at
  `PFM_TABLE_SIZE` (5000) capacity, `ERR 4` for a malformed line
  (including the wrong token count for this board's N).
- **`TABLE:END`** — closes the upload session and reports the final
  entry count (`OK <count>`). Table stays exactly as uploaded even if
  you never send this — it just stops enforcing "must call
  `TABLE:BEGIN` first" for a stray `TABLE:STEP`.
- **`TABLE?`** — reports the current entry count at any time
  (`OK <count>`), upload session open or not.

### `FIRE`

Begins PWM output: (re)starts playback of whatever table is currently
uploaded, from step 0. Playback advances automatically, one table
entry per PWM period, driven by the HRTIM1 master-repetition interrupt
(`HRTIM1_Master_IRQHandler()` in `stm32g4xx_it.c`, calling
`PFM_CycleBoundaryHandler()` in `pfm.c`) — no polling, no further
commands needed once fired. Output stops automatically, at a coherent
period boundary, once the last table entry has completed.

```
> FIRE
< OK
  (HRTIM channels 0..N-1 begin switching, evenly phase-spaced -- see
   hrtim.c's HRTIM1_PWM_Start(); channels beyond N are never started,
   see hrtim.h and ctrlr_config.h)
  ... plays back every uploaded (per, cmp0, ..., cmp(N-1)) entry in
      order, one period each ...
  (output stops on its own after the last entry — no further command
   or reply marks this; poll TABLE? or scope the outputs)
```

- Gated on the state machine as of 2026-09-22: `FIRE` now requires
  `ARM` first (`ERR 13`) and, when `EXTernal:ENAble` is on, the PF13
  interlock to be satisfied (`ERR 15`) -- in addition to the existing
  `ERR 6` (fault latched) and `ERR 5` (empty table) checks. The legacy
  table path is deliberately **not** transitioned to `FIRING` (it isn't
  part of the SM's IDLE/ARMED/FIRING lifecycle — see `state_machine.h`),
  so `STATE?` stays `ARMED` during playback; a fault mid-playback safes
  it via `EnterFault()` from `ARMED`. Re-firing mid-shot restarts from
  step 0.
- Rejects with `ERR 5` if the table is empty (`TABLE?` reports 0).
  Without this check, an empty-table `FIRE` would still briefly enable
  outputs at step 0's (garbage, never-written) register contents before
  the very first master-repetition interrupt stopped them again —
  `ERR 5` catches this before it can happen at all.
- No `STOP` command exists yet — playback only stops on its own, at
  table exhaustion. Adding an operator-initiated stop is future work.

### `FAULT?`, `FAULT:CLEAR`

Status/clear for **two independent fault sources**, combined into one
answer here (`commands.c`'s `AnyFaultLatched()`) — from an operator's
perspective, "is there a fault, and can I `FIRE`" is one question, not
two, even though the two mechanisms underneath stay structurally
separate:

1. **PC10/HRTIM1_FLT6** — a real HRTIM1 hardware fault input
   (`Core/Inc/hrtim.h`), active-low. The peripheral forces every
   fault-enabled channel's outputs to a safe (inactive) level
   autonomously, in silicon, the instant the pin trips — no CPU,
   interrupt, or polling latency, and it keeps working even if the CPU
   is hung.
2. **GateDriverStatus_01..12 EXTI interrupt** (`PE0`-`PE11`,
   `Core/Inc/gate_driver.h`) — software/interrupt-driven. Re-reads all
   12 pins on any edge and, as of 2026-09-17, delegates the fault
   decision to `XrexIo_EvaluateGateDriverFault()` (`xrex_io.c` — see
   `XREX:CHANnel:STATus?` below): these 12 pins are
   `docs/pin_mapping_v4.csv`'s `XR1`-`XR4` `_WATER_FLT`/`_TMP_FLT`/
   `_ENERPRO_FLT` signals, each category independently polarity-
   configurable (`XR_WATER_FLT_POLARITY`/`XR_TMP_FLT_POLARITY`/
   `XR_ENERPRO_FLT_POLARITY`, `Core/Inc/ctrlr_config.h` — replaces the
   old single shared `GDS_FAULT_POLARITY`), and gated so a disabled
   Transrex channel's own pins never count toward a fault. On a fault,
   forces HRTIM output off and latches, exactly as before. Needs the
   EXTI ISR to actually run, unlike PC10's autonomous hardware path — a
   boot-time explicit check (`main.c`) covers the one gap that leaves (a
   pin already faulted before the interrupt is even armed produces no
   edge of its own).

```
> FAULT?
< OK 0
> FIRE
< OK
   ... (a fault trips mid-shot; outputs go safe immediately) ...
> FAULT?
< OK 1
> FIRE
< ERR 6 Fault latched -- send FAULT:CLEAR first
> FAULT:CLEAR
< OK
> FAULT?
< OK 0
```

- **`FAULT?`** — `OK 0` (healthy) or `OK 1` (latched), either source.
- **`FAULT:CLEAR`** — clears both latches unconditionally (clearing a
  source that was never tripped is a harmless no-op). Does **not**
  itself reconnect/restart outputs — that's the next explicit `FIRE`'s
  job. For the GateDriverStatus source specifically, clearing
  immediately re-validates by re-checking all 12 pins: if any pin is
  still in its fault state, `FAULT:CLEAR` re-latches before its own
  `OK` reply even goes out, rather than reporting success while the
  physical condition persists — confirmed on real hardware (see
  `docs/changelog.txt`, 2026-09-08).
- Latched-until-cleared by design, both sources — a fault silently
  clearing itself, with output resuming unnoticed, is the hazard this
  avoids.
- **Verified on real hardware, end to end**: a genuine GateDriverStatus
  fault (GateDriverStatus_03/PE2, present at boot) was correctly
  caught by the boot-time check, correctly blocked `FIRE` with `ERR 6`,
  and correctly re-latched on `FAULT:CLEAR` while still physically
  present — the full sequence above, not just the command-layer shape
  of it. The PC10/HRTIM1_FLT6 path is verified idle-safe (doesn't
  spuriously trip) and does not yet have its own real low-drive bench
  trip confirmed on scope/DSLogic — see `docs/changelog.txt` for what
  specifically remains open there. `FAULT:CLEAR` now also clears the
  top-level state machine (below) back to `IDLE`, if the underlying
  condition is actually gone.
- **Real bug, found and fixed 2026-09-15**: the "still physically
  present → re-latches" case above was genuinely verified, but the
  OPPOSITE case — a fault that's actually gone → `FAULT:CLEAR` actually
  succeeds — was not: `SM_ClearFault()` (`state_machine.c`) never
  actually called `HRTIM1_FaultClear()`/`GateDriver_FaultClear()`
  anywhere in the codebase, despite its own comment claiming it did —
  meaning a GateDriverStatus (and likely PC10) fault, once latched,
  could never actually be cleared at all, requiring a full power cycle
  to recover. Fixed — see `docs/changelog.txt`'s matching entry — but
  not yet re-verified against a real trip-then-actually-clear sequence
  on hardware (no way to trigger a real GateDriverStatus/PC10 fault
  this session); fixed by code inspection and confirming the missing
  call sites, not a live re-test.

### `ARM`, `DISARM`, `STATE?`

Added 2026-09-13, per direct request: a top-level **operating-state
machine** — `IDLE` → `ARMED` → `FIRING` → back to `IDLE`, with `FAULT`
reachable from any of the three the instant a fault is detected (both
sources above, checked continuously — not just while firing). See
`Core/Inc/state_machine.h` for the full design writeup; this is a
first pass, explicitly flagged for revisit (see its own "NOTE TO
REVISIT" comments).

- **`IDLE`** — the normal state: boot, after a shot completes, or
  after `FAULT:CLEAR`. Every channel's HRTIM output is disabled.
- **`ARMED`** — entered via `ARM` (only from `IDLE`), gated by a
  readiness check that is still partly a **STUB** (profile timing
  configured, gains sane, etc. are not checked at `ARM` time — only at
  `SHOT:STARt`, see `ERR 12` there) — with TWO real conditions
  populated: the external-enable interlock (2026-09-16, see
  `EXTernal:ENAble` below), if turned on, and (2026-09-17) every
  currently-enabled channel's `ENA_OUT`/`CONTACT_OUT` both being
  commanded `HIGH` (see `XREX:CHANnel:ENAOut`/`CONTactOut`). Outputs
  are still disabled here, identically to `IDLE` — nothing electrical
  changes on entry; `ARMED` exists purely as a separately-confirmable
  "ready to fire" step before `SHOT:STARt` is allowed to do
  anything. Left via `SHOT:STARt` (→ `FIRING`), `DISARM` (→
  `IDLE`, stand down without firing), or `SOURce:STOP` (→ `IDLE`, abort).

  **`ARM`'s failure reporting, reworked 2026-09-22** (direct request):
  previously every reason `ARM` could refuse — wrong state, the
  external-enable interlock, a specific channel's output not ready —
  collapsed into one generic `ERR 13 "not currently IDLE, or arm
  conditions not met"`, leaving an operator with one misconfigured
  channel out of several no way to tell which from the wire protocol
  alone. Each precondition is now checked explicitly and the first
  failure reported distinctly, same "distinct failure reasons,
  reported distinctly" pattern `SHOT:STARt`/`SOURce:RUN` already
  used:
  ```
  > ARM
  < ERR 13 Can't ARM -- not currently IDLE
  > ARM
  < ERR 15 External enable interlock not satisfied -- PF13 reads LOW
  > ARM
  < ERR 13 Channel 2's ENA_OUT/CONTACT_OUT not both set -- see
    XREX:CHANnel:ENAOut/CONTactOut
  ```
  Only the FIRST failing precondition is reported per attempt — fix it
  and re-send `ARM` to see the next one, if any. `SM_Arm()`'s own
  internal re-check of the same conditions (state_machine.c) still
  backs the actual transition as a last-line-of-defense against a
  narrow race window between these checks and the call; its own
  generic fallback message should never fire outside that race.
- **`FIRING`** — entered only from `ARMED`, via `SHOT:STARt`
  (unchanged command, now gated: `ERR 13` if not currently `ARMED`).
  Every currently-enabled channel (`SOURce:ENAble` — a separate,
  per-channel concern) begins its ramp profile. Returns to `IDLE`
  automatically when the shot completes, or on a manual `SOURce:STOP`.
- **`FAULT`** — entered from ANY state the instant either fault source
  trips. Always stops the legacy (`TABLE:*`/`FIRE`) output path
  immediately (it isn't part of this state machine at all). The
  current (`SOURce:*`) output path's own stop depends on the fault type:
  - **`GENERAL`** (`SM_FAULT_GENERAL`) — populated 2026-09-13, per
    direct instruction: if the fault hit while `FIRING`, every
    actively-outputting channel immediately begins an **open-loop**
    linear ramp-down (no PID/feedback correction at all) to
    `PFM_TURNON_FREQ_HZ` (0A) over `FAULT_RAMP_DOWN_TIME_S`
    (`ctrlr_config.h`, currently 1.0s, a placeholder) seconds — from
    wherever it actually was in its own shot profile, not from a
    shared/synchronized point. Once every participating channel
    reaches the floor, its output is disconnected and the controller
    settles into `FAULT` to wait for `FAULT:CLEAR`. If the fault hit
    while `IDLE`/`ARMED` (nothing was actually outputting), stops
    immediately instead.
  - **`OVERCURRENT`** (`SM_FAULT_OVERCURRENT`) — populated 2026-09-15,
    per direct instruction. PER-CHANNEL, unlike `GENERAL`: reported for
    exactly one channel via `SM_ReportOcpFault()` (see
    `OCP:TEST:FAULT` below — no real per-channel OCP hardware pin is
    wired up anywhere in this codebase yet, mapping still to be
    defined). If the fault hit while `FIRING`: that channel's own HRTIM
    output is disabled IMMEDIATELY (no ramp for it at all — the
    opposite of `GENERAL`'s treatment of every channel), every OTHER
    currently-enabled channel is stepped down by `(100 * 1/N)%` of its
    own current output (`N` = channels enabled at the fault instant,
    the faulted one included) — landed EXACTLY, in a single tick,
    deliberately bypassing the hard slew-rate clamp used by every other
    output write in this codebase (added 2026-09-15 after real DSLogic
    data showed the clamped version never actually reached the literal
    percentage; see `ClampOutputRangeOnly()`'s comment in `pid.c`) — and
    the survivors then ramp on down to
    `PFM_TURNON_FREQ_HZ` over the same `FAULT_RAMP_DOWN_TIME_S` as
    `GENERAL`. If the fault hit while `IDLE`/`ARMED`, or the faulted
    channel was the only one enabled, falls through to the same
    "nothing to ramp, stop immediately" handling `GENERAL` already has
    — the faulted channel is still disabled either way.
  - **`EXTERNAL_ENABLE`** (`SM_FAULT_EXTERNAL_ENABLE`) — added
    2026-09-16, per direct request. System-wide, like `GENERAL` — reuses
    its exact ramp-down response (`HandleGeneralFault()` is called
    directly), reported as its own distinct type purely so an operator
    can tell "the external-enable interlock dropped" apart from "a real
    HRTIM/gate-driver fault." Only enters this while `FIRING` — see
    `EXTernal:ENAble` below for the full interlock design (also gates
    `ARM` and `SHOT:STARt`, separately from this continuous
    FIRING-only check).

  Both of this project's existing system-wide fault sources (`PC10`/
  `HRTIM1_FLT6`, `GateDriverStatus_01..12`) still route to `GENERAL`
  unconditionally — nothing about `OVERCURRENT`/`EXTERNAL_ENABLE`
  changes that. Left only via `FAULT:CLEAR`, always back to `IDLE` —
  never directly to `ARMED`/`FIRING`.

```
> STATE?
< OK IDLE
> ARM
< OK
> STATE?
< OK ARMED
> SHOT:STARt
< OK
> STATE?
< OK FIRING
  ... (shot runs, completes on its own) ...
> STATE?
< OK IDLE
> SHOT:STARt
< ERR 13 Must ARM first -- see the ARM command
  ... (a fault trips, from any state) ...
> STATE?
< OK FAULT GENERAL
> FAULT:CLEAR
< OK
> STATE?
< OK IDLE
```

- **`ARM`** — `OK`, `IDLE` → `ARMED`. `ERR 13` if not currently `IDLE`,
  or if `ArmConditionsMet()` refuses -- as of 2026-09-17 this is a real
  check, not a stub: the external-enable interlock (`EXTernal:ENAble`,
  PF13) must be satisfied, and every currently-enabled channel's own
  `XRn_ENA_OUT`/`XRn_CONTACT_OUT` must both be HIGH (see
  `XREX:CHANnel:ENAOut`/`CONTactOut` below).
- **`DISARM`** — `OK`, `ARMED` → `IDLE`. No-op (still `OK`, not an
  error) if not currently `ARMED`.
- **`STATE?`** — `OK <IDLE|ARMED|FIRING>`, or while faulted:
  `OK FAULT GENERAL` / `OK FAULT OVERCURRENT <ch>` (1-based) /
  `OK FAULT EXTERNAL_ENABLE` /
  `OK FAULT ENABLE_OUTPUT <ch>` (1-based) / `OK FAULT ENERPRO <ch>`
  (1-based — added 2026-09-18, reclassified out of `GENERAL`; see
  below).
- New error code **13**: an invalid state-machine transition for the
  current state (e.g. `SHOT:STARt` sent while not `ARMED`;
  `ARM` sent while not `IDLE`).

**Resolved 2026-09-22**: `SOURce:RUN` (the simpler, non-profile
closed-loop start, distinct from `SHOT:STARt`) is now gated by
the state machine too — it requires `ARMED` (`ERR 13`) and the
`EXTernal:ENAble` interlock (`ERR 15`), then transitions `ARMED` →
`FIRING` via the new `SM_StartPlain()` (state_machine.c/.h). It has no
shot clock, so `SOURce:STOP` (`SM_Stop()`) is what returns it to `IDLE`.

**`ARM`'s failure reporting, reworked 2026-09-22**: see `ARM`'s own
first bullet above and, in full, the `ARM`/`DISARM`/`STATE?` prose
earlier in this section — every precondition (state, external-enable,
a specific channel's `ENA_OUT`/`CONTACT_OUT`) is now checked and
reported distinctly instead of one generic error.

### `DEBUG:FAULT:BYPASS <0|1>` / `DEBUG:FAULT:BYPASS?`

Added 2026-09-21, direct request: a **bench-only** override so a
channel can be exercised with no real fault-detect signal present —
e.g. a controller-target build with no simulator/Transrex physically
connected to drive its Water/Temp/Enerpro/OCP inputs healthy. Those
pins float with nothing wired to them, and a float reads as an instant
fault the moment a channel is enabled, blocking any test of the
DRIVE/FEEDBACK path itself in isolation.

When ON, `EnterFault()` (`state_machine.c` — the single funnel every
fault source already goes through: `SM_PollFaults()`,
`SM_Report*Fault()`) returns immediately without transitioning state,
so **nothing** behaves as faulted regardless of source —
`GENERAL`/`OVERCURRENT`/`EXTERNAL_ENABLE`/`ENABLE_OUTPUT`/`ENERPRO` all
pass through this same bypass uniformly.

**Defaults OFF (`0`) at every boot** — RAM-only, never persisted, so it
cannot silently stay enabled across a power cycle.

**⚠ Never enable this against a real Transrex or any hardware where a
real overcurrent/water/temperature condition is physically possible.**
It exists purely so this project's own two-board bench rig can validate
signal paths (like DRIVE/FEEDBACK capture) in isolation, without a
full, correctly-wired fault loop present.

```
> DEBUG:FAULT:BYPASS 1
< OK
> DEBUG:FAULT:BYPASS?
< OK 1
> DEBUG:FAULT:BYPASS 0
< OK
```

- **`DEBUG:FAULT:BYPASS <0|1>`** — `OK`. `ERR 12` if the argument is
  missing, `ERR 11` if it's neither `0` nor `1`.
- **`DEBUG:FAULT:BYPASS?`** — `OK <0|1>`, current state.

### `DIAGnostic:OPTBytes?`

**TEMPORARY diagnostic**, added while investigating whether `PB8`/`PG10`
are safe to repurpose as GPIO (see `DIAGnostic:RSTCause?` below for the
related investigation that grew out of the same question). Reports the
three option-byte-derived boot-configuration bits from `FLASH->OPTR`
(CMSIS header bit definitions, not assumed from "what the G4 family
usually does"):

```
> DIAGnostic:OPTBytes?
< OK OPTR=XXXXXXXX nBOOT0=<0|1> nSWBOOT0=<0|1> nBOOT1=<0|1>
```

- `nSWBOOT0` — `1` means boot0 is read from the physical `BOOT0`/`PB8`
  pin; `0` means boot0 is taken entirely from the `nBOOT0` option-byte
  value below and the physical pin is never sampled at boot, meaning
  `PB8` is free for GPIO use regardless of any `BOOT0`/`PB8` remap
  question.
- `nBOOT0` — the option-byte-supplied boot0 value, used only when
  `nSWBOOT0` is `0`.
- `nBOOT1` — combines with the effective boot0 value (whichever source)
  to select the final boot target (main flash / system memory / SRAM).

Does **not** itself answer whether `PB8` is safe to repurpose — that
still depends on `nSWBOOT0`'s real value once read back, and separately
on what `PG10` (this board's own `NRST`-labeled net) is actually wired
to on the schematic, which no register on this chip can reveal — only
the schematic can. Remove once the `PB8`/`PG10` GPIO-reuse question is
settled.

### Unsolicited telemetry events (`!EVT`) and `SYS:*` — added 2026-09-22

Phase 1 of `docs/telemetry.md`. The controller now emits **unsolicited**
event lines (device-initiated, not in response to a command) prefixed with
`!EVT` — a prefix that can never collide with an `OK`/`ERR` reply:

```
!EVT <tick_ms> STATE <IDLE|ARMED|FIRING|FAULT>
!EVT <tick_ms> FAULT <GENERAL|OVERCURRENT|EXTERNAL_ENABLE|ENABLE_OUTPUT|ENERPRO> [ch]
!EVT <tick_ms> FAULT CLEAR
!EVT <tick_ms> TRIGGER FIRING
```

`<tick_ms>` is the monotonic millisecond clock (`SYS:TIME?`); `[ch]` is
1-based and only present for the per-channel fault types (`OVERCURRENT`,
`ENABLE_OUTPUT`, `ENERPRO`). `TRIGGER FIRING` is emitted when the external
trigger (rising edge on PF15 while `ARMED`) successfully starts a shot --
distinct from the `STATE FIRING` line so a host can tell a trigger-initiated
shot from a command-initiated one. A host should treat any line not starting
with `OK`/`ERR` as an event, and fall back to polling `STATE?`/`FAULT?` if it
reconnects mid-session (the stream is not buffered across reconnect).

- **`SYS:TIME?`** — `OK <ms-since-boot>` — monotonic 1 ms counter
  (`HAL_GetTick()`); wraps at ~49.7 days.
- **`SYS:TELEM?`** — `OK <schema_version>` — the telemetry data-contract
  version (`TELEMETRY_SCHEMA_VERSION`, currently `3`); bumped on any
  wire-format change so a host can detect/adapt.
- **`SYS:EVENT <0|1>`** — `OK` — gate the *live* `!EVT` stream (`1` = on, the
  default; `0` = off). Gating only silences the live stream — the flight
  recorder (below) keeps recording, so nothing is lost for post-mortem.
- **`SYS:EVENT?`** — `OK <0|1>`.
- **`SYS:EVLOG?`** — the **flight recorder** (docs/telemetry.md Phase 4):
  replays the retained event history since boot, oldest first, as a header
  `OK <n>` followed by `n` one-line `!EVT` packets (same text format as the
  live stream). RAM-only, so it clears on reset. Example:

  ```
  > SYS:EVLOG?
  < OK 4
  < !EVT 141151 STATE ARMED
  < !EVT 141473 STATE FIRING
  < !EVT 141784 FAULT GENERAL
  < !EVT 142097 FAULT CLEAR
  ```

  Unlike the live stream, this is always recorded — `SYS:EVENT 0` does not
  empty it.

The `!EVT` lines are emitted from the main loop (`Telemetry_PollEmit()`),
bounded to a few per iteration, and never from ISR context — they cannot
disturb the PID loop or fault handling (see `docs/telemetry.md` §6).

### `EXTernal:ENAble` / `EXTernal:ENAble?` / `EXTernal:INPut?`

Added 2026-09-16, per direct request: an external operator/facility
permissive interlock. **MOVED 2026-09-17 from PF15 to PF13**
(`docs/pin_mapping_v4.csv` — documented `GPInput_12`, confirmed `GPI`,
unused elsewhere), per direct instruction: enable and trigger are now
independent physical signals on separate pins, not one shared wire —
PF15 keeps *only* the trigger role from here on (see
`EXTernal:TRIGger` below). Every behavior/gating rule below is
otherwise unchanged from the original design; only the pin moved.
(PC14, first proposed for this feature back on 2026-09-16, was
rejected — it's documented `GPO`, `STM_Enable_Pin`, meaning the STM32
drives that one outward, the opposite direction needed here.)

**ON by default as of 2026-09-22** (direct instruction — these are
meant to be usable-by-default features, not something an operator has
to remember to opt into every session; was opt-in/OFF by default from
2026-09-16 through 2026-09-21). RAM-only config (like
`SHOT:CURRent`/`PID:LOOPMODE`/etc.) — resets to ON on every
reboot, not persisted; send `EXTernal:ENAble 0` to opt back out for a
session, e.g. bench testing with nothing physically wired to PF13.

**Real consequence of the ON default, not a paper change**: PF13 is
`GPIO_PULLDOWN` (see below), so a floating/unwired PF13 now reads LOW
= "not satisfied" out of the box — `ARM` and `SHOT:STARt` will
both refuse (`ERR 13`/`ERR 15`) on any bench setup that doesn't have
the real interlock wired, looped back (`DIAGnostic:GPOut11` → PF13,
below), or `EXTernal:ENAble 0` sent first.

When ON:
1. **`ARM`** refuses (folded into its existing generic `ERR 13`) unless
   PF13 currently reads HIGH.
2. **`SHOT:STARt`** ALSO re-checks PF13 immediately before firing
   — closes the real gap where PF13 could drop in the window between a
   successful `ARM` and the eventual `START` (`ARM` alone does not
   guarantee this at the moment of firing). `ERR 15` if refused. This
   re-check also runs when a shot is started via `EXTernal:TRIGger`'s
   rising edge (below), since both paths call the same `SM_Fire()`.
3. While **`FIRING`**, checked continuously (same ~1kHz cadence General
   Fault's own two hardware sources get) — if PF13 drops, enters
   `SM_STATE_FAULT` with `EXTERNAL_ENABLE` (see the `FAULT` state's own
   entry above for the identical-to-`GENERAL` ramp-down response).
   Deliberately **not** actively monitored while merely `IDLE`/`ARMED`
   (nothing outputting yet to protect) — losing PF13 there just means
   the next `SHOT:STARt` attempt fails its own re-check (item 2)
   instead of entering `FAULT`. Not decided either way whether `ARMED`
   should also actively fault on loss — not asked for, flagged rather
   than silently added.
4. **`FAULT:CLEAR`** re-validates PF13 is back HIGH before actually
   clearing an `EXTERNAL_ENABLE` fault — same "only clear if the
   condition is actually gone" treatment `PC10`/`GateDriverStatus`
   already get.

PF13 is configured `GPIO_PULLDOWN` (`main.c`'s `MX_GPIO_Init()`) —
**not** this project's usual `GPIO_NOPULL` for actively-driven inputs
(`GateDriverStatus`, `PFM_Input`, `QUADSPI`). Deliberate: an
unconnected/floating PF13 must read LOW (no permission), never an
undefined level that could accidentally read HIGH and silently permit
firing — the opposite safety direction from `GateDriverStatus`, where
floating-reads-as-fault is already the safe outcome.

```
> EXTernal:ENAble 1
< OK
> EXTernal:ENAble?
< OK 1
> EXTernal:INPut?
< OK 0
> ARM
< ERR 13 Can't ARM -- not currently IDLE, or arm conditions not met
  ... (PF13 goes HIGH) ...
> ARM
< OK
> SHOT:STARt
< OK
  ... (PF13 drops while FIRING) ...
> STATE?
< OK FAULT EXTERNAL_ENABLE
> FAULT:CLEAR
< ERR ...   (still LOW -- refused)
  ... (PF13 restored HIGH) ...
> FAULT:CLEAR
< OK
```

- **`EXTernal:ENAble <0|1>`** — `OK`, turns the interlock on/off.
  `ERR 12` if the argument is missing.
- **`EXTernal:ENAble?`** — `OK <0|1>`, current mode.
- **`EXTernal:INPut?`** — `OK <0|1>`, PF13's raw logic level right now,
  independent of whether the interlock is even turned on — lets an
  operator confirm real wiring/signal presence before relying on it,
  the same diagnostic role `PFMIN:DEBUG:RAW?`/`PFMIN:DEBUG:REG?` played
  for the `PFM_Input` fiber-patching investigation below.
- New error code **15**: `SHOT:STARt` refused because the
  interlock is on and PF13 currently reads LOW.

### `EXTernal:TRIGger` / `EXTernal:TRIGger?` / `EXTernal:TRIGger:INPut?`

Added 2026-09-16, per direct follow-up request: a **rising edge on
PF15** (`Fiber_Enable`) fires a shot while `ARMED` — and *only* from
`ARMED` — exactly as if `SHOT:STARt` had been sent manually.
**RESTRUCTURED 2026-09-17**, per direct instruction: PF15 previously
also carried the `EXTernal:ENAble` role (above); that role moved to
its own separate pin (PF13), so PF15 now backs trigger exclusively.
The old structural dependency — `EXTernal:TRIGger 1` refusing unless
`EXTernal:ENAble` was already on — is **gone**: the two features are
independently configurable now that they're separate physical signals.
The real safety guarantee is unaffected either way — `SM_Fire()` (the
same function both `SHOT:STARt` and this trigger call)
unconditionally re-checks the enable interlock (PF13) every time,
regardless of how or when trigger was turned on.

**ON by default as of 2026-09-22** (direct instruction, same reasoning
and same date as `EXTernal:ENAble` above; was opt-in/OFF 2026-09-16
through 2026-09-21). Lower practical impact than the enable default
above: this only means a PF15 rising edge fires an already-`ARMED`
shot — it never blocks anything by itself. A floating/unwired PF15 (also
`GPIO_PULLDOWN`) just never produces a rising edge, so a bench setup
with nothing physically wired to PF15 sees no behavior change from this
default alone. Send `EXTernal:TRIGger 0` to opt back out for a session.

There is no separate "open-loop start call" to invoke here: open- vs.
closed-loop has always been the **per-channel** `PID:LOOPMODE` flag,
checked inside the same control loop both `SOURce:RUN` and
`SHOT:STARt` already share — not a different start mechanism.
`PID:LOOPMODE 0 <0|1>` (above, also added 2026-09-16) is a plain
convenience for setting every channel's mode at once before an
externally-triggered shot, not something this feature reads or
branches on itself.

Edge-triggered, not level-triggered: a baseline PF15 level is captured
fresh the instant `ARM` succeeds, so a signal already HIGH at the
moment of arming does **not** look like a rising edge on the next
check — only a genuine LOW→HIGH transition after arming fires. If the
resulting `SM_Fire()` call itself fails for some other reason (e.g.
profile timing never set, or the enable interlock isn't currently
satisfied), the state simply stays `ARMED` — a fresh falling-then-
rising edge is needed to try again, not just PF15 remaining HIGH.

```
> EXTernal:TRIGger 1
< OK
> ARM
< OK
  ... (PF15 rises) ...
> STATE?
< OK FIRING
```

- **`EXTernal:TRIGger <0|1>`** — `OK`, turns the trigger feature
  on/off. `ERR 12` if the argument is missing. No longer coupled to
  `EXTernal:ENAble` as of 2026-09-17 — always succeeds.
- **`EXTernal:TRIGger?`** — `OK <0|1>`, current mode.
- **`EXTernal:TRIGger:INPut?`** — `OK <0|1>`, PF15's raw logic level
  right now, independent of whether the trigger feature is even turned
  on. Added 2026-09-17 alongside the PF13/PF15 split — previously
  `EXTernal:INPut?` covered this same pin (it backed both enable and
  trigger, being the same wire); now that they're separate, this is
  trigger's own dedicated diagnostic.
- Error code **16** is **retired** (no longer generated) — previously
  `EXTernal:TRIGger` refused because `EXTernal:ENAble` wasn't on first,
  a rule that only existed because both features read the same wire.
  Kept in the error-code table, not reassigned, per this project's
  "never renumber/reuse" convention.

**CONFIRMED ON REAL HARDWARE, 2026-09-16** (original PF15-for-both
design, full writeup and exact numbers in `docs/changelog.txt` — `ARM`
gating, `SHOT:STARt`'s own re-check, a real FIRING-time drop
entering `FAULT EXTERNAL_ENABLE` with a clean ramp-down (16400 Hz →
floor over ~1.0s), and `FAULT:CLEAR` correctly refusing then
succeeding). **Re-confirmed 2026-09-17, post-split, for TRIGGER
specifically**: edge-triggering (not spurious on an already-HIGH
baseline, not re-firing while held HIGH), `ARMED`→`FIRING` on a genuine
rising edge, and firing succeeding with the enable interlock
(`EXTernal:ENAble`) never turned on at all — direct confirmation the
decoupling works end-to-end, not just at the command-response level.
**Not yet re-verified against PF13 specifically post-split**: the
`ARM`/`SHOT:STARt` gating, the FIRING-time fault entry, and
`FAULT:CLEAR`'s re-validation — only `EXTernal:INPut?`'s raw-read
default (floating, reads `0`) has been checked on PF13 so far, since
nothing is wired to it on the bench yet. See
`pending-hardware-calibration.md` for the itemized open-verification
list.

### `DIAGnostic:GPOut12` / `DIAGnostic:GPOut12?`

Added 2026-09-16, per direct request: a generic, software-driven
diagnostic output on **PD1** (`GPOut_12` in the V4 column,
`docs/pin_mapping_v4.csv` — confirmed `GPO` there; PF13 was proposed
as this OUTPUT pin's own identity first and corrected — it's actually
documented `GPInput_12`, an input, in the same CSV; PF13 later became
a real pin in this project anyway, but as the external-enable INPUT,
not this diagnostic output).
Built specifically to test `EXTernal:ENAble`/`EXTernal:TRIGger` above
without needing a hand-operated bench jumper: loop this pin to PF15
and drive it entirely from the serial console with precise, repeatable
timing. As of 2026-09-17's PF13/PF15 split, this loop exercises
`EXTernal:TRIGger` specifically (PF15's current role) — `DIAGnostic:GPOut11`/PD0
(below) is the equivalent pin for testing signals wired elsewhere.
Not tied to that use case in the pin config itself — a plain
level output, reusable for any future diagnostic that needs one.

```
> DIAGnostic:GPOut12 1
< OK
> DIAGnostic:GPOut12?
< OK 1
```

- **`DIAGnostic:GPOut12 <0|1>`** — `OK`, drives PD1 HIGH/LOW. `ERR 12`
  if the argument is missing.
- **`DIAGnostic:GPOut12?`** — `OK <0|1>`, the pin's current level
  (read back via `HAL_GPIO_ReadPin()`, reflecting the real driven
  state).

### `DIAGnostic:GPOut11` / `DIAGnostic:GPOut11?`

Added 2026-09-17 — a **second, independent** diagnostic output on
**PD0** (`GPOut_11` in the V4 column, `docs/pin_mapping_v4.csv` —
confirmed `GPO`), an exact mirror of `DIAGnostic:GPOut12`/PD1 above in
every respect (generic push-pull level output, GPIO config in
`main.c`, initial state LOW before enable). Added after PD1 was found
double-used for testing two different real inputs (PF15 and, at the
time, PG10) — this gives each test loop its own genuinely separate
diagnostic pin, removing any ambiguity about which diagnostic signal
is driving which real input.

```
> DIAGnostic:GPOut11 1
< OK
> DIAGnostic:GPOut11?
< OK 1
```

- **`DIAGnostic:GPOut11 <0|1>`** — `OK`, drives PD0 HIGH/LOW. `ERR 12`
  if the argument is missing.
- **`DIAGnostic:GPOut11?`** — `OK <0|1>`, the pin's current level.

### `DIAGnostic:GPOut09` / `DIAGnostic:GPOut09?`, `DIAGnostic:GPOut10` / `DIAGnostic:GPOut10?`

Added 2026-09-18 — a **third and fourth**, independent diagnostic
output pair on **PG8** (`GPOut_09` in the V4 column,
`docs/pin_mapping_v4.csv` — confirmed `GPO`; note the V3 column for
this same row is `No connection`, and a *different* pair of pins,
PD8/PD9, carried the `GPOut_09`/`GPOut_10` names under V3 — verified
directly against the CSV before adding these, to avoid repeating the
earlier PC14-vs-PF15/PF13-vs-PD1 V3/V4 name-reuse mistakes) and **PG9**
(`GPOut_10`). Exact mirrors of `DIAGnostic:GPOut11`/`GPOut12` above in
every respect. Added to close the last gap in the Transrex simulator's
fiber-transmitter budget (`docs/pin_mapping_reference.tex` Section 7)
— these two feed XR1_OCP/XR2_OCP on the controller (`GPInput_03`/PF4
and `GPInput_07`/PF8 respectively), completing OCP fault-injection
coverage for all 4 channels (XR3/XR4 OCP were already covered by
`DIAGnostic:GPOut11`/`GPOut12`).

```
> DIAGnostic:GPOut09 1
< OK
> DIAGnostic:GPOut09?
< OK 1
> DIAGnostic:GPOut10 1
< OK
> DIAGnostic:GPOut10?
< OK 1
```

- **`DIAGnostic:GPOut09 <0|1>`** — `OK`, drives PG8 HIGH/LOW. `ERR 12`
  if the argument is missing.
- **`DIAGnostic:GPOut09?`** — `OK <0|1>`, the pin's current level.
- **`DIAGnostic:GPOut10 <0|1>`** — `OK`, drives PG9 HIGH/LOW. `ERR 12`
  if the argument is missing.
- **`DIAGnostic:GPOut10?`** — `OK <0|1>`, the pin's current level.

### `DIAGnostic:RSTCause?` / `DIAGnostic:RSTCause:CLEar`

**TEMPORARY diagnostic**, added 2026-09-17 while investigating a
garbled response from `DIAGnostic:GPOut11 1` during E-stop testing
(the investigation that ultimately found PG10 wired to this MCU's real
`NRST` net — see `docs/changelog.txt`'s 2026-09-17 entry). Reads/clears
the real `RCC->CSR` reset-cause flags, which is what actually proved
the glitch was a genuine hardware `NRST`-pin assertion (`PIN=1`) and
not a brown-out (`BOR=0`) or firmware corruption. Not specific to that
investigation — useful any time a mystery reset needs a real answer
instead of a guess.

```
> DIAGnostic:RSTCause:CLEar
< OK
> DIAGnostic:RSTCause?
< OK CSR=00000000 BOR=0 PIN=0 SFT=0 IWDG=0 WWDG=0 LPWR=0 OBL=0
```

- **`DIAGnostic:RSTCause?`** — `OK CSR=<hex> BOR=<0|1> PIN=<0|1>
  SFT=<0|1> IWDG=<0|1> WWDG=<0|1> LPWR=<0|1> OBL=<0|1>` — the raw
  register plus each individual reset-cause flag
  (`RCC_CSR_BORRSTF`/`PINRSTF`/`SFTRSTF`/`IWDGRSTF`/`WWDGRSTF`/
  `LPWRRSTF`/`OBLRSTF`, `stm32g474xx.h`'s own bit definitions).
- **`DIAGnostic:RSTCause:CLEar`** — `OK`, clears every flag above via
  `RCC_CSR_RMVF` — useful for isolating whether a *specific* action
  (not just "since the last reboot") actually triggers a new reset.
- **Remove once the underlying investigation this was built for is
  fully settled** — kept for now since it's cheap and general-purpose
  enough to be useful again.

### `GPOut:ENAble` / `GPOut:ENAble?`

Added 2026-09-17, per direct request: **PC13** (`GPOut_Enable_Pin` in
`docs/pin_mapping_v4.csv`'s V4 column — confirmed `GPO` there, unused
elsewhere), **default HIGH at boot** (`main.c`, driven HIGH before
`HAL_GPIO_Init()` enables it — the opposite default of every other
software-driven output in this project, all deliberately LOW by
default; a fresh boot must present this pin's real intended default,
not an incidental LOW that happens to match the others). Own top-level
`GPOut:` namespace (not nested under `DIAGnostic:`) since this is a
real, specifically-named board signal from the schematic, not a
generic scratch diagnostic pin — plain `HAL_GPIO_WritePin()`/
`ReadPin()` pair, no dedicated module, same "not enough behavior to
justify one" precedent as `DIAGnostic:GPOut11`/`GPOut12`.

```
> GPOut:ENAble?
< OK 1
> GPOut:ENAble 0
< OK
> GPOut:ENAble?
< OK 0
```

- **`GPOut:ENAble <0|1>`** — `OK`, drives PC13 HIGH/LOW. `ERR 12` if
  the argument is missing.
- **`GPOut:ENAble?`** — `OK <0|1>`, the pin's current driven level.

**Verified on real hardware, 2026-09-17**: defaults `1` (HIGH) fresh
off a reflash, toggles correctly both directions.

### `PWMAlt:ENAble` / `PWMAlt:ENAble?`

Added 2026-09-17, per direct request: **PC15** (`PWM_Alt_Enable` in
`docs/pin_mapping_v4.csv`'s V4 column — confirmed `GPO` there, unused
elsewhere) — exact mirror of `GPOut:ENAble` above in every respect
(default HIGH at boot, own top-level namespace, no dedicated module).

```
> PWMAlt:ENAble?
< OK 1
> PWMAlt:ENAble 0
< OK
> PWMAlt:ENAble?
< OK 0
```

- **`PWMAlt:ENAble <0|1>`** — `OK`, drives PC15 HIGH/LOW. `ERR 12` if
  the argument is missing.
- **`PWMAlt:ENAble?`** — `OK <0|1>`, the pin's current driven level.

**Verified on real hardware, 2026-09-17**: defaults `1` (HIGH) fresh
off a reflash, toggles correctly both directions.

### `OCP:TEST:FAULT`

Added 2026-09-15 — **TEMPORARY** software fault injection for
`SM_FAULT_OVERCURRENT` (above), matching this project's established
temporary-debug-command precedent (`PFMIN:DMASTAT?`, `qspi_test.c`):
exists so the new per-channel OCP behavior can be exercised end-to-end
on real hardware, since no real OCP fault pin is wired up yet. Calls
`SM_ReportOcpFault(ch - 1)` directly — exactly what a real OCP pin's
own (not-yet-written) interrupt handler will eventually call.

```
> OCP:TEST:FAULT 2
< OK
> STATE?
< OK FAULT OVERCURRENT 2
> SOURce:ENAble? 2
< OK 0
> FAULT:CLEAR
< OK
> STATE?
< OK IDLE
```

- **`OCP:TEST:FAULT <ch>`** — `OK`, triggers an OVERCURRENT fault for
  channel `<ch>` (1-based) exactly as described above. `ERR 11` for an
  out-of-range channel, `ERR 12` for a missing argument. Not gated by
  the operator consoles' dangerous-command confirmation — triggering a
  fault only ever stops/reduces output, the same "safe direction, never
  gated" treatment `FAULT:CLEAR`/`SOURce:STOP` already get. Remove once
  real OCP hardware detection exists and has its own real trigger path.

### `GENERAL:TEST:FAULT`

Added 2026-09-15 — **TEMPORARY**, same precedent as `OCP:TEST:FAULT`
above: software fault injection for `SM_FAULT_GENERAL`
(`SM_ReportGeneralFault()`, state_machine.h), so General Fault's
ramp-down can be exercised at a moment of the operator's own choosing
(e.g. mid-ramp-up, not just steady-state flat-top) without needing to
actually trip a real PC10/HRTIM1_FLT6 or GateDriverStatus condition. No
channel argument — General Fault is system-wide.

```
> GENERAL:TEST:FAULT
< OK
> STATE?
< OK FAULT GENERAL
> FAULT:CLEAR
< OK
> STATE?
< OK IDLE
```

- **`GENERAL:TEST:FAULT`** — `OK`, triggers a GENERAL fault exactly as
  described above. Not gated by the operator consoles' dangerous-command
  confirmation, same reasoning as `OCP:TEST:FAULT`. Remove once real
  fault triggers are otherwise well-exercised enough that this is no
  longer useful.

### `PFMIN:DEBUG:RAW?` / `PFMIN:DEBUG:REG?`

Added 2026-09-15 — **TEMPORARY**, diagnostic-only, built while tracking
down why `measuredHz` (`SOURce:STATus?`) read 0 (or, later, a swapped
channel's frequency) for some WHAM channels despite confirmed-correct
real HRTIM output. Root cause turned out to be a physical fiber-optic
patching mix-up on the bench (which physical `PFM_Input_0X` receiver
port each phase's fiber was plugged into), not a firmware bug — these
two commands were how that was actually found: frequency-fingerprint
each of the 6 physical ports (each phase driven at a distinct,
easily-recognizable frequency) to see which port is really receiving
which phase's signal, independent of which WHAM channel firmware
normally associates with which port.

- **`PFMIN:DEBUG:RAW? <ch>`** (`ch` = 1-6, all 6 physical `PFM_Input`
  ports, not just the 4 WHAM channels) — `OK cont=<0|1> run=<0|1>
  firstRise=<0|1> avgCount=<n> lastPeriod=<ticks> overcap=<n>` — a
  non-destructive read of `pfm_input.c`'s internal continuous-mode
  accumulator state (unlike `SOURce:STATus?`'s own consumption of the same
  data, this does NOT reset `avgCount` — repeated polling can watch it
  accumulate, or not, live during a shot). `lastPeriod` in raw 170 MHz
  ticks — convert to Hz as `170000000 / lastPeriod`.
- **`PFMIN:DEBUG:REG? <ch>`** (`ch` = 1-6) — `OK CR1=.. CCER=.. DIER=..
  SR=.. CNT=.. CCR=.. CCMR1=.. MODER=.. AFR=.. IDR=..` — raw TIMx
  peripheral + GPIO register readback for that channel's underlying
  timer/pin, bypassing every software layer. `IDR` is the pin's live
  logic level sampled directly, no software interpretation at all.
- `ERR 8` for an out-of-range channel (1-6, reusing `PFMIN:DATA?`'s own
  code), `ERR 12` for a missing argument.

Remove once the fiber-patching investigation is fully closed and these
are no longer needed for verifying the bench's own physical setup — see
`docs/changelog.txt`'s 2026-09-15 entry for the full investigation
writeup.

### `CONFig:CHANnels?`

Reports `HRTIM_NUM_CHANNELS` (`Core/Inc/ctrlr_config.h`) — the
compile-time HRTIM channel count this specific firmware build was
configured for (1-5; see that file for why not 6). Added 2026-09-08
alongside the channel-count generalization specifically so host
tooling can confirm what a board actually is instead of assuming a
value that could silently drift after a rebuild with a different
channel count.

```
> CONFIG:CHANNELS?
< OK 3
```

Not modifiable at runtime — there is no corresponding `SET` command,
by design. Changing it means editing `ctrlr_config.h`, rebuilding, and
reflashing.

### `CONFig:*` runtime-configurable calibration/limits (added 2026-09-22)

Direct request: eight `ctrlr_config.h` compile-time constants made
runtime-configurable. **None of these persist** — every one resets to
its `ctrlr_config.h` compile-time default on the next reboot/reflash,
same session-only convention as `PID:GAINS`/`SHOT:TIMing`/etc.
Available on both build targets unless noted otherwise.

- **`CONFig:PIDRate <hz>` / `?`** — `PID_LOOP_RATE_HZ`. Live-reprograms
  the real HRTIM Master timebase register (`HRTIM1_SetPidHeartbeatRate()`,
  `hrtim.c`). Range-checked to `[700, 10000]` Hz (raised from `500` on
  2026-09-23 — `500` was actually below the ~649 Hz real hardware floor,
  so every value in `[500, 648]` looked valid but always silently
  failed for an unrelated reason) and **refuses while a shot is
  running** (`ERR 12`) — unlike every other setting here, a mid-shot
  change would corrupt every running channel's integral/slew/
  profile-tick state, which all implicitly assume a constant `dt`.
  **Also affects every other command whose own stored value is
  tick-based** (`SHOT:TIMing`, `SOURce:RAMP`, and `LOG:DATA?`'s
  reported sample rate) — changing the loop rate after those were set
  but before the shot actually fires changes the real wall-clock
  duration those ticks now represent (found and fixed 2026-09-23, see
  `docs/changelog.txt`).
- **`CONFig:TURNONHz <hz>` / `?`**, **`CONFig:MAXFREQHz <hz>` / `?`** —
  `PFM_TURNON_FREQ_HZ`/`PFM_MAX_FREQ_HZ`, the Amps↔Hz calibration
  endpoints (`AmpsToHz()`, `pid.c`). Cross-validated against each other
  (turnon strictly below max) and against `[PID_OUTPUT_MIN_HZ,
  PID_OUTPUT_MAX_HZ]`.
- **`CONFig:MAXCURRent <ch> <amps>` / `<ch>`** — per-channel
  `PFM_MAX_CURRENT_A_PER_CHANNEL[ch]`, 1-based channel. This is the
  real per-channel calibration `ctrlr_config.h`'s own "MUST BE
  CALIBRATED BEFORE FINAL DEPLOYMENT" comment describes — now settable
  over serial instead of requiring a rebuild+reflash. `amps` must be
  `> 0`.
- **`CONFig:SLEWRate <hzPerTick>` / `?`** — `PID_OUTPUT_MAX_SLEW_HZ_PER_TICK`,
  the hard per-tick output-glitch clamp (`ClampOutputSlew()`, `pid.c`).
  Only enforced `> 0` — no upper bound, so it's the operator's own
  responsibility not to configure this so loose it stops meaningfully
  protecting downstream hardware.
- **`CONFig:FaultRampTime <s>` / `?`** — `FAULT_RAMP_DOWN_TIME_S`. Must
  be `> 0`. Only takes effect on the **next** fault, not one already in
  progress.
- **`CONFig:FaultPolarity:WATER <0|1>` / `?`**,
  **`:TEMP`**, **`:ENERPRO`**, **`:OCP`** — the four independent
  `XR_*_FLT_POLARITY` constants (`xrex_io.c`'s fault-detection logic).
  `0` = `FAULT_POLARITY_NORMALLY_HIGH`, `1` = `FAULT_POLARITY_NORMALLY_LOW`
  (same encoding as the constants themselves) — any other value is
  `ERR 12`. Four separate commands, not one with a category argument,
  matching `SIM:FAULT:*`'s existing per-category convention and the
  fact these really are four independent settings on real hardware.
- **`CONFig:MaxCarrierHz <hz>` / `?`** — `PFM_MAX_CARRIER_FREQ_HZ`, the
  legacy `TABle:STEP` carrier ceiling (`ERR 10`'s own trigger). Only
  affects the legacy `TABle:*`/`FIRE` path (Section reference:
  `docs/sop/wham_xrex_pfmg474_sop.tex`'s Section 8) — has no effect on
  the modern `SHOT:*` path, which has its own separate,
  already-configurable output bounds above. Rejected (`ERR 12`) if
  `hz` would over/underflow the 16-bit HRTIM `PER` register at this
  board's fixed clock (roughly `hz >= 2595`).

```
> CONFig:PIDRate?
< OK 1000
> CONFig:TURNONHz 4800
< OK
> CONFig:FaultPolarity:OCP 1
< OK
> CONFig:FaultPolarity:OCP?
< OK 1
```

**Host-side staleness, RESOLVED 2026-09-22:** `python/wham_console.py`
and `python/run_simulator_validation.py` used to hardcode their own
Python copies of `PFM_TURNON_FREQ_HZ`/`PFM_MAX_FREQ_HZ`/
`PFM_MAX_CURRENT_A` for host-side Amps↔Hz plotting conversions
(`hz_to_amps()`/the new `amps_to_hz()`), with no way to notice a live
change made via the commands above. `wham_console.py` now has
`sync_calibration(link)`, which queries `CONFig:TURNONHz?`/
`MAXFREQHz?`/`MAXCURRent? 1` and refreshes those globals — called
automatically on every connect and after any raw `CONFig:TURNONHz`/
`MAXFREQHz`/`MAXCURRent` typed directly in the console.
`run_simulator_validation.py` imports the module (`import wham_console
as wc`) rather than freezing a copy at import time, so it tracks the
same live sync. Still a flat, single scalar per value (not per
channel, even though `CONFig:MAXCURRent` really is per-channel on the
firmware side) — a pre-existing simplification this fix didn't change,
only the staleness. `PID_LOOP_RATE_HZ` in `run_simulator_validation.py`
has the same kind of gap now that `CONFig:PIDRate` exists (used for
`LOG:ARM`/`LOG:DATA?` sample-rate math) — flagged in that file's own
comment, not yet fixed.

### `GDS?`

Raw `HIGH`/`LOW` snapshot of the 12 GateDriverStatus_01..12 pins
(`PE0`–`PE11`, `GPIOE` — `docs/pin_mapping_v4.csv`), added 2026-09-08
as a diagnostic while investigating why a fault wasn't being
registered. Direct, uncached `GPIOE->IDR` read at the moment of the
query — no debounce, no polarity interpretation, no fault-latching of
its own. This command itself is read-only visibility and cannot stop
PWM output by itself. (These same 12 pins DO now gate PWM output, via
a separate mechanism — the GateDriverStatus EXTI fault interrupt, see
`FAULT?`/`FAULT:CLEAR` below, added the same day — `GDS?` is
unaffected by and independent of that mechanism either way, just a raw
snapshot.)

```
> GDS?
< OK 01=LOW 02=LOW 03=HIGH 04=LOW 05=LOW 06=LOW 07=LOW 08=LOW 09=LOW 10=LOW 11=LOW 12=LOW
```

One `OK` line, one `NN=HIGH` or `NN=LOW` token per pin, space
separated, `NN` = `01`–`12` matching the `GateDriverStatus_01`..`_12`
numbering (`01` = `PE0`, `12` = `PE11`). Kept to this project's
existing single-`OK <value>`-line response convention rather than the
sibling PFM-STM32G474 project's multi-line/bitmask `GDS?` reply
shapes.

### `XREX:CHANnel:STATus?`

Added 2026-09-17 alongside `Core/Src/xrex_io.c` — a per-Transrex-
channel diagnostic readback, reporting one channel's `WATER`/`TMP`/
`ENERPRO`/`OCP` fault pins together, by name, rather than needing to
remember which of `GDS?`'s 12 raw pins (or the 4 new OCP pins) maps to
which signal for a given Transrex. `docs/pin_mapping_v4.csv`'s new
"XREX Pin Name" column is the source of this naming: `XR1`-`XR4` are
Transrex 1-4, and each has its own `_WATER_FLT`/`_TMP_FLT`/
`_ENERPRO_FLT`/`_OCP_FLT` pin (Water/Temperature/Enerpro/overcurrent
faults respectively).

```
> XREX:CHANnel:STATus? 1
< OK WATER=LOW TMP=LOW ENERPRO=LOW OCP=LOW
```

(Real captured output, 2026-09-17 — all four pins floating/unconnected
on the bench at the time, hence all `LOW`; see the polarity note below
for why `LOW` here means "faulted" under this project's default
config, not "healthy.")

1-based channel argument (`1`-`HRTIM_NUM_CHANNELS`, matching this
project's universal wire convention), `ERR 12` if missing, `ERR 11` if
out of range. Reports raw `HIGH`/`LOW` pin levels only, **polarity-
agnostic** — same convention as `GDS?`/`EXTernal:INPut?` —
`ctrlr_config.h`'s `XR_WATER_FLT_POLARITY`/
`XR_TMP_FLT_POLARITY`/`XR_ENERPRO_FLT_POLARITY`/`XR_OCP_FLT_POLARITY`
(below) are what decide which level actually means "faulted," not this
command. `WATER`/`TMP`/`ENERPRO` come from `GateDriver_Read()`'s
existing `PE0`-`PE11` snapshot (same pins `GDS?` reports, just
regrouped and renamed per-channel); `OCP` is a direct, uncached read of
that channel's own `PF4`/`PF5`/`PF8`/`PF12` pin (see below) — neither
read is debounced or latched, matching `GDS?`'s own philosophy.

#### Fault architecture (Water/Temp/Enerpro/OCP), 2026-09-17

Real per-channel fault detection for four new/reinterpreted signal
categories, added per direct request against `pin_mapping_v4.csv`'s new
"XREX Pin Name" column:

| Category | XR1 | XR2 | XR3 | XR4 | Mechanism |
|---|---|---|---|---|---|
| `_WATER_FLT` | PE0 | PE1 | PE2 | PE3 | EXTI-driven (existing `GateDriverStatus_01..04`) |
| `_TMP_FLT` | PE4 | PE5 | PE6 | PE7 | EXTI-driven (existing `GateDriverStatus_05..08`) |
| `_ENERPRO_FLT` | PE8 | PE9 | PE10 | PE11 | EXTI-driven (existing `GateDriverStatus_09..12`) — routes to `SM_FAULT_ENERPRO`, not `GENERAL`, as of 2026-09-18 |
| `_OCP_FLT` | PF4 | PF8 | PF12 | PF5 | **Polled** (new pins, no EXTI capacity free — see below) |

- **Water/Temp are "General Fault"** (`SM_FAULT_GENERAL`, `STATE?`) —
  the existing `GateDriverStatus_01..12` EXTI interrupt path
  (`gate_driver.c`) now delegates its fault DECISION to
  `XrexIo_EvaluateGateDriverFault()` (`xrex_io.c`, Water+Temp only as
  of 2026-09-18 — see below) instead of the old single shared
  `GDS_FAULT_POLARITY` — see `FAULT?`/`FAULT:CLEAR` above for the
  unchanged mechanics (force-stop/soft-stop, latch, boot-time check).
- **Enerpro is its own fault type**, like OCP (`SM_FAULT_ENERPRO`,
  `STATE?`) — **RECLASSIFIED 2026-09-18** out of `SM_FAULT_GENERAL`,
  after `docs/Transrex/Transrex_Controls_Upgrade (1).pdf`'s own fault
  table was found to give Enerpro the SAME response as Overcurrent
  (reduce surviving channels' `DEMAND`, keep running —
  `HandleOvercurrentFault()`), not Water/Temp's full stop. Detection
  stays EXTI-driven (`XrexIo_PollEnerproFaults()`, called from
  `gate_driver.c`'s `GateDriver_CheckFault()` alongside the now-
  Water+Temp-only general check, on the same raw `GateDriver_Read()`)
  — only the RESPONSE routing changed, reporting via
  `SM_ReportEnerproFault(channel)` directly instead of the shared
  latch/force-stop path.
- **OCP is its own fault type** (`SM_FAULT_OVERCURRENT`, `STATE?`),
  via `SM_ReportOcpFault(channel)` — a pre-existing entry point
  (originally added for `OCP:TEST:FAULT` software injection) now also
  driven by real hardware: `XrexIo_PollOcpFaults()`, called every
  `PID_Update()` tick and once at boot, alongside the existing
  `SM_PollFaults()` call.
- **Why OCP is polled, not EXTI-driven**: the obvious choice (matching
  Water/Temp/Enerpro) was rejected after checking the actual hardware —
  STM32's 16 EXTI lines are shared project-wide, one GPIO port per line
  number (`SYSCFG_EXTICR`). `PF4`(EXTI4)/`PF5`(EXTI5)/`PF8`(EXTI8)
  directly conflict with `PE4`/`PE5`/`PE8`, already claimed by the
  existing GateDriverStatus EXTI setup — only `PF12`(EXTI12, `XR3_OCP`)
  was actually free. Polling all 4 uniformly (rather than 3 polled + 1
  EXTI) keeps one consistent mechanism instead of mixing two.
- **Per-channel gating, the core new behavior**: a channel's own
  Water/Temp/Enerpro/OCP pins only count toward a fault when
  `PID_GetChannelEnable(channel)` is currently true (`SOURce:
  ENAble?`) — e.g. with only `XR1` enabled, a LOW (faulted) reading on
  `XR2`/`XR3`/`XR4`'s pins neither stops nor blocks output. **Verified
  on real hardware**: with all 4 channels disabled, `FAULT:CLEAR`
  reached and stayed at `STATE? OK IDLE` even with all 12
  GateDriverStatus pins and all 4 OCP pins reading `LOW` (`GDS?`/
  `XREX:CHANnel:STATus?`); enabling only channel 3 and re-polling
  correctly latched `STATE? OK FAULT OVERCURRENT 3` — the right
  channel, not 1 or a different one — confirming both the gating and
  the `PF4`/`PF8`/`PF12`/`PF5` → `XR1`-`XR4` pin mapping.
- **Polarity — `ctrlr_config.h`'s `XR_WATER_FLT_POLARITY`/
  `XR_TMP_FLT_POLARITY`/`XR_ENERPRO_FLT_POLARITY`/`XR_OCP_FLT_POLARITY`**
  (each independently `FAULT_POLARITY_NORMALLY_HIGH` or `_NORMALLY_LOW`
  -- renamed from the XREX-scoped `XREX_POLARITY_*` spelling later the
  same day, once a second, unrelated consumer of the same generic
  concept appeared (that consumer, `EMERGENCY_STOP_POLARITY`, was
  REMOVED 2026-09-22 with the E-stop feature itself); pure rename, same
  values --
  all four default to `NORMALLY_HIGH`) — **replaces** the old single
  shared `GDS_FAULT_POLARITY = GDS_NORMALLY_LOW`, which had itself been
  set from a real hardware snapshot on 2026-09-08 (11/12 pins LOW, 1
  HIGH). The new `NORMALLY_HIGH` default is the OPPOSITE of that
  confirmed value — a deliberate override per direct instruction ("During
  normal non-faulted operation, these pins will be normally high"), not
  a mistake being reintroduced. **Not yet independently re-verified
  against a real healthy (non-floating) signal on these specific
  pins** — the 2026-09-17 real-hardware check above confirmed the
  gating/mapping/fault-latching chain end-to-end, but every pin
  involved was floating (`GPIO_PULLDOWN`, nothing wired to the new
  fault-input pins yet), so it necessarily read as faulted under
  either polarity and cannot by itself confirm `NORMALLY_HIGH` is the
  physically correct choice once real Water/Temp/Enerpro/OCP signal
  sources are actually connected.
- **`_FEEDBACK`/`_DRIVE`** (`docs/pin_mapping_v4.csv`'s remaining new
  XREX Pin Name entries) are pure relabels of the existing
  `PFM_Input_01..04` feedback channels and `HRTIM1_CH*` drive outputs —
  zero functional change, no new command or behavior.
- **`XRn_ENA_OUT`/`XRn_CONTACT_OUT`** — see the dedicated
  `XREX:CHANnel:ENAOut`/`CONTactOut` section just below. Implemented
  2026-09-17 (previously deferred).

### `XREX:CHANnel:ENAOut` / `XREX:CHANnel:ENAOut?` / `XREX:CHANnel:CONTactOut` / `XREX:CHANnel:CONTactOut?`

Added 2026-09-17, per direct request: "Enable and contactor fiber
outputs need to be set by a serial command, one for each supply."
Real per-channel outputs (`PG0`-`PG3`/`PG4`-`PG7` — `XRn_ENA_OUT`/
`XRn_CONTACT_OUT`, `docs/pin_mapping_v4.csv`'s "XREX Pin Name" column)
this firmware itself drives, owned (pin table, GPIO read/write) by
`xrex_io.c`. 1-based channel argument, `ERR 11`/`ERR 12` on the usual
invalid-channel/missing-argument conditions — no new error codes for
this feature.

```
> XREX:CHANnel:ENAOut 1 1
< OK
> XREX:CHANnel:ENAOut? 1
< OK 1
> XREX:CHANnel:CONTactOut 1 1
< OK
> XREX:CHANnel:CONTactOut? 1
< OK 1
```

- **`XREX:CHANnel:ENAOut <ch> <0|1>`** / **`CONTactOut <ch> <0|1>`** —
  `OK`, sets that channel's output HIGH/LOW. Takes effect immediately;
  an operator may change either at any time, including mid-shot.
- **`XREX:CHANnel:ENAOut? <ch>`** / **`CONTactOut? <ch>`** — `OK <0|1>`,
  current driven level (read back via `HAL_GPIO_ReadPin()`, reflecting
  the real driven state — same convention as `DIAGnostic:GPOut11`/
  `GPOut12`).

#### `ARM` precondition and `SM_FAULT_ENABLE_OUTPUT`

Direct instruction: "The controller cannot be armed unless these are
outputting prior to the arm signal, and similarly we cannot transition
to the ARM state i[f] these are not output." Two confirmed design
choices (asked directly, not assumed):

- **Per-channel gated** — only currently-*enabled* channels
  (`SOURce:ENAble?`) are checked, matching the exact same
  per-channel-gating philosophy as Water/Temp/Enerpro/OCP above. A
  channel's own `ENA_OUT`+`CONTACT_OUT` must both read `HIGH` to count
  as "outputting" — treated as one combined condition, not two
  independently-faultable ones.
- **Continuously monitored once `ARMED`** (not just at the `ARM`
  instant) — unlike `EXTernal:ENAble`'s `FIRING`-only carve-out, this
  is checked in both `ARMED` and `FIRING` (never `IDLE` — nothing is
  armed yet there). A drop enters `SM_STATE_FAULT` with a new
  **`SM_FAULT_ENABLE_OUTPUT`** type, `STATE?` reporting
  `OK FAULT ENABLE_OUTPUT <ch>` (1-based) — **per-channel**, like
  `OVERCURRENT`, whose exact response it reuses directly
  (`HandleOvercurrentFault()`): the affected channel is hard-disabled
  immediately, survivors derated 1/N and ramped down exactly like a
  real OCP fault. Reported as its own distinct type purely for
  operator diagnostics — do not confuse with `EXTERNAL_ENABLE`, a
  single system-wide *input* interlock (PF13); this is a per-channel
  check of this firmware's own commanded *output* state.
- `ARM` itself refuses (folded into the existing generic `ERR 13`) if
  any currently-enabled channel's outputs aren't both `HIGH`.
  `FAULT:CLEAR` needs no dedicated re-validation for this fault type
  (matching `OVERCURRENT`'s own precedent) — the affected channel is
  already hard-disabled by the time `FAULT:CLEAR` is considered, so
  the real gate is `ArmConditionsMet()` at the *next* `ARM` attempt.

**Verified on real hardware, 2026-09-17**: `ENAOut`/`CONTactOut` set
and read back correctly and independently per channel; argument
validation (`ERR 11`/`ERR 12`) correct; per-channel gating confirmed
-- with zero channels enabled, `ARM` succeeded regardless of
ENA_OUT/CONTACT_OUT state, confirming disabled channels are correctly
skipped. **NOT yet verified**: the actual refusal path (an *enabled*
channel with outputs not set blocking `ARM`) and the continuous
fault-on-loss-while-`ARMED` path -- enabling any channel on this bench
immediately trips the pre-existing OCP fault (nothing wired to those
pins yet), so a real enabled channel can't currently be gotten far
enough to test this gate specifically. See
`pending-hardware-calibration.md` for the tracked gap.

### `QSPI:ID?`

QUADSPI connectivity test against the W25Q128JVS NOR flash wired to
`PE12`-`PE15`/`PB10`/`PB11` (all AF10 — `docs/pin_mapping_v4.csv`),
added 2026-09-08 as an **easily removable** module
(`Core/Src/Inc/qspi_test.c/.h`) — see `QSPI_TEST_FEATURE_ENABLED` in
`qspi_test.h`; when disabled, this command is entirely absent from the
table (`ERR 1 Unknown command`, like any unrecognized mnemonic), the
same removability pattern `BOOT` uses. Deliberately narrow: issues the
chip's standard JEDEC Read ID instruction (`0x9F`, plain 1-line mode,
no Quad Enable bit required) and reports the 3 raw ID bytes as
uppercase hex — nothing else. No program/erase, no memory-mapped
access.

```
> QSPI:ID?
< OK EF 40 18
```

No expected value is hardcoded or asserted anywhere in this
command — it reports whatever the chip actually says; compare against
the W25Q128JVS datasheet's own JEDEC ID table yourself (`EF` = Winbond
manufacturer ID; the other two bytes are memory type/capacity). `ERR 7`
on any `HAL_QSPI` command/receive failure or timeout — e.g. no chip
present, a wiring fault, or a wedged bus.

**Verified on real hardware**: `OK EF 40 18` — correct Winbond
manufacturer ID and correct W25Q128 memory type/capacity bytes,
confirmed repeatable (3x) after reflashing. A real bug was found and
fixed getting here: the first attempt returned a stable "DE 80 30"
instead, traced to a QUADSPI sample-timing setting
(`SampleShifting`) — see `docs/changelog.txt` and `qspi_test.c`'s own
bugfix comment for the full diagnosis (an exact 1-bit shift of the
correct value, not random noise) and fix.

### `PFMIN:CAPTURE`, `PFMIN:STATus?`, `PFMIN:DATA?`, `PFMIN:DMASTAT?`

Bounded-count period/duty capture on the 6 `PFM_Input_01`..`_06` pins
(`Core/Src/pfm_input.c`) -- distinct from the continuous/free-running
capture mode the closed-loop control path uses internally
(`PfmInput_StartContinuous()`),
which has no wire command of its own; these four are for standalone
bench capture. Present only when `PFM_INPUT_FEATURE_ENABLED` is
nonzero (the default).

- **`PFMIN:CAPTURE <M>`** -- arms all 6 channels for the *next* `FIRE`
  (`M` = target period count per channel, 1-`PFM_INPUT_MAX_PERIODS`).
  Returns `OK` immediately; does not touch hardware or block. Consumed
  (cleared) by the next `FIRE`, whether or not it reached `M`.
- **`PFMIN:STATus?`** -- `OK <n1> <n2> <n3> <n4> <n5> <n6>`, each
  channel's current captured-period count -- poll this to know when an
  armed capture has finished.
- **`PFMIN:DATA? <ch>`** (`ch` = 1-6) -- `OK <count> OVERCAP=<n> <per1>
  <per2> ...`, raw tick periods in capture order (no duty -- see
  `pfm_input.h`'s own "raw ticks" philosophy note). `python/pfm_input_plot.py`
  is the reference host-side consumer (fetch + plot, both
  ticks and converted Hz).
- **`PFMIN:DMASTAT?`** -- TEMPORARY diagnostic, `OK <s1> .. <s6>`, the
  last `HAL_TIM_IC_Start_DMA()` return code per channel.
- `ERR 8` invalid channel, `ERR 9` `M` out of range.

### `SOURce:` / `SHOT:` / `LOG:` / `CHANnel:` -- output, shots, logging, channels

This project's whole point (`Core/Inc/pid.h`/`Core/Src/pid.c` --
Possibility 3 + fixed-rate Master heartbeat, see `docs/changelog.txt`'s
2026-09-09 design-decision entry). Not gated on a feature-enable flag.
`PID:` now means only the genuine PID-loop parameters `PID:GAINS`/
`PID:LOOPMODE` (below); the output/demand commands formerly under `PID:`
moved here on 2026-09-22 -- a clean cut, no aliases (see
`docs/changelog.txt`).
Channel numbering matches `PFMIN:DATA?`'s own convention: `1..N` on the
wire (`N` = `HRTIM_NUM_CHANNELS`, `CONFig:CHANnels?` reports it),
`0..N-1` internally. `ERR 11` invalid channel, `ERR 12` invalid
argument count/value throughout.

`python/wham_console.py` is the reference host-side front end for all
of the below -- an interactive operator console with a guided
shot-profile wizard (`shot`), a low-friction re-fire of whatever's
currently programmed (`refire`, e.g. after `gains <ch> <kp> <ki> <kd>`
-- no wizard prompts), live status, and automatic plotting after every
fire (`shot`, `refire`, or a raw `SHOT:STARt` typed directly all
trigger it) -- including a 4-row (channel 1-4) x 2-column output/FFT
plot showing each channel's commanded output next to its own frequency
spectrum, added 2026-09-22. Use it rather than hand-typing these for
routine bench work; the raw commands below remain available for
scripting or anything the console doesn't cover yet.

- **`SOURce:RUN`** -- begins closed-loop operation on every channel
  (starts free-running `PFM_Input` capture + PWM output together).
  Gated on the state machine as of 2026-09-22: requires `ARMED`
  (`ERR 13`) and the `EXTernal:ENAble` interlock (`ERR 15`), then
  transitions `ARMED` → `FIRING` via `SM_StartPlain()`. No shot clock --
  `SOURce:STOP` (`SM_Stop()`) returns it to `IDLE`.
- **`SOURce:STOP`** -- stops output + feedback capture on every channel,
  also ends any profile in progress (see `SHOT:STARt` below).
- **`SOURce:SETpoint <ch> <hz>`** -- sets channel `ch`'s target output
  frequency (clamped into `[PID_OUTPUT_MIN_HZ, PID_OUTPUT_MAX_HZ]`,
  `ctrlr_config.h`). Cancels any in-progress `SOURce:RAMP` on that channel.
- **`PID:GAINS <ch> <kp> <ki> <kd>`** -- sets channel `ch`'s PID gains
  and resets its integrator (avoids a discontinuous output jump from
  an integral accumulated under the old gains).
- **`PID:GAINS? <ch>`** -- `OK <kp> <ki> <kd>` -- added 2026-09-10 (see
  `PID_GetGains()`); `wham_console.py`'s `config` command uses this for
  a live readback rather than only remembering what it itself last sent.
- **`SOURce:STATus? <ch>`** -- `OK <running> <setpointHz> <measuredHz>
  <outputHz>`. `running` reflects the whole loop (`PID_IsRunning()`),
  not just this channel.
- **`SOURce:RAMP <ch> <startHz> <endHz> <durationMs>`** -- begins a linear
  setpoint ramp on channel `ch`, interpolated fresh each tick (exact
  landing on `endHz`, no rounding drift). Superseded outright by an
  active shot profile (below) while one is running.
- **`LOG:ARM <ch(0=all)> <maxSamples> <decim>`** -- arms waveform
  logging: every `decim`-th REAL `PID_Update()` tick (`maxSamples`
  clamped to `PID_LOG_MAX_SAMPLES`=1000) appends one
  `{setpointHz, measuredHz, outputHz}` sample per logged channel,
  held-value ticks included (see `pid.h`'s own `PID_ArmLog()` comment
  for the 2026-09-10 timebase-accuracy fix this depends on). `ch=0`
  arms EVERY channel at once, from the SAME real ticks
  (`PID_ArmLogAll()`, added 2026-09-10) -- a genuine simultaneous
  cross-channel comparison, not N separate runs; `python/
  wham_console.py`'s `shot` wizard's `all` logging option uses this
  for its one-PNG-per-shot, one-row-per-channel plot.
- **`LOG:DATA? [ch]`** -- `OK <count> <rateHz> s1 m1 o1 s2 m2 o2 ...`,
  channel `ch`'s log so far. `rateHz` = `PID_LOOP_RATE_HZ / decim` --
  sample `i` occurred at `i / rateHz` seconds after `LOG:ARM` was sent
  (the SAME for every channel when `ch=0` was used to arm -- that's
  the whole point). `[ch]` is optional and its own value defaults to
  whichever single channel is armed (unchanged, original behavior)
  ONLY when a single channel (not `ch=0`/all) is currently armed;
  otherwise a channel argument is required, and any channel
  1-`HRTIM_NUM_CHANNELS` is valid under all-channels mode (`ERR 12` if
  omitted then, `ERR 12` if it doesn't match the one single channel
  actually armed). STREAMED reply (one chunk per sample, not one giant
  buffer -- see `cmd_pid_logdata()`'s own comment on why), so it can
  take noticeably longer than other commands at 1000 samples; budget a
  generous read timeout (`wham_console.py` uses 8s).
- **`PID:LOOPMODE <ch> <0|1>`** -- `0` = open-loop (setpoint/profile
  value written straight to HRTIM, no PID correction; feedback still
  read/reported for comparison), `1` = closed-loop (default). `ch = 0`
  (added 2026-09-16) means "every channel at once" -- matches
  `LOG:ARM`'s own existing `ch=0` convention rather than a new command;
  real channels are still `1..HRTIM_NUM_CHANNELS` as always. Added
  originally in service of the external-trigger feature below (a
  single system-wide mode setting, not a new control-loop behavior),
  but usable standalone any time.
- **`PID:LOOPMODE? <ch>`** -- `OK <0|1>` -- added 2026-09-10 (see
  `PID_GetLoopMode()`).
- **`SOURce:ENAble <ch> <0|1>`** -- added 2026-09-11, per direct
  request: a genuine "this channel outputs no PFM waveform at all"
  switch, distinct from `PID:LOOPMODE` (open-loop still drives a real,
  uncorrected PFM waveform) or a 0A `SHOT:CURRent` (still drives
  a real PFM waveform, at the turn-on floor). `0` = fully disabled --
  the channel's HRTIM output pins are physically disconnected
  (`HRTIM1_SetChannelOutputEnable()`, `hrtim.c`) and `PID_Update()`
  skips this channel completely (no setpoint, no feedback consumption,
  no PID math, no log entry). `1` = enabled (default -- every channel
  always output, unchanged unless a channel is explicitly disabled).
  Takes effect on the NEXT `SOURce:RUN`/`SHOT:STARt` if the loop
  isn't currently running; takes effect **immediately, live**, if it
  is -- an operator can kill (or restore) one channel's real output
  mid-shot without touching any other channel or stopping the loop.
  Disabling does NOT stop that channel's own HRTIM counter (kept
  synchronized for an instant, clean re-enable) and does NOT reset its
  PID state (integral, setpoint, gains) -- re-enabling resumes exactly
  where it left off, not from a fresh reset.
- **`SOURce:ENAble? <ch>`** -- `OK <0|1>` -- see `PID_GetChannelEnable()`.
- **`CHANnel:NICKname <ch> <name>`** -- added 2026-09-13, per direct
  request: assigns a purely cosmetic, human-readable name to a channel
  (e.g. `TINKYWINKY`) -- no effect whatsoever on control behavior. This
  is separate from the `Ch1`..`Ch4` numbering used everywhere else on
  the wire (that numbering never changes) -- it's an extra label
  operator tooling (`wham_console.py`, the DSLogic shot plots) can show
  alongside the channel number, not a replacement for it. `name` must
  be 1-`PID_CHANNEL_NICKNAME_MAX_LEN` (15) characters, contain no
  whitespace (the wire protocol is space-tokenized, so a space would
  just look like extra/wrong argument count), and not be the literal
  string `-` (reserved, see the query below). `ERR 14` if `name` fails
  any of those checks -- the channel's existing nickname (if any) is
  left unchanged on a rejected attempt. Persists for the session (same
  model as gains/demand current -- no flash/EEPROM persistence anywhere
  in this firmware) -- cleared back to "no nickname" only by a reboot,
  survives across multiple shots.
- **`CHANnel:NICKname? <ch>`** -- `OK <name>`, or literally `OK -`
  if no nickname has been assigned to this channel yet -- see
  `PID_GetChannelNickname()`.

```
> SOURce:ENAble 3 0
< OK
  (channel 3's output pins disconnect immediately if the loop is
   running; channel 3's own PID state/HRTIM counter keep running
   untouched, just disconnected from the pins)
> SOURce:ENAble? 3
< OK 0
> SOURce:ENAble 3 1
< OK
  (channel 3 resumes output immediately, from where its own PID state
   left off -- not a fresh start)
> CHANnel:NICKname? 1
< OK -
  (no nickname assigned yet)
> CHANnel:NICKname 1 TINKYWINKY
< OK
> CHANnel:NICKname? 1
< OK TINKYWINKY
> CHANnel:NICKname 1 -
< ERR 14 Invalid nickname -- 1-PID_CHANNEL_NICKNAME_MAX_LEN chars, no spaces, and not the reserved value '-'
```

- **`SHOT:TIMing <rampUpTimeS> <flatTopTimeS> <rampDownTimeS>`**
  -- sets the SHARED ramp-up/flat-top/ramp-down durations (seconds) for
  the next `SHOT:STARt`, applied to every channel at once (each
  channel keeps its own peak current, below). All three must be `> 0`.
  Ramp up and ramp down are independently configurable (added
  2026-09-22 -- was two arguments, `<rampTimeS> <flatTopTimeS>`, one
  shared ramp duration for both directions, before this).
- **`SHOT:TIMing?`** -- `OK <rampUpTimeS> <flatTopTimeS>
  <rampDownTimeS>` -- added 2026-09-10 (see `PID_GetProfileTiming()`),
  extended to a third value 2026-09-22. `ERR 12` if never successfully
  set this boot -- a real, distinct "not configured" state
  (`SHOT:STARt` itself refuses to run in it), not reported as a
  bogus `0 0 0`.
- **`SHOT:CURRent <ch> <demandCurrentA>`** -- sets channel
  `ch`'s peak demand current (Amps, clamped to `[0, PFM_MAX_CURRENT_A]`)
  for the next shot.
- **`SHOT:CURRent? <ch>`** -- `OK <demandCurrentA>` -- added
  2026-09-10 (see `PID_GetProfileCurrent()`).
- **`SHOT:STARt`** -- begins a profiled shot on every channel at
  once, from the same synchronized instant: 0A -> linear ramp up ->
  `demandCurrentA` -> flat-top -> linear ramp down -> 0A, per channel's
  own timing-shared/current-independent trapezoid (see `pid.h`'s
  "DEMAND PROFILE" section). Amps -> Hz is a LINEAR PLACEHOLDER mapping
  (`ctrlr_config.h`'s `PFM_TURNON_FREQ_HZ`/`PFM_MAX_FREQ_HZ`/
  `PFM_MAX_CURRENT_A`) pending real hardware characterization. Ends
  automatically (full `SOURce:STOP`-equivalent, not hold-at-floor) when
  the shared clock reaches the shot's total duration -- send
  `SHOT:STARt` again for another shot, nothing resumes on its
  own. `ERR 12` if `SHOT:TIMing` was never (successfully) sent.

```
> PID:LOOPMODE 1 1
< OK
> PID:GAINS 1 1.0 10.0 0.0
< OK
> SHOT:CURRent 1 3000
< OK
> LOG:ARM 1 1000 3
< OK
> SHOT:TIMing 1.0 1.0 1.0
< OK
> SHOT:STARt
< OK
> SOURce:STATus? 1
< OK 1 40568 34700 34774
  ... (shot runs -- 1.0s ramp up, 1.0s flat-top, 1.0s ramp down) ...
> SOURce:STATus? 1
< OK 0 5056 11114 11013
  (running=0 -- shot auto-stopped, no SOURce:STOP needed)
> LOG:DATA?
< OK 1000 333 5741 3000 3000 6653 3000 3000 ...
```

**Hard output slew-rate clamp** (2026-09-10, `ctrlr_config.h`'s
`PID_OUTPUT_MAX_SLEW_HZ_PER_TICK`): every tick's actual write to HRTIM
-- from any of the commands above -- is bounded to at most that many Hz
of change from the previous tick's actual output, in either direction.
Compile-time only, no wire command; see that macro's own extensive
comment for why (a real, DSLogic-confirmed single-tick output glitch)
and its current placeholder status. **One deliberate exception** (added
2026-09-15): the OCP `(100*1/N)%` derate step (`OVERCURRENT` fault,
above) bypasses this clamp for that one write only -- see
`ClampOutputRangeOnly()`'s comment in `pid.c`.

## SIM: namespace (simulator-only)

Added 2026-09-18, backed by `sim_transrex.c/.h` — see that module's own
header comment for the full design. **Does not exist on a controller
build at all** — declarations, definitions, and `cmd_parser.c`
registration all share the same `BUILD_TARGET_SIMULATOR` guard, unlike
`DIAGnostic:GPOut09-12`/etc., which are generic pins present on both
targets. Sending any `SIM:` command to a real controller gets
`ERR 1 Unknown command`.

This is the actual "act like a Transrex" logic: it captures the
controller's real commanded `XRn_DRIVE` frequency (reusing
`pfm_input.c`'s existing continuous capture, unchanged), low-pass
filters it, and drives the result back out on this board's own
`XRn_DRIVE` HRTIM channels — which the controller receives as its own
`XRn_FEEDBACK`. Gated per channel on `ENA_OUT`+`CONTACT_OUT` both being
asserted (received from the controller); when not gated, the output
floors to `PFM_TURNON_FREQ_HZ` (0 A) rather than holding its last
value. Runs unconditionally from boot (started in `main.c`, updated
every main-loop iteration) — not tied to this board's own ARM/FIRE
state, since a real Transrex's response depends on being connected and
enabled, not on its own internal shot cycle.

### `SIM:FAULT:WATERTEMP` / `SIM:FAULT:WATERTEMP?`

Combined Water+Temp fault injection for one channel — these two share a
single fiber+splitter per channel on the finalized wiring
(`docs/pin_mapping_reference.tex` Section 7), so they cannot be
independently faulted; this is an honest single command rather than two
aliased ones. `<0|1>`: 1 injects a fault (drives the transmitter LOW,
`FAULT_POLARITY_NORMALLY_HIGH`'s fault level); 0 restores healthy
(HIGH). Drives the same physical pins `XREX:CHANnel:ENAOut` can also
drive directly — don't use both mechanisms on the same channel at once.

```
> SIM:FAULT:WATERTEMP 1 1
< OK
> SIM:FAULT:WATERTEMP? 1
< OK 1
```

- **`SIM:FAULT:WATERTEMP <ch> <0|1>`** — `OK`. `ERR 11` for an invalid
  channel, `ERR 12` if an argument is missing.
- **`SIM:FAULT:WATERTEMP? <ch>`** — `OK <0|1>`, this module's own
  injected state (not a raw pin read).

### `SIM:FAULT:ENERPRO` / `SIM:FAULT:ENERPRO?`

Independent per-channel Enerpro fault injection (one dedicated
transmitter per channel). Same polarity, argument, and error
convention as `SIM:FAULT:WATERTEMP` above — drives the same pins
`XREX:CHANnel:CONTactOut` can also drive directly.

### `SIM:FAULT:OCP` / `SIM:FAULT:OCP?`

Independent per-channel OCP fault injection (one dedicated transmitter
per channel, XR1-4 in that order matching the controller's own
irregular `kOcpPin[]` order). Same polarity/argument/error convention
as `SIM:FAULT:WATERTEMP` above — drives the same pins
`DIAGnostic:GPOut09`-`GPOut12` can also drive directly.

### `SIM:MODEL:TAU` / `SIM:MODEL:TAU?`

The low-pass filter's time constant, in milliseconds, shared across
every channel. Default `SIM_TRANSREX_DEFAULT_TAU_MS` (100 — a first,
reasonable-sounding guess, not derived from real magnet/supply time
constants; revisit once real bench step-response data exists, same
placeholder status as several `ctrlr_config.h` calibration constants).

```
> SIM:MODEL:TAU 250
< OK
> SIM:MODEL:TAU?
< OK 250
```

- **`SIM:MODEL:TAU <ms>`** — `OK`. `ERR 11` if `ms` is not a positive
  number (a zero time constant is a divide-by-zero in the filter math,
  not a meaningful "instant response" request).
- **`SIM:MODEL:TAU?`** — `OK <ms>`.

### `SIM:DIAGnostic:IDLETONE` / `SIM:DIAGnostic:IDLETONE?`

Board-wide (all 4 channels at once, not per-channel) diagnostic idle
tone, added 2026-09-21 for visually confirming the fiber transmitters
are alive on the bench independent of any real shot. Default `0` (OFF)
— an ungated channel's output stays physically DISCONNECTED, this
module's normal, realistic "not responding" behavior. `1` (ON): an
UNGATED channel's output instead stays CONNECTED and holds a steady
`SIM_TRANSREX_IDLE_TONE_HZ` tone (3000 Hz). A GATED channel (real shot
in progress) is unaffected either way — this only changes the idle
state.

```
> SIM:DIAGnostic:IDLETONE 1
< OK
> SIM:DIAGnostic:IDLETONE?
< OK 1
> SIM:CHANnel:STATus? 1
< OK DRIVE_HZ=0 FEEDBACK_HZ=3000 ENA_OUT=LOW CONTACT_OUT=LOW WATERTEMP_FAULT=0 ENERPRO_FAULT=0 OCP_FAULT=0
```

- **`SIM:DIAGnostic:IDLETONE <0|1>`** — `OK`. `ERR 11` if the value
  isn't `0` or `1`.
- **`SIM:DIAGnostic:IDLETONE?`** — `OK <0|1>`.

### `SIM:CHANnel:STATus?`

One-shot diagnostic snapshot for a channel: the last measured `DRIVE`
frequency, this module's own current filtered `FEEDBACK` output, the
live raw `ENA_OUT`/`CONTACT_OUT` gating bits, and the three injected
fault states.

```
> SIM:CHANnel:STATus? 1
< OK DRIVE_HZ=0 FEEDBACK_HZ=5000 ENA_OUT=LOW CONTACT_OUT=LOW WATERTEMP_FAULT=0 ENERPRO_FAULT=0 OCP_FAULT=0
```

- **`SIM:CHANnel:STATus? <ch>`** — `OK DRIVE_HZ=<n> FEEDBACK_HZ=<n>
  ENA_OUT=HIGH|LOW CONTACT_OUT=HIGH|LOW WATERTEMP_FAULT=<0|1>
  ENERPRO_FAULT=<0|1> OCP_FAULT=<0|1>`. `ERR 11`/`ERR 12` same as
  every other channel-argument command above.

### `SIM:LOG` / `SIM:LOGDATA?`

Added 2026-09-18, per direct correction: `python/run_simulator_validation.py`
originally polled `SIM:CHANnel:STATus?` at a fixed host-side interval
during a shot to build a waveform log — coarse (~10 samples/sec) and
adds serial round-trip jitter. This is the simulator-side equivalent
of the controller's `LOG:ARM`/`LOG:DATA?` (below): arm before a
shot, let `sim_transrex.c`'s `SimTransrex_Update()` log every real
main-loop tick with no host involvement, retrieve after.

**Differs from `LOG:ARM`/`LOG:DATA?` in one way**: `LOG:ARM` decimates
against a REAL fixed hardware tick (`PID_Update()` runs on the HRTIM
Master's 1kHz interrupt, so "every Nth tick" is precise) and reports one
shared `rate_hz` for the whole log. `SimTransrex_Update()` runs off the
main loop instead, which has no fixed rate at all — so `SIM:LOG` takes a
minimum-time-between-samples throttle (milliseconds) rather than a tick
decimation, and `SIM:LOGDATA?` reports each sample's own elapsed-time-
since-armed timestamp explicitly instead of a shared rate.

Single-channel-at-a-time (like `LOG:ARM`'s single-channel mode, not
`LOG:ARM 0 ...`'s all-channels mode) — arming a new channel discards
whatever was previously logged.

```
> SIM:LOG 1 1000 5
< OK
... (run a shot) ...
> SIM:LOGDATA? 1
< OK 847 0 0 5000 5 0 5000 10 8213 6104 15 20007 9821 ...
```
(each triple is `<t_ms> <drive_hz> <feedback_hz>` — `drive_hz` is 0
until the channel is actually gated on and a real DRIVE signal is
first measured; `feedback_hz` starts at the `PFM_TURNON_FREQ_HZ` floor
and converges once gated).

- **`SIM:LOG <ch> <maxSamples> <minIntervalMs>`** — `OK`, arms
  logging. `maxSamples` clamped to `SIM_LOG_MAX_SAMPLES` (1000,
  `sim_transrex.h`); `minIntervalMs` of `0` logs every single
  `SimTransrex_Update()` call, unthrottled. `ERR 11` for an invalid
  channel, `ERR 12` for a missing/invalid argument.
- **`SIM:LOGDATA? <ch>`** — `OK <count> <t0_ms> <drive0_hz>
  <feedback0_hz> <t1_ms> <drive1_hz> <feedback1_hz> ...` — `ch` must be
  the currently-armed channel. `count` stays at whatever it reached
  once `maxSamples` is hit (logging simply stops appending, it doesn't
  wrap or reset). `ERR 11`/`ERR 12` same as `SIM:LOG`.

## Adding a command

From `cmd_parser.c`'s own header comment:

1. Implement the handler in `commands.c`.
2. Declare it in `commands.h`.
3. Add a `{ "PATTern:MNEMonic?", handler }` row to `command_table[]` in
   `cmd_parser.c`.

Nothing else changes — the table is flat, so a new leaf or a whole new
subsystem is always just one more row. Update this document when you do.
