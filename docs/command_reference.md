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
| 11 | Invalid `PID` channel (also reused by `OCP:TEST:FAULT`, same channel-range check) |
| 12 | Invalid `PID:*` argument count/value -- see the specific command's own usage (also reused by `OCP:TEST:FAULT`, `PFMIN:DEBUG:RAW?`/`PFMIN:DEBUG:REG?`) |
| 13 | Invalid state-machine transition for the current state (`ARM`/`DISARM`/`PID:PROFile:STARt`, see that section) |
| 14 | Invalid `PID:CHANnel:NICKname` -- name must be 1-`PID_CHANNEL_NICKNAME_MAX_LEN` chars, no spaces, and not the reserved value `-` |

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

- No `ARM`/state-machine interlock exists in this firmware — `FIRE`
  always takes effect immediately, whether the controller was idle or
  already mid-shot (re-firing mid-shot restarts from step 0). If a
  fuller interlock/fault-gated firing sequence is ever needed, this is
  the command to extend, not a design decision this entry documents as
  final.
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
   12 pins on any edge and evaluates them against the compile-time
   `GDS_FAULT_POLARITY` (`Core/Inc/ctrlr_config.h`); on a fault, forces
   HRTIM output off and latches. Needs the EXTI ISR to actually run,
   unlike PC10's autonomous hardware path — a boot-time explicit check
   (`main.c`) covers the one gap that leaves (a pin already faulted
   before the interrupt is even armed produces no edge of its own).

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
  readiness check that is still mostly a **STUB** (profile timing
  configured, at least one channel enabled, gains sane, etc. are not
  checked yet) — with ONE real condition now populated, 2026-09-16: the
  external-enable interlock (see `EXTernal:ENAble` below), if turned
  on. Outputs are still disabled here, identically to `IDLE` — nothing
  electrical changes on entry; `ARMED` exists purely as a
  separately-confirmable "ready to fire" step before
  `PID:PROFile:STARt` is allowed to do anything. Left via
  `PID:PROFile:STARt` (→ `FIRING`), `DISARM` (→ `IDLE`, stand down
  without firing), or `PID:STOP` (→ `IDLE`, abort).
- **`FIRING`** — entered only from `ARMED`, via `PID:PROFile:STARt`
  (unchanged command, now gated: `ERR 13` if not currently `ARMED`).
  Every currently-enabled channel (`PID:CHANnel:ENAble` — a separate,
  per-channel concern) begins its ramp profile. Returns to `IDLE`
  automatically when the shot completes, or on a manual `PID:STOP`.
- **`FAULT`** — entered from ANY state the instant either fault source
  trips. Always stops the legacy (`TABLE:*`/`FIRE`) output path
  immediately (it isn't part of this state machine at all). The
  current (`PID:*`) output path's own stop depends on the fault type:
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
    `ARM` and `PID:PROFile:STARt`, separately from this continuous
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
> PID:PROFILE:START
< OK
> STATE?
< OK FIRING
  ... (shot runs, completes on its own) ...
> STATE?
< OK IDLE
> PID:PROFILE:START
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
  or if the (currently stub) readiness check refuses.
- **`DISARM`** — `OK`, `ARMED` → `IDLE`. No-op (still `OK`, not an
  error) if not currently `ARMED`.
- **`STATE?`** — `OK <IDLE|ARMED|FIRING>`, or `OK FAULT GENERAL` /
  `OK FAULT OVERCURRENT <ch>` (1-based) / `OK FAULT EXTERNAL_ENABLE`
  while faulted.
- New error code **13**: an invalid state-machine transition for the
  current state (e.g. `PID:PROFile:STARt` sent while not `ARMED`;
  `ARM` sent while not `IDLE`).

**Known gap, flagged not fixed**: `PID:START` (the simpler, non-profile
closed-loop start, distinct from `PID:PROFile:STARt`) is **not** gated
by this state machine at all — it can start real output regardless of
`STATE?`'s current value, and does not move the state machine out of
`IDLE`/`ARMED`. Per direct instruction, only `PID:PROFile:STARt` was
wired up as the "fire" trigger this round; `PID:START` bypassing the
whole state machine is a known, real inconsistency worth resolving
later, not an oversight to silently work around.

### `EXTernal:ENAble` / `EXTernal:ENAble?` / `EXTernal:INPut?`

Added 2026-09-16, per direct request: an external operator/facility
permissive interlock on **PF15** (`Fiber_Enable`, `docs/pin_mapping_v4.csv`
— confirmed `GPI` there; PC14, first proposed for this, was rejected —
it's documented `GPO` in that same file, `STM_Enable_Pin`, meaning the
STM32 drives that one outward, the opposite direction needed here).

**Opt-in, OFF by default** — existing shots/tests are completely
unaffected unless explicitly turned on. RAM-only config (like
`PID:PROFILE:CURRENT`/`PID:LOOPMODE`/etc.) — resets to OFF on every
reboot, not persisted.

When ON:
1. **`ARM`** refuses (folded into its existing generic `ERR 13`) unless
   PF15 currently reads HIGH.
2. **`PID:PROFile:STARt`** ALSO re-checks PF15 immediately before firing
   — closes the real gap where PF15 could drop in the window between a
   successful `ARM` and the eventual `START` (`ARM` alone does not
   guarantee this at the moment of firing). `ERR 15` if refused.
3. While **`FIRING`**, checked continuously (same ~1kHz cadence General
   Fault's own two hardware sources get) — if PF15 drops, enters
   `SM_STATE_FAULT` with `EXTERNAL_ENABLE` (see the `FAULT` state's own
   entry above for the identical-to-`GENERAL` ramp-down response).
   Deliberately **not** actively monitored while merely `IDLE`/`ARMED`
   (nothing outputting yet to protect) — losing PF15 there just means
   the next `PID:PROFile:STARt` attempt fails its own re-check (item 2)
   instead of entering `FAULT`. Not decided either way whether `ARMED`
   should also actively fault on loss — not asked for, flagged rather
   than silently added.
4. **`FAULT:CLEAR`** re-validates PF15 is back HIGH before actually
   clearing an `EXTERNAL_ENABLE` fault — same "only clear if the
   condition is actually gone" treatment `PC10`/`GateDriverStatus`
   already get.

PF15 is configured `GPIO_PULLDOWN` (`main.c`'s `MX_GPIO_Init()`) —
**not** this project's usual `GPIO_NOPULL` for actively-driven inputs
(`GateDriverStatus`, `PFM_Input`, `QUADSPI`). Deliberate: an
unconnected/floating PF15 must read LOW (no permission), never an
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
  ... (PF15 goes HIGH) ...
> ARM
< OK
> PID:PROFILE:START
< OK
  ... (PF15 drops while FIRING) ...
> STATE?
< OK FAULT EXTERNAL_ENABLE
> FAULT:CLEAR
< ERR ...   (still LOW -- refused)
  ... (PF15 restored HIGH) ...
> FAULT:CLEAR
< OK
```

- **`EXTernal:ENAble <0|1>`** — `OK`, turns the interlock on/off.
  `ERR 12` if the argument is missing.
- **`EXTernal:ENAble?`** — `OK <0|1>`, current mode.
- **`EXTernal:INPut?`** — `OK <0|1>`, PF15's raw logic level right now,
  independent of whether the interlock is even turned on — lets an
  operator confirm real wiring/signal presence before relying on it,
  the same diagnostic role `PFMIN:DEBUG:RAW?`/`PFMIN:DEBUG:REG?` played
  for the `PFM_Input` fiber-patching investigation below.
- New error code **15**: `PID:PROFile:STARt` refused because the
  interlock is on and PF15 currently reads LOW.

### `EXTernal:TRIGger` / `EXTernal:TRIGger?`

Added 2026-09-16, per direct follow-up request: a **rising edge on
PF15** (the SAME pin `EXTernal:ENAble` above uses — not a second
signal) fires a shot while `ARMED`, exactly as if `PID:PROFile:STARt`
had been sent manually. **Structurally depends on `EXTernal:ENAble`
being on first** — `EXTernal:TRIGger 1` refuses (`ERR 16`) unless it
is, and turning `EXTernal:ENAble` back off also forces this back off —
the "lose the signal mid-shot → fault" protection this feature relies
on to behave sanely IS that same interlock, not a separate mechanism.

There is no separate "open-loop start call" to invoke here: open- vs.
closed-loop has always been the **per-channel** `PID:LOOPMODE` flag,
checked inside the same control loop both `PID:START` and
`PID:PROFile:STARt` already share — not a different start mechanism.
`PID:LOOPMODE 0 <0|1>` (above, also added this same day) is a plain
convenience for setting every channel's mode at once before an
externally-triggered shot, not something this feature reads or
branches on itself.

Edge-triggered, not level-triggered: a baseline PF15 level is captured
fresh the instant `ARM` succeeds, so a signal already HIGH at the
moment of arming does **not** look like a rising edge on the next
check — only a genuine LOW→HIGH transition after arming fires. If the
resulting `PID:PROFile:STARt` call itself fails for some other reason
(e.g. profile timing never set), the state simply stays `ARMED` — a
fresh falling-then-rising edge is needed to try again, not just PF15
remaining HIGH.

```
> EXTernal:ENAble 1
< OK
> EXTernal:TRIGger 1
< OK
> ARM
< OK
  ... (PF15 rises) ...
> STATE?
< OK FIRING
```

- **`EXTernal:TRIGger <0|1>`** — `OK`, turns the trigger feature
  on/off. `ERR 12` if the argument is missing, `ERR 16` if turning it
  on while `EXTernal:ENAble` is currently off.
- **`EXTernal:TRIGger?`** — `OK <0|1>`, current mode.
- New error code **16**: `EXTernal:TRIGger` refused because
  `EXTernal:ENAble` must be turned on first.

**NOT YET VERIFIED ON REAL HARDWARE** as of this writing — build-clean
only (zero warnings), the board was unavailable this session. In
particular, for BOTH `EXTernal:ENAble` and `EXTernal:TRIGger`: the
`GPIO_PULLDOWN` fail-safe default, the actual PF15 signal once real
external hardware drives it, the FIRING-time fault response, and (new
for `EXTernal:TRIGger`) a real rising edge actually firing a shot from
`ARMED` have not been exercised on the bench yet — see
`docs/changelog.txt`'s matching entries.

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
> PID:CHANNEL:ENABLE? 2
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
  gated" treatment `FAULT:CLEAR`/`PID:STOP` already get. Remove once
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
down why `measuredHz` (`PID:STATus?`) read 0 (or, later, a swapped
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
  accumulator state (unlike `PID:STATus?`'s own consumption of the same
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
capture mode `PID:*` uses internally (`PfmInput_StartContinuous()`),
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

### `PID:*` -- closed-loop control

This project's whole point (`Core/Inc/pid.h`/`Core/Src/pid.c` --
Possibility 3 + fixed-rate Master heartbeat, see `docs/changelog.txt`'s
2026-09-09 design-decision entry). Not gated on a feature-enable flag.
Channel numbering matches `PFMIN:DATA?`'s own convention: `1..N` on the
wire (`N` = `HRTIM_NUM_CHANNELS`, `CONFig:CHANnels?` reports it),
`0..N-1` internally. `ERR 11` invalid channel, `ERR 12` invalid
argument count/value throughout.

`python/wham_console.py` is the reference host-side front end for all
of the below -- an interactive operator console with a guided
shot-profile wizard, live status, and automatic plotting. Use it
rather than hand-typing these for routine bench work; the raw commands
below remain available for scripting or anything the console doesn't
cover yet.

- **`PID:START`** -- begins closed-loop operation on every channel
  (starts free-running `PFM_Input` capture + PWM output together).
- **`PID:STOP`** -- stops output + feedback capture on every channel,
  also ends any profile in progress (see `PID:PROFILE:STARt` below).
- **`PID:SETPOINT <ch> <hz>`** -- sets channel `ch`'s target output
  frequency (clamped into `[PID_OUTPUT_MIN_HZ, PID_OUTPUT_MAX_HZ]`,
  `ctrlr_config.h`). Cancels any in-progress `PID:RAMP` on that channel.
- **`PID:GAINS <ch> <kp> <ki> <kd>`** -- sets channel `ch`'s PID gains
  and resets its integrator (avoids a discontinuous output jump from
  an integral accumulated under the old gains).
- **`PID:GAINS? <ch>`** -- `OK <kp> <ki> <kd>` -- added 2026-09-10 (see
  `PID_GetGains()`); `wham_console.py`'s `config` command uses this for
  a live readback rather than only remembering what it itself last sent.
- **`PID:STATus? <ch>`** -- `OK <running> <setpointHz> <measuredHz>
  <outputHz>`. `running` reflects the whole loop (`PID_IsRunning()`),
  not just this channel.
- **`PID:RAMP <ch> <startHz> <endHz> <durationMs>`** -- begins a linear
  setpoint ramp on channel `ch`, interpolated fresh each tick (exact
  landing on `endHz`, no rounding drift). Superseded outright by an
  active shot profile (below) while one is running.
- **`PID:LOG <ch(0=all)> <maxSamples> <decim>`** -- arms waveform
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
- **`PID:LOGDATA? [ch]`** -- `OK <count> <rateHz> s1 m1 o1 s2 m2 o2 ...`,
  channel `ch`'s log so far. `rateHz` = `PID_LOOP_RATE_HZ / decim` --
  sample `i` occurred at `i / rateHz` seconds after `PID:LOG` was sent
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
  `PID:LOG`'s own existing `ch=0` convention rather than a new command;
  real channels are still `1..HRTIM_NUM_CHANNELS` as always. Added
  originally in service of the external-trigger feature below (a
  single system-wide mode setting, not a new control-loop behavior),
  but usable standalone any time.
- **`PID:LOOPMODE? <ch>`** -- `OK <0|1>` -- added 2026-09-10 (see
  `PID_GetLoopMode()`).
- **`PID:CHANnel:ENAble <ch> <0|1>`** -- added 2026-09-11, per direct
  request: a genuine "this channel outputs no PFM waveform at all"
  switch, distinct from `PID:LOOPMODE` (open-loop still drives a real,
  uncorrected PFM waveform) or a 0A `PID:PROFile:CURRent` (still drives
  a real PFM waveform, at the turn-on floor). `0` = fully disabled --
  the channel's HRTIM output pins are physically disconnected
  (`HRTIM1_SetChannelOutputEnable()`, `hrtim.c`) and `PID_Update()`
  skips this channel completely (no setpoint, no feedback consumption,
  no PID math, no log entry). `1` = enabled (default -- every channel
  always output, unchanged unless a channel is explicitly disabled).
  Takes effect on the NEXT `PID:START`/`PID:PROFile:STARt` if the loop
  isn't currently running; takes effect **immediately, live**, if it
  is -- an operator can kill (or restore) one channel's real output
  mid-shot without touching any other channel or stopping the loop.
  Disabling does NOT stop that channel's own HRTIM counter (kept
  synchronized for an instant, clean re-enable) and does NOT reset its
  PID state (integral, setpoint, gains) -- re-enabling resumes exactly
  where it left off, not from a fresh reset.
- **`PID:CHANnel:ENAble? <ch>`** -- `OK <0|1>` -- see `PID_GetChannelEnable()`.
- **`PID:CHANnel:NICKname <ch> <name>`** -- added 2026-09-13, per direct
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
- **`PID:CHANnel:NICKname? <ch>`** -- `OK <name>`, or literally `OK -`
  if no nickname has been assigned to this channel yet -- see
  `PID_GetChannelNickname()`.

```
> PID:CHANNEL:ENABLE 3 0
< OK
  (channel 3's output pins disconnect immediately if the loop is
   running; channel 3's own PID state/HRTIM counter keep running
   untouched, just disconnected from the pins)
> PID:CHANNEL:ENABLE? 3
< OK 0
> PID:CHANNEL:ENABLE 3 1
< OK
  (channel 3 resumes output immediately, from where its own PID state
   left off -- not a fresh start)
> PID:CHANNEL:NICKNAME? 1
< OK -
  (no nickname assigned yet)
> PID:CHANNEL:NICKNAME 1 TINKYWINKY
< OK
> PID:CHANNEL:NICKNAME? 1
< OK TINKYWINKY
> PID:CHANNEL:NICKNAME 1 -
< ERR 14 Invalid nickname -- 1-PID_CHANNEL_NICKNAME_MAX_LEN chars, no spaces, and not the reserved value '-'
```

- **`PID:PROFile:TIMing <rampTimeS> <flatTopTimeS>`** -- sets the
  SHARED ramp/flat-top durations (seconds) for the next
  `PID:PROFile:STARt`, applied to every channel at once (each channel
  keeps its own peak current, below). Both must be `> 0`.
- **`PID:PROFile:TIMing?`** -- `OK <rampTimeS> <flatTopTimeS>` -- added
  2026-09-10 (see `PID_GetProfileTiming()`). `ERR 12` if never
  successfully set this boot -- a real, distinct "not configured" state
  (`PID:PROFile:STARt` itself refuses to run in it), not reported as a
  bogus `0 0`.
- **`PID:PROFile:CURRent <ch> <demandCurrentA>`** -- sets channel
  `ch`'s peak demand current (Amps, clamped to `[0, PFM_MAX_CURRENT_A]`)
  for the next shot.
- **`PID:PROFile:CURRent? <ch>`** -- `OK <demandCurrentA>` -- added
  2026-09-10 (see `PID_GetProfileCurrent()`).
- **`PID:PROFile:STARt`** -- begins a profiled shot on every channel at
  once, from the same synchronized instant: 0A -> linear ramp up ->
  `demandCurrentA` -> flat-top -> linear ramp down -> 0A, per channel's
  own timing-shared/current-independent trapezoid (see `pid.h`'s
  "DEMAND PROFILE" section). Amps -> Hz is a LINEAR PLACEHOLDER mapping
  (`ctrlr_config.h`'s `PFM_TURNON_FREQ_HZ`/`PFM_MAX_FREQ_HZ`/
  `PFM_MAX_CURRENT_A`) pending real hardware characterization. Ends
  automatically (full `PID:STOP`-equivalent, not hold-at-floor) when
  the shared clock reaches the shot's total duration -- send
  `PID:PROFile:STARt` again for another shot, nothing resumes on its
  own. `ERR 12` if `PID:PROFile:TIMing` was never (successfully) sent.

```
> PID:LOOPMODE 1 1
< OK
> PID:GAINS 1 1.0 10.0 0.0
< OK
> PID:PROFILE:CURRENT 1 3000
< OK
> PID:LOG 1 1000 3
< OK
> PID:PROFILE:TIMING 1.0 1.0
< OK
> PID:PROFILE:START
< OK
> PID:STATus? 1
< OK 1 40568 34700 34774
  ... (shot runs -- 1.0s ramp up, 1.0s flat-top, 1.0s ramp down) ...
> PID:STATus? 1
< OK 0 5056 11114 11013
  (running=0 -- shot auto-stopped, no PID:STOP needed)
> PID:LOGDATA?
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

## Adding a command

From `cmd_parser.c`'s own header comment:

1. Implement the handler in `commands.c`.
2. Declare it in `commands.h`.
3. Add a `{ "PATTern:MNEMonic?", handler }` row to `command_table[]` in
   `cmd_parser.c`.

Nothing else changes — the table is flat, so a new leaf or a whole new
subsystem is always just one more row. Update this document when you do.
