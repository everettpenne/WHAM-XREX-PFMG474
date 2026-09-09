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
< OK WHAM-XREX-PFMG474 REVA v0.1
```

Reports, space-separated: `HW_BOARD_NAME`, `HW_BOARD_REV`,
`FW_VERSION_STRING` — all compile-time constants in `Core/Inc/version.h`.
`HW_BOARD_REV` is currently a placeholder (`REVA`); update it to match
the actual PCB silkscreen revision.

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
  specifically remains open there.

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

## Adding a command

From `cmd_parser.c`'s own header comment:

1. Implement the handler in `commands.c`.
2. Declare it in `commands.h`.
3. Add a `{ "PATTern:MNEMonic?", handler }` row to `command_table[]` in
   `cmd_parser.c`.

Nothing else changes — the table is flat, so a new leaf or a whole new
subsystem is always just one more row. Update this document when you do.
