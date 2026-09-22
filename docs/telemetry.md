# Telemetry Standardization Plan

Status: Phase 1 implemented (2026-09-22); Phase 3 partially implemented
(waveform log captures the fault ramp-down; "one line per packet" framing
invariant formalized); Phase 4 implemented (flight recorder + SYS:EVLOG?,
RAM-only); Phase 5 partially implemented (wham_console.py receives/surfaces
`!EVT`, adds `events` + `evlog` commands); Phases 0, 2 pending.
Date: 2026-09-22.

## 1. Purpose

Standardize how this controller reports runtime data (state, faults, control-loop
waveforms, diagnostics) to a host, so that:

- a host (wham_console.py, the LLM console, a future logger) can decode any
  reply against a versioned contract instead of hard-coding each command's
  reply format;
- faults and state changes are *pushed* to the host the instant they happen,
  instead of being discovered by polling;
- every time-series sample carries a monotonic timestamp, so "when did this
  happen" is never reconstructed by hand;
- telemetry is provably unable to disturb the control loop or fault handling
  (Section 6).

This is observability, not control. It must never be able to change, delay, or
mask the behavior of the PID loop, the HRTIM output, or any fault path.

## 2. Non-negotiable constraints

1. **Never disturb the output or fault handling, especially during a shot.**
   This is the whole point of Section 6. The 1 kHz PID loop (`PID_Update()`,
   `HRTIM1_Master_IRQn` @ priority 1) and every fault path (PC10 in silicon,
   GateDriverStatus EXTI @ priority 1, OCP/Enerpro polled in `PID_Update()`)
   must have identical latency with telemetry on or off.
2. **Zero new ISR-path work beyond tiny, bounded stores.** Sampling already
   happens in the ISR (the PID waveform log); telemetry adds at most a few
   register/array stores there (an event enqueue), never `snprintf`, never a
   UART write, never a blocking call.
3. **Bare-metal, no heap, 128 KiB RAM.** No dynamic allocation. All telemetry
   buffers are fixed-size static arrays sized in the plan (Section 10).
4. **Keep the SCPI console.** Operators can still type raw `SOURce:*`,
   `SHOT:*`, `FAULT?`, `STATE?`, etc. exactly as today. Standardization layers
   *under* it, it does not replace it.
5. **Keep serial reflash working.** The `BOOT` → ROM-bootloader path must not
   be affected (and we do not reintroduce the IWDG, which is incompatible with
   it — see changelog 2026-09-22).
6. **`FAULT:CLEAR` / `SOURce:STOP` responsiveness.** Even mid-dump, an operator
   command must be serviced promptly (this is what forces non-blocking TX,
   Section 7).

## 3. Current state (audit)

The project already has a good *implicit* telemetry system; the plan is to make
it explicit and consistent.

| Tier | What exists today |
|---|---|
| Identity | `*IDN?` (board/rev/FW+git hash), `CONFig:CHANnels?` |
| Snapshot (pull) | `STATE?`, `FAULT?`, `GDS?`, `XREX:CHANnel:STATus?`, `QSPI:ID?`, `SOURce:STATus?`, `PID:GAINS?`, `PID:LOOPMODE?`, `SHOT:*?` |
| Stream (time-series) | `LOG:ARM`/`LOG:DATA?` — decimated, RAM-bounded {setpoint, measured, output} per channel |
| Diagnostics (gated) | `DIAGnostic:*` (GPOut/RSTCause/OPTBytes), `PFM:DIAG?`, `PFM:GAPLOG?`, `PFMIN:DEBUG:*` |
| Host | `wham_console.py` (plot/report/diag), `pfm_input_plot.py`, `memory_report.py` (build-time) |

Known gaps: no timestamps anywhere; no async event push; no schema/version
contract; inconsistent reply grammar and units; no CRC/sequence numbers; no
persistent event history; pull-only stream; **blocking `uart_send`** (a large
`LOG:DATA?` dump holds the main loop for seconds); host parsing is
per-command.

## 4. Target model

Three tiers plus a schema contract, all on top of the existing SCPI shell.

- **Tier 1 — Snapshots** (pull, on-demand, human-readable). The existing `*?`
  queries, standardized to a documented, versioned field order and one
  consistent grammar. Unchanged in spirit.
- **Tier 2 — Stream** (time-series). The PID waveform, framed with a
  self-describing header (rate, start tick, channel count, units, schema) and
  per-sample monotonic timestamps. Capture stays in the ISR (already the case);
  transmission is throttled/async (Section 6).
- **Tier 3 — Events** (async push). Unsolicited, timestamped lines the device
  emits on its own for state transitions and fault latch/clear. This is the
  highest-value addition: the host *subscribes* instead of polling.

Plus a **schema contract**: `SYS:TELEM?` reports a version bumped on any wire
change; `SYS:TIME?` exposes the monotonic clock; `SYS:EVENT <0|1>` gates the
`!EVT` stream.

## 5. Wire grammar (proposed, to finalize in implementation)

Replies and errors stay exactly as today (`OK ...` / `ERR <code> <msg>`). The
only new element is an **unsolicited event prefix that can never collide with a
reply**:

```
!EVT <tick_ms> <EVENT> [arg...]
```

Examples:

```
!EVT 0012345678 STATE FIRING
!EVT 0012345678 FAULT OCP 1
!EVT 0012345678 FAULT CLEAR
!EVT 0012345678 TRIGGER FIRING
```

New `SYS:` namespace (owns the telemetry contract):

```
SYS:TIME?        -> OK <tick_ms>          (monotonic ms since boot)
SYS:TELEM?       -> OK <schema_version>
SYS:EVENT <0|1>  -> OK                     (gate the !EVT stream; default 1)
SYS:EVLOG?       -> OK <n> <tick> <event> ... (flight recorder, Phase 4)
```

Stream header (replaces the bare `OK <count> <rateHz> ...` of `LOG:DATA?`
with a self-describing header):

```
OK STREAM <schema> <rateHz> <startTick> <count> <nChannels> <units>
# then count samples, each <tick> <setpoint> <measured> <output> ...
```

Snapshots: unchanged in shape, but every `*?` reply's field order and units are
frozen into the schema contract (Section 8) and documented in one place.

### Packet framing invariant — one line per packet

**Every telemetry packet is exactly one line.** This is a hard protocol rule,
not a convention:

- One packet = one `\r\n`-terminated line (the same terminator every reply
  already uses). No packet ever contains an embedded `\r` or `\n`.
- This applies to **all** packets: `OK`/`ERR` replies, `!EVT` events, future
  stream lines, and flight-recorder lines alike.
- A host therefore never needs length-prefixing, framing bytes, or multi-line
  reassembly — it parses by reading whole lines (`readline()`), full stop. This
  is what lets `wham_console.py`'s `query()` skip `!EVT` lines and know, by the
  line alone, what kind of packet it just read.

Companion rule — **bounded line length**: "one line" is not "one arbitrarily
huge line." A packet should be a few hundred bytes at most, so a host can size a
fixed receive buffer and the firmware's blocking `uart_send` is never held for a
megabyte-scale blob. Large datasets (the waveform stream) are therefore emitted
as a **sequence** of bounded one-line packets — a header line, then many sample
lines — not one giant line. The existing `LOG:DATA?` (which today dumps the
whole log as a single multi-KB line) is the one place that still violates this,
and is superseded by the stream tier in Phase 3.

## 6. Timing & safety isolation (the "don't disturb the shot" guarantee)

The concern — telemetry transmission during a shot must not interrupt output or
fault handling — is guaranteed by *where each piece runs*, not by hoping. The
architecture already gives us this for free; the plan makes it a stated
invariant and adds the one missing piece (non-blocking TX).

| Operation | Runs where | Priority | Can it delay the 1 kHz loop? | Can it delay fault handling? |
|---|---|---|---|---|
| Sample (waveform log) | `PID_Update()` (ISR) | 1 | already exists; tiny O(n) array writes | no |
| Event enqueue (fault/state) | fault path (ISR, inside existing critical section) | 1 | 2–3 stores only | no — it *is* the fault path |
| Format + transmit | main loop | thread (lowest) | **no** — the HRTIM Master ISR preempts it | **no** — PC10 is silicon, GateDriverStatus is EXTI, OCP/Enerpro still poll in `PID_Update()` |
| TX byte out | TXE ISR (or chunked main-loop) | 3 | no | no |

Key points to make this airtight:

1. **The control loop and faults are at priority 1 (or in silicon); telemetry
   transmission is at thread priority 0.** By NVIC rules the former always
   preempts the latter. Telemetry can *never* delay an HRTIM register write or
   a fault ISR, because it cannot run while they run.
2. **Telemetry never masks interrupts.** No `__disable_irq()` around any
   telemetry path (the only critical sections remain the pre-existing fault
   ones, and they only enqueue a 3-word event).
3. **The ISR-side cost is bounded and constant.** Sampling = the existing log
   writes. Events = a fixed 3-word push into a small ring buffer. No `snprintf`,
   no UART, no format work in the ISR.
4. **The only thing telemetry *can* delay is the main loop itself** — command
   ingestion and the *main-loop copy* of the fault poll. That is a real
   (non-safety) concern, and it is exactly what non-blocking TX fixes: it is a
   *responsiveness* requirement, not an *output/fault-interruption* risk.

**Net effect during a shot:** capture continues in the ISR with zero new
overhead; transmission is drained from the main loop at a bounded bytes/iteration
budget (or after the shot), so `SOURce:STOP` / `FAULT:CLEAR` and the main-loop
fault poll stay responsive, and the shot's own PID writes and fault handling are
provably untouched.

## 7. Non-blocking TX prerequisite

`uart_send` is currently blocking (polls TXE per byte). A large `LOG:DATA?`
dump blocks the main loop for seconds. Two options:

- **A (interim): chunked send.** `uart_send_chunked(buf, len, max_bytes)`
  transmits at most `max_bytes` then returns; the caller loops and, between
  chunks, services `uart_process` + fault poll. Small change.
- **B (target): TX ring buffer + TXE interrupt.** `uart_send` copies into a
  static ring buffer and enables the TXE interrupt; the ISR drains it. Fully
  non-blocking; the main loop is never held.

Target is B; A is acceptable if we want to defer the interrupt work. This is the
same infrastructure that would later let a main-loop-fed watchdog coexist with
large serial replies (see the WWDG discussion in changelog 2026-09-22), so it is
a shared dependency, not telemetry-only.

Verification (regardless of A or B): arm a full-size log, start a shot, and send
`SOURce:STOP` mid-dump — it must be honored within ~1 loop iteration, and
`LOG:DATA?` must still return the complete, uncorrupted log.

## 8. Schema & versioning rules

- `SYS:TELEM?` returns an integer **schema version**. Any change to a reply's
  field order, units, or the `!EVT`/stream grammar **bumps it**.
- `*IDN?` continues to carry FW version + git hash (build identity); the schema
  version is a finer-grained *data-contract* version. Both are checked by the
  host at connect.
- Every snapshot reply's field order and units are documented in one place
  (`docs/command_reference.md` gains a "telemetry schema" section keyed by
  schema version).
- Conventions frozen here: wire channel numbers are 1-based; units are explicit
  (`Hz`, `A`, `ms`, `tick`); raw pin reads stay polarity-agnostic `HIGH`/`LOW`;
  "raw vs interpreted" is always stated.

## 9. Staged implementation plan

Each phase must pass: clean build (both targets), the Section 6 isolation check,
and no regression to serial reflash.

- **Phase 0 — Non-blocking TX (prerequisite).** `uart_send` → ring buffer +
  TXE interrupt (or chunked interim). Also unblocks a future watchdog.
- **Phase 1 — Time + events.** `SYS:TIME?`; a small static event ring buffer;
  `!EVT STATE/FAULT` enqueued from the existing fault/state paths; `SYS:EVENT`
  gate. Verify with `GENERAL:TEST:FAULT` / `OCP:TEST:FAULT` during a real shot:
  the event is emitted without perturbing output (DSLogic cross-check).
- **Phase 2 — Schema contract.** `SYS:TELEM?`; freeze and document every `*?`
  reply; host-side schema module in `wham_console.py`.
- **Phase 3 — Stream standardization.** Framed `OK STREAM ...` header +
  monotonic timestamps; throttled/async transmission of the existing PID log;
  optional continuous `PID:STREAM` push mode.
- **Phase 4 — Flight recorder (stretch).** RAM (then `.noinit` for warm-reset
  retention) event log + `SYS:EVLOG?` for post-mortem.
- **Phase 5 — Host client.** Schema-aware telemetry client in `wham_console.py`:
  CSV/JSON export, wall-clock mapping, `!EVT` subscription, live plotting.

## 10. Memory budget

Static, no heap. At 128 KiB RAM (currently ~110 KiB used):

- Event ring: 256 events × 8 B (tick + type + channel + reserved) = 2 KiB.
- TX ring buffer: 1–2 KiB.
- Stream storage: reuses the existing `PID_LOG_MAX_SAMPLES` arrays (84 KiB) —
  no new allocation.
- Flight recorder (Phase 4): reuse or extend the event ring; sized in that phase.

Total new static RAM ≤ ~4 KiB before Phase 4. Re-verify with
`python3 memory_report.py` at each phase (standing instruction).

## 11. Open questions / decisions

1. Non-blocking TX: go straight to **Option B** (TXE ring buffer), or ship
   **Option A** (chunked) first?
2. Event stream default: `SYS:EVENT` ON (push by default) or OFF (opt-in)?
   (Recommend ON — faults are exactly what the host should not have to poll.)
3. Stream emission during a shot: (a) throttled to N bytes/loop-iteration, (b)
   buffer-then-flush-after-shot, or (c) continuous decimated push? (Recommend
   a+b: throttle a low-rate live stream, full dump after the shot.)
4. Flight recorder retention across reset via `.noinit` (warm-reset only), or
   RAM-only for the first cut? (Recommend RAM-only first; `.noinit` later.)
5. Do we add a checksum to the stream tier now, or defer? (Recommend defer until
   a real corruption case is observed; ASCII over 115200 has been fine.)
