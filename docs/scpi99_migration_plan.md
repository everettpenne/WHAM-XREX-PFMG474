# WHAM-XREX-PFMG474 — Path to SCPI-99 Compliance (Plan, Not Implemented)

**Status: planning only, per direct instruction (2026-09-23) — "don't pursue
changes for the SCPI-99 conversation, but do make a plan when you get
there."** Nothing in this document has been implemented. It picks up a
conversation that was interrupted when we discovered the in-progress
`PID:` → `SOURce:`/`SHOT:`/`LOG:`/`CHANnel:` rename (see
`docs/changelog.txt`, 2026-09-22/23) — that rename is a real step toward
this, but not the same thing, and this document explains why.

Reference: [SCPI-99 (IVI Foundation)](https://www.ivifoundation.org/downloads/SCPI/scpi-99.pdf).
Everything below reflects that spec as summarized during this project's
own research pass; **verify exact keyword spellings against the actual
spec text before implementing anything**, especially the `SOURce:SWEep`/
`SOURce:LIST` subsystems, which this project has no obvious use for and
weren't independently confirmed byte-for-byte.

## 1. Where the current wire protocol actually stands

The 2026-09-22 rename moved output/demand commands out of `PID:` into
`SOURce:`/`SHOT:`/`LOG:`/`CHANnel:` — a real move toward SCPI-flavored
naming, but it is **naming**, not **structure**. Two structural gaps
remain that matter more than any individual keyword's spelling:

1. **Channel addressing.** SCPI's own convention for "which one of
   several identical subsystems" is a **numeric suffix on the mnemonic
   itself** — `SOURce1:CURRent`, `SOURce2:CURRent` — not a channel
   number as the first *argument*, which is what every multi-channel
   command in this project does today (`SOURce:SETpoint <ch> <hz>`,
   `SHOT:CURRent <ch> <demandA>`, etc.). This project's own parser
   (`cmd_parser.c`'s `scpi_match()`/`scpi_token_match()`) has no concept
   of a numeric suffix on a mnemonic segment at all — it strips nothing,
   expects an exact case-insensitive short/long-form string match per
   colon-separated level. Real infrastructure work, not a rename.
2. **No status-reporting subsystem.** SCPI-99 expects a `STATus:OPERation`/
   `STATus:QUEStionable` register pair plus a `SYSTem:ERRor?` queue that a
   host drains asynchronously. This project's actual design — an inline
   `ERR <code> <msg>` as the direct reply to the failing command — is
   arguably a *better* fit for an interactive human operator (immediate,
   synchronous feedback, no polling) and is exactly what
   `docs/labview_interface.md` was just written to document. Migrating to
   a queue model would be a real behavior change, not just a rename, and
   would need a deliberate decision about whether to keep the current
   inline replies alongside it (see §5).

## 2. Proposed command tree

### 2.1 IEEE-488.2 common commands — cheapest, highest-compliance-value item

Currently only `*IDN?` exists. SCPI-99 requires (for any claim of
compliance) `*RST`, `*CLS`, `*ESE`/`*ESE?`, `*ESR?`, `*OPC`/`*OPC?`,
`*STB?`, `*WAI`, and recommends `*TST?`. None of these need the
numeric-suffix parser work below — they're self-contained, low-risk, and
could be done as an independent first phase:

- `*RST` — return every channel to its `PID_Init()` boot defaults
  without an actual reboot. Real design question: does this also
  `DISARM`/`FAULT:CLEAR`, or refuse while `ARMED`/`FIRING`? (Recommend:
  refuse while `FIRING`, matching this project's existing "never
  silently substitute a softer safety response" philosophy — see
  `EnterFault()`'s own history in `docs/changelog.txt`.)
- `*CLS`/`*ESE`/`*ESR?`/`*STB?` — only meaningful once §2.3's status
  registers exist; sequence these together.
- `*OPC`/`*OPC?`/`*WAI` — every command this project has is already
  effectively synchronous (a reply only comes back once the command has
  fully executed), so these would be near-trivial stubs (`*OPC` sets a
  flag immediately, `*OPC?` always replies `1`) rather than doing real
  work — worth having for a compliance checkbox, low value otherwise.
- `*TST?` — a real self-test would need scope (just confirm the command
  parser + HRTIM respond? Actually toggle a diagnostic output and read
  it back?) — a genuine design question, not just plumbing.

### 2.2 `SOURce<n>`/`OUTPut<n>` — the numeric-suffix channel addressing

Requires new parser infrastructure: `scpi_match()` needs to strip a
trailing digit run off the mnemonic's first colon-segment *before*
matching against the table, and hand the extracted channel number to the
handler separately from the argument list (today every handler parses
the channel as its own first `strtok()` token). This is the single
largest piece of new work in this whole plan.

Proposed mapping once that exists:

| Current | Proposed | Notes |
|---|---|---|
| `SOURce:SETpoint <ch> <hz>` | `SOURce<n>:FREQuency <hz>` | No `VOLTage` subsystem — this instrument has no voltage-source concept at all, only current (via frequency-encoded demand) |
| `SHOT:CURRent <ch> <demandA>` | `SOURce<n>:CURRent <demandA>` | The real "SOURce:CURRent" SCPI keyword — arguably where this belongs more than under `SHOT:` |
| `PID:GAINS <ch> <kp> <ki> <kd>` | `SOURce<n>:CURRent:PID:GAINs <kp>,<ki>,<kd>` (or similar) | No standard SCPI subtree for PID gains — vendor extension under `SOURce<n>`, comma-separated per SCPI's own list-argument convention |
| `PID:LOOPMODE <ch> <0\|1>` | `SOURce<n>:FUNCtion <STEP\|PID>` or `SOURce<n>:MODE <OPEN\|CLOSed>` | `SOURce:FUNCtion` is the closest real SCPI keyword (normally waveform shape); repurposing it is a stretch — a vendor-extension `:MODE` may be more honest |
| `SOURce:ENAble <ch> <0\|1>` | `OUTPut<n>[:STATe] <0\|1\|ON\|OFF>` | Exact SCPI keyword match — probably the cleanest single item in this whole table |
| `XREX:CHANnel:ENAOut`/`CONTactOut` | `OUTPut<n>:INTerlock:ENAble`/`:CONTact` (vendor extension) | Not standard SCPI (no concept of a physical enable/contactor fiber pair) — needs its own subtree either way |
| `CHANnel:NICKname <ch> <name>` | `SOURce<n>:LABel <name>` or stays as-is | Low value to move — SCPI has no real convention for a free-text label |

**`SOURce:VOLTage` does not apply anywhere in this project** — worth
stating plainly, since it's the other half of the canonical
`SOURce:CURRent`/`SOURce:VOLTage` pair and a reader familiar with SCPI
power supplies will look for it. This instrument commands current via a
frequency-encoded demand with no voltage regulation concept at all.

### 2.3 `INITiate`/`TRIGger`/`ABORt` — the strongest structural fit in this whole plan

This is worth calling out on its own: SCPI's trigger subsystem is a
near-exact conceptual match for this project's own `ARM`/`SHOT:STARt`/
`DISARM`/`EXTernal:TRIGger` state machine, arguably a *better* fit than
trying to force everything into `SOURce`:

| Current | SCPI-99 equivalent | Fit |
|---|---|---|
| `ARM` | `INITiate[:IMMediate]` | Exact — "prepare the instrument to respond to a trigger" is precisely what `ARM` does today (`IDLE` → `ARMED`, outputs still off) |
| `DISARM` | `ABORt` | Exact — SCPI's own definition of `ABORt` is "stop the trigger sequence, no matter the state" |
| `SHOT:STARt` (manual fire) | `TRIGger[:IMMediate]` | Exact — a software-issued trigger while armed |
| `EXTernal:TRIGger 1` + a PF15 rising edge | `TRIGger:SOURce EXTernal` + `TRIGger:SLOPe POSitive` | This is the standout: SCPI's trigger-source/slope model is *built* for exactly "fire on an external rising edge while armed," which this project re-invented by hand as a bespoke feature |
| (no current equivalent) | `TRIGger:SOURce BUS` | Would formalize "fire only via the serial command," matching current default behavior when `EXTernal:TRIGger` is off |

This mapping is genuinely more valuable than most of §2.2's renames and
worth prioritizing if this work resumes, independent of the numeric-
suffix parser work (it doesn't need it — `INITiate`/`ABORt`/`TRIGger`
are system-wide, no per-channel suffix required, matching how `ARM`
already works today).

### 2.4 `STATus`/`SYSTem:ERRor?` — the error-queue question

SCPI-99's model: a command that fails pushes an entry onto a bounded
error queue; `SYSTem:ERRor?` (optionally `:NEXT?`, `:COUNt?`) drains it
one at a time; the command's own reply is just an ack, not the error
text itself. This project's actual design returns the error text
*directly, synchronously*, as the reply to the failing command — a
design `docs/labview_interface.md` explicitly built around
("classify each line by its first token") and the ARM-diagnostics work
earlier this session specifically improved *within* that model (naming
the specific failed precondition in the inline reply).

**Recommendation if this is pursued: don't replace the inline model,
add the queue alongside it.** Every command keeps returning `ERR <code>
<msg>` exactly as today (nothing breaks, no host tooling needs to
change); `SendErr()` (`commands.c`) additionally pushes the same
`{code, msg}` onto a small ring buffer (same pattern telemetry.c already
established for the flight recorder); `SYSTem:ERRor?`/`SYSTem:ERRor:
COUNt?` become new, purely additive queries for compliance/scripting
convenience. Low risk, no behavior change to anything that exists today.

`STATus:OPERation`/`STATus:QUEStionable` registers are a bigger design
question: SCPI defines the *mechanism* (a condition register, an event
register, an enable register, `PTR`/`NTR` transition filters) but leaves
*which bit means what* entirely vendor-defined. This project would need
to design that bit assignment from scratch (e.g., `STATus:OPERation` bit
0 = `ARMED`, bit 1 = `FIRING`; `STATus:QUEStionable` bits per fault type)
— real design work, not just plumbing, and arguably lower value than
§2.3 given `STATE?`/the `!EVT` telemetry stream already cover the same
ground for both a polling and a push-based consumer.

## 3. What NOT to change

- **Compound command chaining with semicolons** (`SOURce1:CURRent 2.5;
  OUTPut1:STATe ON`) — SCPI allows it, this project's parser doesn't
  support it, and the telemetry work's own "one line per packet" framing
  invariant (`docs/telemetry.md`) would need real thought to reconcile
  with a single input line potentially producing multiple replies.
  Recommend explicitly skipping this — low value for a bench instrument
  with no long command sequences that need one round-trip, real risk of
  breaking the framing invariant telemetry now depends on.
- **`SOURce:SWEep`/`SOURce:LIST`** — real SCPI-99 subsystems for
  frequency/amplitude sweeps and arbitrary waveform lists, but this
  project's actual ramp shape (ramp-up/flat-top/ramp-down,
  `SHOT:TIMing`) doesn't map cleanly onto either without distorting one
  or the other. `SOURce<n>:CURRent:RAMP` (a vendor extension, not
  standard SCPI) is the more honest choice — don't force-fit the
  standard subsystem just to claim compliance for something that isn't
  the same shape.

## 4. Backward compatibility / rollout strategy

Three options, same tradeoff already discussed for the `PID:` rename
itself:

1. **Hard cutover** — rename everything in one pass, break `wham_console.py`/
   `wham_llm_console.py`/`run_simulator_validation.py`/`dslogic_shot_capture.py`/
   `docs/labview_interface.md` simultaneously. Cleanest end state, highest
   coordinated-change risk (this is a small in-house bench instrument, not
   a product with external API-stability customers, so the "never break
   userspace" argument against this is weaker here than usual — still
   real risk given how much host tooling now exists).
2. **Alias, dual-support** — this project's own `command_table[]` is
   already a flat, trivially-extensible array (`scpi_match()` doesn't
   care how many rows point at the same handler) — adding new-spelling
   rows alongside the existing ones is cheap. Old names keep working
   indefinitely or for a stated deprecation window; new tooling/docs use
   the new names going forward.
3. **New-parallel** — build the new tree as pure *additions*, keep
   documenting/recommending the current names as primary, mark the new
   ones as "SCPI-flavored, for tooling that wants stricter compliance."
   Lowest risk, never actually retires anything, may never fully "finish."

**Recommendation, unchanged from the earlier conversation:** option 2
(alias/dual-support), same reasoning as before — this project has
already shown (via the `PID:` rename's own review) that a hand-edited,
no-alias "clean cut" rename is real, nontrivial work to get right and
verify; doing that twice in short succession, for a project with this
much already-built host tooling, is a much bigger ask than the
compliance value likely justifies on its own.

## 5. Open decisions needing a real answer before any implementation

1. **Full strict compliance, or "SCPI-flavored, pragmatic subset"?**
   Recommend the latter — this is a bench instrument, not a certified
   product; §2.4's inline-error-model-plus-queue compromise already
   assumes this answer.
2. **Numeric-suffix channels (`SOURce1`) vs. keeping channel-as-first-
   argument?** This is the one item that actually requires new parser
   infrastructure (§2.2) — worth deciding whether that infrastructure
   investment is worth it before committing to the rest of §2.2's table,
   since everything else in this plan can proceed without it.
3. **Is §2.3 (`INITiate`/`TRIGger`/`ABORt`) worth doing on its own,
   independent of the rest?** It's the highest-fit, lowest-new-
   infrastructure item in this whole plan (no numeric-suffix parser
   needed) — could reasonably be phased first, alone, regardless of what
   happens with §2.2/§2.4.
4. **Priority/timeline** — is this a real near-term project, or a
   someday/backlog item? Given how much time the `PID:` rename review +
   fixes + hardware verification just took for a *smaller* scope than
   this whole plan, treat as a multi-session undertaking if pursued at
   all, not a single sitting.
