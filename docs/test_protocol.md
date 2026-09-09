# WHAM-XREX-PFMG474 — Test Protocol

Revision: 2026-09-04. Covers firmware through: the serial command
architecture (SCPI-style dispatch), `*IDN?`, `BOOT`/serial reflash
(including the `SCB->VTOR`/`SYSCFG->MEMRMP` Go-jump bugfix), PFM table
upload (`TABLE:BEGIN/STEP/END/?`), and `FIRE` (HRTIM master-interrupt-
driven playback). See `docs/changelog.txt` for the dated history behind
each of these.

Conventions: every serial command ends CRLF at **115200 8N1** (raised from 9600 on 2026-09-04, see `docs/changelog.txt`); responses
are `OK …` or `ERR <n> <msg>`. Tests are numbered `T<group>.<n>`; run
groups in order the first time. This protocol is still deliberately
much shorter than the sibling PFM-STM32G474 project's — **this firmware
has no state machine, no ARM precondition, and no fault gating.** `FIRE`
takes effect immediately, unconditionally, once a table is uploaded.
Don't extrapolate test groups from that project onto this one; the
groups below are only the ones that actually apply.

**Safety:** as of 2026-09-04, `HRTIM1_FullInit()` and `PFM_Init()` run
unconditionally at every boot (see `main.c`), and the HRTIM master
interrupt is armed at boot too (`HRTIM1_EnableMasterInterrupt()`) — but
none of that starts the Master counter or enables any output. **T0–T3
involve no code path that starts PWM output** — the board cannot drive
PA8/9/10/11 or PB12/13 during those tests regardless of what's
connected to them, because nothing calls `HRTIM1_PWM_Start()` (directly
or via `PFM_Restart()`) until `FIRE` is sent. **T4 is different**: it
sends a real `FIRE` and drives real voltage transitions on those six
pins the moment it runs. Treat any gate driver or power stage wiring on
those pins as live for the duration of T4, per your lab's normal
HV/gate-driver procedure, even though this firmware has no closed-loop
or fault protection behind it yet.

---

## T0 — Build & flash sanity

| # | Step | Expect |
|---|---|---|
| T0.1 | Build in CubeIDE (Debug config), or `cd Debug && make all -j4` | No errors, no warnings. |
| T0.2 | `arm-none-eabi-objcopy -O binary Debug/WHAM-XREX-PFMG474.elf Debug/WHAM-XREX-PFMG474.bin` | Produces a `.bin` — this project's build config does not do this automatically (see `docs/serial_reflash_guide.md`). |
| T0.3 | Check `.map` (or `arm-none-eabi-nm`) for `g_pfmTable` | **Present**, at 40000 B (5000 × 8-byte `PFM_Step_t`), in `.bss` — as of 2026-09-04, `main.c` calls `PFM_Init()` unconditionally at boot, so `pfm.c` is always linked in on a normal build. (Before that date this row read "absent" — if you're comparing against an old build, that's why.) |
| T0.4 | Flash (ST-Link via CubeIDE, or `python/wham_serial_flash.py` once a `BOOT`-capable build is already on the chip — see `docs/sop/wham_pfmg474_v4_sop.tex`) | Board boots; no spontaneous serial output. |

## T1 — Serial link & identity

| # | Step | Expect |
|---|---|---|
| T1.1 | `*IDN?` | `OK WHAM-XREX-PFMG474 <rev> <version>` (fields from `version.h`). |
| T1.2 | `BOGUS` | `ERR 1 Unknown command`. |
| T1.3 | `*idn?` (lowercase) | Same reply as T1.1 — dispatch is case-insensitive (see `docs/command_reference.md`). |
| T1.4 | `IDN?` (missing the leading `*`) | `ERR 1` — `*` is part of the mandatory short form for this common command, not optional. |

## T2 — Serial bootloader / reflash

Prereq: a `BOOT`-capable build already on the chip (see T0.4 / the SOP's
bootstrap-flash procedure if starting from a board that predates
`boot_jump.c`).

| # | Step | Expect |
|---|---|---|
| T2.1 | Send `BOOT` directly (any terminal) while the board is idle | `OK ENTERING BOOTLOADER`, then the link drops as the MCU resets into the ROM bootloader. |
| T2.2 | `stm32flash <port>` (info query only, no `-w`) against the now-bootloader-resident chip | Reports Device ID `0x0469` (STM32G47xxx/48xxx) — confirms the physical link and bootloader entry independent of any application firmware. |
| T2.3 | Power-cycle to recover, then run `python3 python/wham_serial_flash.py --port <port>` end-to-end with a version-bumped build | Script reports `BOOT` ACK'd, then `[done] firmware written and verified.` |
| T2.4 | **Immediately** after T2.3 (no manual reset), query `*IDN?` | Must return the **new** version on the first query. **Getting no reply here — needing a manual reset to recover — is the Go-jump/VTOR regression** (see `docs/changelog.txt`, 2026-08-31 bugfix entry, and `boot_jump.c`'s `BootJump_CheckAndEnter()`). Report it, don't work around it. |
| T2.5 | Repeat T2.3–T2.4 once more, back to back | Same clean result both times — this was flaky-looking before the fix (worked once, needed a reset the next), so a single pass isn't sufficient evidence. |
| T2.6 | `BOOT` while a build has `BOOT_JUMP_FEATURE_ENABLED` set to `0` (`boot_jump.h`) | `ERR 1 Unknown command` — confirms the module's removability actually removes the command, not just disables its effect. |

## T3 — PWM/HRTIM engine, direct bypass (developer fallback only)

**Superseded by T4 below as the normal way to exercise PWM output** —
`FIRE` (via a real `TABLE:*` upload) is now the real, non-developer
path, and doesn't have the zero-initialized-table trap this section
used to warn about (a real uploaded table has real, non-zero entries).
Keep this section only as a low-level fallback for bring-up work on
`hrtim.c` itself, in isolation from `pfm.c`/`cmd_parser.c` — e.g. if
you suspect the fault is in the table/command layer and want to rule
out `hrtim.c` independently, or if `pfm.c` won't build for some
unrelated reason. Requires a temporary firmware change; not something a
non-developer should run, and the edit must be reverted afterward.

Since `main.c` already calls `HRTIM1_FullInit()` unconditionally at
boot (as of 2026-09-04), do **not** add a second call to it — only add
the direct `HRTIM1_PWM_Start()` call, bypassing `PFM_Restart()`/`FIRE`/
the command layer entirely.

| # | Step | Expect |
|---|---|---|
| T3.1 | In `main.c`, in `USER CODE BEGIN 2` (anywhere after `MX_HRTIM1_Init()` has run), temporarily add `HRTIM1_PWM_Start(1, 1, 1);` | Builds clean. |
| T3.2 | Flash T3.1's build. Scope TA1/TA2 (PA8/PA9), TB1/TB2 (PA10/PA11), TC1/TC2 (PB12/PB13) | All three pairs: ~100 kHz complementary square wave, ~50 % duty, ~100 ns dead time between an edge on one output and the corresponding edge on its complement — `HRTIM1_FullInit()`'s own init defaults, since nothing (no `PFM_Restart()`) has overwritten them. |
| T3.3 | Scope U (PA8) vs. V (PA10) vs. W (PB12) rising edges relative to each other | 120° / 240° spacing (≈3.33 µs / 6.67 µs apart at 100 kHz) — confirms the Master `PER`/`CMP1`/`CMP2` phase-trigger scheme, not just that each pair individually toggles. |
| T3.4 | Revert `main.c` to the pre-T3.1 state; rebuild | Back to a normal build (`HRTIM1_PWM_Start()` not called anywhere, no output starts on its own). **Do not leave T3.1's edit in place** — it was a one-off verification, not a real integration. |

## T4 — PFM table upload + FIRE (end-to-end, real command path)

The normal, non-developer way to drive PWM output. No firmware edits.
Prereq: `python/pfm_table_upload.py` runnable from the host (`pip
install pyserial` if needed) — see that script's header for `PROFILE`
selection.

| # | Step | Expect |
|---|---|---|
| T4.1 | `TABLE?` (before any upload, fresh boot) | `OK 0` — table starts empty every boot (`PFM_Init()`, not persisted across resets). |
| T4.2 | `FIRE` (against the still-empty table from T4.1) | `ERR 5 Table is empty …` — no output pins move. This is the guard `cmd_fire()` adds specifically so an empty-table `FIRE` fails loudly instead of silently blipping outputs on and off for one period. |
| T4.3 | With `PROFILE = 1` (constant 100 kHz/50 % duty) in `pfm_table_upload.py`, run `python3 python/pfm_table_upload.py --port <port>` | `[upload] TABLE:BEGIN ok, uploading...` … `[upload] TABLE:END -> OK 500` … `[done] table uploaded and confirmed.` |
| T4.4 | `TABLE?` | `OK 500`, confirming the upload persisted after the script exited (upload session state is protocol-layer only — the table itself lives in `pfm.c` regardless). |
| T4.5 | Scope TA1/TA2, TB1/TB2, TC1/TC2 (same six pins as T3.2), then send `FIRE` | `OK`, then all three pairs start switching **within one command turnaround** — ~100 kHz complementary, ~50 % duty, ~100 ns dead time, 120°/240° U/V/W phase spacing (same waveform T3.2/T3.3 characterized, now reached through the real command path instead of a firmware edit). The very first visible edge is delayed by up to one Master period (~10 µs at 100 kHz) versus `FIRE`'s `OK` reply — `HRTIM1_PWM_Start()` deliberately waits for phase-lock to settle in hardware before connecting the outputs to the pins, see `docs/changelog.txt`'s cold-start phase-lock entry (2026-09-04) for why; U/V/W's first edges landing at the exact same instant, not staggered, is the regression to watch for if this ever breaks again. |
| T4.6 | Let it run — do not send `FIRE` again | Output stops **on its own** at a coherent boundary once all 500 entries have played (500 periods at 100 kHz ≈ 5 ms after T4.5's `FIRE` — easy to miss on a scope without a decent trigger/timebase; a current probe or persistence mode helps). No reply or notification marks this; it's silent by design (see `docs/command_reference.md`'s `FIRE` entry — no `STOP` command exists to need it either). |
| T4.7 | Re-run T4.3 with `PROFILE = 2` (25→75 % ramp), then repeat T4.5 | Duty visibly increases over the ~5 ms shot — confirms per-entry playback advance (`PFM_CycleBoundaryHandler()`), not just a static waveform. Only a fixed table `per` (100 kHz) is exercised by all three profiles; this checks the *duty* dimension specifically. |
| T4.8 | Send `FIRE` again while a shot from T4.5/T4.7 is still running (mid-shot) | `OK` — restarts playback from step 0 immediately. No `ERR`/rejection: there is no ARM/interlock concept in this firmware (see `docs/command_reference.md`) — confirms that's actually true, not just documented. |

---

## Regression quick-list (run after any firmware change)

1. T1.1 (identity)
2. T2.3–T2.5 (reflash round trip + the Go-jump regression check specifically)
3. T3.1–T3.3 only if `hrtim.c` changed in isolation and you need to rule
   it out independently of `pfm.c`/the command layer — otherwise T4
   below already exercises the same waveform through the real path.
4. T4.1–T4.6 if `hrtim.c`, `pfm.c`, `commands.c`, or `cmd_parser.c`
   changed — the full upload → FIRE → auto-stop round trip.
