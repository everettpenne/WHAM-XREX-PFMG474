# Rebuilding WHAM-XREX-PFMG474 from Scratch — A Step-by-Step Plan

**Scope:** this document describes how I would rebuild this project's
*firmware and host tooling* from a clean start, on the **same PCB** (the
STM32G474QET6 board with its existing pin mapping), while preserving every
user-facing command, every controller output behavior, and every host-side
workflow exactly as they exist today.

**Status:** plan/proposal. Decisions I would want confirmed before execution
are collected in "Open decisions" (Section 10). Nothing here implies the
current code is wrong — it works and is exceptionally well documented — only
that it has accumulated structural debt (a fragile CubeMX build, a 3,400-line
`commands.c`, no host test harness, state smeared across modules) that a clean
rebuild would remove.

---

## 1. Guiding principles

1. **Same hardware, same pin map, same wire behavior.** This is a software
   rebuild. The PCB is fixed, so every hardware-derived constraint carries
   over verbatim: the OCP/GateDriverStatus EXTI-line conflict, `PG10` being
   tied to `NRST`, the `PF13`/`PF15` interlock/trigger pins, the 170 MHz clock,
   the HRTIM "Possibility 3" per-channel shadow-update model.

2. **User-facing compatibility is non-negotiable.** The full SCPI command
   surface, the `OK`/`ERR` reply convention, the error-code numbering, the
   `*IDN?` format, the `!BOOT` banner, the `BOOT` command, the `.bin`
   artifact names, and the `FWUPdate:*` protocol must be byte-for-byte
   compatible, so that `wham_console.py`, `wham_llm_console.py`,
   `wham_serial_flash.py`, `fw_update.py`, and the network tools keep working
   without modification.

3. **Pure logic is separated from hardware and unit-tested on the host.**
   The control math, state machine, SCPI parser, and firmware-update protocol
   logic compile on the host and run under a committed test suite.

4. **One canonical source of truth for system state.** No more
   `g_running`/`g_profileActive`/`g_state` agreement-by-comment.

5. **The build is reproducible and regenerable.** No hand-patched
   CubeMX makefiles, no hardcoded toolchain path, no "don't touch HRTIM in
   the GUI or it breaks" rules.

---

## 2. Functional inventory — what must be preserved

This is the acceptance contract for the rebuild. If something here is not
listed, it is out of scope; if it is listed, it must survive.

### 2.1 Controller output (hardware behavior)

| Behavior | Source today | Must be preserved |
|---|---|---|
| 4 independent PFM demand channels (HRTIM Timers A–D, `HRTIM_NUM_CHANNELS=4`) | `hrtim.c`, `pfm.c` | Yes |
| Per-channel shadow update on that channel's own roll-over (Possibility 3: `ResetTrigger=NONE`/`UpdateTrigger=NONE`/`ResetUpdate=ENABLED`) | `hrtim.c` `HRTIM1_FullInit()` | Yes |
| Fixed-rate Master heartbeat at `PID_LOOP_RATE_HZ` (1 kHz) driving `PID_Update()` | `hrtim.c`, `pid.c` | Yes |
| Closed-loop PID per channel (Kp/Ki/Kd, fixed Δt, frequency-domain, derivative-on-measurement, directional clamp anti-windup) | `pid.c` `PID_Update()` | Yes — bit-for-bit numeric behavior |
| Open-loop mode (setpoint straight to HRTIM, feedback still reported) | `pid.c` | Yes |
| Trapezoidal demand profile (ramp-up / flat-top / ramp-down, shared clock, per-channel Amps) | `pid.c` `TrapezoidalCurrentA()`/`AmpsToHz()` | Yes |
| Output slew-rate clamp (`PID_OUTPUT_MAX_SLEW_HZ_PER_TICK`) | `pid.c` `ClampOutputSlew()` | Yes |
| Per-channel output enable/disable (hardware disconnect) | `pid.c` `PID_SetChannelEnable()` | Yes |
| General Fault → open-loop ramp-to-floor over `FAULT_RAMP_DOWN_TIME_S`, then stop | `state_machine.c` `HandleGeneralFault()` | Yes |
| OCP fault → faulted channel hard-disabled, survivors derated `(100·1/N)%` then ramped | `state_machine.c` `HandleOvercurrentFault()` | Yes (note: OCP stays **polled** — see §3) |
| ENABLE_OUTPUT / ENERPRO faults (shared OCP-shaped response) | `state_machine.c` | Yes |
| EXTERNAL_ENABLE fault (PF13 loss while FIRING) | `state_machine.c` | Yes |
| Silicon-level PC10/HRTIM1_FLT6 fault gating (autonomous, not firmware) | `hrtim.c` | Yes (hardware) |
| State machine IDLE/ARMED/FIRING/FAULT with ARM/DISARM/SHOT:STARt/SOURce:RUN/SOURce:STOP semantics | `state_machine.c` | Yes |
| External enable (PF13) + external trigger (PF15, rising edge fires from ARMED) | `state_machine.c` | Yes |
| PFM input capture (TIM2–TIM5, period-only, continuous/free-running + bounded bench path) | `pfm_input.c` | Yes |
| Waveform logging (1-channel or all-channels, decimation, exact timebase) | `pid.c` | Yes |
| QSPI JEDEC Read ID test (W25Q128JVS) | `qspi_test.c` | Yes |
| `!EVT` telemetry events | `telemetry.c` | Yes |
| `!BOOT` banner (3 lines) + reset-surviving boot diagnostics | `main.c`, `boot_diag.h` | Yes |
| Dual-bank in-application firmware update (erase inactive bank, stream, CRC, sanity-check, `BFB2` swap, rollback) | `fw_update.c` | Yes |
| `BOOT` → ROM bootloader entry + VTOR/MEMRMP restore | `boot_jump.c` | Yes |

### 2.2 User interface (serial + host tooling)

**SCPI command surface** (complete, from `cmd_parser.c`'s table) — every
mnemonic, short/long-form rule, `?`-query rule, and the `ERR 1..17` codes
(with 16 retired) must survive:

- `*IDN?`
- `BOOT`
- `TABle:BEGin|STEP|END`, `TABle?`
- `CONFig:MaxCarrierHz|CHANnels?|PIDRate|TURNONHz|MAXFREQHz|MAXCURRent|SLEWRate|FaultRampTime|FaultPolarity:(WATER|TEMP|ENERPRO|OCP)` (each with `?` form)
- `FIRE`
- `PFM:DIAG?`, `PFM:GAPLOG?`
- `FAULT?`, `FAULT:CLEar`
- `ARM`, `DISARM`, `STATE?`
- `SYS:TIME?`, `SYS:TELEM?`, `SYS:EVENT`, `SYS:EVENT?`, `SYS:EVLOG?`
- `FWUPdate:BEGin|DATA|END|SWAP|ROLLback|ABORt|STATus?`
- `DEBUG:FAULT:BYPASS`, `DEBUG:FAULT:BYPASS?`
- `EXTernal:ENAble|ENAble?|INPut?|TRIGger|TRIGger?|TRIGger:INPut?`
- `DIAGnostic:GPOut09|GPOut10|GPOut11|GPOut12` (each with `?`), `OPTBytes?`, `RSTCause?`, `RSTCause:CLEar`
- `OCP:TEST:FAULT`, `GENERAL:TEST:FAULT`
- `GDS?`
- `XREX:CHANnel:STATus?`, `XREX:CHANnel:ENAOut|CONTactOut` (each with `?`)
- `GPOut:ENAble`, `PWMAlt:ENAble` (each with `?`)
- `QSPI:ID?`
- `PFMIN:CAPTURE|STATus?|DMASTAT?|DEBUG:RAW?|DEBUG:REG?|DATA?`
- `SOURce:RUN|STOP|SETpoint|STATus?|RAMP|ENAble|ENAble?`
- `PID:GAINS|GAINS?|LOOPMODE|LOOPMODE?`
- `LOG:ARM`, `LOG:DATA?`
- `CHANnel:NICKname|NICKname?`
- `SHOT:TIMing|TIMing?|CURRent|CURRent?|STARt`
- `SIM:*` (simulator-only: `FAULT:WATERTEMP`, `FAULT:ENERPRO`, `FAULT:OCP`, `MODEL:TAU`, `DIAGnostic:IDLETONE`, `CHANnel:STATus?`, `LOG`, `LOGDATA?`)

**Host tooling** (must keep working unmodified, or be adapted with the same
CLI):

| Tool | Compatibility requirement |
|---|---|
| `wham_build.py` | same `--target controller\|simulator`, `--clean`, `--no-git-version` flags; same `.bin` output names |
| `gen_git_version.py`, `gen_build_target.py` | same generated headers |
| `wham_serial_flash.py` | same `--target`, `--port`, `--bin`, `BOOT`+stm32flash flow |
| `fw_update.py` | same `--target`, `--url`, `--status`, `--no-swap`, `--rollback`, `--name`; depends on `FWUPdate:*` + `*IDN?` format + `!BOOT` banner |
| `wham_console.py`, `wham_llm_console.py`, `scpi.py` | depend only on the SCPI protocol + `!EVT`/`!BOOT` |
| `pfm_table_upload.py`, `pfm_input_plot.py` | depend on `TABLE:*`, `PFMIN:*`, `CONFig:CHANnels?` |
| `run_simulator_validation.py` | depends on the full command surface + simulator behavior |
| `net_broker.py`, `net_client.py`, `net_terminal.py`, `net_flash.py` | transport only — no firmware dependency except `BOOT`/`FWUPdate` for flash |
| `memory_report.py`, `dsl_viewer.py`, `dslogic_shot_capture.py`, `md_to_pdf.py` | host-only; no firmware dependency |

---

## 3. Hardware constraints that carry over unchanged

Because this is the same PCB, these facts are inputs, not choices:

1. **Pin mapping is fixed** (`.ioc`): HRTIM `TA1/2`=PA8/PA9, `TB1/2`=PA10/PA11,
   `TC1/2`=PB12/PB13, `TD1/2`=PB14/PB15, `TE1/2`=PC8/PC9, `TF1/2`=PC6/PC7;
   USART2=PA2/PA3; GateDriverStatus=PE0–PE11 (EXTI); OCP=PF4/PF5/PF8/PF12;
   ENA_OUT/CONTACT_OUT=PG0–PG7; interlock PF13, trigger PF15; PC10=HRTIM1_FLT6.
2. **OCP is polled, not EXTI-driven**, because PF4/PF8/PF12's EXTI lines collide
   with the GateDriverStatus EXTI on PE4/PE8/PE12. A true OCP interrupt would
   require a **PCB change** — out of scope. The rebuild keeps OCP polling and
   documents it as a deliberate latency characteristic.
3. **`PG10` is `NRST`**, not a usable GPIO — never configure it as GPIO.
4. **`NRST_MODE=11` + `IRHEN=1` option bytes** caused the post-swap reset hang.
   The rebuild sets the option bytes deliberately (see §6 Phase 7) and relies
   on the external NRST pull-up that was added on the bench.
5. **128 KiB SRAM / 512 KiB flash (2×256 KiB dual bank).** The log arrays and
   table share this budget; the rebuild keeps the same sizing math and the
   `memory_report.py` tracking.
6. **170 MHz HSI/PLL clock**, `FLASH_LATENCY_8`, voltage scale BOOST.

---

## 4. Target architecture

Three layers, one-way dependencies:

```
app/            controller/      simulator/      ← thin: command handlers + wiring
                  (main, cmd handlers, shot, source, ...)
core/           pid  state_machine  scpi  fwupdate  profile  log  telemetry
                  (pure logic, NO HAL includes, host-compilable)
platform/       clock  gpio  hrtim  timer  usart  flash  qspi
                  (the ONLY files that #include HAL)
```

- **`platform/`** owns registers/HAL. Exposes small, explicit APIs
  (`hrtim_set_period(ch, per)`, `gpio_read_pin(...)`, `usart_putline(...)`).
- **`core/`** owns logic. `pid.c` becomes a pure function of
  `(setpoint, measured, gains, dt) → (output, new_state)`; `state_machine.c`
  becomes a pure function of `(state, inputs) → (next_state, actions)`;
  `scpi.c` is a pure matcher; `fwupdate.c` is a protocol state machine whose
  flash access is injected.
- **`app/`** owns the two targets. `controller/` and `simulator/` differ only
  in `main.c` composition and the `SIM:*` handlers; there are no `#if
  BUILD_TARGET_SIMULATOR` sprinkles in shared files.

The SCPI **wire contract** (patterns, `OK`/`ERR`, error codes, `*IDN?`,
`!BOOT`, `!EVT`) is defined in one place (a header or the parser table) and is
treated as frozen.

---

## 5. Step-by-step rebuild process

Each phase ends with "Acceptance" — the concrete, verifiable thing that must
be true before moving on. Phases are ordered so the project builds and runs
at every step (no big-bang rewrite).

### Phase 0 — Toolchain and build system (CMake)

**Steps**
1. Create a `CMakeLists.txt` and an `arm-none-eabi.cmake` toolchain file.
   Locate `arm-none-eabi-gcc` via an environment variable
   (`ARM_TOOLCHAIN_DIR`) or `direnv`, pointing at the CubeIDE-bundled 13.3
   toolchain (or Homebrew) — never a hardcoded app-bundle path.
2. Add a `tools/` helper (or reuse `wham_build.py`'s spirit) that emits
   `git_version.h` (commit + dirty) and a build-target selector
   (`controller`/`simulator`) as a **generated compile definition or header**,
   preserving the exact `*IDN?` format.
3. Enable `-O2` (release) / `-Og` (debug), `-ffunction-sections`,
   `-fdata-sections`, `--gc-sections`, `-Wall -Wextra -Werror`, and **no**
   `-u _printf_float` (float formatting is done as scaled integers on the wire).
4. Produce `WHAM-XREX-PFMG474.bin` and `WHAM-XREX-PFMG474-SIM.bin` with the
   exact names today's tools expect; delete the plain `.bin` after a simulator
   build (preserve the existing "never leave an ambiguous .bin" safety rule).

**Acceptance:** a fresh clone → `cmake --build` → both `.bin` artifacts,
byte-identical `*IDN?` string to today's build, flashable via the existing
ST-Link and `wham_serial_flash.py` paths. `memory_report.py` runs unchanged.

### Phase 1 — Platform layer (HAL wrappers)

**Steps**
1. Port `SystemClock_Config()` (170 MHz, `FLASH_LATENCY_8`, BOOST) verbatim —
   the timing model (`HRTIM_TIMER_CLK_HZ`, the `per`↔Hz conversions, the
   17-count dead time) depends on exactly this derivation.
2. Port `MX_GPIO_Init()` **into `platform/gpio.c`** with the exact pin configs
   and the "drive level before enabling the output" convention. This is the
   right home for the long per-pin safety comments, trimmed to invariants.
3. Port `HRTIM1_FullInit()` into `platform/hrtim.c` unchanged in behavior —
   including the PC10/FLT6 silicon fault config, the CMP3-vs-TIMPER SET/RESET
   collision fix, and the per-channel `ResetTrigger=NONE`/`ResetUpdate=ENABLED`
   setup.
4. Port the TIM2–TIM5 input-capture init into `platform/timer.c`.
5. Port USART2 init (115200 8N1) + the interrupt-driven single-byte RX into
   `platform/usart.c`, keeping the NVIC priority scheme (SysTick 0, HRTIM/EXTI
   1, USART2 2).

**Acceptance:** `hrtim.c`/`pfm_input.c`/`uart.c` compile against the platform
layer; a bare "start HRTIM + send a `!BOOT` banner + echo serial" smoke build
runs on hardware.

### Phase 2 — Core logic, extracted and host-tested

**Steps**
1. Extract `pid.c` into `core/pid.c` as a pure module: `PID_Step(ch, setpoint,
   measured, dt)` returns `output` and mutates only its own per-channel struct.
   Move `TrapezoidalCurrentA`, `AmpsToHz`, `ClampOutputSlew`,
   `ClampOutputRangeOnly`, and the anti-windup into it.
2. Extract `state_machine.c` into `core/state_machine.c` with **injected
   inputs** (the fault/interlock/trigger reads become function pointers or
   a struct of booleans), so it is testable without hardware.
3. Extract `cmd_parser.c`'s `scpi_match`/`scpi_token_match` into `core/scpi.c`.
4. Extract the `FWUPdate` protocol state machine into `core/fwupdate.c`, with
   flash operations injected.
5. Write a host test target (CMake `add_executable` compiling these with the
   host compiler):
   - `pid`: ramp lands exactly on `endHz`; anti-windup does not deadlock at
     `PID_OUTPUT_MIN_HZ`; slew clamp bounds a glitch; open-loop ignores
     feedback but still reports it.
   - `state_machine`: every legal transition; every fault type's response;
     the double-entry race is impossible.
   - `scpi`: the short/long-form/query match table.
   - `fwupdate`: the begin/data/end/swap/rollback protocol against a mock
     flash (this is the host emulator from the changelog, now committed).

**Acceptance:** `ctest` green on host; the numeric behavior matches the
current firmware (golden vectors captured from today's build or from
`docs/changelog.txt`'s recorded numbers).

### Phase 3 — Application wiring (controller target)

**Steps**
1. Write a thin `app/controller/main.c` that mirrors today's boot sequence:
   `BootJump_CheckAndEnter()` first → boot-diag snapshot → `HAL_Init` →
   clock → GPIO → HRTIM → USART → `PFM_Init` → fault poll → QSPI → PFMIN →
   PID init → telemetry init → state-machine init → `!BOOT` banner → main loop
   (`SM_PollFaults` + `XrexIo_Poll*` + `Telemetry_PollEmit` +
   `uart_process`).
2. Split `commands.c` (3,423 lines) into per-subsystem files under
   `app/controller/commands/`: `source.c`, `shot.c`, `pid.c`, `log.c`,
   `channel.c`, `config.c`, `fault.c`, `diag.c`, `xrex.c`, `gds.c`,
   `pfmin.c`, `qspi.c`, `sys.c`, `table.c`, `fwup.c`. Each keeps the same
   handler signatures (`void cmd_x(uart_instance_t*, char* args)`) and the
   same `SendOk`/`SendErr` helpers, so the reply bytes are unchanged.
3. Wire the command table (one `command_table[]`, unchanged patterns) into
   `core/scpi.c`'s dispatch.

**Acceptance:** the full command surface answers identically (a recorded
script of every command against today's build replays with identical output
against the new build).

### Phase 4 — Firmware update and boot

**Steps**
1. Port `fw_update.c` onto `core/fwupdate.c` + a `platform/flash.c` adapter,
   preserving the dual-bank erase/write/CRC/sanity-check (SP in SRAM, Thumb
   reset vector, product-name match) and `BFB2` swap + rollback.
2. Keep the `!BOOT` banner and `boot_diag` so `fw_update.py`'s reboot
   detection and the reset-cause diagnostics are unchanged.
3. Keep the product-name strings (`WHAM-XREX-PFMG474` / `WHAM-XREX-PFMG474-SIM`)
   so `fw_update.py`'s cross-flash guard still works.

**Acceptance:** `fw_update.py` flashes both bank directions and rolls back
against the new build, with the same `OK ERASED …`, `OK VERIFIED CRC=…`,
`OK SWAPPING …` replies.

### Phase 5 — Host tooling adaptation

**Steps**
1. Rewrite `wham_build.py` to drive CMake (same CLI), keeping
   `gen_git_version.py`/`gen_build_target.py`'s output format.
2. Leave `fw_update.py`, `wham_serial_flash.py`, `wham_console.py`,
   `wham_llm_console.py`, `net_*`, `pfm_table_upload.py`, `pfm_input_plot.py`,
   `run_simulator_validation.py`, `memory_report.py` **unchanged** — they speak
   the SCPI/wire protocol, which has not changed.

**Acceptance:** every host tool runs against the new firmware with no edits
(other than `wham_build.py`), and the network flash/broker flows work exactly
as demonstrated this session.

### Phase 6 — Simulator target

**Steps**
1. Implement `app/simulator/main.c` (composition includes `sim_transrex.c`)
   and move every `SIM:*` handler into `app/simulator/commands/`. No `#if
   BUILD_TARGET_SIMULATOR` in shared files — the simulator target simply
   compiles a different `main` + an extra handler file.

**Acceptance:** `run_simulator_validation.py` passes against the simulator
build; `wham_build.py --target simulator` produces `WHAM-XREX-PFMG474-SIM.bin`
and deletes the plain `.bin`.

### Phase 7 — Safety hardening

**Steps**
1. Enable **IWDG** (independent watchdog), serviced only in the main loop and
   gated off during `FWUPdate:SWAP`/option-byte reload so a legitimate swap
   doesn't trip it. `Error_Handler` stops servicing so it resets.
2. Set option bytes deliberately: `NRST_MODE=01` (reset input only) or
   `IRHEN=0`, so the option-byte reload no longer holds NRST — the *real*
   root fix for the post-swap hang, independent of the bench pull-up.
3. Make `DEBUG:FAULT:BYPASS` compile-time-gated to a debug configuration (or
   require a physical jumper), so it cannot be left on in a fielded controller.
4. Re-verify the fault ramp-down paths (General and OCP) on hardware, since
   this is the safety-critical core.

**Acceptance:** a `FWUPdate:SWAP` self-boots with the ST-Link **and** the
bench pull-up removed; a hung main loop resets via IWDG; the bypass is absent
from a release build.

### Phase 8 — Full regression and documentation

**Steps**
1. Replay the complete command-replay script (Phase 3) and the simulator
   validation campaign (Phase 6) against both targets.
2. Run `memory_report.py history` + `report` and confirm the footprint is
   no larger than today's (ideally smaller at `-O2`).
3. Update `docs/command_reference.md` (it should need almost no changes — the
   wire contract is unchanged), `AGENTS.md`, and `docs/changelog.txt`.

**Acceptance:** every test in the suite passes; the command reference is
unchanged except for internal file-path references.

---

## 6. Functionality preservation matrix

| Today's subsystem | Rebuild home | Compatibility method |
|---|---|---|
| HRTIM engine (`hrtim.c`) | `platform/hrtim.c` | ported behavior-identical; same pin/timing constants |
| PFM table (`pfm.c`) + `TABLE:*`/`FIRE` | `platform/hrtim.c` + `app/controller/commands/table.c` | kept as the legacy bench path |
| PID loop (`pid.c`) | `core/pid.c` + `platform/hrtim.c` adapter | numeric behavior preserved via golden-vector tests |
| State machine (`state_machine.c`) | `core/state_machine.c` + injected I/O | all transitions/fault responses preserved via tests |
| Command parser (`cmd_parser.c`) | `core/scpi.c` + frozen `command_table[]` | byte-identical replies |
| Command handlers (`commands.c`) | `app/*/commands/*.c` | same handlers, split by subsystem |
| Fault inputs (`gate_driver.c`, `xrex_io.c`) | `platform/gpio.c` + `core/state_machine.c` | same pins, same polarity config |
| PFM input (`pfm_input.c`) | `platform/timer.c` | same TIM2–5 capture |
| Firmware update (`fw_update.c`) | `core/fwupdate.c` + `platform/flash.c` | same protocol + `!BOOT` + product names |
| Boot (`boot_jump.c`) | `platform/boot.c` | same `BOOT` + VTOR/MEMRMP restore |
| QSPI test (`qspi_test.c`) | `platform/qspi.c` | same `QSPI:ID?` |
| Telemetry (`telemetry.c`) | `core/telemetry.c` | same `!EVT` lines |
| Boot diagnostics (`boot_diag.h`, `main.c`) | `core/boot_diag.c` | same `!BOOT` lines |
| Simulator (`sim_transrex.c`, `SIM:*`) | `app/simulator/` | same behavior, moved out of shared files |
| Host tools (`python/`) | unchanged (except `wham_build.py`) | wire protocol unchanged |

---

## 7. Migration / cutover plan (how we switch without bricking)

1. Build the new firmware in parallel; verify it produces the same `*IDN?`
   and `FWUPdate:*` replies on the bench.
2. **First flash via ST-Link** to bank 1 (with `BFB2=0`), keeping the current
   firmware in bank 2 as rollback.
3. Run the command-replay and simulator-validation suites against the new
   build.
4. Exercise a `fw_update.py --rollback` back to the old bank-2 image to prove
   the escape hatch, then `--rollback` forward again.
5. Only after the new build has passed both boards' full regression do we
   retire the old source tree.

This keeps a known-good image always one `--rollback` away during the cutover.

---

## 8. Risks and mitigations

| Risk | Mitigation |
|---|---|
| Numeric drift in PID from refactor | golden-vector host tests + `-O2` behavior check against recorded bench numbers |
| Missed command/reply edge case | frozen `command_table[]` + full command-replay script |
| Post-swap reset hang regresses | `NRST_MODE=01`/`IRHEN=0` set in Phase 7 + hardware pull-up + ST-Link backstop during bring-up |
| `-O2` exposes a latent race/UB | `-Wall -Wextra -Werror`, `volatile`-discipline review, and the existing `__disable_irq` critical sections kept |
| CMake migration loses the `.ioc` pin config | capture the exact pin/peripheral config into `platform/gpio.c`/`hrtim.c` first (Phase 1), treat CubeMX as generation-only from then on |
| Watchdog interferes with `FWUPdate:SWAP` | gate IWDG off around option-byte reload (documented) |

---

## 9. Effort estimate (rough)

| Phase | Relative effort |
|---|---|
| 0 — CMake/toolchain | small |
| 1 — platform layer | small–medium (mostly porting) |
| 2 — core extraction + host tests | medium–large (the highest-value step) |
| 3 — command split + app wiring | medium (mechanical) |
| 4 — fw update | small |
| 5 — host tooling | small |
| 6 — simulator target | small–medium |
| 7 — safety | small |
| 8 — regression/docs | medium |

Total is a few focused work-days of equivalent effort, dominated by Phases 2
and 3. The payoff: a reproducible build, a host test suite, and a codebase a
new engineer can read without archaeology.

---

## 10. Open decisions (please confirm before execution)

1. **Optimization level**: I recommend `-O2` for release. Do you want to keep
   `-O0`/`-Og` during bring-up and flip at the end, or go straight to `-O2`?
2. **CubeMX**: keep it as generation-only (my recommendation), or drop it
   entirely and hand-maintain the init code?
3. **Watchdog**: enable IWDG now, or defer? (I recommend enabling it in
   Phase 7.)
4. **`DEBUG:FAULT:BYPASS`**: compile-time-gate it out of release builds, or
   keep it but require a jumper?
5. **OCP polling**: accept it as a permanent polled input (my recommendation,
   given the fixed PCB), or plan a future PCB spin to give OCP its own EXTI?
6. **Toolchain**: standardize on the CubeIDE-bundled 13.3 (matches today's
   build) via `ARM_TOOLCHAIN_DIR`, or on Homebrew's `arm-none-eabi-gcc`?

---

*Generated from the session review of `main.c`, `pid.c/h`, `state_machine.c`,
`hrtim.c`, `cmd_parser.c`, `uart.c`, `fw_update.c/h`, the `.ioc`, the build
layout, and the `python/` tooling.*
