# Test Campaign — 10 s ramp / 5 s flat-top (controller × simulated Transrex)

**Status:** boards reflashed, smoke test passed, full campaign **staged but not yet run** (awaiting go-ahead).
**Date:** 2026-09-21.
**Assumptions:** two boards cross-wired over fiber (`docs/pin_mapping_reference.tex` §7);
controller on `/dev/cu.usbserial-1130`, simulator on `/dev/cu.usbserial-1120`;
no rebuild/reflash needed beyond what's recorded here.

---

## 1. What changed for this campaign

### 1.1 Firmware: longer waveform log (funded by shrinking the legacy table)

The shot profiles now use **10 s ramp / 5 s flat-top** (25 s total). At the
old 1000-sample log cap that meant a 40 Hz log; the log cap was raised so
the whole shot stays well resolved:

| change | before | after |
|---|---|---|
| `PFM_TABLE_SIZE` (`Core/Inc/pfm.h`) | 5000 (50 000 B) | **1000** (10 000 B) — legacy `TABLE:*`/`FIRE` fallback, still ~10 ms of playback |
| `PID_LOG_MAX_SAMPLES` (`Core/Inc/pid.h`) | 1000 (48 000 B) | **1750** (84 000 B) |
| `SIM_LOG_MAX_SAMPLES` (`Core/Inc/sim_transrex.h`) | 1000 | 1000 (unchanged — see §1.2) |

**Measured RAM (after change, both targets link clean):**

| build | text | data | bss | RAM used | free of 128 KiB |
|---|---|---|---|---|---|
| controller | 90 104 | 476 | 109 724 | 110 200 | **20.4 KiB** |
| simulator | 99 764 | 480 | 121 816 | 122 296 | **8.6 KiB** |

The 25 s shot now logs at `decim = 19` → **~52 Hz** (was 40 Hz), 1315 samples
fully covering the shot (the old 1000-sample cap would have truncated it).

### 1.2 Why `SIM_LOG_MAX_SAMPLES` stayed at 1000

The simulator build carries *both* log arrays (its own 12 B/sample SIM log
plus the 48 B/sample PID log, because it runs the full controller firmware).
Raising both to 1750 would have left only ~256 B of free SRAM. The SIM log is
a secondary cross-check; the plots read the controller's `measured_hz` (which
*is* the simulator's feedback arriving over the fiber), so 1000 there is fine.

### 1.3 Harness: 10/5 defaults + stacked 4-panel plots with fault markers

`python/run_simulator_validation.py` now:

- mirrors `PID_LOG_MAX_SAMPLES = 1750` and passes it to every `PID:LOG …`;
- takes `--ramp-s` / `--flat-s` (defaults **10.0 / 5.0**), threaded through
  every profile scenario and the matrix sweep;
- renders the per-test 4-panel figure as **one column of 4 stacked panels**
  (was 2×2), one panel per channel, shared time axis, with ramp-up /
  flat-top / ramp-down regions shaded and labeled;
- draws **fault markers** — a vertical dashed red line + rotated label at the
  injection instant on every populated panel — for the fault scenarios
  (`ocp`, `enerpro`, `watertemp`, `ena_contact`, `multi_fault`).

---

## 2. Smoke test — 10/5 profiled shot

Command run:

```bash
python3 python/run_simulator_validation.py \
    --controller-port /dev/cu.usbserial-1130 \
    --simulator-port  /dev/cu.usbserial-1120 \
    --scenarios profiled_shot
```

**Result: PASS** — `output_error max_abs = 0 Hz`, **1315 samples @ 52 Hz**
(channel 1, open-loop, demand 3000 A, ramp 10 s / flat-top 5 s). The whole
25 s shot was captured in one contiguous log.

Channel 1 — commanded vs. measured, with the three shot phases shaded:

![profiled_shot ctrl_ch1](../shots/20260921_142027_profiled_shot/ctrl_ch1.png)

Controller (top) vs. simulator's own DRIVE/FEEDBACK (bottom):

![profiled_shot combined](../shots/20260921_142027_profiled_shot/combined.png)

Simulator-side log (measured DRIVE vs. filtered FEEDBACK):

![profiled_shot sim_ch1](../shots/20260921_142027_profiled_shot/sim_ch1.png)

Raw data: `shots/20260921_142027_profiled_shot/{ctrl_ch1,sim_ch1}.csv`.

---

## 3. The full campaign (staged — awaiting go-ahead)

One command for the discrete scenarios, one for the matrix (both read the
10/5 defaults; override with `--ramp-s`/`--flat-s` if needed):

```bash
# Discrete: gain sweep (P+I), 5 fault scenarios, all-4-channels
python3 python/run_simulator_validation.py \
    --controller-port /dev/cu.usbserial-1130 \
    --simulator-port  /dev/cu.usbserial-1120 \
    --scenarios gain_sweep,ocp,enerpro,watertemp,ena_contact,multi_fault,all_channels

# 45-shot channel-combination x demand matrix (open-loop) -- ~20 min at 10/5
python3 python/run_simulator_validation.py \
    --controller-port /dev/cu.usbserial-1130 \
    --simulator-port  /dev/cu.usbserial-1120 \
    --matrix
```

### 3.1 Scenarios and pass criteria

**Fault simulation** (all demand 3000 A, fault injected ~0.5 s into flat-top):

| scenario | injects | pass criterion |
|---|---|---|
| `ocp` | OCP ch1 | `STATE?` → `OVERCURRENT 1`; survivor ch2 derated + running |
| `enerpro` | Enerpro ch1 | `STATE?` → `ENERPRO 1`; survivor derated + running |
| `watertemp` | Water+Temp ch1 | `STATE?` → `GENERAL` (full stop) |
| `ena_contact` | drop `CONTACT_OUT` ch1 | `STATE?` → `ENABLE_OUTPUT 1`; survivor derated + running |
| `multi_fault` | OCP ch1 then Enerpro ch3 | first fault preserved; ch1/ch3 disabled; survivor ch2 derated after each |

**Demand combinations:**

| run | coverage | pass criterion |
|---|---|---|
| `all_channels` | 4 channels at 600/2000/3600/5400 A | flat-top median `measured−output` < 500 Hz per channel |
| `--matrix` | 45 shots: 15 channel subsets × {600, 3000, 5400} A | per-channel `output−setpoint` `max_abs` < 3000 Hz |

**PID constants** (`gain_sweep`, closed-loop, ch1, 4 s step to 40 kHz):

- P sweep `Kp ∈ {0.2, 0.5, 1.0}` (P-only): settle at `Kp·40000/(1+Kp)` ± 500 Hz.
- I sweep `Ki ∈ {10, 50, 100}` @ `Kp=1.0`, `Kd=0`: settle at 40 000 ± 500 Hz.
- `Kd=0` throughout (per confirmation).

Each test writes CSVs + a stacked 4-panel `grid.png` (fault scenarios also
carry the fault markers) under `shots/<timestamp>_<name>/`, and regenerates
`docs/simulator_validation_report.tex` / `docs/simulator_channel_matrix_report.tex`.

---

## 4. Review points

1. **`ki10` (I sweep) is expected to flag borderline.** In the prior run it
   failed because the *final single sample* landed 630 Hz from target while
   still ringing; the loop is converging, not failing. Consider hardening the
   PI verdict to the median of the last ~0.5 s (the `all_channels` scenario
   already uses a median for exactly this reason).
2. **720 Hz ripple is aliased in these logs.** At ~52 Hz log rate the
   simulator's 720/1440 Hz current ripple appears as noise in `measured_hz`,
   not a resolvable tone. Ripple is a scope/DSLogic observation, not a
   waveform-log one.
3. **Fault ramp-down is still 1 s** (`FAULT_RAMP_DOWN_TIME_S`), independent of
   the now-10 s shot ramp — a mid-shot fault ramps to 0 A much faster than the
   shot ramp. Revisit if the fault ramp should scale with shot length.
4. **Closed-loop 25 s profiles are unexercised** — the only closed-loop tests
   are 4 s step setpoints; the integrator over 25 000 ticks (anti-windup
   exists) is worth a look in a future pass.
5. **Calibration placeholders remain**: `PFM_TURNON_FREQ_HZ`=5 kHz,
   `PFM_MAX_FREQ_HZ`=100 kHz, `PFM_MAX_CURRENT_A_PER_CHANNEL`=6000 A each,
   simulator `τ`=100 ms, ripple 0.8/0.2 % split — the campaign validates the
   *firmware against the model*, not the Amps↔Hz plant model itself.
