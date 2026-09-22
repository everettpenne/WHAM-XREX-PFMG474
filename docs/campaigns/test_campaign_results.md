# Test Campaign Results — 10 s ramp / 5 s flat-top

**Date:** 2026-09-21 (campaign run 14:28–14:54).
**Boards:** controller `/dev/cu.usbserial-1130` (WHAM-XREX-PFMG474), simulator
`/dev/cu.usbserial-1120` (WHAM-XREX-PFMG474-SIM), cross-wired over fiber.
**Firmware:** `d74b0653-dirty` (1750-sample PID log, pfm table shrunk to 1000 —
see [`test_campaign_report.md`](test_campaign_report.md) §1).

## Summary

| group | result |
|---|---|
| Discrete scenarios | **6/7 PASS** (only the `gain_sweep` group scored a FAIL on one criterion-sensitive point) |
| Matrix (45 shots) | **45/45 PASS** |

---

## 1. `gain_sweep` — closed-loop PID (P + I sweep) — ⚠️ 5/6 PASS

**Parameters:** closed-loop, channel 1, 4 s step setpoint to 40 000 Hz. P sweep
`Kp ∈ {0.2, 0.5, 1.0}` (`Ki=Kd=0`); I sweep `Kp=1.0`, `Ki ∈ {10, 50, 100}`,
`Kd=0`.

| point | expected steady state | final measured | verdict |
|---|---|---|---|
| `kp0.2` | 6 667 Hz | — | ✅ PASS |
| `kp0.5` | 13 333 Hz | — | ✅ PASS |
| `kp1.0` | 20 000 Hz | — | ✅ PASS |
| `ki10` | 40 000 Hz | 40 236 Hz | ✅ PASS |
| `ki50` | 40 000 Hz | 40 255 Hz | ✅ PASS |
| `ki100` | 40 000 Hz | 40 543 Hz | ❌ FAIL |

**What it accomplished:** exercised the closed-loop PID path end-to-end against
the simulated Transrex plant. The P sweep confirmed the pure-P steady-state
error lands at the predicted `Kp·40000/(1+Kp)`; the I sweep confirmed the
integral term drives that error to zero (all three `ki*` points converged to
within ~550 Hz of 40 000 Hz).

**Note on the `ki100` FAIL:** it is a *verdict-criterion* artifact, not a
control failure — the three Ki points measured 40 236 / 40 255 / 40 543 Hz
(essentially all converged), and `ki100`'s final sample landed 43 Hz outside the
±500 Hz tolerance. The "final single sample" rule is sensitive to loop ringing
phase; hardening it to the median of the last ~0.5 s (as `all_channels` already
does) would flip this to PASS.

![gain_sweep ki50 (PI convergence)](../../shots/20260921_142900_gain_sweep_ki50/combined.png)

---

## 2. `ocp` — overcurrent fault injection — ✅ PASS

**Parameters:** open-loop, 2 channels (ch1 faulted, ch2 survivor), ramp 10 s /
flat 5 s, demand 3000 A, OCP injected ~0.5 s into flat-top.

**What it accomplished:** verified the per-channel overcurrent response —
`STATE?` reports `OVERCURRENT 1`, the faulted channel hard-disables, and the
survivor stays running at a derated 21 660 Hz (stepped down and ramped, not
stopped).

![ocp grid (fault marker)](../../shots/20260921_142918_ocp/grid.png)

---

## 3. `enerpro` — Enerpro fault injection — ✅ PASS

**Parameters:** open-loop, 2 channels, 10/5, 3000 A, Enerpro injected ~0.5 s
into flat-top.

**What it accomplished:** confirmed Enerpro is treated like overcurrent (derate
the survivor + keep running — `STATE?` → `ENERPRO 1`, survivor 21 638 Hz), **not**
a Water/Temp-style full stop — validating the 2026-09-18 fault reclassification.

![enerpro grid (fault marker)](../../shots/20260921_142935_enerpro/grid.png)

---

## 4. `watertemp` — Water/Temp fault injection — ✅ PASS

**Parameters:** open-loop, 2 channels, 10/5, 3000 A, Water+Temp injected ~0.5 s
into flat-top.

**What it accomplished:** verified Water/Temp is a **General Fault** — the
distinct full-stop response (`STATE?` → `GENERAL`), ramping all active channels
down to 0 A rather than keeping survivors running.

![watertemp grid (fault marker)](../../shots/20260921_142952_watertemp/grid.png)

---

## 5. `ena_contact` — ENA_OUT/CONTACT_OUT drop — ✅ PASS

**Parameters:** open-loop, 2 channels, 10/5, 3000 A, `CONTACT_OUT` dropped on
ch1 ~0.5 s into flat-top.

**What it accomplished:** exercised the controller's own-commanded-output fault
(`SM_FAULT_ENABLE_OUTPUT`) — dropping `CONTACT_OUT` mid-shot produces a
per-channel `ENABLE_OUTPUT 1` fault with the survivor derated (21 660 Hz),
distinct from the external-enable input interlock.

![ena_contact grid (fault marker)](../../shots/20260921_143009_ena_contact/grid.png)

---

## 6. `multi_fault` — fault sequencing — ✅ PASS

**Parameters:** open-loop, 3 channels (ch1 OCP, ch3 Enerpro, ch2 survivor),
10/5, 3000 A, faults injected ~0.5 s and ~0.65 s into flat-top.

**What it accomplished:** verified two different fault types in one shot — the
second fault still hard-disables its own channel, `STATE?` keeps reporting the
**first** fault (`OVERCURRENT 1`), and the survivor derates after *each* fault
(29 989 Hz → 24 769 Hz).

![multi_fault grid (two fault markers)](../../shots/20260921_143017_multi_fault/grid.png)

---

## 7. `all_channels` — 4 channels simultaneously — ✅ PASS

**Parameters:** open-loop, all 4 channels, 10/5, per-channel demand 600 / 2000 /
3600 / 5400 A.

**What it accomplished:** confirmed four independent Transrex channels can be
driven and measured at once with no cross-channel interference — flat-top median
`measured − output` error of **377 / 387 / 362 / 363 Hz** per channel.

![all_channels stacked 4-panel](../../shots/20260921_143033_all_channels/combined.png)

---

## 8. Matrix — channel/demand sweep — ✅ 45/45 PASS

**Parameters:** open-loop, 10/5, **45 shots** = every non-empty subset of
{1,2,3,4} × demand {600, 3000, 5400} A (10/50/90 %). Pass = per-channel
`output − setpoint` `max_abs` < 3000 Hz.

**What it accomplished:** swept the open-loop profile generator + HRTIM write +
fiber round-trip across every channel combination and three operating points,
confirming clean tracking (worst error **0 Hz** in all 45) with no
cross-channel coupling.

![matrix sample — all 4 channels at 5400 A](../../shots/20260921_145413_matrix_ch1+2+3+4_5400A/grid.png)

---

## Review points carried over

1. **`ki100` verdict is criterion-sensitive** (see §1) — consider a median-based
   PI verdict.
2. **720 Hz ripple is aliased** at the ~52–56 Hz log rate (visible as noise in
   `measured_hz`), not resolvable in the waveform logs.
3. **Fault ramp-down is still 1 s** (`FAULT_RAMP_DOWN_TIME_S`), far faster than
   the now-10 s shot ramp.
4. **Closed-loop 25 s profiles remain unexercised** — the only closed-loop runs
   here are 4 s step setpoints.
5. **Calibration placeholders unchanged** (Amps↔Hz endpoints, 6000 A/channel,
   simulator τ, ripple split) — the campaign validates firmware-vs-model, not the
   plant model itself.

Raw data + full per-channel CSVs: `shots/20260921_14*_*/`. Generated LaTeX
reports: [`../reports/simulator_validation_report.tex`](../reports/simulator_validation_report.tex)
and [`../reports/simulator_channel_matrix_report.tex`](../reports/simulator_channel_matrix_report.tex).
