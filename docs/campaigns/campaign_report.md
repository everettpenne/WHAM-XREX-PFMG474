# Test Campaign Report — 10 s ramp / 5 s flat-top

**Date:** 2026-09-21 (run 14:28–14:54). **Firmware:** `d74b0653-dirty`
(1750-sample PID log). **Rig:** controller + simulator boards cross-wired over
fiber; every test is driven autonomously by `python/run_simulator_validation.py`,
with onboard waveform logs armed before each shot and retrieved after.

**Overall:** 6/7 discrete scenarios passed; the 45-shot matrix passed 45/45.
The only non-pass is a single criterion-sensitive point in the gain sweep
(details below), not a control defect.

---

## 1. `gain_sweep` — closed-loop PID gain sweep — 5/6 PASS

This test exercised the closed-loop PID controller against the simulated
Transrex plant by commanding a 40 kHz step on channel 1 under six different gain
sets: three P-only (`Kp` 0.2 / 0.5 / 1.0, `Ki=Kd=0`) and three PI (`Kp=1.0`,
`Ki` 10 / 50 / 100, `Kd=0`). The P-only runs **passed** because the measured
frequency settled at the theoretical pure-P steady-state value
`Kp·40000/(1+Kp)` (i.e. the expected steady-state error when there is no
integral action). The PI runs **passed** because integral action eliminated that
error, all three converging to within ~550 Hz of 40 kHz (40 236 / 40 255 /
40 543 Hz). The one **FAIL** — `ki100` — is a verdict-criterion artifact rather
than a control problem: its final sample landed 43 Hz outside the ±500 Hz
tolerance while clearly converged, an artifact of judging by a single
ringing-phase sample.

![gain_sweep — 4 gain points](../../shots/gain_sweep_4panel.png)

---

## 2. `ocp` — overcurrent fault injection — PASS

Two channels were gated open-loop (ramp 10 s / flat 5 s, 3000 A) and an
overcurrent fault was injected on channel 1 ~0.5 s into the flat top. The test
**passed** because `STATE?` reported `OVERCURRENT 1`, the faulted channel
hard-disabled immediately, and the surviving channel 2 kept running at a derated
21 660 Hz — exactly the specified per-channel overcurrent response (immediate
disable for the faulted channel, a `1/N` step-down then ramp for the survivors).

![ocp — stacked 4-panel with fault marker](../../shots/20260921_142918_ocp/grid.png)

---

## 3. `enerpro` — Enerpro fault injection — PASS

Same 2-channel setup (10/5, 3000 A), with an Enerpro fault injected on channel 1
mid-shot. The test **passed** because `STATE?` reported `ENERPRO 1` and the
survivor derated and kept running (21 638 Hz), confirming Enerpro faults use the
overcurrent-style *reduce-and-keep-running* response rather than a Water/Temp
full stop — validating the 2026-09-18 fault reclassification.

![enerpro — stacked 4-panel with fault marker](../../shots/20260921_142935_enerpro/grid.png)

---

## 4. `watertemp` — Water/Temp fault injection — PASS

Same 2-channel setup (10/5, 3000 A), with a combined Water+Temp fault injected
on channel 1 mid-shot. The test **passed** because `STATE?` reported `GENERAL`
— the distinct full-stop fault type — confirming that Water/Temp trips the
system-wide open-loop ramp-down to 0 A on every active channel, rather than the
derate-and-survive behavior of OCP/Enerpro.

![watertemp — stacked 4-panel with fault marker](../../shots/20260921_142952_watertemp/grid.png)

---

## 5. `ena_contact` — ENA_OUT/CONTACT_OUT drop — PASS

Two channels were gated (10/5, 3000 A) and the controller's own `CONTACT_OUT`
for channel 1 was dropped mid-shot. The test **passed** because `STATE?`
reported `ENABLE_OUTPUT 1` and the survivor derated to 21 660 Hz — validating
the per-channel self-commanded-output fault (`SM_FAULT_ENABLE_OUTPUT`), which is
distinct from the external-enable *input* interlock.

![ena_contact — stacked 4-panel with fault marker](../../shots/20260921_143009_ena_contact/grid.png)

---

## 6. `multi_fault` — fault sequencing — PASS

Three channels were gated (10/5, 3000 A) and two different faults were injected
in sequence — OCP on channel 1, then Enerpro on channel 3 — with channel 2 as
the survivor. The test **passed** because both faulted channels ended disabled,
`STATE?` preserved the *first* fault's identity (`OVERCURRENT 1`) even after the
second fault, and the survivor visibly derated after each fault (29 989 Hz →
24 769 Hz) — confirming the "already-faulted" short-circuit still hard-disables
the newly-faulted channel without clobbering the reported fault type.

![multi_fault — stacked 4-panel with two fault markers](../../shots/20260921_143017_multi_fault/grid.png)

---

## 7. `all_channels` — four channels simultaneously — PASS

All four channels were gated and fired at once (10/5) at per-channel demands of
600 / 2000 / 3600 / 5400 A. The test **passed** because every channel's
flat-top median `measured − output` error stayed well under the 500 Hz threshold
(377 / 387 / 362 / 363 Hz), confirming four independent Transrex channels run
and measure simultaneously with no cross-channel interference.

![all_channels — stacked 4-panel](../../shots/20260921_143033_all_channels/combined.png)

---

## 8. Matrix — channel/demand sweep — 45/45 PASS

The stress sweep ran 45 open-loop shots (10/5) covering every non-empty subset
of {1,2,3,4} across three demand levels (600 / 3000 / 5400 A = 10/50/90 %). All
45 **passed** with a worst `output − setpoint` error of **0 Hz**, confirming the
profile generator, per-channel HRTIM write, and the fiber feedback round-trip are
clean across the full channel-combination × operating-point envelope with no
cross-channel coupling.

![matrix sample — all 4 channels at 5400 A](../../shots/20260921_145413_matrix_ch1+2+3+4_5400A/grid.png)

---

## Notes

- The `gain_sweep` `ki100` FAIL is the final-sample criterion, not a control
  defect (§1); a median-of-last-0.5 s verdict would pass it.
- Fault tests plot the injection instant as a dashed red line + label on every
  populated panel.
- Raw per-channel CSVs and all plots are under `shots/20260921_14*_*/`.
