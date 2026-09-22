# Test Campaign Plan — Controller × Simulated Transrex (cross-wired)

**Status:** planned 2026-09-21. **Assumptions:** no rebuild/reflash — the
controller and simulator boards are already flashed with their respective
targets and cross-wired over fiber per `docs/pin_mapping_reference.tex`
§7; the hardware setup is validated. This plan drives the existing
`python/run_simulator_validation.py` harness; every run is autonomous
(ARM/FIRE in software, onboard waveform logs armed before each shot and
retrieved after), writes CSVs + PNGs under `shots/<ts>_<name>/`, and
regenerates the LaTeX report under `docs/`.

Three themes, matching the request: **fault simulation**, **demand
combinations**, and **PID constants**.

---

## Group 1 — Fault simulation

Exercises the controller's fault response through the simulator's fault
injection, mid-shot, with a survivor channel watched throughout.

| Scenario | Injects | Expected | Pass criterion (harness) |
|---|---|---|---|
| `ocp` | OCP on ch1 (`SIM:FAULT:OCP 1 1`) | faulted ch1 hard-disabled; survivor ch2 derated + still running | `STATE?` → `OVERCURRENT 1`; ch2 running & `0 < output < demand_hz` |
| `enerpro` | Enerpro on ch1 | same derate/keep-running response as OCP | `STATE?` → `ENERPRO 1`; ch2 survives |
| `watertemp` | Water+Temp on ch1 | **full system stop** (Water/Temp = General Fault) | `STATE?` → `GENERAL` |
| `ena_contact` | drop `CONTACT_OUT` ch1 mid-shot | per-channel `ENABLE_OUTPUT` fault, survivor derates | `STATE?` → `ENABLE_OUTPUT 1`; ch2 survives |
| `multi_fault` | OCP ch1 then Enerpro ch3 | both faulted channels disabled; `STATE?` keeps FIRST fault type; survivor ch2 derates after each | `OVERCURRENT 1` preserved; ch1/ch3 disabled; ch2 survives |

All use demand 3000 A, ramp 1 s / flat 3 s, fault injected ~0.5 s into
the flat top. Run:

```bash
python3 python/run_simulator_validation.py \
    --scenarios ocp,enerpro,watertemp,ena_contact,multi_fault
```

## Group 2 — Demand combinations

| Run | Coverage | Pass criterion |
|---|---|---|
| `all_channels` | all 4 channels gated simultaneously at 600 / 2000 / 3600 / 5400 A | flat-top **median** `measured − output` error < 500 Hz per channel |
| `--matrix` | **45 shots**: every non-empty subset of {1,2,3,4} × {600, 3000, 5400} A (10/50/90 %) | every enabled channel's `output − setpoint` `max_abs` < 3000 Hz |

Run:

```bash
python3 python/run_simulator_validation.py --scenarios all_channels
python3 python/run_simulator_validation.py --matrix
```

## Group 3 — PID constants

The simulator is a near-unity-DC-gain plant (single-pole LPF, `τ` default
100 ms) plus the 720/1440 Hz ripple, so these values are **bench
coverage of the PID code paths and convergence behaviour, not a real
magnet tune** — same caveat the project already carries for all its gain
work.

- **P sweep (shipped):** `gain_sweep` — closed-loop, ch1, target 40 kHz,
  `Kp ∈ {0.2, 0.5, 1.0}`, `Ki=Kd=0`. Pass: final `measured` ≈
  `Kp·40000/(1+Kp)` ± 500 Hz (pure-P steady-state error).

- **I sweep (implemented 2026-09-21, per confirmation "Add Ki sweep, Kd
  can be set to 0"):** same scenario, `Kp=1.0`, `Ki ∈ {10, 50, 100}`,
  `Kd=0`. Integral drives final `measured → setpoint`; pass = within
  500 Hz of 40 kHz. `Kd` stays 0 throughout — no D sweep.

Run:

```bash
python3 python/run_simulator_validation.py --scenarios gain_sweep
```

---

## Execution prerequisites (host, one-time)

```bash
pip3 install pyserial matplotlib numpy
```

The harness auto-detects the two boards by `*IDN?`
(`WHAM-XREX-PFMG474` vs `WHAM-XREX-PFMG474-SIM`) on
`/dev/cu.usbserial-*`, so no port arguments are strictly needed. The
prior report's `ext_regression` scenario was **BLOCKED** (PD1→PF15 bench
loopback wire unplugged) and is omitted here unless that wire is now
restored.
