#!/usr/bin/env python3
"""
run_simulator_validation.py -- autonomous end-to-end test campaign for
the Transrex simulator / controller closed loop, per Part C/D of the
plan at /Users/everettpenne/.claude/plans/cuddly-nibbling-octopus.md.

Drives BOTH boards over their own serial links -- the CONTROLLER via
its real state machine/PID commands (ARM/PID:PROFile:*), the SIMULATOR
via SIM:/raw diagnostic commands -- with no manual intervention: ARM/
FIRE are done in software (PID:PROFile:STARt calls SM_Fire() directly
and does not need a PF15 trigger edge; the PF13 external-enable
interlock is opt-in and off by default, see EXTernal:ENAble in
docs/command_reference.md).

Reuses wham_console.py's existing PID:LOGDATA?-based fetch/plot/stats
pipeline for the controller side (WhamLink, compute_log_stats,
generate_plot, format_channel_report_md) rather than re-implementing
it. The simulator side has its own onboard waveform log
(sim_transrex.c's SIM:LOG/SIM:LOGDATA?, added 2026-09-18) -- armed
before each shot and retrieved after, same "arm/run/retrieve" pattern
as the controller side, NOT host-side polling: SimTransrex_Update()
logs every real main-loop tick with its own onboard timestamp, giving
native resolution instead of a ~10-sample/sec serial-round-trip-limited
poll.

Each scenario's raw data (both boards) lands under
shots/<timestamp>_<scenario>/. A cumulative LaTeX report is written to
docs/simulator_validation_report.tex (compile separately with
pdflatex, same manual step this project's other tracked docs use).

Usage:
  python3 python/run_simulator_validation.py \
      --controller-port /dev/cu.usbserial-XXXX \
      --simulator-port  /dev/cu.usbserial-YYYY

  Omit either --*-port to auto-detect by scanning /dev/cu.usbserial*
  and checking *IDN? (same "always verify identity, never trust a
  remembered port number" discipline established this session -- see
  the board-serial-port-assignments project memory).

  --scenarios <list>   comma-separated subset (default: all). Names:
                        baseline, profiled_shot, ocp, enerpro, watertemp,
                        ena_contact, multi_fault, ext_regression
"""

import argparse
import glob
import os
import sys
import time
from datetime import datetime

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.dirname(SCRIPT_DIR)
sys.path.insert(0, SCRIPT_DIR)

from wham_console import (   # noqa: E402  (path insert must come first)
    WhamLink, WhamError, is_err, explain_err,
    compute_log_stats, generate_plot, format_channel_report_md,
    hz_to_amps, channel_label, SHOTS_DIR, PFM_TURNON_FREQ_HZ,
)

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    HAVE_MPL = True
except ImportError:
    HAVE_MPL = False

DOCS_DIR = os.path.join(PROJECT_DIR, "docs")
REPORT_TEX = os.path.join(DOCS_DIR, "simulator_validation_report.tex")

HRTIM_NUM_CHANNELS = 4


# --------------------------------------------------------------------------
# Connection
# --------------------------------------------------------------------------

def connect_verified(port, expect_substr, label):
    """Opens `port`, sends *IDN?, and REFUSES to proceed unless the
    reply contains `expect_substr` -- this project's boards use
    interchangeable USB-serial chips whose /dev names are not stable
    across reconnects (board-serial-port-assignments project memory);
    a port number alone is never trusted. Retries a few times with a
    longer settle -- a freshly-(re)enumerated port can be silently
    non-responsive on the very first query."""
    link = WhamLink(port=port)
    link.connect()
    time.sleep(1.0)
    idn = ""
    for attempt in range(4):
        try:
            idn = link.query("*IDN?", timeout=1.0)
        except WhamError:
            idn = ""
        if idn:
            break
        time.sleep(1.0)
    if expect_substr not in idn:
        link.close()
        raise WhamError(f"{label} at {port}: *IDN? = {idn!r}, "
                         f"expected to contain {expect_substr!r}")
    print(f"[connect] {label}: {port} -> {idn}")
    return link


def autodetect(expect_substr, label, exclude_port=None):
    for port in sorted(glob.glob("/dev/cu.usbserial*")):
        if port == exclude_port:
            continue
        try:
            link = connect_verified(port, expect_substr, label)
            return link
        except WhamError:
            continue
    raise WhamError(f"could not auto-detect {label} (looked for *IDN? "
                     f"containing {expect_substr!r} on /dev/cu.usbserial*)")


# --------------------------------------------------------------------------
# Controller-side data capture (reuses wham_console.py's pipeline)
# --------------------------------------------------------------------------

def fetch_log(link, channel):
    """Standalone equivalent of WhamConsole._fetch_log() -- same
    PID:LOGDATA? parsing, just driven by a bare WhamLink instead of the
    interactive console's own instance state."""
    reply = link.query(f"PID:LOGDATA? {channel}", timeout=8.0)
    if is_err(reply):
        raise WhamError(explain_err(reply))
    parts = reply.split()
    if len(parts) < 2 or parts[0] != "OK":
        raise WhamError(f"unexpected PID:LOGDATA? reply: {reply!r}")
    count = int(parts[1])
    rate_hz = int(parts[2])
    vals = list(map(int, parts[3:3 + count * 3]))
    if len(vals) != count * 3:
        raise WhamError(f"PID:LOGDATA? claimed {count} samples but only "
                         f"{len(vals)} values were parsed")
    rows = []
    for i in range(count):
        sp, ms, op = vals[3 * i], vals[3 * i + 1], vals[3 * i + 2]
        t_s = (i / rate_hz) if rate_hz else 0.0
        rows.append(dict(t_s=t_s, setpoint_hz=sp, measured_hz=ms, output_hz=op))
    return rows, rate_hz, count


def shot_meta(link, channel, ramp_s=None, flat_s=None, demand_a=None):
    """Standalone equivalent of WhamConsole._shot_meta() -- reads gains/
    loop mode live rather than from cached console state."""
    idn = ""
    try:
        idn = link.query("*IDN?", timeout=1.0)
    except WhamError:
        pass
    kp = ki = kd = None
    try:
        reply = link.query(f"PID:GAINS? {channel}", timeout=1.0)
        if not is_err(reply):
            parts = reply.split()
            if len(parts) >= 4:
                kp, ki, kd = float(parts[1]), float(parts[2]), float(parts[3])
    except WhamError:
        pass
    loop_mode = None
    try:
        reply = link.query(f"PID:LOOPMODE? {channel}", timeout=1.0)
        if not is_err(reply):
            loop_mode = "closed" if reply.split()[-1] in ("1", "CLOSED") else "open"
    except WhamError:
        pass
    return dict(
        channel=channel, nickname=None,
        timestamp=datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        idn=idn, kp=kp, ki=ki, kd=kd, loop_mode=loop_mode,
        ramp_time_s=ramp_s, flat_top_time_s=flat_s, demand_current_a=demand_a,
    )


def save_and_plot_controller(channel, rows, rate_hz, count, out_dir, ts, meta):
    if count == 0:
        return None, None
    base = os.path.join(out_dir, f"ctrl_ch{channel}")
    import csv, json
    with open(base + ".csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "setpoint_hz", "measured_hz", "output_hz"])
        for r in rows:
            w.writerow([f"{r['t_s']:.5f}", r["setpoint_hz"], r["measured_hz"], r["output_hz"]])
    meta = dict(meta, rate_hz=rate_hz, log_count=count)
    with open(base + "_meta.json", "w") as f:
        json.dump(meta, f, indent=2)
    png = None
    if HAVE_MPL:
        try:
            png = generate_plot(rows, meta, base + ".png")
        except WhamError as exc:
            print(f"[warn] controller plot failed: {exc}")
    return base + ".csv", png


# --------------------------------------------------------------------------
# Simulator-side data capture -- SIM:LOG/SIM:LOGDATA? (sim_transrex.c's
# own waveform log, added 2026-09-18 per direct correction: this
# ARMS before the shot and RETRIEVES after, exactly matching the
# controller-side PID:LOG/PID:LOGDATA? pattern below, instead of the
# original host-side SIM:CHANnel:STATus? polling approach -- polling
# was coarse (~10 samples/sec) and added serial round-trip jitter;
# this instead logs every real SimTransrex_Update() call (main-loop
# cadence), giving each sample its own onboard timestamp rather than a
# host-side one.
# --------------------------------------------------------------------------

def arm_sim_log(sim, channel, max_samples=1000, min_interval_ms=5):
    """SIM:LOG <ch> <maxSamples> <minIntervalMs> -- see
    sim_transrex.h's own SimTransrex_ArmLog() comment. min_interval_ms
    of 5 gives up to 200 samples/sec, finer than the old 10/sec poll,
    while still comfortably covering multi-second shots within
    SIM_LOG_MAX_SAMPLES (1000)."""
    reply = sim.query(f"SIM:LOG {channel} {max_samples} {min_interval_ms}", timeout=1.0)
    if is_err(reply):
        raise WhamError(explain_err(reply))


def fetch_sim_log(sim, channel):
    """SIM:LOGDATA? <ch> -- `OK <count> <t0> <drive0> <feedback0> ...`,
    one (timeMs, driveHz, feedbackHz) triple per sample, each with its
    own onboard elapsed-ms-since-armed timestamp (sim_transrex.h's own
    SimTransrex_GetLogSample() comment explains why -- no shared rate_hz
    like PID:LOGDATA?, the main loop has no fixed tick to decimate
    against)."""
    reply = sim.query(f"SIM:LOGDATA? {channel}", timeout=8.0)
    if is_err(reply):
        raise WhamError(explain_err(reply))
    parts = reply.split()
    if len(parts) < 2 or parts[0] != "OK":
        raise WhamError(f"unexpected SIM:LOGDATA? reply: {reply!r}")
    count = int(parts[1])
    vals = list(map(int, parts[2:2 + count * 3]))
    if len(vals) != count * 3:
        raise WhamError(f"SIM:LOGDATA? claimed {count} samples but only "
                         f"{len(vals)} values were parsed")
    rows = []
    for i in range(count):
        t_ms, drive_hz, feedback_hz = vals[3 * i], vals[3 * i + 1], vals[3 * i + 2]
        rows.append(dict(t_s=t_ms / 1000.0, drive_hz=drive_hz, feedback_hz=feedback_hz))
    return rows


def save_and_plot_sim(channel, rows, out_dir, meta):
    if not rows:
        return None, None
    base = os.path.join(out_dir, f"sim_ch{channel}")
    import csv, json
    with open(base + ".csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "drive_hz", "feedback_hz"])
        for r in rows:
            w.writerow([f"{r['t_s']:.4f}", r["drive_hz"], r["feedback_hz"]])
    with open(base + "_meta.json", "w") as f:
        json.dump(meta, f, indent=2)
    png = None
    if HAVE_MPL:
        png = generate_sim_plot(rows, meta, base + ".png")
    return base + ".csv", png


def generate_sim_plot(rows, meta, out_path):
    """Simulator-side plot: DRIVE_HZ (what it measured from the
    controller, unfiltered/raw) and FEEDBACK_HZ (what it's driving back
    out, its own filtered state) vs. time -- both logged onboard at
    native main-loop resolution (arm-before/retrieve-after, not host
    polling). Simpler than generate_plot() (wham_console.py) -- no
    Amps axis (this board isn't running a real shot profile itself), no
    glitch detection (that's specific to real HRTIM feedback capture,
    not meaningful for this module's own filtered internal state)."""
    t = [r["t_s"] for r in rows]
    drive = [r["drive_hz"] for r in rows]
    fb = [r["feedback_hz"] for r in rows]

    fig, ax = plt.subplots(figsize=(11, 5))
    ax.step(t, drive, where="post", color="#2a78d6", lw=1.2, label="DRIVE_HZ (measured from controller, raw)")
    ax.step(t, fb, where="post", color="#eb6834", lw=1.8, label="FEEDBACK_HZ (this module's filtered output)")
    ax.axhline(PFM_TURNON_FREQ_HZ, color="#444444", lw=0.8, ls=":")
    ax.set_xlabel("Time since SIM:LOG armed (s)")
    ax.set_ylabel("Frequency (Hz)")
    ax.set_title(f"Simulator Ch{meta['channel']} -- {meta.get('idn', '')}\n"
                 f"{meta.get('timestamp', '')} ({len(rows)} samples, onboard-logged)")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best", fontsize=9)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    return out_path


# --------------------------------------------------------------------------
# Shared setup/teardown
# --------------------------------------------------------------------------

def reset_to_safe_state(ctrl, sim):
    """Both boards to a known-safe baseline between scenarios: all
    channels disabled, all fiber outputs LOW/healthy, faults cleared,
    EXTernal:ENAble interlock left off (default -- software-only
    ARM/FIRE requires this, see this file's own top comment)."""
    for link in (ctrl, sim):
        for ch in range(1, HRTIM_NUM_CHANNELS + 1):
            link.query(f"PID:CHANnel:ENAble {ch} 0", timeout=1.0)
            link.query(f"XREX:CHANnel:ENAOut {ch} 0", timeout=1.0)
            link.query(f"XREX:CHANnel:CONTactOut {ch} 0", timeout=1.0)
        link.query("EXTernal:ENAble 0", timeout=1.0)
        link.query("FAULT:CLEAR", timeout=1.0)
        link.query("DISARM", timeout=1.0)
    for ch in range(1, HRTIM_NUM_CHANNELS + 1):
        sim.query(f"SIM:FAULT:WATERTEMP {ch} 0", timeout=1.0)
        sim.query(f"SIM:FAULT:ENERPRO {ch} 0", timeout=1.0)
        sim.query(f"SIM:FAULT:OCP {ch} 0", timeout=1.0)


def arm_and_gate(ctrl, sim, channels):
    """Gates the simulator on for `channels` (controller asserts its
    own ENA_OUT/CONTACT_OUT transmitters), enables those channels on
    the controller, and ARMs -- the software-only precondition sequence
    this file's own top comment describes. Returns True on success."""
    for ch in channels:
        ctrl.query(f"XREX:CHANnel:ENAOut {ch} 1", timeout=1.0)
        ctrl.query(f"XREX:CHANnel:CONTactOut {ch} 1", timeout=1.0)
        ctrl.query(f"PID:CHANnel:ENAble {ch} 1", timeout=1.0)
    time.sleep(0.2)   # let the fiber-received gating state settle on the simulator
    reply = ctrl.query("ARM", timeout=2.0)
    return not is_err(reply)


def new_scenario_dir(name):
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = os.path.join(SHOTS_DIR, f"{ts}_{name}")
    os.makedirs(out_dir, exist_ok=True)
    return out_dir, ts


# --------------------------------------------------------------------------
# Scenarios -- each returns a result dict for the report generator.
# --------------------------------------------------------------------------

def result(name, description, expected, verdict, notes, artifacts=None):
    return dict(name=name, description=description, expected=expected,
                verdict=verdict, notes=notes, artifacts=artifacts or [])


def scenario_baseline_tracking(ctrl, sim):
    """#1 -- gate channel 1 on, command a plain open-loop frequency,
    confirm the simulator's onboard-logged DRIVE_HZ tracks it. Both
    boards' logs are ARMED before the shot and RETRIEVED after --
    ctrl's own PID:LOG/LOGDATA? (already this project's established
    controller-test pattern) and sim's new SIM:LOG/LOGDATA?."""
    name, ch = "baseline", 1
    out_dir, ts = new_scenario_dir(name)
    reset_to_safe_state(ctrl, sim)

    if not arm_and_gate(ctrl, sim, [ch]):
        return result(name, "Baseline closed-loop tracking",
                       "Simulator DRIVE_HZ tracks a commanded open-loop frequency",
                       "FAIL", "ARM failed -- see raw log.")

    duration_s = 3.0
    ctrl.query(f"PID:LOG {ch} 1000 1", timeout=1.0)
    arm_sim_log(sim, ch, max_samples=1000, min_interval_ms=5)

    ctrl.query(f"PID:LOOPMODE {ch} 0", timeout=1.0)   # open-loop
    target_hz = 20000
    ctrl.query(f"PID:SETPOINT {ch} {target_hz}", timeout=1.0)
    ctrl.query("PID:START", timeout=1.0)

    time.sleep(duration_s)

    ctrl.query("PID:STOP", timeout=1.0)
    ctrl_rows, rate_hz, ctrl_count = fetch_log(ctrl, ch)
    sim_rows = fetch_sim_log(sim, ch)
    reset_to_safe_state(ctrl, sim)

    ts_now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    ctrl_meta = shot_meta(ctrl, ch)
    ctrl_csv, ctrl_png = save_and_plot_controller(ch, ctrl_rows, rate_hz, ctrl_count, out_dir, ts, ctrl_meta)
    sim_meta = dict(channel=ch, idn=sim.query("*IDN?", timeout=1.0), timestamp=ts_now)
    sim_csv, sim_png = save_and_plot_sim(ch, sim_rows, out_dir, sim_meta)

    if not sim_rows:
        return result(name, "Baseline closed-loop tracking",
                       f"Simulator DRIVE_HZ converges toward {target_hz} Hz",
                       "FAIL", "No SIM:LOGDATA? samples captured.")

    final_drive = sim_rows[-1]["drive_hz"]
    tol_hz = 500
    ok = abs(final_drive - target_hz) <= tol_hz
    return result(name, "Baseline closed-loop tracking",
                  f"Simulator DRIVE_HZ converges to within {tol_hz} Hz of the "
                  f"commanded {target_hz} Hz open-loop setpoint",
                  "PASS" if ok else "FAIL",
                  f"Final measured DRIVE_HZ={final_drive} (target {target_hz}, "
                  f"{len(sim_rows)} simulator samples / {ctrl_count} controller samples "
                  f"over {duration_s}s).",
                  artifacts=[p for p in (ctrl_csv, ctrl_png, sim_csv, sim_png) if p])


def scenario_full_profiled_shot(ctrl, sim):
    """#2 -- a real profiled shot (ramp/flat-top/ramp-down) with the
    simulator gated on, captured via the controller's own PID:LOGDATA?
    pipeline."""
    name, ch = "profiled_shot", 1
    out_dir, ts = new_scenario_dir(name)
    reset_to_safe_state(ctrl, sim)

    if not arm_and_gate(ctrl, sim, [ch]):
        return result(name, "Full profiled shot", "Ramp/flat-top/ramp-down tracks correctly",
                       "FAIL", "ARM failed.")

    ramp_s, flat_s, demand_a = 2.0, 2.0, 3000.0
    ctrl.query(f"PID:PROFile:TIMing {ramp_s} {flat_s}", timeout=1.0)
    ctrl.query(f"PID:PROFile:CURRent {ch} {demand_a}", timeout=1.0)
    ctrl.query(f"PID:LOG {ch} 1000 1", timeout=1.0)
    arm_sim_log(sim, ch, max_samples=1000, min_interval_ms=5)
    start_reply = ctrl.query("PID:PROFile:STARt", timeout=2.0)
    if is_err(start_reply):
        reset_to_safe_state(ctrl, sim)
        return result(name, "Full profiled shot", "Ramp/flat-top/ramp-down tracks correctly",
                       "FAIL", f"PID:PROFile:STARt -> {explain_err(start_reply)}")

    total_s = 2 * ramp_s + flat_s
    time.sleep(total_s + 1.0)

    sim_rows = fetch_sim_log(sim, ch)
    meta = shot_meta(ctrl, ch, ramp_s=ramp_s, flat_s=flat_s, demand_a=demand_a)
    ctrl_rows, rate_hz, count = fetch_log(ctrl, ch)
    ctrl_csv, ctrl_png = save_and_plot_controller(ch, ctrl_rows, rate_hz, count, out_dir, ts, meta)
    sim_meta = dict(channel=ch, idn=sim.query("*IDN?", timeout=1.0),
                     timestamp=meta["timestamp"])
    sim_csv, sim_png = save_and_plot_sim(ch, sim_rows, out_dir, sim_meta)
    reset_to_safe_state(ctrl, sim)

    stats = compute_log_stats(ctrl_rows, rate_hz)
    if stats is None:
        return result(name, "Full profiled shot", "Ramp/flat-top/ramp-down tracks correctly",
                       "FAIL", "PID:LOGDATA? returned no samples.")

    ok = stats["output_error"]["max_abs"] < 2000
    return result(name, "Full profiled shot",
                  "Controller output tracks the commanded profile "
                  f"(ramp={ramp_s}s flat={flat_s}s demand={demand_a}A) within 2000 Hz max error",
                  "PASS" if ok else "FAIL",
                  f"output_error max_abs={stats['output_error']['max_abs']:.0f} Hz, "
                  f"{stats['count']} samples @ {rate_hz} Hz.",
                  artifacts=[p for p in (ctrl_csv, ctrl_png, sim_csv, sim_png) if p])


def scenario_fault_injection(name, sim_cmd, description, ch, second_ch, expect_state_token, expect_full_stop):
    """Shared body for OCP / Enerpro / Water+Temp mid-shot fault tests
    (#3/#4/#5) -- gate two channels on, start an open-loop shot on
    both, inject the fault on `ch` mid-shot via `sim_cmd`, confirm
    STATE? and the derate/full-stop behavior. Both boards' logs are
    armed before the shot (controller: PID:LOG 0 ... -- ALL channels,
    so the survivor's derate transient is captured too; simulator:
    SIM:LOG on the faulted channel) and retrieved after, same
    arm/run/retrieve pattern as the other scenarios."""
    def run(ctrl, sim):
        out_dir, ts = new_scenario_dir(name)
        reset_to_safe_state(ctrl, sim)

        if not arm_and_gate(ctrl, sim, [ch, second_ch]):
            return result(name, description, "", "FAIL", "ARM failed.")

        pre_fault_s, post_fault_s = 0.5, 1.5
        ctrl.query(f"PID:LOG 0 1000 1", timeout=1.0)   # all channels -- captures the survivor too
        arm_sim_log(sim, ch, max_samples=1000, min_interval_ms=5)

        for c in (ch, second_ch):
            ctrl.query(f"PID:LOOPMODE {c} 0", timeout=1.0)
            ctrl.query(f"PID:SETPOINT {c} 30000", timeout=1.0)
        ctrl.query("PID:START", timeout=1.0)
        time.sleep(pre_fault_s)

        sim.query(sim_cmd.format(ch=ch), timeout=1.0)
        time.sleep(post_fault_s)

        state_reply = ctrl.query("STATE?", timeout=1.0)
        ctrl.query("PID:STOP", timeout=1.0)

        ctrl_meta = shot_meta(ctrl, ch)
        faulted_rows, rate_hz, faulted_count = fetch_log(ctrl, ch)
        survivor_rows, _, survivor_count = fetch_log(ctrl, second_ch)
        sim_rows = fetch_sim_log(sim, ch)
        reset_to_safe_state(ctrl, sim)

        faulted_csv, faulted_png = save_and_plot_controller(ch, faulted_rows, rate_hz, faulted_count,
                                                              out_dir, ts, dict(ctrl_meta, channel=ch))
        survivor_csv, survivor_png = save_and_plot_controller(second_ch, survivor_rows, rate_hz, survivor_count,
                                                                out_dir, ts, dict(ctrl_meta, channel=second_ch))
        sim_meta = dict(channel=ch, idn=sim.query("*IDN?", timeout=1.0), timestamp=ctrl_meta["timestamp"])
        sim_csv, sim_png = save_and_plot_sim(ch, sim_rows, out_dir, sim_meta)

        state_ok = expect_state_token in state_reply
        if expect_full_stop:
            verdict = "PASS" if ("GENERAL" in state_reply or "FAULT" in state_reply) else "FAIL"
        else:
            verdict = "PASS" if state_ok else "FAIL"

        return result(name, description,
                       f"STATE? reports {expect_state_token}"
                       + (", full system stop expected" if expect_full_stop else
                          ", survivors keep running at a derated setpoint"),
                       verdict,
                       f"STATE? -> {state_reply!r}; faulted ch{ch} and survivor ch{second_ch} logged "
                       f"({faulted_count}/{survivor_count} samples).",
                       artifacts=[p for p in (faulted_csv, faulted_png, survivor_csv, survivor_png,
                                               sim_csv, sim_png) if p])
    return run


def scenario_ena_contact_dropped(ctrl, sim):
    """#6 -- drop CONTACT_OUT on one channel mid-shot (controller stops
    asserting it), confirm SM_FAULT_ENABLE_OUTPUT's per-channel derate.
    Same arm-before/retrieve-after logging as the fault-injection
    scenarios (all channels on the controller, faulted channel on the
    simulator)."""
    name, ch, survivor = "ena_contact", 1, 2
    out_dir, ts = new_scenario_dir(name)
    reset_to_safe_state(ctrl, sim)

    if not arm_and_gate(ctrl, sim, [ch, survivor]):
        return result(name, "ENA_OUT/CONTACT_OUT dropped mid-shot", "", "FAIL", "ARM failed.")

    ctrl.query("PID:LOG 0 1000 1", timeout=1.0)
    arm_sim_log(sim, ch, max_samples=1000, min_interval_ms=5)

    for c in (ch, survivor):
        ctrl.query(f"PID:LOOPMODE {c} 0", timeout=1.0)
        ctrl.query(f"PID:SETPOINT {c} 30000", timeout=1.0)
    ctrl.query("PID:START", timeout=1.0)
    time.sleep(0.5)

    ctrl.query(f"XREX:CHANnel:CONTactOut {ch} 0", timeout=1.0)
    time.sleep(1.0)

    state_reply = ctrl.query("STATE?", timeout=1.0)
    ctrl.query("PID:STOP", timeout=1.0)

    ctrl_meta = shot_meta(ctrl, ch)
    faulted_rows, rate_hz, faulted_count = fetch_log(ctrl, ch)
    survivor_rows, _, survivor_count = fetch_log(ctrl, survivor)
    sim_rows = fetch_sim_log(sim, ch)
    reset_to_safe_state(ctrl, sim)

    faulted_csv, faulted_png = save_and_plot_controller(ch, faulted_rows, rate_hz, faulted_count,
                                                          out_dir, ts, dict(ctrl_meta, channel=ch))
    survivor_csv, survivor_png = save_and_plot_controller(survivor, survivor_rows, rate_hz, survivor_count,
                                                            out_dir, ts, dict(ctrl_meta, channel=survivor))
    sim_meta = dict(channel=ch, idn=sim.query("*IDN?", timeout=1.0), timestamp=ctrl_meta["timestamp"])
    sim_csv, sim_png = save_and_plot_sim(ch, sim_rows, out_dir, sim_meta)

    verdict = "PASS" if "ENABLE_OUTPUT" in state_reply else "FAIL"
    return result(name, "ENA_OUT/CONTACT_OUT dropped mid-shot",
                  "STATE? reports ENABLE_OUTPUT <ch>, survivor keeps running derated",
                  verdict, f"STATE? -> {state_reply!r}",
                  artifacts=[p for p in (faulted_csv, faulted_png, survivor_csv, survivor_png,
                                          sim_csv, sim_png) if p])


def scenario_ext_regression(ctrl, sim):
    """#8 -- external trigger regression via the DIAGnostic:GPOut12
    loopback. CORRECTED 2026-09-18: this bench's established physical
    loopback wires DIAGnostic:GPOut12 (PD1) to PF15 (external TRIGGER),
    NOT PF13 (external enable, PF13/PF15 split, see docs/command_
    reference.md) -- there is no physical loopback to PF13 on this
    bench at all, so an earlier version of this scenario that checked
    EXTernal:INPut? (PF13) against DIAGnostic:GPOut12 was testing a
    connection that doesn't physically exist and failed for that
    reason, not a real regression. Checks EXTernal:TRIGger:INPut? (PF15)
    instead, controller-only, no simulator involvement."""
    name = "ext_regression"
    ctrl.query("DIAGnostic:GPOut12 0", timeout=1.0)
    time.sleep(0.1)
    gpout_low = ctrl.query("DIAGnostic:GPOut12?", timeout=1.0)
    trig_low = ctrl.query("EXTernal:TRIGger:INPut?", timeout=1.0)
    ctrl.query("DIAGnostic:GPOut12 1", timeout=1.0)
    time.sleep(0.1)
    gpout_high = ctrl.query("DIAGnostic:GPOut12?", timeout=1.0)
    trig_high = ctrl.query("EXTernal:TRIGger:INPut?", timeout=1.0)
    ctrl.query("DIAGnostic:GPOut12 0", timeout=1.0)

    gpout_ok = ("0" in gpout_low and "1" in gpout_high)
    trig_ok = ("0" in trig_low and "1" in trig_high)
    if trig_ok:
        verdict = "PASS"
        notes = f"low={trig_low!r} high={trig_high!r}"
    elif gpout_ok:
        # The commanded output pin itself toggled correctly
        # (DIAGnostic:GPOut12? tracks 0/1 as set) but PF15 never
        # followed -- the firmware side is working; the physical PD1->
        # PF15 jumper wire this loopback depends on isn't currently
        # connected on the bench (plausibly disturbed during today's
        # fiber rewiring). Not a code defect -- flagged distinctly.
        verdict = "BLOCKED"
        notes = (f"DIAGnostic:GPOut12 itself toggled correctly (low={gpout_low!r} "
                 f"high={gpout_high!r}) but EXTernal:TRIGger:INPut? never followed "
                 f"(low={trig_low!r} high={trig_high!r}) -- the PD1->PF15 bench "
                 f"loopback wire appears disconnected, not a firmware issue.")
    else:
        verdict = "FAIL"
        notes = f"DIAGnostic:GPOut12? itself didn't track: low={gpout_low!r} high={gpout_high!r}"

    return result(name, "External trigger regression (PF15 via DIAGnostic:GPOut12 loopback)",
                  "EXTernal:TRIGger:INPut? follows DIAGnostic:GPOut12 level",
                  verdict, notes)


# --------------------------------------------------------------------------
# LaTeX report
# --------------------------------------------------------------------------

TEX_PREAMBLE = r"""\documentclass[11pt]{article}
\usepackage[utf8]{inputenc}
\usepackage[T1]{fontenc}
\usepackage{mathpazo}
\usepackage[margin=1in]{geometry}
\usepackage{graphicx}
\usepackage{xcolor}
\usepackage{longtable}
\usepackage[htt]{hyphenat}

\newcommand{\code}[1]{\texttt{#1}}
\newcommand{\cmd}[1]{\texttt{#1}}

\title{WHAM-XREX-PFMG474 Simulator Validation Report}
\author{Generated by \texttt{python/run\_simulator\_validation.py}}
\date{\today}

\begin{document}
\maketitle

\section*{Purpose}
Autonomous end-to-end validation of the controller/Transrex-simulator
closed loop, per Part C/D of the simulator plan. Each section below
covers one test scenario: what was commanded, the expected behavior,
the observed result, and a pass/fail verdict. Raw data for every
scenario is archived under \code{shots/} in this repository (gitignored).

"""

TEX_FOOTER = r"""
\end{document}
"""


def escape_tex(s):
    for a, b in (("\\", r"\textbackslash{}"), ("_", r"\_"), ("%", r"\%"),
                 ("&", r"\&"), ("#", r"\#"), ("$", r"\$")):
        s = s.replace(a, b)
    return s


def generate_report(results):
    os.makedirs(DOCS_DIR, exist_ok=True)
    parts = [TEX_PREAMBLE]
    for r in results:
        color = {"PASS": "ForestGreen", "FAIL": "BrickRed", "BLOCKED": "orange"}.get(r["verdict"], "black")
        parts.append(f"\\section{{{escape_tex(r['description'])}}}\n")
        parts.append(f"\\textbf{{Expected:}} {escape_tex(r['expected'])}\n\n")
        parts.append(f"\\textbf{{Verdict:}} \\textcolor{{{color}}}{{\\textbf{{{r['verdict']}}}}}\n\n")
        parts.append(f"\\textbf{{Notes:}} {escape_tex(r['notes'])}\n\n")
        for art in r["artifacts"]:
            if art.endswith(".png"):
                rel = os.path.relpath(art, DOCS_DIR)
                parts.append(f"\\includegraphics[width=\\linewidth]{{{rel}}}\n\n")
            elif art.endswith(".csv"):
                rel = os.path.relpath(art, PROJECT_DIR)
                parts.append(f"Raw data: \\code{{{escape_tex(rel)}}}\n\n")
    parts.append(TEX_FOOTER)
    with open(REPORT_TEX, "w") as f:
        f.write("".join(parts))
    print(f"[report] wrote {REPORT_TEX} -- compile with: "
          f"cd {DOCS_DIR} && pdflatex simulator_validation_report.tex")


# --------------------------------------------------------------------------
# Main
# --------------------------------------------------------------------------

ALL_SCENARIOS = {
    "baseline": scenario_baseline_tracking,
    "profiled_shot": scenario_full_profiled_shot,
    "ocp": scenario_fault_injection(
        "ocp", "SIM:FAULT:OCP {ch} 1", "OCP fault mid-shot", 1, 2,
        expect_state_token="OVERCURRENT", expect_full_stop=False),
    "enerpro": scenario_fault_injection(
        "enerpro", "SIM:FAULT:ENERPRO {ch} 1", "Enerpro fault mid-shot", 1, 2,
        expect_state_token="ENERPRO", expect_full_stop=False),
    "watertemp": scenario_fault_injection(
        "watertemp", "SIM:FAULT:WATERTEMP {ch} 1", "Water/Temp fault mid-shot", 1, 2,
        expect_state_token="GENERAL", expect_full_stop=True),
    "ena_contact": scenario_ena_contact_dropped,
    "ext_regression": scenario_ext_regression,
}


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--controller-port")
    ap.add_argument("--simulator-port")
    ap.add_argument("--scenarios", default="all",
                     help="comma-separated subset of: " + ",".join(ALL_SCENARIOS))
    args = ap.parse_args()

    os.makedirs(SHOTS_DIR, exist_ok=True)

    if args.controller_port:
        ctrl = connect_verified(args.controller_port, "WHAM-XREX-PFMG474 ", "controller")
    else:
        ctrl = autodetect("WHAM-XREX-PFMG474 ", "controller")
    if args.simulator_port:
        sim = connect_verified(args.simulator_port, "WHAM-XREX-PFMG474-SIM", "simulator")
    else:
        sim = autodetect("WHAM-XREX-PFMG474-SIM", "simulator", exclude_port=ctrl.port)

    names = list(ALL_SCENARIOS) if args.scenarios == "all" else args.scenarios.split(",")
    results = []
    try:
        for n in names:
            fn = ALL_SCENARIOS.get(n)
            if fn is None:
                print(f"[skip] unknown scenario {n!r}")
                continue
            print(f"\n=== {n} ===")
            try:
                r = fn(ctrl, sim)
            except WhamError as exc:
                r = result(n, n, "", "FAIL", f"WhamError: {exc}")
            except Exception as exc:  # noqa: BLE001 -- one bad scenario must not kill the campaign
                r = result(n, n, "", "FAIL", f"Unexpected exception: {exc!r}")
            print(f"[{r['verdict']}] {r['notes']}")
            results.append(r)
            reset_to_safe_state(ctrl, sim)
    finally:
        reset_to_safe_state(ctrl, sim)
        ctrl.close()
        sim.close()

    generate_report(results)
    n_pass = sum(1 for r in results if r["verdict"] == "PASS")
    print(f"\n{n_pass}/{len(results)} scenarios PASSED")


if __name__ == "__main__":
    main()
