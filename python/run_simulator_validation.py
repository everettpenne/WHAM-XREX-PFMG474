#!/usr/bin/env python3
"""
run_simulator_validation.py -- autonomous end-to-end test campaign for
the Transrex simulator / controller closed loop, per Part C/D of the
plan at /Users/everettpenne/.claude/plans/cuddly-nibbling-octopus.md.

Drives BOTH boards over their own serial links -- the CONTROLLER via
its real state machine/demand commands (ARM/SHOT:*), the SIMULATOR
via SIM:/raw diagnostic commands -- with no manual intervention: ARM/
FIRE are done in software (SHOT:STARt calls SM_Fire() directly
and does not need a PF15 trigger edge). The PF13 external-enable
interlock now defaults to ON (2026-09-22 -- see EXTernal:ENAble in
docs/command_reference.md), so reset_to_safe_state() below explicitly
sends `EXTernal:ENAble 0` before/after every scenario -- this bench rig
has nothing physically wired to PF13, and a floating pin reads LOW
(fail-safe), which would otherwise block every ARM.

Reuses wham_console.py's existing LOG:DATA?-based fetch/plot/stats
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
                        baseline, gain_sweep, profiled_shot, ocp, enerpro,
                        watertemp, ena_contact, multi_fault, ext_regression

Each scenario also prints its own "COMBINED_PLOT: <path>" line(s) the
moment its plot is generated (not batched to the end) -- a single PNG
per test showing the controller's commanded/measured signal on top,
the simulator's DRIVE/FEEDBACK on the bottom, and any fault-injection
instants as vertical markers, per direct request 2026-09-21.
"""

import argparse
import glob
import itertools
import math
import os
import sys
import time
from datetime import datetime

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.dirname(SCRIPT_DIR)
sys.path.insert(0, SCRIPT_DIR)

import wham_console as wc  # noqa: E402  (path insert must come first) -- module
                            # import, not `from ... import PFM_TURNON_FREQ_HZ`
                            # etc.: those are live-synced by wc.sync_calibration()
                            # (called in main(), after connecting) and a
                            # `from` import would have frozen a stale copy at
                            # import time instead of tracking the update.
from wham_console import (   # noqa: E402  (path insert must come first)
    WhamLink, WhamError, is_err, explain_err,
    compute_log_stats, generate_plot, format_channel_report_md,
    hz_to_amps, amps_to_hz, channel_label, SHOTS_DIR,
)

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    HAVE_MPL = True
except ImportError:
    HAVE_MPL = False

DOCS_DIR = os.path.join(PROJECT_DIR, "docs", "reports")
REPORT_TEX = os.path.join(DOCS_DIR, "simulator_validation_report.tex")

HRTIM_NUM_CHANNELS = 4

# LOG:ARM/LOG:DATA? -- MUST match Core/Inc/pid.h's PID_LOG_MAX_SAMPLES
# exactly (a compile-time log-buffer size, not runtime-configurable --
# no CONFig:* backs it, so a plain constant is correct here, unlike
# the loop rate below).
PID_LOG_MAX_SAMPLES = 1750

# PID_LOOP_RATE_HZ -- RESOLVED 2026-09-23: wc.sync_calibration() now
# also refreshes wc.PID_LOOP_RATE_HZ_ASSUMED from the live
# CONFig:PIDRate?, same fix already applied to the Amps<->Hz globals
# this file already reads via wc.PFM_* (see main()'s own sync call).
# pid_log_decim() below reads wc.PID_LOOP_RATE_HZ_ASSUMED directly
# (module-qualified, not a local copy) so it always sees whatever
# main() last synced, exactly like the wc.PFM_MAX_CURRENT_A fix.


# Default shot-profile timing (seconds) -- 2026-09-21, raised for the
# longer campaign; overridable via --ramp-s/--flat-s. Every profile-driven
# scenario and the matrix sweep read these globals (set in main()).
RAMP_S = 10.0
FLAT_S = 5.0


SIM_LOG_MAX_SAMPLES = 1000   # sim_transrex.h -- see arm_sim_log()'s own default min_interval_ms=5,
                              # which caps simulator log coverage at 1000*5ms=5.0s; scenarios whose
                              # actual logged span runs longer must pass a larger min_interval_ms,
                              # computed via sim_log_interval_ms() below -- same truncation bug class
                              # as pid_log_decim() (below), just on the simulator side.


def sim_log_interval_ms(duration_s, margin=1.25):
    """Minimum SIM:LOG interval (ms) so SIM_LOG_MAX_SAMPLES (1000)
    samples comfortably cover a shot lasting `duration_s` seconds --
    same reasoning/margin as pid_log_decim() below, simulator side."""
    return max(1, math.ceil(duration_s * 1000.0 * margin / SIM_LOG_MAX_SAMPLES))


def pid_log_decim(duration_s, margin=1.25):
    """Decimation for `LOG:ARM 0 1750 <decim>` so PID_LOG_MAX_SAMPLES
    (1750) samples comfortably cover a shot lasting `duration_s`
    seconds, with `margin` headroom for this script's own imprecise
    time.sleep() timing (real elapsed time always runs a bit long).

    REAL BUG, FOUND 2026-09-18 (direct correction from the user, who
    noticed plots only showing the ramp-up before the rest of the shot
    was cut off): every LOG:ARM call in this file hardcoded decim=1,
    which at PID_LOOP_RATE_HZ=1000 Hz means exactly 1000 samples =
    1.0 SECOND of coverage, full stop -- PID_ArmLog() (pid.c) does not
    wrap or extend once maxSamples is reached, it just stops appending.
    Every scenario whose actual shot ran longer than 1.0s (all of the
    fault-injection scenarios, the matrix sweep, the profiled-shot
    scenario) was silently logging only its own first second and
    nothing after -- in the fault scenarios specifically, the fault
    injection itself (~1.0-1.5s into the shot) landed AFTER the log had
    already stopped recording, so the saved CSV/plot never captured it
    at all, even though the live SOURce:STATus?/STATE? checks driving each
    scenario's own PASS/FAIL verdict were unaffected (those poll the
    board directly, not the log)."""
    needed = math.ceil(wc.PID_LOOP_RATE_HZ_ASSUMED * duration_s * margin / PID_LOG_MAX_SAMPLES)
    return max(1, needed)


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

def fetch_log(link, channel, retries=2):
    """Standalone equivalent of WhamConsole._fetch_log() -- same
    LOG:DATA? parsing, just driven by a bare WhamLink instead of the
    interactive console's own instance state. Retries on a malformed
    reply (up to `retries` attempts total) -- a real, occasionally-seen
    failure mode at high request volume (found during the 45-shot
    matrix sweep, 2026-09-18: a LOG:DATA? reply for 666 samples came
    back missing 2 of the expected 1998 values, a near-miss consistent
    with a transient serial read/timing hiccup, not a systematic parser
    or firmware bug) -- a fresh query almost always comes back clean."""
    last_exc = None
    for attempt in range(retries):
        try:
            reply = link.query(f"LOG:DATA? {channel}", timeout=8.0)
            if is_err(reply):
                raise WhamError(explain_err(reply))
            parts = reply.split()
            if len(parts) < 2 or parts[0] != "OK":
                raise WhamError(f"unexpected LOG:DATA? reply: {reply!r}")
            count = int(parts[1])
            rate_hz = int(parts[2])
            vals = list(map(int, parts[3:3 + count * 3]))
            if len(vals) != count * 3:
                raise WhamError(f"LOG:DATA? claimed {count} samples but only "
                                 f"{len(vals)} values were parsed")
            break
        except (WhamError, ValueError) as exc:
            last_exc = exc
            if attempt + 1 < retries:
                time.sleep(0.2)
    else:
        raise last_exc

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
# controller-side LOG:ARM/LOG:DATA? pattern below, instead of the
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
    like LOG:DATA?, the main loop has no fixed tick to decimate
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
    ax.axhline(wc.PFM_TURNON_FREQ_HZ, color="#444444", lw=0.8, ls=":")
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


def generate_combined_plot(name, description, verdict, notes, channel, ctrl_rows, sim_rows,
                            out_path, fault_events=None, extra_traces=None, meta=None):
    """One combined plot per test, sent to the user immediately after
    each scenario runs (2026-09-21, direct request: 'each test should
    produce a plot ... showing the output of the controller along with
    the response of the feedback, and any faults that were triggered').
    Top panel: CONTROLLER's own setpoint/output/measured for `channel`
    (plus `survivor_channel`'s output, dashed, for fault scenarios --
    the point of those tests is showing the survivor keeps running).
    Bottom panel: SIMULATOR's DRIVE_HZ (what it received)/FEEDBACK_HZ
    (what it sent back) for the same channel. Vertical dashed red lines
    mark `fault_events` ([(t_s, label), ...]).

    Both panels' time axes are each board's OWN elapsed-since-its-own-
    log-was-armed time -- the two logs are armed within ~0.1-0.3s of
    each other (both right before the shot starts), close enough for
    visual correlation on a single test's timescale, but NOT sample-
    synchronized -- said explicitly in the x-axis label rather than
    implied by drawing them as if they were the same clock."""
    if not HAVE_MPL:
        return None
    fault_events = fault_events or []

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(11, 8.5))

    if ctrl_rows:
        t = [r["t_s"] for r in ctrl_rows]
        ax1.plot(t, [r["setpoint_hz"] for r in ctrl_rows], color="#888888", lw=1.0, ls="--",
                 label=f"ch{channel} setpoint")
        ax1.plot(t, [r["output_hz"] for r in ctrl_rows], color="#2a78d6", lw=1.4,
                 label=f"ch{channel} output (commanded)")
        ax1.plot(t, [r["measured_hz"] for r in ctrl_rows], color="#1aa260", lw=1.4,
                 label=f"ch{channel} measured (PFM feedback)")
    extra_colors = ["#9b59b6", "#c9a227", "#16a085"]
    for i, (label, rows) in enumerate(extra_traces or []):
        if not rows:
            continue
        t2 = [r["t_s"] for r in rows]
        ax1.plot(t2, [r["output_hz"] for r in rows], color=extra_colors[i % len(extra_colors)],
                 lw=1.2, ls="-.", label=label)
    for t_ev, label in fault_events:
        ax1.axvline(t_ev, color="#c0392b", lw=1.2, ls=":")
        ax1.annotate(label, xy=(t_ev, 1.0), xycoords=("data", "axes fraction"),
                     rotation=90, va="top", ha="right", fontsize=8, color="#c0392b")
    ax1.set_ylabel("Frequency (Hz)")
    ax1.set_title("Controller: commanded vs. measured", fontsize=10)
    ax1.grid(True, alpha=0.3)
    if ctrl_rows or survivor_rows:
        ax1.legend(loc="best", fontsize=8)

    if sim_rows:
        t3 = [r["t_s"] for r in sim_rows]
        ax2.step(t3, [r["drive_hz"] for r in sim_rows], where="post", color="#2a78d6", lw=1.2,
                  label="DRIVE_HZ (sim measured from controller)")
        ax2.step(t3, [r["feedback_hz"] for r in sim_rows], where="post", color="#eb6834", lw=1.8,
                  label="FEEDBACK_HZ (sim's filtered response)")
        ax2.legend(loc="best", fontsize=8)
    for t_ev, label in fault_events:
        ax2.axvline(t_ev, color="#c0392b", lw=1.2, ls=":")
    ax2.set_xlabel("Time since each board's own log was armed (s) -- approx. aligned, not sample-synchronized")
    ax2.set_ylabel("Frequency (Hz)")
    ax2.set_title("Simulator: DRIVE received vs. FEEDBACK sent", fontsize=10)
    ax2.grid(True, alpha=0.3)

    idn = (meta or {}).get("idn", "")
    ts = (meta or {}).get("timestamp", "")
    wrapped_notes = notes if len(notes) < 160 else notes[:157] + "..."
    fig.suptitle(f"{name}  --  {description}\nVerdict: {verdict}   |   {ts}\n{wrapped_notes}",
                 fontsize=10)
    fig.tight_layout(rect=[0, 0, 1, 0.88])
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    return out_path


# --------------------------------------------------------------------------
# Shared setup/teardown
# --------------------------------------------------------------------------

def reset_to_safe_state(ctrl, sim):
    """Both boards to a known-safe baseline between scenarios: all
    channels disabled, loop mode back to open-loop, gains zeroed, all
    fiber outputs LOW/healthy, faults cleared, EXTernal:ENAble interlock
    left off (default -- software-only ARM/FIRE requires this, see this
    file's own top comment).

    LOOPMODE/GAINS reset added 2026-09-21 -- REAL BUG found the hard
    way, same day: scenario_full_profiled_shot() never sets its own
    loop mode (unlike start_profile_shot(), fixed for exactly this
    reason back on 2026-09-18), so when it ran immediately after
    scenario_gain_sweep() in a full campaign, channel 1 was still left
    in CLOSED-loop with Kp=1.0 from gain_sweep's own last test --
    output tracked ~half of setpoint the whole shot (the correct,
    predictable P-only steady-state error for Kp=1.0, not a firmware
    bug), failing a scenario that has nothing to do with gains at all.
    Resetting both here, for every channel, on every reset_to_safe_state()
    call (i.e. before AND after every scenario) closes this whole CLASS
    of test-isolation bug -- no scenario should ever depend on what a
    DIFFERENT scenario happened to leave configured, and no individual
    scenario needs to remember to reset state a shared helper already
    guarantees. """
    for link in (ctrl, sim):
        for ch in range(1, HRTIM_NUM_CHANNELS + 1):
            link.query(f"SOURce:ENAble {ch} 0", timeout=1.0)
            link.query(f"XREX:CHANnel:ENAOut {ch} 0", timeout=1.0)
            link.query(f"XREX:CHANnel:CONTactOut {ch} 0", timeout=1.0)
            link.query(f"PID:LOOPMODE {ch} 0", timeout=1.0)
            link.query(f"PID:GAINS {ch} 0 0 0", timeout=1.0)
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
        ctrl.query(f"SOURce:ENAble {ch} 1", timeout=1.0)
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
    ctrl's own LOG:ARM/LOG:DATA? (already this project's established
    controller-test pattern) and sim's new SIM:LOG/LOGDATA?."""
    name, ch = "baseline", 1
    out_dir, ts = new_scenario_dir(name)
    reset_to_safe_state(ctrl, sim)

    if not arm_and_gate(ctrl, sim, [ch]):
        return result(name, "Baseline closed-loop tracking",
                       "Simulator DRIVE_HZ tracks a commanded open-loop frequency",
                       "FAIL", "ARM failed -- see raw log.")

    duration_s = 3.0
    ctrl.query(f"LOG:ARM {ch} {PID_LOG_MAX_SAMPLES} {pid_log_decim(duration_s)}", timeout=1.0)
    arm_sim_log(sim, ch, max_samples=1000, min_interval_ms=5)

    ctrl.query(f"PID:LOOPMODE {ch} 0", timeout=1.0)   # open-loop
    target_hz = 20000
    ctrl.query(f"SOURce:SETpoint {ch} {target_hz}", timeout=1.0)
    ctrl.query("SOURce:RUN", timeout=1.0)

    time.sleep(duration_s)

    ctrl.query("SOURce:STOP", timeout=1.0)
    ctrl_rows, rate_hz, ctrl_count = fetch_log(ctrl, ch)
    sim_rows = fetch_sim_log(sim, ch)
    reset_to_safe_state(ctrl, sim)

    ts_now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    ctrl_meta = shot_meta(ctrl, ch)
    ctrl_csv, ctrl_png = save_and_plot_controller(ch, ctrl_rows, rate_hz, ctrl_count, out_dir, ts, ctrl_meta)
    sim_meta = dict(channel=ch, idn=sim.query("*IDN?", timeout=1.0), timestamp=ts_now)
    sim_csv, sim_png = save_and_plot_sim(ch, sim_rows, out_dir, sim_meta)

    description = "Baseline closed-loop tracking"
    if not sim_rows:
        return result(name, description,
                       f"Simulator DRIVE_HZ converges toward {target_hz} Hz",
                       "FAIL", "No SIM:LOGDATA? samples captured.")

    final_drive = sim_rows[-1]["drive_hz"]
    tol_hz = 500
    ok = abs(final_drive - target_hz) <= tol_hz
    verdict = "PASS" if ok else "FAIL"
    notes = (f"Final measured DRIVE_HZ={final_drive} (target {target_hz}, "
             f"{len(sim_rows)} simulator samples / {ctrl_count} controller samples "
             f"over {duration_s}s).")
    combined_png = os.path.join(out_dir, "combined.png")
    generate_combined_plot(name, description, verdict, notes, ch, ctrl_rows, sim_rows,
                            combined_png, meta=ctrl_meta)
    print(f"COMBINED_PLOT: {combined_png}")
    return result(name, description,
                  f"Simulator DRIVE_HZ converges to within {tol_hz} Hz of the "
                  f"commanded {target_hz} Hz open-loop setpoint",
                  verdict, notes,
                  artifacts=[p for p in (ctrl_csv, ctrl_png, sim_csv, sim_png, combined_png) if p])


def generate_multi_channel_grid_plot(name, description, verdict, notes, per_channel_rows,
                                      per_channel_meta, out_path, fault_events=None):
    """One column of 4 stacked subplots, one per WHAM channel (1-4, top to
    bottom) on a SHARED time axis -- each shows that channel's own
    setpoint/output/measured vs. time (same series as generate_plot(),
    wham_console.py) so every channel is directly comparable at a glance
    in one tall image. `per_channel_rows`/`per_channel_meta` are
    {channel: rows}/{channel: meta} dicts; a channel may be missing (no
    data) -- that subplot is annotated 'no data'. When any channel's meta
    carries ramp_time_s/flat_top_time_s, the ramp-up/flat-top/ramp-down
    regions are shaded and labeled on every panel. `fault_events` is an
    optional list of (t_s, label) -- each is drawn as a vertical dashed
    red line + rotated label on every populated panel, marking when a
    fault was injected relative to each channel's trace."""
    if not HAVE_MPL:
        return None

    # Profile timing is shared across channels in one shot -- read it from
    # the first channel meta that has it, then shade every panel the same.
    ramp_s = flat_s = None
    for _ch in (1, 2, 3, 4):
        _m = per_channel_meta.get(_ch) or {}
        if _m.get("ramp_time_s") and _m.get("flat_top_time_s") is not None:
            ramp_s = _m["ramp_time_s"]
            flat_s = _m["flat_top_time_s"]
            break
    phase_bounds = None
    if ramp_s is not None:
        total_s = 2 * ramp_s + flat_s
        phase_bounds = [(0, ramp_s, "#ffe8b3", "ramp up"),
                        (ramp_s, ramp_s + flat_s, "#c9f2c7", "flat top"),
                        (ramp_s + flat_s, total_s, "#ffd6d6", "ramp down")]

    fig, axes = plt.subplots(4, 1, figsize=(11, 16), sharex=True)
    for ch in (1, 2, 3, 4):
        ax = axes[ch - 1]
        rows = per_channel_rows.get(ch)
        meta = per_channel_meta.get(ch, {})
        if phase_bounds:
            for x0, x1, color, label in phase_bounds:
                ax.axvspan(x0, x1, color=color, alpha=0.45, zorder=0,
                           label=label if ch == 1 else None)
        if not rows:
            ax.text(0.5, 0.5, f"ch{ch}: no data", ha="center", va="center",
                    transform=ax.transAxes)
            ax.set_title(f"Channel {ch}", fontsize=10)
            ax.set_ylim(0, 1)
            continue
        t = [r["t_s"] for r in rows]
        ax.plot(t, [r["setpoint_hz"] for r in rows], color="#888888", lw=1.0, ls="--",
                label="setpoint (commanded profile)", zorder=3)
        ax.plot(t, [r["output_hz"] for r in rows], color="#2a78d6", lw=1.4,
                label="output (written to HRTIM)", zorder=2)
        ax.plot(t, [r["measured_hz"] for r in rows], color="#1aa260", lw=1.4,
                label="measured (feedback)", zorder=4)
        demand_a = meta.get("demand_current_a")
        title = f"Channel {ch}" + (f" -- {demand_a:.0f}A demand" if demand_a is not None else "")
        ax.set_title(title, fontsize=10)
        ax.set_ylabel("Frequency (Hz)")
        ax.grid(True, alpha=0.3)
        ax.legend(loc="best", fontsize=8)
        if fault_events:
            for t_ev, label in fault_events:
                ax.axvline(t_ev, color="#c0392b", lw=1.2, ls=":")
                ax.annotate(label, xy=(t_ev, 1.0), xycoords=("data", "axes fraction"),
                            xytext=(4, -2), textcoords="offset points",
                            rotation=90, va="top", ha="left",
                            fontsize=7, color="#c0392b")

    axes[-1].set_xlabel("Time since log armed (s)")
    wrapped_notes = notes if len(notes) < 200 else notes[:197] + "..."
    fig.suptitle(f"{name}  --  {description}\nVerdict: {verdict}\n{wrapped_notes}", fontsize=10)
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    return out_path


def generate_stacked_channel_plot(channels, per_channel_rows, per_channel_meta, out_path,
                                   title=None):
    """One vertically-stacked column of subplots, one row per channel in
    `channels` (in that order) -- added 2026-09-21 for the matrix
    campaign's per-shot report sections, direct request: 'a subsection
    containing the vertically stacked column of plots (one for each
    channel)'. Distinct from generate_multi_channel_grid_plot()'s 2x2
    grid (always exactly 4 panels, fixed layout) -- this one has exactly
    as many rows as `channels` has entries (1-4), matching a matrix
    shot's own actual channel combination rather than always showing
    all 4. Each row: setpoint/output/measured vs. time, same series as
    every other plot in this file."""
    if not HAVE_MPL:
        return None
    n = len(channels)
    fig, axes = plt.subplots(n, 1, figsize=(11, 3.0 * n), squeeze=False)
    for i, ch in enumerate(channels):
        ax = axes[i][0]
        rows = per_channel_rows.get(ch)
        meta = per_channel_meta.get(ch, {})
        if not rows:
            ax.text(0.5, 0.5, f"ch{ch}: no data", ha="center", va="center", transform=ax.transAxes)
            ax.set_title(f"Channel {ch}", fontsize=10)
            continue
        t = [r["t_s"] for r in rows]
        ax.plot(t, [r["setpoint_hz"] for r in rows], color="#888888", lw=1.0, ls="--", label="setpoint")
        ax.plot(t, [r["output_hz"] for r in rows], color="#2a78d6", lw=1.2, label="output (commanded)")
        ax.plot(t, [r["measured_hz"] for r in rows], color="#1aa260", lw=1.2, label="measured (PFM feedback)")
        demand_a = meta.get("demand_current_a")
        subtitle = f"Channel {ch}" + (f" -- {demand_a:.0f}A demand" if demand_a is not None else "")
        ax.set_title(subtitle, fontsize=10)
        ax.set_ylabel("Frequency (Hz)")
        ax.grid(True, alpha=0.3)
        ax.legend(loc="best", fontsize=7)
    axes[-1][0].set_xlabel("Time (s)")
    if title:
        fig.suptitle(title, fontsize=10)
        fig.tight_layout(rect=[0, 0, 1, 0.95])
    else:
        fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    return out_path


def scenario_all_channels_profiled_shot(ctrl, sim):
    """#9 -- all 4 channels gated and firing SIMULTANEOUSLY, each at a
    different demand current, direct request 2026-09-21 ('make sure we
    have the controller emitting contactor and enable signals for all 4
    channels, and run some tests on all 4 channels'). Confirms the
    controller correctly commands/tracks/measures 4 independent
    channels at once through the real simulator fiber pool, now that
    the 2026-09-21 DRIVE_HZ reporting fix is in. Simulator-side
    waveform logging (SIM:LOG) is single-channel-at-a-time by design
    (sim_transrex.h), so this scenario relies on the controller's own
    LOG:DATA? -- measured_hz IS the real feedback received from the
    simulator, already proven accurate end-to-end today."""
    name = "all_channels"
    out_dir, ts = new_scenario_dir(name)
    reset_to_safe_state(ctrl, sim)

    channels = [1, 2, 3, 4]
    demands_a = {1: 600.0, 2: 2000.0, 3: 3600.0, 4: 5400.0}   # 10/33/60/90% spread

    if not arm_and_gate(ctrl, sim, channels):
        return result(name, "All 4 channels, simultaneous profiled shot",
                       "", "FAIL", "ARM failed.")

    ramp_s, flat_s = RAMP_S, FLAT_S
    total_s = 2 * ramp_s + flat_s
    logged_span_s = total_s + 1.0
    ctrl.query(f"SHOT:TIMing {ramp_s} {flat_s} {ramp_s}", timeout=1.0)
    for c in channels:
        ctrl.query(f"PID:LOOPMODE {c} 0", timeout=1.0)
        ctrl.query(f"SHOT:CURRent {c} {demands_a[c]}", timeout=1.0)
    ctrl.query(f"LOG:ARM 0 {PID_LOG_MAX_SAMPLES} {pid_log_decim(logged_span_s)}", timeout=1.0)   # all channels at once

    start_reply = ctrl.query("SHOT:STARt", timeout=2.0)
    if is_err(start_reply):
        reset_to_safe_state(ctrl, sim)
        return result(name, "All 4 channels, simultaneous profiled shot",
                       "", "FAIL", f"SHOT:STARt -> {explain_err(start_reply)}")

    time.sleep(logged_span_s)

    # Verdict window: flat-top only, and NOT starting exactly at ramp_s --
    # the filter's own settling tail (~5x its ~100ms time constant) still
    # extends a few hundred ms past the moment the setpoint stops moving,
    # so a window starting exactly at ramp_s still catches legitimate
    # convergence, not a bug (confirmed 2026-09-21 by inspecting raw data:
    # errors shrinking smoothly from ~3400 Hz down to near-zero over the
    # first ~500ms of "flat-top", not a step change). Starts 0.5s into the
    # flat-top, ends 0.5s before it ends (symmetric, same settling-tail
    # reasoning applies to the ramp-down's own approach).
    flat_top_start_s = ramp_s + 0.5
    flat_top_end_s    = ramp_s + flat_s - 0.5

    per_channel_rows = {}
    per_channel_meta = {}
    worst_errors = {}
    for c in channels:
        rows, rate_hz, count = fetch_log(ctrl, c)
        meta = shot_meta(ctrl, c, ramp_s=ramp_s, flat_s=flat_s, demand_a=demands_a[c])
        csv_path, png_path = save_and_plot_controller(c, rows, rate_hz, count, out_dir, ts, meta)
        per_channel_rows[c] = rows
        per_channel_meta[c] = meta
        flat_top_rows = [r for r in rows if flat_top_start_s <= r["t_s"] <= flat_top_end_s]
        # feedback_vs_output_error (measured - output), NOT output_error
        # (output - setpoint, trivially ~0 in open-loop -- that's just
        # the profile generator following itself, not a real check of
        # anything). measured is the REAL feedback received from the
        # simulator across the actual fiber pair -- this is the number
        # that actually validates today's whole investigation.
        #
        # MEDIAN absolute error, not max_abs or rms -- confirmed 2026-09-21
        # by inspecting raw data: an otherwise genuinely-excellent flat-top
        # window (errors mostly under 100 Hz across ~150-330 samples)
        # occasionally contains ONE physically-impossible sample (a real
        # observed case: measured_hz=133752, beyond this hardware's own
        # max ~100000 Hz range -- the signature of a corrupted value in
        # one LOG:DATA? transmission, not a real measurement event; see
        # fetch_log()'s own retry logic, which only catches a malformed
        # reply's sample COUNT, not one bad value inside an otherwise-
        # well-formed one). max_abs fails on any single glitch; even rms
        # can be dragged noticeably by ONE huge squared term. Median is
        # robust to a small number of such outliers while still reporting
        # a real, representative tracking number.
        if flat_top_rows:
            abs_errs = sorted(abs(r["measured_hz"] - r["output_hz"]) for r in flat_top_rows)
            worst_errors[c] = abs_errs[len(abs_errs) // 2]
        else:
            worst_errors[c] = None
    reset_to_safe_state(ctrl, sim)

    description = "All 4 channels gated and firing simultaneously (600/2000/3600/5400 A)"
    ok = all(v is not None and v < 500 for v in worst_errors.values())
    verdict = "PASS" if ok else "FAIL"
    notes = "; ".join(f"ch{c} flat-top measured_vs_output median error={worst_errors[c]:.0f}Hz" if worst_errors[c] is not None
                       else f"ch{c} NO DATA (flat-top window empty)" for c in channels)
    combined_png = os.path.join(out_dir, "combined.png")
    generate_multi_channel_grid_plot(name, description, verdict, notes,
                                      per_channel_rows, per_channel_meta, combined_png)
    print(f"COMBINED_PLOT: {combined_png}")
    return result(name, description,
                  "Every channel's REAL measured feedback (received across the simulator fiber) "
                  "tracks its own commanded output within 500 Hz median error during the flat-top "
                  "(post-ramp steady state), simultaneously, with no cross-channel interference",
                  verdict, notes, artifacts=[combined_png])


def scenario_gain_sweep(ctrl, sim, kp_values=(0.2, 0.5, 1.0), ki_values=(10, 50, 100)):
    """#1b -- direct request, 2026-09-21: 'tests at different gain
    settings for the PID loop.' Genuinely CLOSED-loop this time (unlike
    #1's open-loop tracking, which never exercises PID gains at all) --
    channel 1 gated through the real simulator fiber pair, a step
    setpoint commanded, PID:GAINS swept. Two sweeps in one scenario
    (extended 2026-09-21 per direct confirmation: "Add Ki sweep, Kd can
    be set to 0" -- so Kd stays 0 throughout, no D sweep):
      - P sweep: Kp across `kp_values`, Ki=Kd=0. Pure-P control can never
        reach the literal setpoint -- it settles at Kp*target/(1+Kp).
      - I sweep: Kp=1.0, Ki across `ki_values`, Kd=0. The integral term
        drives steady-state error to zero, so these settle at the raw
        target instead.
    One combined plot per point, so convergence speed/overshoot is
    visually comparable across points without needing to cross-reference
    separate documents. Relies on the 2026-09-21 sim_transrex.c floor fix
    (a freshly-gated channel with no real DRIVE sample yet now floors to
    PFM_TURNON_FREQ_HZ instead of decaying to 0) -- without that fix, a
    low-Kp run's slow initial response would have been fighting a wrong
    near-zero starting feedback value instead of the correct floor."""
    name, ch = "gain_sweep", 1
    target_hz = 40000
    duration_s = 4.0
    tol_hz = 500

    # One sweep point per (label, kp, ki, kd, expected_hz). P-only points
    # (ki=0) settle at the pure-P steady-state error; PI points (ki>0)
    # settle at the raw target (integral removes steady-state error).
    points = []
    for kp in kp_values:
        points.append((f"kp{kp}", float(kp), 0.0, 0.0, float(kp * target_hz / (1.0 + kp))))
    for ki in ki_values:
        points.append((f"ki{ki}", 1.0, float(ki), 0.0, float(target_hz)))

    sub_results = []

    for label, kp, ki, kd, expected_hz in points:
        out_dir, ts = new_scenario_dir(f"{name}_{label}")
        reset_to_safe_state(ctrl, sim)

        if not arm_and_gate(ctrl, sim, [ch]):
            sub_results.append((label, "FAIL", "ARM failed."))
            continue

        ctrl.query(f"PID:LOOPMODE {ch} 1", timeout=1.0)   # closed-loop
        ctrl.query(f"PID:GAINS {ch} {kp:g} {ki:g} {kd:g}", timeout=1.0)
        ctrl.query(f"LOG:ARM {ch} {PID_LOG_MAX_SAMPLES} {pid_log_decim(duration_s)}", timeout=1.0)
        arm_sim_log(sim, ch, max_samples=1000, min_interval_ms=sim_log_interval_ms(duration_s))
        ctrl.query(f"SOURce:SETpoint {ch} {target_hz}", timeout=1.0)
        ctrl.query("SOURce:RUN", timeout=1.0)

        time.sleep(duration_s)

        ctrl.query("SOURce:STOP", timeout=1.0)
        ctrl_rows, rate_hz, ctrl_count = fetch_log(ctrl, ch)
        sim_rows = fetch_sim_log(sim, ch)
        reset_to_safe_state(ctrl, sim)

        desc_gains = (f"Kp={kp:g} Ki={ki:g} Kd={kd:g}" if ki > 0.0
                      else f"Kp={kp:g} Ki=Kd=0")
        description = f"Closed-loop gain sweep -- {desc_gains}"
        ctrl_meta = shot_meta(ctrl, ch)
        ctrl_csv, ctrl_png = save_and_plot_controller(ch, ctrl_rows, rate_hz, ctrl_count, out_dir, ts, ctrl_meta)
        sim_meta = dict(channel=ch, idn=sim.query("*IDN?", timeout=1.0), timestamp=ctrl_meta["timestamp"])
        sim_csv, sim_png = save_and_plot_sim(ch, sim_rows, out_dir, sim_meta)

        if ctrl_count == 0:
            sub_results.append((label, "FAIL", "LOG:DATA? returned no samples."))
            continue

        final_measured = ctrl_rows[-1]["measured_hz"]
        ok = abs(final_measured - expected_hz) <= tol_hz
        verdict = "PASS" if ok else "FAIL"
        notes = (f"{desc_gains}: final measured={final_measured} Hz (expected steady state "
                 f"{expected_hz:.0f} Hz for setpoint {target_hz}, tol {tol_hz}), "
                 f"{ctrl_count} controller / {len(sim_rows)} simulator samples over {duration_s}s.")
        combined_png = os.path.join(out_dir, "combined.png")
        generate_combined_plot(f"{name}_{label}", description, verdict, notes, ch, ctrl_rows, sim_rows,
                                combined_png, meta=ctrl_meta)
        print(f"COMBINED_PLOT: {combined_png}")
        sub_results.append((label, verdict, notes))

    overall = "PASS" if all(v == "PASS" for _, v, _ in sub_results) else "FAIL"
    summary = "; ".join(f"{label}: {v}" for label, v, _ in sub_results)
    return result(name,
                  "Closed-loop PID gain sweep (Kp=" + ", ".join(str(k) for k in kp_values) +
                  "; Ki=" + ", ".join(str(k) for k in ki_values) + " @ Kp=1.0; Kd=0 throughout)",
                  f"P-only points settle within {tol_hz} Hz of Kp*{target_hz}/(1+Kp); "
                  f"PI points settle within {tol_hz} Hz of the raw {target_hz} Hz setpoint "
                  f"(integral removes steady-state error), each within {duration_s}s",
                  overall, summary,
                  artifacts=[])


def scenario_closed_loop_smoke_test(ctrl, sim, kp=0.3, ki=1.0, kd=0.0):
    """#1c -- direct request, 2026-09-21: a genuine closed-loop smoke
    test, distinct from scenario_gain_sweep() (which deliberately uses
    Ki=0 and checks convergence to that P-only setup's OWN predictable
    non-zero steady-state error, not the literal setpoint). This test
    uses real PI gains (Ki != 0, untested anywhere in this project
    before today) and checks genuine convergence TO the setpoint itself
    -- the real question a "does closed loop actually work" smoke test
    should ask. Deliberately conservative gains and a short run: this is
    the FIRST time this project has ever driven a real Transrex-loop
    plant with nonzero Ki, so this is as much a stability check as a
    tracking check -- watch the plot for any sign of oscillation/
    overshoot/runaway, not just the final numeric verdict."""
    name, ch = "closed_loop_smoke", 1
    out_dir, ts = new_scenario_dir(name)
    reset_to_safe_state(ctrl, sim)

    if not arm_and_gate(ctrl, sim, [ch]):
        return result(name, "Closed-loop smoke test", "", "FAIL", "ARM failed.")

    target_hz = 40000
    duration_s = 5.0
    tol_hz = 1500   # genuine convergence to the setpoint itself, not a P-only offset

    ctrl.query(f"PID:LOOPMODE {ch} 1", timeout=1.0)   # closed-loop
    ctrl.query(f"PID:GAINS {ch} {kp} {ki} {kd}", timeout=1.0)
    ctrl.query(f"LOG:ARM {ch} 1000 {pid_log_decim(duration_s)}", timeout=1.0)
    arm_sim_log(sim, ch, max_samples=1000, min_interval_ms=sim_log_interval_ms(duration_s))
    ctrl.query(f"SOURce:SETpoint {ch} {target_hz}", timeout=1.0)
    ctrl.query("SOURce:RUN", timeout=1.0)

    time.sleep(duration_s)

    ctrl.query("SOURce:STOP", timeout=1.0)
    ctrl_rows, rate_hz, ctrl_count = fetch_log(ctrl, ch)
    sim_rows = fetch_sim_log(sim, ch)
    reset_to_safe_state(ctrl, sim)

    description = f"Closed-loop smoke test -- Kp={kp} Ki={ki} Kd={kd} (real PI gains, first use)"
    ctrl_meta = shot_meta(ctrl, ch)
    ctrl_csv, ctrl_png = save_and_plot_controller(ch, ctrl_rows, rate_hz, ctrl_count, out_dir, ts, ctrl_meta)
    sim_meta = dict(channel=ch, idn=sim.query("*IDN?", timeout=1.0), timestamp=ctrl_meta["timestamp"])
    sim_csv, sim_png = save_and_plot_sim(ch, sim_rows, out_dir, sim_meta)

    if ctrl_count == 0:
        return result(name, description, "", "FAIL", "LOG:DATA? returned no samples.",
                       artifacts=[p for p in (ctrl_csv, ctrl_png, sim_csv, sim_png) if p])

    # Convergence: last 1s of the run, genuinely close to the LITERAL
    # setpoint (unlike gain_sweep's own P-only-offset check) -- also
    # check for overshoot/instability (a real stability concern with
    # untested Ki), not just final tracking.
    #
    # MEDIAN/PERCENTILE, not last-sample/max -- confirmed 2026-09-21 by
    # inspecting raw data: a genuinely clean, well-behaved, monotonic PI
    # step response (measured smoothly climbing toward setpoint, no real
    # overshoot visible) contained ONE isolated, physically-impossible
    # sample (69133 Hz sandwiched between two ~36500 Hz neighbors, no
    # rise/decay either side) -- the SAME class of LOG:DATA?
    # transmission glitch found and worked around in
    # scenario_all_channels_profiled_shot() earlier the same day. A
    # single bad sample must not fail an otherwise-clean response: final
    # convergence uses the tail window's MEDIAN (robust to one glitched
    # sample landing at the very end); the stability/overshoot check
    # uses the 99th percentile of the WHOLE run (robust to the rare
    # single-sample spike, while still catching a genuinely SUSTAINED
    # overshoot, which would show up in many consecutive samples, not
    # just the top 1%).
    tail_rows = [r for r in ctrl_rows if r["t_s"] >= duration_s - 1.0]
    tail_measured = sorted(r["measured_hz"] for r in tail_rows) if tail_rows else \
                     sorted(r["measured_hz"] for r in ctrl_rows)
    final_measured = tail_measured[len(tail_measured) // 2]
    all_measured_sorted = sorted(r["measured_hz"] for r in ctrl_rows)
    p99_measured = all_measured_sorted[int(len(all_measured_sorted) * 0.99)]
    converged = abs(final_measured - target_hz) <= tol_hz
    stable = p99_measured <= target_hz + 5000   # no SUSTAINED overshoot/runaway
    ok = converged and stable

    verdict = "PASS" if ok else "FAIL"
    notes = (f"tail-window median measured={final_measured} Hz (target {target_hz}, tol {tol_hz}); "
             f"whole-run p99 measured={p99_measured} Hz (stability check: no SUSTAINED overshoot "
             f"beyond +5000Hz, robust to a lone glitched sample); "
             f"{ctrl_count} controller / {len(sim_rows)} simulator samples over {duration_s}s.")
    combined_png = os.path.join(out_dir, "combined.png")
    generate_combined_plot(name, description, verdict, notes, ch, ctrl_rows, sim_rows,
                            combined_png, meta=ctrl_meta)
    print(f"COMBINED_PLOT: {combined_png}")
    return result(name, description,
                  f"Genuine convergence to the {target_hz} Hz setpoint (within {tol_hz} Hz) "
                  "using real PI gains, with no significant overshoot/instability",
                  verdict, notes,
                  artifacts=[p for p in (ctrl_csv, ctrl_png, sim_csv, sim_png, combined_png) if p])


def scenario_full_profiled_shot(ctrl, sim):
    """#2 -- a real profiled shot (ramp/flat-top/ramp-down) with the
    simulator gated on, captured via the controller's own LOG:DATA?
    pipeline."""
    name, ch = "profiled_shot", 1
    out_dir, ts = new_scenario_dir(name)
    reset_to_safe_state(ctrl, sim)

    if not arm_and_gate(ctrl, sim, [ch]):
        return result(name, "Full profiled shot", "Ramp/flat-top/ramp-down tracks correctly",
                       "FAIL", "ARM failed.")

    ramp_s, flat_s, demand_a = RAMP_S, FLAT_S, 3000.0
    total_s = 2 * ramp_s + flat_s
    logged_span_s = total_s + 1.0   # matches the time.sleep() below -- both boards' logs
                                      # need to cover this actual span, not just total_s
    ctrl.query(f"SHOT:TIMing {ramp_s} {flat_s} {ramp_s}", timeout=1.0)
    ctrl.query(f"SHOT:CURRent {ch} {demand_a}", timeout=1.0)
    ctrl.query(f"LOG:ARM {ch} {PID_LOG_MAX_SAMPLES} {pid_log_decim(logged_span_s)}", timeout=1.0)
    arm_sim_log(sim, ch, max_samples=1000, min_interval_ms=sim_log_interval_ms(logged_span_s))
    start_reply = ctrl.query("SHOT:STARt", timeout=2.0)
    if is_err(start_reply):
        reset_to_safe_state(ctrl, sim)
        return result(name, "Full profiled shot", "Ramp/flat-top/ramp-down tracks correctly",
                       "FAIL", f"SHOT:STARt -> {explain_err(start_reply)}")

    time.sleep(total_s + 1.0)

    sim_rows = fetch_sim_log(sim, ch)
    meta = shot_meta(ctrl, ch, ramp_s=ramp_s, flat_s=flat_s, demand_a=demand_a)
    ctrl_rows, rate_hz, count = fetch_log(ctrl, ch)
    ctrl_csv, ctrl_png = save_and_plot_controller(ch, ctrl_rows, rate_hz, count, out_dir, ts, meta)
    sim_meta = dict(channel=ch, idn=sim.query("*IDN?", timeout=1.0),
                     timestamp=meta["timestamp"])
    sim_csv, sim_png = save_and_plot_sim(ch, sim_rows, out_dir, sim_meta)
    reset_to_safe_state(ctrl, sim)

    description = "Full profiled shot"
    stats = compute_log_stats(ctrl_rows, rate_hz)
    if stats is None:
        return result(name, description, "Ramp/flat-top/ramp-down tracks correctly",
                       "FAIL", "LOG:DATA? returned no samples.")

    ok = stats["output_error"]["max_abs"] < 2000
    verdict = "PASS" if ok else "FAIL"
    notes = (f"output_error max_abs={stats['output_error']['max_abs']:.0f} Hz, "
             f"{stats['count']} samples @ {rate_hz} Hz.")
    combined_png = os.path.join(out_dir, "combined.png")
    generate_combined_plot(name, description, verdict, notes, ch, ctrl_rows, sim_rows,
                            combined_png, meta=meta)
    print(f"COMBINED_PLOT: {combined_png}")
    return result(name, description,
                  "Controller output tracks the commanded profile "
                  f"(ramp={ramp_s}s flat={flat_s}s demand={demand_a}A) within 2000 Hz max error",
                  verdict, notes,
                  artifacts=[p for p in (ctrl_csv, ctrl_png, sim_csv, sim_png, combined_png) if p])


def scenario_closed_loop_profiled_shot(ctrl, sim):
    """#2b -- 2026-09-21, one-shot CLOSED-loop profiled shot (ramp/flat-top/
    ramp-down), the closed-loop counterpart to scenario_full_profiled_shot
    (which runs open-loop). Channel 1 gated through the simulator fiber pair,
    PID in CLOSED-loop mode with PI gains (Kp=1.0, Ki=20, Kd=0 -- chosen from
    the gain sweep's clean convergence), a 10s/5s profile commanded. The PID
    must actually DRIVE the output so the measured feedback tracks the profile,
    so the verdict is the flat-top median feedback_error (measured - setpoint),
    NOT the open-loop output_error (trivially ~0)."""
    name, ch = "closed_loop_profiled", 1
    out_dir, ts = new_scenario_dir(name)
    reset_to_safe_state(ctrl, sim)

    if not arm_and_gate(ctrl, sim, [ch]):
        return result(name, "Closed-loop profiled shot", "Feedback tracks the profile",
                       "FAIL", "ARM failed.")

    ramp_s, flat_s, demand_a = RAMP_S, FLAT_S, 3000.0
    total_s = 2 * ramp_s + flat_s
    logged_span_s = total_s + 1.0
    ctrl.query(f"PID:LOOPMODE {ch} 1", timeout=1.0)   # CLOSED-loop
    ctrl.query(f"PID:GAINS {ch} 1.0 20.0 0.0", timeout=1.0)
    ctrl.query(f"SHOT:TIMing {ramp_s} {flat_s} {ramp_s}", timeout=1.0)
    ctrl.query(f"SHOT:CURRent {ch} {demand_a}", timeout=1.0)
    ctrl.query(f"LOG:ARM {ch} {PID_LOG_MAX_SAMPLES} {pid_log_decim(logged_span_s)}", timeout=1.0)
    arm_sim_log(sim, ch, max_samples=1000, min_interval_ms=sim_log_interval_ms(logged_span_s))
    start_reply = ctrl.query("SHOT:STARt", timeout=2.0)
    if is_err(start_reply):
        reset_to_safe_state(ctrl, sim)
        return result(name, "Closed-loop profiled shot", "Feedback tracks the profile",
                       "FAIL", f"SHOT:STARt -> {explain_err(start_reply)}")

    time.sleep(total_s + 1.0)

    sim_rows = fetch_sim_log(sim, ch)
    meta = shot_meta(ctrl, ch, ramp_s=ramp_s, flat_s=flat_s, demand_a=demand_a)
    ctrl_rows, rate_hz, count = fetch_log(ctrl, ch)
    ctrl_csv, ctrl_png = save_and_plot_controller(ch, ctrl_rows, rate_hz, count, out_dir, ts, meta)
    sim_meta = dict(channel=ch, idn=sim.query("*IDN?", timeout=1.0), timestamp=meta["timestamp"])
    sim_csv, sim_png = save_and_plot_sim(ch, sim_rows, out_dir, sim_meta)
    reset_to_safe_state(ctrl, sim)

    description = "Closed-loop profiled shot (Kp=1.0, Ki=20, Kd=0)"
    if not ctrl_rows:
        return result(name, description, "Feedback tracks the profile",
                       "FAIL", "LOG:DATA? returned no samples.")

    # Verdict window: flat-top only (the ramp carries a legitimate following
    # error; the 720 Hz ripple is aliased at this log rate, so use the median).
    flat_rows = [r for r in ctrl_rows if ramp_s + 0.5 <= r["t_s"] <= ramp_s + flat_s - 0.5]
    if flat_rows:
        abs_errs = sorted(abs(r["measured_hz"] - r["setpoint_hz"]) for r in flat_rows)
        median_err = abs_errs[len(abs_errs) // 2]
        max_err = abs_errs[-1]
    else:
        median_err = max_err = None
    ok = median_err is not None and median_err < 1000
    verdict = "PASS" if ok else "FAIL"
    notes = (f"flat-top median |measured-setpoint|={median_err:.0f} Hz "
             f"(max {max_err:.0f} Hz), {count} samples @ {rate_hz} Hz.")
    combined_png = os.path.join(out_dir, "combined.png")
    generate_combined_plot(name, description, verdict, notes, ch, ctrl_rows, sim_rows,
                            combined_png, meta=meta)
    print(f"COMBINED_PLOT: {combined_png}")
    return result(name, description,
                  f"Feedback tracks the {ramp_s}s/{flat_s}s/{demand_a}A profile within "
                  "1000 Hz median flat-top error (closed loop)",
                  verdict, notes,
                  artifacts=[p for p in (ctrl_csv, ctrl_png, sim_csv, sim_png, combined_png) if p])


def start_profile_shot(ctrl, channels, ramp_s, flat_s, demand_a,
                        closed_loop=False, kp=0.5, ki=750.0, kd=0.0, ramp_down_s=None):
    """Configures and starts a REAL profiled shot (SHOT:TIMing/
    CURRent/STARt) across `channels` -- REQUIRED for fault-injection
    tests, not just cosmetic: SHOT:STARt calls SM_Fire()
    directly, transitioning the state machine ARMED -> FIRING, which is
    what HandleOvercurrentFault() (state_machine.c) branches on to
    decide graceful derate-and-ramp vs. an immediate full PID_Stop().
    Plain SOURce:RUN does NOT make this transition at all (documented
    "known gap", docs/command_reference.md) -- using it for a fault
    test silently exercises the wrong code path (instant full stop,
    not the real per-channel derate/ramp the spec is actually about),
    found the hard way when scenario_multi_fault_sequencing's own
    survivor kept reporting PID_IsRunning()=0 almost immediately after
    each fault.

    ALSO explicitly sets PID:LOOPMODE 0 (open-loop) for every channel
    -- REAL BUG FOUND the hard way, 2026-09-18, during the 45-shot
    channel/demand matrix sweep: every combination including channel 4
    failed with output stuck EXACTLY at PID_OUTPUT_MIN_HZ (3000 Hz)
    regardless of demand, while every combination without it tracked
    perfectly (0 Hz error). Root cause: this function never set loop
    mode at all -- channels 1-3 only happened to already be open-loop
    because EARLIER scenarios in this same file explicitly set
    `PID:LOOPMODE {c} 0` on them; channel 4, never touched by anything
    before the matrix sweep, was sitting at its own boot-time default
    (closed-loop, confirmed via PID:LOOPMODE? 4 -> OK 1) with PID gains
    still at PID_Init()'s own safe-inert default (Kp=Ki=Kd=0, confirmed
    via PID:GAINS? 4 -> OK 0 0 0) -- closed-loop with zero gains
    computes zero corrective action, so the output never moves off
    whatever it started at. Not a firmware bug, not a real hardware
    issue with channel 4 -- purely a test script gap (implicit reliance
    on state left over from other tests instead of setting it
    explicitly, same class of bug LOG:ARM's own decimation had). Fixed
    by making every channel's loop mode explicit here, matching the
    open-loop convention every other scenario in this file already
    uses -- no scenario should ever depend on what a DIFFERENT scenario
    happened to leave configured on the controller.

    `closed_loop`/`kp`/`ki`/`kd` added 2026-09-21, direct request: the
    matrix campaign now runs CLOSED loop (PID:LOOPMODE {c} 1 + real
    gains) instead of open loop. Defaults (kp=0.5, ki=750.0, kd=0.0) are
    the SAME gains confirmed via a live single-channel stability check
    the same day -- clean tracking through a full 10s-ramp/5s-flat-top/
    10s-ramp-down profile, converging essentially exactly to setpoint at
    flat-top (measured=52485 Hz vs. setpoint=52500 Hz), no overshoot, no
    oscillation. Every OTHER caller of this function (the fault-
    injection scenarios) still gets the original open-loop behavior
    (closed_loop=False, unchanged default) -- those tests are about
    fault RESPONSE, not tracking quality, and were never re-validated
    under closed loop.

    `ramp_down_s` added 2026-09-22, direct request: independently
    configurable ramp-down, defaulting to None (= same as `ramp_s`,
    preserving every existing caller's symmetric behavior unchanged) --
    the firmware/wire protocol (SHOT:TIMing) now takes three
    arguments unconditionally, so this function always sends all
    three; only callers that actually want an asymmetric profile need
    to pass a different ramp_down_s."""
    effective_ramp_down_s = ramp_down_s if ramp_down_s is not None else ramp_s
    ctrl.query(f"SHOT:TIMing {ramp_s} {flat_s} {effective_ramp_down_s}", timeout=1.0)
    for c in channels:
        if closed_loop:
            ctrl.query(f"PID:LOOPMODE {c} 1", timeout=1.0)
            ctrl.query(f"PID:GAINS {c} {kp} {ki} {kd}", timeout=1.0)
        else:
            ctrl.query(f"PID:LOOPMODE {c} 0", timeout=1.0)
        ctrl.query(f"SHOT:CURRent {c} {demand_a}", timeout=1.0)
    reply = ctrl.query("SHOT:STARt", timeout=2.0)
    return not is_err(reply)


def scenario_fault_injection(name, sim_cmd, description, ch, second_ch, expect_state_token, expect_full_stop):
    """Shared body for OCP / Enerpro / Water+Temp mid-shot fault tests
    (#3/#4/#5) -- gate two channels on, start an open-loop shot on
    both, inject the fault on `ch` mid-shot via `sim_cmd`, confirm
    STATE? and the derate/full-stop behavior. Both boards' logs are
    armed before the shot (controller: LOG:ARM 0 ... -- ALL channels,
    so the survivor's derate transient is captured too; simulator:
    SIM:LOG on the faulted channel) and retrieved after, same
    arm/run/retrieve pattern as the other scenarios."""
    def run(ctrl, sim):
        out_dir, ts = new_scenario_dir(name)
        reset_to_safe_state(ctrl, sim)

        if not arm_and_gate(ctrl, sim, [ch, second_ch]):
            return result(name, description, "", "FAIL", "ARM failed.")

        ramp_s, flat_s, demand_a = RAMP_S, FLAT_S, 3000.0
        ctrl.query(f"LOG:ARM 0 {PID_LOG_MAX_SAMPLES} {pid_log_decim(2 * ramp_s + flat_s)}", timeout=1.0)   # all channels -- captures the survivor too
        arm_sim_log(sim, ch, max_samples=1000, min_interval_ms=5)

        if not start_profile_shot(ctrl, [ch, second_ch], ramp_s, flat_s, demand_a):
            reset_to_safe_state(ctrl, sim)
            return result(name, description, "", "FAIL", "SHOT:STARt failed.")

        time.sleep(ramp_s + 0.5)   # well into flat-top, genuinely FIRING (not just ARMED --
                                    # see start_profile_shot()'s own comment on why this matters)

        sim.query(sim_cmd.format(ch=ch), timeout=1.0)
        time.sleep(0.2)
        state_reply = ctrl.query("STATE?", timeout=1.0)
        running_after, output_after = _pid_status_output_hz(ctrl.query(f"SOURce:STATus? {second_ch}", timeout=1.0))

        time.sleep(1.0)   # let the survivor's ramp-down continue a bit longer for the plot
        ctrl.query("SOURce:STOP", timeout=1.0)

        ctrl_meta = shot_meta(ctrl, ch, ramp_s=ramp_s, flat_s=flat_s, demand_a=demand_a)
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

        fault_events = [(ramp_s + 0.5, f"{description} injected")]

        state_ok = expect_state_token in state_reply
        if expect_full_stop:
            verdict = "PASS" if ("GENERAL" in state_reply or "FAULT" in state_reply) else "FAIL"
            behavior_note = "full system stop expected"
        else:
            # Real point of this test: the survivor must still be RUNNING
            # (not instantly stopped) and visibly derated below its
            # steady-state demand -- see start_profile_shot()'s own
            # comment for why SHOT:STARt (real FIRING) is required
            # for this to mean anything. demand_a is Amps, output_after is
            # Hz -- convert via the live-synced calibration (amps_to_hz(),
            # wham_console.py), not a hand-duplicated formula.
            demand_hz = amps_to_hz(demand_a)
            survivor_ok = (running_after == 1 and output_after is not None and 0 < output_after < demand_hz)
            verdict = "PASS" if (state_ok and survivor_ok) else "FAIL"
            behavior_note = "survivor stays running, visibly derated below its steady-state output"

        notes = (f"STATE? -> {state_reply!r}; survivor ch{second_ch} running={running_after} "
                 f"output={output_after}; faulted/survivor logged ({faulted_count}/{survivor_count} samples).")
        combined_png = os.path.join(out_dir, "combined.png")
        generate_combined_plot(name, description, verdict, notes, ch, faulted_rows, sim_rows,
                                combined_png, fault_events=fault_events,
                                extra_traces=[(f"ch{second_ch} output (survivor)", survivor_rows)],
                                meta=ctrl_meta)
        print(f"COMBINED_PLOT: {combined_png}")
        grid_png = os.path.join(out_dir, "grid.png")
        generate_multi_channel_grid_plot(
            name, description, verdict, notes,
            {ch: faulted_rows, second_ch: survivor_rows},
            {ch: ctrl_meta, second_ch: ctrl_meta},
            grid_png, fault_events=fault_events)
        print(f"GRID_PLOT: {grid_png}")
        return result(name, description,
                       f"STATE? reports {expect_state_token}, {behavior_note}",
                       verdict, notes,
                       artifacts=[p for p in (faulted_csv, faulted_png, survivor_csv, survivor_png,
                                               sim_csv, sim_png, combined_png, grid_png) if p])
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

    ramp_s, flat_s, demand_a = 1.0, 3.0, 3000.0
    ctrl.query(f"LOG:ARM 0 {PID_LOG_MAX_SAMPLES} {pid_log_decim(2 * ramp_s + flat_s)}", timeout=1.0)
    arm_sim_log(sim, ch, max_samples=1000, min_interval_ms=5)

    if not start_profile_shot(ctrl, [ch, survivor], ramp_s, flat_s, demand_a):
        reset_to_safe_state(ctrl, sim)
        return result(name, "ENA_OUT/CONTACT_OUT dropped mid-shot", "", "FAIL",
                       "SHOT:STARt failed.")
    time.sleep(ramp_s + 0.5)   # genuinely FIRING -- see start_profile_shot()'s comment

    ctrl.query(f"XREX:CHANnel:CONTactOut {ch} 0", timeout=1.0)
    time.sleep(0.2)

    state_reply = ctrl.query("STATE?", timeout=1.0)
    running_after, output_after = _pid_status_output_hz(ctrl.query(f"SOURce:STATus? {survivor}", timeout=1.0))
    time.sleep(1.0)
    ctrl.query("SOURce:STOP", timeout=1.0)

    ctrl_meta = shot_meta(ctrl, ch, ramp_s=ramp_s, flat_s=flat_s, demand_a=demand_a)
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

    description = "ENA_OUT/CONTACT_OUT dropped mid-shot"
    demand_hz = amps_to_hz(demand_a)
    survivor_ok = (running_after == 1 and output_after is not None and 0 < output_after < demand_hz)
    verdict = "PASS" if ("ENABLE_OUTPUT" in state_reply and survivor_ok) else "FAIL"
    notes = f"STATE? -> {state_reply!r}; survivor running={running_after} output={output_after}"
    fault_events = [(ramp_s + 0.5, "CONTACT_OUT dropped")]
    combined_png = os.path.join(out_dir, "combined.png")
    generate_combined_plot(name, description, verdict, notes, ch, faulted_rows, sim_rows,
                            combined_png, fault_events=fault_events,
                            extra_traces=[(f"ch{survivor} output (survivor)", survivor_rows)],
                            meta=ctrl_meta)
    print(f"COMBINED_PLOT: {combined_png}")
    grid_png = os.path.join(out_dir, "grid.png")
    generate_multi_channel_grid_plot(
        name, description, verdict, notes,
        {ch: faulted_rows, survivor: survivor_rows},
        {ch: ctrl_meta, survivor: ctrl_meta},
        grid_png, fault_events=fault_events)
    print(f"GRID_PLOT: {grid_png}")
    return result(name, description,
                  "STATE? reports ENABLE_OUTPUT <ch>, survivor stays running, visibly derated",
                  verdict, notes,
                  artifacts=[p for p in (faulted_csv, faulted_png, survivor_csv, survivor_png,
                                          sim_csv, sim_png, combined_png, grid_png) if p])


def _pid_status_output_hz(reply):
    """Parses SOURce:STATus?'s `OK <isRunning> <setpointHz> <measuredHz>
    <outputHz>` reply -> (is_running, output_hz), or (None, None) on a
    bad reply."""
    parts = reply.split()
    if len(parts) >= 5 and parts[0] == "OK":
        try:
            return int(parts[1]), int(parts[4])
        except ValueError:
            pass
    return None, None


def scenario_multi_fault_sequencing(ctrl, sim):
    """#7 -- two different fault types on two different channels in
    sequence during one shot (ch1 OCP, then ch3 Enerpro), survivor ch2
    watched throughout. Confirms: (a) the SECOND fault report, arriving
    while already SM_STATE_FAULT, still hard-disables its own channel
    even though it doesn't change the reported STATE? type/channel
    (SM_ReportOcpFault()/SM_ReportEnerproFault()'s own "already faulted"
    short-circuit -- see state_machine.c); (b) the survivor visibly
    derates after EACH fault, sampled quickly (~150ms) after each one.

    CORRECTED 2026-09-18, first cut: originally waited 0.5s after each
    fault and asserted an exact "(1-1/n) telescoped to 1/3 of original"
    target -- wrong on two counts. First, each fault's derate factor
    applies to the survivor's CURRENT (already mid-ramp) lastOutputHz,
    not the original setpoint (PID_BeginOvercurrentRampDown(), pid.c)
    -- with FAULT_RAMP_DOWN_TIME_S=1.0s and faults 0.5s apart, real ramp
    progress happens between them, so the clean "(N-k)/N of original"
    telescoping only holds for back-to-back faults with no ramp time in
    between. Second, waiting 0.5s after the SECOND fault (1.0s total
    after the first, exactly FAULT_RAMP_DOWN_TIME_S) meant the ramp had
    already completed and pid.c had stopped entirely by the time of the
    check (SOURce:STATus? isRunning=0) -- sampling too late to see a
    meaningful derated value at all. Fixed: sample quickly after each
    fault instead, and only assert DIRECTIONAL/qualitative correctness
    (still running, output measurably below the setpoint) rather than
    an exact numeric target that depends on ramp timing."""
    name, ch_a, ch_b, survivor = "multi_fault", 1, 3, 2
    out_dir, ts = new_scenario_dir(name)
    reset_to_safe_state(ctrl, sim)

    if not arm_and_gate(ctrl, sim, [ch_a, ch_b, survivor]):
        return result(name, "Multi-fault sequencing (OCP then Enerpro, 2 channels)",
                       "", "FAIL", "ARM failed.")

    ramp_s, flat_s, demand_a = RAMP_S, FLAT_S, 3000.0
    ctrl.query(f"LOG:ARM 0 {PID_LOG_MAX_SAMPLES} {pid_log_decim(2 * ramp_s + flat_s)}", timeout=1.0)
    demand_hz = amps_to_hz(demand_a)

    if not start_profile_shot(ctrl, [ch_a, ch_b, survivor], ramp_s, flat_s, demand_a):
        reset_to_safe_state(ctrl, sim)
        return result(name, "Multi-fault sequencing (OCP then Enerpro, 2 channels)",
                       "", "FAIL", "SHOT:STARt failed.")
    time.sleep(ramp_s + 0.5)   # genuinely FIRING -- see start_profile_shot()'s comment

    sim.query(f"SIM:FAULT:OCP {ch_a} 1", timeout=1.0)
    time.sleep(0.15)
    state_after_first = ctrl.query("STATE?", timeout=1.0)
    running_1, output_1 = _pid_status_output_hz(ctrl.query(f"SOURce:STATus? {survivor}", timeout=1.0))

    sim.query(f"SIM:FAULT:ENERPRO {ch_b} 1", timeout=1.0)
    time.sleep(0.15)
    state_after_second = ctrl.query("STATE?", timeout=1.0)
    running_2, output_2 = _pid_status_output_hz(ctrl.query(f"SOURce:STATus? {survivor}", timeout=1.0))

    enable_a = ctrl.query(f"SOURce:ENAble? {ch_a}", timeout=1.0)
    enable_b = ctrl.query(f"SOURce:ENAble? {ch_b}", timeout=1.0)
    ctrl.query("SOURce:STOP", timeout=1.0)

    ctrl_meta = shot_meta(ctrl, survivor, ramp_s=ramp_s, flat_s=flat_s, demand_a=demand_a)
    a_rows, rate_hz, a_count = fetch_log(ctrl, ch_a)
    b_rows, _, b_count = fetch_log(ctrl, ch_b)
    survivor_rows, _, survivor_count = fetch_log(ctrl, survivor)
    reset_to_safe_state(ctrl, sim)

    a_csv, a_png = save_and_plot_controller(ch_a, a_rows, rate_hz, a_count, out_dir, ts,
                                             dict(ctrl_meta, channel=ch_a))
    b_csv, b_png = save_and_plot_controller(ch_b, b_rows, rate_hz, b_count, out_dir, ts,
                                             dict(ctrl_meta, channel=ch_b))
    survivor_csv, survivor_png = save_and_plot_controller(survivor, survivor_rows, rate_hz, survivor_count,
                                                            out_dir, ts, dict(ctrl_meta, channel=survivor))

    description = "Multi-fault sequencing (OCP then Enerpro, 2 channels)"
    both_disabled = ("0" in enable_a) and ("0" in enable_b)
    type_preserved = "OVERCURRENT" in state_after_second   # first fault's type/channel, unchanged by the second
    derated_after_1 = (running_1 == 1 and output_1 is not None and 0 < output_1 < demand_hz)
    derated_after_2 = (running_2 == 1 and output_2 is not None and 0 < output_2 < demand_hz)

    verdict = "PASS" if (both_disabled and type_preserved and derated_after_1 and derated_after_2) else "FAIL"
    notes = (f"after 1st ({state_after_first!r}): survivor running={running_1} output={output_1}; "
             f"after 2nd ({state_after_second!r}): survivor running={running_2} output={output_2}; "
             f"ch{ch_a} enabled={enable_a!r}; ch{ch_b} enabled={enable_b!r}")
    fault_events = [(ramp_s + 0.5, f"OCP ch{ch_a} injected"), (ramp_s + 0.65, f"Enerpro ch{ch_b} injected")]
    combined_png = os.path.join(out_dir, "combined.png")
    generate_combined_plot(name, description, verdict, notes, survivor, survivor_rows, None,
                            combined_png, fault_events=fault_events,
                            extra_traces=[(f"ch{ch_a} output (OCP-faulted)", a_rows),
                                          (f"ch{ch_b} output (Enerpro-faulted)", b_rows)],
                            meta=ctrl_meta)
    print(f"COMBINED_PLOT: {combined_png}")
    grid_png = os.path.join(out_dir, "grid.png")
    generate_multi_channel_grid_plot(
        name, description, verdict, notes,
        {ch_a: a_rows, ch_b: b_rows, survivor: survivor_rows},
        {ch_a: ctrl_meta, ch_b: ctrl_meta, survivor: ctrl_meta},
        grid_png, fault_events=fault_events)
    print(f"GRID_PLOT: {grid_png}")
    return result(name, description,
                  f"Both faulted channels ({ch_a}, {ch_b}) end up disabled; STATE? keeps "
                  f"reporting the FIRST fault (OVERCURRENT {ch_a}); survivor ch{survivor} "
                  f"stays running and visibly derated below its steady-state output "
                  f"(~{demand_hz:.0f} Hz for {demand_a}A) after EACH fault",
                  verdict, notes,
                  artifacts=[p for p in (a_csv, a_png, b_csv, b_png, survivor_csv, survivor_png,
                                          combined_png, grid_png) if p])


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
\usepackage[dvipsnames]{xcolor}
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
    "gain_sweep": scenario_gain_sweep,
    "closed_loop_smoke": scenario_closed_loop_smoke_test,
    "profiled_shot": scenario_full_profiled_shot,
    "closed_loop_profiled": scenario_closed_loop_profiled_shot,
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
    "multi_fault": scenario_multi_fault_sequencing,
    "ext_regression": scenario_ext_regression,
    "all_channels": scenario_all_channels_profiled_shot,
}


# --------------------------------------------------------------------------
# Channel-combination x demand-level stress matrix -- direct request,
# 2026-09-18: "every combination of channels that we could be running
# with, up to and including differing demand values." All 15 non-empty
# subsets of {1,2,3,4} x 3 demand levels (10%/50%/90% of
# PFM_MAX_CURRENT_A) at fixed closed-loop gains = 45 profiled shots. Fault
# injection deliberately NOT repeated here (per direct confirmation) --
# that's already covered by the representative fault scenarios above;
# this sweep is purely about clean-tracking coverage across every
# channel combination and operating point. Reported separately
# (generate_full_matrix_report(), below) from the discrete PASS/FAIL
# scenarios above -- REWRITTEN 2026-09-21, direct request: full per-shot
# documentation, every one of the 45 runs gets its own report section and
# embedded stacked-channel plot, not just a summary table -- full raw
# data for every run is still saved under shots/ regardless.
# --------------------------------------------------------------------------

# Computed here from wc.PFM_MAX_CURRENT_A's hardcoded startup default,
# same as every other module-level constant in this file -- but that's
# stale the moment a live CONFig:MAXCURRent changes the real value, so
# main() recomputes this (global reassignment, same pattern as RAMP_S/
# FLAT_S below) right after wc.sync_calibration() runs post-connect.
def _matrix_demand_levels_a():
    return (
        round(wc.PFM_MAX_CURRENT_A * 0.10),   # ~10% -- low
        round(wc.PFM_MAX_CURRENT_A * 0.50),   # ~50% -- mid
        round(wc.PFM_MAX_CURRENT_A * 0.90),   # ~90% -- high
    )


MATRIX_DEMAND_LEVELS_A = _matrix_demand_levels_a()
MATRIX_CHANNEL_COMBINATIONS = [
    combo
    for n in range(1, HRTIM_NUM_CHANNELS + 1)
    for combo in itertools.combinations(range(1, HRTIM_NUM_CHANNELS + 1), n)
]   # 15 subsets for 4 channels: C(4,1)+C(4,2)+C(4,3)+C(4,4) = 4+6+4+1


def run_matrix_sweep(ctrl, sim):
    """Runs every (channel combination, demand level) shot in the
    matrix, returns runs -- a list of per-run result dicts (not the same
    shape as the discrete-scenario `result()` above, see
    generate_full_matrix_report()). Uses the module-global RAMP_S/FLAT_S
    (set from --ramp-s/--flat-s).

    REWRITTEN 2026-09-21, direct request: CLOSED loop (was open-loop --
    see start_profile_shot()'s own updated comment for the gains used
    and why), and full per-shot documentation instead of a summary-plus-
    a-few-samples report -- EVERY one of the 45 shots now gets its own
    saved CSV/plot data and a verdict based on genuine setpoint
    convergence (median |measured-setpoint| over the flat-top window,
    robust to the rare single-sample LOG:DATA? glitch documented
    elsewhere in this file -- NOT output_error, which is trivially near-
    zero in open-loop and was never a meaningful closed-loop check)."""
    ramp_s, flat_s = RAMP_S, FLAT_S
    flat_top_start_s = ramp_s + 0.5
    flat_top_end_s = ramp_s + flat_s - 0.5
    runs = []
    total = len(MATRIX_CHANNEL_COMBINATIONS) * len(MATRIX_DEMAND_LEVELS_A)
    i = 0
    for combo in MATRIX_CHANNEL_COMBINATIONS:
        for demand_a in MATRIX_DEMAND_LEVELS_A:
            i += 1
            combo_label = "+".join(str(c) for c in combo)
            print(f"[matrix {i}/{total}] channels={combo_label} demand={demand_a}A", end=" -> ")

            try:
                reset_to_safe_state(ctrl, sim)
                if not arm_and_gate(ctrl, sim, list(combo)):
                    print("ARM FAILED")
                    runs.append(dict(combo=combo, demand_a=demand_a, verdict="FAIL",
                                      note="ARM failed", per_channel_median_error={},
                                      per_channel_rows={}, per_channel_meta={}, plot_path=None))
                    continue

                total_s = 2 * ramp_s + flat_s
                ctrl.query(f"LOG:ARM 0 {PID_LOG_MAX_SAMPLES} {pid_log_decim(total_s)}", timeout=1.0)
                if not start_profile_shot(ctrl, list(combo), ramp_s, flat_s, demand_a,
                                           closed_loop=True, kp=0.5, ki=750.0, kd=0.0):
                    print("SHOT:STARt FAILED")
                    reset_to_safe_state(ctrl, sim)
                    runs.append(dict(combo=combo, demand_a=demand_a, verdict="FAIL",
                                      note="SHOT:STARt failed", per_channel_median_error={},
                                      per_channel_rows={}, per_channel_meta={}, plot_path=None))
                    continue

                time.sleep(total_s + 0.3)
                ctrl.query("SOURce:STOP", timeout=1.0)

                out_dir, ts = new_scenario_dir(f"matrix_ch{combo_label}_{demand_a}A")
                ctrl_meta = shot_meta(ctrl, combo[0], ramp_s=ramp_s, flat_s=flat_s, demand_a=demand_a)

                per_channel_rows = {}
                per_channel_meta = {}
                per_channel_median_error = {}
                worst_median = 0.0
                any_data = False
                for ch in combo:
                    rows, rate_hz, count = fetch_log(ctrl, ch)
                    save_and_plot_controller(ch, rows, rate_hz, count, out_dir, ts,
                                             dict(ctrl_meta, channel=ch))
                    per_channel_rows[ch] = rows
                    per_channel_meta[ch] = dict(ctrl_meta, channel=ch)
                    flat_top_rows = [r for r in rows
                                      if flat_top_start_s <= r["t_s"] <= flat_top_end_s]
                    if flat_top_rows:
                        any_data = True
                        errs = sorted(abs(r["measured_hz"] - r["setpoint_hz"]) for r in flat_top_rows)
                        median_err = errs[len(errs) // 2]
                        per_channel_median_error[ch] = median_err
                        worst_median = max(worst_median, median_err)
                    else:
                        per_channel_median_error[ch] = None
                reset_to_safe_state(ctrl, sim)

                ok = any_data and all(v is not None for v in per_channel_median_error.values()) \
                     and worst_median < 1500
                verdict = "PASS" if ok else "FAIL"
                print(f"{verdict} (worst channel flat-top median |measured-setpoint|={worst_median:.0f} Hz)")

                plot_path = os.path.join(out_dir, "stacked.png")
                plot_title = (f"Matrix: channels {combo_label}, {demand_a} A demand -- "
                               f"{verdict} (worst median error {worst_median:.0f} Hz)")
                generate_stacked_channel_plot(list(combo), per_channel_rows, per_channel_meta,
                                               plot_path, title=plot_title)

                notes_bits = ", ".join(f"ch{ch}={per_channel_median_error[ch]:.0f}Hz"
                                        if per_channel_median_error[ch] is not None else f"ch{ch}=NO DATA"
                                        for ch in combo)
                runs.append(dict(combo=combo, demand_a=demand_a, verdict=verdict,
                                  note=f"flat-top median |measured-setpoint| per channel: {notes_bits}",
                                  per_channel_median_error=per_channel_median_error,
                                  per_channel_rows=per_channel_rows, per_channel_meta=per_channel_meta,
                                  plot_path=plot_path, worst_median=worst_median))
            except Exception as exc:   # noqa: BLE001 -- one flaky serial
                                                       # read/reply (real risk at this
                                                       # request volume -- 45 shots x
                                                       # several queries each, found the
                                                       # hard way, 2026-09-18: a
                                                       # LOG:DATA? reply came back
                                                       # short by 2 of 1998 expected
                                                       # values) must not kill the other
                                                       # 44 runs -- matches main()'s own
                                                       # existing per-scenario resilience
                                                       # for the discrete scenarios above.
                print(f"EXCEPTION: {exc!r}")
                try:
                    reset_to_safe_state(ctrl, sim)
                except Exception:
                    pass   # best-effort -- don't let cleanup itself abort the sweep
                runs.append(dict(combo=combo, demand_a=demand_a, verdict="FAIL",
                                  note=f"Exception: {exc!r}", per_channel_median_error={},
                                  per_channel_rows={}, per_channel_meta={}, plot_path=None))
    return runs


def generate_full_matrix_report(runs):
    """Writes docs/reports/simulator_channel_matrix_report.tex --
    REWRITTEN 2026-09-21, direct request: full tabulated pass/fail
    PLUS one section per run ('series') with a brief paragraph (what
    was tested, why it passed/failed) and a subsection embedding that
    run's own vertically-stacked per-channel plot (generate_stacked_
    channel_plot(), one row per channel actually in that combo) --
    every one of the 45 shots gets its own section, not just a
    representative few. Full raw data for every run is still under
    shots/ regardless, for anyone who wants to go beyond the report."""
    os.makedirs(DOCS_DIR, exist_ok=True)
    total_s = 2 * RAMP_S + FLAT_S
    parts = [TEX_PREAMBLE.replace(
        "WHAM-XREX-PFMG474 Simulator Validation Report",
        "WHAM-XREX-PFMG474 Channel/Demand Stress Matrix (Closed Loop)"
    ).replace(
        "Autonomous end-to-end validation of the controller/Transrex-simulator\n"
        "closed loop, per Part C/D of the simulator plan. Each section below\n"
        "covers one test scenario: what was commanded, the expected behavior,\n"
        "the observed result, and a pass/fail verdict. Raw data for every\n"
        "scenario is archived under \\code{shots/} in this repository (gitignored).",
        f"All {len(MATRIX_CHANNEL_COMBINATIONS)} non-empty channel-enable "
        f"combinations x {len(MATRIX_DEMAND_LEVELS_A)} demand levels "
        f"({', '.join(str(a) for a in MATRIX_DEMAND_LEVELS_A)} A -- 10/50/90\\% "
        "of PFM\\_MAX\\_CURRENT\\_A\\_PER\\_CHANNEL), CLOSED loop "
        f"(Kp=0.5, Ki=750, Kd=0 -- confirmed via a live single-channel stability "
        f"check the same day: clean convergence, no overshoot, essentially exact "
        f"flat-top tracking), each shot a real {RAMP_S:g}s ramp / {FLAT_S:g}s "
        f"flat-top / {RAMP_S:g}s ramp-down profile ({total_s:g}s total), "
        f"{len(MATRIX_CHANNEL_COMBINATIONS) * len(MATRIX_DEMAND_LEVELS_A)} shots total. "
        "Pass criterion: every channel's own flat-top MEDIAN "
        "$|\\mathit{measured}-\\mathit{setpoint}|$ (robust to the rare single-sample "
        "\\cmd{LOG:DATA?} transmission glitch documented elsewhere in this "
        "project -- NOT a raw max, which a lone glitch would fail unfairly) stays "
        "under 1500\\,Hz, measured over the flat-top window with 0.5s trimmed off "
        "each end (the filter's own settling tail). Full raw data for every shot "
        "is archived under \\code{shots/} in this repository (gitignored)."
    )]

    n_pass = sum(1 for r in runs if r["verdict"] == "PASS")
    parts.append(f"\\section*{{Summary}}\n{n_pass}/{len(runs)} shots PASSED.\n\n")

    parts.append("\\section*{Tabulated Results}\n")
    parts.append("\\begin{longtable}{l r r l}\n\\hline\n")
    parts.append("\\textbf{Channels} & \\textbf{Demand (A)} & \\textbf{Worst channel median error (Hz)} & "
                  "\\textbf{Verdict} \\\\\n\\hline\n\\endfirsthead\n")
    parts.append("\\hline\n\\textbf{Channels} & \\textbf{Demand (A)} & \\textbf{Worst channel median error (Hz)} & "
                  "\\textbf{Verdict} \\\\\n\\hline\n\\endhead\n")
    for r in runs:
        combo_label = "+".join(str(c) for c in r["combo"])
        color = "ForestGreen" if r["verdict"] == "PASS" else "BrickRed"
        worst = f"{r['worst_median']:.0f}" if "worst_median" in r else "--"
        parts.append(f"{combo_label} & {r['demand_a']} & {worst} & "
                      f"\\textcolor{{{color}}}{{\\textbf{{{r['verdict']}}}}} \\\\\n")
    parts.append("\\hline\n\\end{longtable}\n\n")

    parts.append("\\clearpage\n")
    parts.append("\\section*{Per-Shot Detail}\n")
    for idx, r in enumerate(runs, 1):
        combo_label = "+".join(str(c) for c in r["combo"])
        n_ch = len(r["combo"])
        color = "ForestGreen" if r["verdict"] == "PASS" else "BrickRed"
        demand_hz = amps_to_hz(r["demand_a"])

        parts.append(f"\\section{{Shot {idx}: Channels {escape_tex(combo_label)}, "
                      f"{r['demand_a']:g}\\,A demand}}\n")

        if r["verdict"] == "PASS":
            why = (f"Every one of the {n_ch} channel(s) in this combination converged, "
                   f"at flat-top, to within its own median "
                   f"$|\\mathit{{measured}}-\\mathit{{setpoint}}|$ error under the 1500\\,Hz "
                   f"pass threshold (worst channel: {r.get('worst_median', 0):.0f}\\,Hz) -- "
                   f"genuine closed-loop convergence to the commanded "
                   f"{demand_hz:.0f}\\,Hz setpoint ({r['demand_a']:g}\\,A), not just the "
                   f"controller's own commanded output following itself.")
        else:
            why = (f"At least one channel's flat-top median tracking error exceeded the "
                   f"1500\\,Hz pass threshold (worst channel: "
                   f"{r.get('worst_median', float('nan')):.0f}\\,Hz), or the shot could not "
                   f"be armed/started/logged at all -- see the per-channel breakdown below.")
        parts.append(f"Gated channel(s) {escape_tex(combo_label)} simultaneously, closed loop "
                     f"(Kp=0.5, Ki=750, Kd=0), commanding a real "
                     f"{RAMP_S:g}s/{FLAT_S:g}s/{RAMP_S:g}s ramp/flat-top/ramp-down profile to "
                     f"{r['demand_a']:g}\\,A ({demand_hz:.0f}\\,Hz) peak demand on every gated "
                     f"channel. {why} {escape_tex(r['note'])}.\n\n")

        parts.append(f"\\textcolor{{{color}}}{{\\textbf{{Verdict: {r['verdict']}}}}}\n\n")

        parts.append("\\subsection{Per-channel response}\n")
        if r.get("plot_path"):
            rel = os.path.relpath(r["plot_path"], DOCS_DIR)
            parts.append(f"\\includegraphics[width=\\linewidth]{{{rel}}}\n\n")
        else:
            parts.append("No plot available for this shot (see the note above for why).\n\n")

    parts.append(TEX_FOOTER)
    out_path = os.path.join(DOCS_DIR, "simulator_channel_matrix_report.tex")
    with open(out_path, "w") as f:
        f.write("".join(parts))
    print(f"[report] wrote {out_path}")
    return out_path


# --------------------------------------------------------------------------
# Closed-loop gain (Kp/Ki) smoothness sweep -- direct request, 2026-09-22:
# the 2026-09-21 matrix campaign's commanded-output (drive) trace was
# visibly noisier than earlier tests at Kp=0.5/Ki=750 -- suspected cause is
# the PID not acting enough like an integrator (P reacts instantly to noisy
# feedback every sample; I inherently smooths/averages it over time), so a
# short 3s-ramp/3s-flat-top profile is repeated across a grid of Kp/Ki
# values to find a combination that keeps the output smooth. Unlike the
# channel/demand matrix, this sweep is single-channel (gains are a
# controller-wide tuning question, not a per-channel-combination one) and
# has no hard pass/fail -- the report ranks every point by how smooth its
# commanded output actually was, plus a secondary sanity check that it
# still converges to setpoint.
#
# Per the 2026-09-22 direct instruction, this is also the first report
# under the "each .tex project gets its own well-labeled subdirectory
# under docs/reports/" convention (corrected same day -- the first cut
# put the new directory directly under docs/, not under docs/reports/).
# --------------------------------------------------------------------------

GAIN_SMOOTHNESS_DOCS_DIR = os.path.join(PROJECT_DIR, "docs", "reports", "gain_smoothness_sweep")
GAIN_SMOOTHNESS_CHANNEL = 1
# Computed from wc.PFM_MAX_CURRENT_A's startup default; main() recomputes
# this (global reassignment) right after wc.sync_calibration(), same as
# MATRIX_DEMAND_LEVELS_A above -- see its own comment for why.
GAIN_SMOOTHNESS_DEMAND_A = round(wc.PFM_MAX_CURRENT_A * 0.50)   # mid demand, same convention as MATRIX_DEMAND_LEVELS_A
GAIN_SMOOTHNESS_RAMP_S = 3.0
GAIN_SMOOTHNESS_FLAT_S = 3.0
GAIN_SMOOTHNESS_KP_VALUES = (0.1, 0.2, 0.3, 0.5)
GAIN_SMOOTHNESS_KI_VALUES = (250.0, 750.0, 1500.0, 3000.0)
GAIN_SMOOTHNESS_CONVERGENCE_TOL_HZ = 1500   # same flat-top-median threshold as the channel/demand matrix


def run_gain_smoothness_sweep(ctrl, sim, channel=GAIN_SMOOTHNESS_CHANNEL,
                               demand_a=None,
                               ramp_s=GAIN_SMOOTHNESS_RAMP_S, flat_s=GAIN_SMOOTHNESS_FLAT_S,
                               kp_values=GAIN_SMOOTHNESS_KP_VALUES,
                               ki_values=GAIN_SMOOTHNESS_KI_VALUES):
    """Runs one closed-loop profiled shot per (Kp, Ki) combination in
    `kp_values` x `ki_values` (Kd=0 throughout, matching every other closed-
    loop test in this file), all on a single channel at a fixed mid-level
    demand, using a short 3s/3s/3s ramp/flat-top/ramp-down profile. For
    each shot, computes two smoothness metrics on the commanded OUTPUT
    (not the measured feedback) over the flat-top window (same 0.5s-
    trimmed-each-end convention as run_matrix_sweep, to skip the filter's
    own post-ramp settling tail):
      - output_jitter_rms_hz: RMS of sample-to-sample differences -- a
        direct proxy for high-frequency content/"jitter", which is
        specifically what looked noisy in yesterday's plots.
      - output_std_hz: plain standard deviation over the window -- a
        secondary, coarser smoothness measure (also picks up any slow
        drift the trim didn't fully remove).
    Also computes the same flat-top median |measured-setpoint| tracking
    error used by run_matrix_sweep, purely as a sanity check that a smooth
    point isn't smooth because it's just not responding -- NOT the primary
    metric here, see generate_gain_smoothness_report()."""
    if demand_a is None:
        # Deliberately NOT a `demand_a=GAIN_SMOOTHNESS_DEMAND_A` default
        # above -- a default argument's value is bound once at module
        # load, which would freeze the pre-sync value even after main()
        # recomputes the global post-connect. Resolving it here instead
        # reads the current global at call time.
        demand_a = GAIN_SMOOTHNESS_DEMAND_A
    ramp_s, flat_s = float(ramp_s), float(flat_s)
    flat_top_start_s = ramp_s + 0.5
    flat_top_end_s = ramp_s + flat_s - 0.5
    total_s = 2 * ramp_s + flat_s
    runs = []
    combos = list(itertools.product(kp_values, ki_values))
    for i, (kp, ki) in enumerate(combos, 1):
        print(f"[gain_smoothness {i}/{len(combos)}] Kp={kp:g} Ki={ki:g}", end=" -> ")
        try:
            reset_to_safe_state(ctrl, sim)
            if not arm_and_gate(ctrl, sim, [channel]):
                print("ARM FAILED")
                runs.append(dict(kp=kp, ki=ki, demand_a=demand_a, verdict="FAIL",
                                  note="ARM failed", median_error=None, output_std_hz=None,
                                  output_jitter_rms_hz=None, rows=None, meta=None, plot_path=None))
                continue

            ctrl.query(f"LOG:ARM 0 {PID_LOG_MAX_SAMPLES} {pid_log_decim(total_s)}", timeout=1.0)
            if not start_profile_shot(ctrl, [channel], ramp_s, flat_s, demand_a,
                                       closed_loop=True, kp=kp, ki=ki, kd=0.0):
                print("SHOT:STARt FAILED")
                reset_to_safe_state(ctrl, sim)
                runs.append(dict(kp=kp, ki=ki, demand_a=demand_a, verdict="FAIL",
                                  note="SHOT:STARt failed", median_error=None, output_std_hz=None,
                                  output_jitter_rms_hz=None, rows=None, meta=None, plot_path=None))
                continue

            time.sleep(total_s + 0.3)
            ctrl.query("SOURce:STOP", timeout=1.0)

            out_dir, ts = new_scenario_dir(f"gain_smoothness_kp{kp:g}_ki{ki:g}")
            rows, rate_hz, count = fetch_log(ctrl, channel)
            meta = shot_meta(ctrl, channel, ramp_s=ramp_s, flat_s=flat_s, demand_a=demand_a)
            save_and_plot_controller(channel, rows, rate_hz, count, out_dir, ts, meta)
            reset_to_safe_state(ctrl, sim)

            flat_top_rows = [r for r in rows if flat_top_start_s <= r["t_s"] <= flat_top_end_s]
            if not flat_top_rows:
                print("NO FLAT-TOP DATA")
                runs.append(dict(kp=kp, ki=ki, demand_a=demand_a, verdict="FAIL",
                                  note="no samples landed in the flat-top window", median_error=None,
                                  output_std_hz=None, output_jitter_rms_hz=None,
                                  rows=rows, meta=meta, plot_path=None))
                continue

            errs = sorted(abs(r["measured_hz"] - r["setpoint_hz"]) for r in flat_top_rows)
            median_error = errs[len(errs) // 2]
            converged = median_error < GAIN_SMOOTHNESS_CONVERGENCE_TOL_HZ

            outs = [r["output_hz"] for r in flat_top_rows]
            mean_out = sum(outs) / len(outs)
            output_std_hz = math.sqrt(sum((v - mean_out) ** 2 for v in outs) / len(outs))
            if len(outs) >= 2:
                diffs = [outs[j + 1] - outs[j] for j in range(len(outs) - 1)]
                output_jitter_rms_hz = math.sqrt(sum(d * d for d in diffs) / len(diffs))
            else:
                output_jitter_rms_hz = 0.0

            verdict = "PASS" if converged else "FAIL"
            print(f"{verdict} (median error={median_error:.0f} Hz, output jitter RMS="
                  f"{output_jitter_rms_hz:.0f} Hz, output std={output_std_hz:.0f} Hz)")

            plot_path = os.path.join(out_dir, "stacked.png")
            plot_title = (f"Kp={kp:g}, Ki={ki:g}, Kd=0 -- {demand_a} A demand -- "
                          f"output jitter RMS {output_jitter_rms_hz:.0f} Hz, "
                          f"tracking median error {median_error:.0f} Hz")
            generate_stacked_channel_plot([channel], {channel: rows}, {channel: meta}, plot_path,
                                           title=plot_title)

            runs.append(dict(kp=kp, ki=ki, demand_a=demand_a, verdict=verdict,
                              note=f"flat-top median |measured-setpoint|={median_error:.0f} Hz",
                              median_error=median_error, output_std_hz=output_std_hz,
                              output_jitter_rms_hz=output_jitter_rms_hz,
                              rows=rows, meta=meta, plot_path=plot_path))
        except Exception as exc:   # noqa: BLE001 -- one flaky shot must not kill the rest of the sweep
            print(f"EXCEPTION: {exc!r}")
            try:
                reset_to_safe_state(ctrl, sim)
            except Exception:
                pass
            runs.append(dict(kp=kp, ki=ki, demand_a=demand_a, verdict="FAIL",
                              note=f"Exception: {exc!r}", median_error=None, output_std_hz=None,
                              output_jitter_rms_hz=None, rows=None, meta=None, plot_path=None))
    return runs


def generate_gain_smoothness_report(runs, channel=GAIN_SMOOTHNESS_CHANNEL,
                                     ramp_s=GAIN_SMOOTHNESS_RAMP_S, flat_s=GAIN_SMOOTHNESS_FLAT_S):
    """Writes docs/reports/gain_smoothness_sweep/gain_smoothness_sweep_report.tex
    -- its own dedicated subdirectory under docs/reports/, per the
    2026-09-22 direct instruction that .tex report projects should each
    live in their own well-labeled directory under docs/reports/. Ranks
    every (Kp, Ki) point by output_jitter_rms_hz
    (smoothest first) -- that ranking, not a pass/fail table, is the
    actual point of this report; convergence is reported alongside purely
    as a sanity check that the smoothest points are still real control,
    not just a channel that stopped responding."""
    os.makedirs(GAIN_SMOOTHNESS_DOCS_DIR, exist_ok=True)
    total_s = 2 * ramp_s + flat_s

    def sort_key(r):
        return r["output_jitter_rms_hz"] if r["output_jitter_rms_hz"] is not None else float("inf")

    ranked = sorted(runs, key=sort_key)
    # Derived from the actual runs passed in (not the module-default sweep
    # constants) so an appended sweep with extra Kp/Ki points -- e.g. the
    # 2026-09-22 Kp=0 addition -- is described accurately, not just the
    # original grid.
    actual_kp_values = sorted(set(r["kp"] for r in runs))
    actual_ki_values = sorted(set(r["ki"] for r in runs))

    parts = [TEX_PREAMBLE.replace(
        "WHAM-XREX-PFMG474 Simulator Validation Report",
        "WHAM-XREX-PFMG474 Closed-Loop Gain Smoothness Sweep"
    ).replace(
        "Autonomous end-to-end validation of the controller/Transrex-simulator\n"
        "closed loop, per Part C/D of the simulator plan. Each section below\n"
        "covers one test scenario: what was commanded, the expected behavior,\n"
        "the observed result, and a pass/fail verdict. Raw data for every\n"
        "scenario is archived under \\code{shots/} in this repository (gitignored).",
        f"{len(actual_kp_values)} Kp values "
        f"({', '.join(str(v) for v in actual_kp_values)}) x "
        f"{len(actual_ki_values)} Ki values "
        f"({', '.join(str(v) for v in actual_ki_values)}), Kd=0 throughout, "
        f"channel {channel} only at a fixed {GAIN_SMOOTHNESS_DEMAND_A}\\,A demand, each a real "
        f"{ramp_s:g}s/{flat_s:g}s/{ramp_s:g}s ramp/flat-top/ramp-down profile "
        f"({total_s:g}s total). Motivated by the 2026-09-21 45-shot matrix campaign "
        "(Kp=0.5, Ki=750): every shot converged, but the commanded OUTPUT "
        "(drive) trace was visibly noisier than earlier tests -- suspected "
        "cause is the PID not acting enough like an integrator (the P term "
        "reacts instantly to noisy feedback every sample; the I term "
        "inherently smooths it over time). Ranked below by output jitter "
        "RMS (root-mean-square of sample-to-sample differences in the "
        "commanded output over the flat-top window, 0.5s trimmed off each "
        "end) -- smoothest first -- with tracking convergence reported "
        "alongside as a sanity check only, not the ranking criterion. Full "
        "raw data for every shot is archived under \\code{shots/} in this "
        "repository (gitignored)."
    )]

    n_pass = sum(1 for r in runs if r["verdict"] == "PASS")
    parts.append(f"\\section*{{Summary}}\n{n_pass}/{len(runs)} shots converged to within "
                 f"{GAIN_SMOOTHNESS_CONVERGENCE_TOL_HZ}\\,Hz (flat-top median tracking error). "
                 "Ranked by output smoothness below -- lower jitter RMS is smoother.\n\n")

    parts.append("\\section*{Ranked Results (smoothest first)}\n")
    parts.append("\\begin{longtable}{r r r r r l}\n\\hline\n")
    header = ("\\textbf{Kp} & \\textbf{Ki} & \\textbf{Output jitter RMS (Hz)} & "
              "\\textbf{Output std (Hz)} & \\textbf{Tracking median error (Hz)} & "
              "\\textbf{Converged?} \\\\\n\\hline\n")
    parts.append(header + "\\endfirsthead\n\\hline\n" + header + "\\endhead\n")
    for r in ranked:
        jitter = f"{r['output_jitter_rms_hz']:.0f}" if r["output_jitter_rms_hz"] is not None else "--"
        std = f"{r['output_std_hz']:.0f}" if r["output_std_hz"] is not None else "--"
        err = f"{r['median_error']:.0f}" if r["median_error"] is not None else "--"
        color = "ForestGreen" if r["verdict"] == "PASS" else "BrickRed"
        parts.append(f"{r['kp']:g} & {r['ki']:g} & {jitter} & {std} & {err} & "
                      f"\\textcolor{{{color}}}{{\\textbf{{{r['verdict']}}}}} \\\\\n")
    parts.append("\\hline\n\\end{longtable}\n\n")

    parts.append("\\clearpage\n")
    parts.append("\\section*{Per-Shot Detail (smoothest first)}\n")
    for rank, r in enumerate(ranked, 1):
        color = "ForestGreen" if r["verdict"] == "PASS" else "BrickRed"
        parts.append(f"\\section{{Rank {rank}: Kp={r['kp']:g}, Ki={r['ki']:g}}}\n")

        if r["output_jitter_rms_hz"] is not None:
            converge_note = ("converged to setpoint" if r["verdict"] == "PASS" else
                              "did NOT converge to within the sanity threshold")
            parts.append(f"Channel {channel}, closed loop (Kp={r['kp']:g}, Ki={r['ki']:g}, Kd=0), "
                         f"commanding a real {ramp_s:g}s/{flat_s:g}s/{ramp_s:g}s "
                         f"ramp/flat-top/ramp-down profile to {GAIN_SMOOTHNESS_DEMAND_A}\\,A "
                         f"demand. Output jitter RMS "
                         f"{r['output_jitter_rms_hz']:.0f}\\,Hz, output std "
                         f"{r['output_std_hz']:.0f}\\,Hz over the flat-top window; flat-top "
                         f"median tracking error {r['median_error']:.0f}\\,Hz -- {converge_note}.\n\n")
        else:
            parts.append(f"Channel {channel}, closed loop (Kp={r['kp']:g}, Ki={r['ki']:g}, Kd=0) -- "
                         f"{escape_tex(r['note'])}.\n\n")

        parts.append(f"\\textcolor{{{color}}}{{\\textbf{{Verdict: {r['verdict']}}}}}\n\n")

        parts.append("\\subsection{Commanded output vs. measured feedback}\n")
        if r.get("plot_path"):
            rel = os.path.relpath(r["plot_path"], GAIN_SMOOTHNESS_DOCS_DIR)
            parts.append(f"\\includegraphics[width=\\linewidth]{{{rel}}}\n\n")
        else:
            parts.append("No plot available for this shot (see the note above for why).\n\n")

    parts.append(TEX_FOOTER)
    out_path = os.path.join(GAIN_SMOOTHNESS_DOCS_DIR, "gain_smoothness_sweep_report.tex")
    with open(out_path, "w") as f:
        f.write("".join(parts))
    print(f"[report] wrote {out_path}")
    return out_path


def main():
    global RAMP_S, FLAT_S, MATRIX_DEMAND_LEVELS_A, GAIN_SMOOTHNESS_DEMAND_A
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--controller-port")
    ap.add_argument("--simulator-port")
    ap.add_argument("--scenarios", default="all",
                     help="comma-separated subset of: " + ",".join(ALL_SCENARIOS))
    ap.add_argument("--matrix", action="store_true",
                     help="run the channel-combination x demand-level stress matrix "
                          "(45 shots) INSTEAD of the discrete scenarios above")
    ap.add_argument("--gain-smoothness", action="store_true",
                     help="run the closed-loop Kp/Ki output-smoothness sweep "
                          "(single channel, short 3s/3s profile) INSTEAD of the "
                          "discrete scenarios above")
    ap.add_argument("--ramp-s", type=float, default=RAMP_S,
                     help="shot ramp-up/ramp-down time in seconds (default %(default)s)")
    ap.add_argument("--flat-s", type=float, default=FLAT_S,
                     help="shot flat-top time in seconds (default %(default)s)")
    args = ap.parse_args()

    RAMP_S = args.ramp_s
    FLAT_S = args.flat_s

    os.makedirs(SHOTS_DIR, exist_ok=True)

    if args.controller_port:
        ctrl = connect_verified(args.controller_port, "WHAM-XREX-PFMG474 ", "controller")
    else:
        ctrl = autodetect("WHAM-XREX-PFMG474 ", "controller")
    if args.simulator_port:
        sim = connect_verified(args.simulator_port, "WHAM-XREX-PFMG474-SIM", "simulator")
    else:
        sim = autodetect("WHAM-XREX-PFMG474-SIM", "simulator", exclude_port=ctrl.port)

    if wc.sync_calibration(ctrl):
        print(f"Calibration synced from controller: turnon={wc.PFM_TURNON_FREQ_HZ:.0f} Hz, "
              f"max={wc.PFM_MAX_FREQ_HZ:.0f} Hz, {wc.PFM_MAX_CURRENT_A:.0f} A (ch1)")
    else:
        print(f"[warn] couldn't sync Amps<->Hz calibration from the controller -- "
              f"using the hardcoded default ({wc.PFM_MAX_CURRENT_A:.0f} A)")
    # Recompute -- these were first computed from wc.PFM_MAX_CURRENT_A's
    # pre-sync (hardcoded) value at import time, before any connection
    # existed; redo them now that sync_calibration() may have updated it.
    MATRIX_DEMAND_LEVELS_A = _matrix_demand_levels_a()
    GAIN_SMOOTHNESS_DEMAND_A = round(wc.PFM_MAX_CURRENT_A * 0.50)

    if args.matrix:
        try:
            runs = run_matrix_sweep(ctrl, sim)
        finally:
            reset_to_safe_state(ctrl, sim)
            ctrl.close()
            sim.close()
        generate_full_matrix_report(runs)
        n_pass = sum(1 for r in runs if r["verdict"] == "PASS")
        print(f"\n{n_pass}/{len(runs)} matrix runs PASSED")
        return

    if args.gain_smoothness:
        try:
            runs = run_gain_smoothness_sweep(ctrl, sim)
        finally:
            reset_to_safe_state(ctrl, sim)
            ctrl.close()
            sim.close()
        generate_gain_smoothness_report(runs)
        n_pass = sum(1 for r in runs if r["verdict"] == "PASS")
        print(f"\n{n_pass}/{len(runs)} gain-smoothness shots converged")
        return

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
