#!/usr/bin/env python3
"""
wham_console.py -- interactive operator console for WHAM-XREX-PFMG474.

The intended day-to-day front end for a human operator working with a
real board: connect, program and fire a shot profile with guided
prompts (no need to remember PID:PROFILE:* argument order), send any
raw SCPI command directly, watch live status, and automatically
generate and save the same diagnostic plots used throughout this
project's own bench sessions (docs/changelog.txt's 2026-09-10 entries)
-- without hand-writing a one-off script each time.

THIS IS A MAINTAINED FRONT END, not a one-off script -- as new PID:*/
other commands are added to the firmware (commands.c/commands.h/
cmd_parser.c) and documented in docs/command_reference.md, extend this
file to match rather than leaving operators to fall back on raw SCPI
for everything. See "Adding a new console command" near the bottom of
this file before adding one.

Usage:
  python3 python/wham_console.py                      # auto-detect port
  python3 python/wham_console.py --port /dev/cu.usbserial-130
  python3 python/wham_console.py --no-connect          # start disconnected

Once running, type `help` for the full command list, or `help <cmd>`
for one command. Anything NOT recognized as a console command (see
`help`) is sent to the controller VERBATIM as a raw SCPI command --
e.g. just type `*IDN?`, `FAULT?`, `PID:GAINS 1 1.0 10.0 0.0`, or
`TABLE:BEGIN` directly. See docs/command_reference.md for the full
wire protocol this passes through to.

Prerequisites:
  pip install pyserial          (required)
  pip install matplotlib        (optional -- only needed for `plot`/`shot`'s
                                  auto-plot step; everything else works without it)
"""

import argparse
import cmd
import csv
import glob
import json
import math
import os
import re
import shlex
import sys
import time
from datetime import datetime

try:
    import serial  # pyserial
    from serial.tools import list_ports
except ImportError:
    sys.exit("error: pyserial not installed. Run:  pip install pyserial")

try:
    import matplotlib
    matplotlib.use("Agg")  # headless -- this tool only ever SAVES plots, never shows a window
    import matplotlib.pyplot as plt
    HAVE_MPL = True
except ImportError:
    HAVE_MPL = False

try:
    import dslogic_shot_capture  # optional DSLogic cross-check plot for `shot` --
                                  # see that module's own docstring; it never
                                  # raises and no-ops cleanly if no DSLogic is
                                  # connected. Guarded the same as the
                                  # matplotlib import above -- this file should
                                  # always ship alongside wham_console.py, but a
                                  # missing/broken sibling file shouldn't take
                                  # down the whole console over an optional
                                  # feature.
    HAVE_DSLOGIC_MODULE = True
except ImportError as exc:
    HAVE_DSLOGIC_MODULE = False
    print(f"[warn] dslogic_shot_capture.py not importable ({exc}) -- "
          f"`shot` will skip the DSLogic cross-check plot")

# --------------------------------------------------------------------------
# Defaults / constants
# --------------------------------------------------------------------------

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.dirname(SCRIPT_DIR)  # this script lives in python/
SHOTS_DIR = os.path.join(PROJECT_DIR, "shots")     # generated plots/logs -- gitignored
LOGS_DIR = os.path.join(PROJECT_DIR, "logs")       # session transcripts -- gitignored

APP_BAUD = 115200  # see AGENTS.md / docs/changelog.txt -- WHAM-XREX-PFMG474-only
DEFAULT_TIMEOUT = 2.0     # seconds, most commands
LOGDATA_TIMEOUT = 8.0     # PID:LOGDATA? can be a long single line (~1000 samples,
                          # up to ~18KB at 115200 baud -- budget generously)

# Amps<->Hz calibration -- MUST MATCH Core/Inc/ctrlr_config.h's
# PFM_TURNON_FREQ_HZ/PFM_MAX_FREQ_HZ/PFM_MAX_CURRENT_A exactly. There is
# no wire command that reports these (same situation as
# pfm_input_plot.py's HRTIM_TIMER_CLK_HZ duplication -- see that
# file's own comment) -- if those firmware constants ever change,
# update here too, or the Amps-axis plot will silently be wrong even
# though the Hz-axis one (driven entirely by on-wire values) stays
# correct regardless.
PFM_TURNON_FREQ_HZ = 5000.0
PFM_MAX_FREQ_HZ = 100000.0
PFM_MAX_CURRENT_A = 5000.0

# Same caveat as above -- MUST MATCH Core/Inc/ctrlr_config.h's
# PID_LOOP_RATE_HZ and Core/Inc/pid.h's PID_LOG_MAX_SAMPLES. Used only
# to auto-suggest a decim value in the shot wizard (see do_shot) that
# spreads PID_LOG_MAX_SAMPLES samples across a whole shot's duration
# -- the log itself is always read back with its own real rate_hz
# (PID_GetLogSampleRateHz(), reported directly in PID:LOGDATA?'s own
# reply), so a stale value here only makes the SUGGESTED decim
# non-optimal, never wrong/misleading data.
PID_LOOP_RATE_HZ_ASSUMED = 1000
PID_LOG_MAX_SAMPLES_ASSUMED = 1000

# Error codes -- see docs/command_reference.md's own table (kept here
# too so a raw ERR reply can be explained inline without forcing the
# operator to go look it up). Update alongside commands.c/commands.h
# and docs/command_reference.md if a new code is ever assigned --
# codes are never renumbered/reused per that doc's own convention.
ERROR_CODES = {
    1: "Unknown command",
    2: "Not currently uploading a table -- send TABLE:BEGIN first",
    3: "Table full",
    4: "Invalid TABLE:STEP arguments, or a value out of uint16 range (0-65535)",
    5: "Table is empty -- upload one first (TABLE:BEGIN/STEP/END)",
    6: "Fault latched -- send FAULT:CLEAR first",
    7: "QUADSPI command failed or timed out",
    8: "Invalid PFM_Input channel",
    9: "M out of range for PFMIN:CAPTURE",
    10: "TABLE:STEP per value implies a carrier frequency above the allowed max",
    11: "Invalid PID channel",
    12: "Invalid PID:* argument count/value -- see the command's own usage",
}

# Console meta-commands are deliberately named to NEVER collide (even
# case-insensitively) with the identchar-only FIRST TOKEN of any real
# SCPI mnemonic (see "Adding a new console command" at the bottom of
# this file for why that matters and how cmd.Cmd's dispatch works) --
# the current such first tokens are: BOOT, TABLE/TAB, FIRE, PFM,
# CONFIG/CONF, FAULT, GDS, QSPI, PFMIN, PID. None of this file's do_*
# methods are named any of those.


def find_port():
    """Same glob pattern as wham_serial_flash.py's auto-detect, for
    consistency -- returns the first match or None."""
    candidates = (
        glob.glob("/dev/cu.usbserial*")
        + glob.glob("/dev/tty.usbserial*")
        + glob.glob("/dev/ttyUSB*")
        + glob.glob("/dev/ttyACM*")
    )
    return candidates[0] if candidates else None


def ensure_dir(path):
    os.makedirs(path, exist_ok=True)
    return path


# --------------------------------------------------------------------------
# WhamLink -- the serial transport. One query() call = one command sent,
# one line read back, matching this firmware's own single-line-per-reply
# protocol (docs/command_reference.md's response conventions) exactly --
# no sleep-based buffering, no guessing how many bytes to expect.
# --------------------------------------------------------------------------

class WhamError(Exception):
    """Raised for anything that stops a command from completing cleanly
    (not connected, timeout, a malformed reply) -- NOT raised for a
    device-side ERR reply, which is returned as normal data (see
    query()'s own doc comment) since an ERR is a legitimate, expected
    protocol response an operator should be able to see and react to,
    not a Python-level failure."""


class WhamLink:
    def __init__(self, port=None, baud=APP_BAUD, log_fh=None):
        self.port = port
        self.baud = baud
        self.ser = None
        self.log_fh = log_fh  # session transcript file handle, or None

    @property
    def connected(self):
        return self.ser is not None and self.ser.is_open

    def connect(self, port=None, baud=None):
        if self.connected:
            self.close()
        self.port = port or self.port or find_port()
        self.baud = baud or self.baud
        if not self.port:
            raise WhamError("no serial port given and none auto-detected -- "
                             "pass one explicitly: connect /dev/cu.usbserial-XXXX")
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=DEFAULT_TIMEOUT)
        except serial.SerialException as exc:
            self.ser = None
            raise WhamError(f"couldn't open {self.port}: {exc}") from exc
        time.sleep(0.2)  # matches this project's other scripts -- lets the
                          # USART2 line settle before the first write
        self.ser.reset_input_buffer()

    def close(self):
        if self.ser is not None:
            try:
                self.ser.close()
            except serial.SerialException:
                pass
            self.ser = None

    def _log(self, direction, text):
        if self.log_fh is not None:
            ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            self.log_fh.write(f"[{ts}] {direction} {text}\n")
            self.log_fh.flush()

    def send_raw(self, command):
        """Fire-and-forget -- no reply is read. Only for commands that
        don't produce a normal reply on this link (BOOT: the MCU resets
        into the ROM bootloader immediately after acknowledging, and
        the acknowledgement itself races the reset -- see
        wham_serial_flash.py, which is the real tool for that flow)."""
        if not self.connected:
            raise WhamError("not connected -- try: connect")
        line = command if command.endswith(("\r", "\n")) else command + "\r\n"
        self._log(">>>", command)
        self.ser.write(line.encode("ascii", errors="replace"))
        self.ser.flush()

    def query(self, command, timeout=None):
        """Sends `command`, reads exactly one line back (this
        firmware's entire wire protocol is single-line-per-reply --
        see docs/command_reference.md), and returns it stripped of the
        trailing CRLF. Returns whatever the device sent, including a
        leading "ERR n ..." -- that is normal protocol data, not a
        Python exception (callers that care can check the prefix
        themselves, see parse_ok()/is_err() below). Raises WhamError
        only for a transport-level problem (not connected, no reply at
        all within `timeout`)."""
        if not self.connected:
            raise WhamError("not connected -- try: connect")
        if timeout is not None:
            self.ser.timeout = timeout
        try:
            self.ser.reset_input_buffer()
            line = command if command.endswith(("\r", "\n")) else command + "\r\n"
            self._log(">>>", command)
            self.ser.write(line.encode("ascii", errors="replace"))
            self.ser.flush()
            reply = self.ser.readline()
        finally:
            if timeout is not None:
                self.ser.timeout = DEFAULT_TIMEOUT
        if not reply:
            raise WhamError(f"no reply to {command!r} within {timeout or DEFAULT_TIMEOUT}s "
                             "-- board unresponsive, wrong port/baud, or still booting")
        text = reply.decode("ascii", errors="replace").strip()
        self._log("<<<", text)
        return text


def is_err(reply):
    return reply.startswith("ERR")


def explain_err(reply):
    """A raw 'ERR n message' reply, decorated with the error-code table
    above if the code is one this console knows about (it always
    should -- see ERROR_CODES' own comment on keeping it in sync)."""
    m = re.match(r"ERR\s+(\d+)\s*(.*)", reply)
    if not m:
        return reply
    code, msg = int(m.group(1)), m.group(2)
    known = ERROR_CODES.get(code)
    if known and known.lower() not in msg.lower():
        return f"{reply}   [code {code}: {known}]"
    return reply


# --------------------------------------------------------------------------
# Plotting -- generalized from the ad-hoc scratchpad scripts used to
# produce every plot in docs/changelog.txt's 2026-09-10 entries. Two
# modes: a full shot-profile plot (Hz + Amps, phase-shaded) when the
# caller has ramp/flat-top/demand-current metadata, or a plain Hz-only
# plot (setpoint/output/measured) for an ad-hoc PID:RAMP/PID:SETPOINT
# bench test where no profile was ever programmed.
# --------------------------------------------------------------------------

def hz_to_amps(hz):
    a = (hz - PFM_TURNON_FREQ_HZ) / (PFM_MAX_FREQ_HZ - PFM_TURNON_FREQ_HZ) * PFM_MAX_CURRENT_A
    return max(0.0, a)


def _find_glitches(measured, threshold=15000):
    """Isolated single-tick spikes -- see docs/changelog.txt's
    2026-09-10 entries for what these are (a real, DSLogic-confirmed
    feedback-measurement anomaly, now guarded against reaching HRTIM
    by PID_OUTPUT_MAX_SLEW_HZ_PER_TICK, but still visible in the raw
    Measured log by design). Flagged on the plot rather than silently
    smoothed away, matching this project's whole diagnostic
    philosophy."""
    idx = []
    for i in range(1, len(measured) - 1):
        if abs(measured[i] - measured[i - 1]) > threshold and abs(measured[i] - measured[i + 1]) > threshold:
            idx.append(i)
    return idx


def generate_plot(rows, meta, out_path):
    """rows: list of dicts with t_s/setpoint_hz/measured_hz/output_hz
    (float/int already converted). meta: see WhamConsole's own
    _shot_meta() for the exact keys: channel, timestamp, idn,
    ramp_time_s, flat_top_time_s, demand_current_a (any of the last
    three may be None -- ad-hoc test, not a profiled shot; or a query
    failure right at plot time), kp, ki, kd, loop_mode (normally read
    live from the device via _refresh_config(), may be None if that
    readback failed). Saves a PNG to out_path. Returns out_path."""
    if not HAVE_MPL:
        raise WhamError("matplotlib not installed -- run: pip install matplotlib")

    t = [r["t_s"] for r in rows]
    sp = [r["setpoint_hz"] for r in rows]
    ms = [r["measured_hz"] for r in rows]
    op = [r["output_hz"] for r in rows]

    have_profile = meta.get("ramp_time_s") and meta.get("flat_top_time_s") is not None \
        and meta.get("demand_current_a") is not None

    gains_str = (f"Kp={meta['kp']:g} Ki={meta['ki']:g} Kd={meta['kd']:g}"
                 if meta.get("kp") is not None else "gains unknown this session")
    title_top = (f"WHAM-XREX-PFMG474 channel {meta['channel']} -- {meta.get('idn', '')}\n"
                 f"{meta.get('timestamp', '')}   |   {gains_str}")

    if have_profile:
        fig, (ax_hz, ax_a) = plt.subplots(2, 1, figsize=(11, 9), sharex=True)
        ramp_s = meta["ramp_time_s"]
        flat_s = meta["flat_top_time_s"]
        total_s = 2 * ramp_s + flat_s
        demand_a = meta["demand_current_a"]
        sp_a = [hz_to_amps(v) for v in sp]
        ms_a = [hz_to_amps(v) for v in ms]
        op_a = [hz_to_amps(v) for v in op]

        phase_bounds = [(0, ramp_s, "#ffe8b3", "ramp up"),
                        (ramp_s, ramp_s + flat_s, "#c9f2c7", "flat top"),
                        (ramp_s + flat_s, total_s, "#ffd6d6", "ramp down")]

        for ax, ysp, yms, yop, ylabel in (
            (ax_hz, sp, ms, op, "Frequency (Hz)"),
            (ax_a, sp_a, ms_a, op_a, "Current (A)"),
        ):
            for x0, x1, color, label in phase_bounds:
                ax.axvspan(x0, x1, color=color, alpha=0.5, zorder=0,
                           label=label if ax is ax_hz else None)
            ax.step(t, ysp, where="post", color="#888888", lw=1.4, ls="--",
                     label="Setpoint (commanded profile)", zorder=3)
            ax.step(t, yop, where="post", color="#2a78d6", lw=1.8,
                     label="Output (written to HRTIM)", zorder=2)
            ax.step(t, yms, where="post", color="#eb6834", lw=1.8,
                     label="Measured (feedback)", zorder=4)
            ax.set_ylabel(ylabel)
            ax.grid(True, alpha=0.3)

        ax_a.axhline(demand_a, color="#444444", lw=0.8, ls=":", zorder=1)
        ax_a.annotate(f"Demand Current = {demand_a:.0f} A", xy=(total_s * 0.5, demand_a),
                      xytext=(4, 4), textcoords="offset points", fontsize=9, color="#444444")
        ax_hz.axhline(PFM_TURNON_FREQ_HZ, color="#444444", lw=0.8, ls=":", zorder=1)
        ax_hz.annotate(f"PFM_TURNON_FREQ_HZ = {PFM_TURNON_FREQ_HZ:.0f} Hz (0A floor)",
                        xy=(total_s * 0.02, PFM_TURNON_FREQ_HZ), xytext=(4, 6),
                        textcoords="offset points", fontsize=8, color="#444444")

        glitches = _find_glitches(ms)
        if glitches:
            first = glitches[0]
            ax_hz.annotate(
                f"{len(glitches)} isolated single-tick feedback dropout(s) in Measured\n"
                f"(see docs/changelog.txt 2026-09-10 -- guarded from reaching HRTIM by\n"
                f"PID_OUTPUT_MAX_SLEW_HZ_PER_TICK, still visible here as raw log data)",
                xy=(t[first], ms[first]), xytext=(0.98, 0.95), textcoords="axes fraction",
                fontsize=8, color="#a33", ha="right", va="top",
                arrowprops=dict(arrowstyle="->", color="#a33", lw=1.0),
            )

        ax_hz.legend(loc="upper left", fontsize=9)
        ax_a.legend(loc="upper right", fontsize=9)
        ax_a.set_xlabel("Time since PID:PROFILE:START (s)")
        fig.suptitle(
            title_top + f"\nRamp={ramp_s:.2f}s  FlatTop={flat_s:.2f}s  "
            f"Demand={demand_a:.0f}A  Total={total_s:.2f}s",
            fontsize=11,
        )
        fig.tight_layout(rect=(0, 0, 1, 0.90))
    else:
        fig, ax = plt.subplots(figsize=(11, 6))
        ax.step(t, sp, where="post", color="#888888", lw=1.4, ls="--", label="Setpoint")
        ax.step(t, op, where="post", color="#2a78d6", lw=1.8, label="Output (written to HRTIM)")
        ax.step(t, ms, where="post", color="#eb6834", lw=1.8, label="Measured (feedback)")
        ax.set_xlabel("Time since log armed (s)")
        ax.set_ylabel("Frequency (Hz)")
        ax.grid(True, alpha=0.3)
        ax.legend(loc="best", fontsize=9)
        ax.set_title(title_top + "\n(no shot profile active -- plain setpoint/output/measured)",
                     fontsize=11)
        fig.tight_layout()

    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    return out_path


def generate_multi_channel_plot(channel_rows, metas, out_path):
    """One figure, one row of subplots per channel (sharing a single
    time axis), each showing that channel's own setpoint/output/
    measured in Hz -- for a genuine simultaneous, directly-comparable
    view across every channel of the SAME shot (see PID_ArmLogAll()'s
    own doc comment in pid.h for why this needs firmware support, not
    just running the same shot N times and overlaying the results).
    channel_rows: {channel: rows} (rows as in generate_plot()).
    metas: {channel: meta} -- ramp_time_s/flat_top_time_s are expected
    to be the SAME across every entry (one shared shot profile);
    demand_current_a/kp/ki/kd/loop_mode are per-channel. Saves one PNG
    to out_path. Returns out_path."""
    if not HAVE_MPL:
        raise WhamError("matplotlib not installed -- run: pip install matplotlib")

    channels = sorted(channel_rows)
    if not channels:
        raise WhamError("no channel data to plot")

    any_meta = metas[channels[0]]
    ramp_s = any_meta.get("ramp_time_s")
    flat_s = any_meta.get("flat_top_time_s")
    have_profile = ramp_s is not None and flat_s is not None
    phase_bounds = []
    if have_profile:
        total_s = 2 * ramp_s + flat_s
        phase_bounds = [(0, ramp_s, "#ffe8b3", "ramp up"),
                        (ramp_s, ramp_s + flat_s, "#c9f2c7", "flat top"),
                        (ramp_s + flat_s, total_s, "#ffd6d6", "ramp down")]

    fig, axes = plt.subplots(len(channels), 1, figsize=(11, 2.6 * len(channels)),
                              sharex=True, squeeze=False)
    axes = axes[:, 0]

    for ax, ch in zip(axes, channels):
        rows = channel_rows[ch]
        meta = metas[ch]
        t = [r["t_s"] for r in rows]
        sp = [r["setpoint_hz"] for r in rows]
        ms = [r["measured_hz"] for r in rows]
        op = [r["output_hz"] for r in rows]

        for x0, x1, color, label in phase_bounds:
            ax.axvspan(x0, x1, color=color, alpha=0.4, zorder=0,
                       label=label if ax is axes[0] else None)
        ax.step(t, sp, where="post", color="#888888", lw=1.2, ls="--",
                 label="Setpoint" if ax is axes[0] else None, zorder=3)
        ax.step(t, op, where="post", color="#2a78d6", lw=1.6,
                 label="Output" if ax is axes[0] else None, zorder=2)
        ax.step(t, ms, where="post", color="#eb6834", lw=1.6,
                 label="Measured" if ax is axes[0] else None, zorder=4)

        demand_a = meta.get("demand_current_a")
        loop_mode = meta.get("loop_mode") or "?"
        subtitle = f"Ch{ch}  {loop_mode}-loop"
        if demand_a is not None:
            subtitle += f"  demand={demand_a:g}A"
        ax.set_ylabel("Hz", fontsize=9)
        ax.set_title(subtitle, fontsize=10, loc="left")
        ax.grid(True, alpha=0.3)
        ax.tick_params(labelsize=8)

    axes[0].legend(loc="upper right", fontsize=8, ncol=4)
    axes[-1].set_xlabel("Time since log armed (s)")

    idn = any_meta.get("idn", "")
    ts = any_meta.get("timestamp", "")
    title = f"WHAM-XREX-PFMG474 -- all channels -- {idn}\n{ts}"
    if have_profile:
        title += f"   |   Ramp={ramp_s:.2f}s FlatTop={flat_s:.2f}s Total={total_s:.2f}s"
    fig.suptitle(title, fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.94))

    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    return out_path


# --------------------------------------------------------------------------
# The console itself
# --------------------------------------------------------------------------

DANGEROUS_EXACT = {"FIRE", "BOOT"}


def _is_dangerous(first_token):
    """Best-effort heuristic, NOT a clone of cmd_parser.c's scpi_match()
    -- see this function's own limitation note. Catches: FIRE, BOOT,
    and PID:START/PID:PROFILE:START in any valid SCPI short/long form
    (the mandatory short form for "STARt" is always "STAR", so
    matching the final colon-segment's prefix covers every valid
    on-wire spelling of that leaf regardless of how earlier segments
    -- PID, PROFILE -- are themselves abbreviated).

    KNOWN LIMITATION: this is a small, explicit heuristic, not a full
    reimplementation of the firmware's own short/long-form matcher --
    it will not catch every conceivable future dangerous command
    automatically. If a new command is added that begins real output
    (matching FIRE/PID:START's danger level), add it here explicitly
    -- see "Adding a new console command" below."""
    t = first_token.upper().lstrip(":")
    if t in DANGEROUS_EXACT:
        return True
    segs = t.split(":")
    return segs[0] == "PID" and len(segs) >= 2 and segs[-1].startswith("STAR")


class WhamConsole(cmd.Cmd):
    intro = (
        "=" * 70 + "\n"
        "WHAM-XREX-PFMG474 operator console\n"
        "Type a raw SCPI command directly (e.g. *IDN?, FAULT?, PID:GAINS ...)\n"
        "or `help` for console commands (shot wizard, plotting, status, ...).\n"
        + "=" * 70
    )

    def __init__(self, port=None, baud=APP_BAUD, auto_connect=True):
        super().__init__()
        ensure_dir(SHOTS_DIR)
        ensure_dir(LOGS_DIR)
        self._log_fh = open(
            os.path.join(LOGS_DIR, datetime.now().strftime("session_%Y%m%d_%H%M%S.log")),
            "a", encoding="ascii", errors="replace",
        )
        self.link = WhamLink(port=port, baud=baud, log_fh=self._log_fh)
        self.confirm_dangerous = True
        self.num_channels = None
        # Cache of the device's own gains/loop-mode/demand-current/
        # profile-timing -- refreshed from a live PID:GAINS?/
        # PID:LOOPMODE?/PID:PROFILE:TIMING?/PID:PROFILE:CURRENT? readback
        # on every connect and on demand via `config` (see
        # _refresh_config()), and kept up to date incrementally by the
        # gains/loopmode/shot wrapper commands as they send new values.
        # Used to pre-fill `shot` wizard prompts with real current
        # values rather than guessed ones.
        self.channel_config = {}   # {ch: {"kp":, "ki":, "kd":, "loop_mode":, "demand_a":}}
        self.profile_timing = None  # {"ramp_s":, "flat_s":}
        self.last_log_channel = None
        self.last_log_all = False   # True if `log all`/the shot wizard last armed
                                     # PID:LOG 0 (every channel at once)
        self._update_prompt()

        if auto_connect:
            try:
                self.link.connect(port=port, baud=baud)
                self._on_connect()
            except WhamError as exc:
                print(f"[not connected] {exc}")
                self._update_prompt()

    # -- plumbing -----------------------------------------------------

    def _update_prompt(self):
        if self.link.connected:
            self.prompt = f"wham({os.path.basename(self.link.port)})> "
        else:
            self.prompt = "wham(disconnected)> "

    def _on_connect(self):
        self._update_prompt()
        self.channel_config = {}
        self.profile_timing = None
        try:
            idn = self.link.query("*IDN?")
            print(f"Connected to {self.link.port} @ {self.link.baud} -- {idn}")
            ch_reply = self.link.query("CONFig:CHANnels?")
            if not is_err(ch_reply):
                self.num_channels = int(ch_reply.split()[1])
                print(f"CONFig:CHANnels? -> {self.num_channels}")
        except (WhamError, ValueError, IndexError) as exc:
            print(f"[warn] connected, but couldn't query *IDN?/CONFig:CHANnels?: {exc}")
            return
        self._refresh_config()

    # -- live device readback for gains/loopmode/profile timing/current --
    # (added 2026-09-10, once PID:GAINS?/PID:LOOPMODE?/PID:PROFile:TIMing?/
    # PID:PROFile:CURRent? existed on the firmware side -- see docs/
    # changelog.txt. Before this, channel_config/profile_timing could only
    # be POPULATED by this console's own setter wrappers remembering what
    # THEY sent; now every value is read straight from the device, so it's
    # correct even after a reconnect or after another tool/operator changed
    # something. do_config's own docstring still explains this history.)

    def _query_gains(self, ch):
        reply = self.link.query(f"PID:GAINS? {ch}")
        if is_err(reply):
            return None
        parts = reply.split()
        return dict(kp=float(parts[1]), ki=float(parts[2]), kd=float(parts[3]))

    def _query_loopmode(self, ch):
        reply = self.link.query(f"PID:LOOPMODE? {ch}")
        if is_err(reply):
            return None
        return "closed" if reply.split()[1] == "1" else "open"

    def _query_channel_enable(self, ch):
        reply = self.link.query(f"PID:CHANNEL:ENABLE? {ch}")
        if is_err(reply):
            return None
        return reply.split()[1] == "1"

    def _query_profile_current(self, ch):
        reply = self.link.query(f"PID:PROFILE:CURRENT? {ch}")
        if is_err(reply):
            return None
        return float(reply.split()[1])

    def _query_profile_timing(self):
        reply = self.link.query("PID:PROFILE:TIMING?")
        if is_err(reply):
            return None  # not set yet on the device -- a real, distinct state
        parts = reply.split()
        return dict(ramp_s=float(parts[1]), flat_s=float(parts[2]))

    def _refresh_config(self):
        """Repopulates self.channel_config / self.profile_timing from the
        device itself (not this console's own memory of what it sent) --
        called on connect, and available on demand via `config`."""
        n = self.num_channels or 4
        for ch in range(1, n + 1):
            entry = self.channel_config.setdefault(ch, {})
            try:
                gains = self._query_gains(ch)
                if gains:
                    entry.update(gains)
                loop_mode = self._query_loopmode(ch)
                if loop_mode:
                    entry["loop_mode"] = loop_mode
                enabled = self._query_channel_enable(ch)
                if enabled is not None:
                    entry["enabled"] = enabled
                demand_a = self._query_profile_current(ch)
                if demand_a is not None:
                    entry["demand_a"] = demand_a
            except WhamError:
                pass
        try:
            timing = self._query_profile_timing()
            self.profile_timing = timing
        except WhamError:
            pass

    def _require_link(self):
        if not self.link.connected:
            print("Not connected -- try: connect")
            return False
        return True

    def _confirm(self, prompt_text):
        if not self.confirm_dangerous:
            return True
        try:
            ans = input(f"{prompt_text} [y/N] ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            return False
        return ans in ("y", "yes")

    def _query_print(self, command, timeout=None):
        """The shared path for both raw passthrough (default()) and
        every wrapper command below -- one place that applies the
        danger-confirmation gate, prints the reply, and decorates an
        ERR reply with its known meaning."""
        first_token = command.strip().split(None, 1)[0] if command.strip() else ""
        if _is_dangerous(first_token):
            if not self._confirm(f"'{command.strip()}' will command real PWM output. Proceed?"):
                print("Cancelled.")
                return None
        try:
            reply = self.link.query(command, timeout=timeout)
        except WhamError as exc:
            print(f"[error] {exc}")
            return None
        print(explain_err(reply) if is_err(reply) else reply)
        return reply

    def default(self, line):
        if not line.strip():
            return
        if not self._require_link():
            return
        self._query_print(line)

    def emptyline(self):
        pass  # don't repeat the last command on a bare Enter (cmd.Cmd's default)

    # -- connection -----------------------------------------------------

    def do_connect(self, arg):
        """connect [port] [baud]  -- open (or reopen) the serial link.
        With no arguments, auto-detects a USB-serial port and uses the
        default baud (115200). Examples:
          connect
          connect /dev/cu.usbserial-130
          connect /dev/cu.usbserial-130 115200"""
        parts = shlex.split(arg)
        port = parts[0] if len(parts) >= 1 else None
        baud = int(parts[1]) if len(parts) >= 2 else None
        try:
            self.link.connect(port=port, baud=baud)
            self._on_connect()
        except WhamError as exc:
            print(f"[error] {exc}")
            self._update_prompt()

    def do_disconnect(self, arg):
        """disconnect  -- close the serial link."""
        self.link.close()
        self._update_prompt()
        print("Disconnected.")

    def do_ports(self, arg):
        """ports  -- list available serial ports (with descriptions)."""
        found = list(list_ports.comports())
        if not found:
            print("No serial ports found.")
            return
        for p in found:
            marker = " *" if p.device == self.link.port else ""
            print(f"  {p.device}{marker}  {p.description}")

    # -- quick diagnostics (thin, unambiguous wrappers) ------------------

    def do_idn(self, arg):
        """idn  -- shortcut for *IDN?"""
        if self._require_link():
            self._query_print("*IDN?")

    def do_channels(self, arg):
        """channels  -- shortcut for CONFig:CHANnels?, also refreshes
        the channel count this console uses elsewhere (e.g. `status`,
        `shot`)."""
        if not self._require_link():
            return
        reply = self._query_print("CONFig:CHANnels?")
        if reply and not is_err(reply):
            try:
                self.num_channels = int(reply.split()[1])
            except (ValueError, IndexError):
                pass

    # -- status / monitoring ---------------------------------------------

    def _status_row(self, ch):
        reply = self.link.query(f"PID:STATus? {ch}")
        if is_err(reply):
            return None
        parts = reply.split()
        # OK <running> <setpointHz> <measuredHz> <outputHz>
        return dict(running=int(parts[1]), setpoint=int(parts[2]),
                    measured=int(parts[3]), output=int(parts[4]))

    def do_status(self, arg):
        """status [channel]  -- top-level state (STATE?) plus live
        PID:STATus? for one channel, or every channel if none given
        (uses CONFig:CHANnels?'s count). Pretty-printed table: Ch |
        Running | Setpoint | Measured | Output (Hz)."""
        if not self._require_link():
            return
        state_reply = self.link.query("STATE?")
        print(f"State: {state_reply[3:] if state_reply.startswith('OK ') else state_reply}")
        channels = [int(arg)] if arg.strip() else range(1, (self.num_channels or 4) + 1)
        print(f"{'Ch':>3} {'Running':>8} {'Setpoint(Hz)':>13} {'Measured(Hz)':>13} {'Output(Hz)':>11}")
        for ch in channels:
            try:
                row = self._status_row(ch)
            except WhamError as exc:
                print(f"[error] channel {ch}: {exc}")
                continue
            if row is None:
                print(f"{ch:>3}  <ERR -- invalid channel?>")
                continue
            print(f"{ch:>3} {('yes' if row['running'] else 'no'):>8} "
                  f"{row['setpoint']:>13} {row['measured']:>13} {row['output']:>11}")

    def do_monitor(self, arg):
        """monitor [interval_s]  -- repeatedly print `status` every
        interval_s seconds (default 0.5) until Ctrl-C. Does not stop
        anything on the device when interrupted -- it's read-only."""
        if not self._require_link():
            return
        interval = float(arg.strip()) if arg.strip() else 0.5
        print(f"Monitoring every {interval}s -- Ctrl-C to stop.")
        try:
            while True:
                print(f"--- {datetime.now().strftime('%H:%M:%S')} ---")
                self.do_status("")
                time.sleep(interval)
        except KeyboardInterrupt:
            print("\nStopped.")

    def do_config(self, arg):
        """config  -- live readback of gains/loop-mode/demand-current/
        profile-timing for every channel, straight from the device
        (PID:GAINS?/PID:LOOPMODE?/PID:PROFILE:TIMING?/PID:PROFILE:CURRENT?,
        added 2026-09-10) -- always current, including after a
        reconnect or after another tool/operator changed something.
        Also refreshes this console's own local cache (used to
        pre-fill `shot` wizard defaults) as a side effect."""
        if not self._require_link():
            return
        self._refresh_config()
        if self.profile_timing:
            print(f"Profile timing: ramp={self.profile_timing['ramp_s']:g}s "
                  f"flat-top={self.profile_timing['flat_s']:g}s")
        else:
            print("Profile timing: not set on the device (PID:PROFILE:TIMING never sent)")
        if not self.channel_config:
            print("No per-channel config readable.")
            return
        for ch in sorted(self.channel_config):
            c = self.channel_config[ch]
            gains = (f"Kp={c['kp']:g} Ki={c['ki']:g} Kd={c['kd']:g}"
                     if c.get("kp") is not None else "gains: unavailable")
            enabled = c.get("enabled")
            enabled_str = "DISABLED (no output)" if enabled is False else ("enabled" if enabled else "unavailable")
            print(f"  ch{ch}: {enabled_str}  loop_mode={c.get('loop_mode', 'unavailable')}  "
                  f"demand={c.get('demand_a', 'unavailable')}A  {gains}")

    # -- direct wrappers (safe: no first-token collision, see module header) --

    def do_gains(self, arg):
        """gains <ch> <kp> <ki> <kd>  -- wrapper for PID:GAINS, also
        remembers the value in `config` (session-local, see its own
        help)."""
        if not self._require_link():
            return
        parts = shlex.split(arg)
        if len(parts) != 4:
            print("usage: gains <ch> <kp> <ki> <kd>")
            return
        ch, kp, ki, kd = parts
        reply = self._query_print(f"PID:GAINS {ch} {kp} {ki} {kd}")
        if reply and not is_err(reply):
            self.channel_config.setdefault(int(ch), {}).update(
                kp=float(kp), ki=float(ki), kd=float(kd))

    def do_loopmode(self, arg):
        """loopmode <ch> <open|closed>  -- wrapper for PID:LOOPMODE
        (translates open/closed to 0/1). Also remembers it in `config`."""
        if not self._require_link():
            return
        parts = shlex.split(arg)
        if len(parts) != 2 or parts[1].lower() not in ("open", "closed"):
            print("usage: loopmode <ch> <open|closed>")
            return
        ch, mode = parts
        bit = 1 if mode.lower() == "closed" else 0
        reply = self._query_print(f"PID:LOOPMODE {ch} {bit}")
        if reply and not is_err(reply):
            self.channel_config.setdefault(int(ch), {})["loop_mode"] = mode.lower()

    def do_enable(self, arg):
        """enable <ch> <on|off>  -- wrapper for PID:CHANNEL:ENABLE: a
        genuine "this channel outputs nothing at all" switch, distinct
        from `loopmode` (open-loop still drives a real, uncorrected PFM
        waveform) or a 0A demand current (still drives a real PFM
        waveform, at the turn-on floor). Takes effect on the NEXT
        `start`/`shot` if the loop isn't running yet; takes effect
        IMMEDIATELY, live, if it is -- turning a channel back on while
        the loop is already running asks to confirm first (see
        `confirm`), since that's a real, immediate new output; turning
        one off never asks (always the safe direction)."""
        if not self._require_link():
            return
        parts = shlex.split(arg)
        if len(parts) != 2 or parts[1].lower() not in ("on", "off"):
            print("usage: enable <ch> <on|off>")
            return
        ch, state = parts
        turning_on = state.lower() == "on"
        if turning_on and not self._confirm(
                f"Enabling channel {ch}'s output -- if the loop is already running, "
                f"this takes effect immediately. Proceed?"):
            print("Cancelled.")
            return
        reply = self._query_print(f"PID:CHANNEL:ENABLE {ch} {1 if turning_on else 0}")
        if reply and not is_err(reply):
            self.channel_config.setdefault(int(ch), {})["enabled"] = turning_on

    def do_setpoint(self, arg):
        """setpoint <ch> <hz>  -- wrapper for PID:SETPOINT."""
        if not self._require_link():
            return
        parts = shlex.split(arg)
        if len(parts) != 2:
            print("usage: setpoint <ch> <hz>")
            return
        self._query_print(f"PID:SETPOINT {parts[0]} {parts[1]}")

    def do_ramp(self, arg):
        """ramp <ch> <startHz> <endHz> <durationMs>  -- wrapper for PID:RAMP."""
        if not self._require_link():
            return
        parts = shlex.split(arg)
        if len(parts) != 4:
            print("usage: ramp <ch> <startHz> <endHz> <durationMs>")
            return
        self._query_print("PID:RAMP " + " ".join(parts))

    def do_start(self, arg):
        """start  -- wrapper for PID:START (asks to confirm first)."""
        if self._require_link():
            self._query_print("PID:START")

    def do_stop(self, arg):
        """stop  -- wrapper for PID:STOP (always safe, no confirmation)."""
        if self._require_link():
            reply = self.link.query("PID:STOP")
            print(explain_err(reply) if is_err(reply) else reply)

    def do_log(self, arg):
        """log <ch|all> <maxSamples> <decim>  -- wrapper for PID:LOG
        (arms waveform logging). `all` arms every channel at once, from
        the SAME real ticks (PID:LOG 0 ...) -- for a genuine
        simultaneous cross-channel comparison, see `shot`'s own
        multi-channel plotting. Remembers the target so a later `plot`
        with no arguments knows what to fetch."""
        if not self._require_link():
            return
        parts = shlex.split(arg)
        if len(parts) != 3:
            print("usage: log <ch|all> <maxSamples> <decim>")
            return
        ch_arg, max_samples, decim = parts
        all_channels = ch_arg.strip().lower() == "all"
        wire_ch = "0" if all_channels else ch_arg
        reply = self._query_print(f"PID:LOG {wire_ch} {max_samples} {decim}")
        if reply and not is_err(reply):
            self.last_log_all = all_channels
            self.last_log_channel = None if all_channels else int(ch_arg)

    # -- plotting ---------------------------------------------------------

    def _fetch_log(self, channel):
        """Runs PID:LOGDATA? <channel>, returns (rows, rate_hz, count)
        or raises WhamError. rows: list of dicts with t_s/setpoint_hz/
        measured_hz/output_hz."""
        reply = self.link.query(f"PID:LOGDATA? {channel}", timeout=LOGDATA_TIMEOUT)
        if is_err(reply):
            raise WhamError(explain_err(reply))
        parts = reply.split()
        if len(parts) < 2 or parts[0] != "OK":
            raise WhamError(f"unexpected PID:LOGDATA? reply: {reply!r}")
        count = int(parts[1])
        rate_hz = int(parts[2])  # always present, even for count=0 -- see cmd_pid_logdata()
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

    def _shot_meta(self, channel):
        cfg = self.channel_config.get(channel, {})
        idn = ""
        try:
            idn = self.link.query("*IDN?", timeout=1.0)
        except WhamError:
            pass
        meta = dict(
            channel=channel,
            timestamp=datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            idn=idn,
            kp=cfg.get("kp"), ki=cfg.get("ki"), kd=cfg.get("kd"),
            loop_mode=cfg.get("loop_mode"),
        )
        if self.profile_timing:
            meta["ramp_time_s"] = self.profile_timing["ramp_s"]
            meta["flat_top_time_s"] = self.profile_timing["flat_s"]
            meta["demand_current_a"] = cfg.get("demand_a")
        else:
            meta["ramp_time_s"] = meta["flat_top_time_s"] = meta["demand_current_a"] = None
        return meta

    def _save_and_plot(self, channel, rows, rate_hz, count, ts=None):
        if count == 0:
            print("Log is empty (PID:LOGDATA? returned 0 samples) -- nothing to plot. "
                  "Did you arm logging (`log`) before the shot/test ran?")
            return
        ts = ts or datetime.now().strftime("%Y%m%d_%H%M%S")
        base = os.path.join(SHOTS_DIR, f"{ts}_ch{channel}")
        meta = self._shot_meta(channel)
        meta["rate_hz"] = rate_hz
        meta["log_count"] = count

        with open(base + ".csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t_s", "setpoint_hz", "measured_hz", "output_hz"])
            for r in rows:
                w.writerow([f"{r['t_s']:.5f}", r["setpoint_hz"], r["measured_hz"], r["output_hz"]])
        with open(base + "_meta.json", "w") as f:
            json.dump(meta, f, indent=2)
        print(f"Saved {base}.csv, {base}_meta.json")

        if not HAVE_MPL:
            print("matplotlib not installed -- skipping plot (data still saved above). "
                  "Run: pip install matplotlib")
            return
        try:
            png = generate_plot(rows, meta, base + ".png")
            print(f"Saved {png}")
        except WhamError as exc:
            print(f"[error] plotting: {exc}")

    def _fetch_save_plot_all(self, ts=None):
        """Fetches PID:LOGDATA? for every channel (all-channels mode
        must be currently/previously armed -- PID_ArmLogAll()), saves
        one combined wide-format CSV + one metadata JSON + ONE PNG
        (generate_multi_channel_plot()) showing every channel on a
        shared time axis. Used by both `shot` (right after a shot with
        `all` logging) and `plot all`."""
        n = self.num_channels or 4
        channel_rows = {}
        rate_hz = 0
        count = 0
        for ch in range(1, n + 1):
            try:
                rows, rate_hz, count = self._fetch_log(ch)
            except WhamError as exc:
                print(f"[error] channel {ch}: {exc}")
                continue
            channel_rows[ch] = rows

        if not channel_rows or count == 0:
            print("No data (log empty, or all-channels logging was never armed -- "
                  "`log all <maxSamples> <decim>` first).")
            return

        metas = {ch: self._shot_meta(ch) for ch in channel_rows}
        for ch, meta in metas.items():
            meta["rate_hz"] = rate_hz
            meta["log_count"] = count

        ts = ts or datetime.now().strftime("%Y%m%d_%H%M%S")
        base = os.path.join(SHOTS_DIR, f"{ts}_allch")

        # One wide CSV: t_s, then setpoint/measured/output per channel --
        # all channels share the same t_s (same real ticks), so a wide
        # format is both more compact and easier to open in one sheet
        # than N separate files.
        with open(base + ".csv", "w", newline="") as f:
            w = csv.writer(f)
            header = ["t_s"]
            for ch in channel_rows:
                header += [f"ch{ch}_setpoint_hz", f"ch{ch}_measured_hz", f"ch{ch}_output_hz"]
            w.writerow(header)
            for i in range(count):
                row = [f"{channel_rows[next(iter(channel_rows))][i]['t_s']:.5f}"]
                for ch in channel_rows:
                    r = channel_rows[ch][i]
                    row += [r["setpoint_hz"], r["measured_hz"], r["output_hz"]]
                w.writerow(row)
        with open(base + "_meta.json", "w") as f:
            json.dump(metas, f, indent=2)
        print(f"Saved {base}.csv, {base}_meta.json ({len(channel_rows)} channels, "
              f"{count} samples each)")

        if not HAVE_MPL:
            print("matplotlib not installed -- skipping plot (data still saved above). "
                  "Run: pip install matplotlib")
            return
        try:
            png = generate_multi_channel_plot(channel_rows, metas, base + ".png")
            print(f"Saved {png}")
        except WhamError as exc:
            print(f"[error] plotting: {exc}")

    def do_plot(self, arg):
        """plot [channel|all]  -- fetches whatever waveform log is
        currently in the controller (PID:LOGDATA?) right now and saves
        a CSV + metadata JSON + PNG plot under shots/. With no
        argument, uses whatever `log`/`shot` last armed this session
        (a single channel, or all channels). `all` (or a channel
        number) overrides that -- pass one explicitly if you armed
        logging some other way (e.g. raw `PID:LOG 0 500 1`)."""
        if not self._require_link():
            return
        want_all = arg.strip().lower() == "all" or (not arg.strip() and self.last_log_all)
        if want_all:
            self._fetch_save_plot_all()
            return
        channel = int(arg.strip()) if arg.strip() else self.last_log_channel
        if channel is None:
            print("No channel known -- pass one: plot <channel>  (or `plot all`, "
                  "or arm logging first with `log <ch|all> ...`)")
            return
        try:
            rows, rate_hz, count = self._fetch_log(channel)
        except WhamError as exc:
            print(f"[error] {exc}")
            return
        self._save_and_plot(channel, rows, rate_hz, count)

    # -- the shot-profile wizard -------------------------------------------

    def _prompt_float(self, text, default):
        raw = input(f"{text} [{default:g}]: ").strip()
        if not raw:
            return default
        try:
            return float(raw)
        except ValueError:
            print("  not a number, keeping default")
            return default

    def do_shot(self, arg):
        """shot  -- guided wizard to program and run a full shot
        profile (Ramp Time / Flat Top Time / per-channel enable/
        disable, Demand Current, loop mode, gains), then watches it run
        and
        automatically fetches + plots the result. Logging can target
        one channel or 'all' -- 'all' arms every channel from the SAME
        real ticks (PID:LOG 0 ...) and produces ONE combined plot with
        one row per channel, a genuine simultaneous comparison rather
        than N separate runs. Re-running `shot` reuses your previous
        answers as the new defaults (just press Enter to repeat a shot
        unchanged). Ctrl-C at any prompt cancels without sending
        anything.

        If a DreamSourceLab DSLogic is connected (Phase U/V/W/X wired to
        its Ch0/Ch1/Ch2/Ch3, see dslogic_shot_capture.py), this also arms
        a DSLogic capture right before firing and saves an independent
        frequency-vs-firmware-ground-truth cross-check plot -- one panel
        per channel, including disabled/idle ones -- to shots/
        (<timestamp>_dslogic.png) alongside the usual CSV/JSON/PNG --
        silently skipped if no DSLogic is plugged in."""
        if not self._require_link():
            return
        n = self.num_channels or 4
        try:
            print(f"\n--- Shot wizard ({n} channels) --- (Ctrl-C to cancel)\n")
            prev_timing = self.profile_timing or {"ramp_s": 5.0, "flat_s": 2.0}
            ramp_s = self._prompt_float("Ramp Time (s)", prev_timing["ramp_s"])
            flat_s = self._prompt_float("Flat Top Time (s)", prev_timing["flat_s"])

            per_channel = {}
            for ch in range(1, n + 1):
                prev = self.channel_config.get(ch, {})
                prev_enabled = prev.get("enabled", True)
                raw = input(f"  Ch{ch} enabled? yes/no [{'yes' if prev_enabled else 'no'}]: ").strip().lower()
                enabled = prev_enabled if not raw else (raw not in ("n", "no"))
                if not enabled:
                    print(f"  Ch{ch}: DISABLED -- no PFM waveform at all, skipping its other prompts.")
                    per_channel[ch] = dict(enabled=False, demand_a=0.0, loop_mode="open",
                                            kp=None, ki=None, kd=None)
                    continue

                demand_a = self._prompt_float(f"  Ch{ch} Demand Current (A)", prev.get("demand_a", 0.0))
                loop_mode = prev.get("loop_mode", "closed")
                if demand_a > 0:
                    raw = input(f"  Ch{ch} loop mode open/closed [{loop_mode}]: ").strip().lower()
                    if raw in ("open", "closed"):
                        loop_mode = raw
                    kp = prev.get("kp")
                    ki = prev.get("ki")
                    kd = prev.get("kd")
                    if loop_mode == "closed":
                        default_gains = f"{kp:g} {ki:g} {kd:g}" if kp is not None else "skip"
                        raw = input(f"  Ch{ch} gains 'Kp Ki Kd' [{default_gains}]: ").strip()
                        if raw and raw != "skip":
                            try:
                                kp, ki, kd = (float(x) for x in raw.split())
                            except ValueError:
                                print("  couldn't parse 3 numbers, keeping previous")
                    per_channel[ch] = dict(enabled=True, demand_a=demand_a, loop_mode=loop_mode,
                                            kp=kp, ki=ki, kd=kd)
                else:
                    per_channel[ch] = dict(enabled=True, demand_a=0.0, loop_mode="open",
                                            kp=None, ki=None, kd=None)

            active = [ch for ch, c in per_channel.items() if c["enabled"] and c["demand_a"] > 0]
            if self.last_log_all:
                default_log = "all"
            elif self.last_log_channel:
                default_log = str(self.last_log_channel)
            else:
                default_log = str(active[0]) if active else "1"
            raw = input(f"Arm waveform log on channel [{default_log}, 'all', "
                        f"or 'none']: ").strip().lower()
            if not raw:
                raw = default_log
            log_all = raw == "all"
            log_channel = None if raw == "none" else (None if log_all else int(raw))

            total_s = 2 * ramp_s + flat_s
            total_ticks = total_s * PID_LOOP_RATE_HZ_ASSUMED
            auto_decim = max(1, math.ceil(total_ticks / PID_LOG_MAX_SAMPLES_ASSUMED))
            raw = input(f"Log decim (ticks/sample) [{auto_decim}, auto-sized so "
                        f"~1000 samples cover the full {total_s:.2f}s shot]: ").strip()
            decim = int(raw) if raw else auto_decim

            print("\n--- Summary ---")
            print(f"  Ramp={ramp_s:g}s  FlatTop={flat_s:g}s  Total={total_s:g}s")
            for ch, c in per_channel.items():
                if not c["enabled"]:
                    print(f"  Ch{ch}: DISABLED -- no output")
                elif c["demand_a"] > 0:
                    g = f"Kp={c['kp']:g} Ki={c['ki']:g} Kd={c['kd']:g}" if c["kp"] is not None else "gains unchanged"
                    print(f"  Ch{ch}: {c['demand_a']:g}A, {c['loop_mode']}-loop, {g}")
                else:
                    print(f"  Ch{ch}: enabled, idle (0A)")
            if log_all:
                log_desc = f"ALL {n} channels, decim={decim}"
            elif log_channel:
                log_desc = f"channel {log_channel}, decim={decim}"
            else:
                log_desc = "none"
            print(f"  Log: {log_desc}\n")

            if not self._confirm("Program and START this shot now?"):
                print("Cancelled.")
                return
        except KeyboardInterrupt:
            print("\nCancelled.")
            return

        # -- program it --
        self.link.query("PID:STOP")
        for ch, c in per_channel.items():
            # Sent explicitly every time, enabled or not -- matches this
            # project's "explicit, no implicit magic" convention (same
            # reasoning as TABLE:BEGIN before every upload, FAULT:CLEAR
            # even after a condition clears) -- so a channel enabled in
            # a previous session/shot doesn't silently stay enabled (or
            # vice versa) just because this wizard run didn't mention it.
            self.link.query(f"PID:CHANNEL:ENABLE {ch} {1 if c['enabled'] else 0}")
            if c["enabled"]:
                bit = 1 if c["loop_mode"] == "closed" else 0
                self.link.query(f"PID:LOOPMODE {ch} {bit}")
                self.link.query(f"PID:PROFILE:CURRENT {ch} {c['demand_a']}")
                if c["kp"] is not None:
                    self.link.query(f"PID:GAINS {ch} {c['kp']} {c['ki']} {c['kd']}")
            self.channel_config[ch] = dict(enabled=c["enabled"], demand_a=c["demand_a"],
                                            loop_mode=c["loop_mode"],
                                            kp=c["kp"], ki=c["ki"], kd=c["kd"])
        if log_all:
            self.link.query(f"PID:LOG 0 1000 {decim}")
            self.last_log_all = True
            self.last_log_channel = None
        elif log_channel:
            self.link.query(f"PID:LOG {log_channel} 1000 {decim}")
            self.last_log_all = False
            self.last_log_channel = log_channel
        self.link.query(f"PID:PROFILE:TIMING {ramp_s} {flat_s}")
        self.profile_timing = dict(ramp_s=ramp_s, flat_s=flat_s)

        ts = datetime.now().strftime("%Y%m%d_%H%M%S")  # shared across every artifact
                                                         # this one shot produces, so
                                                         # they're easy to find together

        # State machine (added 2026-09-13, firmware side): PID:PROFILE:START
        # now only works from ARMED -- see ARM's own reply for why, if it
        # fails (e.g. a fault is already latched).
        reply = self.link.query("ARM")
        if is_err(reply):
            print(explain_err(reply))
            return

        # Optional DSLogic cross-check plot (see dslogic_shot_capture.py's own
        # docstring) -- armed here, right before START, matching the
        # arm-then-fire timing this was validated with. Cleanly skipped if no
        # DSLogic is connected. Once connected, it always captures the fixed
        # DSLogic-wired channels (WHAM ch 1/2/3/4 -- Phase U/V/W/X) regardless of
        # which channels are active this shot -- a disabled/idle channel
        # still gets a panel showing its output stayed silent.
        dsl_capture = None
        if HAVE_DSLOGIC_MODULE and dslogic_shot_capture.is_available():
            dsl_capture = dslogic_shot_capture.ShotCapture()
            if not dsl_capture.arm(per_channel, total_s):
                dsl_capture = None

        reply = self.link.query("PID:PROFILE:START")
        if is_err(reply):
            print(explain_err(reply))
            return
        print(reply)
        fire_t0 = time.time()

        if log_channel:
            watch_ch = log_channel
        elif log_all and active:
            watch_ch = active[0]
        elif active:
            watch_ch = active[0]
        else:
            watch_ch = 1
        # For the DSLogic plot's per-channel ground-truth panels: poll
        # EVERY channel (the plot shows one panel per channel regardless
        # of enable state), not just watch_ch -- console output below
        # still only prints watch_ch, to keep the existing UX unchanged.
        dsl_fw_log = []
        dsl_poll_channels = list(per_channel) if dsl_capture else []
        print(f"Shot running -- watching channel {watch_ch} (Ctrl-C to stop watching, "
              "shot keeps running)...")
        t0 = time.time()
        try:
            while time.time() - t0 < total_s + 2.0:
                row = self._status_row(watch_ch)
                if row is None:
                    break
                print(f"  t+{time.time()-t0:5.2f}s  running={row['running']}  "
                      f"setpoint={row['setpoint']}  measured={row['measured']}  "
                      f"output={row['output']}")
                if dsl_poll_channels:
                    fw_row = {"t_s": time.time() - fire_t0}
                    for ch in dsl_poll_channels:
                        r = row if ch == watch_ch else self._status_row(ch)
                        if r is not None:
                            fw_row[ch] = dict(setpoint=r["setpoint"], measured=r["measured"])
                    dsl_fw_log.append(fw_row)
                if row["running"] == 0 and time.time() - t0 > 0.5:
                    break
                time.sleep(0.3)
        except KeyboardInterrupt:
            print("\n(stopped watching -- shot itself keeps running on the device)")

        if dsl_capture is not None:
            print("Waiting for DSLogic capture and plotting cross-check...")
            out_path = os.path.join(SHOTS_DIR, f"{ts}_dslogic.png")
            saved = dsl_capture.finish_and_plot(dsl_fw_log, out_path)
            if saved:
                print(f"Saved {saved}")

        if log_all:
            print("Fetching all-channel log and plotting...")
            self._fetch_save_plot_all(ts=ts)
        elif log_channel:
            print("Fetching log and plotting...")
            try:
                rows, rate_hz, count = self._fetch_log(log_channel)
            except WhamError as exc:
                print(f"[error] {exc}")
                return
            self._save_and_plot(log_channel, rows, rate_hz, count, ts=ts)

    # -- reflash integration -------------------------------------------

    def do_reflash(self, arg):
        """reflash [bin_path]  -- runs wham_serial_flash.py to reflash
        firmware over this same serial link (BOOT + stm32flash write +
        verify + go). Confirms first. Closes this console's own serial
        link during the flash and reconnects after."""
        if not self._require_link():
            return
        if not self._confirm("This will re-flash firmware over the current link. Proceed?"):
            print("Cancelled.")
            return
        port = self.link.port
        self.link.close()
        cmd_list = [sys.executable, os.path.join(SCRIPT_DIR, "wham_serial_flash.py"),
                    "--port", port]
        if arg.strip():
            cmd_list += ["--bin", arg.strip()]
        import subprocess
        rc = subprocess.call(cmd_list)
        print(f"wham_serial_flash.py exited {rc}")
        time.sleep(1.0)
        try:
            self.link.connect(port=port)
            self._on_connect()
        except WhamError as exc:
            print(f"[warn] couldn't reconnect after reflash: {exc}")
            self._update_prompt()

    # -- misc -------------------------------------------------------------

    def do_confirm(self, arg):
        """confirm [on|off]  -- toggle the safety prompt before
        commands that start real PWM output (FIRE, PID:START,
        PID:PROFILE:START, and BOOT). With no argument, shows the
        current setting. Defaults to on."""
        arg = arg.strip().lower()
        if arg in ("on", "off"):
            self.confirm_dangerous = (arg == "on")
        print(f"confirm: {'on' if self.confirm_dangerous else 'off'}")

    def do_quit(self, arg):
        """quit  -- close the link and exit."""
        return True

    def do_exit(self, arg):
        """exit  -- same as quit."""
        return True

    def do_EOF(self, arg):
        """Ctrl-D  -- same as quit."""
        print()
        return True

    def postloop(self):
        self.link.close()
        self._log_fh.close()


# --------------------------------------------------------------------------
# Adding a new console command
#
# 1. If the real work is just sending one SCPI command with friendlier
#    argument handling (like do_gains/do_setpoint/do_ramp above), add a
#    `do_<name>` method following that pattern.
# 2. Pick a name that CANNOT be typed as the leading identchar-only
#    token of a real SCPI mnemonic -- cmd.Cmd dispatches on an exact,
#    case-SENSITIVE match against `do_<token>` where `token` is
#    whatever identchar run starts the input line (stopping at the
#    first ':', '?', '*', digit-after-nothing, or space). Concretely:
#    do NOT name a command (in any case) boot, table, tab, fire, pfm,
#    config, conf, fault, gds, qspi, pfmin, pid, arm, disarm, or state
#    -- those are the current first (and, for arm/disarm/state, ONLY)
#    tokens of real commands, and shadowing one would silently break
#    raw passthrough for that command (or whole family) whenever an
#    operator happens to type it in the matching case -- ARM/DISARM/
#    STATE? (state_machine.h, added 2026-09-13) are bare, single-level
#    commands with no further exception the way *IDN? has, so this
#    applies to them exactly as literally as to fire/pfm/pid. (idn is
#    fine: the real command is "*IDN?", and a leading '*' is never an
#    identchar, so it always reaches default() regardless of what
#    do_idn exists.)
# 3. Update docs/command_reference.md if the underlying firmware
#    command is new -- this console should never be the only place a
#    command's behavior is documented.
# 4. If it's genuinely dangerous (starts real output), add it to
#    _is_dangerous()'s check above.
# --------------------------------------------------------------------------


def main():
    ap = argparse.ArgumentParser(description="Interactive operator console for WHAM-XREX-PFMG474")
    ap.add_argument("--port", help="serial device (auto-detected if omitted)")
    ap.add_argument("--baud", type=int, default=APP_BAUD, help=f"default: {APP_BAUD}")
    ap.add_argument("--no-connect", action="store_true", help="start without connecting")
    args = ap.parse_args()

    console = WhamConsole(port=args.port, baud=args.baud, auto_connect=not args.no_connect)
    try:
        console.cmdloop()
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        console.link.close()
        if not console._log_fh.closed:
            console._log_fh.close()


if __name__ == "__main__":
    main()
