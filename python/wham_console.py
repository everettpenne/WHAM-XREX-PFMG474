#!/usr/bin/env python3
"""
wham_console.py -- interactive operator console for WHAM-XREX-PFMG474.

The intended day-to-day front end for a human operator working with a
real board: connect, program and fire a shot profile with guided
prompts (no need to remember SHOT:* argument order), send any
raw SCPI command directly, watch live status, and automatically
generate and save the same diagnostic plots used throughout this
project's own bench sessions (docs/changelog.txt's 2026-09-10 entries)
-- without hand-writing a one-off script each time. `diag` gives a
one-shot debugging snapshot (state/fault/gate-driver pins/config/live
status, all channels, one action); `report [ch|all]` does everything
`plot` does plus a written shot-performance summary saved under
shots/, headlined by output-vs-commanded error (does the frequency the
controller actually DROVE match what the shot profile commands? --
computed independently of any feedback measurement) alongside
feedback-vs-commanded and a feedback-vs-output bench-wiring check --
see compute_log_stats()'s own doc comment for why those are kept
separate (this bench's feedback is currently a loopback of the output,
not an independent supply).

Firing a shot -- through `shot`'s own wizard, `refire` (re-fires
whatever is currently programmed, e.g. after `gains <ch> <kp> <ki>
<kd>` -- no wizard prompts), OR a raw SHOT:STARt typed
directly -- always auto-fetches + plots afterward (direct request,
2026-09-22), INCLUDING a 4-row (channel 1-4) x 2-col output/FFT plot
(generate_output_fft_plot()) showing each channel's commanded output
AND measured feedback -- full shot span, not just a flat-top snippet
-- in the time domain next to each trace's own frequency spectrum, so
high-frequency noise on the drive side (see this project's
closed-loop-output-noise-vs-integrator-action finding) and how much of
it survives into the feedback are both visible immediately after every
shot without a separate analysis pass.

THIS IS A MAINTAINED FRONT END, not a one-off script -- as new SOURce:/SHOT:/LOG:/CHANnel:/
commands are added to the firmware (commands.c/commands.h/
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
  pip install numpy             (optional -- only needed for the output/FFT
                                  plot `shot`/`refire`/a raw SHOT:STARt
                                  auto-generate; everything else works without it)
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
    import numpy as np  # only needed for generate_output_fft_plot()'s FFT math
    HAVE_NUMPY = True
except ImportError:
    HAVE_NUMPY = False

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
LOGDATA_TIMEOUT = 8.0     # LOG:DATA? can be a long single line (~1000 samples,
                          # up to ~18KB at 115200 baud -- budget generously)

# Amps<->Hz calibration -- ctrlr_config.h's PFM_TURNON_FREQ_HZ/
# PFM_MAX_FREQ_HZ/PFM_MAX_CURRENT_A_PER_CHANNEL[0] compile-time
# defaults, used as the startup/fallback values. As of 2026-09-22
# these are ALSO live-configurable on the controller itself
# (CONFig:TURNONHz/MAXFREQHz/MAXCURRent -- docs/command_reference.md),
# which is exactly what makes hardcoded copies dangerous: an operator
# changing the real value with those commands would silently leave
# this Amps-axis plot conversion wrong (the Hz-axis one, driven
# entirely by on-wire values, stays correct regardless). sync_calibration()
# below refreshes these three globals from the live device -- called
# automatically on every connect (_on_connect()) and after any raw
# CONFig:TURNONHz/MAXFREQHz/MAXCURRent typed directly (default(), via
# _is_calibration_set()) -- so treat these three as "the last-synced
# value", not a permanent constant. Per-channel: PFM_MAX_CURRENT_A is
# queried from channel 1 only -- this conversion has always used one
# flat scalar for every channel's plot, even though the firmware
# calibration is genuinely per-channel; that simplification predates
# this sync and isn't something it changes.
PFM_TURNON_FREQ_HZ = 5000.0
PFM_MAX_FREQ_HZ = 100000.0
PFM_MAX_CURRENT_A = 6000.0  # updated 2026-09-18 -- ctrlr_config.h's
                            # PFM_MAX_CURRENT_A_PER_CHANNEL changed from
                            # 5000.0f to 6000.0f (confirmed real installed
                            # LEMs are 6kA, not the original 5kA spec)


def sync_calibration(link):
    """Refreshes the Amps<->Hz globals above AND PID_LOOP_RATE_HZ_ASSUMED
    (below) from the live controller's own CONFig:TURNONHz?/
    CONFig:MAXFREQHz?/CONFig:MAXCURRent? (channel 1)/CONFig:PIDRate? --
    added 2026-09-22 alongside those commands, closing the staleness
    gap the comments above/below used to just warn about (the loop-
    rate half added slightly later the same day, once the same gap was
    noticed there too -- same hand-kept-in-sync-copy problem, same
    fix). Returns True on success. Never raises: an older firmware
    without these commands, or any query error, just leaves the
    current values (the hardcoded defaults, or whatever a previous
    sync last set) in place -- none of these four are a safety check,
    only a plot axis or a suggested-decim calculation, so silently
    keeping the last-known-good value is the right failure mode here."""
    global PFM_TURNON_FREQ_HZ, PFM_MAX_FREQ_HZ, PFM_MAX_CURRENT_A, PID_LOOP_RATE_HZ_ASSUMED
    try:
        turnon = link.query("CONFig:TURNONHz?", timeout=2.0)
        maxfreq = link.query("CONFig:MAXFREQHz?", timeout=2.0)
        maxcurr = link.query("CONFig:MAXCURRent? 1", timeout=2.0)
        looprate = link.query("CONFig:PIDRate?", timeout=2.0)
    except WhamError:
        return False
    if is_err(turnon) or is_err(maxfreq) or is_err(maxcurr) or is_err(looprate):
        return False
    try:
        PFM_TURNON_FREQ_HZ = float(turnon.split()[1])
        PFM_MAX_FREQ_HZ = float(maxfreq.split()[1])
        PFM_MAX_CURRENT_A = float(maxcurr.split()[1])
        PID_LOOP_RATE_HZ_ASSUMED = int(float(looprate.split()[1]))
    except (ValueError, IndexError):
        return False
    return True

# PID_LOOP_RATE_HZ_ASSUMED: now kept in sync by sync_calibration()
# above (added 2026-09-23, same live-staleness fix as the Amps<->Hz
# globals -- CONFig:PIDRate made this genuinely runtime-configurable
# 2026-09-22, and this copy hadn't been wired up to it yet). Used only
# to auto-suggest a decim value in the shot wizard (see do_shot) that
# spreads PID_LOG_MAX_SAMPLES_ASSUMED samples across a whole shot's
# duration -- the log itself is always read back with its own real
# rate_hz (PID_GetLogSampleRateHz(), reported directly in LOG:DATA?'s
# own reply), so a stale value here only makes the SUGGESTED decim
# non-optimal, never wrong/misleading data.
#
# PID_LOG_MAX_SAMPLES_ASSUMED: *** REAL BUG, FOUND AND FIXED
# 2026-09-23 *** -- this was hardcoded 1000, but Core/Inc/pid.h's
# PID_LOG_MAX_SAMPLES is 1750 and always has been (confirmed directly
# against source, not assumed) -- NOT runtime-configurable at all (a
# compile-time log-buffer size, no CONFig:* command backs it), so this
# was unconditionally wrong by 43%, not just a live-sync gap. Corrected
# to the real value; nothing here needs live syncing since the real
# constant can't change without a rebuild.
PID_LOOP_RATE_HZ_ASSUMED = 1000
PID_LOG_MAX_SAMPLES_ASSUMED = 1750

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
    11: "Invalid channel",
    12: "Invalid command arguments",
    13: "Invalid state-machine transition for the current state (ARM/DISARM/SHOT:STARt)",
    14: "Invalid CHANnel:NICKname -- 1-PID_CHANNEL_NICKNAME_MAX_LEN chars, no spaces, not the reserved value '-'",
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
        self._rx_buf = b""    # bytes received but not yet split into a
                               # complete line -- see _read_line()

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
        self._rx_buf = b""

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

    def _emit_event(self, text):
        """Print (and log) a received unsolicited !EVT line, tagged with
        this host's wall-clock. The line already carries the device's
        monotonic ms tick; the wall-clock is just for the transcript."""
        ts = datetime.now().strftime("%H:%M:%S")
        self._log("<<<", text)
        print(f"[{ts}] {text}")

    def _read_line(self, timeout=None):
        """Blocks up to `timeout` seconds for one complete CRLF/LF-
        terminated line, pulling first from self._rx_buf (bytes already
        read off the wire but not yet split into a line) before reading
        more. A trailing partial line -- one still arriving, not yet
        newline-terminated -- is left in self._rx_buf for the *next*
        call rather than discarded, so a line can never be clipped by a
        badly timed read. Returns the decoded, stripped line, or None on
        timeout with no complete line available.

        This replaces an earlier design where drain_events()/query()
        each did a one-shot self.ser.read(in_waiting)/reset_input_buffer()
        and threw away whatever partial bytes that read happened to
        catch, on the assumption leftover bytes could only ever be a
        stale, already-fully-arrived line. That assumption broke in
        practice: a command that triggers a fault emits its own OK reply
        immediately, then a !EVT line a poll cycle later (real, observed
        latency, not hypothetical) -- if the next command was sent with
        ~no delay, that !EVT line could still be trickling in over the
        UART when reset_input_buffer() fired, silently eating the back
        half of it and desyncing every reply after it by one line."""
        deadline = time.monotonic() + (self.ser.timeout if timeout is None else timeout)
        while b"\n" not in self._rx_buf:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return None
            old_timeout = self.ser.timeout
            self.ser.timeout = remaining
            try:
                chunk = self.ser.read(max(1, self.ser.in_waiting or 1))
            finally:
                self.ser.timeout = old_timeout
            if not chunk:
                return None
            self._rx_buf += chunk
        idx = self._rx_buf.index(b"\n")
        raw, self._rx_buf = self._rx_buf[:idx + 1], self._rx_buf[idx + 1:]
        return raw.decode("ascii", errors="replace").strip()

    def drain_events(self):
        """Non-blocking: emit any !EVT lines that have fully arrived
        (via _read_line(), so a still-trickling-in line is left buffered
        rather than clipped), discarding any complete non-!EVT line
        found the same way (stale by definition -- query() always fully
        consumes its own reply before returning, so nothing legitimate
        should be sitting here between commands). Returns how many
        events were emitted. Safe to call while idle or pre-send."""
        emitted = 0
        if not self.connected:
            return emitted
        while True:
            if self.ser.in_waiting:
                self._rx_buf += self.ser.read(self.ser.in_waiting)
            if b"\n" not in self._rx_buf:
                break
            line = self._read_line(timeout=0)
            if line is None:
                break
            if line.startswith("!EVT"):
                self._emit_event(line)
                emitted += 1
        return emitted

    def query(self, command, timeout=None):
        """Sends `command`, reads back its single-line reply (this
        firmware's wire protocol is single-line-per-reply -- see
        docs/command_reference.md), and returns it stripped of CRLF.

        Unsolicited `!EVT` telemetry lines -- which can arrive at any
        time between or during commands (docs/telemetry.md, Phase 1) --
        are surfaced (printed + logged) and skipped, so they can never
        be mistaken for a reply; the first non-!EVT line is the reply.
        Returns whatever the device sent, including a leading "ERR n ..."
        (normal protocol data, not a Python exception -- callers check
        is_err()). Raises WhamError only for a transport-level problem
        (not connected, no reply at all within `timeout`)."""
        if not self.connected:
            raise WhamError("not connected -- try: connect")
        eff_timeout = DEFAULT_TIMEOUT if timeout is None else timeout
        # Surface (and discard) anything already sitting in the buffer
        # before sending. No reset_input_buffer() here -- see
        # _read_line()'s doc for why that used to be able to clip an
        # in-flight !EVT line; draining via complete lines only is safe.
        self.drain_events()
        line = command if command.endswith(("\r", "\n")) else command + "\r\n"
        self._log(">>>", command)
        self.ser.write(line.encode("ascii", errors="replace"))
        self.ser.flush()
        seen_events = 0
        while True:
            text = self._read_line(timeout=eff_timeout)
            if text is None:
                raise WhamError(f"no reply to {command!r} within {eff_timeout}s "
                                "-- board unresponsive, wrong port/baud, or still booting")
            if text.startswith("!EVT"):
                self._emit_event(text)
                seen_events += 1
                if seen_events > 64:
                    raise WhamError(f"too many interleaved !EVT lines while awaiting {command!r}")
                continue
            self._log("<<<", text)
            return text

    def query_evlog(self, timeout=None):
        """SYS:EVLOG? -- the flight-recorder replay (docs/telemetry.md
        Phase 4). Its reply is MULTI-line: a header "OK <n>" (which the
        normal query() returns) followed by n one-line `!EVT` packets.
        Reads the header via query(), then drains the n event lines.
        Returns (header, [event lines]). If the header isn't "OK <n>",
        returns (header, [])."""
        header = self.query("SYS:EVLOG?", timeout=timeout)
        if not header.startswith("OK"):
            return header, []
        try:
            n = int(header.split()[1])
        except (IndexError, ValueError):
            return header, []
        events = []
        for _ in range(n):
            line = self._read_line(timeout=timeout)
            if line is None:
                break   # truncated replay -- return what we got
            self._log("<<<", line)
            events.append(line)
        return header, events


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
# plot (setpoint/output/measured) for an ad-hoc SOURce:RAMP/SOURce:SETpoint
# bench test where no profile was ever programmed.
# --------------------------------------------------------------------------

def hz_to_amps(hz):
    a = (hz - PFM_TURNON_FREQ_HZ) / (PFM_MAX_FREQ_HZ - PFM_TURNON_FREQ_HZ) * PFM_MAX_CURRENT_A
    return max(0.0, a)


def hz_delta_to_amps(delta_hz):
    """Like hz_to_amps() but for a DIFFERENCE in Hz (e.g. tracking
    error), not an absolute setpoint/measured/output value -- no
    PFM_TURNON_FREQ_HZ offset subtraction and no clamp-to-zero.
    hz_to_amps() itself would misleadingly report ~0.00A for any delta
    smaller than the turn-on threshold, which is true of nearly every
    real tracking-error value -- this is the correct linear slope
    (PFM_MAX_CURRENT_A over the Hz span above turn-on) applied to a
    signed delta instead. Same 'nominal' calibration caveat."""
    return delta_hz / (PFM_MAX_FREQ_HZ - PFM_TURNON_FREQ_HZ) * PFM_MAX_CURRENT_A


def amps_to_hz(amps):
    """Inverse of hz_to_amps(): converts a target Amps demand into the
    Hz value the wire protocol actually encodes it as (SHOT:
    CURRent and every other demand-current command take a Hz-domain
    value, matching firmware's own AmpsToHz(), pid.c). Added 2026-09-22
    to replace several copies of this same formula that had been
    hand-inlined elsewhere (run_simulator_validation.py) with
    PFM_MAX_FREQ_HZ hardcoded as a bare 100000.0 -- doubly stale, since
    that's now also live-configurable (CONFig:MAXFREQHz). No clamping
    to [PFM_TURNON_FREQ_HZ, PFM_MAX_FREQ_HZ] -- same 'nominal
    calibration, not a safety check' caveat as hz_to_amps()."""
    return PFM_TURNON_FREQ_HZ + (amps / PFM_MAX_CURRENT_A) * (PFM_MAX_FREQ_HZ - PFM_TURNON_FREQ_HZ)


def channel_label(channel, nickname=None):
    """'Ch3' or 'Ch3 (TINKYWINKY)' -- per direct instruction, the bare
    Ch<N> label is ALWAYS kept even when a nickname is set (it's the
    actual wire-protocol channel identity; CHANnel:NICKname is a
    purely cosmetic extra label, never a replacement for the channel
    number everywhere else on the wire)."""
    if nickname:
        return f"Ch{channel} ({nickname})"
    return f"Ch{channel}"


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


def _error_stats(a, b):
    """mean/RMS/max|abs| of (a[i]-b[i]) across two equal-length sequences
    -- the one signed-error computation every comparison below reuses."""
    n = len(a)
    err = [a[i] - b[i] for i in range(n)]
    return dict(mean=sum(err) / n,
                max_abs=max(abs(e) for e in err),
                rms=math.sqrt(sum(e * e for e in err) / n))


def compute_log_stats(rows, rate_hz):
    """Summary statistics for one channel's fetched waveform log (rows:
    see WhamLink._fetch_log()/generate_plot()'s own doc for the exact
    shape) -- backs `report`/`diag`. Computes THREE separate, distinctly-
    named comparisons rather than one generic "tracking error" -- direct
    request, and an important distinction on this hardware:

      - output_error (output - setpoint): does the frequency the
        controller actually DROVE match what the shot profile COMMANDS?
        setpoint_hz is computed by TrapezoidalCurrentA()/AmpsToHz()
        (pid.c) purely from ramp/flat-top/demand-current -- it never
        depends on any feedback measurement, so this is a clean, always-
        meaningful check of the controller's own output stage (PID math
        + the hard slew clamp), valid in BOTH open- and closed-loop mode.
        THIS is "does the output match what it should" -- the headline
        number, see _print_report_summary_line().
      - feedback_error (measured - setpoint): does the FEEDBACK signal
        reach the target? In closed loop this is what the PID is
        actually trying to drive to zero via measured feedback, so it
        also reflects loop convergence/tuning, not just the output
        stage alone.
      - feedback_vs_output_error (measured - output): a bench-wiring
        self-consistency check. *** IMPORTANT CAVEAT, direct instruction
        to keep in mind: on THIS bench, TODAY, the feedback measurement
        is physically wired as a loopback of the controller's own output
        -- not an independent Transrex supply's real response. A small
        feedback_vs_output_error today mostly just confirms the loopback
        wiring/timer-capture path itself, NOT real hardware tracking; a
        LARGE one would still be a real finding (wiring/measurement
        fault) worth flagging even so. Once a real Transrex feeds this
        channel's feedback pin, feedback_error becomes the meaningful
        closed-loop-performance number and this row starts reflecting
        real external supply dynamics instead of a wiring self-check. ***

    All three computed over the WHOLE log, ramp segments included -- no
    attempt to isolate a "steady-state" window, since this project has
    no established rise-time/overshoot spec to judge against yet; report
    the numbers, don't invent a pass/fail verdict. Glitch detection
    reuses _find_glitches() (the same DSLogic-confirmed single-tick
    feedback anomaly _save_and_plot()'s own plot already flags).
    Returns None for an empty log (nothing to summarize)."""
    n = len(rows)
    if n == 0:
        return None
    sp = [r["setpoint_hz"] for r in rows]
    ms = [r["measured_hz"] for r in rows]
    op = [r["output_hz"] for r in rows]
    glitches = _find_glitches(ms)
    return dict(
        count=n, rate_hz=rate_hz, duration_s=(n / rate_hz) if rate_hz else 0.0,
        setpoint_min=min(sp), setpoint_max=max(sp), setpoint_final=sp[-1],
        measured_min=min(ms), measured_max=max(ms), measured_final=ms[-1],
        output_min=min(op), output_max=max(op), output_final=op[-1],
        output_error=_error_stats(op, sp),
        feedback_error=_error_stats(ms, sp),
        feedback_vs_output_error=_error_stats(ms, op),
        glitch_count=len(glitches),
        glitch_sample_indices=glitches[:20],  # capped -- a full list belongs
                                               # in the saved CSV, not this
                                               # summary
    )


def format_channel_report_md(meta, stats):
    """One channel's section of a `report`/`diag`-generated markdown
    file. meta: see _shot_meta(). stats: see compute_log_stats(), or
    None (channel had no log data -- reported as such, not skipped, so
    a report never silently omits a channel the operator asked about).
    Amps figures use the same linear placeholder as generate_plot()
    (hz_to_amps()) -- labeled 'nominal' throughout, per this project's
    standing calibration caveat (see AGENTS.md)."""
    label = channel_label(meta["channel"], meta.get("nickname"))
    lines = [f"## {label}", ""]
    lines.append(f"- Loop mode: {meta.get('loop_mode') or 'unknown'}")
    if meta.get("kp") is not None:
        lines.append(f"- Gains: Kp={meta['kp']:g} Ki={meta['ki']:g} Kd={meta['kd']:g}")
    else:
        lines.append("- Gains: unavailable (PID:GAINS? failed or never read)")
    if meta.get("demand_current_a") is not None:
        lines.append(f"- Demand current: {meta['demand_current_a']:g} A (nominal)")
    if meta.get("ramp_up_time_s") is not None:
        lines.append(f"- Profile timing: ramp up={meta['ramp_up_time_s']:g}s "
                      f"flat-top={meta['flat_top_time_s']:g}s "
                      f"ramp down={meta['ramp_down_time_s']:g}s")
    lines.append("")

    if stats is None:
        lines.append("**No log data** -- LOG:DATA? returned 0 samples for this "
                      "channel (logging was never armed before the run, or the "
                      "channel never ran).")
        lines.append("")
        return "\n".join(lines)

    lines.append(f"- Samples: {stats['count']} @ {stats['rate_hz']} Hz "
                  f"({stats['duration_s']:.3f} s)")
    lines.append("")
    lines.append("| | Hz | nominal A |")
    lines.append("|---|---|---|")
    for label_, key in (("Setpoint/commanded (min/max/final)", "setpoint"),
                         ("Measured/feedback (min/max/final)", "measured"),
                         ("Output/driven (min/max/final)", "output")):
        lo, hi, fin = stats[f"{key}_min"], stats[f"{key}_max"], stats[f"{key}_final"]
        lines.append(f"| {label_} | {lo}/{hi}/{fin} | "
                      f"{hz_to_amps(lo):.2f}/{hz_to_amps(hi):.2f}/{hz_to_amps(fin):.2f} |")

    def _err_row(label_, e):
        return (f"| {label_} | {e['mean']:.1f}/{e['rms']:.1f}/{e['max_abs']:.1f} "
                f"| {hz_delta_to_amps(e['mean']):.2f}/{hz_delta_to_amps(e['rms']):.2f}/"
                f"{hz_delta_to_amps(e['max_abs']):.2f} |")

    lines.append(_err_row("**Output vs commanded**, mean/RMS/max abs (output - setpoint) "
                           "-- did the controller drive what the shot profile commands?",
                           stats["output_error"]))
    lines.append(_err_row("Feedback vs commanded, mean/RMS/max abs (measured - setpoint) "
                           "-- did the feedback signal reach target? (loop convergence)",
                           stats["feedback_error"]))
    lines.append(_err_row("Feedback vs output, mean/RMS/max abs (measured - output) -- "
                           "bench wiring self-check, see caveat below",
                           stats["feedback_vs_output_error"]))
    lines.append("")
    lines.append("*Feedback is currently wired as a loopback of this controller's own "
                  "output, not an independent Transrex supply -- \"Feedback vs commanded\" "
                  "above mostly validates loop math/convergence against that loopback "
                  "today, not real external hardware tracking, and \"Feedback vs output\" "
                  "is a wiring/measurement self-check rather than a performance number. "
                  "**\"Output vs commanded\" is the one that answers whether the output "
                  "frequency matches what it should, independent of the feedback path.**")
    lines.append("")
    if stats["glitch_count"]:
        lines.append(f"- **{stats['glitch_count']} isolated single-tick glitch(es)** in "
                      f"Measured at sample index(es) {stats['glitch_sample_indices']}"
                      f"{' (first 20)' if stats['glitch_count'] > 20 else ''} -- see "
                      f"_find_glitches()'s own doc comment (wham_console.py) for what these "
                      f"are; HRTIM output itself is slew-clamped against them, this is a raw "
                      f"feedback-measurement artifact, not necessarily a real output anomaly.")
    else:
        lines.append("- No glitches detected (_find_glitches()).")
    lines.append("")
    return "\n".join(lines)


def generate_plot(rows, meta, out_path):
    """rows: list of dicts with t_s/setpoint_hz/measured_hz/output_hz
    (float/int already converted). meta: see WhamConsole's own
    _shot_meta() for the exact keys: channel, timestamp, idn,
    ramp_up_time_s, flat_top_time_s, ramp_down_time_s, demand_current_a
    (any of the last four may be None -- ad-hoc test, not a profiled
    shot; or a query failure right at plot time), kp, ki, kd, loop_mode
    (normally read live from the device via _refresh_config(), may be
    None if that readback failed). Saves a PNG to out_path. Returns
    out_path."""
    if not HAVE_MPL:
        raise WhamError("matplotlib not installed -- run: pip install matplotlib")

    t = [r["t_s"] for r in rows]
    sp = [r["setpoint_hz"] for r in rows]
    ms = [r["measured_hz"] for r in rows]
    op = [r["output_hz"] for r in rows]

    have_profile = meta.get("ramp_up_time_s") and meta.get("flat_top_time_s") is not None \
        and meta.get("ramp_down_time_s") and meta.get("demand_current_a") is not None

    gains_str = (f"Kp={meta['kp']:g} Ki={meta['ki']:g} Kd={meta['kd']:g}"
                 if meta.get("kp") is not None else "gains unknown this session")
    title_top = (f"WHAM-XREX-PFMG474 {channel_label(meta['channel'], meta.get('nickname'))} "
                 f"-- {meta.get('idn', '')}\n"
                 f"{meta.get('timestamp', '')}   |   {gains_str}")

    if have_profile:
        fig, (ax_hz, ax_a) = plt.subplots(2, 1, figsize=(11, 9), sharex=True)
        ramp_up_s = meta["ramp_up_time_s"]
        flat_s = meta["flat_top_time_s"]
        ramp_down_s = meta["ramp_down_time_s"]
        total_s = ramp_up_s + flat_s + ramp_down_s
        demand_a = meta["demand_current_a"]
        sp_a = [hz_to_amps(v) for v in sp]
        ms_a = [hz_to_amps(v) for v in ms]
        op_a = [hz_to_amps(v) for v in op]

        phase_bounds = [(0, ramp_up_s, "#ffe8b3", "ramp up"),
                        (ramp_up_s, ramp_up_s + flat_s, "#c9f2c7", "flat top"),
                        (ramp_up_s + flat_s, total_s, "#ffd6d6", "ramp down")]

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
        ax_a.set_xlabel("Time since SHOT:STARt (s)")
        fig.suptitle(
            title_top + f"\nRampUp={ramp_up_s:.2f}s  FlatTop={flat_s:.2f}s  "
            f"RampDown={ramp_down_s:.2f}s  Demand={demand_a:.0f}A  Total={total_s:.2f}s",
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
    metas: {channel: meta} -- ramp_up_time_s/flat_top_time_s/
    ramp_down_time_s are expected to be the SAME across every entry
    (one shared shot profile); demand_current_a/kp/ki/kd/loop_mode are
    per-channel. Saves one PNG to out_path. Returns out_path."""
    if not HAVE_MPL:
        raise WhamError("matplotlib not installed -- run: pip install matplotlib")

    channels = sorted(channel_rows)
    if not channels:
        raise WhamError("no channel data to plot")

    any_meta = metas[channels[0]]
    ramp_up_s = any_meta.get("ramp_up_time_s")
    flat_s = any_meta.get("flat_top_time_s")
    ramp_down_s = any_meta.get("ramp_down_time_s")
    have_profile = ramp_up_s is not None and flat_s is not None and ramp_down_s is not None
    phase_bounds = []
    if have_profile:
        total_s = ramp_up_s + flat_s + ramp_down_s
        phase_bounds = [(0, ramp_up_s, "#ffe8b3", "ramp up"),
                        (ramp_up_s, ramp_up_s + flat_s, "#c9f2c7", "flat top"),
                        (ramp_up_s + flat_s, total_s, "#ffd6d6", "ramp down")]

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
        subtitle = f"{channel_label(ch, meta.get('nickname'))}  {loop_mode}-loop"
        if demand_a is not None:
            subtitle += f"  demand={demand_a:g}A"
        # Gains -- direct request, 2026-09-22: always visible on every
        # channel-comparison plot, same as generate_plot()'s own
        # single-channel title already does, so it's never necessary to
        # go dig up a shot's meta.json just to know what was tested.
        if meta.get("kp") is not None:
            subtitle += f"  Kp={meta['kp']:g} Ki={meta['ki']:g} Kd={meta['kd']:g}"
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
        title += (f"   |   RampUp={ramp_up_s:.2f}s FlatTop={flat_s:.2f}s "
                   f"RampDown={ramp_down_s:.2f}s Total={total_s:.2f}s")
    fig.suptitle(title, fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.94))

    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    return out_path


def _fft_spectrum(x, fs):
    """Hann-windowed amplitude spectrum of x (already-uniform-rate
    samples: x[i] sampled at times i/fs). Coherent-gain-corrected so
    the returned magnitude is in the same units as x (Hz) rather than
    an arbitrary FFT-bin scale -- same method used for the 2026-09-22
    gain-smoothness-sweep FFT investigation (see
    closed-loop-output-noise-vs-integrator-action). Returns
    (freqs, mag); DC (freqs[0]==0) is included but every caller here
    ignores it via min_hz in _fft_top_peaks()/its own xlim."""
    x = np.asarray(x, dtype=float)
    n = len(x)
    x = x - np.mean(x)
    win = np.hanning(n)
    coherent_gain = np.sum(win) / 2.0
    mag = np.abs(np.fft.rfft(x * win)) / coherent_gain
    freqs = np.fft.rfftfreq(n, d=1.0 / fs)
    return freqs, mag


def _fft_top_peaks(freqs, mag, n=3, min_hz=0.3, min_separation_hz=1.0):
    """Top `n` local peaks in `mag` at or above `min_hz`, each at least
    `min_separation_hz` apart from every peak already picked (so one
    broad spectral lobe doesn't fill the whole list with adjacent
    bins)."""
    idx = np.where(freqs >= min_hz)[0]
    if len(idx) == 0:
        return []
    order = idx[np.argsort(mag[idx])[::-1]]
    peaks, used = [], []
    for i in order:
        if any(abs(freqs[i] - u) < min_separation_hz for u in used):
            continue
        peaks.append((freqs[i], mag[i]))
        used.append(freqs[i])
        if len(peaks) >= n:
            break
    return peaks


def generate_output_fft_plot(channel_rows, metas, out_path):
    """One figure, 4 rows (channel 1-4, ALWAYS -- an idle/unlogged
    channel gets a 'no data' placeholder row rather than being omitted,
    same convention generate_plot()'s DSLogic sibling uses) x 2
    columns: left = that channel's OUTPUT (commanded/drive) AND
    MEASURED (feedback) traces in the time domain, right = each of
    those same traces' own amplitude spectrum (_fft_spectrum(), top
    peaks annotated, same color-coding as the time-domain column).
    Direct request, 2026-09-22, to make the same FFT analysis used for
    the 2026-09-22 gain-smoothness-sweep investigation (see this
    project's closed-loop-output-noise-vs-integrator-action finding) a
    routine, automatic part of firing a shot instead of a one-off
    scratch script -- extended the same day to plot measured alongside
    output (not output alone) in both columns, per direct request, so
    it's directly visible how much of the drive-side noise survives
    into the physically-filtered feedback vs. how much is purely a
    commanded-signal artifact.

    Always uses the FULL logged span (direct request, 2026-09-22 --
    the earlier cut of this auto-trimmed to a flat-top-only window when
    profile timing was known, same convention as the gain-smoothness
    sweep's own verdict logic; the operator wants the entire shot,
    ramp included, not just a snippet). channel_rows/metas: as
    generate_multi_channel_plot() (a channel may be absent from either
    dict -- shown as 'no data'). Returns out_path."""
    if not HAVE_MPL:
        raise WhamError("matplotlib not installed -- run: pip install matplotlib")
    if not HAVE_NUMPY:
        raise WhamError("numpy not installed -- run: pip install numpy")

    fig, axes = plt.subplots(4, 2, figsize=(13, 11))
    any_meta = next(iter(metas.values()), {})
    idn = any_meta.get("idn", "")
    ts = any_meta.get("timestamp", "")

    # Same output/measured color convention as generate_plot()/
    # generate_multi_channel_plot() -- output blue, measured orange --
    # so a panel here reads the same way as every other plot in this file.
    SERIES = (("output_hz", "Output", "#2a78d6"), ("measured_hz", "Measured", "#eb6834"))

    for row, ch in enumerate(range(1, 5)):
        ax_t, ax_f = axes[row][0], axes[row][1]
        rows = channel_rows.get(ch)
        meta = metas.get(ch, {})
        if not rows:
            for ax in (ax_t, ax_f):
                ax.text(0.5, 0.5, f"ch{ch}: no data", ha="center", va="center",
                         transform=ax.transAxes)
                ax.set_xticks([])
                ax.set_yticks([])
            ax_t.set_ylabel(f"Ch{ch}", fontsize=10)
            continue

        t = np.array([r["t_s"] for r in rows])
        dt = np.median(np.diff(t)) if len(t) > 1 else None
        fs = (1.0 / dt) if dt else None

        label = channel_label(ch, meta.get("nickname"))
        # Gains -- direct request, 2026-09-22: always visible here too
        # (both the time-domain and FFT titles below reuse `label`) --
        # this is exactly the plot used to compare successive `refire`s
        # at different gains, so knowing which is which at a glance
        # matters more here than almost anywhere else in this file.
        if meta.get("kp") is not None:
            label += f" (Kp={meta['kp']:g} Ki={meta['ki']:g} Kd={meta['kd']:g})"
        for key, series_label, color in SERIES:
            vals = np.array([r[key] for r in rows], dtype=float)
            ax_t.plot(t, vals, lw=1.0, color=color,
                      label=series_label if row == 0 else None)
        ax_t.set_ylabel("Hz", fontsize=9)
        ax_t.set_title(f"{label} -- output/measured, full shot ({t[-1] - t[0]:.2f}s)",
                        fontsize=9, loc="left")
        ax_t.grid(True, alpha=0.3)
        ax_t.tick_params(labelsize=8)
        if row == 0:
            ax_t.legend(loc="upper right", fontsize=7, ncol=2)

        if fs is None or len(t) < 8:
            ax_f.text(0.5, 0.5, "too few samples for FFT", ha="center", va="center",
                       transform=ax_f.transAxes)
            ax_f.set_xticks([])
            ax_f.set_yticks([])
            continue

        for key, series_label, color in SERIES:
            vals = np.array([r[key] for r in rows], dtype=float)
            freqs, mag = _fft_spectrum(vals, fs)
            ax_f.plot(freqs, mag, lw=1.0, color=color,
                      label=series_label if row == 0 else None)
            peak_offset = (3, 3) if key == "output_hz" else (3, -10)
            for f, m in _fft_top_peaks(freqs, mag, n=3):
                ax_f.annotate(f"{f:.1f}Hz", (f, m), textcoords="offset points",
                               xytext=peak_offset, fontsize=7, color=color)
        ax_f.set_xlim(0, fs / 2)
        ax_f.set_ylabel("Amplitude (Hz)", fontsize=9)
        ax_f.set_title(f"{label} -- output/measured spectrum (fs={fs:.1f}Hz, "
                        f"Nyquist={fs / 2:.1f}Hz)", fontsize=9, loc="left")
        ax_f.grid(True, alpha=0.3)
        ax_f.tick_params(labelsize=8)
        if row == 0:
            ax_f.legend(loc="upper right", fontsize=7, ncol=2)

    axes[-1][0].set_xlabel("Time (s)")
    axes[-1][1].set_xlabel("Frequency (Hz)")
    fig.suptitle(f"WHAM-XREX-PFMG474 -- output + measured, time domain + FFT, "
                 f"all channels -- {idn}\n{ts}", fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.95))

    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    return out_path


# --------------------------------------------------------------------------
# The console itself
# --------------------------------------------------------------------------

DANGEROUS_EXACT = {"FIRE", "BOOT"}


def _is_dangerous(command):
    """Best-effort heuristic, NOT a clone of cmd_parser.c's scpi_match()
    -- see this function's own limitation note. Takes the FULL command
    line (not just its first token) because one of its checks --
    channel-enable, below -- needs to look at an argument, not just
    the mnemonic. Catches:
      - FIRE, BOOT (exact).
      - SOURce:RUN / SHOT:STARt, in any valid SCPI short/long
        form (the mandatory short forms for "STARt"/"RUN" are "STAR"/
        "RUN", so matching the final colon-segment's prefix covers
        every valid on-wire spelling of those leaves regardless of how
        the SOURce root is itself abbreviated).
      - SOURce:ENAble <ch> <1|nonzero> -- turning a channel's
        output ON. This is real, immediate hardware effect if a loop
        is already running (PID_SetChannelEnable() drives
        HRTIM1_SetChannelOutputEnable() directly, unconditionally --
        see pid.c), so it belongs in the same danger class as
        FIRE/SOURce:RUN even though it doesn't itself start the state
        machine. Turning a channel OFF is deliberately never gated
        (always the safe direction, matching do_enable()'s own
        docstring) -- a malformed/unparseable enable value is treated
        as dangerous, not silently waved through.

    THIS is the single source of truth for "dangerous" on the wire --
    both this console's own raw passthrough/wrapper commands (via
    _query_print(), below) AND wham_llm_console.py's LLM-proposed
    actions consult it, specifically so an LLM front end can't bypass
    a gate just by phrasing a command differently than a human would
    (e.g. raw `SOURce:ENAble` instead of the `enable` wrapper).

    KNOWN LIMITATION: this is a small, explicit heuristic, not a full
    reimplementation of the firmware's own short/long-form matcher --
    it will not catch every conceivable future dangerous command
    automatically. If a new command is added that begins real output
    (matching FIRE/SOURce:RUN's danger level), add it here explicitly
    -- see "Adding a new console command" below."""
    command = command.strip()
    first_token = command.split(None, 1)[0] if command else ""
    t = first_token.upper().lstrip(":")
    if t in DANGEROUS_EXACT:
        return True
    segs = t.split(":")
    root = segs[0]
    is_sour = root.startswith("SOUR")
    is_shot = root.startswith("SHOT")
    if len(segs) < 2 or not (is_sour or is_shot):
        return False
    if is_shot and segs[-1].startswith("STAR"):
        return True
    if is_sour and segs[-1].startswith("RUN"):
        return True
    if is_sour and segs[-1].startswith("ENA"):
        args = command.split()[1:]
        if len(args) < 2:
            return True  # malformed -- fail closed, don't wave it through
        try:
            return int(args[1]) != 0
        except ValueError:
            return True  # unparseable value -- fail closed
    return False


def _is_profile_start(command):
    """True iff `command` is specifically SHOT:STARt, in any
    valid SCPI short/long form -- the same STARt-suffix matching
    _is_dangerous() uses, but narrowed to the SHOT root so
    it does NOT also match plain SOURce:RUN (the open-ended, non-
    profiled continuous run driven by `start`/`stop`, which has no
    fixed duration to watch for and isn't what this project calls a
    "shot"). Also deliberately does NOT match the legacy FIRE/TABLE:*
    playback path (a separate, older mechanism -- see pfm.h's own
    comment on PFM_TABLE_SIZE) since that doesn't produce a
    LOG:DATA?-compatible log to auto-plot.

    Used by default() to auto-watch/auto-plot after a raw
    SHOT:STARt typed directly, matching what `shot`/`refire`
    already do -- see WhamConsole._auto_plot_after_shot()."""
    command = command.strip()
    first_token = command.split(None, 1)[0] if command else ""
    t = first_token.upper().lstrip(":")
    segs = t.split(":")
    return (len(segs) >= 2 and segs[0].startswith("SHOT")
            and segs[-1].startswith("STAR"))


def _is_calibration_set(command):
    """True iff `command` is a CONFig:TURNONHz/MAXFREQHz/MAXCURRent/
    PIDRate SET (the plain form, not its `?` query) -- the four live-
    configurable values sync_calibration() caches into
    PFM_TURNON_FREQ_HZ/PFM_MAX_FREQ_HZ/PFM_MAX_CURRENT_A/
    PID_LOOP_RATE_HZ_ASSUMED. Used by default() to re-sync right after
    a raw command that could have just changed one of them, the same
    way _is_profile_start() triggers auto-plot after a raw SHOT:STARt
    -- so a plot or a suggested decim value generated later in the
    same session never uses a stale calibration just because the
    operator typed the SCPI directly instead of through a dedicated
    wrapper command (there isn't one yet)."""
    command = command.strip()
    first_token = command.split(None, 1)[0] if command else ""
    t = first_token.upper().lstrip(":")
    if t.endswith("?"):
        return False
    segs = t.split(":")
    if len(segs) != 2 or not segs[0].startswith("CONF"):
        return False
    return (segs[1].startswith("TURNONH")
            or segs[1].startswith("MAXFREQH")
            or segs[1].startswith("MAXCURR")
            or segs[1].startswith("PIDR"))


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
        # PID:LOOPMODE?/SHOT:TIMing?/SHOT:CURRent? readback
        # on every connect and on demand via `config` (see
        # _refresh_config()), and kept up to date incrementally by the
        # gains/loopmode/shot wrapper commands as they send new values.
        # Used to pre-fill `shot` wizard prompts with real current
        # values rather than guessed ones.
        self.channel_config = {}   # {ch: {"kp":, "ki":, "kd":, "loop_mode":, "demand_a":}}
        self.profile_timing = None  # {"ramp_up_s":, "flat_s":, "ramp_down_s":}
        self.last_log_channel = None
        self.last_log_all = False   # True if `log all`/the shot wizard last armed
                                     # LOG:ARM 0 (every channel at once)
        self.last_log_max_samples = None
        self.last_log_decim = None  # both remembered (not just the target channel)
                                     # so `refire` can re-arm identically -- see its
                                     # own doc comment for why this matters
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
            telem_reply = self.link.query("SYS:TELEM?")
            if not is_err(telem_reply):
                # Telemetry data-contract schema (docs/telemetry.md). ERR
                # here just means an older firmware without SYS:TELEM? --
                # not a problem, this console doesn't hard-require it.
                print(f"SYS:TELEM? -> schema {telem_reply.split()[1]}")
        except (WhamError, ValueError, IndexError) as exc:
            print(f"[warn] connected, but couldn't query *IDN?/CONFig:CHANnels?: {exc}")
            return
        if sync_calibration(self.link):
            print(f"CONFig:TURNONHz?/MAXFREQHz?/MAXCURRent?/PIDRate? -> calibration synced "
                  f"(turnon={PFM_TURNON_FREQ_HZ:.0f} Hz, max={PFM_MAX_FREQ_HZ:.0f} Hz, "
                  f"{PFM_MAX_CURRENT_A:.0f} A on ch1, loop rate={PID_LOOP_RATE_HZ_ASSUMED} Hz)")
            if HAVE_DSLOGIC_MODULE:
                # Pass the same already-fetched values through rather than
                # having dslogic_shot_capture.py re-query the device (or
                # import wham_console itself, which it deliberately avoids
                # -- see set_calibration()'s own comment) -- closes the
                # same staleness gap there too, added 2026-09-23.
                dslogic_shot_capture.set_calibration(
                    PFM_TURNON_FREQ_HZ, PFM_MAX_FREQ_HZ, PFM_MAX_CURRENT_A)
        else:
            print("[warn] couldn't sync calibration from the device -- "
                  f"using the last-known values (turnon={PFM_TURNON_FREQ_HZ:.0f} Hz, "
                  f"max={PFM_MAX_FREQ_HZ:.0f} Hz, {PFM_MAX_CURRENT_A:.0f} A, "
                  f"loop rate={PID_LOOP_RATE_HZ_ASSUMED} Hz)")
        self._refresh_config()

    # -- live device readback for gains/loopmode/profile timing/current --
    # (added 2026-09-10, once PID:GAINS?/PID:LOOPMODE?/SHOT:TIMing?/
    # SHOT:CURRent? existed on the firmware side -- see docs/
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
        reply = self.link.query(f"SOURce:ENAble? {ch}")
        if is_err(reply):
            return None
        return reply.split()[1] == "1"

    def _query_profile_current(self, ch):
        reply = self.link.query(f"SHOT:CURRent? {ch}")
        if is_err(reply):
            return None
        return float(reply.split()[1])

    def _query_profile_timing(self):
        reply = self.link.query("SHOT:TIMing?")
        if is_err(reply):
            return None  # not set yet on the device -- a real, distinct state
        parts = reply.split()
        return dict(ramp_up_s=float(parts[1]), flat_s=float(parts[2]), ramp_down_s=float(parts[3]))

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
        if _is_dangerous(command):
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
        reply = self._query_print(line)
        # A raw SHOT:STARt typed directly (rather than through
        # `shot`/`refire`) gets the same auto-watch/auto-plot treatment
        # -- direct request, 2026-09-22, so "manually fire a shot" works
        # whichever way it's actually typed. See _is_profile_start()'s
        # own doc comment for why this deliberately does NOT also match
        # plain SOURce:RUN or the legacy FIRE/TABLE:* path.
        if reply is not None and not is_err(reply) and _is_profile_start(line):
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            self._watch_shot()
            self._auto_plot_after_shot(ts=ts)
        if reply is not None and not is_err(reply) and _is_calibration_set(line):
            if sync_calibration(self.link) and HAVE_DSLOGIC_MODULE:
                dslogic_shot_capture.set_calibration(
                    PFM_TURNON_FREQ_HZ, PFM_MAX_FREQ_HZ, PFM_MAX_CURRENT_A)

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
        reply = self.link.query(f"SOURce:STATus? {ch}")
        if is_err(reply):
            return None
        parts = reply.split()
        # OK <running> <setpointHz> <measuredHz> <outputHz>
        return dict(running=int(parts[1]), setpoint=int(parts[2]),
                    measured=int(parts[3]), output=int(parts[4]))

    def _get_nickname(self, channel):
        """Fetches this channel's CHANnel:NICKname? fresh from the
        device -- "-" (the wire's "no nickname set" sentinel) comes back
        as None. Queried fresh each time rather than cached client-side,
        matching *IDN? in _shot_meta() -- avoids ever showing a stale
        nickname if it was changed mid-session."""
        try:
            reply = self.link.query(f"CHANnel:NICKname? {channel}", timeout=1.0)
        except WhamError:
            return None
        if is_err(reply):
            return None
        parts = reply.split()
        if len(parts) < 2 or parts[1] == "-":
            return None
        return parts[1]

    def do_status(self, arg):
        """status [channel]  -- top-level state (STATE?) plus live
        SOURce:STATus? for one channel, or every channel if none given
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

    def do_events(self, arg):
        """events [0|1|on|off]  -- the unsolicited !EVT telemetry stream
        (firmware Phase 1 of docs/telemetry.md). No argument: show the
        current SYS:EVENT gate (ON/OFF) and print any events that have
        arrived since the last command. With an argument, send
        SYS:EVENT 1 (on) or SYS:EVENT 0 (off). (Same thing is reachable
        by typing the raw SCPI `SYS:EVENT 0` directly.)"""
        if not self._require_link():
            return
        a = arg.strip().lower()
        if a in ("1", "on", "true", "0", "off", "false"):
            bit = "1" if a in ("1", "on", "true") else "0"
            self._query_print(f"SYS:EVENT {bit}")
            return
        self.link.drain_events()
        reply = self.link.query("SYS:EVENT?")
        if is_err(reply):
            print(explain_err(reply))
            return
        state = "ON" if reply.split()[-1] == "1" else "OFF"
        print(f"events: {state}  (!EVT stream {'enabled' if state == 'ON' else 'disabled'})")

    def do_evlog(self, arg):
        """evlog  -- dump the flight recorder (SYS:EVLOG?, docs/telemetry.md
        Phase 4): the retained state/fault event history since boot, oldest
        first, as a header "OK <n>" then n !EVT lines. Read-only."""
        if not self._require_link():
            return
        header, events = self.link.query_evlog(timeout=3.0)
        if header.startswith("ERR"):
            print(explain_err(header))
            return
        print(f"evlog: {header}  ({len(events)} events)")
        for line in events:
            print(f"  {line}")

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
        (PID:GAINS?/PID:LOOPMODE?/SHOT:TIMing?/SHOT:CURRent?,
        added 2026-09-10) -- always current, including after a
        reconnect or after another tool/operator changed something.
        Also refreshes this console's own local cache (used to
        pre-fill `shot` wizard defaults) as a side effect."""
        if not self._require_link():
            return
        self._refresh_config()
        if self.profile_timing:
            print(f"Profile timing: ramp up={self.profile_timing['ramp_up_s']:g}s "
                  f"flat-top={self.profile_timing['flat_s']:g}s "
                  f"ramp down={self.profile_timing['ramp_down_s']:g}s")
        else:
            print("Profile timing: not set on the device (SHOT:TIMing never sent)")
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

    def do_diag(self, arg):
        """diag  -- one consolidated debug snapshot: *IDN?, STATE?,
        FAULT?, GDS? (raw 12 gate-driver pins), QSPI:ID?, PFMIN:STATus?
        (PFM_Input capture counts, if a capture was ever armed), and
        every channel's config (enable/nickname/loop-mode/gains/demand)
        PLUS live SOURce:STATus? in one table -- everything needed to
        characterize "what's going on right now" in ONE action instead
        of stitching together a dozen queries by hand. Entirely
        read-only -- never asks to confirm anything, safe to run at any
        time including mid-fault or mid-shot."""
        if not self._require_link():
            return
        n = self.num_channels or 4
        print(f"idn:    {self.link.query('*IDN?', timeout=1.0)}")
        state_reply = self.link.query("STATE?")
        print(f"state:  {state_reply[3:] if state_reply.startswith('OK ') else state_reply}")
        print(f"fault:  {self.link.query('FAULT?')}")
        for label, cmd in (("gds", "GDS?"), ("qspi", "QSPI:ID?"), ("pfmin", "PFMIN:STATus?")):
            try:
                print(f"{label}:  {self.link.query(cmd)}")
            except WhamError as exc:
                print(f"{label}:  [error] {exc}")

        self._refresh_config()
        if self.profile_timing:
            print(f"profile timing: ramp up={self.profile_timing['ramp_up_s']:g}s "
                  f"flat-top={self.profile_timing['flat_s']:g}s "
                  f"ramp down={self.profile_timing['ramp_down_s']:g}s")
        else:
            print("profile timing: not set (SHOT:TIMing never sent)")

        print(f"{'Ch':>3} {'Enabled':>7} {'Loop':>6} {'Demand(A)':>9} "
              f"{'Kp':>6} {'Ki':>6} {'Kd':>6} {'Running':>7} {'Setpt':>7} "
              f"{'Meas':>7} {'Out':>7}  Nickname")
        for ch in range(1, n + 1):
            c = self.channel_config.get(ch, {})
            nickname = self._get_nickname(ch) or ""
            try:
                row = self._status_row(ch)
            except WhamError:
                row = None
            enabled = c.get("enabled")
            enabled_str = "no" if enabled is False else ("yes" if enabled else "?")
            demand = f"{c['demand_a']:g}" if c.get("demand_a") is not None else "?"
            kp = f"{c['kp']:g}" if c.get("kp") is not None else "?"
            ki = f"{c['ki']:g}" if c.get("ki") is not None else "?"
            kd = f"{c['kd']:g}" if c.get("kd") is not None else "?"
            if row:
                running, sp, ms, op = ("yes" if row["running"] else "no"), row["setpoint"], row["measured"], row["output"]
            else:
                running = sp = ms = op = "?"
            print(f"{ch:>3} {enabled_str:>7} {c.get('loop_mode', '?'):>6} {demand:>9} "
                  f"{kp:>6} {ki:>6} {kd:>6} {running:>7} {sp!s:>7} {ms!s:>7} {op!s:>7}  {nickname}")

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
        """enable <ch> <on|off>  -- wrapper for SOURce:ENAble: a
        genuine "this channel outputs nothing at all" switch, distinct
        from `loopmode` (open-loop still drives a real, uncorrected PFM
        waveform) or a 0A demand current (still drives a real PFM
        waveform, at the turn-on floor). Takes effect on the NEXT
        `start`/`shot` if the loop isn't running yet; takes effect
        IMMEDIATELY, live, if it is -- turning a channel back on asks
        to confirm first (via _query_print()'s shared _is_dangerous()
        gate, same as `start`/`FIRE` -- NOT a bespoke confirm here
        anymore, so an operator or LLM front end can't get a different
        answer by phrasing this as raw SCPI instead of this wrapper);
        turning one off never asks (always the safe direction --
        _is_dangerous() only matches the ON case)."""
        if not self._require_link():
            return
        parts = shlex.split(arg)
        if len(parts) != 2 or parts[1].lower() not in ("on", "off"):
            print("usage: enable <ch> <on|off>")
            return
        ch, state = parts
        turning_on = state.lower() == "on"
        reply = self._query_print(f"SOURce:ENAble {ch} {1 if turning_on else 0}")
        if reply and not is_err(reply):
            self.channel_config.setdefault(int(ch), {})["enabled"] = turning_on

    def do_nickname(self, arg):
        """nickname <ch> [name]  -- wrapper for CHANnel:NICKname:
        assigns a purely cosmetic label to a channel (e.g. `nickname 1
        TINKYWINKY`) -- no effect on control behavior. The bare Ch<N>
        label is never dropped, on the wire or in plots -- a nickname
        is always shown ALONGSIDE it (`Ch1 (TINKYWINKY)`), never instead
        of it. With no `name`, queries and prints the channel's current
        nickname (CHANnel:NICKname?) instead of setting one."""
        if not self._require_link():
            return
        parts = shlex.split(arg)
        if len(parts) == 1:
            self._query_print(f"CHANnel:NICKname? {parts[0]}")
            return
        if len(parts) != 2:
            print("usage: nickname <ch> [name]  (no name = query the current one)")
            return
        self._query_print(f"CHANnel:NICKname {parts[0]} {parts[1]}")

    def do_setpoint(self, arg):
        """setpoint <ch> <hz>  -- wrapper for SOURce:SETpoint."""
        if not self._require_link():
            return
        parts = shlex.split(arg)
        if len(parts) != 2:
            print("usage: setpoint <ch> <hz>")
            return
        self._query_print(f"SOURce:SETpoint {parts[0]} {parts[1]}")

    def do_ramp(self, arg):
        """ramp <ch> <startHz> <endHz> <durationMs>  -- wrapper for SOURce:RAMP."""
        if not self._require_link():
            return
        parts = shlex.split(arg)
        if len(parts) != 4:
            print("usage: ramp <ch> <startHz> <endHz> <durationMs>")
            return
        self._query_print("SOURce:RAMP " + " ".join(parts))

    def do_timing(self, arg):
        """timing [<rampUpS> <flatS> <rampDownS>]  -- wrapper for
        SHOT:TIMing, the shared shot clock every enabled
        channel's profile uses (see `shot`/SHOT:STARt). No
        effect on any channel's setpoint by itself -- just how long
        the up-ramp/flat-top/down-ramp phases last once a profile
        actually starts. Ramp up and ramp down are independently
        configurable (added 2026-09-22, direct request -- was
        <rampS> <flatS>, one shared ramp duration for both directions,
        before this). With no arguments, queries the current setting
        (SHOT:TIMing?) instead of setting one -- a profile
        can't start at all until this has been set at least once this
        session (ERR 12), so this is worth being able to check as easily
        as set. Added 2026-09-14 -- SHOT:TIMing/CURRent had no
        wrapper before this, unlike every other setting; a real LLM-
        console session invented plausible but wrong syntax for it twice
        in a row (`profile timing 1 1 2`, `pid profile timing 1 2`) rather
        than falling back to the exact raw-SCPI form its own system prompt
        had -- see docs/changelog.txt's matching entry. This wrapper
        exists so a natural word-based guess actually works instead of
        requiring exact colon-separated SCPI recall, same reasoning as
        every other wrapper in this file."""
        if not self._require_link():
            return
        parts = shlex.split(arg)
        if not parts:
            self._query_print("SHOT:TIMing?")
            return
        if len(parts) != 3:
            print("usage: timing <rampUpSeconds> <flatTopSeconds> <rampDownSeconds>  "
                  "(no args = query)")
            return
        reply = self._query_print("SHOT:TIMing " + " ".join(parts))
        if reply and not is_err(reply):
            try:
                self.profile_timing = dict(ramp_up_s=float(parts[0]), flat_s=float(parts[1]),
                                            ramp_down_s=float(parts[2]))
            except ValueError:
                pass

    def do_demand(self, arg):
        """demand <ch> [<amps>]  -- wrapper for SHOT:CURRent: this
        channel's target current for the NEXT profiled shot (SHOT:
        STARt) -- distinct from `setpoint`, which is an immediate raw Hz
        value with no profile/ramp involved. With no `amps`, queries the
        current value (SHOT:CURRent?) instead of setting one. Added
        2026-09-14 alongside `timing` -- see that command's own doc
        comment for why (SHOT:CURRent had the same no-wrapper gap)."""
        if not self._require_link():
            return
        parts = shlex.split(arg)
        if len(parts) == 1:
            self._query_print(f"SHOT:CURRent? {parts[0]}")
            return
        if len(parts) != 2:
            print("usage: demand <ch> [<amps>]  (no amps = query the current value)")
            return
        reply = self._query_print(f"SHOT:CURRent {parts[0]} {parts[1]}")
        if reply and not is_err(reply):
            try:
                self.channel_config.setdefault(int(parts[0]), {})["demand_a"] = float(parts[1])
            except ValueError:
                pass

    def do_start(self, arg):
        """start  -- wrapper for SOURce:RUN (asks to confirm first)."""
        if self._require_link():
            self._query_print("SOURce:RUN")

    def do_stop(self, arg):
        """stop  -- wrapper for SOURce:STOP (always safe, no confirmation)."""
        if self._require_link():
            reply = self.link.query("SOURce:STOP")
            print(explain_err(reply) if is_err(reply) else reply)

    def do_log(self, arg):
        """log <ch|all> <maxSamples> <decim>  -- wrapper for LOG:ARM
        (arms waveform logging). `all` arms every channel at once, from
        the SAME real ticks (LOG:ARM 0 ...) -- for a genuine
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
        reply = self._query_print(f"LOG:ARM {wire_ch} {max_samples} {decim}")
        if reply and not is_err(reply):
            self.last_log_all = all_channels
            self.last_log_channel = None if all_channels else int(ch_arg)
            self.last_log_max_samples = int(max_samples)
            self.last_log_decim = int(decim)

    # -- plotting ---------------------------------------------------------

    def _fetch_log(self, channel):
        """Runs LOG:DATA? <channel>, returns (rows, rate_hz, count)
        or raises WhamError. rows: list of dicts with t_s/setpoint_hz/
        measured_hz/output_hz."""
        reply = self.link.query(f"LOG:DATA? {channel}", timeout=LOGDATA_TIMEOUT)
        if is_err(reply):
            raise WhamError(explain_err(reply))
        parts = reply.split()
        if len(parts) < 2 or parts[0] != "OK":
            raise WhamError(f"unexpected LOG:DATA? reply: {reply!r}")
        count = int(parts[1])
        rate_hz = int(parts[2])  # always present, even for count=0 -- see cmd_pid_logdata()
        vals = list(map(int, parts[3:3 + count * 3]))
        if len(vals) != count * 3:
            raise WhamError(f"LOG:DATA? claimed {count} samples but only "
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
            nickname=self._get_nickname(channel),
            timestamp=datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            idn=idn,
            kp=cfg.get("kp"), ki=cfg.get("ki"), kd=cfg.get("kd"),
            loop_mode=cfg.get("loop_mode"),
        )
        if self.profile_timing:
            meta["ramp_up_time_s"] = self.profile_timing["ramp_up_s"]
            meta["flat_top_time_s"] = self.profile_timing["flat_s"]
            meta["ramp_down_time_s"] = self.profile_timing["ramp_down_s"]
            meta["demand_current_a"] = cfg.get("demand_a")
        else:
            meta["ramp_up_time_s"] = meta["flat_top_time_s"] = meta["ramp_down_time_s"] = None
            meta["demand_current_a"] = None
        return meta

    def _shot_dir(self, ts):
        """Returns (creating if needed) shots/<ts>/ -- every file a
        single shot/fetch produces (CSV, meta JSON, PNG plots, the FFT
        plot, a DSLogic cross-check, a written report) lands in ONE
        subdirectory together, named by that shot's own timestamp,
        instead of loose as <ts>_ch1.csv/<ts>_ch1.png/... directly in
        shots/ (direct request, 2026-09-22, to keep shots/ organized as
        the file count grows). Matches run_simulator_validation.py's
        own new_scenario_dir() convention -- filenames inside no longer
        need their own ts prefix, since the directory name already
        carries it (ch1.csv, allch.png, fft.png, ... instead of
        <ts>_ch1.csv etc.)."""
        d = os.path.join(SHOTS_DIR, ts)
        os.makedirs(d, exist_ok=True)
        return d

    def _save_and_plot(self, channel, rows, rate_hz, count, ts=None):
        if count == 0:
            print("Log is empty (LOG:DATA? returned 0 samples) -- nothing to plot. "
                  "Did you arm logging (`log`) before the shot/test ran?")
            return
        ts = ts or datetime.now().strftime("%Y%m%d_%H%M%S")
        base = os.path.join(self._shot_dir(ts), f"ch{channel}")
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
        """Fetches LOG:DATA? for every channel (all-channels mode
        must be currently/previously armed -- PID_ArmLogAll()), saves
        one combined wide-format CSV + one metadata JSON + ONE PNG
        (generate_multi_channel_plot()) showing every channel on a
        shared time axis. Used by `shot` (right after a shot with `all`
        logging), `plot all`, and `report all` (which reuses the SAME
        fetch this makes -- see its own return value below -- rather
        than hitting LOG:DATA? a second time over the wire).

        Returns (channel_rows, metas, rate_hz, count), or None if there
        was nothing to report (empty log / never armed) -- callers that
        only want the CSV/JSON/PNG side effect (do_plot, do_shot) can
        ignore the return value entirely, as they did before this
        returned anything."""
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
            return None

        metas = {ch: self._shot_meta(ch) for ch in channel_rows}
        for ch, meta in metas.items():
            meta["rate_hz"] = rate_hz
            meta["log_count"] = count

        ts = ts or datetime.now().strftime("%Y%m%d_%H%M%S")
        base = os.path.join(self._shot_dir(ts), "allch")

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
        else:
            try:
                png = generate_multi_channel_plot(channel_rows, metas, base + ".png")
                print(f"Saved {png}")
            except WhamError as exc:
                print(f"[error] plotting: {exc}")
        return channel_rows, metas, rate_hz, count

    def _watch_shot(self, watch_ch=None, total_s=None):
        """Polls STATus? on `watch_ch` and prints a live line until the
        shot finishes (running drops to 0) or a generous timeout
        elapses -- the same watch loop `shot` has always used, factored
        out (2026-09-22) so `refire` and a raw SHOT:STARt typed
        directly (see default()) get it too, not just the guided
        wizard. `watch_ch` defaults to whatever `log`/`shot` last
        armed, else the first enabled channel, else channel 1.
        `total_s` defaults to ramp_up_s+flat_s+ramp_down_s from
        self.profile_timing if known (set by `shot`/`timing`, or a
        live SHOT:TIMing? readback -- see _refresh_config());
        if that's unknown too, this just watches for a fixed generous
        timeout instead of a precise one. Ctrl-C stops watching
        without stopping the shot itself."""
        if watch_ch is None:
            watch_ch = (self.last_log_channel
                        or next((ch for ch, c in self.channel_config.items()
                                 if c.get("enabled")), None)
                        or 1)
        if total_s is None and self.profile_timing:
            total_s = (self.profile_timing["ramp_up_s"] + self.profile_timing["flat_s"]
                       + self.profile_timing["ramp_down_s"])
        timeout_s = (total_s + 2.0) if total_s is not None else 60.0
        print(f"Shot running -- watching channel {watch_ch} (Ctrl-C to stop watching, "
              "shot keeps running)...")
        t0 = time.time()
        try:
            while time.time() - t0 < timeout_s:
                row = self._status_row(watch_ch)
                if row is None:
                    break
                print(f"  t+{time.time() - t0:5.2f}s  running={row['running']}  "
                      f"setpoint={row['setpoint']}  measured={row['measured']}  "
                      f"output={row['output']}")
                if row["running"] == 0 and time.time() - t0 > 0.5:
                    break
                time.sleep(0.3)
        except KeyboardInterrupt:
            print("\n(stopped watching -- shot itself keeps running on the device)")

    def _auto_plot_after_shot(self, ts=None):
        """Auto-fetch + plot right after a shot finishes -- shared by
        `shot`, `refire`, and a raw SHOT:STARt typed directly
        (see default()). Honors whatever `log`/`shot` last armed (all
        channels vs. a single one) exactly like `plot` does. Direct
        request, 2026-09-22: ALSO generates the 4-row (channel 1-4) x
        2-col output/FFT plot (generate_output_fft_plot()) every time
        there's anything to plot at all -- a channel that wasn't logged
        just shows as 'no data' in its own row rather than being
        skipped, so this works whether `all` or a single channel was
        armed."""
        if self.last_log_all:
            print("Fetching all-channel log and plotting...")
            result = self._fetch_save_plot_all(ts=ts)
            if result is None:
                return
            channel_rows, metas, rate_hz, count = result
        elif self.last_log_channel:
            print("Fetching log and plotting...")
            try:
                rows, rate_hz, count = self._fetch_log(self.last_log_channel)
            except WhamError as exc:
                print(f"[error] {exc}")
                return
            self._save_and_plot(self.last_log_channel, rows, rate_hz, count, ts=ts)
            if count == 0:
                return
            channel_rows = {self.last_log_channel: rows}
            metas = {self.last_log_channel: self._shot_meta(self.last_log_channel)}
            for m in metas.values():
                m["rate_hz"] = rate_hz
                m["log_count"] = count
        else:
            print("(no logging armed -- nothing to auto-plot; use `log <ch|all> ...` "
                  "before firing next time)")
            return

        if not HAVE_MPL:
            return  # already warned above, from _fetch_save_plot_all/_save_and_plot
        if not HAVE_NUMPY:
            print("numpy not installed -- skipping output/FFT plot. Run: pip install numpy")
            return
        ts = ts or datetime.now().strftime("%Y%m%d_%H%M%S")
        fft_path = os.path.join(self._shot_dir(ts), "fft.png")
        try:
            png = generate_output_fft_plot(channel_rows, metas, fft_path)
            print(f"Saved {png}")
        except WhamError as exc:
            print(f"[error] FFT plot: {exc}")

    def do_plot(self, arg):
        """plot [channel|all]  -- fetches whatever waveform log is
        currently in the controller (LOG:DATA?) right now and saves
        a CSV + metadata JSON + PNG plot under shots/. With no
        argument, uses whatever `log`/`shot` last armed this session
        (a single channel, or all channels). `all` (or a channel
        number) overrides that -- pass one explicitly if you armed
        logging some other way (e.g. raw `LOG:ARM 0 500 1`)."""
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

    def _print_report_summary_line(self, meta, stats):
        """One compact line per channel -- what `report` prints to the
        terminal (and what an LLM front end actually sees back, given
        wham_llm_console.py's MAX_RESULT_CHARS cap); the full detail is
        only in the saved .md file, not repeated here. Leads with
        output-vs-commanded (see compute_log_stats()'s own doc comment
        for why that, not measured-vs-commanded, is "does the output
        match what it should")."""
        label = channel_label(meta["channel"], meta.get("nickname"))
        if stats is None:
            print(f"  {label}: no log data (logging not armed before this run, "
                  f"or the channel never ran)")
            return
        oe, fe = stats["output_error"], stats["feedback_error"]
        print(f"  {label}: {stats['count']} samples/{stats['duration_s']:.2f}s -- "
              f"output vs commanded RMS {oe['rms']:.0f}Hz ({hz_delta_to_amps(oe['rms']):.2f}A), "
              f"feedback vs commanded RMS {fe['rms']:.0f}Hz ({hz_delta_to_amps(fe['rms']):.2f}A, "
              f"today = loopback), {stats['glitch_count']} glitch(es)")

    def do_report(self, arg):
        """report [channel|all]  -- shot PERFORMANCE analysis: does the
        frequency the controller actually output match what the shot
        profile commanded? Like `plot` (fetches LOG:DATA?, saves CSV
        + metadata JSON + PNG under shots/<ts>/), PLUS a written summary
        saved as shots/<ts>/ch<N>_report.md (or allch_report.md) in
        that same directory with THREE separate error comparisons -- output-vs-commanded
        (the headline "did it output what it should" number, valid
        regardless of loop mode since the commanded setpoint never
        depends on feedback), feedback-vs-commanded (closed-loop
        convergence), and feedback-vs-output (today, a bench-wiring
        self-check -- feedback is currently looped back from this
        controller's own output, NOT an independent Transrex supply, so
        don't read that row as real hardware performance). See
        compute_log_stats()'s own doc comment for the full reasoning,
        and format_channel_report_md() for exactly what's written. No
        invented pass/fail verdict -- this project has no established
        tracking-error spec yet, so the report states the numbers and
        lets the operator/LLM judge them. Also queries STATE?/FAULT? at
        report time, so the report itself shows whether the run ended
        cleanly or in a fault. Only a short summary goes to the
        terminal -- the full detail is in the saved file. Same
        channel-selection rule as `plot` (arg, else whatever `log`/
        `shot` last armed)."""
        if not self._require_link():
            return
        want_all = arg.strip().lower() == "all" or (not arg.strip() and self.last_log_all)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")

        state_reply = self.link.query("STATE?")
        fault_reply = self.link.query("FAULT?")
        header = (f"# WHAM-XREX-PFMG474 run report\n\n"
                  f"- Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n"
                  f"- State at report time: "
                  f"{state_reply[3:] if state_reply.startswith('OK ') else state_reply}\n"
                  f"- Fault at report time: {fault_reply}\n\n")

        if want_all:
            result = self._fetch_save_plot_all(ts=ts)
            if result is None:
                return
            channel_rows, metas, rate_hz, count = result
            stats_by_ch = {ch: compute_log_stats(channel_rows[ch], rate_hz) for ch in channel_rows}
            md = header + "\n".join(format_channel_report_md(metas[ch], stats_by_ch[ch])
                                     for ch in sorted(channel_rows))
            base = os.path.join(self._shot_dir(ts), "allch")
            summary_metas = metas
        else:
            channel = int(arg.strip()) if arg.strip() else self.last_log_channel
            if channel is None:
                print("No channel known -- pass one: report <channel>  (or `report all`, "
                      "or arm logging first with `log <ch|all> ...`)")
                return
            try:
                rows, rate_hz, count = self._fetch_log(channel)
            except WhamError as exc:
                print(f"[error] {exc}")
                return
            self._save_and_plot(channel, rows, rate_hz, count, ts=ts)
            meta = self._shot_meta(channel)
            meta["rate_hz"] = rate_hz
            meta["log_count"] = count
            stats_by_ch = {channel: compute_log_stats(rows, rate_hz)}
            md = header + format_channel_report_md(meta, stats_by_ch[channel])
            base = os.path.join(self._shot_dir(ts), f"ch{channel}")
            summary_metas = {channel: meta}

        report_path = base + "_report.md"
        with open(report_path, "w") as f:
            f.write(md)
        print(f"Saved {report_path}")

        print("--- summary ---")
        for ch in sorted(stats_by_ch):
            self._print_report_summary_line(summary_metas[ch], stats_by_ch[ch])

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
        profile (Ramp Up Time / Flat Top Time / Ramp Down Time --
        independently configurable, added 2026-09-22 -- / per-channel
        enable/disable, Demand Current, loop mode, gains), then watches
        it run and automatically fetches + plots the result (see
        _auto_plot_after_shot() -- also generates the 4-row x 2-col
        output/FFT plot). Logging can target one channel or 'all' --
        'all' arms every channel from the SAME real ticks (LOG:ARM 0
        ...) and produces ONE combined plot with one row per channel, a
        genuine simultaneous comparison rather than N separate runs.
        Re-running `shot` reuses your previous answers as the new
        defaults (just press Enter to repeat a shot unchanged) -- for
        changing ONLY the gains and re-firing without re-answering
        every prompt, see `refire` instead. Ctrl-C at any prompt
        cancels without sending anything.

        If a DreamSourceLab DSLogic is connected (Phase U/V/W/X wired to
        its Ch0/Ch1/Ch2/Ch3, see dslogic_shot_capture.py), this also arms
        a DSLogic capture right before firing and saves an independent
        frequency-vs-firmware-ground-truth cross-check plot -- one panel
        per channel, including disabled/idle ones -- to shots/<ts>/
        dslogic.png alongside the usual CSV/JSON/PNG -- silently
        skipped if no DSLogic is plugged in."""
        if not self._require_link():
            return
        n = self.num_channels or 4
        try:
            print(f"\n--- Shot wizard ({n} channels) --- (Ctrl-C to cancel)\n")
            prev_timing = self.profile_timing or {"ramp_up_s": 5.0, "flat_s": 2.0, "ramp_down_s": 5.0}
            ramp_up_s = self._prompt_float("Ramp Up Time (s)", prev_timing["ramp_up_s"])
            flat_s = self._prompt_float("Flat Top Time (s)", prev_timing["flat_s"])
            ramp_down_s = self._prompt_float("Ramp Down Time (s)", prev_timing["ramp_down_s"])

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

            for ch, c in per_channel.items():
                c["nickname"] = self._get_nickname(ch)

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

            total_s = ramp_up_s + flat_s + ramp_down_s
            total_ticks = total_s * PID_LOOP_RATE_HZ_ASSUMED
            auto_decim = max(1, math.ceil(total_ticks / PID_LOG_MAX_SAMPLES_ASSUMED))
            raw = input(f"Log decim (ticks/sample) [{auto_decim}, auto-sized so "
                        f"~1000 samples cover the full {total_s:.2f}s shot]: ").strip()
            decim = int(raw) if raw else auto_decim

            print("\n--- Summary ---")
            print(f"  RampUp={ramp_up_s:g}s  FlatTop={flat_s:g}s  RampDown={ramp_down_s:g}s  "
                  f"Total={total_s:g}s")
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
        self.link.query("SOURce:STOP")
        for ch, c in per_channel.items():
            # Sent explicitly every time, enabled or not -- matches this
            # project's "explicit, no implicit magic" convention (same
            # reasoning as TABLE:BEGIN before every upload, FAULT:CLEAR
            # even after a condition clears) -- so a channel enabled in
            # a previous session/shot doesn't silently stay enabled (or
            # vice versa) just because this wizard run didn't mention it.
            #
            # XREX:CHANnel:ENAOut/CONTactOut -- added 2026-09-22, real
            # gap found the hard way: ArmConditionsMet() (state_machine.c)
            # requires BOTH of these HIGH for every currently PID-enabled
            # channel before ARM will succeed (XrexIo_EnableOutputsReadyToArm(),
            # added 2026-09-17) -- this wizard never sent them, so `shot`
            # always failed ARM with ERR 13 regardless of gains/timing/
            # DEBUG:FAULT:BYPASS (a SEPARATE precondition, not a fault).
            # Matches run_simulator_validation.py's own arm_and_gate(),
            # which already got this right for the automated campaigns.
            self.link.query(f"XREX:CHANnel:ENAOut {ch} {1 if c['enabled'] else 0}")
            self.link.query(f"XREX:CHANnel:CONTactOut {ch} {1 if c['enabled'] else 0}")
            self.link.query(f"SOURce:ENAble {ch} {1 if c['enabled'] else 0}")
            if c["enabled"]:
                bit = 1 if c["loop_mode"] == "closed" else 0
                self.link.query(f"PID:LOOPMODE {ch} {bit}")
                self.link.query(f"SHOT:CURRent {ch} {c['demand_a']}")
                if c["kp"] is not None:
                    self.link.query(f"PID:GAINS {ch} {c['kp']} {c['ki']} {c['kd']}")
            self.channel_config[ch] = dict(enabled=c["enabled"], demand_a=c["demand_a"],
                                            loop_mode=c["loop_mode"],
                                            kp=c["kp"], ki=c["ki"], kd=c["kd"])
        if log_all:
            self.link.query(f"LOG:ARM 0 1000 {decim}")
            self.last_log_all = True
            self.last_log_channel = None
            self.last_log_max_samples = 1000
            self.last_log_decim = decim
        elif log_channel:
            self.link.query(f"LOG:ARM {log_channel} 1000 {decim}")
            self.last_log_all = False
            self.last_log_channel = log_channel
            self.last_log_max_samples = 1000
            self.last_log_decim = decim
        self.link.query(f"SHOT:TIMing {ramp_up_s} {flat_s} {ramp_down_s}")
        self.profile_timing = dict(ramp_up_s=ramp_up_s, flat_s=flat_s, ramp_down_s=ramp_down_s)

        ts = datetime.now().strftime("%Y%m%d_%H%M%S")  # shared across every artifact
                                                         # this one shot produces, so
                                                         # they're easy to find together

        # State machine (added 2026-09-13, firmware side): SHOT:STARt
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

        reply = self.link.query("SHOT:STARt")
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
            out_path = os.path.join(self._shot_dir(ts), "dslogic.png")
            saved = dsl_capture.finish_and_plot(dsl_fw_log, out_path)
            if saved:
                print(f"Saved {saved}")

        self._auto_plot_after_shot(ts=ts)

    def do_refire(self, arg):
        """refire  -- re-fires the shot profile with whatever is
        CURRENTLY programmed on the device (channel enable/demand/
        gains/loop mode, SHOT:TIMing) -- no wizard prompts. The
        low-friction way to repeat a shot after changing just the PID
        constants: `gains <ch> <kp> <ki> <kd>` then `refire`, instead
        of re-running the whole `shot` wizard and re-answering every
        question. Sends ARM then SHOT:STARt (same danger-
        confirmation as typing SHOT:STARt directly -- see
        _is_dangerous()), watches it run, then auto-fetches + plots
        exactly like `shot` does (including the output/FFT plot, see
        _auto_plot_after_shot()). Does NOT include `shot`'s DSLogic
        cross-check capture -- use the full `shot` wizard for that.

        ALSO re-arms the waveform log (same target/maxSamples/decim as
        whatever `log`/`shot` last armed) immediately before firing --
        REAL BUG found the hard way, 2026-09-22: a log stops advancing
        once it fills (PID_GetLogSampleCount()'s own documented "keeps
        reporting the final count once full" behavior), so three
        `gains ...` + `refire` cycles in a row against a log that was
        only ever armed once (by the original `shot`) silently kept
        fetching and re-saving the FIRST shot's frozen data every time
        -- three different real shots genuinely ran (confirmed live in
        the watch output), but all three saved CSVs came back byte-
        for-byte identical, making a real Ki change look like it did
        nothing. Falls back to recomputing a decim from the current
        profile timing (same auto-sizing formula `shot` uses) if
        nothing was ever armed via `log`/`shot` this session to
        remember a decim from."""
        if not self._require_link():
            return
        if not (self.last_log_all or self.last_log_channel):
            print("[warn] no waveform log currently armed -- firing anyway, but there will "
                  "be nothing to auto-plot afterward. Arm one first: "
                  "log <ch|all> <maxSamples> <decim>")
        else:
            max_samples = self.last_log_max_samples or 1000
            decim = self.last_log_decim
            if decim is None and self.profile_timing:
                total_s = (self.profile_timing["ramp_up_s"] + self.profile_timing["flat_s"]
                           + self.profile_timing["ramp_down_s"])
                decim = max(1, math.ceil(total_s * PID_LOOP_RATE_HZ_ASSUMED / PID_LOG_MAX_SAMPLES_ASSUMED))
            decim = decim or 1
            wire_ch = "0" if self.last_log_all else self.last_log_channel
            self.link.query(f"LOG:ARM {wire_ch} {max_samples} {decim}")
        reply = self._query_print("ARM")
        if reply is None or is_err(reply):
            return
        reply = self._query_print("SHOT:STARt")
        if reply is None or is_err(reply):
            return
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self._watch_shot()
        self._auto_plot_after_shot(ts=ts)

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
        commands that start real PWM output (FIRE, SOURce:RUN,
        SHOT:STARt, and BOOT). With no argument, shows the
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
