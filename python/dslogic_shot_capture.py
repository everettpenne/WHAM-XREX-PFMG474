#!/usr/bin/env python3
"""
dslogic_shot_capture.py -- optional DSLogic cross-check plot for
WHAM-XREX-PFMG474 shots.

If a DreamSourceLab DSLogic is physically connected, this module arms a
capture spanning a shot (right before it fires) and, once the shot
completes, saves a plot comparing the DSLogic's own decoded frequency
against the firmware's own PID:STATus? self-report -- the same
comparison style validated on real hardware in this session (see the
project memory note "dslogic-tool" and ~/dslogic-tool/README.md for the
full story of how that capture-and-decode pipeline was debugged and
confirmed correct).

If the DSLogic isn't connected, or ~/dslogic-tool isn't present, or
numpy/matplotlib aren't installed, every public function here degrades
to a clean no-op (returns False / None and prints one line explaining
why) -- a shot must behave identically whether or not a DSLogic happens
to be plugged in. Callers (wham_console.py's `shot` command) should
call is_available() before deciding whether to bother, but every other
function is itself safe to call unconditionally.

WIRING ASSUMPTION (fixed, per the user's own direct hardware setup):
    WHAM channel 1 (Phase U) <-> DSLogic Ch0
    WHAM channel 2 (Phase V) <-> DSLogic Ch1
    WHAM channel 3 (Phase W) <-> DSLogic Ch2
WHAM channel 4 (if used) has no DSLogic wiring and is simply left out of
this plot -- see WHAM_TO_DSLOGIC_CHANNEL below if that wiring ever
changes.
"""

import os
import subprocess
import sys
import threading
import time

DSLOGIC_TOOL_DIR = os.path.expanduser("~/dslogic-tool")

WHAM_TO_DSLOGIC_CHANNEL = {1: 0, 2: 1, 3: 2}

DEFAULT_SAMPLERATE_HZ = 2_000_000
DSLOGIC_ATOMIC_SIZE = 8  # bytes of one channel's own data per rotation --
                          # see dslogic_capture.py's CaptureResult docstring.

_dsl_mod = None
_import_attempted = False


def _try_import():
    """Imports the dslogic_capture module from ~/dslogic-tool (kept
    outside this repo -- it vendors GPL'd third-party source unrelated
    to WHAM firmware, see that project's own README for why). Cached --
    only actually tries once per process."""
    global _dsl_mod, _import_attempted
    if _import_attempted:
        return _dsl_mod
    _import_attempted = True
    if not os.path.isdir(DSLOGIC_TOOL_DIR):
        return None
    if DSLOGIC_TOOL_DIR not in sys.path:
        sys.path.insert(0, DSLOGIC_TOOL_DIR)
    try:
        import dslogic_capture
        _dsl_mod = dslogic_capture
    except Exception as exc:
        print(f"[dslogic] {DSLOGIC_TOOL_DIR} present but import failed ({exc}) "
              f"-- skipping DSLogic cross-check plot")
        _dsl_mod = None
    return _dsl_mod


def _have_plotting_deps():
    try:
        import numpy  # noqa
        import matplotlib  # noqa
        return True
    except ImportError:
        return False


def is_available():
    """True if a real (non-Demo) DSLogic is currently enumerated over
    USB AND this module can actually drive it (~/dslogic-tool present,
    numpy+matplotlib installed). Uses system_profiler rather than
    touching the DSLogic driver itself -- cheap, and avoids the
    LIBUSB_ERROR_ACCESS conflict that occurs if DSView.app happens to
    be running (see dslogic-tool/README.md's own gotcha -- that's a
    real failure this function deliberately does NOT try to work around
    by killing DSView.app itself; the actual capture call will just
    fail cleanly later and this whole thing gets skipped for that shot)."""
    if _try_import() is None:
        return False
    if not _have_plotting_deps():
        print("[dslogic] numpy/matplotlib not installed -- skipping DSLogic "
              "cross-check plot (pip install numpy matplotlib)")
        return False
    try:
        out = subprocess.run(
            ["system_profiler", "SPUSBDataType"],
            capture_output=True, text=True, timeout=5.0,
        ).stdout
    except Exception:
        return False
    return "DreamSourceLab" in out


class ShotCapture:
    """One DSLogic capture spanning a single WHAM shot. Usage:

        cap = ShotCapture()
        if cap.arm(wham_channels=[1, 2, 3], shot_total_s=3.0):
            ... send PID:PROFILE:START, poll status, etc ...
            cap.finish_and_plot(fw_log, out_path)

    Every method is safe to call even if arm() returned False or was
    never called -- they just no-op."""

    def __init__(self, samplerate_hz=DEFAULT_SAMPLERATE_HZ, margin_s=1.0):
        self.samplerate_hz = samplerate_hz
        self.margin_s = margin_s
        self.wham_channels = []
        self._dsl = None
        self._thread = None
        self._result = None
        self._error = None
        self._armed = False

    def arm(self, wham_channels, shot_total_s):
        """Arms a free-running capture (triggering doesn't gate yet --
        see dslogic_capture.py's own docstring -- so this is
        deliberately disable_trigger(), not set_simple_trigger()) sized
        to comfortably cover the whole shot plus margin on both ends.
        wham_channels is the set of WHAM firmware channels actually
        active this shot -- only the ones with real DSLogic wiring
        (WHAM_TO_DSLOGIC_CHANNEL) are captured; if none of the active
        channels are wired to the DSLogic at all, this is a deliberate
        no-op (returns False without even opening the device -- no
        point capturing 3 flat/idle lines). Returns True iff a capture
        thread is now running and the caller should proceed to fire the
        shot promptly."""
        mod = _try_import()
        if mod is None:
            return False

        self.wham_channels = [ch for ch in wham_channels if ch in WHAM_TO_DSLOGIC_CHANNEL]
        if not self.wham_channels:
            print("[dslogic] no active channel has DSLogic wiring (only "
                  f"{sorted(WHAM_TO_DSLOGIC_CHANNEL)} are wired) -- skipping capture")
            return False

        dsl_channels = sorted(WHAM_TO_DSLOGIC_CHANNEL[ch] for ch in self.wham_channels)
        try:
            self._dsl = mod.DSLogic()
            self._dsl.open_first()
            self._dsl.set_enabled_channels(dsl_channels)
            self._dsl.set_samplerate(self.samplerate_hz)
            capture_s = shot_total_s + 2 * self.margin_s
            self._dsl.set_limit_samples(int(self.samplerate_hz * capture_s))
            self._dsl.disable_trigger()
        except Exception as exc:
            print(f"[dslogic] arm failed ({exc}) -- skipping DSLogic cross-check plot "
                  f"(is DSView.app running? it holds the USB device exclusively)")
            self._cleanup()
            return False

        def _run():
            try:
                self._result = self._dsl.arm_and_capture(timeout_s=capture_s + 5.0)
            except Exception as exc:
                self._error = exc

        self._thread = threading.Thread(target=_run, daemon=True)
        self._thread.start()
        time.sleep(0.3)  # let the FPGA actually arm before the caller fires the shot
        self._armed = True
        return True

    def finish_and_plot(self, fw_log, out_path):
        """Waits for the capture to complete (bounded -- the arm()-time
        timeout already covers shot_total_s+margin, so this shouldn't
        normally block long) and, if it completed cleanly, saves the
        comparison plot to out_path. fw_log: list of dicts, each
        {"t_s": <float, seconds since shot start>, <wham_channel_int>:
        {"setpoint": .., "measured": ..}, ...} -- see
        wham_console.py's do_shot() for how this gets built (it already
        polls PID:STATus? per channel during the shot for its own
        console output; this just also keeps every row instead of
        discarding them). Always releases the DSLogic device before
        returning, even on failure. Returns the saved path, or None."""
        if not self._armed:
            return None
        try:
            self._thread.join(timeout=30.0)
            if self._error is not None:
                print(f"[dslogic] capture failed ({self._error}) -- skipping plot")
                return None
            if self._result is None or not self._result.ended:
                print("[dslogic] capture did not complete cleanly (still running or "
                      "timed out) -- skipping plot")
                return None
            try:
                return self._plot(fw_log, out_path)
            except Exception as exc:
                print(f"[dslogic] plotting failed ({exc}) -- raw capture data is lost "
                      f"(not saved separately), but the shot itself is unaffected")
                return None
        finally:
            self._cleanup()

    def _plot(self, fw_log, out_path):
        import numpy as np
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        result = self._result
        n_channels = result.n_channels
        raw = np.frombuffer(bytes(result.raw), dtype=np.uint8)
        group = n_channels * DSLOGIC_ATOMIC_SIZE
        n_groups = len(raw) // group
        raw2 = raw[: n_groups * group].reshape(n_groups, group)

        dsl_channels = sorted(WHAM_TO_DSLOGIC_CHANNEL[ch] for ch in self.wham_channels)
        colors = plt.cm.tab10.colors
        fig, axes = plt.subplots(2, 1, figsize=(11, 9), sharex=True)

        ax = axes[0]
        for i, dsl_ch in enumerate(dsl_channels):
            wham_ch = self.wham_channels[i]
            ch_bytes = raw2[:, i * DSLOGIC_ATOMIC_SIZE : (i + 1) * DSLOGIC_ATOMIC_SIZE].reshape(-1)
            bits = np.unpackbits(ch_bytes, bitorder="little")
            t = np.arange(len(bits)) / self.samplerate_hz
            edges = np.flatnonzero((bits[:-1] == 0) & (bits[1:] == 1)) + 1
            if len(edges) < 2:
                continue
            edge_t = t[edges]
            freq = 1.0 / np.diff(edge_t)
            ax.plot(edge_t[1:], freq, ".", ms=2, alpha=0.5, color=colors[i % len(colors)],
                     label=f"DSLogic Ch{dsl_ch} (WHAM ch{wham_ch})")
        ax.set_title("DSLogic: decoded frequency vs time")
        ax.set_ylabel("Hz")
        ax.legend()
        ax.grid(alpha=0.3)

        ax = axes[1]
        for i, wham_ch in enumerate(self.wham_channels):
            ts, sp, ms = [], [], []
            for row in fw_log:
                entry = row.get(wham_ch)
                if entry is None:
                    continue
                ts.append(row["t_s"])
                sp.append(entry["setpoint"])
                ms.append(entry["measured"])
            c = colors[i % len(colors)]
            ax.plot(ts, sp, "-", color=c, alpha=0.5, label=f"WHAM ch{wham_ch} setpoint (fw)")
            ax.plot(ts, ms, "--", color=c, alpha=0.9, label=f"WHAM ch{wham_ch} measured (fw)")
        ax.set_title("Firmware's own PID:STATus? self-report (same shot, ground truth)")
        ax.set_xlabel("time since shot start (s)")
        ax.set_ylabel("Hz")
        ax.legend(fontsize=8)
        ax.grid(alpha=0.3)

        plt.tight_layout()
        plt.savefig(out_path, dpi=130)
        plt.close(fig)
        return out_path

    def _cleanup(self):
        if self._dsl is not None:
            try:
                self._dsl.close()
            except Exception:
                pass
            self._dsl = None
