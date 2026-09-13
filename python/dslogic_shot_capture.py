#!/usr/bin/env python3
"""
dslogic_shot_capture.py -- optional DSLogic cross-check plot for
WHAM-XREX-PFMG474 shots.

If a DreamSourceLab DSLogic is physically connected, this module arms a
capture spanning a shot (right before it fires) and, once the shot
completes, saves a plot -- one panel per WHAM channel -- comparing the
DSLogic's own decoded frequency against the firmware's own
PID:STATus? self-report, the same comparison style validated on real
hardware in this session (see the project memory note "dslogic-tool"
and ~/dslogic-tool/README.md for the full story of how that capture-
and-decode pipeline was debugged and confirmed correct).

If the DSLogic isn't connected, or ~/dslogic-tool isn't present, or
numpy/matplotlib aren't installed, every public function here degrades
to a clean no-op (returns False / None and prints one line explaining
why) -- a shot must behave identically whether or not a DSLogic happens
to be plugged in. Callers (wham_console.py's `shot` command) should
call is_available() before deciding whether to bother, but every other
function is itself safe to call unconditionally.

Once a DSLogic IS connected, though, a plot always gets generated for
every shot, with one panel per WHAM channel regardless of whether that
channel is enabled this shot -- a disabled (PID:CHANnel:ENAble 0) or
idle (0A demand) channel still gets a panel, labeled as such, rather
than being silently dropped. This is deliberate: confirming a disabled
channel's HRTIM output really did stay low for the whole shot (no
switching at all, not just 0A) is exactly the kind of thing this
cross-check should be able to show at a glance.

WIRING ASSUMPTION (fixed, per the user's own direct hardware setup):
    WHAM channel 1 (Phase U) <-> DSLogic Ch0
    WHAM channel 2 (Phase V) <-> DSLogic Ch1
    WHAM channel 3 (Phase W) <-> DSLogic Ch2
WHAM channel 4 (if used) has no DSLogic wiring -- its panel shows the
firmware's own self-report only, clearly labeled as unwired rather than
silently missing DSLogic data. See WHAM_TO_DSLOGIC_CHANNEL below if the
physical wiring ever changes.
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
        if cap.arm(per_channel, shot_total_s):
            ... send PID:PROFILE:START, poll status, etc ...
            cap.finish_and_plot(fw_log, out_path)

    Every method is safe to call even if arm() returned False or was
    never called -- they just no-op."""

    def __init__(self, samplerate_hz=DEFAULT_SAMPLERATE_HZ, margin_s=1.0):
        self.samplerate_hz = samplerate_hz
        self.margin_s = margin_s
        self.per_channel = {}
        self._dsl = None
        self._thread = None
        self._result = None
        self._error = None
        self._armed = False

    def arm(self, per_channel, shot_total_s):
        """Arms a free-running capture (triggering doesn't gate yet --
        see dslogic_capture.py's own docstring -- so this is
        deliberately disable_trigger(), not set_simple_trigger()) sized
        to comfortably cover the whole shot plus margin on both ends.

        per_channel: {wham_channel: {"enabled": bool, "demand_a": float,
        ...}} for EVERY WHAM channel this shot knows about (not just
        active ones) -- kept for labeling every panel of the eventual
        plot accurately, including disabled/idle channels and channels
        with no DSLogic wiring at all.

        ALWAYS captures the fixed set of DSLogic-wired channels
        (WHAM_TO_DSLOGIC_CHANNEL), regardless of which WHAM channels are
        actually enabled/active this particular shot -- a disabled
        channel should still show a flat, silent trace, which is itself
        useful confirmation, not something to skip capturing. Returns
        True iff a capture thread is now running and the caller should
        proceed to fire the shot promptly."""
        mod = _try_import()
        if mod is None:
            return False

        self.per_channel = per_channel
        dsl_wham_channels = sorted(WHAM_TO_DSLOGIC_CHANNEL)
        dsl_channels = [WHAM_TO_DSLOGIC_CHANNEL[ch] for ch in dsl_wham_channels]
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
        normally block long) and saves a comparison plot to out_path --
        one panel per WHAM channel in self.per_channel. fw_log: list of
        dicts, each {"t_s": <float, seconds since shot start>,
        <wham_channel_int>: {"setpoint": .., "measured": ..}, ...} --
        see wham_console.py's do_shot() for how this gets built (it
        already polls PID:STATus? per channel during the shot for its
        own console output; this just also keeps every row instead of
        discarding them). If the DSLogic capture itself failed or timed
        out, the plot still gets produced from fw_log alone (each panel
        just says DSLogic data wasn't available) -- a real device being
        connected means a plot should always come out, even in that
        degraded case. Always releases the DSLogic device before
        returning. Returns the saved path, or None only if arm() itself
        never succeeded (DSLogic wasn't connected to begin with)."""
        if not self._armed:
            return None
        try:
            self._thread.join(timeout=30.0)
            if self._error is not None:
                print(f"[dslogic] capture failed ({self._error}) -- plot will show "
                      f"firmware data only")
            elif self._result is None or not self._result.ended:
                print("[dslogic] capture did not complete cleanly (still running or "
                      "timed out) -- plot will show firmware data only")
                self._result = None
            try:
                return self._plot(fw_log, out_path)
            except Exception as exc:
                print(f"[dslogic] plotting failed ({exc}) -- shot itself is unaffected")
                return None
        finally:
            self._cleanup()

    def _decode_channel(self, dsl_ch_index, n_channels_captured, np):
        """Returns (edge_t, freq) numpy arrays for one captured DSLogic
        channel (by its 0-based POSITION among the channels actually
        enabled at capture time, not its DSLogic Ch number), or (None,
        None) if no capture data is available at all."""
        if self._result is None:
            return None, None
        raw = np.frombuffer(bytes(self._result.raw), dtype=np.uint8)
        group = n_channels_captured * DSLOGIC_ATOMIC_SIZE
        n_groups = len(raw) // group
        if n_groups == 0:
            return None, None
        raw2 = raw[: n_groups * group].reshape(n_groups, group)
        ch_bytes = raw2[:, dsl_ch_index * DSLOGIC_ATOMIC_SIZE : (dsl_ch_index + 1) * DSLOGIC_ATOMIC_SIZE].reshape(-1)
        bits = np.unpackbits(ch_bytes, bitorder="little")
        t = np.arange(len(bits)) / self.samplerate_hz
        edges = np.flatnonzero((bits[:-1] == 0) & (bits[1:] == 1)) + 1
        if len(edges) < 2:
            return np.array([]), np.array([])
        edge_t = t[edges]
        freq = 1.0 / np.diff(edge_t)
        return edge_t[1:], freq

    def _plot(self, fw_log, out_path):
        import numpy as np
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        channels = sorted(self.per_channel) if self.per_channel else sorted(WHAM_TO_DSLOGIC_CHANNEL)
        n = len(channels)
        n_captured = self._result.n_channels if self._result is not None else len(WHAM_TO_DSLOGIC_CHANNEL)
        # position of each DSLogic-wired WHAM channel among the channels
        # actually enabled at capture time (sorted DSLogic Ch numbers) --
        # arm() always enables sorted(WHAM_TO_DSLOGIC_CHANNEL) as a fixed
        # set, so this is just each wired channel's rank in that set.
        wired_order = sorted(WHAM_TO_DSLOGIC_CHANNEL, key=lambda ch: WHAM_TO_DSLOGIC_CHANNEL[ch])
        dsl_position = {ch: i for i, ch in enumerate(wired_order)}

        fig, axes = plt.subplots(n, 1, figsize=(11, 2.6 * n), sharex=True, squeeze=False)
        axes = [row[0] for row in axes]

        for ax, ch in zip(axes, channels):
            cfg = self.per_channel.get(ch, {})
            enabled = cfg.get("enabled", True)
            demand_a = cfg.get("demand_a", 0.0)
            wired = ch in WHAM_TO_DSLOGIC_CHANNEL
            dsl_ch = WHAM_TO_DSLOGIC_CHANNEL.get(ch)

            notes = []
            plotted_anything = False

            if wired:
                edge_t, freq = self._decode_channel(dsl_position[ch], n_captured, np)
                if edge_t is None:
                    notes.append("DSLogic data unavailable this shot")
                elif len(edge_t) == 0:
                    notes.append("no switching detected on DSLogic Ch"
                                 f"{dsl_ch} (output confirmed low)")
                else:
                    ax.plot(edge_t, freq, ".", ms=2, alpha=0.6, color="#1f77b4",
                             label=f"DSLogic Ch{dsl_ch}")
                    plotted_anything = True
            else:
                notes.append("no DSLogic wiring on this channel")

            ts, sp, ms = [], [], []
            for row in fw_log:
                entry = row.get(ch)
                if entry is None:
                    continue
                ts.append(row["t_s"])
                sp.append(entry["setpoint"])
                ms.append(entry["measured"])
            if ts:
                ax.plot(ts, sp, "-", color="#ff7f0e", alpha=0.6, label="setpoint (fw)")
                ax.plot(ts, ms, "--", color="#2ca02c", alpha=0.9, label="measured (fw)")
                plotted_anything = True

            if not enabled:
                status = "DISABLED"
            elif demand_a <= 0:
                status = "enabled, idle (0A)"
            else:
                status = f"{demand_a:g}A"
            ax.set_title(f"Ch{ch} -- {status}", fontsize=10, loc="left")
            ax.set_ylabel("Hz")
            ax.grid(alpha=0.3)
            if plotted_anything:
                ax.legend(fontsize=8, loc="upper right")
            if notes:
                ax.text(0.5, 0.5, " / ".join(notes), transform=ax.transAxes,
                         ha="center", va="center", fontsize=9, color="gray",
                         style="italic")

        axes[-1].set_xlabel("time since shot start (s)")
        fig.suptitle("DSLogic vs. firmware self-report, per channel")
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
