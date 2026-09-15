#!/usr/bin/env python3
"""
dslogic_shot_capture.py -- optional DSLogic cross-check plot for
WHAM-XREX-PFMG474 shots.

If a DreamSourceLab DSLogic is physically connected, this module arms a
capture spanning a shot (right before it fires) and, once the shot
completes, saves a plot -- one ROW per WHAM channel, two panels per row
(firmware's own demand/measured feedback on the left, this channel's
independently-captured DSLogic frequency on the right) -- rather than
overlaying both sources on one axis, so each source's own shape stays
legible on its own scale. Validated against real hardware this session
(see the project memory note "dslogic-tool" and ~/dslogic-tool/
README.md for the full story of how the underlying capture-and-decode
pipeline was debugged and confirmed correct).

If the DSLogic isn't connected, or ~/dslogic-tool isn't present, or
numpy/matplotlib aren't installed, every public function here degrades
to a clean no-op (returns False / None and prints one line explaining
why) -- a shot must behave identically whether or not a DSLogic happens
to be plugged in. Callers (wham_console.py's `shot` command) should
call is_available() before deciding whether to bother, but every other
function is itself safe to call unconditionally.

Once a DSLogic IS connected, though, a plot always gets generated for
every shot, with one row per WHAM channel regardless of whether that
channel is enabled this shot -- a disabled (PID:CHANnel:ENAble 0) or
idle (0A demand) channel still gets a row, labeled as such, rather than
being silently dropped. This is deliberate: confirming a disabled
channel's HRTIM output really did stay low for the whole shot (no
switching at all, not just 0A) is exactly the kind of thing this
cross-check should be able to show at a glance.

Each row's title also shows that channel's PID:CHANnel:NICKname, if one
has been assigned (e.g. "Ch1 (TINKYWINKY)") -- see _channel_label()
below. The bare Ch<N> label is never dropped even when a nickname is
set; the nickname is purely an extra, human-friendly label alongside
it, matching wham_console.py's own channel_label() convention.

WIRING ASSUMPTION (CORRECTED 2026-09-15 -- see that day's dslogic-tool
memory note for the full story, including why this superseded not just
one but TWO earlier wrong assumptions across two separate sessions):
    WHAM channel 1 (Phase U)  <-> DSLogic Ch0  (Ch1 = Phase UN, the
    WHAM channel 2 (Phase V)  <-> DSLogic Ch2   complementary/inverted
    WHAM channel 3 (Phase W)  <-> DSLogic Ch4   output of the SAME HRTIM
    WHAM channel 4 (Phase X)  <-> DSLogic Ch6   channel -- not captured
                                                 here, carries identical
                                                 frequency information)
Each WHAM/HRTIM channel drives a COMPLEMENTARY PAIR of physical outputs
(HRTIM1_SetChannelOutputEnable()'s own doc comment: "output-1-ACTIVE/
output-2-INACTIVE"), wired out to TWO adjacent DSLogic channels each
(U/UN, V/VN, W/WN, X/XN) -- 8 DSLogic channels total for this board's 4
WHAM channels, not a 1:1 mapping. This module only ever captures the
non-inverted ("positive") phase of each pair -- confirmed on real
hardware, 2026-09-15, that the inverted phase carries the exact same
frequency information (same edge count/timing to within ~1 sample),
so capturing both would be redundant, not additional verification.
Direct real-hardware consequence of NOT knowing this earlier: a
2026-09-13/09-14 investigation that looked exactly like "WHAM Ch2/Ch3
are physically swapped on the bench" (see the memory note) was very
likely this SAME complementary-pair confusion misread as a swap, not a
real wiring defect -- confirmed on 2026-09-15 once the ACTUAL 8-channel
mapping was known: with it, every one of 3 simultaneously-active WHAM
channels (three genuinely different commanded frequencies) decoded
correctly and unambiguously on the first try, no swap-like symptom
anywhere. See WHAM_TO_DSLOGIC_CHANNEL below if the physical wiring
ever changes again.
"""

import os
import subprocess
import sys
import threading
import time

DSLOGIC_TOOL_DIR = os.path.expanduser("~/dslogic-tool")

WHAM_TO_DSLOGIC_CHANNEL = {1: 0, 2: 2, 3: 4, 4: 6}

DEFAULT_SAMPLERATE_HZ = 2_000_000
DSLOGIC_ATOMIC_SIZE = 8  # bytes of one channel's own data per rotation --
                          # see dslogic_capture.py's CaptureResult docstring.

# Amps<->Hz mapping -- mirrors wham_console.py's own hz_to_amps() (kept
# as a separate copy, same reasoning as _channel_label() below: this
# module is meant to be usable standalone). PLACEHOLDER LINEAR mapping,
# same caveat as wham_console.py's own copy: the real firmware's Amps
# demand for the highest channel is per-channel
# (PFM_MAX_CURRENT_A_PER_CHANNEL), but this Python-side constant is
# still a single global value -- see pending-hardware-calibration
# project memory. A channel's demand below PFM_TURNON_FREQ_HZ (e.g. a
# disabled channel sitting at PID_OUTPUT_MIN_HZ, 3000 Hz -- see
# ctrlr_config.h) maps to a clamped 0A, not a negative current.
PFM_TURNON_FREQ_HZ = 5000.0
PFM_MAX_FREQ_HZ = 100000.0
PFM_MAX_CURRENT_A = 5000.0


def _hz_to_amps(hz):
    import numpy as np
    hz = np.asarray(hz, dtype=float)
    a = (hz - PFM_TURNON_FREQ_HZ) / (PFM_MAX_FREQ_HZ - PFM_TURNON_FREQ_HZ) * PFM_MAX_CURRENT_A
    return np.maximum(0.0, a)


def _amps_to_hz(a):
    import numpy as np
    a = np.asarray(a, dtype=float)
    return a * (PFM_MAX_FREQ_HZ - PFM_TURNON_FREQ_HZ) / PFM_MAX_CURRENT_A + PFM_TURNON_FREQ_HZ


def _channel_label(channel, nickname=None):
    """'Ch3' or 'Ch3 (TINKYWINKY)' -- mirrors wham_console.py's own
    channel_label() (kept as a separate copy rather than importing from
    there, since this module is meant to be usable standalone). The
    bare Ch<N> label is never dropped even when a nickname is set --
    it's the real wire-protocol channel identity; the nickname is a
    purely cosmetic extra label alongside it, per direct instruction."""
    if nickname:
        return f"Ch{channel} ({nickname})"
    return f"Ch{channel}"


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
        """One row per WHAM channel, two columns: left = firmware's own
        demand (setpoint) and measured feedback (PID:STATus?), right =
        this same channel's independently-captured DSLogic frequency --
        side by side rather than overlaid, per direct request, so each
        source's own shape is legible on its own axes rather than
        competing for the same one."""
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

        fig, axes = plt.subplots(n, 2, figsize=(15, 2.6 * n), sharex=True, squeeze=False)

        for row, ch in zip(axes, channels):
            ax_fw, ax_dsl = row
            cfg = self.per_channel.get(ch, {})
            enabled = cfg.get("enabled", True)
            demand_a = cfg.get("demand_a", 0.0)
            nickname = cfg.get("nickname")
            wired = ch in WHAM_TO_DSLOGIC_CHANNEL
            dsl_ch = WHAM_TO_DSLOGIC_CHANNEL.get(ch)

            # -- left: firmware's own demand + measured feedback --
            ts, sp, ms = [], [], []
            for fw_row in fw_log:
                entry = fw_row.get(ch)
                if entry is None:
                    continue
                ts.append(fw_row["t_s"])
                sp.append(entry["setpoint"])
                ms.append(entry["measured"])
            if ts:
                ax_fw.plot(ts, sp, "-", color="#ff7f0e", alpha=0.7, label="demand (setpoint)")
                ax_fw.plot(ts, ms, "--", color="#2ca02c", alpha=0.9, label="measured (feedback)")
                ax_fw.legend(fontsize=8, loc="upper right")
            else:
                ax_fw.text(0.5, 0.5, "no firmware log for this channel", transform=ax_fw.transAxes,
                            ha="center", va="center", fontsize=9, color="gray", style="italic")

            # -- right: this channel's own DSLogic-captured frequency --
            dsl_note = None
            if wired:
                edge_t, freq = self._decode_channel(dsl_position[ch], n_captured, np)
                if edge_t is None:
                    dsl_note = "DSLogic data unavailable this shot"
                elif len(edge_t) == 0:
                    dsl_note = f"no switching detected on DSLogic Ch{dsl_ch} (output confirmed low)"
                else:
                    ax_dsl.plot(edge_t, freq, ".", ms=2, alpha=0.6, color="#1f77b4",
                                 label=f"DSLogic Ch{dsl_ch}")
                    ax_dsl.legend(fontsize=8, loc="upper right")
            else:
                dsl_note = "no DSLogic wiring on this channel"
            if dsl_note:
                ax_dsl.text(0.5, 0.5, dsl_note, transform=ax_dsl.transAxes,
                             ha="center", va="center", fontsize=9, color="gray", style="italic")

            if not enabled:
                status = "DISABLED"
            elif demand_a <= 0:
                status = "enabled, idle (0A)"
            else:
                status = f"{demand_a:g}A"
            label = f"{_channel_label(ch, nickname)} -- {status}"
            ax_fw.set_title(f"{label}  (firmware self-report)", fontsize=10, loc="left")
            ax_dsl.set_title(f"{label}  (DSLogic)", fontsize=10, loc="left")
            ax_fw.set_ylabel("Hz")
            ax_fw.grid(alpha=0.3)
            ax_dsl.grid(alpha=0.3)

            # Dual-scaled right axis, current corresponding to each
            # frequency -- direct request. secondary_yaxis (not twinx())
            # keeps the Amps scale an exact live function of whatever
            # the Hz axis ends up autoscaled to, no separate re-plot
            # needed. Left (firmware) panels only, per the request --
            # the right (DSLogic) panels already show a directly-
            # measured frequency, not a commanded demand, so an Amps
            # axis there wouldn't represent the same thing.
            ax_fw_a = ax_fw.secondary_yaxis("right", functions=(_hz_to_amps, _amps_to_hz))
            ax_fw_a.set_ylabel("A")

        axes[-1][0].set_xlabel("time since shot start (s)")
        axes[-1][1].set_xlabel("time since shot start (s)")
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
