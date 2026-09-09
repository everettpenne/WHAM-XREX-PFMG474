#!/usr/bin/env python3
"""
dsl_viewer.py -- interactive viewer for DSView/DSLogic .dsl capture files.

Grew directly out of hand-decoding several .dsl files by hand during the
2026-09-04 cold-start phase-lock investigation (see docs/changelog.txt) --
every one of those was a one-off script: unzip, parse the INI `header`,
unpack the bit-packed `L-<channel>/0` files, print edge timestamps. This
generalizes that into an actual interactive tool instead of redoing it by
hand next time, with the one thing DSView itself was awkward for in this
workflow: a fast, precise way to zoom the time axis and measure between
two points (dead time, phase spacing, pulse width -- the exact
measurements this project keeps needing off real captures).

.dsl format (reverse-engineered from real captures, not from any DSView
source or spec -- see the comments below for exactly what's assumed and
why): a zip archive containing an INI-style `header` (samplerate, total
samples/probes, trigger position), a JSON `session` (channel names/
enabled state -- used here so custom names set in DSView itself, e.g.
renaming a channel to "U", carry over automatically), and one `L-<N>/0`
file per channel: raw digital samples, bit-packed 8 samples per byte,
LSB-first within each byte, one such file per probe index.

Usage:
  python3 python/dsl_viewer.py capture.dsl
  python3 python/dsl_viewer.py capture.dsl --labels U,UN,V,W
  python3 python/dsl_viewer.py capture.dsl --all-channels
  python3 python/dsl_viewer.py capture.dsl --window-start -2 --window-end 20
  python3 python/dsl_viewer.py capture.dsl --save out.png   # headless, no window

Interactive controls (once the window is open):
  - Standard matplotlib toolbar (magnifying glass / pan / home icons) --
    drag a rectangle to zoom to any time range, pan, or reset to full view.
  - Scroll wheel over the plot: zooms the TIME axis only, centered on the
    cursor -- the fast way to zoom in without dragging a box, which is
    what this script exists for.
  - Left click: place/move measurement cursor A (dashed line).
  - Right click: place/move measurement cursor B (dashed line).
  - With both placed, the delta (time + implied frequency) between them
    is shown in the title -- e.g. select two rising edges of the same
    channel to read off a period, or a rising edge and the following
    falling edge of the SAME pulse to read off a pulse width / dead time.

Prerequisites:
  pip install matplotlib numpy
"""

import argparse
import configparser
import json
import sys
import zipfile

try:
    import numpy as np
except ImportError:
    sys.exit("error: numpy not installed. Run: pip install numpy")

try:
    import matplotlib
    import matplotlib.pyplot as plt
except ImportError:
    sys.exit("error: matplotlib not installed. Run: pip install matplotlib")


def _parse_freq(text):
    """Parses a header samplerate string like '10 MHz' or '500 kHz' into Hz."""
    text = text.strip()
    parts = text.split()
    if len(parts) != 2:
        raise ValueError(f"unrecognized samplerate format: {text!r}")
    value = float(parts[0])
    unit = parts[1].lower()
    scale = {"hz": 1, "khz": 1e3, "mhz": 1e6, "ghz": 1e9}
    if unit not in scale:
        raise ValueError(f"unrecognized samplerate unit: {parts[1]!r}")
    return value * scale[unit]


class DslCapture:
    """Parsed contents of one .dsl file: per-channel sample arrays (as
    numpy uint8, values 0/1), sample rate, trigger sample index, and
    whatever channel names/enabled flags the `session` JSON carried."""

    def __init__(self, path):
        with zipfile.ZipFile(path) as zf:
            header_text = zf.read("header").decode("ascii", errors="replace")
            session_text = None
            if "session" in zf.namelist():
                session_text = zf.read("session").decode("utf-8", errors="replace")

            cfg = configparser.ConfigParser()
            cfg.read_string(header_text)
            h = cfg["header"]

            device_mode = int(h.get("device mode", "0"))
            if device_mode != 0:
                # Only ever seen mode 0 (plain digital/logic capture) on
                # this project's DSLogic U3Pro32 -- other device modes
                # (oscilloscope, data acquisition) likely use a different
                # sample encoding this class doesn't know how to unpack.
                # Fail loudly rather than silently misdecode.
                raise ValueError(
                    f"device mode {device_mode} is not a plain digital "
                    f"capture (mode 0) -- this parser only handles mode 0, "
                    f"see the class docstring"
                )

            total_blocks = int(h.get("total blocks", "1"))
            if total_blocks != 1:
                raise ValueError(
                    f"total blocks = {total_blocks} -- this parser has only "
                    f"ever been exercised against single-block captures and "
                    f"doesn't know how multi-block data is laid out; fix "
                    f"this before trusting it on a file like that"
                )

            self.total_samples = int(h["total samples"])
            self.total_probes = int(h["total probes"])
            self.sample_rate_hz = _parse_freq(h["samplerate"])
            self.trigger_sample = int(h.get("trigger pos", "0"))

            # probeN=<index> maps view position N to the actual probe
            # index used in the L-<index>/0 filenames -- in every capture
            # seen so far this has been the identity mapping (probe0=0,
            # probe1=1, ...), but read it properly rather than assume.
            probe_indices = []
            for n in range(self.total_probes):
                key = f"probe{n}"
                probe_indices.append(int(h[key]) if key in h else n)

            # session JSON, if present, carries per-channel name/enabled
            # state -- exactly what a user may have already customized in
            # DSView itself (e.g. renaming channel 0 to "U"). Best-effort:
            # a missing or unexpected-shape session just means we fall
            # back to generic names, not a hard failure.
            names_by_index = {}
            enabled_by_index = {}
            if session_text:
                try:
                    session = json.loads(session_text)
                    for ch in session.get("channel", []):
                        idx = ch.get("index")
                        if idx is not None:
                            names_by_index[idx] = ch.get("name")
                            enabled_by_index[idx] = ch.get("enabled", True)
                except (json.JSONDecodeError, AttributeError, TypeError):
                    pass

            self.channels = []  # list of dicts: index, name, enabled, samples
            n_bytes = (self.total_samples + 7) // 8
            for idx in probe_indices:
                raw = zf.read(f"L-{idx}/0")
                if len(raw) < n_bytes:
                    raise ValueError(
                        f"L-{idx}/0 is {len(raw)} bytes, expected at least "
                        f"{n_bytes} for {self.total_samples} samples"
                    )
                bits = np.unpackbits(
                    np.frombuffer(raw[:n_bytes], dtype=np.uint8),
                    bitorder="little",
                )[: self.total_samples]
                name = names_by_index.get(idx) or str(idx)
                self.channels.append(
                    {
                        "index": idx,
                        "name": name,
                        "enabled": enabled_by_index.get(idx, True),
                        "samples": bits,
                    }
                )

    def time_us(self):
        """Sample timestamps in microseconds, t=0 at the trigger sample."""
        n = np.arange(self.total_samples, dtype=np.float64)
        return (n - self.trigger_sample) / self.sample_rate_hz * 1e6


class Viewer:
    """The interactive matplotlib window: stacked digital traces, scroll-
    to-zoom on the time axis, and a two-cursor time/frequency measurement
    readout. See the module docstring for the actual control scheme."""

    def __init__(self, capture, channel_indices, labels, initial_window_us):
        self.capture = capture
        self.channel_indices = channel_indices  # positions into capture.channels
        self.labels = labels
        self.t = capture.time_us()

        self.cursor_a = None
        self.cursor_b = None
        self.line_a = None
        self.line_b = None

        n = len(channel_indices)
        self.fig, self.ax = plt.subplots(figsize=(14, max(3, 1.1 * n + 1)))
        self.fig.canvas.manager.set_window_title("DSLogic viewer")

        for row, ch_pos in enumerate(channel_indices):
            ch = capture.channels[ch_pos]
            offset = (n - 1 - row) * 1.5  # first channel drawn at the top
            level = ch["samples"].astype(np.float64) * 0.8 + offset
            self.ax.step(self.t, level, where="post", linewidth=1.1)
            self.ax.text(
                -0.01,
                offset + 0.4,
                labels[row],
                transform=self.ax.get_yaxis_transform(),
                ha="right",
                va="center",
                fontsize=10,
                fontweight="bold",
            )

        self.ax.set_yticks([])
        self.ax.set_xlabel("Time (µs, t=0 at trigger)")
        self.ax.grid(True, axis="x", linestyle=":", linewidth=0.6, alpha=0.6)
        self.ax.axvline(0, color="red", linestyle="--", linewidth=0.8, alpha=0.5)

        if initial_window_us is not None:
            self.ax.set_xlim(*initial_window_us)
        else:
            self.ax.set_xlim(self.t[0], self.t[-1])

        self._update_title()

        self.fig.canvas.mpl_connect("scroll_event", self._on_scroll)
        self.fig.canvas.mpl_connect("button_press_event", self._on_click)

        self.fig.tight_layout()

    def _update_title(self):
        parts = [
            f"{self.capture.sample_rate_hz / 1e6:g} MHz, "
            f"{self.capture.total_samples} samples"
        ]
        if self.cursor_a is not None and self.cursor_b is not None:
            dt_us = self.cursor_b - self.cursor_a
            parts.append(f"Δt = {dt_us:+.3f} µs ({dt_us * 1000:+.0f} ns)")
            if dt_us != 0:
                parts.append(f"f = {abs(1e6 / dt_us):,.1f} Hz")
        elif self.cursor_a is not None:
            parts.append(f"cursor A = {self.cursor_a:+.3f} µs (right-click to place B)")
        self.ax.set_title("   |   ".join(parts), fontsize=10)

    def _on_scroll(self, event):
        if event.inaxes != self.ax or event.xdata is None:
            return
        # Zoom factor per scroll step. Scrolling "up" (positive step)
        # zooms in; "down" zooms out. Centered on the cursor's x position,
        # not the axis center, so you can zoom straight into a specific
        # edge without also having to re-pan afterward.
        zoom_factor = 0.85 if event.step > 0 else 1 / 0.85
        xmin, xmax = self.ax.get_xlim()
        x = event.xdata
        new_min = x - (x - xmin) * zoom_factor
        new_max = x + (xmax - x) * zoom_factor
        self.ax.set_xlim(new_min, new_max)
        self.fig.canvas.draw_idle()

    def _on_click(self, event):
        if event.inaxes != self.ax or event.xdata is None:
            return
        if event.button == 1:  # left click: cursor A
            self.cursor_a = event.xdata
            if self.line_a is not None:
                self.line_a.remove()
            self.line_a = self.ax.axvline(
                self.cursor_a, color="tab:orange", linestyle="--", linewidth=1.2
            )
        elif event.button == 3:  # right click: cursor B
            self.cursor_b = event.xdata
            if self.line_b is not None:
                self.line_b.remove()
            self.line_b = self.ax.axvline(
                self.cursor_b, color="tab:blue", linestyle="--", linewidth=1.2
            )
        else:
            return
        self._update_title()
        self.fig.canvas.draw_idle()


def main():
    ap = argparse.ArgumentParser(
        description="Interactive viewer for DSView/DSLogic .dsl capture files"
    )
    ap.add_argument("dsl_file", help="path to a .dsl capture file")
    ap.add_argument(
        "--labels",
        help="comma-separated channel labels, in capture order, overriding "
        "any names already set in DSView (e.g. --labels U,UN,V,W)",
    )
    ap.add_argument(
        "--all-channels",
        action="store_true",
        help="show every channel in the capture, including ones that are "
        "entirely zero for the whole capture (unconnected probes) -- by "
        "default those are skipped to declutter the view",
    )
    ap.add_argument(
        "--window-start",
        type=float,
        help="initial x-axis view start, in us (e.g. --window-start -2 "
        "--window-end 20); two separate args, not one comma-joined value, "
        "because argparse mishandles a leading '-' on a negative start "
        "time otherwise. You can still zoom/pan freely afterward -- this "
        "only sets where the window opens. Both --window-start and "
        "--window-end must be given together.",
    )
    ap.add_argument(
        "--window-end",
        type=float,
        help="initial x-axis view end, in us -- see --window-start",
    )
    ap.add_argument(
        "--save",
        metavar="PNG_PATH",
        help="save a static PNG instead of opening an interactive window "
        "(useful headlessly, or over a remote/no-display session)",
    )
    args = ap.parse_args()

    capture = DslCapture(args.dsl_file)

    if args.all_channels:
        channel_indices = list(range(len(capture.channels)))
    else:
        channel_indices = [
            i
            for i, ch in enumerate(capture.channels)
            if ch["enabled"] and np.any(ch["samples"])
        ]
        if not channel_indices:
            print(
                "warning: every channel is either disabled or all-zero; "
                "showing all channels instead (pass --all-channels to "
                "silence this)",
                file=sys.stderr,
            )
            channel_indices = list(range(len(capture.channels)))

    if args.labels:
        custom = [s.strip() for s in args.labels.split(",")]
        if len(custom) != len(channel_indices):
            sys.exit(
                f"error: --labels has {len(custom)} names but {len(channel_indices)} "
                f"channels are being shown (use --all-channels if you intended "
                f"to label the skipped ones too)"
            )
        labels = custom
    else:
        labels = [capture.channels[i]["name"] for i in channel_indices]

    if (args.window_start is None) != (args.window_end is None):
        sys.exit("error: --window-start and --window-end must be given together")
    initial_window_us = None
    if args.window_start is not None:
        initial_window_us = (args.window_start, args.window_end)

    if args.save:
        matplotlib.use("Agg")

    viewer = Viewer(capture, channel_indices, labels, initial_window_us)

    if args.save:
        viewer.fig.savefig(args.save, dpi=150)
        print(f"saved: {args.save}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
