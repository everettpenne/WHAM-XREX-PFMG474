#!/usr/bin/env python3
"""
pfm_input_plot.py -- pulls whatever PFM_Input capture is currently
sitting in the controller (PFMIN:DATA? <ch>, see commands.h) and plots
it. A pure read-and-visualize tool: it does NOT arm a capture or send
FIRE -- run PFMIN:CAPTURE <M> and FIRE yourself first (by hand over a
terminal, or with another script), then run this to look at the
result. Matches this project's existing python/ tooling conventions
(pfm_table_upload.py's port-open/reply pattern, dsl_viewer.py's
matplotlib usage).

Plots period (raw ticks, as the wire protocol reports it -- see
pfm_input.h's own "raw ticks, not converted to time units" philosophy)
alongside the same data converted to instantaneous frequency (Hz), so
a frequency ramp/sweep is easy to read at a glance. OVERCAP and the
entry count are both shown in the plot title -- OVERCAP > 0 means some
entries may not be trustworthy (see PfmInput_GetOvercaptureCount()'s
own doc comment in pfm_input.c).

Multiple channels can be requested at once (--channel 1,2,3 or --channel
all) -- all of them are fetched and overlaid on ONE plot (one legend
entry per channel), not one figure each, so channels are directly
comparable. A channel with zero captured periods (nothing physically
wired to it, or never armed/fired) is still reported -- in the console
output and the plot title -- rather than silently dropped.

A channel with NOTHING physically wired to it doesn't just read zero --
a floating pin picks up electrical noise as spurious rising edges,
which pfm_input.c's ISR captures exactly as faithfully as a real signal
(that's the whole point of it having no debounce/filter -- see
pfm_input.c's own ICFilter=0 comment). That noise is typically many
orders of magnitude off from any real period (observed: individual
tick values approaching the 32-bit counter's own range, ~4.3e9) and a
high PfmInput_GetOvercaptureCount() (rapid spurious edges arriving
faster than the ISR can service them). The Y-AXIS AUTOSCALE below is
deliberately ROBUST to this: each subplot's visible range is set from
the 90th percentile of ITS OWN data (with margin), not the true max --
so one noisy/floating channel's spikes don't flatten a real channel's
data into invisibility. Genuine outlier points still plot (the line
just runs off the top of the visible area) -- nothing is deleted or
hidden from the data itself, only from the default view.

Usage:
  python3 python/pfm_input_plot.py --port /dev/cu.usbserial-XXXXX
  python3 python/pfm_input_plot.py --port /dev/cu.usbserial-XXXXX --channel 1
  python3 python/pfm_input_plot.py --port /dev/cu.usbserial-XXXXX --channel 1,3,5
  python3 python/pfm_input_plot.py --port /dev/cu.usbserial-XXXXX --channel all
  python3 python/pfm_input_plot.py --port /dev/cu.usbserial-XXXXX --save out.png
  python3 python/pfm_input_plot.py --port /dev/cu.usbserial-XXXXX --csv out.csv

Prerequisites:
  pip install pyserial matplotlib
"""

import argparse
import sys
import time

try:
    import serial  # pyserial
except ImportError:
    sys.exit("error: pyserial not installed. Run:  pip install pyserial")

try:
    import matplotlib
    import matplotlib.pyplot as plt
except ImportError:
    sys.exit("error: matplotlib not installed. Run:  pip install matplotlib")

APP_BAUD = 115200  # see AGENTS.md / docs/changelog.txt -- WHAM-XREX-PFMG474-only

# Must match hrtim.h's HRTIM_TIMER_CLK_HZ exactly -- the real,
# hardware-verified HRTIM kernel clock (see main.c's SystemClock_Config()),
# not something this script can discover on its own. Used only to
# convert ticks -> Hz for the second subplot; the wire data itself
# stays raw ticks throughout, per this project's own philosophy (see
# this file's header comment).
HRTIM_TIMER_CLK_HZ = 170_000_000

PFM_INPUT_NUM_CHANNELS = 6


def fetch_capture(ser, channel):
    """Sends PFMIN:DATA? <channel> and parses the reply. Returns
    (count, overcap, periods) -- periods is a list of raw tick values,
    in capture order. Raises RuntimeError on ERR or a malformed reply."""
    ser.reset_input_buffer()
    ser.write(f"PFMIN:DATA? {channel}\r\n".encode("ascii"))
    ser.flush()

    reply = ser.read_until(b"\n").decode("ascii", errors="replace").strip()
    if not reply:
        raise RuntimeError("no reply -- board unresponsive or wrong port/baud")
    if not reply.startswith("OK"):
        raise RuntimeError(f"PFMIN:DATA? {channel} -> {reply!r}")

    parts = reply.split()
    # "OK <count> OVERCAP=<n> <per1> <per2> ..." -- see commands.c's
    # cmd_pfmin_data() for the exact format this parses.
    try:
        count = int(parts[1])
        overcap = int(parts[2].split("=", 1)[1])
        periods = [int(p) for p in parts[3:]]
    except (IndexError, ValueError) as exc:
        raise RuntimeError(f"couldn't parse reply {reply!r}: {exc}") from exc

    if len(periods) != count:
        print(f"[warn] reply claims {count} entries but {len(periods)} were "
              f"parsed -- showing what was actually parsed", file=sys.stderr)

    return count, overcap, periods


_CHANNEL_COLORS = [
    "tab:blue", "tab:orange", "tab:green", "tab:red", "tab:purple", "tab:brown",
]


def _robust_ylim(all_values):
    """A y-axis upper limit that isn't blown out by a handful of extreme
    outliers (e.g. a floating/unconnected channel's noise, which can be
    orders of magnitude larger than any real period) -- see this file's
    header comment. Falls back to the plain max for a small or
    already-tight dataset, where a percentile isn't meaningfully
    different/safer anyway."""
    values = sorted(v for v in all_values if v > 0)
    if len(values) < 5:
        return max(values) * 1.1 if values else 1.0
    idx90 = int(0.90 * (len(values) - 1))
    p90 = values[idx90]
    true_max = values[-1]
    # If the true max isn't much bigger than the 90th percentile, there's
    # no real outlier problem -- just use it directly with a little
    # headroom rather than second-guessing a clean dataset.
    return true_max * 1.1 if true_max <= p90 * 2 else p90 * 1.3


def make_plot(results):
    """results: list of (channel, count, overcap, periods) tuples, one per
    requested channel -- all overlaid on ONE pair of period/frequency axes
    so multiple channels can be compared directly. A channel with zero
    captured periods (e.g. nothing physically wired to it) is skipped
    from the plotted lines but still listed in the legend/title, so an
    empty channel is visibly reported, not silently dropped."""
    fig, (ax_period, ax_freq) = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    chan_label = "+".join(f"{ch:02d}" for ch, *_ in results)
    fig.canvas.manager.set_window_title(f"PFM_Input_{chan_label} capture")

    title_parts = []
    any_plotted = False
    all_periods = []
    all_freqs_khz = []
    for (channel, count, overcap, periods), color in zip(results, _CHANNEL_COLORS):
        label = f"PFM_Input_{channel:02d}"
        overcap_note = f"OVERCAP={overcap}" if overcap else ""
        if count == 0:
            title_parts.append(f"{label}: no data")
            continue
        any_plotted = True
        indices = list(range(len(periods)))
        freqs_khz = [(HRTIM_TIMER_CLK_HZ / p / 1000.0) if p > 0 else 0.0 for p in periods]

        ax_period.plot(indices, periods, marker=".", markersize=4, linewidth=1,
                       color=color, label=label)
        ax_freq.plot(indices, freqs_khz, marker=".", markersize=4, linewidth=1,
                     color=color, label=label)
        all_periods.extend(periods)
        all_freqs_khz.extend(freqs_khz)

        title_parts.append(f"{label}: {count}" + (f" {overcap_note}" if overcap_note else ""))

    if not any_plotted:
        sys.exit("[fail] no channel in this request captured any periods -- "
                 "did you arm (PFMIN:CAPTURE) and FIRE first?")

    ax_period.set_ylabel("Period (raw ticks)")
    ax_period.set_ylim(0, _robust_ylim(all_periods))
    ax_period.grid(True, linestyle=":", linewidth=0.6, alpha=0.6)
    ax_period.legend(fontsize=9, loc="best")

    ax_freq.set_ylabel("Frequency (kHz)")
    ax_freq.set_ylim(0, _robust_ylim(all_freqs_khz))
    ax_freq.set_xlabel("Period index (capture order)")
    ax_freq.grid(True, linestyle=":", linewidth=0.6, alpha=0.6)
    ax_freq.legend(fontsize=9, loc="best")

    fig.suptitle("  |  ".join(title_parts), fontsize=10)
    fig.tight_layout()
    return fig


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0],
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", required=True, help="serial device, e.g. /dev/cu.usbserial-XXXXX")
    ap.add_argument("--baud", type=int, default=APP_BAUD, help=f"default: {APP_BAUD}")
    ap.add_argument("--channel", "--channels", dest="channels", default="1",
                     help="PFM_Input channel(s), 1-6 -- a single number (default: 1), "
                          "a comma-separated list (e.g. 1,3,5), or 'all' for 1-6. "
                          "All requested channels are fetched and overlaid on ONE plot.")
    ap.add_argument("--save", metavar="FILE.png", help="save the plot headlessly instead of showing a window")
    ap.add_argument("--csv", metavar="FILE.csv", help="also write the raw (channel, index, period_ticks, freq_hz) data to a CSV file")
    args = ap.parse_args()

    if args.channels.strip().lower() == "all":
        channels = list(range(1, PFM_INPUT_NUM_CHANNELS + 1))
    else:
        try:
            channels = [int(c) for c in args.channels.split(",")]
        except ValueError:
            sys.exit(f"[fail] --channel: couldn't parse {args.channels!r} "
                     f"(expected a number, a comma-separated list, or 'all')")
    for ch in channels:
        if not (1 <= ch <= PFM_INPUT_NUM_CHANNELS):
            sys.exit(f"[fail] --channel: {ch} out of range (1-{PFM_INPUT_NUM_CHANNELS})")

    if args.save:
        matplotlib.use("Agg")  # headless backend -- no display needed

    results = []
    try:
        with serial.Serial(args.port, args.baud, timeout=2.0) as ser:
            time.sleep(0.2)  # let the port settle before the first command
            for ch in channels:
                count, overcap, periods = fetch_capture(ser, ch)
                print(f"[fetch] PFM_Input_{ch:02d}: {count} periods, OVERCAP={overcap}")
                if overcap:
                    print(f"[warn] PFM_Input_{ch:02d}: OVERCAP={overcap} -- some entries "
                          f"may not be trustworthy (see PfmInput_GetOvercaptureCount()'s "
                          f"doc comment in pfm_input.c)")
                results.append((ch, count, overcap, periods))
    except (serial.SerialException, RuntimeError) as exc:
        sys.exit(f"[fail] {exc}")

    if args.csv:
        with open(args.csv, "w") as f:
            f.write("channel,index,period_ticks,freq_hz\n")
            for ch, count, overcap, periods in results:
                for i, p in enumerate(periods):
                    freq = HRTIM_TIMER_CLK_HZ / p if p > 0 else 0.0
                    f.write(f"{ch},{i},{p},{freq:.1f}\n")
        print(f"[csv] wrote {args.csv}")

    fig = make_plot(results)

    if args.save:
        fig.savefig(args.save, dpi=150)
        print(f"[save] wrote {args.save}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
