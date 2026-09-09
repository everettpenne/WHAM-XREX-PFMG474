#!/usr/bin/env python3
"""
pfm_table_upload.py -- build a PFM table on the host and upload it to
WHAM-XREX-PFMG474 over the TABle:BEGin / TABle:STEP / TABle:END serial
commands (see docs/command_reference.md).

Table CONSTRUCTION deliberately lives entirely in this script, not in
firmware -- see pfm.h's "ADDED" header note and Core/Src/commands.c's
cmd_table_step() for why. The firmware side only knows how to accept
and store whatever (per, cmp0, ..., cmp(N-1)) tuples it's sent; it does
no frequency/duty math and has no idea what "profile" produced them.

N (HRTIM_NUM_CHANNELS below) must match the specific board's firmware
build -- ctrlr_config.h's HRTIM_NUM_CHANNELS, a compile-time constant,
2026-09-08. This script queries CONFig:CHANnels? at startup and refuses
to upload on a mismatch, rather than silently sending a table shaped
for the wrong channel count.

This first version is intentionally narrow: a small set of fixed
profiles, selected by editing PROFILE below before running -- no CLI
flag, no general-purpose profile language. All five share the same
100 kHz carrier and the same ~5 ms total shot duration (500 periods),
so they're directly comparable on a scope. What "profile" ends up
meaning long-term (a real DSL? more shapes? per-phase-independent
duty?) is explicitly undecided -- don't read more structure into this
than "fixed test cases," and expect this file to be rewritten, not
incrementally extended forever, once that's figured out.

Usage:
  1. Edit PROFILE below (1-5).
  2. python3 python/pfm_table_upload.py --port /dev/cu.usbserial-XXXXX
"""

import argparse
import sys
import time

try:
    import serial  # pyserial
except ImportError:
    sys.exit("error: pyserial not installed. Run:  pip install pyserial")

# --- Profile selection -----------------------------------------------------
# Edit this, then run the script. No other configuration is read from
# the command line for the profile itself.
PROFILE = 10
#   1 = constant 100 kHz, constant 50% duty, all 3 phases
#   2 = constant 100 kHz, duty ramps 25% -> 75%, all 3 phases
#   3 = constant 100 kHz, duty ramps 90% -> 10%, all 3 phases
#   4 = constant 100 kHz, duty ramps 10% -> 90%, all 3 phases
#   5 = constant 100 kHz, duty ramps 90% -> 25%, all 3 phases
#   6 = FREQUENCY ramp 60 kHz -> 100 kHz, constant 50% duty, all 3
#       phases -- added 2026-09-09 to verify pfm_input.c's rising-edge
#       period capture actually tracks a CHANGING period, not just a
#       constant one (every prior real-hardware test used a fixed
#       100 kHz carrier). 200 total periods (FREQ_RAMP_STEPS *
#       FREQ_RAMP_DWELL_PERIODS below), deliberately sized to exactly
#       match PFM_INPUT_MAX_PERIODS (pfm_input.h) so the entire ramp
#       fits in one PFMIN:CAPTURE 200 / FIRE, not just a leading
#       fraction of it the way the constant-frequency profiles' first
#       ~20 periods were sampled in earlier sessions. End frequency
#       capped at ctrlr_config.h's PFM_MAX_CARRIER_FREQ_HZ (100 kHz) --
#       real-hardware testing at this range found intermittent
#       missing-pulse glitches on transitions landing above ~85 kHz,
#       worsening toward 140 kHz, and even the transition INTO exactly
#       100 kHz itself glitched (see docs/changelog.txt, 2026-09-09) --
#       root cause still open.
#   7 = FREQUENCY sweep 60 kHz -> 79 kHz ONLY, constant 50% duty, all 3
#       phases -- added 2026-09-09 specifically to validate (or
#       disprove) the working theory from PROFILE 6's real-hardware
#       results: that transitions are clean as long as every step's
#       period stays comfortably above where glitching started
#       (~85 kHz / ~2000 ticks). Deliberately kept well below that,
#       not just below PFM_MAX_CARRIER_FREQ_HZ -- this is a
#       theory-validation sweep, not a max-allowed-range test. Same
#       200-period/20-step/10-dwell structure as PROFILE 6 (reuses
#       _freq_ramp()) so results are directly comparable -- if this
#       comes back with ZERO doubled periods (firmware OVERCAP=0 and
#       every DATA? value matching its table entry), that's real
#       evidence for the theory; any doubling here would disprove it.
#   8 = FREQUENCY ramp 100 kHz -> 60 kHz (DECREASING), constant 50%
#       duty, all 3 phases -- added 2026-09-09, the reverse direction
#       of PROFILE 6, after the HRTIM SET/RESET-collision fix (see
#       docs/changelog.txt) was confirmed on an INCREASING sweep only.
#       per is inversely related to freq, so this ramp's `per` values
#       INCREASE step to step (the opposite of PROFILE 6's shrinking
#       `per`) -- worth checking on real hardware rather than assumed
#       symmetric, since the fixed mechanism (a SET/RESET collision
#       whose timing depends on exactly how the compare registers
#       change) has no obvious guarantee of being direction-agnostic.
#       Same 200-period/20-step/10-dwell structure, same frequency
#       endpoints as PROFILE 6, just traversed in reverse -- directly
#       comparable to that run's results.
#   10 = SLOWER-changing profile: ramp 50 kHz -> 100 kHz, hold at
#       100 kHz for exactly 1 ms (100 periods), ramp 100 kHz -> 33 kHz
#       -- added 2026-09-09, directly requested to test whether
#       PROFILE 9's real-hardware misalignment (4 of 6 channels
#       genuinely scrambled, not just index-shifted -- see
#       docs/changelog.txt) is specific to that profile's extreme
#       DWELL=1/every-period-a-transition rate, or present at more
#       moderate, realistic rates too. Ramps use DWELL=5 (each
#       frequency held 5 periods, vs PROFILE 9's 1) -- 50 periods each,
#       plus the 100-period hold = 200 total, fit deliberately to
#       PFM_INPUT_MAX_PERIODS so the WHOLE shot (not just a leading
#       fraction) is capturable in one PFMIN:CAPTURE 200, matching how
#       PROFILE 9 was tested. Real elapsed time comes out to a bit
#       under 3 ms (dominated by the 1ms hold, per the numbers
#       build_table() itself prints) -- shorter than the "~5 ms" this
#       was described with, a direct consequence of fitting the whole
#       thing into the 200-period capture cap; raise
#       PFM_INPUT_MAX_PERIODS (pfm_input.h) first if a literal ~5 ms
#       fully-captured run is ever needed.
#   9 = EXTREME zigzag: a fast triangle wave 5 kHz <-> 100 kHz, DWELL=1
#       (a genuinely different frequency every single real period, not
#       held for several like every other profile), 8 full up-down
#       cycles across 200 periods -- added 2026-09-09 as the most
#       aggressive real-hardware test yet of the now-DMA-based
#       PFM_Input capture (see docs/changelog.txt): widest frequency
#       range this project has tested (20x, vs. the 60-100 kHz ramps'
#       1.67x), maximum possible transition frequency (every period is
#       a transition), and both directions repeatedly, not a single
#       monotonic sweep. 5 kHz is comfortably inside `per`'s uint16
#       range (per=33999, nowhere near the 65535 ceiling) and well
#       below PFM_MAX_CARRIER_FREQ_HZ's 100 kHz ceiling on the other
#       end.

# --- Fixed parameters, shared by all five profiles --------------------------

APP_BAUD = 115200  # raised from 9600 on 2026-09-04 -- see AGENTS.md and
                    # docs/changelog.txt. WHAM-XREX-PFMG474-only; the
                    # sibling PFM-STM32G474 project still uses 9600.

# Must match Core/Inc/ctrlr_config.h's HRTIM_NUM_CHANNELS exactly for
# whatever board this is run against -- verified live at startup
# (main(), via CONFig:CHANnels?), not just trusted. Kept as a plain
# constant here (not auto-fetched and used for everything) so the
# script's own math stays readable without a round-trip on every run;
# the startup check is what actually prevents building a table shaped
# for the wrong channel count from ever being uploaded.
HRTIM_NUM_CHANNELS = 3

# Must match hrtim.h's HRTIM_TIMER_CLK_HZ exactly -- this is the real,
# hardware-verified HRTIM kernel clock (see main.c's SystemClock_Config()),
# not something this script can discover on its own.
HRTIM_TIMER_CLK_HZ = 170_000_000

FREQ_HZ = 100_000  # carrier frequency, all five profiles

# One PWM period = one table entry, held for one period each (matches
# firmware's PFM_HOLD_PERIODS = 1 -- see pfm.h). 500 periods at 100 kHz
# = 5 ms, matching the sibling PFM-STM32G474 project's own 5 ms default
# pulse-length convention, so results are easy to compare against it.
TOTAL_PERIODS = 500

# For all four ramp profiles: number of DISTINCT duty levels the ramp
# is broken into, each held for several periods (a staircase
# approximating a linear ramp, matching the sibling project's
# SWEEP-segment convention of "N steps x M dwell periods"). 50 x 10 =
# 500, matching TOTAL_PERIODS above.
RAMP_STEPS = 50
RAMP_DWELL_PERIODS = TOTAL_PERIODS // RAMP_STEPS  # 10

# Profile 6 (frequency ramp) only -- deliberately separate constants
# from the duty-ramp ones above, and a much smaller total (200, not
# 500) so the whole shot is capturable in a single PFMIN:CAPTURE --
# see PROFILE's own comment above.
FREQ_RAMP_START_HZ = 60_000
FREQ_RAMP_END_HZ = 100_000  # capped 2026-09-09 -- see ctrlr_config.h's
                            # PFM_MAX_CARRIER_FREQ_HZ; anything above
                            # it is now rejected (ERR 10) by
                            # commands.c's cmd_table_step(), and the
                            # real-hardware investigation that limit
                            # came from is exactly what this profile
                            # was built to explore (was 140_000)
FREQ_RAMP_DUTY_PCT = 50.0
FREQ_RAMP_STEPS = 20
FREQ_RAMP_DWELL_PERIODS = 10
FREQ_RAMP_TOTAL_PERIODS = FREQ_RAMP_STEPS * FREQ_RAMP_DWELL_PERIODS  # 200

# Profile 7 (theory-validation sweep) only -- same step/dwell structure
# as profile 6 (reuses _freq_ramp() directly with these bounds instead
# of FREQ_RAMP_START/END_HZ), deliberately entirely below where
# profile 6 started glitching.
SWEEP_START_HZ = 60_000
SWEEP_END_HZ = 79_000
SWEEP_DUTY_PCT = 50.0

# Profile 9 (extreme zigzag) only.
ZIGZAG_LOW_HZ = 5_000
ZIGZAG_HIGH_HZ = 100_000
ZIGZAG_DUTY_PCT = 50.0
ZIGZAG_CYCLES = 8           # full low->high->low triangle cycles
ZIGZAG_TOTAL_PERIODS = 200  # matches PFM_INPUT_MAX_PERIODS, dwell=1 throughout

# Profile 10 (ramp / hold / ramp) only.
RHR_RAMP1_START_HZ = 50_000
RHR_PEAK_HZ = 100_000
RHR_RAMP2_END_HZ = 33_000
RHR_DUTY_PCT = 50.0
RHR_RAMP_STEPS = 10          # distinct frequency levels per ramp segment
RHR_RAMP_DWELL_PERIODS = 5   # periods held per step -- 10*5 = 50 periods/ramp
RHR_HOLD_PERIODS = 100       # at RHR_PEAK_HZ -- exactly 1 ms at 100 kHz


def per_from_freq(freq_hz):
    """PWM period register value for a given carrier frequency -- same
    formula as hrtim.c's own hardcoded init default (Period=1699 for
    100 kHz at 170 MHz): round(clock / freq) - 1."""
    return round(HRTIM_TIMER_CLK_HZ / freq_hz) - 1


def cmp_from_duty(per, duty_pct):
    """Compare register value for a given duty percentage at this
    period. Output is ACTIVE from the period-start (PER) event until
    CMP1 (see hrtim.c's pOutputCfg: SetSource=TIMPER, ResetSource=
    TIMCMP1), so duty fraction = CMP1/PER, same relationship
    HRTIM1_FullInit()'s own init default already uses (850/1699 =~
    50.0% at PER=1699). Clamped to [2, per-2] -- matching hrtim.c's
    HRTIM1_ClampCompare() margin -- so this script never relies on
    that firmware-side clamp as anything but a backstop, per pfm.h's
    documented philosophy (validate before, don't rely on it after)."""
    cmp_val = round(per * (duty_pct / 100.0))
    return max(2, min(per - 2, cmp_val))


def build_table():
    """Returns a list of (per, cmp0, ..., cmp(N-1)) tuples for the
    selected PROFILE, N = HRTIM_NUM_CHANNELS. Every channel always gets
    the same duty -- none of the five profiles differ per-channel."""
    per = per_from_freq(FREQ_HZ)
    steps = []

    if PROFILE == 1:
        cmp_val = cmp_from_duty(per, 50.0)
        steps = [_entry(per, cmp_val)] * TOTAL_PERIODS

    elif PROFILE == 2:
        steps = _ramp(per, 25.0, 75.0)

    elif PROFILE == 3:
        steps = _ramp(per, 90.0, 10.0)

    elif PROFILE == 4:
        steps = _ramp(per, 10.0, 90.0)

    elif PROFILE == 5:
        steps = _ramp(per, 90.0, 25.0)

    elif PROFILE == 6:
        steps = _freq_ramp(FREQ_RAMP_START_HZ, FREQ_RAMP_END_HZ, FREQ_RAMP_DUTY_PCT)

    elif PROFILE == 7:
        steps = _freq_ramp(SWEEP_START_HZ, SWEEP_END_HZ, SWEEP_DUTY_PCT)

    elif PROFILE == 8:
        steps = _freq_ramp(FREQ_RAMP_END_HZ, FREQ_RAMP_START_HZ, FREQ_RAMP_DUTY_PCT)

    elif PROFILE == 9:
        steps = _zigzag(ZIGZAG_LOW_HZ, ZIGZAG_HIGH_HZ, ZIGZAG_DUTY_PCT,
                        ZIGZAG_CYCLES, ZIGZAG_TOTAL_PERIODS)

    elif PROFILE == 10:
        steps = _ramp_hold_ramp(RHR_RAMP1_START_HZ, RHR_PEAK_HZ, RHR_RAMP2_END_HZ,
                                RHR_DUTY_PCT, RHR_RAMP_STEPS, RHR_RAMP_DWELL_PERIODS,
                                RHR_HOLD_PERIODS)

    else:
        sys.exit(f"error: PROFILE must be 1-10 (got {PROFILE!r})")

    return steps


def _entry(per, cmp_val):
    """One table row: per, followed by HRTIM_NUM_CHANNELS copies of the
    same compare value (every channel always shares one duty here --
    see build_table())."""
    return (per,) + (cmp_val,) * HRTIM_NUM_CHANNELS


def _ramp(per, duty_start_pct, duty_end_pct):
    steps = []
    for i in range(RAMP_STEPS):
        # Duty at this level -- linear interpolation across RAMP_STEPS
        # levels, inclusive of both endpoints (level 0 = duty_start,
        # level RAMP_STEPS-1 = duty_end).
        frac = i / (RAMP_STEPS - 1)
        duty_pct = duty_start_pct + frac * (duty_end_pct - duty_start_pct)
        cmp_val = cmp_from_duty(per, duty_pct)
        steps.extend([_entry(per, cmp_val)] * RAMP_DWELL_PERIODS)
    return steps


def _freq_ramp(freq_start_hz, freq_end_hz, duty_pct):
    """Profile 6 only: FREQ_RAMP_STEPS distinct frequency levels
    (linear interpolation, inclusive of both endpoints, same staircase
    approach as _ramp() above but varying `per` instead of `cmp`),
    each held for FREQ_RAMP_DWELL_PERIODS periods at a fixed duty --
    `cmp` is recomputed at every step since it depends on `per`, which
    changes every step here (unlike _ramp(), where `per` is constant
    and only `cmp` varies)."""
    steps = []
    for i in range(FREQ_RAMP_STEPS):
        frac = i / (FREQ_RAMP_STEPS - 1)
        freq_hz = freq_start_hz + frac * (freq_end_hz - freq_start_hz)
        per = per_from_freq(freq_hz)
        cmp_val = cmp_from_duty(per, duty_pct)
        steps.extend([_entry(per, cmp_val)] * FREQ_RAMP_DWELL_PERIODS)
    return steps


def _zigzag(low_hz, high_hz, duty_pct, cycles, total_periods):
    """Profile 9 only: `cycles` full low->high->low triangle cycles
    across `total_periods` periods, DWELL=1 throughout -- unlike every
    other frequency profile here, every single entry is its own
    distinct frequency (no held steps at all). Each cycle is split
    into a rising half (linear frequency ramp low->high) and a
    falling half (high->low); `total_periods` is assumed evenly
    divisible by `cycles` (200/8 = 25, used as of this writing) so
    there's no leftover to pad."""
    periods_per_cycle = total_periods // cycles
    rising_len = periods_per_cycle // 2
    falling_len = periods_per_cycle - rising_len
    steps = []

    for _c in range(cycles):
        for i in range(rising_len):
            frac = i / (rising_len - 1) if rising_len > 1 else 0.0
            freq_hz = low_hz + frac * (high_hz - low_hz)
            per = per_from_freq(freq_hz)
            cmp_val = cmp_from_duty(per, duty_pct)
            steps.append(_entry(per, cmp_val))

        for i in range(falling_len):
            frac = i / (falling_len - 1) if falling_len > 1 else 0.0
            freq_hz = high_hz - frac * (high_hz - low_hz)
            per = per_from_freq(freq_hz)
            cmp_val = cmp_from_duty(per, duty_pct)
            steps.append(_entry(per, cmp_val))

    # Defensive -- only matters if total_periods isn't evenly
    # divisible by cycles (not the case for the defaults above).
    while len(steps) < total_periods:
        steps.append(steps[-1])
    return steps[:total_periods]


def _ramp_hold_ramp(freq1_hz, freq_peak_hz, freq2_hz, duty_pct,
                    ramp_steps, ramp_dwell, hold_periods):
    """Profile 10 only: ramp freq1->freq_peak (ramp_steps levels,
    ramp_dwell periods each), hold at freq_peak for hold_periods
    periods (a single, constant value -- not a staircase), then ramp
    freq_peak->freq2 (same steps/dwell as the first ramp). Reuses
    _freq_ramp()'s own linear-interpolation logic for both ramp
    segments so the shape matches every other ramp profile in this
    file, just with an explicit flat hold spliced in between."""
    # Temporarily borrow _freq_ramp()'s own module-level step/dwell
    # knobs rather than duplicating its loop -- restored immediately
    # after, so this has no effect on any other profile.
    global FREQ_RAMP_STEPS, FREQ_RAMP_DWELL_PERIODS
    saved = (FREQ_RAMP_STEPS, FREQ_RAMP_DWELL_PERIODS)
    FREQ_RAMP_STEPS, FREQ_RAMP_DWELL_PERIODS = ramp_steps, ramp_dwell
    try:
        ramp_up = _freq_ramp(freq1_hz, freq_peak_hz, duty_pct)
        ramp_down = _freq_ramp(freq_peak_hz, freq2_hz, duty_pct)
    finally:
        FREQ_RAMP_STEPS, FREQ_RAMP_DWELL_PERIODS = saved

    per_peak = per_from_freq(freq_peak_hz)
    cmp_peak = cmp_from_duty(per_peak, duty_pct)
    hold = [_entry(per_peak, cmp_peak)] * hold_periods

    return ramp_up + hold + ramp_down


def send_and_expect_ok(ser, line, timeout=2.0):
    """Send one command line, read back a reply, return it. Raises on
    ERR or a missing/garbled reply -- this script does not try to
    recover mid-upload, it just stops and reports where it got to.

    Uses read_until() rather than a manual read(128)-in-a-loop poll.
    pyserial's read(size) keeps trying to fill the *entire* requested
    size until the port's own timeout elapses, even once a short reply
    (our replies are always just a few bytes, e.g. "OK\\r\\n") has
    already fully arrived -- so with the port opened at timeout=1.0,
    every single round-trip was costing close to a full second
    regardless of the real ~20-30 ms wire time at 9600 baud. 500
    TABLE:STEP entries were taking 8+ minutes because of this read
    pattern, not because of the baud rate itself -- read_until(b"\\n")
    returns the instant the terminator shows up, only falling back to
    the port's configured timeout if a reply never arrives at all
    (a real failure, which still needs to time out and raise below)."""
    ser.reset_input_buffer()
    ser.write((line + "\r\n").encode("ascii"))
    ser.flush()

    ser.timeout = timeout
    reply = ser.read_until(b"\n")
    reply = reply.decode("ascii", errors="replace").strip()

    if not reply.startswith("OK"):
        raise RuntimeError(f"sent {line!r}, got {reply!r}")
    return reply


def main():
    ap = argparse.ArgumentParser(description="Upload a PFM table to WHAM-XREX-PFMG474")
    ap.add_argument("--port", required=True, help="serial device, e.g. /dev/cu.usbserial-XXXXX")
    ap.add_argument("--baud", type=int, default=APP_BAUD, help=f"default: {APP_BAUD}")
    args = ap.parse_args()

    table = build_table()
    print(f"=== PFM table upload: profile {PROFILE} ===")
    print(f"    channels: {HRTIM_NUM_CHANNELS}")
    print(f"    entries : {len(table)}")
    if PROFILE in (6, 7, 8, 9, 10):
        per_first, per_last = table[0][0], table[-1][0]
        print(f"    per     : {per_first} -> {per_last}  "
              f"({HRTIM_TIMER_CLK_HZ / (per_first + 1):.1f} Hz -> "
              f"{HRTIM_TIMER_CLK_HZ / (per_last + 1):.1f} Hz carrier)")
    else:
        per = table[0][0]
        print(f"    per     : {per}  (={HRTIM_TIMER_CLK_HZ / (per + 1):.1f} Hz carrier)")
    print(f"    port    : {args.port} @ {args.baud} 8N1")
    print()

    try:
        with serial.Serial(args.port, args.baud, timeout=1.0) as ser:
            time.sleep(0.2)  # let the port settle before the first command

            # Confirm the board actually agrees with HRTIM_NUM_CHANNELS
            # above before sending a single TABLE:STEP shaped for it --
            # see this file's header comment. A mismatch here would
            # otherwise only surface many steps into the upload, as a
            # confusing ERR 4 with no obvious cause.
            reply = send_and_expect_ok(ser, "CONFIG:CHANNELS?")
            try:
                board_channels = int(reply.split()[1])
            except (IndexError, ValueError):
                sys.exit(f"\n[fail] couldn't parse CONFIG:CHANNELS? reply: {reply!r}")
            if board_channels != HRTIM_NUM_CHANNELS:
                sys.exit(
                    f"\n[fail] board is built for {board_channels} channel(s), "
                    f"but this script (HRTIM_NUM_CHANNELS={HRTIM_NUM_CHANNELS}) "
                    f"would upload rows shaped for {HRTIM_NUM_CHANNELS} -- update "
                    f"HRTIM_NUM_CHANNELS above to match before running again."
                )
            print(f"[check] board confirms {board_channels} channel(s), matches this script")

            send_and_expect_ok(ser, "TABLE:BEGIN")
            print("[upload] TABLE:BEGIN ok, uploading...")

            for i, entry in enumerate(table):
                per_val, *cmp_vals = entry
                cmp_str = " ".join(str(v) for v in cmp_vals)
                send_and_expect_ok(ser, f"TABLE:STEP {per_val} {cmp_str}")
                if (i + 1) % 50 == 0 or (i + 1) == len(table):
                    print(f"[upload] {i + 1}/{len(table)}")

            reply = send_and_expect_ok(ser, "TABLE:END")
            print(f"[upload] TABLE:END -> {reply}")

    except (serial.SerialException, RuntimeError) as exc:
        sys.exit(f"\n[fail] {exc}")

    print("\n[done] table uploaded and confirmed.")


if __name__ == "__main__":
    main()
