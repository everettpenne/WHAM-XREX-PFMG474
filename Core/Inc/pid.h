#ifndef INC_PID_H_
#define INC_PID_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "ctrlr_config.h"

/*
 * pid.h / pid.c
 *
 * Closed-loop PID control for up to HRTIM_NUM_CHANNELS (ctrlr_config.h,
 * currently 4) independent Transrex channels -- this project's actual
 * reason for existing. See docs/changelog.txt's 2026-09-09
 * design-decision entry for the full architecture writeup this
 * implements; the short version:
 *
 *   - HRTIM side: Possibility 3
 *     (docs/Transrex/hrtim_divergent_period_timing.pdf) -- each active
 *     HRTIM channel is configured (hrtim.c's HRTIM1_FullInit())
 *     ResetTrigger=NONE/UpdateTrigger=NONE/ResetUpdate=ENABLED, so a
 *     PER/CMP1 write via HRTIM1_SetChannelPeriod() is always safe, at
 *     any time, and the hardware promotes it to active at THAT
 *     CHANNEL'S OWN next roll-over -- never mid-cycle, no relationship
 *     required between channels' frequencies.
 *
 *   - Timing side: the HRTIM Master repetition interrupt
 *     (HRTIM1_Master_IRQn, stm32g4xx_it.c) is kept as a FIXED-RATE
 *     heartbeat (PID_LOOP_RATE_HZ, ctrlr_config.h -- reprogrammed onto
 *     Master's own timebase by hrtim.c's HRTIM1_FullInit()),
 *     deliberately decoupled from any channel's own carrier frequency.
 *     This was a real, reconsidered design decision, not the obvious
 *     first idea -- a self-clocked alternative (each channel's own
 *     roll-over triggers its own PID recompute) was considered and
 *     rejected: it ties that channel's control-loop sample interval to
 *     the very frequency the PID is adjusting, which is both bad
 *     control practice (discrete PID assumes a fixed Delta-t; a
 *     drifting one shifts the effective I/D gains as the loop
 *     converges) and a category error (the V-to-F carrier's own
 *     frequency has no inherent relationship to the control bandwidth
 *     a magnet supply's real electrical time constant calls for). Do
 *     not "simplify" this back to self-clocked without re-reading that
 *     changelog entry first.
 *
 *   - PID_Update() (this module) runs once per Master heartbeat,
 *     looping over all HRTIM_NUM_CHANNELS channels: read channel i's
 *     latest measured period (pfm_input.c's PfmInput_GetLatestPeriod(),
 *     continuous/free-running mode -- PfmInput_StartContinuous()),
 *     convert to Hz, run that channel's own PID against its own
 *     setpoint with a FIXED Delta-t (= 1/PID_LOOP_RATE_HZ), clamp, write
 *     the new output frequency (converted back to a HRTIM `per`) via
 *     HRTIM1_SetChannelPeriod(). Each channel: fully independent state
 *     (setpoint, Kp/Ki/Kd, integrator) -- no shared table, no
 *     relationship to any other channel's own values.
 *
 * Units: setpoint, gains, and every reported value here are in Hz, not
 * raw timer ticks -- deliberately, per docs/changelog.txt's own note:
 * the V-to-F encoding this whole project serves is linear in
 * FREQUENCY, not period, so PID math needs to run in frequency space
 * for the loop to behave linearly/predictably across HRTIM's wide
 * dynamic range. The tick<->Hz conversion happens once per channel per
 * update, inside PID_Update() and HRTIM1_SetChannelPeriod().
 *
 * PER/CMP coherency (per the Possibility 3 reference document's own
 * caveat): HRTIM1_SetChannelPeriod() writes PER then CMP1 as two
 * separate register writes; a roll-over landing between them would
 * apply new-PER with old-CMP for one cycle. Not specifically guarded
 * against here -- CMP1 is always recomputed as per/2 (a clean 50%
 * duty, see that function's own comment), so the worst case is one
 * single cycle at a slightly-off duty, immaterial to a V-to-F receiver
 * that only cares about frequency. Revisit (HRTIM's UPDGAT, or a
 * DMA-burst transfer to make the pair atomic) only if real bench
 * testing ever shows this matters.
 *
 * FEEDBACK MEASUREMENT (2026-09-10, real production decision, see
 * docs/changelog.txt): PID_Update() reads pfm_input.c's
 * PfmInput_ConsumeAveragePeriod() -- the AVERAGE over every complete
 * period PFM_Input measured since the last tick (standard reciprocal-
 * frequency-counting practice: average the raw PERIODS, then convert
 * once to Hz), not a single most-recently-captured sample
 * (PfmInput_GetLatestPeriod(), still exists, no longer what the
 * control loop itself uses). A real Transrex output can be tens of
 * periods per 1 ms Master tick; averaging all of them gives a real
 * noise-rejecting measurement of the actual physical supply output
 * instead of one arbitrary edge.
 *
 * DEMAND PROFILE (2026-09-10, the real production shot shape, see
 * docs/changelog.txt): an operator programs Ramp Time / Flat Top Time
 * (SHARED across all 4 channels -- one synchronized shot clock, see
 * PID_SetProfileTiming()) and, per channel, a Demand Current in Amps
 * (PID_SetProfileCurrent()). PID_ProfileStart() begins a shot: 0 A ->
 * linear ramp to Demand Current over Ramp Time -> hold for Flat Top
 * Time -> linear ramp back to 0 A over Ramp Time -> full stop
 * (PID_Stop(), PFM output off entirely, per direct instruction --
 * NOT a hold-at-floor). Computed ON THE FLY from the compact
 * {rampTicks, flatTopTicks, demandCurrentA} description each
 * PID_Update() tick (TrapezoidalCurrentA(), pid.c) -- there is
 * deliberately no stored per-period table (not enough RAM for one,
 * and the whole point of Possibility 3 plus a fixed heartbeat is that
 * nothing needs one). The resulting instantaneous current target is
 * converted to a frequency (AmpsToHz(), pid.c -- LINEAR, a
 * placeholder pending real hardware characterization, see
 * ctrlr_config.h's own note on PFM_TURNON_FREQ_HZ) and that becomes
 * this channel's setpoint for the tick, feeding the same error/PID
 * math as always.
 *
 * OPEN-LOOP MODE (2026-09-10): each channel independently
 * (PID_SetLoopMode()) can run the profile-generated setpoint straight
 * to HRTIM with NO error correction at all -- feedback is still read
 * and reported (so open- vs. closed-loop behavior can be compared
 * directly against the same profile), just not used to adjust the
 * output. Defaults to closed-loop.
 *
 * OUTPUT SLEW-RATE CLAMP (2026-09-10): a real, independently-confirmed
 * finding -- a 250 MHz DSLogic capture of a real shot's actual HRTIM
 * output pin showed that a single bad feedback sample can make
 * PID_Update() genuinely WRITE a wildly wrong output for one tick
 * (not just log one), before the next tick's clean feedback corrects
 * it. Harmless on a bench loopback; not acceptable once a real
 * Transrex/magnet is in the loop. Every tick's actual write to HRTIM
 * -- open-loop or closed-loop, PID math or a plain setpoint step --
 * now passes through a hard clamp (ClampOutputSlew(), pid.c) bounding
 * how much it may differ from the previous tick's actual output, per
 * ctrlr_config.h's PID_OUTPUT_MAX_SLEW_HZ_PER_TICK. See that macro's
 * own extensive comment for the full rationale, the accepted trade-off
 * (it also bounds legitimate fast PID correction -- deliberate, a hard
 * clamp can't first classify which large jump is "real"), and why its
 * current value is a placeholder pending real hardware
 * characterization, same status as PFM_TURNON_FREQ_HZ/PFM_MAX_FREQ_HZ.
 */

/* Called once at boot (main.c) -- zeroes every channel's PID state and
 * sets a safe, inert default (all gains 0, so PID_Update() commands
 * PID_OUTPUT_MIN_HZ regardless of feedback, per project convention:
 * explicit, no implicit magic -- an operator must send real PID:GAINS
 * before this loop does anything meaningful). Does not touch HRTIM or
 * PFM_Input hardware -- see PID_Start() for that. Safe to call more
 * than once; each call re-zeroes everything (unlike pfm.c's
 * PFM_Init(), there is no durable per-channel state here worth
 * preserving across a re-init). */
void PID_Init(void);

/* Begins closed-loop operation: starts free-running PFM_Input capture
 * (PfmInput_StartContinuous()) on channels 0..HRTIM_NUM_CHANNELS-1 for
 * feedback, resets every channel's integrator/derivative history, and
 * starts HRTIM output (HRTIM1_PWM_Start()) with every active channel
 * enabled. A channel whose feedback signal isn't physically connected
 * (PfmInput_StartContinuous() still "succeeds" in the sense that DMA
 * capture starts -- it just never sees an edge) is not treated as an
 * error here: PID_Update() safely holds that channel's last output
 * rather than dividing by a period of 0, so a single unconnected
 * channel doesn't block the other 3. Returns 1 (always succeeds, or is
 * already running) -- kept non-void for symmetry with the rest of this
 * project's start/stop pairs and in case a real failure path is added
 * later. Idempotent -- calling while already running is a harmless
 * no-op. */
uint8_t PID_Start(void);

/* Stops closed-loop operation: stops HRTIM output (HRTIM1_PWM_Stop())
 * and every channel's continuous PFM_Input capture
 * (PfmInput_StopContinuous()). Idempotent -- harmless if not running.
 * Does NOT reset setpoints/gains (those are durable operator state,
 * same convention as pfm.c's g_channelEnabled) -- only PID_Start()
 * resets the transient integrator/derivative-history state. */
void PID_Stop(void);

/* The control loop itself -- called from HRTIM1_Master_IRQHandler()
 * (stm32g4xx_it.c) once per PID_LOOP_RATE_HZ heartbeat. A cheap no-op
 * when not running (PID_Start() never called, or PID_Stop() since).
 * See this file's own header comment for the full per-channel
 * algorithm. Not meant to be called from anywhere else. */
void PID_Update(void);

uint8_t PID_IsRunning(void);

/* Sets channel `channel`'s (0..HRTIM_NUM_CHANNELS-1) target output
 * frequency, clamped to [PID_OUTPUT_MIN_HZ, PID_OUTPUT_MAX_HZ]
 * (ctrlr_config.h). Returns 1 on success, 0 if `channel` is out of
 * range. Durable -- survives PID_Start()/PID_Stop(), same as pfm.c's
 * g_channelEnabled. Takes effect on this channel's very next
 * PID_Update() tick, whether or not the loop is currently running (a
 * setpoint set while stopped is simply what the loop will chase once
 * started). */
uint8_t PID_SetSetpoint(uint8_t channel, uint32_t setpointHz);

/* Arms and begins a LINEAR setpoint ramp on `channel`, added
 * 2026-09-10 for trajectory-tracking bench tests -- setpoint moves
 * linearly from `startHz` to `endHz` over `durationMs` milliseconds,
 * interpolated fresh from elapsed/total ticks every PID_Update() call
 * (not a fixed per-tick increment accumulated forward, so rounding
 * never compounds -- lands exactly on `endHz` on the final tick).
 * Real time resolution is bounded by PID_LOOP_RATE_HZ: a ramp is a
 * staircase of discrete setpoint steps at the control loop's own tick
 * rate, not a smooth analog ramp -- e.g. at the default 1 kHz loop
 * rate, a 10 ms ramp is exactly 10 steps. `startHz`/`endHz` clamped to
 * [PID_OUTPUT_MIN_HZ, PID_OUTPUT_MAX_HZ], same as PID_SetSetpoint().
 * Takes effect immediately (setpointHz jumps to `startHz` the instant
 * this is called, not just from the next tick) and from THIS
 * channel's very next PID_Update() tick onward -- does not itself
 * call PID_Start(); arm a ramp before or after starting the loop as
 * needed (a ramp only actually advances while PID_Update() is
 * running, i.e. between PID_Start()/PID_Stop() -- see that function's
 * own doc comment). Once `durationMs` has elapsed, setpointHz simply
 * stays at `endHz` as the new durable value (the same field
 * PID_SetSetpoint() sets) -- there is no separate "ramp mode" a
 * caller needs to track or clear; calling PID_SetSetpoint() at any
 * point cancels an in-progress ramp outright. Returns 1 on success, 0
 * if `channel` is out of range or `durationMs` is 0. */
uint8_t PID_StartRamp(uint8_t channel, uint32_t startHz, uint32_t endHz, uint32_t durationMs);

/* Sets channel `channel`'s PID gains and resets its integrator to 0 --
 * deliberately, to avoid a discontinuous output jump from an
 * integrator value accumulated under the OLD gains suddenly being
 * multiplied by new ones. Returns 1 on success, 0 if `channel` is out
 * of range. No sign/range validation on kp/ki/kd -- a badly-chosen
 * gain is an operator/tuning error to observe via PID:STATus?, not
 * something this function tries to guess is wrong. */
uint8_t PID_SetGains(uint8_t channel, float kp, float ki, float kd);

/* Reads back channel `channel`'s CURRENT gains, via `*kp`/`*ki`/`*kd`
 * (any may be NULL to skip that one) -- added 2026-09-10, alongside
 * PID:GAINS? (commands.c), specifically so a host tool (or an
 * operator reconnecting mid-session) can find out what's actually
 * programmed on the device instead of only being able to remember
 * what it itself last sent (see python/wham_console.py's `config`
 * command, whose whole reason for existing was this exact gap).
 * Returns 1 on success, 0 if `channel` is out of range (outputs left
 * unwritten in that case). Always succeeds otherwise -- gains default
 * to 0/0/0 from PID_Init(), not an "unset" sentinel, so this has
 * nothing to report as missing the way PID_GetProfileTiming() below
 * does. */
uint8_t PID_GetGains(uint8_t channel, float *kp, float *ki, float *kd);

/* Current setpoint/measured/output (Hz) for `channel`, via `*setpointHz`/
 * `*measuredHz`/`*outputHz` (any may be NULL to skip that one) --
 * valid whether or not the loop is running (measured/output simply
 * hold whatever they last were). Returns 1 on success, 0 if `channel`
 * is out of range (outputs left unwritten in that case). */
uint8_t PID_GetStatus(uint8_t channel, uint32_t *setpointHz,
                      uint32_t *measuredHz, uint32_t *outputHz);

/* Sets channel `channel`'s loop mode -- 1 = closed-loop (the default,
 * PID error correction against measured feedback, same as always), 0
 * = open-loop (the profile-generated -- or plain PID_SetSetpoint()/
 * PID_StartRamp() -- setpoint is written straight to HRTIM every
 * tick, NO error correction at all). Feedback is still read and
 * reported (PID_GetStatus()'s measuredHz, the waveform log) in open
 * loop -- it's just not used to adjust the output -- so open- vs.
 * closed-loop behavior against the identical commanded profile can be
 * compared directly. Per-channel, not global, matching this whole
 * architecture's independence between channels. Returns 1 on success,
 * 0 if `channel` is out of range. Takes effect on this channel's very
 * next PID_Update() tick. */
uint8_t PID_SetLoopMode(uint8_t channel, uint8_t closedLoop);
uint8_t PID_GetLoopMode(uint8_t channel);

/* Enables/disables channel `channel`'s output ENTIRELY -- added
 * 2026-09-11, per direct request. A genuinely different switch from
 * PID_SetLoopMode() above: open-loop still drives a real PFM waveform
 * (just uncorrected), and a 0A demand current still drives a real PFM
 * waveform too (at PFM_TURNON_FREQ_HZ, representing 0A -- not "off").
 * Disabled means NO PFM waveform at all -- the channel's HRTIM output
 * pin is physically disconnected (HRTIM1_SetChannelOutputEnable(),
 * hrtim.c) and PID_Update() skips this channel completely (no
 * setpoint, no feedback consumption, no PID math, no log entry --
 * see PID_Update()'s own comment).
 *
 * Takes effect on the NEXT PID_Start() if the loop isn't currently
 * running; takes effect IMMEDIATELY, live, if it is -- an operator can
 * kill (or restore) one channel's real output mid-shot without
 * touching any other channel or stopping the loop. Disabling does NOT
 * stop this channel's own HRTIM counter (kept synchronized for an
 * instant, clean re-enable -- see HRTIM1_SetChannelOutputEnable()'s
 * own comment) and does NOT reset this channel's PID state (integral,
 * setpoint, gains) -- re-enabling resumes exactly where it left off.
 * Defaults to enabled (1) for every channel -- today's only prior
 * behavior, unchanged unless a channel is explicitly disabled.
 * Returns 1 on success, 0 if `channel` is out of range. */
uint8_t PID_SetChannelEnable(uint8_t channel, uint8_t enabled);
uint8_t PID_GetChannelEnable(uint8_t channel);

/* Per-channel human-readable nickname (e.g. "TINKYWINKY") -- added
 * 2026-09-13, per direct request: purely a label, no effect whatsoever
 * on control behavior. Exists so an operator can assign a name to each
 * physical Transrex/phase output (independent of this firmware's own
 * Ch1..HRTIM_NUM_CHANNELS numbering) and have that name show up in
 * logged/plotted output instead of (alongside) the bare channel number.
 *
 * PID_CHANNEL_NICKNAME_MAX_LEN chosen to comfortably fit the specific
 * names given as examples ("TINKYWINKY" is 10 chars) with headroom, not
 * derived from any hardware constraint.
 *
 * Defaults to empty (no nickname) for every channel at boot -- PID_Init()
 * clears it, nothing else does (same persistence model as kp/ki/kd/
 * demandCurrentA: set once, survives across shots in the same session,
 * cleared only by a reboot -- there is no flash/EEPROM persistence
 * anywhere in this firmware, and this doesn't add any).
 *
 * The wire protocol (commands.c's cmd_pid_channel_nickname*()) is
 * whitespace-tokenized, so a nickname literally cannot contain spaces
 * (the parser would just see it as extra arguments) -- not separately
 * enforced here beyond the length check, since the parser already
 * can't hand this function a multi-word string. The literal string "-"
 * is reserved (PID:CHANnel:NICKname? uses it to mean "no nickname set"
 * on the wire) and rejected as a nickname value.
 *
 * PID_SetChannelNickname() returns 1 on success, 0 if `channel` is out
 * of range, `name` (or its length) is NULL/too long, or `name` is the
 * reserved "-" sentinel -- the channel's existing nickname (if any) is
 * left untouched on failure. PID_GetChannelNickname() returns a pointer
 * to this channel's own persistent buffer (NOT a copy -- valid as long
 * as the firmware runs, but treat as read-only) -- empty string ""
 * (not NULL) for both an unset nickname and an out-of-range channel, so
 * callers can always safely check name[0] without a NULL check. */
#define PID_CHANNEL_NICKNAME_MAX_LEN  (15U)
uint8_t PID_SetChannelNickname(uint8_t channel, const char *name);
const char *PID_GetChannelNickname(uint8_t channel);

/* --------------------------------------------------------------------------
 * General-Fault open-loop ramp-down -- added 2026-09-13, per direct
 * instruction (state_machine.h's HandleGeneralFault(), state_machine.c).
 * On a General Fault detected while FIRING, every channel that was
 * actively outputting immediately begins a linear, OPEN-LOOP ramp
 * (no PID/feedback correction at all) from wherever it actually was
 * down to PFM_TURNON_FREQ_HZ (0A) over FAULT_RAMP_DOWN_TIME_S
 * (ctrlr_config.h) seconds -- regardless of where in the normal
 * programmed shot profile it happened to be. Once every participating
 * channel reaches the floor, PID_Stop() is called (disconnects every
 * channel's HRTIM output unconditionally, same mechanism as every
 * other stop in this codebase) and the controller settles into FAULT,
 * waiting for FAULT:CLEAR.
 * -------------------------------------------------------------------------- */

/* Called ONCE by state_machine.c's HandleGeneralFault() the instant a
 * General Fault is detected while FIRING with real output -- captures
 * each currently-enabled channel's actual output Hz right now as its
 * ramp start point and arms the shared ramp clock; PID_Update() (via
 * its own internal ProcessFaultRampDown()) then drives the ramp
 * forward, one tick at a time, from here on. Deliberately does NOT
 * call PID_Stop() itself if any channel actually needs to ramp --
 * g_running must stay 1 (Master's counter must keep running) for
 * PID_Update() to keep being called at all. If no channel was actually
 * enabled/outputting (nothing for this fault to have interrupted),
 * stops immediately instead -- same as a fault caught while IDLE/
 * ARMED. Not meaningful to call this directly for any other reason;
 * it is not gated/validated the way the public SCPI-facing setters
 * are, since it is only ever invoked from inside the fault path. */
void PID_BeginFaultRampDown(void);

/* --------------------------------------------------------------------------
 * Demand profile -- added 2026-09-10, the real production shot shape.
 * See this file's own header comment for the full picture. Ramp Time/
 * Flat Top Time are SHARED (one synchronized clock for all
 * HRTIM_NUM_CHANNELS channels); Demand Current is per-channel.
 * -------------------------------------------------------------------------- */

/* Sets the SHARED ramp/flat-top durations, in milliseconds (matching
 * PID_StartRamp()'s existing convention) -- an operator-facing wire
 * command converts from the natural operator unit (seconds) before
 * calling this. Applies to every channel's NEXT PID_ProfileStart(),
 * not retroactively to a shot already in progress. Returns 1 on
 * success, 0 if either duration is 0. */
uint8_t PID_SetProfileTiming(uint32_t rampTimeMs, uint32_t flatTopTimeMs);

/* Reads back the SHARED ramp/flat-top durations, in milliseconds
 * (same unit as the setter -- an operator-facing wire command
 * converts to/from seconds), via `*rampTimeMs`/`*flatTopTimeMs` (either
 * may be NULL to skip). Added 2026-09-10 alongside PID:PROFile:TIMing?
 * -- same host-tool-readback motivation as PID_GetGains() above.
 * Returns 1 if timing has ever been successfully set this boot
 * (both outputs written), 0 if PID_SetProfileTiming() was never
 * called or was last rejected (both would-be durations are 0,
 * outputs left unwritten) -- distinct from PID_GetGains()'s always-
 * succeeds behavior because "never configured" is a real, meaningful
 * state here (PID_ProfileStart() itself refuses to run in it). */
uint8_t PID_GetProfileTiming(uint32_t *rampTimeMs, uint32_t *flatTopTimeMs);

/* Sets channel `channel`'s peak demand current for the profile, in
 * Amps -- clamped to [0, PFM_MAX_CURRENT_A_PER_CHANNEL[channel]]
 * (ctrlr_config.h) -- THIS channel's own full-range current, not one
 * value shared by all four (added 2026-09-11; *** see that macro's
 * own "MUST BE CALIBRATED BEFORE FINAL DEPLOYMENT" comment ***, every
 * entry is currently the same unverified placeholder). Takes effect
 * on this channel's next PID_ProfileStart(). Returns 1 on success, 0
 * if `channel` is out of range. */
uint8_t PID_SetProfileCurrent(uint8_t channel, float demandCurrentA);

/* Reads back channel `channel`'s current peak demand current (Amps),
 * via `*demandCurrentA` (may be NULL). Added 2026-09-10, same
 * host-tool-readback motivation as PID_GetGains() above. Returns 1 on
 * success, 0 if `channel` is out of range. Always succeeds otherwise
 * -- demandCurrentA defaults to 0.0f from PID_Init(), not an "unset"
 * sentinel, same reasoning as PID_GetGains(). */
uint8_t PID_GetProfileCurrent(uint8_t channel, float *demandCurrentA);

/* Begins a profiled shot on every channel at once, from the SAME
 * synchronized instant: zeroes the one shared elapsed-tick clock,
 * marks the profile active, and calls PID_Start() -- per direct
 * instruction, closed-loop operation begins the moment PFM output
 * does, not as a separate step. Each channel computes its own
 * trapezoidal current target from the shared clock and its own
 * PID_SetProfileCurrent() value (see TrapezoidalCurrentA(), pid.c);
 * channels currently in open-loop mode (PID_SetLoopMode()) still
 * follow the same profile, just without error correction. When the
 * shared clock reaches the end of the down-ramp, the shot ends
 * automatically: PID_Stop() (full stop, PFM output off entirely, per
 * direct instruction) -- an operator must send PID:PROFILE:START
 * again for another shot, nothing resumes on its own. Returns 1 on
 * success, 0 if PID_SetProfileTiming() was never called (rampTimeMs/
 * flatTopTimeMs both 0). */
uint8_t PID_ProfileStart(void);

uint8_t PID_IsProfileActive(void);

/* --------------------------------------------------------------------------
 * Waveform logging -- added 2026-09-10, so "demanded vs. closed-loop
 * output" can actually be PLOTTED, not just sampled a few times a
 * second over a serial round-trip (PID:STATus? polling, as used for
 * the first bench convergence test -- far too coarse to show the real
 * waveform shape at PID_LOOP_RATE_HZ). Logs either ONE channel
 * (PID_ArmLog()) or EVERY channel at once, from the same real ticks
 * (PID_ArmLogAll(), added later the same day for a genuine
 * simultaneous cross-channel comparison -- see its own comment), both
 * the measured feedback AND the output actually written to HRTIM that
 * same tick, so a host-side plot can overlay "commanded" against
 * "closed-loop measured" on the same time axis with no reconstruction
 * needed. Independent of PID_Start()/PID_Stop() -- arming a log
 * doesn't start or stop the loop, it just decides what PID_Update()
 * also records while running.
 * -------------------------------------------------------------------------- */

/* Cap on logged samples -- RAM-bounded: 3 arrays (setpoint/measured/
 * output) x HRTIM_NUM_CHANNELS x this many x 4 bytes for the log
 * storage itself (pid.c), since PID_ArmLogAll() (added 2026-09-10)
 * needs a row per channel even though PID_ArmLog() only ever uses one
 * of them -- at today's 4 channels that's 48000 bytes, a real,
 * deliberate chunk of this MCU's 128 KiB (see pid.c's own comment on
 * those arrays). 1000 chosen to still cover several real seconds of a
 * control-loop transient at a sane decimation (see PID_ArmLog()'s own
 * `decim` parameter) -- e.g. decim=4 covers 4 seconds at
 * PID_LOOP_RATE_HZ=1000 with 1000 samples, an effective 250 Hz log
 * rate, plenty to see a convergence curve's real shape. Revisit
 * downward if a future RAM-hungry feature needs the headroom back --
 * see docs/memory_report.html for current utilization. */
#define PID_LOG_MAX_SAMPLES  (1000U)

/* Arms logging for `channel` (0..HRTIM_NUM_CHANNELS-1): clears any
 * previous log, starts fresh. Every `decim`-th REAL PID_Update() tick
 * FOR THIS CHANNEL -- one entry per elapsed Master heartbeat, always,
 * regardless of whether that tick had fresh feedback -- appends one
 * {setpointHz, measuredHz, outputHz} sample, until `maxSamples`
 * (clamped to PID_LOG_MAX_SAMPLES) is reached, after which logging
 * simply stops appending -- the control loop itself is entirely
 * unaffected either way. `decim` clamped to at least 1 (log every
 * tick).
 *
 * FIXED 2026-09-10 (was previously wrong): a tick with no fresh
 * feedback yet (see PID_Update()'s own comment on the zero-samples
 * case, and pfm_input.c's PFM_DMA_BUF_LEN comment for why this is
 * common near PFM_TURNON_FREQ_HZ) used to be skipped entirely --
 * didn't count toward decimation, didn't get logged -- which silently
 * compressed the reported time axis (PID_GetLogSampleRateHz() assumes
 * a fixed decim*DT spacing between samples) whenever any ticks lacked
 * fresh feedback. Now every real tick logs -- a held-over
 * setpoint/measured/output value on a no-fresh-feedback tick shows up
 * as a genuine flat/staircase segment, an honest picture of "the
 * control loop had nothing new to act on this tick," not a gap that
 * silently vanishes from the timeline.
 *
 * Independent of PID_Start()/PID_Stop() and of any other channel's
 * own setpoint/gains/state -- arm this before or after starting the
 * loop, either works, logging just records whatever happens on this
 * channel from the moment this is called. Returns 1 on success, 0 if
 * `channel` is out of range. Cancels any PID_ArmLogAll() currently
 * armed (mutually exclusive with it -- the two share the same
 * underlying storage/index, see that function's own comment). */
uint8_t PID_ArmLog(uint8_t channel, uint16_t maxSamples, uint16_t decim);

/* Arms logging for EVERY channel at once, from the SAME real ticks --
 * added 2026-09-10, so a shot's simultaneous behavior across every
 * channel can be directly compared on one time axis (running the same
 * shot N separate times, once per channel, can only approximate this:
 * real feedback noise means no two runs are bit-for-bit identical --
 * see docs/changelog.txt's glitch-finding entries). Same semantics as
 * PID_ArmLog() otherwise (decim/maxSamples, independent of
 * PID_Start()/PID_Stop(), etc.) -- just every channel's
 * {setpointHz, measuredHz, outputHz} is written into the SAME sample
 * index each qualifying tick, instead of one channel's. Cancels any
 * PID_ArmLog() currently armed. RAM cost: this is WHY the log arrays
 * are 2D (HRTIM_NUM_CHANNELS x PID_LOG_MAX_SAMPLES) even for the
 * single-channel case above -- see pid.c's own comment on those
 * arrays for the exact byte count. Always succeeds (no channel
 * argument to validate). */
uint8_t PID_ArmLogAll(uint16_t maxSamples, uint16_t decim);

/* How many samples have been logged so far (<= whatever PID_ArmLog()'s
 * maxSamples was, clamped to PID_LOG_MAX_SAMPLES) -- keeps growing
 * (until the cap) as long as the armed channel(s) keep ticking,
 * whether or not the loop is still running. Shared across every
 * logged channel under PID_ArmLogAll() (see pid.c's own comment on
 * why that has to be a single shared counter, not one per channel). */
uint16_t PID_GetLogCount(void);

/* The effective sample rate the current log was/is being recorded at,
 * in Hz -- PID_LOOP_RATE_HZ / decim (the `decim` PID_ArmLog()/
 * PID_ArmLogAll() was last called with). A host-side plotter
 * reconstructs the time axis from this and PID_GetLogCount() -- sample
 * i occurred at i / PID_GetLogSampleRateHz() seconds after logging
 * started. EXACT (not approximate) as of the 2026-09-10 fix described
 * in PID_ArmLog()'s own comment -- every real tick is now accounted
 * for, logged or not. */
uint32_t PID_GetLogSampleRateHz(void);

/* 1 if PID_ArmLogAll() is the currently-armed mode, 0 if PID_ArmLog()
 * (a single channel) is, or if nothing is armed at all -- added
 * 2026-09-10 so a host tool (or cmd_pid_logdata(), commands.c) can
 * tell which validation rule applies to a channel argument: under
 * PID_ArmLogAll(), any channel 0..HRTIM_NUM_CHANNELS-1 has real data;
 * under PID_ArmLog(), only PID_GetLogChannel()'s one channel does. */
uint8_t PID_IsLogAllChannels(void);

/* The single channel PID_ArmLog() last armed (0..HRTIM_NUM_CHANNELS-1),
 * or 0xFF if nothing is armed (including while PID_ArmLogAll() is the
 * active mode -- check PID_IsLogAllChannels() first). Added 2026-09-10. */
uint8_t PID_GetLogChannel(void);

/* Raw logged arrays for `channel` (0..HRTIM_NUM_CHANNELS-1), index
 * 0..PID_GetLogCount()-1, in recording order -- measured feedback and
 * the output actually written to HRTIM that same tick, respectively.
 * Returns NULL for an out-of-range channel; otherwise always a
 * non-NULL pointer into a fixed static array regardless of how many
 * entries are actually valid -- callers must use PID_GetLogCount() to
 * know how many to read, and PID_IsLogAllChannels()/PID_GetLogChannel()
 * to know whether `channel`'s row actually holds meaningful data (a
 * channel not currently/previously armed holds stale or zeroed data,
 * not an error, but not meaningful either). */
const uint32_t *PID_GetLogMeasured(uint8_t channel);
const uint32_t *PID_GetLogOutput(uint8_t channel);

/* The setpoint in effect at each logged sample -- added 2026-09-10
 * alongside PID_StartRamp(), so a moving reference trajectory shows
 * up in the log, not just a constant that could be read once via
 * PID_GetStatus(). Same indexing/validity convention as
 * PID_GetLogMeasured()/PID_GetLogOutput() above. */
const uint32_t *PID_GetLogSetpoint(uint8_t channel);

#ifdef __cplusplus
}
#endif

#endif /* INC_PID_H_ */
