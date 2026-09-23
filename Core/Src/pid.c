/*
 * pid.c
 *
 * See pid.h for the full architecture writeup (Possibility 3 + fixed-
 * rate Master heartbeat) and docs/changelog.txt's 2026-09-09
 * design-decision entry for the reasoning behind it.
 */

#include "pid.h"
#include "hrtim.h"
#include "pfm_input.h"
#include "state_machine.h"
#include "xrex_io.h"
#include <string.h>   /* strlen/strcpy/strcmp -- PID_SetChannelNickname() */

typedef struct
{
    uint32_t setpointHz;
    float    kp;
    float    ki;
    float    kd;

    float    integral;
    uint32_t lastMeasuredHz;   /* also what PID_GetStatus() reports -- always
                                  current as of the end of this channel's most
                                  recent PID_Update() pass, see that function */
    uint8_t  haveLastMeasured;

    uint32_t lastOutputHz;

    /* Linear setpoint ramp -- see pid.h's own comment on PID_StartRamp().
       rampTicksLeft == 0 means no ramp in progress (the common case);
       setpointHz above is then just a plain durable value, exactly as
       before this feature existed. While a ramp is active, setpointHz
       is OVERWRITTEN every tick by PID_Update() (interpolated from
       rampStartHz/rampEndHz/rampTotalTicks/rampTicksLeft) -- a plain
       PID_SetSetpoint() call during an active ramp still works (it's
       the same field), but will just be overwritten again next tick
       until the ramp finishes. */
    uint32_t rampStartHz;
    uint32_t rampEndHz;
    uint32_t rampTotalTicks;
    uint32_t rampTicksLeft;

    /* Demand profile / open-loop mode -- added 2026-09-10, see pid.h's
       header comment ("DEMAND PROFILE" / "OPEN-LOOP MODE" sections)
       and PID_ProfileStart()/PID_SetLoopMode()'s own doc comments.
       demandCurrentA is this channel's own peak (Amps) for the SHARED
       trapezoidal shot profile below -- meaningless unless g_profileActive.
       closedLoopEnabled defaults to 1 (closed-loop, today's only prior
       behavior) so existing PID_SetSetpoint()/PID_StartRamp()-driven
       use keeps working unchanged unless a channel is explicitly
       switched to open-loop. */
    float    demandCurrentA;
    uint8_t  closedLoopEnabled;

    /* Channel output enable -- added 2026-09-11, per direct request: a
       genuine "this channel outputs nothing at all" switch, distinct
       from closedLoopEnabled/open-loop (which still drives a real PFM
       waveform, just without error correction) or a 0A demand current
       (which still drives a real PFM waveform too -- at
       PFM_TURNON_FREQ_HZ, representing 0A, not "off"). Defaults to 1
       (enabled, today's only prior behavior -- every channel always
       output) so nothing changes unless a channel is explicitly
       disabled. See PID_SetChannelEnable()'s own doc comment in pid.h
       for exactly what "disabled" does and doesn't do. */
    uint8_t  outputEnabled;

    /* Human-readable nickname -- added 2026-09-13, see PID_SetChannelNickname()'s
       own doc comment in pid.h. Purely a label, no effect on control
       behavior. Empty string ("") means no nickname assigned. */
    char     nickname[PID_CHANNEL_NICKNAME_MAX_LEN + 1U];

    /* General-Fault open-loop ramp-down -- added 2026-09-13, see
       PID_BeginFaultRampDown()/ProcessFaultRampDown()'s own comments
       below for the full mechanism. faultRampStartHz is this
       channel's actual output Hz at the exact instant the fault was
       detected (captured once, at ramp start -- NOT recomputed) --
       the ramp interpolates from here down to PFM_TURNON_FREQ_HZ,
       wherever in the normal shot profile this channel happened to
       be. faultRampParticipating is 1 only for channels that were
       actually enabled (outputEnabled != 0) at the moment the fault
       hit -- an already-disabled/idle channel has nothing to ramp and
       is left alone entirely. Both meaningless unless g_faultRampActive. */
    uint32_t faultRampStartHz;
    uint8_t  faultRampParticipating;
} PidChannelState_t;

static PidChannelState_t g_ch[HRTIM_NUM_CHANNELS];
static uint8_t g_running = 0U;

/* --------------------------------------------------------------------------
 * General-Fault ramp-down -- SHARED clock, same reasoning as the
 * shot-profile's own g_profile* globals above: every participating
 * channel begins ramping at the exact same tick (whenever the fault
 * was detected) and ramps for the exact same fixed duration
 * (FAULT_RAMP_DOWN_TIME_S, ctrlr_config.h) -- only each channel's own
 * START point (faultRampStartHz, per-channel, above) differs, since
 * each channel could genuinely be at a different frequency/point in
 * its own shot profile when the fault hit. One shared elapsed/total
 * tick pair is enough; no per-channel clock needed. This is
 * deliberately a SEPARATE mechanism from g_profile* -- the fault ramp
 * does not care where the normal profile was or resume it; see
 * PID_BeginFaultRampDown()'s own comment on why g_profileActive is
 * cleared outright rather than paused.
 * -------------------------------------------------------------------------- */
static uint8_t  g_faultRampActive       = 0U;
static uint32_t g_faultRampElapsedTicks = 0U;
static uint32_t g_faultRampTotalTicks   = 0U;

/* FAULT_RAMP_DOWN_TIME_S -- RUNTIME-CONFIGURABLE as of 2026-09-22,
   direct request. Backs CONFig:FaultRampTime (commands.c) via
   PID_SetFaultRampDownTimeS()/PID_GetFaultRampDownTimeS() (below).
   Read by PID_BeginFaultRampDown() (converted to ticks there, against
   whatever g_pidLoopRateHz currently is -- see that function's own
   updated comment). */
static float g_faultRampDownTimeS = FAULT_RAMP_DOWN_TIME_S;

/* --------------------------------------------------------------------------
 * Demand profile -- SHARED shot clock. Deliberately ONE global elapsed-
 * tick counter, not one per channel, so every channel's ramp/flat-top/
 * ramp shape stays perfectly synchronized in time even though each
 * channel has its own peak demandCurrentA (see pid.h). Ticks, not ms,
 * same convention as the pre-existing rampTicksLeft feature above.
 * -------------------------------------------------------------------------- */
static uint8_t  g_profileActive        = 0U;
static uint32_t g_profileElapsedTicks  = 0U;
static uint32_t g_profileRampUpTicks   = 0U;   /* independently configurable from
                                                   the down-ramp -- added 2026-09-22,
                                                   direct request. Was one shared
                                                   g_profileRampTicks (up == down)
                                                   before this. */
static uint32_t g_profileRampDownTicks = 0U;
static uint32_t g_profileFlatTopTicks  = 0U;
static uint32_t g_profileTotalTicks    = 0U;   /* rampUpTicks + flatTopTicks + rampDownTicks */

/* Control-loop sample rate/interval -- RUNTIME-CONFIGURABLE as of
   2026-09-22, direct request (was PID_LOOP_RATE_HZ, a #define, with
   PID_DT_SEC computed from it once at compile time). g_pidLoopRateHz
   defaults to the ctrlr_config.h compile-time value; PID_SetLoopRateHz()
   (below) is the only way to change it, and recomputes g_pidDtSec in
   the same call so the two can never drift out of sync with each
   other. g_pidDtSec is what every per-tick PID_Update() calculation
   actually reads (it used to read the PID_DT_SEC macro directly) --
   still just a fixed-rate heartbeat, still deliberately decoupled from
   any channel's own carrier/demand frequency, see pid.h's own header
   comment; only WHAT that fixed rate is can now change live. */
static uint32_t g_pidLoopRateHz = PID_LOOP_RATE_HZ;
static float    g_pidDtSec      = 1.0f / (float)PID_LOOP_RATE_HZ;

/* Waveform log -- see pid.h's own comment block on PID_ArmLog(). Plain
   parallel arrays (SoA), matching pfm_input.c's own period[] array
   convention rather than an array of structs. Setpoint included
   (2026-09-10, alongside PID_StartRamp()) so a moving reference
   trajectory -- not just a static setpoint -- shows up in the log
   too; before ramps existed, setpoint was constant for the whole log
   anyway and wasn't worth logging. */
/* Added 2026-09-10, ROW PER CHANNEL: previously one flat
   [PID_LOG_MAX_SAMPLES] array, only ever holding the single armed
   channel's data. Generalized to [HRTIM_NUM_CHANNELS][PID_LOG_MAX_SAMPLES]
   so PID_ArmLogAll() (below) can log every channel from the SAME
   shot, on the SAME shared time axis -- the whole point being a
   direct, simultaneous, apples-to-apples comparison across channels,
   which running the same shot N separate times (once per channel)
   can only approximate, not guarantee (real hardware, real feedback
   noise -- see docs/changelog.txt's glitch-finding entries -- two
   "identical" runs are never bit-for-bit identical). RAM cost: 4x a
   single channel's own (3 arrays * HRTIM_NUM_CHANNELS *
   PID_LOG_MAX_SAMPLES * 4 bytes = 84000 bytes at today's 4
   channels/1750 samples, up from 12000 -- tracked in the usual
   memory-footprint commit; a real, deliberate RAM/capability
   trade-off, not an accident. */
static uint32_t g_logSetpoint[HRTIM_NUM_CHANNELS][PID_LOG_MAX_SAMPLES];
static uint32_t g_logMeasured[HRTIM_NUM_CHANNELS][PID_LOG_MAX_SAMPLES];
static uint32_t g_logOutput[HRTIM_NUM_CHANNELS][PID_LOG_MAX_SAMPLES];
static uint16_t g_logCount   = 0U;
static uint16_t g_logCap     = 0U;
static uint8_t  g_logChannel = 0xFFU;   /* 0xFF = no single channel armed
                                            (either nothing armed at all --
                                            see g_logAllChannels -- or
                                            all-channels mode is active) */
static uint8_t  g_logAllChannels = 0U;  /* 1 = PID_ArmLogAll() armed every
                                            channel at once; g_logChannel
                                            is meaningless in this mode */
static uint16_t g_logDecim   = 1U;
static uint16_t g_logDecimCounter = 0U;

/* Per-channel Transrex full-range current -- see ctrlr_config.h's own
   extensive comment on PFM_MAX_CURRENT_A_PER_CHANNEL (added
   2026-09-11, *** MUST BE CALIBRATED BEFORE FINAL DEPLOYMENT ***,
   currently all 4 entries are the SAME unverified placeholder). The
   _Static_assert below is the compile-time guard promised in that
   comment: HRTIM_NUM_CHANNELS is the number of entries AmpsToHz()
   actually indexes (0..HRTIM_NUM_CHANNELS-1) -- if a future edit ever
   changes HRTIM_NUM_CHANNELS without updating this initializer list to
   match, this fails the BUILD instead of silently indexing past the
   array (or leaving a channel's entry as 0, quietly making that
   channel's Amps<->Hz conversion degenerate). */
/* RUNTIME-CONFIGURABLE as of 2026-09-22, direct request (was `const`,
   initialized once from the ctrlr_config.h placeholder and never
   touched again) -- PID_SetMaxCurrentA() (below) is the only way to
   change an entry, backing CONFig:MAXCURRent (commands.c). This is
   exactly the real per-channel calibration ctrlr_config.h's own "MUST
   BE CALIBRATED BEFORE FINAL DEPLOYMENT" comment describes -- making
   it live means that calibration no longer needs a rebuild+reflash
   per channel, just a serial command (though it's still session-only,
   never persisted -- see PID_SetMaxCurrentA()'s own doc comment). */
static float g_pfmMaxCurrentA[HRTIM_NUM_CHANNELS] = PFM_MAX_CURRENT_A_PER_CHANNEL;
_Static_assert(sizeof(g_pfmMaxCurrentA) / sizeof(g_pfmMaxCurrentA[0]) == HRTIM_NUM_CHANNELS,
               "PFM_MAX_CURRENT_A_PER_CHANNEL (ctrlr_config.h) must have exactly "
               "HRTIM_NUM_CHANNELS entries");

/* PFM_TURNON_FREQ_HZ/PFM_MAX_FREQ_HZ (ctrlr_config.h) -- RUNTIME-
   CONFIGURABLE as of 2026-09-22, direct request. PID_SetTurnonFreqHz()/
   PID_SetMaxFreqHz() (below) are the only ways to change them, backing
   CONFig:TURNONHz/CONFig:MAXFREQHz (commands.c). Both cross-validate
   against each other (turnon must stay strictly below max) and against
   [PID_OUTPUT_MIN_HZ, PID_OUTPUT_MAX_HZ] -- the documented nesting
   relationship AmpsToHz() below relies on (see that function's own
   comment) -- rather than trusting either setter's argument blindly. */
static uint32_t g_pfmTurnonFreqHz = PFM_TURNON_FREQ_HZ;
static uint32_t g_pfmMaxFreqHz    = PFM_MAX_FREQ_HZ;

/* --------------------------------------------------------------------------
 * Demand profile helpers -- see pid.h's "DEMAND PROFILE" doc section.
 * -------------------------------------------------------------------------- */

/* This channel's target current (Amps), on the shared trapezoidal
   shot shape, at `elapsedTicks` into the shot -- 0 -> linear up-ramp
   -> demandCurrentA -> flat-top -> linear down-ramp -> 0. The up-ramp
   and down-ramp each use their OWN independently-configured length
   (g_profileRampUpTicks/g_profileRampDownTicks, added 2026-09-22,
   direct request -- previously one shared g_profileRampTicks, up ==
   down always). Only ever called with elapsedTicks < g_profileTotalTicks
   (PID_Update() checks shot completion BEFORE calling this, see
   there) -- both ramp tick counts are guaranteed nonzero whenever
   g_profileActive, since PID_SetProfileTiming() refuses a zero ramp
   on either side, so the divisions below are safe. Interpolated from
   elapsed/total each call (not a fixed per-tick increment accumulated
   forward), matching the existing PID_StartRamp() convention just
   above -- lands exactly on the flat-top/zero boundaries regardless
   of how evenly the durations divide into whole ticks. */
static float TrapezoidalCurrentA(uint32_t elapsedTicks, float demandCurrentA)
{
    if (elapsedTicks < g_profileRampUpTicks)
    {
        float frac = (float)elapsedTicks / (float)g_profileRampUpTicks;
        return demandCurrentA * frac;
    }

    uint32_t flatEndTicks = g_profileRampUpTicks + g_profileFlatTopTicks;
    if (elapsedTicks < flatEndTicks)
    {
        return demandCurrentA;
    }

    /* Down-ramp. */
    uint32_t downElapsed = elapsedTicks - flatEndTicks;
    float frac = (float)downElapsed / (float)g_profileRampDownTicks;
    if (frac > 1.0f)
    {
        frac = 1.0f;   /* defensive only -- PID_Update()'s completion
                           check should always catch this first */
    }
    return demandCurrentA * (1.0f - frac);
}

/* Amps -> Hz, LINEAR PLACEHOLDER -- see ctrlr_config.h's own extensive
   comment on PFM_TURNON_FREQ_HZ/PFM_MAX_FREQ_HZ/PFM_MAX_CURRENT_A_PER_CHANNEL
   for why this is flagged as provisional (real Transrex SCR/phase-control
   physics may not be linear) and left for post-characterization
   revisit. Per direct instruction, 0A maps to EXACTLY
   PFM_TURNON_FREQ_HZ (not some frequency below it -- there is no
   "off but nonzero" output state between 0A and turn-on).

   `channel`, added 2026-09-11: PFM_MAX_CURRENT_A_PER_CHANNEL[channel]
   is THIS channel's own full-range current, not a single value shared
   by all four -- *** SEE THAT MACRO'S OWN "MUST BE CALIBRATED BEFORE
   FINAL DEPLOYMENT" COMMENT ***, ctrlr_config.h; every entry is
   currently the same unverified placeholder. Clamps currentA to
   [0, g_pfmMaxCurrentA[channel]] first (a profile's own math should
   never produce outside that range, but this is the last line of
   defense before a value reaches hardware) and the resulting Hz to
   [PID_OUTPUT_MIN_HZ, PID_OUTPUT_MAX_HZ] (the hardware-register safety
   clamp -- PFM_TURNON_FREQ_HZ/PFM_MAX_FREQ_HZ are documented to nest
   inside that range, this is defense-in-depth, not expected to ever
   actually bind). Caller's responsibility to pass a valid channel
   (0..HRTIM_NUM_CHANNELS-1) -- this is only ever called from
   PID_Update()'s own per-channel loop, which already guarantees that. */
static uint32_t AmpsToHz(uint8_t channel, float currentA)
{
    float maxCurrentA = g_pfmMaxCurrentA[channel];

    if (currentA < 0.0f)
    {
        currentA = 0.0f;
    }
    else if (currentA > maxCurrentA)
    {
        currentA = maxCurrentA;
    }

    float frac = currentA / maxCurrentA;
    float hzF = (float)g_pfmTurnonFreqHz +
                frac * ((float)g_pfmMaxFreqHz - (float)g_pfmTurnonFreqHz);

    if (hzF < (float)PID_OUTPUT_MIN_HZ)
    {
        hzF = (float)PID_OUTPUT_MIN_HZ;
    }
    else if (hzF > (float)PID_OUTPUT_MAX_HZ)
    {
        hzF = (float)PID_OUTPUT_MAX_HZ;
    }

    return (uint32_t)hzF;
}

/* RUNTIME-CONFIGURABLE as of 2026-09-22, direct request. Backs
   CONFig:SLEWRate (commands.c) via PID_SetSlewRateHzPerTick()/
   PID_GetSlewRateHzPerTick() below. This is a real hardware-safety
   clamp (see ClampOutputSlew()'s own comment just below) -- the
   setter enforces > 0 only (same "don't invent a paranoid extra
   bound the operator didn't ask for" philosophy as this project's
   other runtime setters), so it's the operator's own responsibility
   not to configure this so loose it stops meaningfully protecting
   downstream hardware; ctrlr_config.h's placeholder default (2000)
   still applies at boot either way. */
static float g_pidOutputMaxSlewHzPerTick = (float)PID_OUTPUT_MAX_SLEW_HZ_PER_TICK;

/* Hard per-tick output slew-rate clamp -- see ctrlr_config.h's own
   extensive comment on PID_OUTPUT_MAX_SLEW_HZ_PER_TICK for the full
   rationale (a REAL, DSLogic-confirmed single-tick output glitch,
   2026-09-10). Bounds `desiredHz` to within
   +/-g_pidOutputMaxSlewHzPerTick of `prevHz` (the previous tick's
   ACTUAL output, i.e. st->lastOutputHz), then re-clamps to
   [PID_OUTPUT_MIN_HZ, PID_OUTPUT_MAX_HZ] as defense-in-depth (prevHz
   is itself always already in that range by construction, so this
   second clamp should never actually bind -- included anyway since
   it's nearly free and this is a hardware-safety function). Called
   from EVERY code path that writes a channel's output, open-loop and
   closed-loop alike -- a hard clamp protecting real downstream
   hardware has to apply regardless of which code path computed the
   number. */
static float ClampOutputSlew(float desiredHz, float prevHz)
{
    float maxDelta = g_pidOutputMaxSlewHzPerTick;
    float clampedHz = desiredHz;

    if (clampedHz > (prevHz + maxDelta))
    {
        clampedHz = prevHz + maxDelta;
    }
    else if (clampedHz < (prevHz - maxDelta))
    {
        clampedHz = prevHz - maxDelta;
    }

    if (clampedHz < (float)PID_OUTPUT_MIN_HZ)
    {
        clampedHz = (float)PID_OUTPUT_MIN_HZ;
    }
    else if (clampedHz > (float)PID_OUTPUT_MAX_HZ)
    {
        clampedHz = (float)PID_OUTPUT_MAX_HZ;
    }

    return clampedHz;
}

/* ClampOutputRangeOnly() -- range clamp ONLY, deliberately no slew limiting.
   *** FIRST AND ONLY exception to ClampOutputSlew() in this codebase, added
   2026-09-15. *** Every other output write in this file goes through the
   full ClampOutputSlew() specifically because an unclamped frequency jump
   is a real, DSLogic-confirmed hardware glitch (see docs/changelog.txt,
   2026-09-10 entry) -- that risk is NOT waived lightly.

   This exists for exactly one caller: PID_BeginOvercurrentRampDown()'s
   derate step. The OCP fault spec calls for the surviving channels to
   step to a literal (100 * 1/N)% reduction; with the normal slew clamp
   (PID_OUTPUT_MAX_SLEW_HZ_PER_TICK, 2000 Hz/tick) that target is instead
   silently overridden by whatever the clamp allows, then the channel
   immediately proceeds into the ramp-to-floor -- meaning the derated
   value is never actually reached or held, just transited through
   incidentally. Confirmed on real hardware via DSLogic (3-channel shot,
   N=3): observed step matched the ~2000 Hz clamp prediction, not the
   33% target (e.g. one channel: ~16420 Hz -> ~14400 Hz measured vs.
   ~10933 Hz target). User explicitly decided (2026-09-15, asked directly
   when this tension was found) that the literal percentage matters more
   than slew-limiting this one step -- OCP is already the FASTER, more
   aggressive fault path (immediate hard disable of the faulted channel,
   no slew limit there either), so a single larger step on the survivors
   is consistent with that same urgency, not a new risk class. */
static float ClampOutputRangeOnly(float desiredHz)
{
    float clampedHz = desiredHz;

    if (clampedHz < (float)PID_OUTPUT_MIN_HZ)
    {
        clampedHz = (float)PID_OUTPUT_MIN_HZ;
    }
    else if (clampedHz > (float)PID_OUTPUT_MAX_HZ)
    {
        clampedHz = (float)PID_OUTPUT_MAX_HZ;
    }

    return clampedHz;
}

void PID_Init(void)
{
    for (uint8_t ch = 0U; ch < HRTIM_NUM_CHANNELS; ch++)
    {
        g_ch[ch].setpointHz         = (uint32_t)PID_OUTPUT_MIN_HZ;
        g_ch[ch].kp                 = 0.0f;
        g_ch[ch].ki                 = 0.0f;
        g_ch[ch].kd                 = 0.0f;
        g_ch[ch].integral           = 0.0f;
        g_ch[ch].lastMeasuredHz     = 0U;
        g_ch[ch].haveLastMeasured   = 0U;
        g_ch[ch].lastOutputHz       = 0U;
        g_ch[ch].rampStartHz        = 0U;
        g_ch[ch].rampEndHz          = 0U;
        g_ch[ch].rampTotalTicks     = 0U;
        g_ch[ch].rampTicksLeft      = 0U;
        g_ch[ch].demandCurrentA     = 0.0f;
        g_ch[ch].closedLoopEnabled  = 1U;   /* default: closed-loop, prior-only behavior */
        g_ch[ch].outputEnabled      = 1U;   /* default: enabled, prior-only behavior */
        g_ch[ch].nickname[0]        = '\0';  /* default: no nickname assigned */
        g_ch[ch].faultRampStartHz        = 0U;
        g_ch[ch].faultRampParticipating  = 0U;
    }
    g_running               = 0U;
    g_profileActive          = 0U;
    g_profileElapsedTicks    = 0U;
    g_profileRampUpTicks     = 0U;
    g_profileRampDownTicks   = 0U;
    g_profileFlatTopTicks    = 0U;
    g_profileTotalTicks      = 0U;
    g_faultRampActive        = 0U;
    g_faultRampElapsedTicks  = 0U;
    g_faultRampTotalTicks    = 0U;
}

/* Sane operating range for PID_SetLoopRateHz() -- NOT the full range
   HRTIM1_SetPidHeartbeatRate() (hrtim.c) could technically accept
   (roughly 649 Hz-42.5 MHz at this board's fixed /4 Master prescale);
   this is a much tighter, deliberately conservative band centered on
   the compile-time 1 kHz default this project has actually run and
   tuned against. Lower bound (700 Hz) stays comfortably clear of the
   hardware's own ~649 Hz floor rather than flirting with it; upper
   bound (10 kHz) is well above "tens of Hz to low kHz," the normal
   range this project's own PID_LOOP_RATE_HZ comment (ctrlr_config.h)
   already documents for a magnet-supply current loop -- raise it only
   after a real reason to run faster shows up.

   *** REAL BUG, FOUND AND FIXED 2026-09-23 ***: this used to say 500
   Hz, which is actually BELOW the ~649 Hz hardware floor the comment
   itself cites -- not unsafe (HRTIM1_SetPidHeartbeatRate() correctly
   rejects anything in [500, 648] regardless, so PID_SetLoopRateHz()
   never actually let an unreachable rate through), but every value in
   that dead sub-range looked like a valid, in-range request right up
   until it silently failed for a completely different, undocumented
   reason -- confusing for an operator with no way to tell from this
   range alone which low values are real vs. dead. Raised to a value
   genuinely clear of the real floor instead. */
#define PID_LOOP_RATE_HZ_MIN  (700UL)
#define PID_LOOP_RATE_HZ_MAX  (10000UL)

/* PID_LOOP_RATE_HZ -- RUNTIME-CONFIGURABLE as of 2026-09-22, direct
   request. Backs CONFig:PIDRate (commands.c). Refuses to change while
   g_running (ERR-equivalent 0 return) -- changing the control loop's
   OWN sample rate mid-shot is categorically different from every
   other runtime-configurable value added this same day (PID:GAINS,
   SHOT:TIMing, etc. all only take effect on the NEXT shot by
   design already): every already-armed/running channel's integral
   accumulator, slew-clamp history, and profile tick-counting all
   implicitly assume a CONSTANT dt across the shot they're mid-way
   through, so this refuses outright rather than accepting a value
   that would silently corrupt all of that for a shot already in
   progress. Range-checked against [PID_LOOP_RATE_HZ_MIN,
   PID_LOOP_RATE_HZ_MAX] above, then handed to
   HRTIM1_SetPidHeartbeatRate() (hrtim.c) for the actual register
   write/hardware-range check -- only committed to g_pidLoopRateHz/
   g_pidDtSec if THAT succeeds too, so a rejected hardware write never
   leaves the two out of sync with the real Master timebase. */
uint8_t PID_SetLoopRateHz(uint32_t hz)
{
    if (g_running != 0U)
    {
        return 0U;   /* refuse mid-shot -- see this function's own doc comment */
    }
    if ((hz < PID_LOOP_RATE_HZ_MIN) || (hz > PID_LOOP_RATE_HZ_MAX))
    {
        return 0U;
    }
    if (HRTIM1_SetPidHeartbeatRate(hz) == 0U)
    {
        return 0U;   /* hardware register range check failed -- see hrtim.c */
    }

    g_pidLoopRateHz = hz;
    g_pidDtSec      = 1.0f / (float)hz;
    return 1U;
}

uint32_t PID_GetLoopRateHz(void)
{
    return g_pidLoopRateHz;
}

/* PFM_TURNON_FREQ_HZ/PFM_MAX_FREQ_HZ -- RUNTIME-CONFIGURABLE as of
   2026-09-22, direct request. Back CONFig:TURNONHz/CONFig:MAXFREQHz
   (commands.c). Each cross-validates against the OTHER's current
   value (turnon must stay strictly below max -- AmpsToHz()'s linear
   interpolation inverts nonsensically otherwise) and against
   [PID_OUTPUT_MIN_HZ, PID_OUTPUT_MAX_HZ], the documented nesting
   relationship AmpsToHz() itself assumes (its own final clamp to that
   range is defense-in-depth regardless, so this isn't a hard safety
   requirement -- just what keeps the calibration physically
   sensible). No refuses-while-running policy like PID_SetLoopRateHz()
   -- a mid-shot change here just means the NEXT AmpsToHz() call (next
   tick) uses the new calibration, no discontinuity in timing/
   integration the way a heartbeat-rate change would cause. */
uint8_t PID_SetTurnonFreqHz(uint32_t hz)
{
    if ((hz < (uint32_t)PID_OUTPUT_MIN_HZ) || (hz >= g_pfmMaxFreqHz))
    {
        return 0U;
    }
    g_pfmTurnonFreqHz = hz;
    return 1U;
}

uint32_t PID_GetTurnonFreqHz(void)
{
    return g_pfmTurnonFreqHz;
}

uint8_t PID_SetMaxFreqHz(uint32_t hz)
{
    if ((hz > (uint32_t)PID_OUTPUT_MAX_HZ) || (hz <= g_pfmTurnonFreqHz))
    {
        return 0U;
    }
    g_pfmMaxFreqHz = hz;
    return 1U;
}

uint32_t PID_GetMaxFreqHz(void)
{
    return g_pfmMaxFreqHz;
}

/* PFM_MAX_CURRENT_A_PER_CHANNEL[channel] -- RUNTIME-CONFIGURABLE as of
   2026-09-22, direct request (see g_pfmMaxCurrentA's own doc comment
   above -- this is the real per-channel calibration ctrlr_config.h's
   "MUST BE CALIBRATED BEFORE FINAL DEPLOYMENT" comment describes).
   Backs CONFig:MAXCURRent (commands.c). `amps` must be > 0 -- a
   channel with a zero or negative full-range current makes
   AmpsToHz()'s own frac = currentA/maxCurrentA divide-by-zero/
   nonsensical, matching PID_SetProfileTiming()'s own "no zero
   durations" convention. */
uint8_t PID_SetMaxCurrentA(uint8_t channel, float amps)
{
    if (channel >= HRTIM_NUM_CHANNELS)
    {
        return 0U;
    }
    if (amps <= 0.0f)
    {
        return 0U;
    }
    g_pfmMaxCurrentA[channel] = amps;
    return 1U;
}

uint8_t PID_GetMaxCurrentA(uint8_t channel, float *amps)
{
    if (channel >= HRTIM_NUM_CHANNELS)
    {
        return 0U;
    }
    if (amps != NULL)
    {
        *amps = g_pfmMaxCurrentA[channel];
    }
    return 1U;
}

/* PID_OUTPUT_MAX_SLEW_HZ_PER_TICK -- RUNTIME-CONFIGURABLE as of
   2026-09-22, direct request. Backs CONFig:SLEWRate (commands.c). See
   g_pidOutputMaxSlewHzPerTick's own doc comment (above, next to
   ClampOutputSlew()) for why this only enforces > 0, no upper
   bound. */
uint8_t PID_SetSlewRateHzPerTick(float hzPerTick)
{
    if (hzPerTick <= 0.0f)
    {
        return 0U;
    }
    g_pidOutputMaxSlewHzPerTick = hzPerTick;
    return 1U;
}

float PID_GetSlewRateHzPerTick(void)
{
    return g_pidOutputMaxSlewHzPerTick;
}

/* FAULT_RAMP_DOWN_TIME_S -- RUNTIME-CONFIGURABLE as of 2026-09-22,
   direct request. Backs CONFig:FaultRampTime (commands.c). Must be
   > 0, same "no zero durations" convention as PID_SetProfileTiming().
   Does NOT retroactively affect a fault ramp-down already in progress
   -- PID_BeginFaultRampDown() only reads this at the moment a fault
   is actually detected, converting to a fixed tick count for that one
   ramp; changing it mid-ramp has no effect until the NEXT fault. */
uint8_t PID_SetFaultRampDownTimeS(float s)
{
    if (s <= 0.0f)
    {
        return 0U;
    }
    g_faultRampDownTimeS = s;
    return 1U;
}

float PID_GetFaultRampDownTimeS(void)
{
    return g_faultRampDownTimeS;
}

uint8_t PID_Start(void)
{
    if (g_running != 0U)
    {
        return 1U;   /* already running -- idempotent */
    }

    for (uint8_t ch = 0U; ch < HRTIM_NUM_CHANNELS; ch++)
    {
        g_ch[ch].integral         = 0.0f;
        g_ch[ch].haveLastMeasured = 0U;
        g_ch[ch].lastMeasuredHz   = 0U;

        /* lastOutputHz starts at PID_OUTPUT_MIN_HZ, not 0 -- added
           2026-09-10 alongside the slew-rate clamp (ClampOutputSlew()
           below): 0 was never a real, physically-achievable output
           value (the hardware floor is PID_OUTPUT_MIN_HZ), so
           starting from it would make the very first tick's slew
           clamp count a fake, oversized "jump" that never actually
           happened, needlessly slowing this channel's first real
           response after PID_Start(). */
        g_ch[ch].lastOutputHz     = (uint32_t)PID_OUTPUT_MIN_HZ;

        /* Best-effort -- see this function's own doc comment in pid.h
           on why a channel with nothing physically connected is not
           treated as an error here. */
        (void)PfmInput_StartContinuous(ch);
    }

    {
        /* Built from each channel's own outputEnabled (added
           2026-09-11, PID_SetChannelEnable()) instead of hardcoded 1 --
           a channel disabled before this PID_Start() call never gets
           its output connected in the first place. A channel disabled
           WHILE already running is handled separately, live, by
           PID_SetChannelEnable() itself calling
           HRTIM1_SetChannelOutputEnable() directly -- this array only
           matters for the initial bring-up here. */
        uint8_t channelEnabled[HRTIM_NUM_CHANNELS];
        for (uint8_t ch = 0U; ch < HRTIM_NUM_CHANNELS; ch++)
        {
            channelEnabled[ch] = g_ch[ch].outputEnabled;
        }
        HRTIM1_PWM_Start(channelEnabled);
    }

    g_running = 1U;
    return 1U;
}

void PID_Stop(void)
{
    if (g_running == 0U)
    {
        return;   /* idempotent */
    }

    HRTIM1_PWM_Stop();

    for (uint8_t ch = 0U; ch < HRTIM_NUM_CHANNELS; ch++)
    {
        PfmInput_StopContinuous(ch);
    }

    g_running      = 0U;
    g_profileActive = 0U;   /* a stop -- fault, SOURce:STOP, or shot completion
                                (see PID_Update()) -- always ends any
                                profile in progress too; an operator
                                must send SHOT:STARt again */

    /* A General-Fault ramp-down (added 2026-09-13, see
       PID_BeginFaultRampDown()) in progress when PID_Stop() is called
       for some OTHER reason (a manual SOURce:STOP abort during the ramp
       -- allowed, deliberately: an operator asking for an immediate
       stop should get one, overriding the graceful ramp) is simply
       abandoned here -- HRTIM1_PWM_Stop() above already disconnects
       every channel's output unconditionally regardless of which ones
       were mid-ramp, so there's nothing left for the ramp to finish
       doing. Clearing this state (rather than leaving it stale) keeps
       a later PID_Start() from getting confused by leftover
       "participating" flags from a ramp that never got to complete
       normally. */
    g_faultRampActive       = 0U;
    g_faultRampElapsedTicks = 0U;
    for (uint8_t ch = 0U; ch < HRTIM_NUM_CHANNELS; ch++)
    {
        g_ch[ch].faultRampParticipating = 0U;
    }
}

uint8_t PID_IsRunning(void)
{
    return g_running;
}

/* General-Fault open-loop ramp-down -- added 2026-09-13, per direct
   instruction. Called ONCE, by state_machine.c's HandleGeneralFault(),
   the instant a General Fault is detected while FIRING with real
   output. Captures each currently-enabled channel's ACTUAL output Hz
   right now (wherever it happened to be in the normal shot profile)
   as that channel's ramp start point, sets up the shared ramp clock,
   and hands off to ProcessFaultRampDown() (PID_Update(), below) to
   actually drive it, one tick at a time, from here on.

   Deliberately does NOT call PID_Stop() itself -- the whole point is
   for g_running to stay 1 and Master's counter to keep running, so
   PID_Update() keeps being called and ProcessFaultRampDown() can keep
   ticking the ramp forward. PID_Stop() only happens once, at the far
   end, when the ramp actually finishes (see ProcessFaultRampDown()).

   Clears g_profileActive outright (not "paused, resume later") -- a
   fault ramp-down replaces the normal shot profile's own trapezoid
   completely; there is no resuming a shot after a fault, an operator
   must clear the fault and start a fresh one. */
void PID_BeginFaultRampDown(void)
{
    /* g_pidLoopRateHz, NOT the PID_LOOP_RATE_HZ compile-time default --
       fixed 2026-09-22 alongside making the loop rate itself runtime-
       configurable (PID_SetLoopRateHz()): this conversion MUST use
       whatever rate the loop is actually running at right now, or a
       changed rate would silently make this ramp run for the wrong
       real-world duration (ticks would still count out at
       g_faultRampTotalTicks, but each tick no longer takes
       1/PID_LOOP_RATE_HZ seconds). Also now reads g_faultRampDownTimeS
       (below), not the compile-time FAULT_RAMP_DOWN_TIME_S default
       directly -- see PID_SetFaultRampDownTimeS()'s own doc comment. */
    g_faultRampTotalTicks = (uint32_t)(g_faultRampDownTimeS * (float)g_pidLoopRateHz);
    if (g_faultRampTotalTicks == 0U)
    {
        g_faultRampTotalTicks = 1U;   /* defensive -- a zero-tick ramp would
                                          divide-by-zero below */
    }
    g_faultRampElapsedTicks = 0U;
    g_profileActive         = 0U;

    uint8_t anyParticipating = 0U;
    for (uint8_t ch = 0U; ch < HRTIM_NUM_CHANNELS; ch++)
    {
        PidChannelState_t *st = &g_ch[ch];
        if (st->outputEnabled != 0U)
        {
            st->faultRampStartHz       = st->lastOutputHz;
            st->faultRampParticipating = 1U;
            anyParticipating           = 1U;
        }
        else
        {
            st->faultRampParticipating = 0U;
        }
    }

    if (anyParticipating == 0U)
    {
        /* Every channel was already disabled/idle -- nothing was
           actually outputting for this fault to have interrupted, so
           there's nothing to ramp. Stop immediately, same as a fault
           caught while IDLE/ARMED. */
        PID_Stop();
        return;
    }

    g_faultRampActive = 1U;
}

/* --------------------------------------------------------------------------
 * Waveform-log helpers -- extracted 2026-09-22 (docs/telemetry.md Phase 3) so
 * the fault-ramp-down path (ProcessFaultRampDown, below) keeps logging with
 * the SAME decimation and timebase as the normal PID_Update() loop, instead of
 * silently freezing the log at the fault boundary. That was a real gap found
 * during the telemetry demo: GENERAL:TEST:FAULT mid-shot ramped the output
 * down gracefully, but LOG:DATA? showed only the pre-fault flat-top.
 * -------------------------------------------------------------------------- */

/* Advance the shared decimation counter once per real tick and report whether
 * THIS tick is a logging tick (a log is armed AND this tick hits the decimation
 * stride AND the cap hasn't been reached). Must be called exactly once per
 * PID_Update() tick -- the normal loop or the fault-ramp path, never both --
 * so g_logDecimCounter advances once per tick either way. */
static uint8_t LogDecimationTick(void)
{
    if (((g_logChannel != 0xFFU) || (g_logAllChannels != 0U)) && (g_logCount < g_logCap))
    {
        g_logDecimCounter++;
        if (g_logDecimCounter >= g_logDecim)
        {
            g_logDecimCounter = 0U;
            return 1U;
        }
    }
    return 0U;
}

/* Write channel `ch`'s {setpoint, measured, output} into the current g_logCount
 * slot, if that channel is the armed one (or all-channels logging is on). Call
 * only when LogDecimationTick() returned 1 this tick. */
static void LogSample(uint8_t ch)
{
    if ((g_logAllChannels != 0U) || (ch == g_logChannel))
    {
        PidChannelState_t *st = &g_ch[ch];
        g_logSetpoint[ch][g_logCount] = st->setpointHz;
        g_logMeasured[ch][g_logCount] = st->lastMeasuredHz;
        g_logOutput[ch][g_logCount]   = st->lastOutputHz;
    }
}

/* Drives one tick of an already-armed General-Fault ramp-down (see
   PID_BeginFaultRampDown() above) -- called from PID_Update(), once
   per Master heartbeat, for as long as g_faultRampActive stays 1.
   OPEN LOOP throughout, per direct instruction: no feedback is
   consumed or considered here at all, purely a linear interpolation
   from each participating channel's own faultRampStartHz down to
   PFM_TURNON_FREQ_HZ (0A), using the SAME elapsed/total-ticks fraction
   convention as every other ramp in this file (PID_StartRamp(),
   TrapezoidalCurrentA()) -- lands exactly on the floor on the final
   tick regardless of rounding, not a fixed per-tick decrement
   accumulated forward -- the UNCLAMPED interpolated value lands
   exactly on PFM_TURNON_FREQ_HZ when elapsed reaches total. Still
   passes through the hard slew-rate clamp (ClampOutputSlew(), same as
   every other output-Hz write in this file) as defense-in-depth --
   for any sane FAULT_RAMP_DOWN_TIME_S this never actually binds (the
   ramp's own math already produces per-tick steps far below
   PID_OUTPUT_MAX_SLEW_HZ_PER_TICK), but if it somehow did, the
   channel's real final value could land a few Hz short of the exact
   floor rather than precisely on it -- harmless, since the channel's
   output is disconnected entirely at the same fixed tick regardless
   (see PID_Stop(), below) and being a few Hz above the floor for one
   tick before disconnecting is not a safety concern. Once every
   participating channel reaches the floor
   (elapsed >= total), calls PID_Stop() -- which unconditionally
   disconnects every channel's HRTIM output regardless of which ones
   were mid-ramp, satisfying "disable the HRTIM output channels" with
   the same mechanism every other stop in this codebase already uses,
   not a new one -- and the controller settles into FAULT, waiting for
   FAULT:CLEAR. */
static void ProcessFaultRampDown(void)
{
    g_faultRampElapsedTicks++;

    float frac = (float)g_faultRampElapsedTicks / (float)g_faultRampTotalTicks;
    uint8_t rampDone = (g_faultRampElapsedTicks >= g_faultRampTotalTicks) ? 1U : 0U;
    if (frac > 1.0f)
    {
        frac = 1.0f;
    }

    for (uint8_t ch = 0U; ch < HRTIM_NUM_CHANNELS; ch++)
    {
        PidChannelState_t *st = &g_ch[ch];
        if (st->faultRampParticipating == 0U)
        {
            continue;
        }

        float startHz = (float)st->faultRampStartHz;
        float endHz   = (float)g_pfmTurnonFreqHz;
        float hzF      = rampDone ? endHz : (startHz + frac * (endHz - startHz));

        float clampedHzF = ClampOutputSlew(hzF, (float)st->lastOutputHz);
        uint32_t outputHz = (uint32_t)clampedHzF;

        st->lastOutputHz = outputHz;
        st->setpointHz   = outputHz;   /* keep SOURce:STATus?'s reported setpoint/output
                                           consistent -- this IS the target now, there's
                                           no separate PID error to report during an
                                           open-loop ramp */

        uint16_t per = (uint16_t)((HRTIM_TIMER_CLK_HZ / outputHz) - 1U);
        HRTIM1_SetChannelPeriod(ch, per);
    }

    /* Log the ramp-down too (2026-09-22, docs/telemetry.md Phase 3): the
       normal per-channel logging loop is skipped once PID_Update() takes the
       FAULT branch, so without this the waveform log freezes at the fault
       boundary and the graceful ramp never shows up in LOG:DATA?.
       Measured is NOT updated during the open-loop ramp (no feedback consumed
       -- see the function's own header comment), so the log records the last
       HELD measured value; setpoint/output both follow the ramp (setpointHz is
       overwritten to the ramp value just above, per the reporting-consistency
       note). Logs only participating channels, same as the loop above. */
    {
        uint8_t logThisTick = LogDecimationTick();
        if (logThisTick != 0U)
        {
            for (uint8_t ch = 0U; ch < HRTIM_NUM_CHANNELS; ch++)
            {
                if (g_ch[ch].faultRampParticipating != 0U)
                {
                    LogSample(ch);
                }
            }
            g_logCount++;
        }
    }

    if (rampDone != 0U)
    {
        g_faultRampActive = 0U;
        PID_Stop();
    }
}

/* OCP (per-channel overcurrent) ramp-down -- see this function's own
   extensive doc comment in pid.h for the 3-step behavior. Implementation
   notes that belong here, not the public header:

   - N (channels enabled at the fault instant, faultedChannel included)
     is counted in its own first pass, BEFORE PID_SetChannelEnable()
     touches anything -- so faultedChannel still counts toward N even
     though it's about to be disabled, matching "the number of channels
     enabled AT THE TIME OF THE FAULT" literally, not "after".

   - The derate step writes directly into st->lastOutputHz/setpointHz and
     HRTIM1_SetChannelPeriod(), the SAME pattern ProcessFaultRampDown()
     itself uses for its own per-tick writes -- this is a single,
     one-time write here (not a per-tick loop), since it happens once, at
     the moment the fault is reported, before the ramp-down even starts.

   - ClampOutputSlew() is applied to the derated value, exactly like
     every other output-Hz write in this file. *** WORTH FLAGGING, NOT
     SILENTLY DECIDED ***: per direct instruction, this step-down should
     happen "simultaneously" with disabling faultedChannel -- but
     ClampOutputSlew() bounds any single tick's change to
     PID_OUTPUT_MAX_SLEW_HZ_PER_TICK (2000 Hz, ctrlr_config.h). A large
     derate on a channel running near PFM_MAX_FREQ_HZ (e.g. N=4,
     100000 Hz -> 75000 Hz is a 25000 Hz cut) CANNOT actually land in one
     tick under that clamp -- ClampOutputSlew() only lets it move 2000 Hz
     this tick, and the true derated level is reached gradually over
     several more ticks instead, governed by the slew rate, not achieved
     as a literal single-tick step. Chose to keep the slew clamp active
     anyway rather than carve out an exception for this one case --
     EVERY other output write in this file, including the General-Fault
     ramp-down itself, goes through this same clamp with zero exceptions
     found anywhere, and it exists specifically because an unclamped
     frequency change is a real, DSLogic-confirmed hardware glitch risk
     (ctrlr_config.h's own PID_OUTPUT_MAX_SLEW_HZ_PER_TICK comment) --
     bypassing it for an "urgent" case felt like exactly the kind of
     reasoning that glitch already disproved once. If literal same-tick
     simultaneity is actually required here, that's a direct, explicit
     decision to make (and likely means raising
     PID_OUTPUT_MAX_SLEW_HZ_PER_TICK for this one path, or accepting the
     glitch risk), not something to assume silently either way. */
void PID_BeginOvercurrentRampDown(uint8_t faultedChannel)
{
    if (faultedChannel >= HRTIM_NUM_CHANNELS)
    {
        return;   /* defensive -- state_machine.c already validates */
    }

    uint8_t n = 0U;
    for (uint8_t ch = 0U; ch < HRTIM_NUM_CHANNELS; ch++)
    {
        if (g_ch[ch].outputEnabled != 0U)
        {
            n++;
        }
    }

    /* Immediate hard disable -- exactly PID_SetChannelEnable(ch, 0)'s
       existing behavior (HRTIM1_SetChannelOutputEnable(), no ramp, no
       slew clamp -- a full disconnect, same path `enable <ch> off`
       already uses). g_running is still 1 here (we're mid-FIRING), so
       this takes effect on real hardware immediately, not just in
       bookkeeping. */
    (void)PID_SetChannelEnable(faultedChannel, 0U);

    if (n > 1U)
    {
        float derateFactor = 1.0f - (1.0f / (float)n);
        for (uint8_t ch = 0U; ch < HRTIM_NUM_CHANNELS; ch++)
        {
            if (ch == faultedChannel)
            {
                continue;
            }
            PidChannelState_t *st = &g_ch[ch];
            if (st->outputEnabled == 0U)
            {
                continue;   /* wasn't contributing output -- nothing to derate */
            }

            float desiredHz = (float)st->lastOutputHz * derateFactor;
            /* Deliberately ClampOutputRangeOnly(), NOT ClampOutputSlew() --
               see that function's own comment for the full justification.
               This lands the derate step on the literal (100*1/N)% target,
               range-clamped to [PID_OUTPUT_MIN_HZ, PID_OUTPUT_MAX_HZ] only,
               skipping the per-tick slew limit for this one write. */
            float clampedHzF = ClampOutputRangeOnly(desiredHz);
            uint32_t outputHz = (uint32_t)clampedHzF;

            st->lastOutputHz = outputHz;
            st->setpointHz   = outputHz;   /* same reporting-consistency reasoning
                                               as ProcessFaultRampDown()'s own write */

            uint16_t per = (uint16_t)((HRTIM_TIMER_CLK_HZ / outputHz) - 1U);
            HRTIM1_SetChannelPeriod(ch, per);
        }
    }

    /* Survivors now ramp from their newly-derated lastOutputHz down to
       the floor, exactly like General Fault -- see PID_BeginFaultRampDown()'s
       own comment. faultedChannel is automatically excluded (outputEnabled
       is now 0, just set above). */
    PID_BeginFaultRampDown();
}

void PID_Update(void)
{
    if (g_running == 0U)
    {
        return;
    }

    /* Hardware fault check, BOTH independent sources -- now routed
       through state_machine.c's SM_PollFaults() (added 2026-09-13),
       which is the one place either fault source actually gets
       latched into the explicit top-level state machine (see
       state_machine.h's own header comment for the full design and
       why this same check ALSO runs from main.c's main loop, not just
       here -- fault detection must work "no matter which state the
       supply is in," including while nothing is firing and this ISR
       isn't even the thing driving PID_Update() calls).

       UPDATED 2026-09-13, General-Fault ramp-down: SM_PollFaults()
       does NOT always stop this channel's ticking anymore the instant
       a fault is detected -- HandleGeneralFault() (state_machine.c),
       called from inside SM_PollFaults() the first tick a fault is
       seen, starts an open-loop ramp-down (PID_BeginFaultRampDown()
       below) INSTEAD of calling PID_Stop() when the fault hit while
       FIRING with real output -- Master must keep running (g_running
       stays 1) for ProcessFaultRampDown() below to keep being called
       every tick until that ramp actually completes. So: once faulted,
       this function now branches on g_faultRampActive instead of
       returning unconditionally -- a channel with nothing to ramp
       (fault while IDLE/ARMED, or the ramp already finished) still
       gets PID_Stop() called somewhere (by HandleGeneralFault()
       directly, or by ProcessFaultRampDown() at the end of a ramp),
       which clears g_running -- so the check at the very top of this
       function (`if (g_running == 0U) return;`) already covers that
       case on the NEXT call; this SM_GetState() check below only
       needs to handle "faulted, ramp not (or no longer) active."
       XrexIo_PollOcpFaults() (xrex_io.h, added 2026-09-17) runs
       alongside it -- OCP is polled, not EXTI-driven (a real EXTI-line
       hardware conflict with the existing GateDriverStatus setup, see
       xrex_io.h's own header comment), so it needs this same real-tick
       cadence to actually catch anything while FIRING. */
    SM_PollFaults();
    XrexIo_PollOcpFaults();
    XrexIo_PollEnableOutputFaults();   /* added 2026-09-17 -- same real-
                                           tick cadence, but itself only
                                           acts while ARMED/FIRING
                                           (xrex_io.h) */
    if (SM_GetState() == SM_STATE_FAULT)
    {
        if (g_faultRampActive != 0U)
        {
            ProcessFaultRampDown();
        }
        return;
    }

    /* Demand profile -- shared shot clock, one elapsed value for every
       channel this tick (see pid.h's "DEMAND PROFILE" section and the
       g_profile* globals' own comment above). Completion is checked
       FIRST, before any channel is touched: per direct instruction,
       end-of-shot means a full stop (PID_Stop(), output off entirely),
       not hold-at-floor -- so this tick does no further work at all
       once the shot's total duration has elapsed. Also notifies the
       state machine (SM_NotifyShotComplete(), added 2026-09-13) --
       this is the automatic, no-fault, no-operator-action shot end,
       FIRING -> IDLE. */
    uint32_t profileElapsedThisTick = 0U;
    if (g_profileActive != 0U)
    {
        if (g_profileElapsedTicks >= g_profileTotalTicks)
        {
            PID_Stop();
            SM_NotifyShotComplete();
            return;
        }
        profileElapsedThisTick = g_profileElapsedTicks;
        g_profileElapsedTicks++;
    }

    /* Waveform log -- decimation/count bookkeeping moved OUT of the
       per-channel loop below and decided ONCE per real tick, 2026-09-10,
       alongside PID_ArmLogAll(): with all-channels logging, every
       channel "qualifies" every tick, so an increment inside the
       per-channel loop would advance g_logDecimCounter/g_logCount once
       PER CHANNEL instead of once per tick, corrupting the decimation
       timebase (see pid.h's own PID_ArmLog() comment on why an exact
       timebase matters here -- the 2026-09-10 log-timing bugfix this
       would otherwise silently reintroduce). Decided here, applied to
       every armed channel identically inside the loop, so every
       channel's sample `i` is written from the SAME real tick -- the
       entire point of PID_ArmLogAll() over running the same shot N
       separate times. */
    uint8_t logThisTick = LogDecimationTick();

    for (uint8_t ch = 0U; ch < HRTIM_NUM_CHANNELS; ch++)
    {
        PidChannelState_t *st = &g_ch[ch];

        /* Output-disabled channel -- added 2026-09-11, PID_SetChannelEnable().
           Completely inert: no setpoint computed, no feedback consumed,
           no PID math, no HRTIM write, no log entry this tick (a
           disabled channel's log simply stops advancing -- the same
           flat/held semantics an unarmed/no-fresh-feedback tick already
           has, nothing new to represent). The channel's actual output
           PIN is already disconnected at the hardware level
           (HRTIM1_SetChannelOutputEnable(), called from
           PID_SetChannelEnable() itself the moment it was disabled) --
           this skip is what stops pid.c from doing meaningless work
           for a channel with nowhere for its output to go. */
        if (st->outputEnabled == 0U)
        {
            continue;
        }

        /* Setpoint: either this tick's profile-computed demand (shared
           timing, this channel's own peak demandCurrentA), or -- when
           no profile is active -- the pre-existing static-setpoint/
           linear-ramp behavior, completely unchanged. The profile
           supersedes PID_StartRamp()'s own ramp outright while active;
           they're mutually exclusive ways of driving setpointHz, never
           combined. */
        if (g_profileActive != 0U)
        {
            float demandA = TrapezoidalCurrentA(profileElapsedThisTick, st->demandCurrentA);
            st->setpointHz = AmpsToHz(ch, demandA);
        }
        else if (st->rampTicksLeft > 0U)
        {
            /* Linear setpoint ramp -- see pid.h's own comment on
               PID_StartRamp(). Interpolated from elapsed/total ticks
               each time (not a fixed per-tick increment accumulated
               forward), so rounding never compounds over the ramp --
               lands exactly on rampEndHz on the final tick regardless
               of how evenly (endHz-startHz) divides by the tick count. */
            uint32_t elapsed = st->rampTotalTicks - st->rampTicksLeft;
            float frac = (float)elapsed / (float)st->rampTotalTicks;
            float interpHz = (float)st->rampStartHz +
                              frac * ((float)st->rampEndHz - (float)st->rampStartHz);
            st->setpointHz = (uint32_t)interpHz;

            st->rampTicksLeft--;
            if (st->rampTicksLeft == 0U)
            {
                st->setpointHz = st->rampEndHz;   /* exact landing, no rounding drift */
            }
        }

        /* Feedback -- windowed average over everything captured since
           this channel's last consume, not a single latest-period
           sample (see pfm_input.h's own doc comment on
           PfmInput_ConsumeAveragePeriod() for the averaging/atomicity
           rationale). Zero samples this window (capture just started,
           nothing physically connected, or -- per direct instruction --
           a genuine zero-edges-this-period failure mode, e.g. a slow-
           responding supply) is reported the same way regardless of
           cause: no fresh measurement to act on. */
        uint32_t avgPeriodTicks = 0U;
        uint16_t sampleCount    = 0U;
        uint8_t  haveFeedback   = PfmInput_ConsumeAveragePeriod(ch, &avgPeriodTicks, &sampleCount);
        uint32_t measuredHz     = haveFeedback ? (HRTIM_TIMER_CLK_HZ / avgPeriodTicks) : 0U;

        if (st->closedLoopEnabled == 0U)
        {
            /* Open-loop -- per direct instruction: write the profile/
               setpoint value straight to HRTIM, NO PID error
               correction at all. Feedback is still consumed above (so
               the averaging accumulator doesn't silently build up
               unbounded across ticks) and still recorded into
               lastMeasuredHz whenever fresh, purely for reporting/
               comparison -- it just never influences this channel's
               output.

               Still passes through the hard slew clamp (ctrlr_config.h's
               PID_OUTPUT_MAX_SLEW_HZ_PER_TICK) -- added 2026-09-10 --
               same as the closed-loop path below: a hard clamp
               protecting real downstream hardware has to apply
               regardless of loop mode, not just where a PID-math bug
               happened to be found. Open-loop's own setpoint source
               (the profile generator, or a plain PID_SetSetpoint())
               is smooth by construction and won't normally be
               affected by this in practice. */
            uint32_t outputHz = (uint32_t)ClampOutputSlew((float)st->setpointHz, (float)st->lastOutputHz);
            st->lastOutputHz  = outputHz;

            if (haveFeedback != 0U)
            {
                st->lastMeasuredHz   = measuredHz;
                st->haveLastMeasured = 1U;
            }

            uint16_t per = (uint16_t)((HRTIM_TIMER_CLK_HZ / outputHz) - 1U);
            HRTIM1_SetChannelPeriod(ch, per);
        }
        else if (haveFeedback == 0U)
        {
            /* Closed-loop, no fresh feedback yet this window -- hold
               whatever HRTIM last had rather than dividing by zero or
               treating "no data" as "zero Hz," which would slam this
               channel's integrator toward the setpoint's full error
               every tick. Per direct instruction (the zero-edges
               failure mode): just skip this channel's PID math/output
               write for the tick -- there can be real millisecond-
               scale delay in the supply's own response.

               REAL, EXPECTED cause on THIS hardware, confirmed
               2026-09-10: PfmInput's continuous capture only drains
               edges out of its raw DMA buffer into the averaging
               accumulator on a half/full-transfer interrupt
               (PFM_DMA_BUF_LEN=32, so every 16 edges -- see
               pfm_input.c). Near PFM_TURNON_FREQ_HZ (~5kHz, ~200us
               period), 16 edges take ~3.2ms to arrive -- slower than
               this 1ms heartbeat -- so several consecutive ticks
               legitimately see zero fresh samples there before the
               next chunk lands all at once. Nothing to "fix" in the
               control loop for this -- it's a real measurement-
               granularity floor at low frequency, not a bug; see
               PID_ArmLog()'s own comment for how the waveform log
               represents it (a genuine held/staircase segment, not a
               skipped one). */
        }
        else
        {
            float error = (float)st->setpointHz - (float)measuredHz;

            /* Provisional integral step -- only actually committed below
               if this update's output isn't saturated (simple clamp
               anti-windup: cheap, robust, easy to reason about -- good
               enough for a first working version; revisit with
               back-calculation anti-windup only if real bench tuning shows
               a real need). */
            float integralNext = st->integral + (error * g_pidDtSec);

            /* Derivative ON MEASUREMENT, not on error -- avoids "derivative
               kick" (a huge transient D term) the instant a setpoint
               changes, standard practice for exactly that reason. Zero on
               this channel's very first update (no prior measurement to
               take a derivative of yet). */
            float derivativeTerm = 0.0f;
            if (st->haveLastMeasured != 0U)
            {
                derivativeTerm = -((float)measuredHz - (float)st->lastMeasuredHz) / g_pidDtSec;
            }
            st->lastMeasuredHz   = measuredHz;
            st->haveLastMeasured = 1U;

            float outputHzF = (st->kp * error) + (st->ki * integralNext) + (st->kd * derivativeTerm);

            /* Two independent clamp stages, both bounding what
               actually reaches HRTIM: first the hardware-register
               range [PID_OUTPUT_MIN_HZ, PID_OUTPUT_MAX_HZ] (unchanged
               since 2026-09-09), then -- added 2026-09-10 -- the hard
               per-tick slew clamp (ClampOutputSlew(), ctrlr_config.h's
               PID_OUTPUT_MAX_SLEW_HZ_PER_TICK). clampedHzF ends up
               holding the FINAL value actually written this tick,
               whichever stage (if any) ended up binding. */
            float rangeClampedHzF = outputHzF;
            if (rangeClampedHzF < (float)PID_OUTPUT_MIN_HZ)
            {
                rangeClampedHzF = (float)PID_OUTPUT_MIN_HZ;
            }
            else if (rangeClampedHzF > (float)PID_OUTPUT_MAX_HZ)
            {
                rangeClampedHzF = (float)PID_OUTPUT_MAX_HZ;
            }
            float clampedHzF = ClampOutputSlew(rangeClampedHzF, (float)st->lastOutputHz);

            /* Commit the integrator step UNLESS output is clamped AND
               integrating would push it further INTO that same rail --
               the real, directional form of simple-clamp anti-windup.
               REAL BUG, caught on the bench, 2026-09-09: an earlier version
               of this check was `if (clampedHzF == outputHzF)` -- freeze
               the integrator any time output is clamped AT ALL, regardless
               of direction. That deadlocks exactly the common case of
               starting near PID_OUTPUT_MIN_HZ with a setpoint well above
               it: output computes small (Kp*error alone, before the
               integral has had any chance to build, is nowhere near enough
               to clear MIN_HZ on its own), gets clamped UP to MIN_HZ, and
               the old check then REFUSED to integrate because output was
               "clamped" -- even though integrating was exactly what would
               have pushed it up and out of the clamp. Confirmed on real
               hardware: output and measured both sat frozen at exactly
               PID_OUTPUT_MIN_HZ indefinitely with a real loopback wired
               (this board's own PFM_Input_01<->HRTIM channel-0 bench
               loopback, still connected from earlier testing) and a
               setpoint/gains combination that should have climbed steadily
               -- error stayed large and positive, integral never moved.
               The fix only blocks integration when it would make the
               saturation WORSE (high-clamped and still-positive error, or
               low-clamped and still-negative error) -- otherwise integrating
               is exactly what should happen to escape the rail.

               EXTENDED 2026-09-10 to cover the slew clamp too, for
               free: this check already compares the FINAL clamped
               value (clampedHzF, now potentially slew-limited as well
               as range-limited) against the raw PID output
               (outputHzF) -- the exact same directional logic
               correctly protects against slew-clamp windup with no
               separate case needed, since "clamped" here has always
               meant "whatever actually got written differs from what
               the raw math wanted," regardless of which stage caused
               that difference. */
            {
                uint8_t blockIntegration =
                    ((clampedHzF > outputHzF) && (error < 0.0f)) ||   /* clamped UP, error wants down */
                    ((clampedHzF < outputHzF) && (error > 0.0f));     /* clamped DOWN, error wants up */

                if (blockIntegration == 0U)
                {
                    st->integral = integralNext;
                }
            }

            uint32_t outputHz = (uint32_t)clampedHzF;
            st->lastOutputHz  = outputHz;

            uint16_t per = (uint16_t)((HRTIM_TIMER_CLK_HZ / outputHz) - 1U);
            HRTIM1_SetChannelPeriod(ch, per);
        }

        /* Waveform log -- see pid.h's own comment on PID_ArmLog() and
           this function's own logThisTick comment above (why the
           decimation decision now happens once per tick, not once per
           channel). Logs whatever st->setpointHz/lastMeasuredHz/
           lastOutputHz currently hold (freshly computed this tick, or
           still held over from the last tick that had feedback -- see
           the branches above) -- a held/stale-feedback stretch shows
           up as a genuine flat/staircase segment, the accurate, honest
           picture of what the control loop is actually doing, rather
           than silently vanishing from the time axis (a REAL bug,
           fixed 2026-09-10 -- see docs/changelog.txt). Every currently-
           armed channel (just g_logChannel, or all of them under
           PID_ArmLogAll()) is written into the SAME g_logCount slot on
           a logging tick -- same real tick, same index, directly
           comparable across channels. */
        if (logThisTick)
        {
            LogSample(ch);
        }
    }

    if (logThisTick)
    {
        g_logCount++;
    }
}

uint8_t PID_SetSetpoint(uint8_t channel, uint32_t setpointHz)
{
    if (channel >= HRTIM_NUM_CHANNELS)
    {
        return 0U;
    }

    if (setpointHz < (uint32_t)PID_OUTPUT_MIN_HZ)
    {
        setpointHz = (uint32_t)PID_OUTPUT_MIN_HZ;
    }
    else if (setpointHz > (uint32_t)PID_OUTPUT_MAX_HZ)
    {
        setpointHz = (uint32_t)PID_OUTPUT_MAX_HZ;
    }

    g_ch[channel].setpointHz = setpointHz;

    /* Cancels any in-progress ramp (PID_StartRamp()) -- otherwise this
       call's effect would just be silently overwritten again on the
       channel's next PID_Update() tick, which is confusing/non-obvious
       at the wire-command layer. Matches this project's "explicit, no
       implicit magic" convention elsewhere (TABLE:BEGIN required
       before a fresh upload, FAULT:CLEAR required even after a
       condition clears). */
    g_ch[channel].rampTicksLeft = 0U;

    return 1U;
}

uint8_t PID_SetGains(uint8_t channel, float kp, float ki, float kd)
{
    if (channel >= HRTIM_NUM_CHANNELS)
    {
        return 0U;
    }

    g_ch[channel].kp = kp;
    g_ch[channel].ki = ki;
    g_ch[channel].kd = kd;

    /* Avoid a discontinuous output jump from an integrator value
       accumulated under the OLD gains suddenly being scaled by new
       ones -- see this function's own doc comment in pid.h. */
    g_ch[channel].integral = 0.0f;

    return 1U;
}

uint8_t PID_GetGains(uint8_t channel, float *kp, float *ki, float *kd)
{
    if (channel >= HRTIM_NUM_CHANNELS)
    {
        return 0U;
    }

    if (kp != NULL) { *kp = g_ch[channel].kp; }
    if (ki != NULL) { *ki = g_ch[channel].ki; }
    if (kd != NULL) { *kd = g_ch[channel].kd; }
    return 1U;
}

uint8_t PID_GetStatus(uint8_t channel, uint32_t *setpointHz,
                      uint32_t *measuredHz, uint32_t *outputHz)
{
    if (channel >= HRTIM_NUM_CHANNELS)
    {
        return 0U;
    }

    if (setpointHz != NULL)
    {
        *setpointHz = g_ch[channel].setpointHz;
    }
    if (measuredHz != NULL)
    {
        *measuredHz = g_ch[channel].lastMeasuredHz;
    }
    if (outputHz != NULL)
    {
        *outputHz = g_ch[channel].lastOutputHz;
    }
    return 1U;
}

/* Shared by PID_ArmLog()/PID_ArmLogAll() -- added 2026-09-22, real bug
   found the hard way on real hardware: a DISABLED channel is skipped
   entirely by PID_Update()'s per-channel loop (st->outputEnabled == 0U
   -> continue, well before the log-write below), so it never writes
   anything into g_logSetpoint/g_logMeasured/g_logOutput for the new
   run -- but re-arming only resets g_logCount/g_logCap/g_logDecim*,
   never the arrays themselves, so a disabled channel's log slots kept
   showing whatever was written there the LAST time that memory was
   used (a completely different, possibly hours-old shot -- real
   output was correctly OFF the whole time, confirmed via a live
   DRIVE_HZ measurement on the simulator staying at 0 throughout, but
   LOG:DATA?/every plot built from it showed a smooth, plausible-
   looking, fully stale trace for that channel, indistinguishable from
   a genuine live one). Zeroing every channel's full log capacity here
   (not just up to `maxSamples` -- cheap, and removes any possibility
   of stale data leaking through regardless of future caller/count
   mismatches) means a disabled channel now reads back as a flat 0 for
   this run -- unambiguous, and matches the actual electrical truth:
   no ENABLE command this run means no data this run either. */
static void ClearLogArrays(void)
{
    for (uint8_t ch = 0U; ch < HRTIM_NUM_CHANNELS; ch++)
    {
        for (uint16_t i = 0U; i < (uint16_t)PID_LOG_MAX_SAMPLES; i++)
        {
            g_logSetpoint[ch][i] = 0U;
            g_logMeasured[ch][i] = 0U;
            g_logOutput[ch][i]   = 0U;
        }
    }
}

uint8_t PID_ArmLog(uint8_t channel, uint16_t maxSamples, uint16_t decim)
{
    if (channel >= HRTIM_NUM_CHANNELS)
    {
        return 0U;
    }

    ClearLogArrays();

    g_logChannel      = channel;
    g_logAllChannels  = 0U;
    g_logCap          = (maxSamples > (uint16_t)PID_LOG_MAX_SAMPLES)
                             ? (uint16_t)PID_LOG_MAX_SAMPLES : maxSamples;
    g_logCount        = 0U;
    g_logDecim        = (decim == 0U) ? 1U : decim;
    g_logDecimCounter = 0U;

    return 1U;
}

/* Arms logging for EVERY channel at once, from the SAME real ticks --
   added 2026-09-10, see this file's own comment on g_logSetpoint/
   g_logMeasured/g_logOutput's 2D shape and PID_Update()'s logThisTick
   for why this needed more than just looping PID_ArmLog() once per
   channel (that would give each channel its OWN decimation timebase,
   not a shared one -- fine for one channel at a time, wrong for a
   simultaneous cross-channel comparison, which is the entire point).
   See pid.h's own comment for the full rationale. */
uint8_t PID_ArmLogAll(uint16_t maxSamples, uint16_t decim)
{
    ClearLogArrays();

    g_logChannel      = 0xFFU;
    g_logAllChannels  = 1U;
    g_logCap          = (maxSamples > (uint16_t)PID_LOG_MAX_SAMPLES)
                             ? (uint16_t)PID_LOG_MAX_SAMPLES : maxSamples;
    g_logCount        = 0U;
    g_logDecim        = (decim == 0U) ? 1U : decim;
    g_logDecimCounter = 0U;

    return 1U;
}

uint16_t PID_GetLogCount(void)
{
    return g_logCount;
}

uint32_t PID_GetLogSampleRateHz(void)
{
    /* *** REAL BUG, FOUND AND FIXED 2026-09-23, via code review, not a
       live symptom anyone had actually hit yet ***: this read the
       compile-time PID_LOOP_RATE_HZ macro, not g_pidLoopRateHz -- once
       CONFig:PIDRate (2026-09-22) made the loop rate genuinely runtime-
       configurable, this meant LOG:DATA?'s own reported sample rate
       silently went stale the instant an operator changed it, with no
       way to notice from the wire protocol (the header value is just
       wrong, not flagged as wrong) -- corrupting the time axis of any
       waveform plot built from it after a rate change. */
    return g_pidLoopRateHz / g_logDecim;
}

/* channel: 0..HRTIM_NUM_CHANNELS-1, added 2026-09-10 alongside the
   row-per-channel log storage (see that comment). Returns NULL for an
   out-of-range channel -- callers (cmd_log_data(), commands.c) are
   expected to validate the wire-level channel argument themselves
   before calling, same convention as everywhere else in this file, so
   this is a defensive backstop, not the primary validation. */
const uint32_t *PID_GetLogMeasured(uint8_t channel)
{
    return (channel < HRTIM_NUM_CHANNELS) ? g_logMeasured[channel] : NULL;
}

const uint32_t *PID_GetLogOutput(uint8_t channel)
{
    return (channel < HRTIM_NUM_CHANNELS) ? g_logOutput[channel] : NULL;
}

const uint32_t *PID_GetLogSetpoint(uint8_t channel)
{
    return (channel < HRTIM_NUM_CHANNELS) ? g_logSetpoint[channel] : NULL;
}

/* Which mode the current (or most recently armed) log is in -- added
   2026-09-10 so cmd_log_data() can apply the right validation rule
   for its optional channel argument (see that function's own comment):
   1 = PID_ArmLogAll() (any channel argument 1..HRTIM_NUM_CHANNELS is
   valid), 0 = PID_ArmLog() (only the single armed channel is valid --
   use PID_GetLogChannel() to find out which, 0xFF if nothing is armed
   at all). */
uint8_t PID_IsLogAllChannels(void)
{
    return g_logAllChannels;
}

uint8_t PID_GetLogChannel(void)
{
    return g_logChannel;
}

uint8_t PID_StartRamp(uint8_t channel, uint32_t startHz, uint32_t endHz, uint32_t durationMs)
{
    if ((channel >= HRTIM_NUM_CHANNELS) || (durationMs == 0U))
    {
        return 0U;
    }

    if (startHz < (uint32_t)PID_OUTPUT_MIN_HZ) { startHz = (uint32_t)PID_OUTPUT_MIN_HZ; }
    if (startHz > (uint32_t)PID_OUTPUT_MAX_HZ) { startHz = (uint32_t)PID_OUTPUT_MAX_HZ; }
    if (endHz   < (uint32_t)PID_OUTPUT_MIN_HZ) { endHz   = (uint32_t)PID_OUTPUT_MIN_HZ; }
    if (endHz   > (uint32_t)PID_OUTPUT_MAX_HZ) { endHz   = (uint32_t)PID_OUTPUT_MAX_HZ; }

    /* Ticks, not milliseconds, is PID_Update()'s own unit (one
       heartbeat = one PID_DT_SEC) -- rounds to the nearest whole tick;
       at least 1, so a very short durationMs never produces a
       zero-tick ramp (which PID_Update()'s elapsed/total division
       would divide-by-zero on). *** REAL BUG, FOUND AND FIXED
       2026-09-23, same class as PID_GetLogSampleRateHz()'s own fix
       above ***: used the compile-time PID_LOOP_RATE_HZ macro, not
       g_pidLoopRateHz -- SOURce:RAMP's own durationMs silently ran 2x
       too fast/slow after a CONFig:PIDRate change, with no way to tell
       from the wire protocol. */
    uint32_t ticks = (durationMs * g_pidLoopRateHz) / 1000U;
    if (ticks == 0U)
    {
        ticks = 1U;
    }

    g_ch[channel].setpointHz     = startHz;   /* takes effect immediately,
                                                  not just from the next tick */
    g_ch[channel].rampStartHz    = startHz;
    g_ch[channel].rampEndHz      = endHz;
    g_ch[channel].rampTotalTicks = ticks;
    g_ch[channel].rampTicksLeft  = ticks;

    return 1U;
}

uint8_t PID_SetLoopMode(uint8_t channel, uint8_t closedLoop)
{
    if (channel >= HRTIM_NUM_CHANNELS)
    {
        return 0U;
    }

    g_ch[channel].closedLoopEnabled = (closedLoop != 0U) ? 1U : 0U;
    return 1U;
}

uint8_t PID_GetLoopMode(uint8_t channel)
{
    if (channel >= HRTIM_NUM_CHANNELS)
    {
        return 1U;   /* safe default (closed-loop) for an out-of-range channel */
    }

    return g_ch[channel].closedLoopEnabled;
}

/* Enables/disables channel `channel`'s output entirely -- added
   2026-09-11, per direct request. Updates the stored state (so the
   NEXT PID_Start() picks it up via channelEnabled[], see that
   function) AND, if the loop is already running, applies it LIVE via
   HRTIM1_SetChannelOutputEnable() -- an operator can disable/re-enable
   a single channel's real output at any time, mid-shot included, not
   only before starting. Disabling does NOT stop this channel's HRTIM
   counter (see HRTIM1_SetChannelOutputEnable()'s own doc comment) and
   does NOT reset its PID state (integral, setpoint, gains all held
   exactly as they were) -- re-enabling resumes from where it left off,
   not from a fresh PID_Start()-like reset. Returns 1 on success, 0 if
   `channel` is out of range. */
uint8_t PID_SetChannelEnable(uint8_t channel, uint8_t enabled)
{
    if (channel >= HRTIM_NUM_CHANNELS)
    {
        return 0U;
    }

    g_ch[channel].outputEnabled = (enabled != 0U) ? 1U : 0U;

    if (g_running != 0U)
    {
        HRTIM1_SetChannelOutputEnable(channel, g_ch[channel].outputEnabled);
    }

    return 1U;
}

uint8_t PID_GetChannelEnable(uint8_t channel)
{
    if (channel >= HRTIM_NUM_CHANNELS)
    {
        return 1U;   /* safe default (enabled) for an out-of-range channel */
    }

    return g_ch[channel].outputEnabled;
}

uint8_t PID_SetChannelNickname(uint8_t channel, const char *name)
{
    size_t len;

    if (channel >= HRTIM_NUM_CHANNELS)
    {
        return 0U;
    }
    if (name == NULL)
    {
        return 0U;
    }

    len = strlen(name);
    if ((len == 0U) || (len > (size_t)PID_CHANNEL_NICKNAME_MAX_LEN))
    {
        return 0U;
    }
    if (strcmp(name, "-") == 0)
    {
        /* reserved -- see this function's own doc comment in pid.h */
        return 0U;
    }

    /* len already checked <= PID_CHANNEL_NICKNAME_MAX_LEN, and the
       buffer is PID_CHANNEL_NICKNAME_MAX_LEN+1 bytes, so this always
       fits with room for the NUL -- strcpy, not strncpy, deliberately:
       a silently-truncated nickname would be a worse failure mode than
       just rejecting an over-length one up front (already done above). */
    (void)strcpy(g_ch[channel].nickname, name);

    return 1U;
}

const char *PID_GetChannelNickname(uint8_t channel)
{
    if (channel >= HRTIM_NUM_CHANNELS)
    {
        return "";
    }

    return g_ch[channel].nickname;
}

uint8_t PID_SetProfileTiming(uint32_t rampUpTimeMs, uint32_t flatTopTimeMs, uint32_t rampDownTimeMs)
{
    if ((rampUpTimeMs == 0U) || (flatTopTimeMs == 0U) || (rampDownTimeMs == 0U))
    {
        return 0U;
    }

    /* Ticks, not ms -- same rounding convention as PID_StartRamp()
       above; at least 1 each, so a very short duration never produces
       a zero-tick phase (TrapezoidalCurrentA()'s own division would
       divide-by-zero on a zero g_profileRampUpTicks/RampDownTicks).
       *** REAL BUG, FOUND AND FIXED 2026-09-23, same class as
       PID_GetLogSampleRateHz()/PID_StartRamp()'s own fixes above ***:
       used the compile-time PID_LOOP_RATE_HZ macro, not
       g_pidLoopRateHz -- SHOT:TIMing's stored durations silently ran
       2x too fast/slow after a CONFig:PIDRate change.

       Narrower issue NOT fixed here, flagged instead: these ticks are
       computed once, at SET time, from whatever rate is current then --
       if CONFig:PIDRate changes AGAIN before the shot actually fires
       (allowed: PID_SetLoopRateHz() only refuses while g_running != 0,
       i.e. mid-shot, not while merely ARMED with timing already set),
       the stored tick counts stay tied to the OLD rate while
       PID_Update() will tick at the NEW one, so the real elapsed wall-
       clock duration would still drift from what was originally
       requested in ms -- a real gap, but a different, deeper design
       question (re-derive ticks at fire time instead of set time?)
       than this specific bug, not resolved here. */
    uint32_t rampUpTicks   = (rampUpTimeMs   * g_pidLoopRateHz) / 1000U;
    uint32_t flatTopTicks  = (flatTopTimeMs  * g_pidLoopRateHz) / 1000U;
    uint32_t rampDownTicks = (rampDownTimeMs * g_pidLoopRateHz) / 1000U;
    if (rampUpTicks == 0U)   { rampUpTicks   = 1U; }
    if (flatTopTicks == 0U)  { flatTopTicks  = 1U; }
    if (rampDownTicks == 0U) { rampDownTicks = 1U; }

    g_profileRampUpTicks   = rampUpTicks;
    g_profileFlatTopTicks  = flatTopTicks;
    g_profileRampDownTicks = rampDownTicks;
    g_profileTotalTicks    = rampUpTicks + flatTopTicks + rampDownTicks;

    return 1U;
}

uint8_t PID_GetProfileTiming(uint32_t *rampUpTimeMs, uint32_t *flatTopTimeMs, uint32_t *rampDownTimeMs)
{
    if ((g_profileRampUpTicks == 0U) && (g_profileFlatTopTicks == 0U) && (g_profileRampDownTicks == 0U))
    {
        return 0U;   /* never successfully set -- see this function's own
                        doc comment in pid.h */
    }

    /* Ticks -> ms, the inverse of PID_SetProfileTiming()'s own
       rounding -- reports the ACTUAL internal tick counts converted
       back, not necessarily bit-exact to whatever fractional-ms value
       an operator originally sent (that rounding already happened
       once, at set time; this is an honest readback of what's really
       active, not a replay of the original input). */
    if (rampUpTimeMs != NULL)
    {
        *rampUpTimeMs = (g_profileRampUpTicks * 1000U) / g_pidLoopRateHz;
    }
    if (flatTopTimeMs != NULL)
    {
        *flatTopTimeMs = (g_profileFlatTopTicks * 1000U) / g_pidLoopRateHz;
    }
    if (rampDownTimeMs != NULL)
    {
        *rampDownTimeMs = (g_profileRampDownTicks * 1000U) / g_pidLoopRateHz;
    }
    return 1U;
}

uint8_t PID_SetProfileCurrent(uint8_t channel, float demandCurrentA)
{
    if (channel >= HRTIM_NUM_CHANNELS)
    {
        return 0U;
    }

    /* Clamped against THIS channel's own full-range current, not a
       single value shared by all four -- see g_pfmMaxCurrentA's own
       comment (2026-09-11) and ctrlr_config.h's PFM_MAX_CURRENT_A_PER_CHANNEL
       ("MUST BE CALIBRATED BEFORE FINAL DEPLOYMENT"). A channel whose
       real full-range current turns out lower than another's will
       correctly refuse a demand beyond ITS OWN ceiling, once that
       ceiling is a real calibrated number instead of today's shared
       placeholder. */
    float maxCurrentA = g_pfmMaxCurrentA[channel];
    if (demandCurrentA < 0.0f)
    {
        demandCurrentA = 0.0f;
    }
    else if (demandCurrentA > maxCurrentA)
    {
        demandCurrentA = maxCurrentA;
    }

    g_ch[channel].demandCurrentA = demandCurrentA;
    return 1U;
}

uint8_t PID_GetProfileCurrent(uint8_t channel, float *demandCurrentA)
{
    if (channel >= HRTIM_NUM_CHANNELS)
    {
        return 0U;
    }

    if (demandCurrentA != NULL)
    {
        *demandCurrentA = g_ch[channel].demandCurrentA;
    }
    return 1U;
}

uint8_t PID_ProfileStart(void)
{
    if ((g_profileRampUpTicks == 0U) && (g_profileFlatTopTicks == 0U) && (g_profileRampDownTicks == 0U))
    {
        /* PID_SetProfileTiming() never called (or it was rejected) --
           refuse rather than run a degenerate zero-length shot. */
        return 0U;
    }

    g_profileElapsedTicks = 0U;
    g_profileActive       = 1U;

    /* Per direct instruction: closed-loop operation (for channels not
       explicitly switched to open-loop, see PID_SetLoopMode()) starts
       the instant PFM output does -- PID_Start() both begins the PFM
       output AND is what makes PID_Update() actually run each
       heartbeat, so there is no separate "now start correcting" step. */
    return PID_Start();
}

uint8_t PID_IsProfileActive(void)
{
    return g_profileActive;
}
