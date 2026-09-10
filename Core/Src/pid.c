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
} PidChannelState_t;

static PidChannelState_t g_ch[HRTIM_NUM_CHANNELS];
static uint8_t g_running = 0U;

/* --------------------------------------------------------------------------
 * Demand profile -- SHARED shot clock. Deliberately ONE global elapsed-
 * tick counter, not one per channel, so every channel's ramp/flat-top/
 * ramp shape stays perfectly synchronized in time even though each
 * channel has its own peak demandCurrentA (see pid.h). Ticks, not ms,
 * same convention as the pre-existing rampTicksLeft feature above.
 * -------------------------------------------------------------------------- */
static uint8_t  g_profileActive       = 0U;
static uint32_t g_profileElapsedTicks = 0U;
static uint32_t g_profileRampTicks    = 0U;   /* one ramp's length (up == down) */
static uint32_t g_profileFlatTopTicks = 0U;
static uint32_t g_profileTotalTicks   = 0U;   /* 2*rampTicks + flatTopTicks */

/* Fixed control-loop sample interval, in seconds -- the whole point of
   keeping Master as a fixed-rate heartbeat instead of a self-clocked
   per-channel design, see pid.h's own header comment. Computed once as
   a compile-time constant, not re-derived every call. */
#define PID_DT_SEC  (1.0f / (float)PID_LOOP_RATE_HZ)

/* Waveform log -- see pid.h's own comment block on PID_ArmLog(). Plain
   parallel arrays (SoA), matching pfm_input.c's own period[] array
   convention rather than an array of structs. Setpoint included
   (2026-09-10, alongside PID_StartRamp()) so a moving reference
   trajectory -- not just a static setpoint -- shows up in the log
   too; before ramps existed, setpoint was constant for the whole log
   anyway and wasn't worth logging. */
static uint32_t g_logSetpoint[PID_LOG_MAX_SAMPLES];
static uint32_t g_logMeasured[PID_LOG_MAX_SAMPLES];
static uint32_t g_logOutput[PID_LOG_MAX_SAMPLES];
static uint16_t g_logCount   = 0U;
static uint16_t g_logCap     = 0U;
static uint8_t  g_logChannel = 0xFFU;   /* 0xFF = no channel armed */
static uint16_t g_logDecim   = 1U;
static uint16_t g_logDecimCounter = 0U;

/* --------------------------------------------------------------------------
 * Demand profile helpers -- see pid.h's "DEMAND PROFILE" doc section.
 * -------------------------------------------------------------------------- */

/* This channel's target current (Amps), on the shared trapezoidal
   shot shape, at `elapsedTicks` into the shot -- 0 -> linear up-ramp
   -> demandCurrentA -> flat-top -> linear down-ramp -> 0. Only ever
   called with elapsedTicks < g_profileTotalTicks (PID_Update() checks
   shot completion BEFORE calling this, see there) -- g_profileRampTicks
   is guaranteed nonzero whenever g_profileActive, since
   PID_SetProfileTiming() refuses a zero ramp, so the divisions below
   are safe. Interpolated from elapsed/total each call (not a fixed
   per-tick increment accumulated forward), matching the existing
   PID_StartRamp() convention just above -- lands exactly on the
   flat-top/zero boundaries regardless of how evenly the durations
   divide into whole ticks. */
static float TrapezoidalCurrentA(uint32_t elapsedTicks, float demandCurrentA)
{
    if (elapsedTicks < g_profileRampTicks)
    {
        float frac = (float)elapsedTicks / (float)g_profileRampTicks;
        return demandCurrentA * frac;
    }

    uint32_t flatEndTicks = g_profileRampTicks + g_profileFlatTopTicks;
    if (elapsedTicks < flatEndTicks)
    {
        return demandCurrentA;
    }

    /* Down-ramp. */
    uint32_t downElapsed = elapsedTicks - flatEndTicks;
    float frac = (float)downElapsed / (float)g_profileRampTicks;
    if (frac > 1.0f)
    {
        frac = 1.0f;   /* defensive only -- PID_Update()'s completion
                           check should always catch this first */
    }
    return demandCurrentA * (1.0f - frac);
}

/* Amps -> Hz, LINEAR PLACEHOLDER -- see ctrlr_config.h's own extensive
   comment on PFM_TURNON_FREQ_HZ/PFM_MAX_FREQ_HZ/PFM_MAX_CURRENT_A for
   why this is flagged as provisional (real Transrex SCR/phase-control
   physics may not be linear) and left for post-characterization
   revisit. Per direct instruction, 0A maps to EXACTLY
   PFM_TURNON_FREQ_HZ (not some frequency below it -- there is no
   "off but nonzero" output state between 0A and turn-on). Clamps
   currentA to [0, PFM_MAX_CURRENT_A] first (a profile's own math
   should never produce outside that range, but this is the last line
   of defense before a value reaches hardware) and the resulting Hz to
   [PID_OUTPUT_MIN_HZ, PID_OUTPUT_MAX_HZ] (the hardware-register safety
   clamp -- PFM_TURNON_FREQ_HZ/PFM_MAX_FREQ_HZ are documented to nest
   inside that range, this is defense-in-depth, not expected to ever
   actually bind). */
static uint32_t AmpsToHz(float currentA)
{
    if (currentA < 0.0f)
    {
        currentA = 0.0f;
    }
    else if (currentA > (float)PFM_MAX_CURRENT_A)
    {
        currentA = (float)PFM_MAX_CURRENT_A;
    }

    float frac = currentA / (float)PFM_MAX_CURRENT_A;
    float hzF = (float)PFM_TURNON_FREQ_HZ +
                frac * ((float)PFM_MAX_FREQ_HZ - (float)PFM_TURNON_FREQ_HZ);

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

/* Hard per-tick output slew-rate clamp -- see ctrlr_config.h's own
   extensive comment on PID_OUTPUT_MAX_SLEW_HZ_PER_TICK for the full
   rationale (a REAL, DSLogic-confirmed single-tick output glitch,
   2026-09-10). Bounds `desiredHz` to within
   +/-PID_OUTPUT_MAX_SLEW_HZ_PER_TICK of `prevHz` (the previous tick's
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
    float maxDelta = (float)PID_OUTPUT_MAX_SLEW_HZ_PER_TICK;
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
    }
    g_running               = 0U;
    g_profileActive          = 0U;
    g_profileElapsedTicks    = 0U;
    g_profileRampTicks       = 0U;
    g_profileFlatTopTicks    = 0U;
    g_profileTotalTicks      = 0U;
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
        uint8_t channelEnabled[HRTIM_NUM_CHANNELS];
        for (uint8_t ch = 0U; ch < HRTIM_NUM_CHANNELS; ch++)
        {
            channelEnabled[ch] = 1U;
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
    g_profileActive = 0U;   /* a stop -- fault, PID:STOP, or shot completion
                                (see PID_Update()) -- always ends any
                                profile in progress too; an operator
                                must send PID:PROFILE:START again */
}

uint8_t PID_IsRunning(void)
{
    return g_running;
}

void PID_Update(void)
{
    if (g_running == 0U)
    {
        return;
    }

    /* Hardware fault check (PC10/HRTIM1_FLT6, see hrtim.h) -- carried
       over from pfm.c's PFM_CycleBoundaryHandler(), which this ISR
       used to call. By the time this ever reads tripped, HRTIM has
       ALREADY forced every fault-enabled channel's outputs to their
       safe level autonomously, in silicon -- this is bookkeeping only:
       stop the Master counter (PID_Stop(), via HRTIM1_PWM_Stop()) so
       this ISR doesn't keep firing forever at PID_LOOP_RATE_HZ
       underneath outputs that are already safed, and get PID_IsRunning()
       out of a stale "running" state. Does NOT clear the fault latch
       itself -- unlike pfm.c's fault path, there is no FAULT:CLEAR-
       gated command wired to this yet (see docs/changelog.txt's
       "explicitly NOT yet resolved" list); commands.c's existing
       FAULT?/FAULT:CLEAR already work for status/clearing the latch,
       just nothing currently calls PID_Start() again afterward -- an
       operator must do that explicitly once a real command exists. */
    if (HRTIM1_FaultIsTripped() != 0U)
    {
        PID_Stop();
        return;
    }

    /* Demand profile -- shared shot clock, one elapsed value for every
       channel this tick (see pid.h's "DEMAND PROFILE" section and the
       g_profile* globals' own comment above). Completion is checked
       FIRST, before any channel is touched: per direct instruction,
       end-of-shot means a full stop (PID_Stop(), output off entirely),
       not hold-at-floor -- so this tick does no further work at all
       once the shot's total duration has elapsed. */
    uint32_t profileElapsedThisTick = 0U;
    if (g_profileActive != 0U)
    {
        if (g_profileElapsedTicks >= g_profileTotalTicks)
        {
            PID_Stop();
            return;
        }
        profileElapsedThisTick = g_profileElapsedTicks;
        g_profileElapsedTicks++;
    }

    for (uint8_t ch = 0U; ch < HRTIM_NUM_CHANNELS; ch++)
    {
        PidChannelState_t *st = &g_ch[ch];

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
            st->setpointHz = AmpsToHz(demandA);
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
            float integralNext = st->integral + (error * PID_DT_SEC);

            /* Derivative ON MEASUREMENT, not on error -- avoids "derivative
               kick" (a huge transient D term) the instant a setpoint
               changes, standard practice for exactly that reason. Zero on
               this channel's very first update (no prior measurement to
               take a derivative of yet). */
            float derivativeTerm = 0.0f;
            if (st->haveLastMeasured != 0U)
            {
                derivativeTerm = -((float)measuredHz - (float)st->lastMeasuredHz) / PID_DT_SEC;
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

        /* Waveform log -- see pid.h's own comment on PID_ArmLog().
           REAL BUG, caught reviewing a shot-profile log against the
           real elapsed shot duration, 2026-09-10: this used to sit
           INSIDE the "have fresh feedback" branches above and never
           run at all on a held/skipped tick (see the branch above) --
           meaning g_logDecimCounter only ever advanced on ticks WITH
           fresh feedback, so PID_GetLogSampleRateHz()'s reported
           `PID_LOOP_RATE_HZ / decim` was only actually correct when
           every tick had fresh feedback. Near PFM_TURNON_FREQ_HZ (see
           the "no fresh feedback" branch's own comment above) that
           assumption silently breaks -- several ticks in a row produce
           no log entry at all, and the *next* logged sample's assumed
           timestamp (index * decim / rate) ends up compressed relative
           to when it was actually taken, distorting a plotted time
           axis without any error being raised. Confirmed on real
           hardware: a 3.0s shot's log stopped 611 samples short of the
           armed 1000 (PID:LOG 1 1000 3), i.e. covering only ~1.83s of
           the log's own claimed timebase for what was actually a full
           3.0s run.

           Fixed by moving this block OUTSIDE the feedback-freshness
           branching entirely -- it now runs once per REAL Master tick
           for the armed channel, unconditionally, logging whatever
           st->setpointHz/lastMeasuredHz/lastOutputHz currently hold
           (freshly computed this tick, or still held over from the
           last tick that had feedback -- see the branches above). This
           makes a held/stale-feedback stretch show up as a genuine
           flat/staircase segment in the log, which is the accurate,
           honest picture of what the control loop is actually doing,
           rather than silently vanishing from the time axis. */
        if ((ch == g_logChannel) && (g_logCount < g_logCap))
        {
            g_logDecimCounter++;
            if (g_logDecimCounter >= g_logDecim)
            {
                g_logDecimCounter = 0U;
                g_logSetpoint[g_logCount] = st->setpointHz;
                g_logMeasured[g_logCount] = st->lastMeasuredHz;
                g_logOutput[g_logCount]   = st->lastOutputHz;
                g_logCount++;
            }
        }
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

uint8_t PID_ArmLog(uint8_t channel, uint16_t maxSamples, uint16_t decim)
{
    if (channel >= HRTIM_NUM_CHANNELS)
    {
        return 0U;
    }

    g_logChannel      = channel;
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
    return (uint32_t)PID_LOOP_RATE_HZ / g_logDecim;
}

const uint32_t *PID_GetLogMeasured(void)
{
    return g_logMeasured;
}

const uint32_t *PID_GetLogOutput(void)
{
    return g_logOutput;
}

const uint32_t *PID_GetLogSetpoint(void)
{
    return g_logSetpoint;
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
       would divide-by-zero on). */
    uint32_t ticks = (durationMs * (uint32_t)PID_LOOP_RATE_HZ) / 1000U;
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

uint8_t PID_SetProfileTiming(uint32_t rampTimeMs, uint32_t flatTopTimeMs)
{
    if ((rampTimeMs == 0U) || (flatTopTimeMs == 0U))
    {
        return 0U;
    }

    /* Ticks, not ms -- same rounding convention as PID_StartRamp()
       above; at least 1 each, so a very short duration never produces
       a zero-tick phase (TrapezoidalCurrentA()'s own division would
       divide-by-zero on a zero g_profileRampTicks). */
    uint32_t rampTicks    = (rampTimeMs    * (uint32_t)PID_LOOP_RATE_HZ) / 1000U;
    uint32_t flatTopTicks = (flatTopTimeMs * (uint32_t)PID_LOOP_RATE_HZ) / 1000U;
    if (rampTicks == 0U)    { rampTicks    = 1U; }
    if (flatTopTicks == 0U) { flatTopTicks = 1U; }

    g_profileRampTicks    = rampTicks;
    g_profileFlatTopTicks = flatTopTicks;
    g_profileTotalTicks   = (2U * rampTicks) + flatTopTicks;   /* up-ramp + flat-top + down-ramp */

    return 1U;
}

uint8_t PID_GetProfileTiming(uint32_t *rampTimeMs, uint32_t *flatTopTimeMs)
{
    if ((g_profileRampTicks == 0U) && (g_profileFlatTopTicks == 0U))
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
    if (rampTimeMs != NULL)
    {
        *rampTimeMs = (g_profileRampTicks * 1000U) / (uint32_t)PID_LOOP_RATE_HZ;
    }
    if (flatTopTimeMs != NULL)
    {
        *flatTopTimeMs = (g_profileFlatTopTicks * 1000U) / (uint32_t)PID_LOOP_RATE_HZ;
    }
    return 1U;
}

uint8_t PID_SetProfileCurrent(uint8_t channel, float demandCurrentA)
{
    if (channel >= HRTIM_NUM_CHANNELS)
    {
        return 0U;
    }

    if (demandCurrentA < 0.0f)
    {
        demandCurrentA = 0.0f;
    }
    else if (demandCurrentA > (float)PFM_MAX_CURRENT_A)
    {
        demandCurrentA = (float)PFM_MAX_CURRENT_A;
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
    if ((g_profileRampTicks == 0U) && (g_profileFlatTopTicks == 0U))
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
