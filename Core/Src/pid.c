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
} PidChannelState_t;

static PidChannelState_t g_ch[HRTIM_NUM_CHANNELS];
static uint8_t g_running = 0U;

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
    }
    g_running = 0U;
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
        g_ch[ch].lastOutputHz     = 0U;

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

    g_running = 0U;
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

    for (uint8_t ch = 0U; ch < HRTIM_NUM_CHANNELS; ch++)
    {
        PidChannelState_t *st = &g_ch[ch];

        uint32_t periodTicks = PfmInput_GetLatestPeriod(ch);
        if (periodTicks == 0U)
        {
            /* No fresh feedback yet (capture just started, or nothing
               physically connected to this channel) -- hold whatever
               HRTIM last had rather than dividing by zero or treating
               "no data" as "zero Hz," which would slam this channel's
               integrator toward the setpoint's full error every tick.
               See PID_Start()'s own doc comment. */
            continue;
        }

        uint32_t measuredHz = HRTIM_TIMER_CLK_HZ / periodTicks;

        /* Linear setpoint ramp -- see pid.h's own comment on
           PID_StartRamp(). Interpolated from elapsed/total ticks each
           time (not a fixed per-tick increment accumulated forward),
           so rounding never compounds over the ramp -- lands exactly
           on rampEndHz on the final tick regardless of how evenly
           (endHz-startHz) divides by the tick count. Advances only on
           a tick with fresh feedback (this function's own `continue`
           above already skipped anything else) -- fine for this
           project's current use (a channel with a ramp armed has
           feedback connected, that's the whole point of running one),
           but worth knowing if a ramp on a feedback-less channel is
           ever wanted: it would stall at rampStartHz until feedback
           arrives, not march forward on the heartbeat alone. */
        if (st->rampTicksLeft > 0U)
        {
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

        float clampedHzF = outputHzF;
        if (clampedHzF < (float)PID_OUTPUT_MIN_HZ)
        {
            clampedHzF = (float)PID_OUTPUT_MIN_HZ;
        }
        else if (clampedHzF > (float)PID_OUTPUT_MAX_HZ)
        {
            clampedHzF = (float)PID_OUTPUT_MAX_HZ;
        }

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
           is exactly what should happen to escape the rail. */
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

        /* Waveform log -- see pid.h's own comment on PID_ArmLog(). Only
           the armed channel, only once every `decim`-th qualifying
           tick (a tick with no fresh feedback already `continue`d
           above, before reaching here, so it never counts toward
           decimation or gets logged -- see that early-return's own
           comment). Logs the SAME measuredHz/outputHz this tick just
           computed and wrote, so "demanded" and "measured" are always
           exactly co-timed, one pair per logged sample. */
        if ((ch == g_logChannel) && (g_logCount < g_logCap))
        {
            g_logDecimCounter++;
            if (g_logDecimCounter >= g_logDecim)
            {
                g_logDecimCounter = 0U;
                g_logSetpoint[g_logCount] = st->setpointHz;
                g_logMeasured[g_logCount] = measuredHz;
                g_logOutput[g_logCount]   = outputHz;
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
