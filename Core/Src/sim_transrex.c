#include "sim_transrex.h"
#include "ctrlr_config.h"
#include "hrtim.h"
#include "pfm_input.h"
#include "gate_driver.h"
#include "xrex_io.h"
#include "stm32g4xx_hal.h"
#include <math.h>

/* Not M_PI -- newlib's math.h on this bare-metal ARM toolchain doesn't
   reliably define it without extra feature-test macros; a local
   constant avoids that portability question entirely. */
#define SIM_TRANSREX_PI   (3.14159265358979323846f)

/*
 * sim_transrex.c
 *
 * See sim_transrex.h for the full design writeup (DRIVE capture ->
 * filter -> FEEDBACK generation, ENA_OUT/CONTACT_OUT gating, fault
 * injection) and its KNOWN LIMITATION note (this build still runs the
 * full controller state machine unconditionally -- don't
 * SOURce:ENAble the simulator's own channels).
 *
 * Pin-role table for this file's own reference (docs/
 * pin_mapping_reference.tex Section 7 is the authoritative source --
 * this is a summary, not a redefinition):
 *
 *   PE0..PE7  (GateDriverStatus_01..08, read via GateDriver_Read())
 *             -- on the CONTROLLER, XR1..4's own _WATER_FLT/_TMP_FLT
 *             inputs; on THIS board (simulator), repurposed as the
 *             RECEIVE side of the controller's own GPOut_01..08
 *             (ENA_OUT ch1..4 = bits 0..3, CONTACT_OUT ch1..4 = bits
 *             4..7) -- this module's ENA_OUT/CONTACT_OUT GATING INPUT.
 *   PG0..PG7  (this board's own GPOut_01..08, XrexIo_Set/GetEnableOutput()/
 *             Set/GetContactorOutput()) -- on the CONTROLLER, its own
 *             XRn_ENA_OUT/CONTACT_OUT transmit pins; on THIS board,
 *             repurposed as the Water+Temp (GPOut_01..04) and Enerpro
 *             (GPOut_05..08) FAULT-INJECTION TRANSMITTERS this module
 *             drives via SimTransrex_SetFaultWaterTemp()/SetFaultEnerpro()
 *             -- the SAME physical pins/registers XREX:CHANnel:ENAOut/
 *             CONTactOut (commands.c) can also drive directly; don't
 *             use both mechanisms on the same channel at once, they'd
 *             fight over the same GPIO.
 *   PG8/PG9, PD0/PD1  (GPOut_09..12, DIAGnostic:GPOut09-12) -- this
 *             module's OCP fault-injection transmitters
 *             (SimTransrex_SetFaultOcp()), driven directly by this
 *             file (no dedicated xrex_io.c setter exists for these,
 *             see commands.c's cmd_diag_gpout09..12()). Same
 *             don't-fight-the-raw-command caveat as above.
 *   PA15/PD4/PB2/PC12 (PFM_Input_01..04, pfm_input.c) -- this board's
 *             own XR1..4_FEEDBACK inputs, receiving the CONTROLLER's
 *             XRn_DRIVE -- this module's DRIVE CAPTURE input.
 *   this board's own XRn_DRIVE (HRTIM channels 0..3, same physical
 *             HRTIM1_SetChannelPeriod() pid.c uses on the controller)
 *             -- this module's FEEDBACK GENERATION output, received by
 *             the controller as ITS XRn_FEEDBACK.
 */

static float    s_filteredHz[HRTIM_NUM_CHANNELS];        /* clean, ripple-free
                                                              low-pass-filtered
                                                              average -- the
                                                              filter's OWN
                                                              recursive memory,
                                                              deliberately never
                                                              has ripple mixed
                                                              into it (see
                                                              SimTransrex_Update()'s
                                                              own comment on
                                                              why) */
static uint32_t s_lastOutputHz[HRTIM_NUM_CHANNELS];       /* what actually got
                                                              written to HRTIM
                                                              this tick --
                                                              s_filteredHz PLUS
                                                              ripple while
                                                              gated. This is
                                                              what SIM:CHANnel:
                                                              STATus?'s
                                                              FEEDBACK_HZ and
                                                              the waveform log
                                                              report -- the
                                                              real, physical
                                                              signal, not the
                                                              filter's own
                                                              internal state */
static uint32_t s_lastMeasuredHz[HRTIM_NUM_CHANNELS];
static uint8_t  s_haveMeasuredSinceGate[HRTIM_NUM_CHANNELS];   /* 0 until the
                                                                    first real
                                                                    PfmInput
                                                                    sample
                                                                    lands after
                                                                    THIS gating
                                                                    on-edge --
                                                                    see the
                                                                    2026-09-21
                                                                    fix note in
                                                                    docs/
                                                                    changelog.txt:
                                                                    without
                                                                    this, a
                                                                    freshly-
                                                                    gated
                                                                    channel's
                                                                    filter
                                                                    target fell
                                                                    back to
                                                                    s_lastMeasuredHz's
                                                                    stale 0
                                                                    (held there
                                                                    every tick
                                                                    while
                                                                    ungated,
                                                                    correctly,
                                                                    for DRIVE_HZ
                                                                    reporting)
                                                                    instead of
                                                                    the healthy
                                                                    0A floor */
static uint8_t  s_faultWaterTemp[HRTIM_NUM_CHANNELS];
static uint8_t  s_faultEnerpro[HRTIM_NUM_CHANNELS];
static uint8_t  s_faultOcp[HRTIM_NUM_CHANNELS];
static uint8_t  s_outputConnected[HRTIM_NUM_CHANNELS];   /* last state actually
                                                              passed to
                                                              HRTIM1_SetChannelOutputEnable(),
                                                              so Update() only
                                                              calls it on a real
                                                              gating EDGE -- see
                                                              that function's own
                                                              doc comment: the
                                                              enable==1 branch
                                                              forces a one-time
                                                              output-level reset
                                                              every call, which
                                                              would glitch an
                                                              already-connected,
                                                              actively-PWMing pair
                                                              if called every
                                                              main-loop tick
                                                              instead of once per
                                                              transition */
static uint32_t s_tauMs = SIM_TRANSREX_DEFAULT_TAU_MS;
static uint32_t s_lastUpdateTick;

/* Idle tone -- added 2026-09-21 as an opt-in diagnostic ("for the sake
 * of visual testing... output a baseline frequency of 3kHz and hold it
 * when not actively in a shot"), DEFAULT FLIPPED TO ON 2026-09-22 per
 * direct instruction: the simulator should be "pretty passive" out of
 * the box -- 3kHz on every channel with no serial setup at all, then
 * respond to the controller's real PFM drive once ENA_OUT+CONTACT_OUT
 * are both asserted (already unconditional/hardware-gated, no serial
 * needed for that part either -- see SimTransrex_Update()'s own
 * comment). Boots ON (1) now, so idle tone IS the default ungated
 * steady state, not a special diagnostic mode someone has to remember
 * to turn on. SIM:DIAGnostic:IDLETONE (commands.c) still exists to
 * turn it OFF live, board-wide (all 4 channels at once), for the rare
 * case where the original stricter realism (ungated = physically
 * DISCONNECTED, no output at all) is actually wanted for a specific
 * test. When ON, an UNGATED channel's output stays CONNECTED and the
 * filter targets SIM_TRANSREX_IDLE_TONE_HZ instead of
 * PFM_TURNON_FREQ_HZ. A GATED channel (real shot) is completely
 * unaffected either way -- this only changes the ungated/idle state. */
static uint8_t s_idleToneEnabled = 1U;

/* Waveform log -- see sim_transrex.h's own extensive comment on why
 * this differs from pid.h's PID_ArmLog() (no fixed tick to decimate
 * against; explicit per-sample timestamp instead). Single-channel-at-
 * a-time, matching PID_ArmLog()'s own simpler (non-"All") shape.
 * s_logChannel == 0xFF means nothing is currently armed. Plain static
 * arrays, not dynamically sized -- same "cheap against this MCU's
 * 128KiB SRAM" reasoning as pid.c's own g_logSetpoint/Measured/Output
 * (48000 bytes there; this is 2000*3*4 = 24000 bytes, well within
 * budget). */
static uint32_t s_logTimeMs[SIM_LOG_MAX_SAMPLES];
static uint32_t s_logDriveHz[SIM_LOG_MAX_SAMPLES];
static uint32_t s_logFeedbackHz[SIM_LOG_MAX_SAMPLES];
static uint16_t s_logCount;
static uint16_t s_logMaxSamples;
static uint16_t s_logMinIntervalMs;
static uint32_t s_logArmTimeMs;
static uint32_t s_logLastSampleMs;
static uint8_t  s_logChannel = 0xFFU;

/* Direct GPIO for the OCP fault transmitters (GPOut_09..12 -- PG8/PG9/
 * PD0/PD1), channel-indexed 0..3 = XR1..4, matching the finalized
 * transmitter table (docs/pin_mapping_reference.tex Section 7). No
 * dedicated xrex_io.c setter exists for these (unlike Water+Temp/
 * Enerpro's XrexIo_Set*Output(), which this file reuses directly) --
 * they were added purely as generic DIAGnostic:GPOut09-12 outputs
 * (commands.c), so this module owns its own small HAL_GPIO_WritePin()
 * pair here rather than adding a misleadingly-named xrex_io.c
 * function for a pin pair that has no real meaning on the controller
 * itself. `faulted != 0` drives LOW (FAULT_POLARITY_NORMALLY_HIGH's
 * fault level); 0 drives HIGH (healthy). Out-of-range channel is a
 * no-op. */
static void DriveOcpFaultPin(uint8_t channel, uint8_t faulted)
{
    GPIO_PinState level = (faulted != 0U) ? GPIO_PIN_RESET : GPIO_PIN_SET;

    switch (channel)
    {
        case 0U: HAL_GPIO_WritePin(GPIOG, GPIO_PIN_8, level); break;   /* GPOut_09 -> XR1_OCP */
        case 1U: HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, level); break;   /* GPOut_10 -> XR2_OCP */
        case 2U: HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, level); break;   /* GPOut_11 -> XR3_OCP */
        case 3U: HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, level); break;   /* GPOut_12 -> XR4_OCP */
        default: break;   /* out of range, no-op */
    }
}

void SimTransrex_Init(void)
{
    uint8_t channelEnabled[HRTIM_NUM_CHANNELS];
    uint8_t ch;

    for (ch = 0U; ch < HRTIM_NUM_CHANNELS; ch++)
    {
        (void)PfmInput_StartContinuous(ch);

        s_filteredHz[ch]           = (float)PFM_TURNON_FREQ_HZ;
        s_lastOutputHz[ch]         = PFM_TURNON_FREQ_HZ;
        s_lastMeasuredHz[ch]       = 0U;
        s_haveMeasuredSinceGate[ch] = 0U;

        s_faultWaterTemp[ch]   = 0U;
        s_faultEnerpro[ch]     = 0U;
        s_faultOcp[ch]         = 0U;
        s_outputConnected[ch]  = 0U;   /* starts DISCONNECTED -- see
                                            SimTransrex_Update()'s own
                                            comment: per direct
                                            instruction, a channel with
                                            no ENA_OUT/CONTACT_OUT
                                            asserted must output nothing
                                            (stay LOW), not a floored
                                            5kHz PFM signal */

        /* Healthy = HIGH on every fault transmitter, per the confirmed
           "default healthy" scope decision -- see this file's header
           and sim_transrex.h's own SimTransrex_Init() doc comment for
           why this is a deliberate exception to this project's usual
           boot-LOW convention. */
        XrexIo_SetEnableOutput(ch, 1U);
        XrexIo_SetContactorOutput(ch, 1U);
        DriveOcpFaultPin(ch, 0U);

        channelEnabled[ch] = 0U;   /* counters start running (below) but
                                       no channel is CONNECTED yet --
                                       SimTransrex_Update() connects each
                                       one independently the first time
                                       it actually sees that channel
                                       gated on */
    }

    /* Starts this board's own HRTIM Master+slave counters running
       UNCONDITIONALLY (not tied to this board's own ARM/FIRE state --
       see this file's/sim_transrex.h's own comment: a real Transrex's
       response is gated by ENA_OUT/CONTACT_OUT, not by an internal
       "shot" concept) -- but with every channel PASSED AS DISABLED
       (channelEnabled[] above), so no output pin is actually driven
       yet: "counter running, output disconnected" is exactly
       HRTIM1_PWM_Start()'s own documented meaning for a 0 entry.
       SimTransrex_Update() is what connects/disconnects each channel's
       actual output pins, live, per its own gating state. */
    HRTIM1_PWM_Start(channelEnabled);

    s_tauMs         = SIM_TRANSREX_DEFAULT_TAU_MS;
    s_lastUpdateTick = HAL_GetTick();
}

void SimTransrex_Update(void)
{
    uint32_t now    = HAL_GetTick();
    uint32_t dtMs    = now - s_lastUpdateTick;   /* unsigned wraparound-safe
                                                      across HAL_GetTick()'s
                                                      own 32-bit rollover */
    uint16_t rawGds = GateDriver_Read();
    float    alpha  = (float)dtMs / ((float)s_tauMs + (float)dtMs);
    float    nowSec = fmodf((float)now / 1000.0f, 1.0f);   /* shared time base
                                                    for the ripple waveform below
                                                    -- computed once, not per-
                                                    channel, so every channel's
                                                    ripple stays phase-locked
                                                    together (matching a real
                                                    Transrex system, where every
                                                    channel's ripple derives from
                                                    the SAME AC line). Wrapped to
                                                    [0,1)s -- EXACT, not an
                                                    approximation (720Hz and
                                                    1440Hz both complete a whole
                                                    number of cycles every
                                                    second) -- keeps the sinf()
                                                    argument below from growing
                                                    unbounded with HAL_GetTick()'s
                                                    own uptime, which would
                                                    otherwise lose real angle
                                                    precision in float (24-bit
                                                    mantissa) after a long-
                                                    running session. */
    uint8_t  ch;

    s_lastUpdateTick = now;

    for (ch = 0U; ch < HRTIM_NUM_CHANNELS; ch++)
    {
        uint8_t enaGated     = (uint8_t)((rawGds >> ch) & 0x1U);
        uint8_t contactGated = (uint8_t)((rawGds >> (4U + ch)) & 0x1U);
        uint8_t gated        = (uint8_t)(enaGated && contactGated);
        float   targetHz;

        /* Physical output connection tracks live gating, per direct
           instruction: no ENA_OUT (per channel) means that channel's
           output is gated off and stays LOW; independently, no
           CONTACT_OUT means the same -- EITHER missing signal alone
           gates this channel off (the `&&` above), and this is fully
           independent per channel (4 channels, not one shared gate).
           Only called on an actual transition (see s_outputConnected's
           own comment above) -- HRTIM1_SetChannelOutputEnable(ch,1)
           forces a one-time output-level reset every call, which would
           glitch an already-connected, actively-PWMing channel if
           called on every tick instead of once per edge. */
        {
            uint8_t desiredConnected = (uint8_t)(gated != 0U ? 1U :
                                                   (s_idleToneEnabled != 0U ? 1U : 0U));

            if (desiredConnected != s_outputConnected[ch])
            {
                HRTIM1_SetChannelOutputEnable(ch, desiredConnected);
                s_outputConnected[ch] = desiredConnected;

                if (gated != 0U)
                {
                    s_haveMeasuredSinceGate[ch] = 0U;   /* off->on edge: no
                                                              real DRIVE sample
                                                              captured under
                                                              THIS gating
                                                              window yet -- see
                                                              the 2026-09-21
                                                              fix note above */
                }
            }
        }

        /* DRIVE capture -- 2026-09-21 CORRECTED to run every tick,
           UNCONDITIONALLY, regardless of `gated`. Previously this whole
           block (and s_lastMeasuredHz[ch]'s only real update site) was
           inside `if (gated != 0U)`, with the ungated branch below
           forcibly zeroing s_lastMeasuredHz[ch] every tick -- a REAL
           BUG, found via a direct self-loopback diagnostic: PFMIN:
           DEBUG:RAW?/REG? (raw, unconditional hardware registers)
           showed genuine, healthy, continuously-updating captures the
           whole time (firstRise=1, avgCount climbing, real toggling
           IDR), while SIM:CHANnel:STATus?'s DRIVE_HZ field sat frozen
           at 0 throughout -- because nothing was ever gated, so this
           block never ran and never even looked at the real data
           sitting right there in pfm_input.c. DRIVE_HZ is now a true,
           always-live measurement of whatever is actually arriving on
           this channel's receive pin, completely decoupled from
           gating -- gating still governs ONLY the simulated response
           (targetHz below), matching "a real Transrex doesn't respond
           when disconnected, but its receiver still sees whatever
           light actually arrives" -- these are two genuinely different
           concepts that were incorrectly conflated into one gated
           block before. */
        {
            uint32_t avgPeriodTicks = 0U;
            uint16_t sampleCount    = 0U;
            uint8_t  haveNewSample  = PfmInput_ConsumeAveragePeriod(ch, &avgPeriodTicks, &sampleCount);

            if ((haveNewSample != 0U) && (avgPeriodTicks != 0U))
            {
                s_lastMeasuredHz[ch] = HRTIM_TIMER_CLK_HZ / avgPeriodTicks;   /* same
                                                                                  formula
                                                                                  pid.c
                                                                                  uses for
                                                                                  measuredHz */
                if (gated != 0U)
                {
                    s_haveMeasuredSinceGate[ch] = 1U;
                }
            }
            /* else: hold s_lastMeasuredHz[ch] at its previous value --
               a main-loop poll can easily land between two DRIVE edges
               near the low end of the frequency range, and momentarily
               flooring to 0 there would fight the filter for no real
               reason; genuinely stays 0 only if nothing has EVER been
               received (Init()'s own default). */
        }

        if (gated != 0U)
        {
            targetHz = (s_haveMeasuredSinceGate[ch] != 0U) ?
                       (float)s_lastMeasuredHz[ch] : (float)PFM_TURNON_FREQ_HZ;   /* 2026-09-21
                                                                                      fix: before
                                                                                      the first
                                                                                      real sample
                                                                                      under this
                                                                                      gating
                                                                                      window,
                                                                                      target the
                                                                                      healthy 0A
                                                                                      floor, not
                                                                                      a stale value
                                                                                      held over from
                                                                                      being ungated */
        }
        else
        {
            targetHz = (s_idleToneEnabled != 0U) ? (float)SIM_TRANSREX_IDLE_TONE_HZ
                                                  : (float)PFM_TURNON_FREQ_HZ;   /* floor
                                                          (or, in idle-tone
                                                          diagnostic mode, the
                                                          3kHz visual-test
                                                          tone) -- not
                                                          hold-last: a real
                                                          Transrex with
                                                          ENA_OUT/CONTACT_OUT
                                                          dropped stops
                                                          RESPONDING, it
                                                          doesn't keep echoing
                                                          its last output --
                                                          but note this is
                                                          now independent of
                                                          DRIVE_HZ reporting,
                                                          which stays live
                                                          above regardless */
        }

        s_filteredHz[ch] += alpha * (targetHz - s_filteredHz[ch]);   /* clean,
                                                                          ripple-free
                                                                          -- see this
                                                                          variable's
                                                                          own comment
                                                                          above */

        {
            /* Real Transrex current-output ripple (720Hz fundamental +
               1440Hz 2nd harmonic), added 2026-09-21 -- see sim_transrex.h's
               own SIM_TRANSREX_RIPPLE_FUNDAMENTAL_HZ comment for the full
               reasoning/assumptions. Only added while genuinely gated (real
               simulated current flowing) -- an ungated channel's floor (or
               idle-tone diagnostic frequency) represents no real current at
               all, so it stays perfectly clean, matching this module's own
               existing "no current, no ripple" physical intent. Computed
               directly in Hz (not Amps-then-reconverted) via this channel's
               own full-scale-derived amplitude -- same linearity reasoning
               this file's own top comment already applies to the base
               filter. */
            float rippleHz = 0.0f;
            if (gated != 0U)
            {
                /* "X% of this channel's own full-scale current" always
                   equals "X% of the full [PFM_TURNON_FREQ_HZ,
                   PFM_MAX_FREQ_HZ] span" here, for ANY channel, REGARDLESS
                   of that channel's own PFM_MAX_CURRENT_A_PER_CHANNEL[ch]
                   entry -- the Amps->Hz mapping is linear, and its two Hz
                   endpoints are shared/global (not per-channel), so a
                   channel's own Amps calibration only changes the SLOPE,
                   never the span itself. No per-channel lookup needed as a
                   result -- not an oversight, this cancels out exactly. */
                float freqSpanHz = (float)PFM_MAX_FREQ_HZ - (float)PFM_TURNON_FREQ_HZ;
                float fundamentalAmpHz = freqSpanHz *
                                          (SIM_TRANSREX_RIPPLE_FUNDAMENTAL_PERCENT / 100.0f);
                float secondHarmonicAmpHz = freqSpanHz *
                                             (SIM_TRANSREX_RIPPLE_SECOND_HARMONIC_PERCENT / 100.0f);
                rippleHz = fundamentalAmpHz *
                               sinf(2.0f * SIM_TRANSREX_PI * SIM_TRANSREX_RIPPLE_FUNDAMENTAL_HZ * nowSec) +
                           secondHarmonicAmpHz *
                               sinf(2.0f * SIM_TRANSREX_PI * SIM_TRANSREX_RIPPLE_SECOND_HARMONIC_HZ * nowSec);
            }

            float clampedHz = s_filteredHz[ch] + rippleHz;

            if (clampedHz < (float)PID_OUTPUT_MIN_HZ)
            {
                clampedHz = (float)PID_OUTPUT_MIN_HZ;
            }
            else if (clampedHz > (float)PID_OUTPUT_MAX_HZ)
            {
                clampedHz = (float)PID_OUTPUT_MAX_HZ;
            }

            {
                uint32_t outputHz = (uint32_t)clampedHz;
                uint16_t per      = (uint16_t)((HRTIM_TIMER_CLK_HZ / outputHz) - 1U);   /* same
                                                                                            formula
                                                                                            pid.c
                                                                                            uses */
                HRTIM1_SetChannelPeriod(ch, per);
                s_lastOutputHz[ch] = outputHz;   /* the real, ripple-inclusive
                                                      value -- see this array's
                                                      own comment above */
            }
        }

        /* Fault-injection pins are level-driven directly by
           SimTransrex_SetFault*() below, not touched here every tick --
           nothing about them needs a per-update refresh. */

        /* Waveform log -- see sim_transrex.h's own extensive comment.
           Only the currently-armed channel (if any) is ever appended
           to; the min-interval throttle and maxSamples cap are both
           checked here, every real Update() call, not host-polled. */
        if ((s_logChannel == ch) && (s_logCount < s_logMaxSamples))
        {
            uint32_t sinceLastSample = now - s_logLastSampleMs;
            if ((s_logCount == 0U) || (sinceLastSample >= s_logMinIntervalMs))
            {
                s_logTimeMs[s_logCount]     = now - s_logArmTimeMs;
                s_logDriveHz[s_logCount]    = s_lastMeasuredHz[ch];
                s_logFeedbackHz[s_logCount] = s_lastOutputHz[ch];   /* the real,
                                                                        ripple-
                                                                        inclusive
                                                                        value --
                                                                        see that
                                                                        array's
                                                                        own comment */
                s_logCount++;
                s_logLastSampleMs = now;
            }
        }
    }
}

void SimTransrex_SetFaultWaterTemp(uint8_t channel, uint8_t faulted)
{
    if (channel >= HRTIM_NUM_CHANNELS)
    {
        return;
    }
    s_faultWaterTemp[channel] = (faulted != 0U) ? 1U : 0U;
    XrexIo_SetEnableOutput(channel, (faulted != 0U) ? 0U : 1U);
}

uint8_t SimTransrex_GetFaultWaterTemp(uint8_t channel)
{
    if (channel >= HRTIM_NUM_CHANNELS)
    {
        return 0U;
    }
    return s_faultWaterTemp[channel];
}

void SimTransrex_SetFaultEnerpro(uint8_t channel, uint8_t faulted)
{
    if (channel >= HRTIM_NUM_CHANNELS)
    {
        return;
    }
    s_faultEnerpro[channel] = (faulted != 0U) ? 1U : 0U;
    XrexIo_SetContactorOutput(channel, (faulted != 0U) ? 0U : 1U);
}

uint8_t SimTransrex_GetFaultEnerpro(uint8_t channel)
{
    if (channel >= HRTIM_NUM_CHANNELS)
    {
        return 0U;
    }
    return s_faultEnerpro[channel];
}

void SimTransrex_SetFaultOcp(uint8_t channel, uint8_t faulted)
{
    if (channel >= HRTIM_NUM_CHANNELS)
    {
        return;
    }
    s_faultOcp[channel] = (faulted != 0U) ? 1U : 0U;
    DriveOcpFaultPin(channel, faulted);
}

uint8_t SimTransrex_GetFaultOcp(uint8_t channel)
{
    if (channel >= HRTIM_NUM_CHANNELS)
    {
        return 0U;
    }
    return s_faultOcp[channel];
}

uint8_t SimTransrex_SetTauMs(uint32_t ms)
{
    if (ms == 0U)
    {
        return 0U;
    }
    s_tauMs = ms;
    return 1U;
}

uint32_t SimTransrex_GetTauMs(void)
{
    return s_tauMs;
}

void SimTransrex_SetIdleToneEnabled(uint8_t enabled)
{
    s_idleToneEnabled = (enabled != 0U) ? 1U : 0U;
}

uint8_t SimTransrex_GetIdleToneEnabled(void)
{
    return s_idleToneEnabled;
}

uint8_t SimTransrex_GetChannelStatus(uint8_t channel,
                                      uint32_t *measuredDriveHz,
                                      uint32_t *filteredFeedbackHz,
                                      uint8_t *enaOutGated,
                                      uint8_t *contactOutGated,
                                      uint8_t *faultWaterTemp,
                                      uint8_t *faultEnerpro,
                                      uint8_t *faultOcp)
{
    uint16_t rawGds;

    if (channel >= HRTIM_NUM_CHANNELS)
    {
        return 0U;
    }

    rawGds = GateDriver_Read();

    if (measuredDriveHz != NULL)
    {
        *measuredDriveHz = s_lastMeasuredHz[channel];
    }
    if (filteredFeedbackHz != NULL)
    {
        *filteredFeedbackHz = s_lastOutputHz[channel];   /* the real, ripple-
                                                              inclusive value --
                                                              see that array's
                                                              own comment */
    }
    if (enaOutGated != NULL)
    {
        *enaOutGated = (uint8_t)((rawGds >> channel) & 0x1U);
    }
    if (contactOutGated != NULL)
    {
        *contactOutGated = (uint8_t)((rawGds >> (4U + channel)) & 0x1U);
    }
    if (faultWaterTemp != NULL)
    {
        *faultWaterTemp = s_faultWaterTemp[channel];
    }
    if (faultEnerpro != NULL)
    {
        *faultEnerpro = s_faultEnerpro[channel];
    }
    if (faultOcp != NULL)
    {
        *faultOcp = s_faultOcp[channel];
    }

    return 1U;
}

uint8_t SimTransrex_ArmLog(uint8_t channel, uint16_t maxSamples, uint16_t minIntervalMs)
{
    if (channel >= HRTIM_NUM_CHANNELS)
    {
        return 0U;
    }

    s_logChannel      = channel;
    s_logCount        = 0U;
    s_logMaxSamples   = (maxSamples > SIM_LOG_MAX_SAMPLES) ? (uint16_t)SIM_LOG_MAX_SAMPLES : maxSamples;
    s_logMinIntervalMs = minIntervalMs;
    s_logArmTimeMs    = HAL_GetTick();
    s_logLastSampleMs = s_logArmTimeMs;

    return 1U;
}

uint16_t SimTransrex_GetLogCount(uint8_t channel)
{
    if ((channel >= HRTIM_NUM_CHANNELS) || (channel != s_logChannel))
    {
        return 0U;
    }
    return s_logCount;
}

uint8_t SimTransrex_GetLogSample(uint8_t channel, uint16_t index,
                                  uint32_t *timeMs, uint32_t *driveHz, uint32_t *feedbackHz)
{
    if ((channel >= HRTIM_NUM_CHANNELS) || (channel != s_logChannel) || (index >= s_logCount))
    {
        return 0U;
    }

    if (timeMs != NULL)     { *timeMs     = s_logTimeMs[index]; }
    if (driveHz != NULL)    { *driveHz    = s_logDriveHz[index]; }
    if (feedbackHz != NULL) { *feedbackHz = s_logFeedbackHz[index]; }

    return 1U;
}
