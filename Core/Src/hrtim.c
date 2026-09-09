/*
 * hrtim.c
 *
 * Ported from the sibling PFM-STM32G474 project's hrtim.c, unchanged in
 * behavior -- see hrtim.h for what was deliberately left out
 * (FEEDBACK-mode anything) and why. Fault handling (below) is NOT a
 * port of that project's software HRTIM1_EmergencyStop() -- see
 * hrtim.h's own header comment for why this is a different, stronger
 * mechanism (a real HRTIM hardware fault input, not a GPIO poll).
 *
 * GENERALIZED (2026-09-08) from a hardcoded 3 channels (A/B/C) to a
 * compile-time HRTIM_NUM_CHANNELS (ctrlr_config.h), 1-5 -- see that
 * file and hrtim.h for the full explanation. Every lookup table below
 * indexed by "channel" uses the same ordering as HRTIM_TIMERINDEX_
 * TIMER_A..F (0..5, confirmed sequential in the HAL headers) and
 * PFM_Step_t's cmp[] array (pfm.h).
 */

#include "hrtim.h"
#include <string.h>

HRTIM_HandleTypeDef hhrtim1;

/* Timer index for channel 0..5 (A..F) -- HRTIM_TIMERINDEX_TIMER_A..F
   are confirmed sequential (0x0..0x5) in the HAL headers, so this
   table is really just documentation of that fact, not load-bearing
   arithmetic -- kept as an explicit array rather than computed so
   every other per-channel table below reads the same way. */
static const uint32_t kTimerIndex[6] = {
    HRTIM_TIMERINDEX_TIMER_A, HRTIM_TIMERINDEX_TIMER_B, HRTIM_TIMERINDEX_TIMER_C,
    HRTIM_TIMERINDEX_TIMER_D, HRTIM_TIMERINDEX_TIMER_E, HRTIM_TIMERINDEX_TIMER_F
};

/* Output 1/2 identifiers (OENR bits) for channel 0..5. */
static const uint32_t kOutput1[6] = {
    HRTIM_OUTPUT_TA1, HRTIM_OUTPUT_TB1, HRTIM_OUTPUT_TC1,
    HRTIM_OUTPUT_TD1, HRTIM_OUTPUT_TE1, HRTIM_OUTPUT_TF1
};
static const uint32_t kOutput2[6] = {
    HRTIM_OUTPUT_TA2, HRTIM_OUTPUT_TB2, HRTIM_OUTPUT_TC2,
    HRTIM_OUTPUT_TD2, HRTIM_OUTPUT_TE2, HRTIM_OUTPUT_TF2
};

/* Counter-enable identifiers (MCR bits, HAL_HRTIM_WaveformCounterStart/
   Stop) for channel 0..5. */
static const uint32_t kTimerId[6] = {
    HRTIM_TIMERID_TIMER_A, HRTIM_TIMERID_TIMER_B, HRTIM_TIMERID_TIMER_C,
    HRTIM_TIMERID_TIMER_D, HRTIM_TIMERID_TIMER_E, HRTIM_TIMERID_TIMER_F
};

/* Software-update (CR2) identifiers for channel 0..5. */
static const uint32_t kTimerUpdate[6] = {
    HRTIM_TIMERUPDATE_A, HRTIM_TIMERUPDATE_B, HRTIM_TIMERUPDATE_C,
    HRTIM_TIMERUPDATE_D, HRTIM_TIMERUPDATE_E, HRTIM_TIMERUPDATE_F
};

/* Master-timer reset-trigger identifiers for channel 1..4 (channel 0
   uses MASTER_PER, handled separately -- it has no compare unit of its
   own). Index [0] is unused padding so kMasterResetTrigger[channel]
   reads naturally for channel=1..4; never indexed at channel=0. */
static const uint32_t kMasterResetTrigger[5] = {
    0U, /* unused */
    HRTIM_TIMRESETTRIGGER_MASTER_CMP1, HRTIM_TIMRESETTRIGGER_MASTER_CMP2,
    HRTIM_TIMRESETTRIGGER_MASTER_CMP3, HRTIM_TIMRESETTRIGGER_MASTER_CMP4
};

/* Master-timer flags (MISR, polled by HRTIM1_WaitForPhaseAndConnect())
   for channel 1..4's reset event, same indexing as above. */
static const uint32_t kMasterFlag[5] = {
    0U, /* unused */
    HRTIM_MASTER_FLAG_MCMP1, HRTIM_MASTER_FLAG_MCMP2,
    HRTIM_MASTER_FLAG_MCMP3, HRTIM_MASTER_FLAG_MCMP4
};

/* Master-timer interrupt-enable identifiers, same indexing, used only
   to mask/clear/re-enable around the settling sequence in
   HRTIM1_PWM_Start() -- HRTIM1_EnableMasterInterrupt() itself only
   ever arms HRTIM_MASTER_IT_MREP long-term (channel 0's event, the one
   PFM_CycleBoundaryHandler() actually runs on); channels 1..N-1's IT
   bits are enabled only transiently, for HRTIM1_WaitForPhaseAndConnect()
   to have something to mask, and disabled again before this function
   returns -- see the mask/restore block in HRTIM1_PWM_Start(). */
static const uint32_t kMasterIT[5] = {
    0U, /* unused */
    HRTIM_MASTER_IT_MCMP1, HRTIM_MASTER_IT_MCMP2,
    HRTIM_MASTER_IT_MCMP3, HRTIM_MASTER_IT_MCMP4
};

/* Master compare-unit identifiers (HRTIM_COMPAREUNIT_1..4, bit flags
   not sequential integers) for channel 1..4, same indexing. */
static const uint32_t kMasterCompareUnit[5] = {
    0U, /* unused */
    HRTIM_COMPAREUNIT_1, HRTIM_COMPAREUNIT_2, HRTIM_COMPAREUNIT_3, HRTIM_COMPAREUNIT_4
};

static uint16_t HRTIM1_ClampCompare(uint16_t cmp, uint16_t per)
{
    uint16_t maxCmp;

    if (per <= 4U)
    {
        return HRTIM_COMPARE_MIN;
    }

    maxCmp = (uint16_t)(per - 2U);

    if (cmp < HRTIM_COMPARE_MIN)
    {
        cmp = HRTIM_COMPARE_MIN;
    }

    if (cmp > maxCmp)
    {
        cmp = maxCmp;
    }

    return cmp;
}

void HRTIM1_FullInit(void)
{
    HRTIM_TimeBaseCfgTypeDef pTimeBaseCfg;
    HRTIM_TimerCfgTypeDef pTimerCfg;
    HRTIM_CompareCfgTypeDef pCompareCfg;
    HRTIM_OutputCfgTypeDef pOutputCfg;
    HRTIM_DeadTimeCfgTypeDef pDeadTimeCfg;

    memset(&pTimeBaseCfg, 0, sizeof(pTimeBaseCfg));
    memset(&pTimerCfg, 0, sizeof(pTimerCfg));
    memset(&pCompareCfg, 0, sizeof(pCompareCfg));
    memset(&pOutputCfg, 0, sizeof(pOutputCfg));
    memset(&pDeadTimeCfg, 0, sizeof(pDeadTimeCfg));

    hhrtim1.Instance = HRTIM1;

    if (HAL_HRTIM_Init(&hhrtim1) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_HRTIM_DLLCalibrationStart(&hhrtim1, HRTIM_CALIBRATIONRATE_3) != HAL_OK)
        Error_Handler();
    if (HAL_HRTIM_PollForDLLCalibration(&hhrtim1, 10) != HAL_OK)
        Error_Handler();

    /* Hardware fault input: PC10 / HRTIM1_FLT6 (docs/pin_mapping_v4.csv,
       HRTIM_FAULT_CHANNEL in hrtim.h), active-low (confirmed against
       the real fault-sensing circuit, 2026-09-08). Two-step HAL
       pattern: HAL_HRTIM_FaultConfig() sets polarity/source/filter for
       this fault CHANNEL, then HAL_HRTIM_FaultModeCtl() actually arms
       it -- neither one, by itself, makes any TIMER react to it; that
       part is pTimerCfg.FaultEnable and pOutputCfg.FaultLevel, set
       further down in the per-channel loops below. Filter is NONE:
       fastest possible response is the whole point of using real HRTIM
       silicon instead of a software-polled GPIO (see hrtim.h) --
       revisit if the real fault-sensing circuit turns out to be noisy
       enough to need debouncing at the cost of that latency. */
    {
        HRTIM_FaultCfgTypeDef faultCfg;
        memset(&faultCfg, 0, sizeof(faultCfg));

        faultCfg.Source = HRTIM_FAULTSOURCE_DIGITALINPUT;
        faultCfg.Polarity = HRTIM_FAULTPOLARITY_LOW;
        faultCfg.Filter = HRTIM_FAULTFILTER_NONE;
        faultCfg.Lock = HRTIM_FAULTLOCK_READWRITE;

        if (HAL_HRTIM_FaultConfig(&hhrtim1, HRTIM_FAULT_CHANNEL, &faultCfg) != HAL_OK)
        {
            Error_Handler();
        }
        HAL_HRTIM_FaultModeCtl(&hhrtim1, HRTIM_FAULT_CHANNEL, HRTIM_FAULTMODECTL_ENABLED);
    }

    /* Default initial period:
       100 kHz => 170000000 / 100000 - 1 = 1699 */
    pTimeBaseCfg.Period = 1699U;
    pTimeBaseCfg.RepetitionCounter = 0U;
    pTimeBaseCfg.PrescalerRatio = HRTIM_PRESCALERRATIO_DIV1;
    pTimeBaseCfg.Mode = HRTIM_MODE_CONTINUOUS;

    if (HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_MASTER, &pTimeBaseCfg) != HAL_OK)
    {
        Error_Handler();
    }
    /* All 6 timers get the same initial period, regardless of
       HRTIM_NUM_CHANNELS -- channels beyond it are still fully
       configured (dead time, complementary outputs, pins reserved),
       just never phase-locked or started. See the WaveformTimerConfig
       loop below for the one place channel count actually matters. */
    for (uint8_t ch = 0U; ch < 6U; ch++)
    {
        if (HAL_HRTIM_TimeBaseConfig(&hhrtim1, kTimerIndex[ch], &pTimeBaseCfg) != HAL_OK)
        {
            Error_Handler();
        }
    }

    /* The Master timer's own MPER/MCMP1R-MCMP4R registers are written
       every cycle by HRTIM1_ApplyPfmStep(), exactly like each slave
       timer's PERxR/CMP1xR. HAL_HRTIM_TimeBaseConfig() (called above)
       only configures counter mode/prescaler/period/repetition-counter
       -- it does NOT configure preload/update-trigger behavior for the
       Master, that is HAL_HRTIM_WaveformTimerConfig()'s job. Without an
       explicit, enabled preload and a defined update event (here: on
       the Master's own repetition event, matching the convention used
       for the slaves), the Master's own PER/CMPx shadow-to-active
       transfer timing is undefined. A minimal HRTIM_TimerCfgTypeDef is
       used here (not the fully-populated pTimerCfg reused below for
       the slaves) since fields like ResetTrigger/DeadTimeInsertion/
       FaultEnable don't apply to the Master timer. */
    {
        HRTIM_TimerCfgTypeDef masterTimerCfg;
        memset(&masterTimerCfg, 0, sizeof(masterTimerCfg));

        masterTimerCfg.InterruptRequests = HRTIM_MASTER_IT_NONE;
        masterTimerCfg.DMARequests = HRTIM_TIM_DMA_NONE;
        masterTimerCfg.HalfModeEnable = DISABLE;
        masterTimerCfg.StartOnSync = DISABLE;
        masterTimerCfg.ResetOnSync = DISABLE;
        masterTimerCfg.DACSynchro = HRTIM_DACSYNC_NONE;
        masterTimerCfg.PreloadEnable = HRTIM_PRELOAD_ENABLED;
        masterTimerCfg.UpdateGating = HRTIM_UPDATEGATING_INDEPENDENT;
        masterTimerCfg.BurstMode = HRTIM_TIMERBURSTMODE_MAINTAINCLOCK;
        masterTimerCfg.RepetitionUpdate = HRTIM_UPDATEONREPETITION_ENABLED;

        if (HAL_HRTIM_WaveformTimerConfig(&hhrtim1, HRTIM_TIMERINDEX_MASTER, &masterTimerCfg) != HAL_OK)
        {
            Error_Handler();
        }
    }

    pTimerCfg.InterruptRequests = HRTIM_MASTER_IT_NONE;
    pTimerCfg.DMARequests = HRTIM_TIM_DMA_NONE;
    pTimerCfg.HalfModeEnable = DISABLE;
    pTimerCfg.StartOnSync = DISABLE;
    pTimerCfg.ResetOnSync = DISABLE;
    pTimerCfg.DACSynchro = HRTIM_DACSYNC_NONE;
    pTimerCfg.PreloadEnable = HRTIM_PRELOAD_ENABLED;
    pTimerCfg.UpdateGating = HRTIM_UPDATEGATING_INDEPENDENT;
    pTimerCfg.BurstMode = HRTIM_TIMERBURSTMODE_MAINTAINCLOCK;
    pTimerCfg.RepetitionUpdate = HRTIM_UPDATEONREPETITION_ENABLED;
    pTimerCfg.PushPull = HRTIM_TIMPUSHPULLMODE_DISABLED;
    /* Subscribes every timer's outputs to the hardware fault channel
       configured above (HRTIM_FAULT_CHANNEL/FLT6) -- applied
       unconditionally to all 6 timers in the loop below, same as every
       other field here, so channels beyond HRTIM_NUM_CHANNELS are
       harmlessly fault-gated too even though they're never started. */
    pTimerCfg.FaultEnable = HRTIM_TIMFAULTENABLE_FAULT6;
    pTimerCfg.DeadTimeInsertion = HRTIM_TIMDEADTIMEINSERTION_ENABLED;
    pTimerCfg.ResetUpdate = HRTIM_TIMUPDATEONRESET_DISABLED;

    /* Channels 0..HRTIM_NUM_CHANNELS-1: phase-locked to the Master --
       UpdateTrigger = MASTER ties each one's shadow->active transfer to
       the Master timer's own update event, so a channel's new PER/CMP
       values become active at the same well-defined, coherent boundary
       as its reset. This matches ST's own official multiphase
       reference example (STM32CubeF3 HRTIM_Multiphase), which uses
       this exact reset-from-master architecture. Channel 0's
       ResetTrigger is MASTER_PER (0 deg, the reference); channels
       1..N-1 use MASTER_CMP1..MASTER_CMP(N-1) respectively (kMasterResetTrigger).

       Channels HRTIM_NUM_CHANNELS..5: initialized identically
       otherwise (dead time, complementary outputs, pins reserved --
       see the DeadTimeConfig/WaveformOutputConfig loops below, both
       unconditional over all 6 channels) but deliberately left with
       ResetTrigger = NONE (free-running from their own period, no
       defined phase relationship to anything else) and
       UpdateTrigger = NONE (self-updating at their own repetition
       event via RepetitionUpdate, set above, rather than gated on the
       Master's) -- exactly how Timers D/E/F behaved before this
       generalization, for the N=3 default. See ctrlr_config.h's
       HRTIM_NUM_CHANNELS comment for why a 6th phase-locked channel
       isn't a simple extension of this loop. Also per project decision
       (2026-08-31, still true): HRTIM1_PWM_Start() never starts these
       -- they are configured, reserved, and pin-muxed, but nothing
       starts their counters. */
    for (uint8_t ch = 0U; ch < 6U; ch++)
    {
        if (ch < HRTIM_NUM_CHANNELS)
        {
            pTimerCfg.UpdateTrigger = HRTIM_TIMUPDATETRIGGER_MASTER;
            pTimerCfg.ResetTrigger = (ch == 0U) ? HRTIM_TIMRESETTRIGGER_MASTER_PER
                                                 : kMasterResetTrigger[ch];
        }
        else
        {
            pTimerCfg.UpdateTrigger = HRTIM_TIMUPDATETRIGGER_NONE;
            pTimerCfg.ResetTrigger = HRTIM_TIMRESETTRIGGER_NONE;
        }

        if (HAL_HRTIM_WaveformTimerConfig(&hhrtim1, kTimerIndex[ch], &pTimerCfg) != HAL_OK)
        {
            Error_Handler();
        }
    }

    /* Initial compare ~50%, all 6 channels unconditionally (matches
       the TimeBaseConfig loop above -- channel count only affects
       phase-locking, not which timers get configured at all). */
    pCompareCfg.CompareValue = 850U;
    pCompareCfg.AutoDelayedMode = HRTIM_AUTODELAYEDMODE_REGULAR;
    pCompareCfg.AutoDelayedTimeout = 0U;

    for (uint8_t ch = 0U; ch < 6U; ch++)
    {
        if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, kTimerIndex[ch], HRTIM_COMPAREUNIT_1, &pCompareCfg) != HAL_OK)
        {
            Error_Handler();
        }
    }

    /* Master compare values for each active channel's initial phase
       offset, evenly spaced across HRTIM_NUM_CHANNELS at the
       init-default 100 kHz period (1699, set above) -- channel k's
       offset is k * 1700 / N, matching PFM_PhaseForChannel()'s runtime
       formula exactly (per+1 = 1700). Channel 0 has no phase register
       (implicit 0 deg via MASTER_PER, not a Master CMP unit). Channels
       HRTIM_NUM_CHANNELS..4 (if N < 5) simply never get a compare
       value written here -- their MCMPxR registers stay at reset
       default, harmless since nothing's ResetTrigger references them
       when N doesn't reach that far. */
    for (uint8_t ch = 1U; ch < HRTIM_NUM_CHANNELS; ch++)
    {
        pCompareCfg.CompareValue = ((uint32_t)ch * 1700U) / HRTIM_NUM_CHANNELS;
        if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_MASTER,
                                            kMasterCompareUnit[ch], &pCompareCfg) != HAL_OK)
        {
            Error_Handler();
        }
    }

    /* BUGFIX, real hardware, 2026-09-09 -- see docs/changelog.txt for
       the full diagnostic writeup. First finding: V/W's output SET
       (tied to TIMPER, which for a channel with ResetTrigger =
       MASTER_CMPk means "this channel's own reset-trigger event", a
       point that itself shifts every table entry -- phase =
       k*(per+1)/N) could land within a few HRTIM clocks of the
       PREVIOUS cycle's own duty compare (CMP1, this timer's RESET
       source) during a fast frequency change -- confirmed via ST's
       own community engineering thread (search "HRTIM PWM transients
       greater than a period"): when a timer's SET and RESET events
       fall within ~3 HRTIM clocks of each other, RESET always wins
       and SET is silently dropped, leaving the output low for one
       extra full cycle -- exactly the "measured period = old + new"
       signature found on real hardware.

       First fix attempt moved ONLY channels 1..N-1 (V/W) off TIMPER,
       reasoning channel 0 (U)'s SET (MASTER_PER, always far from its
       own mid-cycle CMP1 under normal duty) couldn't hit the same
       collision. Real-hardware re-test confirmed V/W fully fixed
       (zero anomalies) -- but U started glitching, with the IDENTICAL
       old+new signature, at essentially the same per-value transitions
       V/W used to fail at (~1993 ticks and below, matching table steps
       12-16 of the same ramp, plus the final 100 kHz step). This
       means U was never actually immune -- its own MASTER_PER-tied SET
       has SOME collision-prone pairing too (not yet fully diagrammed),
       just rarer/narrower than V/W's, and was probably masked by V/W's
       much-more-frequent failures in earlier testing rather than
       genuinely unaffected.

       FIX, extended: apply the SAME CMP3-based SET (see the ST thread
       above) to ALL active channels, including channel 0 -- not just
       1..N-1. Channel 0's ResetTrigger stays MASTER_PER (its COUNTER
       still resets exactly on the Master boundary, unchanged); only
       its OUTPUT's SET SOURCE moves off the un-prioritized TIMPER
       event onto CMP3, same as V/W, so ANY collision against its own
       CMP1 resolves in SET's favor instead of losing. CMP3xR is set
       once, here, to a small fixed tick count (not duty-dependent, no
       per-cycle updates needed) -- just needs to fire shortly after
       this channel's own counter resets, well clear of any
       single-digit-count edge case, and utterly negligible against
       real period lengths (1200+ ticks). Only the unused channels
       beyond HRTIM_NUM_CHANNELS (free-running, ResetTrigger = NONE, no
       Master-synced reset at all) still keep SetSource = TIMPER, since
       "own period" for them really is just their own natural overflow,
       nothing to collide with. */
    #define HRTIM_SET_CMP3_TICKS  (16U)
    {
        HRTIM_CompareCfgTypeDef setFixCmp3Cfg;
        memset(&setFixCmp3Cfg, 0, sizeof(setFixCmp3Cfg));
        setFixCmp3Cfg.CompareValue = HRTIM_SET_CMP3_TICKS;
        setFixCmp3Cfg.AutoDelayedMode = HRTIM_AUTODELAYEDMODE_REGULAR;
        setFixCmp3Cfg.AutoDelayedTimeout = 0U;

        for (uint8_t ch = 0U; ch < HRTIM_NUM_CHANNELS; ch++)
        {
            if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, kTimerIndex[ch], HRTIM_COMPAREUNIT_3,
                                                &setFixCmp3Cfg) != HAL_OK)
            {
                Error_Handler();
            }
        }
    }

    pDeadTimeCfg.Prescaler = HRTIM_TIMDEADTIME_PRESCALERRATIO_DIV1;
    pDeadTimeCfg.RisingValue = HRTIM_DEADTIME_COUNTS;
    pDeadTimeCfg.RisingSign = HRTIM_TIMDEADTIME_RISINGSIGN_POSITIVE;
    pDeadTimeCfg.RisingLock = HRTIM_TIMDEADTIME_RISINGLOCK_WRITE;
    pDeadTimeCfg.RisingSignLock = HRTIM_TIMDEADTIME_RISINGSIGNLOCK_WRITE;
    pDeadTimeCfg.FallingValue = HRTIM_DEADTIME_COUNTS;
    pDeadTimeCfg.FallingSign = HRTIM_TIMDEADTIME_FALLINGSIGN_POSITIVE;
    pDeadTimeCfg.FallingLock = HRTIM_TIMDEADTIME_FALLINGLOCK_WRITE;
    pDeadTimeCfg.FallingSignLock = HRTIM_TIMDEADTIME_FALLINGSIGNLOCK_WRITE;

    for (uint8_t ch = 0U; ch < 6U; ch++)
    {
        if (HAL_HRTIM_DeadTimeConfig(&hhrtim1, kTimerIndex[ch], &pDeadTimeCfg) != HAL_OK)
        {
            Error_Handler();
        }
    }

    /* Output Configuration:
       Output 1: set at CMP3 for every active channel (0..HRTIM_NUM_
       CHANNELS-1, see the 2026-09-09 bugfix comment above) or at timer
       period for the unused channels beyond it; reset at timer CMP1
       either way.

       Output 2 (the complement) uses the SAME pOutputCfg as output 1 --
       same Polarity, same SetSource, same ResetSource. This matches
       ST's own reference for DeadTimeInsertion=ENABLED timers
       (Examples/HRTIM/HRTIM_BuckBoost): when dead-time insertion is
       enabled on the timer (set above in pTimerCfg), the dead-time
       hardware unit automatically drives output 2 as the inverse of
       output 1 with the configured dead-time gap inserted between
       edges. You do NOT give output 2 a different SetSource/ResetSource
       to make it "complementary" -- the silicon does that inversion
       itself once DeadTimeInsertion is enabled; SetSource/ResetSource
       on output 2 in that mode is effectively ignored / re-derived from
       output 1's crossbar.
    */
    pOutputCfg.Polarity = HRTIM_OUTPUTPOLARITY_HIGH;
    pOutputCfg.ResetSource = HRTIM_OUTPUTRESET_TIMCMP1;
    pOutputCfg.IdleMode = HRTIM_OUTPUTIDLEMODE_NONE;
    pOutputCfg.IdleLevel = HRTIM_OUTPUTIDLELEVEL_INACTIVE;
    /* THE actual output-safing mechanism: forces this output to its
       INACTIVE level (driven low, given HRTIM_OUTPUTPOLARITY_HIGH
       above) the instant the fault channel this timer is subscribed
       to (pTimerCfg.FaultEnable above) trips -- in hardware, no CPU
       involvement. Was HRTIM_OUTPUTFAULTLEVEL_NONE (fault had zero
       effect on the output) before the fault input existed. */
    pOutputCfg.FaultLevel = HRTIM_OUTPUTFAULTLEVEL_INACTIVE;
    pOutputCfg.ChopperModeEnable = DISABLE;
    pOutputCfg.BurstModeEntryDelayed = DISABLE;

    for (uint8_t ch = 0U; ch < 6U; ch++)
    {
        pOutputCfg.SetSource = (ch < HRTIM_NUM_CHANNELS)
                                    ? HRTIM_OUTPUTSET_TIMCMP3
                                    : HRTIM_OUTPUTSET_TIMPER;

        if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, kTimerIndex[ch], kOutput1[ch], &pOutputCfg) != HAL_OK)
        {
            Error_Handler();
        }
    }

    /* Output 2: same pOutputCfg as output 1, per the ST reference cited
       above. Do NOT change Polarity, SetSource, or ResetSource here --
       see the comment block above this section for why. */
    for (uint8_t ch = 0U; ch < 6U; ch++)
    {
        if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, kTimerIndex[ch], kOutput2[ch], &pOutputCfg) != HAL_OK)
        {
            Error_Handler();
        }
    }
}

void HAL_HRTIM_MspInit(HRTIM_HandleTypeDef *hhrtim)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    if (hhrtim->Instance == HRTIM1)
    {
        memset(&GPIO_InitStruct, 0, sizeof(GPIO_InitStruct));

        __HAL_RCC_HRTIM1_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        __HAL_RCC_GPIOC_CLK_ENABLE();

        /* All 6 channel pairs' pins are configured unconditionally,
           regardless of HRTIM_NUM_CHANNELS -- pin/electrical
           reservation is independent of how many channels are actually
           phase-locked and started (see HRTIM1_FullInit()'s own
           comment on this same point). */

        /* PA8/9 = HRTIM1_CHA1/2 (PHASE_U/UN), PA10/11 = HRTIM1_CHB1/2
           (PHASE_V/VN) -- per docs/pin_mapping_v4.csv, unchanged from
           the sibling project's pinout for these 4 pins. */
        GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF13_HRTIM1;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /* PB12/13 = HRTIM1_CHC1/2 (PHASE_W/WN), PB14/15 = HRTIM1_CHD1/2
           (PHASE_X/XN, new on this board) -- both pairs use AF13, same
           as A/B/C. AF number confirmed against the STM32G474
           datasheet's Table 13 (Alternate function), column-position
           verified against PB12's own known-good AF13 entry as a
           calibration point -- see docs/changelog.txt. */
        GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF13_HRTIM1;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        /* PC6/7 = HRTIM1_CHF1/2 (PHASE_Z/ZN) -- AF13, same as the rest. */
        GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF13_HRTIM1;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

        /* PC8/9 = HRTIM1_CHE1/2 (PHASE_Y/YN) -- the ONE exception: this
           pair maps to AF3, not AF13, on this package. Confirmed via
           the same datasheet column-position method (verified twice,
           including cross-checking neighboring I2C3_SCL/SDA and
           TIM3_CH3/CH4 land on their own textbook-correct AF columns)
           -- do not "fix" this to match the other five pairs, it is
           deliberately different and a wrong AF here means the pin
           silently never connects to HRTIM1 at all. */
        GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF3_HRTIM1;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

        /* PC10 = HRTIM1_FLT6 (HR_FAULT_INPUT_PIN, docs/pin_mapping_v4.csv)
           -- AF13, confirmed via the same datasheet column-position
           method as the channel pins above (cross-checked twice within
           the same wrapped table cell, reproducing the known-good PC8/
           AF3 calibration point exactly before trusting this result --
           see docs/changelog.txt, 2026-09-08).

           An input, not an output like the channel pins -- GPIO_PULLUP,
           not NOPULL: no external pull-up on this net was confirmed, so
           this is a defensive default matching the active-low
           convention (idle/healthy = high; an unconnected or
           not-yet-populated fault source then reads as healthy rather
           than an unpredictable floating level). Redundant and harmless
           if the board already has its own external pull-up; revisit
           if there's a reason to prefer NOPULL instead. */
        GPIO_InitStruct.Pin = GPIO_PIN_10;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF13_HRTIM1;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    }
}

void HAL_HRTIM_MspDeInit(HRTIM_HandleTypeDef *hhrtim)
{
    if (hhrtim->Instance == HRTIM1)
    {
        __HAL_RCC_HRTIM1_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11);
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15);
        HAL_GPIO_DeInit(GPIOC, GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10);
    }
}

/* Waits for one specific Master-timer flag (channel 0's MREP, or
 * channel k's MCMPk, k=1..4) to go true in hardware, then connects
 * exactly one channel's outputs to the pins -- see HRTIM1_PWM_Start()
 * below for why this exists and why it must be called with global
 * interrupts already masked. `reforceActive` re-asserts output1
 * ACTIVE/output2 INACTIVE (the same SETx1R/RSTx2R "software trigger"
 * bits HAL_HRTIM_WaveformSetOutputLevel() uses, written directly for
 * the same reason OENR is below: this call site owns hhrtim1
 * exclusively while it runs, so the HAL's lock/state-machine overhead
 * is pure avoidable latency here) immediately before unmasking --
 * needed for every active channel (0..N-1, as of the 2026-09-09
 * SET/RESET-collision fix -- see HRTIM1_FullInit()'s own comment):
 * each one's output SET source is CMP3, a genuinely separate event
 * from whatever resets its counter (MASTER_PER for channel 0,
 * MASTER_CMP1..4 for channels 1..N-1), so the reset alone does NOT
 * itself generate a fresh SET event for any of them (confirmed
 * against real hardware data, see the call sites' comments) and the
 * output would otherwise still reflect whatever state the very first
 * force-ACTIVE call (before ANY counter started) left it in. Before
 * that fix, channel 0's SET source WAS TIMPER (coinciding exactly
 * with its own MASTER_PER reset), so it alone didn't need this --
 * that is no longer true, and this function's only channel-0-specific
 * call site (HRTIM1_PWM_Start(), below) now passes reforceActive=1
 * for it too.
 *
 * Returns 0 on success, 1 if the spin-count ceiling was hit (a real
 * fault -- the counters aren't actually running -- not a timing corner
 * case; see HRTIM1_PWM_Start()'s comment on why this can't be a
 * wall-clock timeout here). */
static uint8_t HRTIM1_WaitForPhaseAndConnect(uint32_t masterFlag,
                                             uint32_t timerIdx,
                                             uint32_t outputMask,
                                             uint8_t reforceActive)
{
    uint32_t spins = 0U;

    while (__HAL_HRTIM_MASTER_GET_FLAG(&hhrtim1, masterFlag) == RESET)
    {
        spins++;
        if (spins >= 1000000U)
        {
            return 1U;
        }
    }

    if (outputMask != 0U)
    {
        if (reforceActive != 0U)
        {
            hhrtim1.Instance->sTimerxRegs[timerIdx].SETx1R |= HRTIM_SET1R_SST;
            hhrtim1.Instance->sTimerxRegs[timerIdx].RSTx2R |= HRTIM_RST2R_SRT;
        }
        hhrtim1.Instance->sCommonRegs.OENR |= outputMask;
    }

    return 0U;
}

void HRTIM1_PWM_Start(const uint8_t *channelEnabled)
{
    uint32_t outputMask[HRTIM_NUM_CHANNELS];
    uint32_t counterStartMask = HRTIM_TIMERID_MASTER;

    /* With DeadTimeInsertion enabled, output 2 of each timer is driven
       as the hardware-inverted complement of output 1 -- but there is
       no natural "first state" for a complementary pair coming out of
       reset. Per ST's HAL documentation: "when dead-time insertion is
       enabled it is necessary to force the output level by software to
       have the outputs in a complementary state as soon as the RUN
       mode is entered." Force output 1 ACTIVE / output 2 INACTIVE on
       each timer before starting the counters, so the pair begins in a
       known, genuinely complementary state rather than whatever level
       the deadtime unit happens to reset into. Only done for channels
       that will actually be enabled below -- harmless either way since
       WaveformSetOutputLevel doesn't itself enable an output, but no
       reason to touch a channel that's staying disabled.

       This is a PREREQUISITE for the counters to start into a defined
       state, not the last word on what the pins eventually show -- see
       HRTIM1_WaitForPhaseAndConnect()'s `reforceActive` for why
       channels 1..N-1 need this repeated later, right as they
       individually connect. */
    for (uint8_t ch = 0U; ch < HRTIM_NUM_CHANNELS; ch++)
    {
        if (channelEnabled[ch] != 0U)
        {
            HAL_HRTIM_WaveformSetOutputLevel(&hhrtim1, kTimerIndex[ch], kOutput1[ch], HRTIM_OUTPUTLEVEL_ACTIVE);
            HAL_HRTIM_WaveformSetOutputLevel(&hhrtim1, kTimerIndex[ch], kOutput2[ch], HRTIM_OUTPUTLEVEL_INACTIVE);
            outputMask[ch] = kOutput1[ch] | kOutput2[ch];
        }
        else
        {
            outputMask[ch] = 0U;
        }
        counterStartMask |= kTimerId[ch];
    }

    /* Cold-start phase-lock settling -- per-channel, individually
       timed. The force-ACTIVE calls above put every enabled channel's
       main output HIGH at the same instant, with no regard for their
       intended even stagger -- necessary (per ST's own guidance,
       cited above) to avoid an undefined complementary-pair state, but
       it means the pins do NOT yet reflect each timer's real phase
       relationship. Channel 0's ResetTrigger=MASTER_PER coincides with
       its own natural rollover; channels 1..N-1 (ResetTrigger=
       MASTER_CMP1..CMP(N-1)) only become correctly phase-locked once
       they've received that first Master-CMPk-triggered reset, at
       k/N of a period.

       Two real-hardware findings (DSLogic captures, 2026-09-04, back
       when this was hardcoded to 3 channels -- see docs/changelog.txt)
       shaped this function:
         1. Without ANY delay, every channel's first-ever output edges
            all land on the exact same sample -- a garbled ~1-2 cycles
            before self-correcting. Fixed by not connecting outputs
            until phase-lock had settled.
         2. Waiting for a single event (one full Master period) and
            connecting all channels together stopped the catastrophic
            collision, but exposed channels 1..N-1 mid-way through
            whatever their comparators had been doing since their OWN
            (earlier, still-hidden) MASTER_CMPk reset -- a real duty
            cycle, just not starting from a clean edge, and not
            reliably positioned at the intended phase point either.

       Fix (unchanged by this generalization, just looped instead of
       named 3 times): wait for and connect each channel SEPARATELY, at
       ITS OWN reset-trigger event -- channel k (1..N-1) at
       MASTER_CMPk (k/N of a period), channel 0 LAST, at MASTER_PER/
       MREP (the full period) -- with channels 1..N-1's output
       re-forced ACTIVE at that exact moment (see
       HRTIM1_WaitForPhaseAndConnect()'s `reforceActive`). Each
       channel's first VISIBLE pulse is therefore a genuine fresh
       start referenced from ITS OWN phase point.

       All channels' Master flags/interrupts used below are masked and
       cleared of stale state before the counters start: HRTIM1_Master_
       IRQn is armed from boot (or a previous shot) for MREP specifically
       (see HRTIM1_EnableMasterInterrupt()) and must not react until
       this function is done setting up the current shot --
       PFM_CycleBoundaryHandler() firing early would silently advance
       the table before step 0 was ever visible. Channels 1..N-1's
       MCMPk interrupts are never armed long-term (only MREP is, by
       HRTIM1_EnableMasterInterrupt()) -- but mask/clear them here
       anyway, defensively, in case that ever changes. */
    __HAL_HRTIM_MASTER_DISABLE_IT(&hhrtim1, HRTIM_MASTER_IT_MREP);
    for (uint8_t ch = 1U; ch < HRTIM_NUM_CHANNELS; ch++)
    {
        __HAL_HRTIM_MASTER_DISABLE_IT(&hhrtim1, kMasterIT[ch]);
        __HAL_HRTIM_MASTER_CLEAR_IT(&hhrtim1, kMasterIT[ch]);
    }
    __HAL_HRTIM_MASTER_CLEAR_IT(&hhrtim1, HRTIM_MASTER_IT_MREP);

    /* The Master counter plus every channel 0..N-1's counter always
       start, regardless of which channels are enabled -- a disabled
       channel's timer must stay running and synchronized with the
       Master via its ResetTrigger, or re-enabling it later would not
       be coherent with the other channels. Only the output pins
       themselves are gated per-channel below. */
    if (HAL_HRTIM_WaveformCounterStart(&hhrtim1, counterStartMask) != HAL_OK)
    {
        Error_Handler();
    }

    /* Global interrupts masked for the entire settling sequence below
       (not just one flag/write): SysTick (priority 0, the HIGHEST in
       this firmware) or USART2 (priority 2) preempting any one of
       these latency-sensitive windows would reintroduce exactly the
       jitter this mechanism exists to remove. Bounded by each call's
       own 1,000,000-spin ceiling (HRTIM1_WaitForPhaseAndConnect()) --
       not a wall-clock timeout, since HAL_GetTick() cannot advance
       while global interrupts are masked (it's SysTick-driven, and
       SysTick is masked too). Total worst-case masked duration is
       still bounded by one Master period (~385 us worst case for a
       uint16_t `per`), same as before -- N sequential sub-waits within
       that same one-period budget, not N separate one-period waits. */
    __disable_irq();
    {
        uint8_t timedOut = 0U;

        /* Channels 1..N-1 in ascending order -- MASTER_CMP1 fires
           before MASTER_CMP2, before MASTER_CMP3, before MASTER_CMP4,
           each strictly before the full-period MREP channel 0 waits
           for last, below. */
        for (uint8_t ch = 1U; ch < HRTIM_NUM_CHANNELS; ch++)
        {
            if (HRTIM1_WaitForPhaseAndConnect(kMasterFlag[ch], kTimerIndex[ch],
                                              outputMask[ch], 1U) != 0U)
            {
                timedOut = 1U;
            }
        }

        /* Channel 0 at the full period (MASTER_PER/MREP) -- WITH
           reforceActive, same as channels 1..N-1, since the 2026-09-09
           SET/RESET-collision fix (see HRTIM1_FullInit()'s own comment)
           moved channel 0's output SET off TIMPER onto CMP3 too. Its
           own reset (MASTER_PER) is no longer the same event as its
           SET source, so it needs the same explicit re-force this
           function already does for channels 1..N-1 -- without this,
           the first pulse would not reliably reflect the correct
           phase, reintroducing the exact garbled-cold-start failure
           mode HRTIM1_WaitForPhaseAndConnect() was built to prevent
           (2026-09-04, see docs/changelog.txt). */
        if (HRTIM1_WaitForPhaseAndConnect(HRTIM_MASTER_FLAG_MREP, kTimerIndex[0],
                                          outputMask[0], 1U) != 0U)
        {
            timedOut = 1U;
        }

        /* Not time-critical -- the interrupts just need to be back in
           their normal armed state before this function returns, not
           before any of the connects above. Restores the always-armed
           MREP state HRTIM1_EnableMasterInterrupt() established at
           boot; channels 1..N-1's MCMPk interrupts stay disabled
           (matching their normal not-armed-long-term state, see the
           comment above the mask/clear block before the counter
           start). Also clears whatever flags the waits above just
           consumed. */
        __HAL_HRTIM_MASTER_CLEAR_IT(&hhrtim1, HRTIM_MASTER_IT_MREP);
        __HAL_HRTIM_MASTER_ENABLE_IT(&hhrtim1, HRTIM_MASTER_IT_MREP);
        for (uint8_t ch = 1U; ch < HRTIM_NUM_CHANNELS; ch++)
        {
            __HAL_HRTIM_MASTER_CLEAR_IT(&hhrtim1, kMasterIT[ch]);
        }

        if (timedOut != 0U)
        {
            __enable_irq();
            Error_Handler();
        }
    }
    __enable_irq();
    /* If every channel is disabled, no outputs are connected at all --
       counters run but nothing is ever driven. This is a legal, if
       unusual, state; the operator explicitly disabled every channel,
       so no output is exactly correct here. */
}

void HRTIM1_EnableMasterInterrupt(void)
{
    /* Priority scheme, originally matching the sibling PFM-STM32G474
       project exactly (see FixSysTickPriority()'s doc comment in
       main.c): SysTick=0 (highest), HRTIM1_Master=1, USART2=2. Lower
       number = higher priority on this Cortex-M4's NVIC.

       BUGFIX, confirmed on real hardware, 2026-09-08: HRTIM1_Master
       moved from 1 to 2 (USART2 correspondingly moved from 2 to 3,
       MX_USART2_UART_Init() in main.c) -- HRTIM1_Master_IRQn was tied
       at the same priority (1) as PFM_Input's TIM2/TIM3/TIM4/TIM5
       capture interrupts (pfm_input.c) and the GateDriverStatus EXTI
       interrupts (main.c). Since HRTIM1_Master fires at the exact same
       ~10 us cadence as the very signal PFM_Input measures (both
       driven by the same underlying PWM period), tied priority meant
       no preemption between them -- when both became pending close
       together, whichever ran first could delay the other long enough
       for a SECOND real capture edge to arrive before the first was
       serviced, silently overwriting it in hardware (a genuine
       overcapture, CCxOF) -- confirmed on real hardware capturing
       phase U's own known-clean 100 kHz output: ~36% of real edges
       were being dropped this way. Demoting HRTIM1_Master to priority
       2 lets PFM_Input's (and GateDriverStatus's) interrupts always
       preempt it -- see docs/changelog.txt for the full diagnostic
       writeup (DSLogic ground truth, a raw unfiltered capture log,
       live ISR register snapshots) that led here.

       REVERTED, confirmed on real hardware, 2026-09-09: back to
       priority 1 (highest below SysTick) -- PFM_Input's capture
       technique changed again since the note above was written (see
       pfm_input.c's own "Capture technique -- history" comment, step
       3/4): edge capture is now DMA-driven, with the DMA controller
       grabbing each CCRx value directly in hardware, with ZERO CPU/
       IRQ involvement per edge. The per-edge overcapture race that
       justified demoting HRTIM1_Master no longer exists -- a DMA
       transfer happens regardless of what priority any IRQ is
       running at. What DOES still run on the CPU is PFM_Input's
       DMA1_ChannelN half/full-transfer ISR, which batch-processes 16
       captured edges at a time (pfm_input.c) -- and leaving THAT at
       priority 1 (tied with, and therefore able to preempt,
       HRTIM1_Master at 2) let it delay the table-advance ISR by up
       to ~18500 cycles (~109 us) roughly every 8 periods -- the
       exact new bug this session root-caused: a long PFM table hold
       run accumulating many of these small losses until real output
       fell ~77 periods behind the table. Diagnosed with a new raw
       per-call gap log (PFM_GetDiagGapLog(), pfm.c, PFM:GAPLOG?) --
       every spike lined up with a DMA half/full-transfer boundary
       (16 edges / 2 edges-per-period = 8 periods), each spike
       ~15000-18500 cycles, matching PFM_Input's DMA-processing burst
       exactly. Restoring HRTIM1_Master to priority 1 (see
       pfm_input.c's PfmInput_Init() for the corresponding DMA-channel
       demotion to priority 3) fixes this: HRTIM1_Master can no
       longer be starved by PFM_Input's own housekeeping, and
       PFM_Input's actual capture fidelity is unaffected since it no
       longer depends on IRQ latency at all -- only on the DMA
       controller keeping up, which it does autonomously. See
       docs/changelog.txt for the full writeup. */
    HAL_NVIC_SetPriority(HRTIM1_Master_IRQn, 1U, 0U);
    HAL_NVIC_EnableIRQ(HRTIM1_Master_IRQn);

    __HAL_HRTIM_MASTER_ENABLE_IT(&hhrtim1, HRTIM_MASTER_IT_MREP);
}

void HRTIM1_PWM_Stop(void)
{
    uint32_t outputMask = 0U;
    uint32_t counterStopMask = HRTIM_TIMERID_MASTER;

    for (uint8_t ch = 0U; ch < HRTIM_NUM_CHANNELS; ch++)
    {
        outputMask |= (kOutput1[ch] | kOutput2[ch]);
        counterStopMask |= kTimerId[ch];
    }

    HAL_HRTIM_WaveformOutputStop(&hhrtim1, outputMask);

    /* Must also stop the underlying Master/channel counters, not just
       the outputs -- see the sibling project's documented history:
       leaving the counters running after outputs stop means the
       Master repetition ISR keeps firing forever at the full carrier
       rate, which can starve the main loop (a blocking uart_send()
       call, for instance) indefinitely. Stopping the counters here
       (mirroring exactly which timers HRTIM1_PWM_Start() started)
       means the ISR genuinely stops firing once a shot ends. */
    HAL_HRTIM_WaveformCounterStop(&hhrtim1, counterStopMask);
}

uint8_t HRTIM1_FaultIsTripped(void)
{
    return (__HAL_HRTIM_GET_FLAG(&hhrtim1, HRTIM_FAULT_FLAG) != RESET) ? 1U : 0U;
}

void HRTIM1_FaultClear(void)
{
    /* Stop everything cleanly first -- HRTIM1_PWM_Stop() disconnects
       outputs AND stops the counters, exactly the same clean-stopped
       state PFM_CycleBoundaryHandler() already leaves the peripheral
       in at the end of any normal shot (table exhaustion). Then clear
       the latched FLT6 flag.

       Deliberately does NOT try to reconnect/restart outputs itself
       (an earlier version of this plan considered re-issuing
       HAL_HRTIM_WaveformOutputStart() here defensively) -- there is no
       need to duplicate that logic: HRTIM1_PWM_Start() already
       performs a complete from-scratch reconnect sequence (force
       output level, wait for phase-lock, connect via OENR) with no
       assumption about prior connection state, and it is EXACTLY what
       the next FIRE (via PFM_Restart()) will call. Reimplementing a
       second, slightly-different reconnect path here would be a real
       correctness risk (two places to keep in sync) for no benefit --
       "clear the fault, then require an explicit FIRE to actually
       resume output" is also the safer behavior anyway, matching the
       no-silent-resume principle behind latching the fault at all. */
    HRTIM1_PWM_Stop();
    __HAL_HRTIM_CLEAR_FLAG(&hhrtim1, HRTIM_FAULT_FLAG);
}

void HRTIM1_SoftwareUpdate(void)
{
    /* Force shadow→active transfer on Master + every active channel's
       timer in a single CR2 write. CR2 is the ONLY home of the
       software-update bits (MSWU/TxSWU) -- there is no per-timer
       equivalent in MCR/TIMxCR, which only configure update SOURCES.
       Setting MSWU alone would also cascade to the slave timers (they
       are configured with UpdateTrigger = MASTER), but writing every
       bit explicitly avoids any ambiguity about update-propagation
       order. The bits are self-clearing in hardware -- they trigger
       exactly one update transfer and then reset to 0 without software
       intervention. */
    uint32_t updateMask = HRTIM_TIMERUPDATE_MASTER;

    for (uint8_t ch = 0U; ch < HRTIM_NUM_CHANNELS; ch++)
    {
        updateMask |= kTimerUpdate[ch];
    }

    hhrtim1.Instance->sCommonRegs.CR2 |= updateMask;
}

void HRTIM1_ApplyPfmStep(uint16_t per,
                         const uint16_t *cmp,
                         const uint16_t *phase)
{
    /* Master period, shared by every active channel. */
    *HRTIM1_GetMasterPerRegAddress() = per;

    for (uint8_t ch = 0U; ch < HRTIM_NUM_CHANNELS; ch++)
    {
        uint16_t clampedCmp = HRTIM1_ClampCompare(cmp[ch], per);

        /* Each channel's own period register (all share the same
           `per`, matching the shared-carrier design this generalizes
           -- see pfm.h). */
        *HRTIM1_GetTimerPerRegAddress(ch) = per;

        /* Duty for this channel. */
        *HRTIM1_GetTimerCmp1RegAddress(ch) = clampedCmp;

        /* Channel 0 has no phase register -- it's the Master-PER 0 deg
           reference. Channels 1..N-1 get their Master CMPk phase
           offset, clamped the same way HRTIM1_FullInit()'s cold-start
           default logic implicitly relies on `per` being sane -- if
           `phase[ch] >= per`, fall back to this channel's own evenly-
           spaced default position rather than writing a nonsensical
           value past the period (mirrors the original 3-channel
           code's phaseB/phaseC >= per fallback exactly, generalized to
           any channel/N). */
        if (ch >= 1U)
        {
            uint16_t phaseVal = phase[ch];
            if (phaseVal >= per)
            {
                phaseVal = (uint16_t)(((uint32_t)ch * ((uint32_t)per + 1U)) / HRTIM_NUM_CHANNELS);
            }
            *HRTIM1_GetMasterCmpRegAddress(ch) = phaseVal;
        }
    }
}

volatile uint32_t *HRTIM1_GetMasterPerRegAddress(void)
{
    return (volatile uint32_t *)&(HRTIM1->sMasterRegs.MPER);
}

volatile uint32_t *HRTIM1_GetMasterCmpRegAddress(uint8_t masterCompareUnit)
{
    switch (masterCompareUnit)
    {
        case 1U:
            return (volatile uint32_t *)&(HRTIM1->sMasterRegs.MCMP1R);
        case 2U:
            return (volatile uint32_t *)&(HRTIM1->sMasterRegs.MCMP2R);
        case 3U:
            return (volatile uint32_t *)&(HRTIM1->sMasterRegs.MCMP3R);
        default:
            return (volatile uint32_t *)&(HRTIM1->sMasterRegs.MCMP4R);
    }
}

volatile uint32_t *HRTIM1_GetTimerPerRegAddress(uint8_t channel)
{
    return (volatile uint32_t *)&(HRTIM1->sTimerxRegs[channel].PERxR);
}

volatile uint32_t *HRTIM1_GetTimerCmp1RegAddress(uint8_t channel)
{
    return (volatile uint32_t *)&(HRTIM1->sTimerxRegs[channel].CMP1xR);
}
