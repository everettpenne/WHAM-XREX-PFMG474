/*
 * pfm_input.c
 *
 * See pfm_input.h for full scope/design. HAL_TIM_IC_MspInit()/
 * MspDeInit() live here, not in stm32g4xx_hal_msp.c, matching
 * hrtim.c/qspi_test.c's precedent for hand-added (non-CubeMX)
 * peripherals in this project.
 */

#include "pfm_input.h"
#include "main.h"

#if (PFM_INPUT_FEATURE_ENABLED != 0)

/* --------------------------------------------------------------------------
 * Channel descriptor table -- one row per PFM_Input_01..06, from
 * docs/pin_mapping_v4.csv + the datasheet-verified AF numbers (see
 * pfm_input.h's own header comment / docs/changelog.txt for how these
 * were confirmed, not guessed). GPIO_NOPULL throughout -- these are
 * actively-driven signal inputs, matching every other actively-driven
 * bus pin already configured in this project (GateDriverStatus,
 * QUADSPI CLK/IO0-3).
 *
 * PA15 (PFM_Input_01) and PB4 (PFM_Input_05) both default to
 * JTAG-debug-port pins at reset (JTDI and JTRST respectively) --
 * reclaiming them as plain AF-muxed GPIO here is standard when only
 * SWD is used (this project's debug link, confirmed by its existing
 * ST-Link/st-flash tooling), not a real risk, just worth this note.
 *
 * dmaInstance/dmaRequest/dmaIrqn added 2026-09-09 for the DMA-based
 * capture redesign -- see this file's own top-of-DMA-section comment
 * below for why. All 6 assigned to DMA1_Channel1..6 (sequentially,
 * one per PFM_Input channel) -- nothing else in this project uses any
 * DMA channel yet, confirmed by grep before choosing these, so there
 * is no conflict to resolve. Request IDs are fixed, chip-defined
 * DMAMUX values (stm32g4xx_hal_dma.h) for each timer/channel's own
 * capture-compare DMA request -- not chosen, just the correct ID for
 * that specific (timer, channel) pair.
 * -------------------------------------------------------------------------- */
typedef struct
{
    GPIO_TypeDef       *port;
    uint16_t            pin;
    uint8_t             af;
    TIM_TypeDef        *timer;
    uint32_t            timChannel;   /* TIM_CHANNEL_1 or TIM_CHANNEL_2 */
    uint8_t             is16Bit;      /* 1 for TIM3/TIM4 (16-bit counters), 0 for
                                          TIM2/TIM5 (32-bit) -- see ProcessDmaChunk()'s
                                          own comment on why this matters for
                                          correct wraparound handling. */
    DMA_Channel_TypeDef *dmaInstance;
    uint32_t             dmaRequest;
    IRQn_Type            dmaIrqn;
} PfmInputDesc_t;

static const PfmInputDesc_t kDesc[PFM_INPUT_NUM_CHANNELS] =
{
    /* PFM_Input_01 */ { GPIOA, GPIO_PIN_15, GPIO_AF1_TIM2, TIM2, TIM_CHANNEL_1, 0U,
                          DMA1_Channel1, DMA_REQUEST_TIM2_CH1, DMA1_Channel1_IRQn },
    /* PFM_Input_02 */ { GPIOD, GPIO_PIN_4,  GPIO_AF2_TIM2, TIM2, TIM_CHANNEL_2, 0U,
                          DMA1_Channel2, DMA_REQUEST_TIM2_CH2, DMA1_Channel2_IRQn },
    /* PFM_Input_03 */ { GPIOB, GPIO_PIN_2,  GPIO_AF2_TIM5, TIM5, TIM_CHANNEL_1, 0U,
                          DMA1_Channel3, DMA_REQUEST_TIM5_CH1, DMA1_Channel3_IRQn },
    /* PFM_Input_04 */ { GPIOC, GPIO_PIN_12, GPIO_AF1_TIM5, TIM5, TIM_CHANNEL_2, 0U,
                          DMA1_Channel4, DMA_REQUEST_TIM5_CH2, DMA1_Channel4_IRQn },
    /* PFM_Input_05 */ { GPIOB, GPIO_PIN_4,  GPIO_AF2_TIM3, TIM3, TIM_CHANNEL_1, 1U,
                          DMA1_Channel5, DMA_REQUEST_TIM3_CH1, DMA1_Channel5_IRQn },
    /* PFM_Input_06 */ { GPIOD, GPIO_PIN_12, GPIO_AF2_TIM4, TIM4, TIM_CHANNEL_1, 1U,
                          DMA1_Channel6, DMA_REQUEST_TIM4_CH1, DMA1_Channel6_IRQn },
};

/* See pfm_input.h's own comment on PFM_INPUT_ACTIVE_CHANNEL_MASK --
   single point of control for which channels this build actually
   initializes/arms. */
static uint8_t ChannelIsActive(uint8_t channel)
{
    return (uint8_t)((PFM_INPUT_ACTIVE_CHANNEL_MASK >> channel) & 1U);
}

/* True if `instance` hosts at least one active channel -- PfmInput_Init()
   uses this to decide whether to touch that timer's clock/NVIC/config
   AT ALL, not just whether to arm it. */
static uint8_t TimerHasActiveChannel(const TIM_TypeDef *instance)
{
    for (uint8_t i = 0U; i < PFM_INPUT_NUM_CHANNELS; i++)
    {
        if ((kDesc[i].timer == instance) && (ChannelIsActive(i) != 0U))
        {
            return 1U;
        }
    }
    return 0U;
}

/* Non-static, declared extern in pfm_input.h -- stm32g4xx_it.c's
   TIM2/TIM3/TIM4/TIM5_IRQHandler()s need to reach these directly for
   HAL_TIM_IRQHandler(), the same pattern hrtim.h's own `extern
   HRTIM_HandleTypeDef hhrtim1` already uses for HRTIM1_Master_IRQHandler(). */
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

/* Non-static, same reasoning -- stm32g4xx_it.c's 6 new
   DMA1_ChannelN_IRQHandler()s need to reach these directly for
   HAL_DMA_IRQHandler(). One per PFM_Input channel, indexed the same
   way as kDesc[]/g_state[] (0 = PFM_Input_01, ..., 5 = PFM_Input_06). */
DMA_HandleTypeDef pfmInputDma[PFM_INPUT_NUM_CHANNELS];

/* --------------------------------------------------------------------------
 * Capture technique -- history, real hardware, 2026-09-08/09. See
 * docs/changelog.txt for the full diagnostic history (DSLogic ground
 * truth, a raw unfiltered capture log, live ISR register snapshots).
 *
 *   1. TIM2/3/4/5's NVIC priority was tied with HRTIM1_Master_IRQn,
 *      which fires at the exact same ~10 us cadence as the very
 *      signal being measured -- no preemption meant real edges were
 *      occasionally serviced too late, dropped via hardware
 *      overcapture (CCxOF). Fixed by demoting HRTIM1_Master_IRQn to a
 *      lower priority (hrtim.c's HRTIM1_EnableMasterInterrupt()) so
 *      capture interrupts always preempt it. This fix is real and
 *      kept.
 *
 *   2. Even with that fixed and overcapture confirmed at zero, capture
 *      was STILL wrong -- the original BOTHEDGE-plus-GPIO-read design
 *      had a genuine race (HAL clears CCxIF before the callback runs,
 *      so a later edge can silently overwrite CCR without tripping
 *      CCxOF). Replaced with an alternating-single-polarity technique,
 *      then that was dropped too (duty-cycle measurement abandoned
 *      entirely, RISING-only from here on) -- see git history and
 *      docs/changelog.txt for the full, now-closed dead end.
 *
 *   3. Even RISING-only, real-hardware testing with genuinely-wired
 *      real signals (not floating pins) on PFM_Input_01+02 found the
 *      REAL PFM output itself stalling on one table entry for 100+
 *      periods -- root-caused (PFM:DIAG?, pfm.c) to HRTIM1_Master's
 *      own interrupt (intentionally LOW NVIC priority, so it never
 *      steals a real capture edge) being genuinely STARVED by these
 *      per-edge capture-channel interrupts, confirmed with a measured
 *      734 us max inter-call gap (~67x a normal period) with only 2
 *      channels active. This is what DMA-based capture (below) is
 *      for: removing per-edge CPU/interrupt involvement almost
 *      entirely, rather than trying to out-prioritize the problem
 *      again the way step 1 did.
 *
 * DMA REDESIGN, 2026-09-09: each active channel's TIMx now DMAs its
 * captured CCRx value directly into a small RAM ring buffer
 * (g_dmaBuf[]) on every edge, with NO per-edge ISR at all -- the CPU
 * only wakes up on a DMA half-transfer or transfer-complete interrupt
 * (HAL_TIM_IC_CaptureHalfCpltCallback()/HAL_TIM_IC_CaptureCallback(),
 * both below), once per PFM_DMA_BUF_LEN/2 edges instead of once per
 * edge. Everything downstream of "here is a batch of new raw ticks"
 * (period computation, the 16-bit wraparound mask, target-count
 * stopping) is unchanged from the old per-edge design, just moved
 * into ProcessDmaChunk() and called once per batch instead of once
 * per edge. Wire protocol (PFMIN:CAPTURE/STATus?/DATA?) is completely
 * unchanged -- this is purely an internal capture-mechanism swap.
 * -------------------------------------------------------------------------- */
typedef struct
{
    uint8_t  haveFirstRise;   /* 0 until the very first rise seen since arming */
    uint32_t lastRiseTick;
    uint16_t capturedCount;
    uint16_t targetCount;       /* 0 = not armed */
    uint16_t overcaptureCount;  /* diagnostic: CCxOF observed -- see the note above */
    uint32_t period[PFM_INPUT_MAX_PERIODS];

    /* Continuous ("free-running") mode -- added 2026-09-09, see
       pfm_input.h's own comment block on PfmInput_StartContinuous().
       `running` is shared between BOTH capture modes (set by
       PfmInput_OnShotStart() for a bench capture, or by
       PfmInput_StartContinuous() for this one) -- the single source of
       truth for "is something already using this channel's DMA/timer,"
       so the two modes can't collide. `continuous` distinguishes which
       mode `running` refers to, for ProcessDmaChunk()'s branch and
       PfmInput_StopContinuous()'s own "only stop what I started" check.
       `lastPeriod` is written by EITHER mode (see PfmInput_GetLatestPeriod()'s
       own comment) -- always safe to read, meaningless (0) until the
       first period closes. */
    uint8_t  running;
    uint8_t  continuous;
    uint32_t lastPeriod;
} PfmInputState_t;

static PfmInputState_t g_state[PFM_INPUT_NUM_CHANNELS];

/* Temporary debug aid, 2026-09-09, first real-hardware test of the DMA
   redesign: HAL_TIM_IC_Start_DMA()'s return code per channel, so a
   silent start failure is visible instead of just showing up as
   "captured 0, no error" indistinguishable from "armed but nothing
   physically connected." Remove once DMA capture is confirmed
   reliable and this diagnostic is no longer needed. */
static HAL_StatusTypeDef g_dmaStartStatus[PFM_INPUT_NUM_CHANNELS];

/* DMA ring buffer, one per channel -- circular, continuously
   overwritten by hardware. Must be even (split cleanly in half by the
   HT/TC callbacks below). 32 is arbitrary but reasonable: small
   enough that even the fastest signal this project has ever measured
   (~140 kHz, ~1200 ticks/period during the frequency-ramp testing)
   completes a half-buffer (16 edges) in ~130 us, comfortably faster
   than any real shot's duration, and small enough to cost nothing
   (32 x 6 channels x 4 bytes = 768 B). */
#define PFM_DMA_BUF_LEN  (32U)
static uint32_t g_dmaBuf[PFM_INPUT_NUM_CHANNELS][PFM_DMA_BUF_LEN];

/* Tracks, per channel, the buffer offset ProcessDmaChunk() will start
   at NEXT (0 or PFM_DMA_BUF_LEN/2 in normal operation -- updated at
   the end of ProcessDmaChunk() to (startOffset + count) % BUF_LEN).
   BUGFIX, real hardware, 2026-09-09: without this, a shot that ends
   mid-chunk (before the next half/full DMA interrupt) silently loses
   however many edges had already landed in the buffer since the last
   processed chunk -- confirmed on real hardware, a 200-period capture
   on channel 1 reaching only 191 before the shot's own natural end
   stopped it. PfmInput_OnShotEnd() uses this (see its own comment) to
   flush that partial tail before actually stopping DMA. Reset to 0 in
   PfmInput_Arm() for a fresh shot. */
static uint16_t g_dmaNextOffset[PFM_INPUT_NUM_CHANNELS];

/* HAL_TIM_ACTIVE_CHANNEL_1/2 (htim->Channel, set by the DMA capture
 * callbacks below around HAL_TIM_IC_CaptureCallback()/
 * HAL_TIM_IC_CaptureHalfCpltCallback()) and TIM_CHANNEL_1/2 (used by
 * every config/start/read API call, and stored in kDesc[]) are
 * different numeric encodings on this HAL -- this converts the former
 * to the latter. Only CH1/CH2 are ever used by this module (no
 * PFM_Input pin lands on CH3/CH4), so this two-way mapping is
 * complete for our purposes. */
static uint32_t ActiveChannelToTimChannel(HAL_TIM_ActiveChannel active)
{
    return (active == HAL_TIM_ACTIVE_CHANNEL_1) ? TIM_CHANNEL_1 : TIM_CHANNEL_2;
}

static int8_t FindChannelIndex(const TIM_TypeDef *instance, uint32_t timChannel)
{
    for (uint8_t i = 0U; i < PFM_INPUT_NUM_CHANNELS; i++)
    {
        if ((kDesc[i].timer == instance) && (kDesc[i].timChannel == timChannel))
        {
            return (int8_t)i;
        }
    }
    return -1;
}

/* Forward declaration -- PfmInput_OnShotEnd() (defined before this
   function's own body, further down) needs to call it for the
   tail-flush fix; see that function's own comment. */
static void ProcessDmaChunk(uint8_t idx, uint16_t startOffset, uint16_t count);

static TIM_HandleTypeDef *HandleForTimer(const TIM_TypeDef *instance)
{
    if (instance == TIM2) { return &htim2; }
    if (instance == TIM3) { return &htim3; }
    if (instance == TIM4) { return &htim4; }
    if (instance == TIM5) { return &htim5; }
    return NULL;
}

/* --------------------------------------------------------------------------
 * PfmInput_Init()
 *
 * No prescaler on any of the 4 counters -- full 170 MHz kernel-clock
 * resolution (APB1 undivided, same derivation already documented for
 * HRTIM). Confirmed adequate for this project's ~100 kHz-ish PFM_Input
 * signal range on BOTH the 32-bit (TIM2/TIM5) and 16-bit (TIM3/TIM4)
 * counters involved -- see docs/changelog.txt for the numbers. Each
 * counter's ARR is left at its natural maximum (0xFFFFFFFF for
 * TIM2/TIM5, 0xFFFF for TIM3/TIM4) so it free-runs across its full
 * width rather than wrapping early.
 * -------------------------------------------------------------------------- */
static void InitOneTimer(TIM_HandleTypeDef *htim, TIM_TypeDef *instance, uint32_t period)
{
    TIM_IC_InitTypeDef icCfg = {0};

    htim->Instance               = instance;
    htim->Init.Prescaler         = 0U;
    htim->Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim->Init.Period            = period;
    htim->Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim->Init.RepetitionCounter = 0U;
    htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_IC_Init(htim) != HAL_OK)
    {
        Error_Handler();
    }

    /* RISING only, set once here and never changed again -- see this
       file's top comment for why (period-only measurement, no
       in-ISR polarity flip). No ICFilter: no evidence a filter is
       needed for this signal, left at 0. */
    icCfg.ICPolarity  = TIM_ICPOLARITY_RISING;
    icCfg.ICSelection = TIM_ICSELECTION_DIRECTTI;
    icCfg.ICPrescaler = TIM_ICPSC_DIV1;
    icCfg.ICFilter    = 0U;

    for (uint8_t i = 0U; i < PFM_INPUT_NUM_CHANNELS; i++)
    {
        if ((kDesc[i].timer == instance) && (ChannelIsActive(i) != 0U))
        {
            if (HAL_TIM_IC_ConfigChannel(htim, &icCfg, kDesc[i].timChannel) != HAL_OK)
            {
                Error_Handler();
            }
        }
    }
}

/* One (timer, IRQn, ARR) row per instance -- iterated below so a timer
   with no active channel (PFM_INPUT_ACTIVE_CHANNEL_MASK) is skipped
   ENTIRELY: no HAL_TIM_IC_Init(), no clock enable (InitOneTimer's own
   HAL_TIM_IC_Init() -> HAL_TIM_IC_MspInit() path is what enables the
   peripheral clock), no NVIC priority/enable, no GPIO AF config. Not
   just "initialized but never armed" -- genuinely untouched, per
   pfm_input.h's PFM_INPUT_ACTIVE_CHANNEL_MASK comment (added
   2026-09-09 to cut CPU/interrupt load while a real-hardware timing
   investigation is open -- see docs/changelog.txt). */
typedef struct
{
    TIM_HandleTypeDef *htim;
    TIM_TypeDef        *instance;
    IRQn_Type           irqn;
    uint32_t            arr;
} PfmInputTimerRow_t;

void PfmInput_Init(void)
{
    const PfmInputTimerRow_t rows[] =
    {
        { &htim2, TIM2, TIM2_IRQn, 0xFFFFFFFFU },
        { &htim5, TIM5, TIM5_IRQn, 0xFFFFFFFFU },
        { &htim3, TIM3, TIM3_IRQn, 0xFFFFU },
        { &htim4, TIM4, TIM4_IRQn, 0xFFFFU },
    };

    for (uint8_t r = 0U; r < (sizeof(rows) / sizeof(rows[0])); r++)
    {
        if (TimerHasActiveChannel(rows[r].instance) == 0U)
        {
            continue;   /* no active channel on this timer -- leave it alone */
        }

        InitOneTimer(rows[r].htim, rows[r].instance, rows[r].arr);

        /* Priority (1,0) -- tied with the GateDriverStatus EXTI
           interrupts and (again, as of 2026-09-09 -- see
           HRTIM1_EnableMasterInterrupt()'s own doc comment in hrtim.c
           for why HRTIM1_Master_IRQn moved back to 1) HRTIM1_Master
           itself. Harmless either way: with capture now DMA-driven,
           this TIMx_IRQn is only used for the (now essentially
           unused, since IC DMA mode doesn't route capture events
           through it -- kept for safety/completeness) timer
           update/error paths -- the real per-edge work moved to the
           DMA channel's own IRQn, set up in HAL_TIM_IC_MspInit()
           below, which is DELIBERATELY at a LOWER priority than
           HRTIM1_Master (see that IRQn's own priority comment there)
           -- unlike this essentially-idle IRQn, that one does real,
           non-trivial batch work every 16 captured edges and was
           confirmed on real hardware to starve HRTIM1_Master when
           left this high. */
        HAL_NVIC_SetPriority(rows[r].irqn, 1U, 0U);
        HAL_NVIC_EnableIRQ(rows[r].irqn);
    }
}

void PfmInput_Arm(uint16_t m)
{
    uint16_t clamped = (m > (uint16_t)PFM_INPUT_MAX_PERIODS) ? (uint16_t)PFM_INPUT_MAX_PERIODS : m;

    for (uint8_t i = 0U; i < PFM_INPUT_NUM_CHANNELS; i++)
    {
        /* Defensive, added alongside continuous mode (2026-09-09): a
           channel currently owned by PfmInput_StartContinuous() (e.g.
           a PID feedback channel) must not have its capture state
           reset out from under it by an operator accidentally sending
           PFMIN:CAPTURE -- leave it untouched. targetCount stays 0 for
           it either way, so PfmInput_OnShotStart() below won't try to
           start a second, colliding DMA transfer on it. */
        if (g_state[i].running != 0U)
        {
            continue;
        }

        g_state[i].haveFirstRise    = 0U;
        g_state[i].lastRiseTick     = 0U;
        g_state[i].capturedCount    = 0U;
        g_state[i].overcaptureCount = 0U;
        g_state[i].targetCount      = (ChannelIsActive(i) != 0U) ? clamped : 0U;
        g_dmaNextOffset[i]          = 0U;
    }
}

void PfmInput_OnShotStart(void)
{
    for (uint8_t i = 0U; i < PFM_INPUT_NUM_CHANNELS; i++)
    {
        if (g_state[i].targetCount == 0U)
        {
            continue;   /* never armed for this shot */
        }

        TIM_HandleTypeDef *htim = HandleForTimer(kDesc[i].timer);
        if (htim != NULL)
        {
            g_dmaStartStatus[i] = HAL_TIM_IC_Start_DMA(htim, kDesc[i].timChannel,
                                                       g_dmaBuf[i], PFM_DMA_BUF_LEN);
            if (g_dmaStartStatus[i] == HAL_OK)
            {
                g_state[i].running = 1U;   /* see PfmInputState_t's own comment */
            }
        }
    }
}

uint8_t PfmInput_GetDmaStartStatus(uint8_t channel)
{
    if (channel >= PFM_INPUT_NUM_CHANNELS)
    {
        return 0xFFU;
    }
    return (uint8_t)g_dmaStartStatus[channel];
}

void PfmInput_OnShotEnd(void)
{
    for (uint8_t i = 0U; i < PFM_INPUT_NUM_CHANNELS; i++)
    {
        /* A channel that was never armed, or already finished on its
           own (capturedCount reached targetCount, which already
           called HAL_TIM_IC_Stop_DMA() -- see ProcessDmaChunk()),
           has nothing to stop here. HAL_TIM_IC_Stop_DMA() on a channel
           that isn't running is otherwise harmless, but this avoids
           the call entirely for the common case. */
        if ((g_state[i].targetCount == 0U) ||
            (g_state[i].capturedCount >= g_state[i].targetCount))
        {
            continue;
        }

        /* BUGFIX, real hardware, 2026-09-09: flush whatever's already
           landed in g_dmaBuf[i][] since the last processed chunk
           BEFORE stopping DMA -- without this, a shot ending mid-chunk
           (before the next half/full DMA interrupt) silently loses up
           to PFM_DMA_BUF_LEN/2-1 real edges that DMA already captured
           but nothing ever processed. Confirmed on real hardware: a
           200-period capture on channel 1 reached only 191 before the
           shot's own natural end stopped it -- exactly this
           mechanism. CNDTR counts DOWN from PFM_DMA_BUF_LEN to 0 then
           reloads (circular mode), so PFM_DMA_BUF_LEN - CNDTR is the
           current write position; only flush the gap since
           g_dmaNextOffset (clamped defensively to less than one full
           half-buffer -- the normal case -- rather than trusting an
           unexpected wrap, which should never happen at these
           capture rates but isn't worth risking a bad array index
           over). */
        {
            uint32_t cndtr = pfmInputDma[i].Instance->CNDTR;
            uint16_t pos = (uint16_t)((cndtr == 0U) ? 0U : (PFM_DMA_BUF_LEN - cndtr));
            uint16_t expected = g_dmaNextOffset[i];
            uint16_t newCount = (pos >= expected) ? (uint16_t)(pos - expected) : 0U;

            if (newCount > ((PFM_DMA_BUF_LEN / 2U) - 1U))
            {
                newCount = (PFM_DMA_BUF_LEN / 2U) - 1U;   /* defensive clamp, shouldn't trigger */
            }
            if (newCount > 0U)
            {
                ProcessDmaChunk(i, expected, newCount);
            }
        }

        /* Re-check: the flush above may have reached targetCount on
           its own (calling HAL_TIM_IC_Stop_DMA() already, inside
           ProcessDmaChunk()) -- stopping again here would be
           harmless but redundant, so only stop if it's still running. */
        if (g_state[i].capturedCount < g_state[i].targetCount)
        {
            TIM_HandleTypeDef *htim = HandleForTimer(kDesc[i].timer);
            if (htim != NULL)
            {
                HAL_TIM_IC_Stop_DMA(htim, kDesc[i].timChannel);
            }
        }

        /* Consumed either way -- a channel that timed out short of
           its target should not silently keep listening into some
           later, unrelated shot. */
        g_state[i].targetCount = 0U;
        g_state[i].running     = 0U;   /* see PfmInputState_t's own comment */
    }
}

uint16_t PfmInput_GetCount(uint8_t channel)
{
    if (channel >= PFM_INPUT_NUM_CHANNELS)
    {
        return 0U;
    }
    return g_state[channel].capturedCount;
}

const uint32_t *PfmInput_GetPeriods(uint8_t channel)
{
    if (channel >= PFM_INPUT_NUM_CHANNELS)
    {
        return NULL;
    }
    return g_state[channel].period;
}

uint16_t PfmInput_GetOvercaptureCount(uint8_t channel)
{
    if (channel >= PFM_INPUT_NUM_CHANNELS)
    {
        return 0U;
    }
    return g_state[channel].overcaptureCount;
}

uint8_t PfmInput_StartContinuous(uint8_t channel)
{
    if ((channel >= PFM_INPUT_NUM_CHANNELS) || (ChannelIsActive(channel) == 0U))
    {
        return 0U;
    }
    if (g_state[channel].running != 0U)
    {
        return 0U;   /* already running -- continuous or an armed bench capture */
    }

    TIM_HandleTypeDef *htim = HandleForTimer(kDesc[channel].timer);
    if (htim == NULL)
    {
        return 0U;
    }

    g_state[channel].haveFirstRise = 0U;
    g_state[channel].lastRiseTick  = 0U;
    g_state[channel].lastPeriod    = 0U;
    g_dmaNextOffset[channel]       = 0U;

    g_dmaStartStatus[channel] = HAL_TIM_IC_Start_DMA(htim, kDesc[channel].timChannel,
                                                      g_dmaBuf[channel], PFM_DMA_BUF_LEN);
    if (g_dmaStartStatus[channel] != HAL_OK)
    {
        return 0U;
    }

    g_state[channel].continuous = 1U;
    g_state[channel].running    = 1U;
    return 1U;
}

void PfmInput_StopContinuous(uint8_t channel)
{
    if (channel >= PFM_INPUT_NUM_CHANNELS)
    {
        return;
    }
    if (g_state[channel].continuous == 0U)
    {
        return;   /* not running in continuous mode -- leave a bench capture alone */
    }

    TIM_HandleTypeDef *htim = HandleForTimer(kDesc[channel].timer);
    if (htim != NULL)
    {
        HAL_TIM_IC_Stop_DMA(htim, kDesc[channel].timChannel);
    }
    g_state[channel].continuous = 0U;
    g_state[channel].running    = 0U;
}

uint32_t PfmInput_GetLatestPeriod(uint8_t channel)
{
    if (channel >= PFM_INPUT_NUM_CHANNELS)
    {
        return 0U;
    }
    return g_state[channel].lastPeriod;
}

/* --------------------------------------------------------------------------
 * ProcessDmaChunk() -- shared by both DMA batch callbacks below.
 * Processes `count` freshly-DMA'd raw tick values starting at
 * g_dmaBuf[idx][startOffset], exactly the same period/wraparound
 * logic the old per-edge ISR used, just iterating a batch instead of
 * a single value. Stops early (and calls HAL_TIM_IC_Stop_DMA()) the
 * moment targetCount is reached, ignoring any remaining entries in
 * this chunk -- they're stale/don't-care once a channel is done.
 * -------------------------------------------------------------------------- */
static void ProcessDmaChunk(uint8_t idx, uint16_t startOffset, uint16_t count)
{
    PfmInputState_t *st = &g_state[idx];

    /* 16-bit-counter wraparound mask -- see this file's top comment
       (DMA REDESIGN section) and TIM3/TIM4's real-hardware findings,
       2026-09-09. Unchanged from the old per-edge design, just
       computed once per chunk instead of once per edge. Needed by
       both modes below, so computed once here rather than twice. */
    uint32_t mask = kDesc[idx].is16Bit ? 0xFFFFU : 0xFFFFFFFFU;

    /* Diagnostic: coarser-grained than the old per-edge check (this
       can only say "at least one overcapture happened somewhere in
       this half-buffer's worth of edges", not which one), but still
       meaningful -- CCxOF is a sticky hardware flag, so it survives
       from whenever it was set until explicitly cleared here.
       Expected to read 0 in normal operation: DMA services CCRx far
       faster than the old software ISR did, so this should be even
       less likely to fire now than before. Unconditional, applies to
       both capture modes below. */
    {
        TIM_HandleTypeDef *htim = HandleForTimer(kDesc[idx].timer);
        if (htim != NULL)
        {
            uint32_t ocFlag = (kDesc[idx].timChannel == TIM_CHANNEL_1) ? TIM_FLAG_CC1OF : TIM_FLAG_CC2OF;
            if (__HAL_TIM_GET_FLAG(htim, ocFlag) != 0U)
            {
                st->overcaptureCount++;
                __HAL_TIM_CLEAR_FLAG(htim, ocFlag);
            }
        }
    }

    /* Continuous ("free-running") mode -- added 2026-09-09, see
       pfm_input.h's own comment on PfmInput_StartContinuous(). Handled
       as its own early branch, BEFORE the targetCount-based early
       return below: targetCount is always 0 for a continuous channel
       (PfmInput_Arm() was never called for it), so falling through to
       that check would incorrectly look "already finished" and never
       process anything. No array accumulation here (can't overflow
       PFM_INPUT_MAX_PERIODS since nothing is ever appended), no target
       count, never stops DMA on its own -- just keeps lastPeriod
       current. */
    if (st->continuous != 0U)
    {
        for (uint16_t k = 0U; k < count; k++)
        {
            uint32_t tick = g_dmaBuf[idx][(uint16_t)(startOffset + k)];

            if (st->haveFirstRise == 0U)
            {
                st->haveFirstRise = 1U;   /* first rise -- reference only */
            }
            else
            {
                st->lastPeriod = (tick - st->lastRiseTick) & mask;
            }
            st->lastRiseTick = tick;
        }
        g_dmaNextOffset[idx] = (uint16_t)((startOffset + count) % PFM_DMA_BUF_LEN);
        return;
    }

    /* Bounded bench-capture mode (unchanged below, aside from also
       keeping lastPeriod current -- see PfmInput_GetLatestPeriod()'s
       own comment on why that's meaningful here too). Already finished
       (see the old per-edge callback's own comment, kept for the same
       reason: a DMA batch already in flight when the target is reached
       should be a clean no-op, not an out-of-bounds write). */
    if (st->capturedCount >= st->targetCount)
    {
        return;
    }

    for (uint16_t k = 0U; k < count; k++)
    {
        uint32_t tick = g_dmaBuf[idx][(uint16_t)(startOffset + k)];

        if (st->haveFirstRise == 0U)
        {
            /* First rise since arming -- establishes the reference
               only, no period to close yet. */
            st->haveFirstRise = 1U;
        }
        else
        {
            uint16_t n = st->capturedCount;
            st->period[n]     = (tick - st->lastRiseTick) & mask;
            st->lastPeriod    = st->period[n];
            st->capturedCount = (uint16_t)(n + 1U);

            if (st->capturedCount >= st->targetCount)
            {
                TIM_HandleTypeDef *htim = HandleForTimer(kDesc[idx].timer);
                if (htim != NULL)
                {
                    HAL_TIM_IC_Stop_DMA(htim, kDesc[idx].timChannel);
                }
                /* targetCount deliberately left as-is (nonzero) --
                   PfmInput_GetCount()/DATA? still need to know a
                   capture happened here; PfmInput_OnShotEnd()'s own
                   capturedCount >= targetCount check is what tells it
                   this channel needs no further action. Remaining
                   entries in this chunk (if any) are stale -- ignore. */
                st->running = 0U;   /* see PfmInputState_t's own comment */
                return;
            }
        }

        st->lastRiseTick = tick;
    }

    /* Record where the NEXT chunk (periodic or the final flush in
       PfmInput_OnShotEnd()) should start reading from -- see
       g_dmaNextOffset's own comment above. Only reached on the
       "processed the whole chunk without hitting targetCount" path;
       the two early returns above (already finished / just reached
       target this chunk) both mean DMA is being or already was
       stopped, so tracking is moot there. */
    g_dmaNextOffset[idx] = (uint16_t)((startOffset + count) % PFM_DMA_BUF_LEN);
}

/* --------------------------------------------------------------------------
 * HAL_TIM_IC_CaptureHalfCpltCallback() / HAL_TIM_IC_CaptureCallback()
 * -- called from the DMA channel's own IRQHandler (stm32g4xx_it.c's
 * DMA1_ChannelN_IRQHandler() -> HAL_DMA_IRQHandler() -> HAL's own
 * TIM_DMACaptureHalfCplt()/TIM_DMACaptureCplt()), with htim->Channel
 * set to whichever channel (HAL_TIM_ACTIVE_CHANNEL_1/2) this DMA
 * stream belongs to. Half-complete means the FIRST half of
 * g_dmaBuf[idx][] (indices 0..LEN/2-1) was just filled; (full)
 * complete means the SECOND half (indices LEN/2..LEN-1) was just
 * filled and the buffer is about to wrap. Together these process
 * every entry exactly once, in order, with no gap and no overlap.
 * -------------------------------------------------------------------------- */
void HAL_TIM_IC_CaptureHalfCpltCallback(TIM_HandleTypeDef *htim)
{
    uint32_t timChannel = ActiveChannelToTimChannel(htim->Channel);
    int8_t   idx = FindChannelIndex(htim->Instance, timChannel);

    if (idx < 0)
    {
        return;   /* not one of ours -- shouldn't happen, defensive only */
    }

    ProcessDmaChunk((uint8_t)idx, 0U, PFM_DMA_BUF_LEN / 2U);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    uint32_t timChannel = ActiveChannelToTimChannel(htim->Channel);
    int8_t   idx = FindChannelIndex(htim->Instance, timChannel);

    if (idx < 0)
    {
        return;   /* not one of ours -- shouldn't happen, defensive only */
    }

    ProcessDmaChunk((uint8_t)idx, PFM_DMA_BUF_LEN / 2U, PFM_DMA_BUF_LEN / 2U);
}

/* --------------------------------------------------------------------------
 * HAL_TIM_IC_MspInit()/MspDeInit() -- GPIO AF config + peripheral clock
 * enable for whichever of TIM2/TIM3/TIM4/TIM5 is being initialized,
 * dispatched on htim->Instance since all 4 timers share this one
 * callback, PLUS (2026-09-09) the DMA channel setup for every active
 * channel on this timer -- see kDesc[] at the top of this file for
 * the pin/AF/DMA table.
 * -------------------------------------------------------------------------- */
void HAL_TIM_IC_MspInit(TIM_HandleTypeDef *htim)
{
    GPIO_InitTypeDef gpioInit = {0};

    if (htim->Instance == TIM2)      { __HAL_RCC_TIM2_CLK_ENABLE(); }
    else if (htim->Instance == TIM3) { __HAL_RCC_TIM3_CLK_ENABLE(); }
    else if (htim->Instance == TIM4) { __HAL_RCC_TIM4_CLK_ENABLE(); }
    else if (htim->Instance == TIM5) { __HAL_RCC_TIM5_CLK_ENABLE(); }
    else                             { return; }

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /* All 6 possible DMA channels live on DMA1 -- one clock enable
       covers whichever of them this build actually uses. DMAMUX1 is a
       SEPARATE peripheral on G4 (does the actual request routing,
       e.g. "TIM2_CH1 -> DMA1_Channel1") -- BUGFIX, real hardware,
       2026-09-09: without its own clock enabled, DMA1 itself reports
       success (HAL_TIM_IC_Start_DMA() returns HAL_OK) but no request
       ever actually reaches it, so no transfers ever happen -- capture
       silently reports 0 periods with no error anywhere. Confirmed via
       a temporary PFMIN:DMASTAT? debug command (commands.c) showing
       HAL_OK while PFMIN:STATUS? stayed at 0. */
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_DMAMUX1_CLK_ENABLE();

    gpioInit.Mode  = GPIO_MODE_AF_PP;
    gpioInit.Pull  = GPIO_NOPULL;
    gpioInit.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

    for (uint8_t i = 0U; i < PFM_INPUT_NUM_CHANNELS; i++)
    {
        if ((kDesc[i].timer != htim->Instance) || (ChannelIsActive(i) == 0U))
        {
            continue;
        }

        gpioInit.Pin       = kDesc[i].pin;
        gpioInit.Alternate = kDesc[i].af;
        HAL_GPIO_Init(kDesc[i].port, &gpioInit);

        /* DMA channel setup for this PFM_Input channel -- circular
           mode (continuous capture, matching the old IT design's
           "keep capturing until targetCount, then explicitly stop"
           behavior), peripheral-to-memory, no peripheral increment
           (always reading the same CCRx register), memory increment
           through g_dmaBuf[i][] (word-aligned, matches CCRx's own
           32-bit register width even on TIM3/TIM4's 16-bit counters --
           the upper 16 bits just read as zero). Priority: medium --
           no evidence yet that this needs tuning; revisit if multiple
           simultaneous channels show any sign of contention. */
        pfmInputDma[i].Instance                 = kDesc[i].dmaInstance;
        pfmInputDma[i].Init.Request              = kDesc[i].dmaRequest;
        pfmInputDma[i].Init.Direction            = DMA_PERIPH_TO_MEMORY;
        pfmInputDma[i].Init.PeriphInc            = DMA_PINC_DISABLE;
        pfmInputDma[i].Init.MemInc               = DMA_MINC_ENABLE;
        pfmInputDma[i].Init.PeriphDataAlignment  = DMA_PDATAALIGN_WORD;
        pfmInputDma[i].Init.MemDataAlignment     = DMA_MDATAALIGN_WORD;
        pfmInputDma[i].Init.Mode                 = DMA_CIRCULAR;
        pfmInputDma[i].Init.Priority             = DMA_PRIORITY_MEDIUM;

        if (HAL_DMA_Init(&pfmInputDma[i]) != HAL_OK)
        {
            Error_Handler();
        }

        if (kDesc[i].timChannel == TIM_CHANNEL_1)
        {
            __HAL_LINKDMA(htim, hdma[TIM_DMA_ID_CC1], pfmInputDma[i]);
        }
        else
        {
            __HAL_LINKDMA(htim, hdma[TIM_DMA_ID_CC2], pfmInputDma[i]);
        }

        /* Priority 3 (was 1), DEMOTED, confirmed on real hardware,
           2026-09-09 -- see HRTIM1_EnableMasterInterrupt()'s own doc
           comment in hrtim.c for the full writeup. Root cause of a
           real-hardware bug this session found (a long PFM table
           hold run leaves real output ~77 periods behind the table):
           this ISR runs HAL_TIM_IC_CaptureHalfCpltCallback()/
           CaptureCallback(), which batch-processes 16 already-DMA-
           captured edges at a time -- genuine, non-trivial work, not
           the near-instant per-edge register read the old IT design
           had. Left at priority 1 (tied with, so able to preempt,
           HRTIM1_Master_IRQn's old priority 2), that batch work
           delayed PFM's table-advance ISR by up to ~18500 cycles
           (~109 us) roughly every 8 periods -- confirmed with the new
           raw per-call gap log (PFM_GetDiagGapLog(), pfm.c,
           PFM:GAPLOG?), where every spike lined up exactly with a
           16-edge DMA half/full-transfer boundary. Demoting to 3
           (below HRTIM1_Master's restored priority 1 AND below the
           GateDriverStatus/TIMx_IRQn tier at 2) costs nothing here:
           the actual edge CAPTURE is DMA hardware, autonomous of any
           IRQ priority -- only this batch-processing ISR's own
           completion needs to happen before the 32-entry circular
           buffer wraps a further 16 entries, a window of ~8 more
           periods (worlds more headroom than HRTIM1_Master's own
           ISR, which is short enough to add negligible delay even if
           it preempts this one repeatedly). */
        HAL_NVIC_SetPriority(kDesc[i].dmaIrqn, 3U, 0U);
        HAL_NVIC_EnableIRQ(kDesc[i].dmaIrqn);
    }
}

void HAL_TIM_IC_MspDeInit(TIM_HandleTypeDef *htim)
{
    for (uint8_t i = 0U; i < PFM_INPUT_NUM_CHANNELS; i++)
    {
        if ((kDesc[i].timer == htim->Instance) && (ChannelIsActive(i) != 0U))
        {
            HAL_GPIO_DeInit(kDesc[i].port, kDesc[i].pin);
            HAL_NVIC_DisableIRQ(kDesc[i].dmaIrqn);
            HAL_DMA_DeInit(&pfmInputDma[i]);
        }
    }
}

#else /* PFM_INPUT_FEATURE_ENABLED == 0 */

void PfmInput_Init(void) { }
void PfmInput_Arm(uint16_t m) { (void)m; }
void PfmInput_OnShotStart(void) { }
void PfmInput_OnShotEnd(void) { }
uint16_t PfmInput_GetCount(uint8_t channel) { (void)channel; return 0U; }
const uint32_t *PfmInput_GetPeriods(uint8_t channel) { (void)channel; return NULL; }
uint16_t PfmInput_GetOvercaptureCount(uint8_t channel) { (void)channel; return 0U; }
uint8_t PfmInput_GetDmaStartStatus(uint8_t channel) { (void)channel; return 0xFFU; }
uint8_t PfmInput_StartContinuous(uint8_t channel) { (void)channel; return 0U; }
void PfmInput_StopContinuous(uint8_t channel) { (void)channel; }
uint32_t PfmInput_GetLatestPeriod(uint8_t channel) { (void)channel; return 0U; }

#endif /* PFM_INPUT_FEATURE_ENABLED */
