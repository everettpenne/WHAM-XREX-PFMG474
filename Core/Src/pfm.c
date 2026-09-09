/*
 * pfm.c
 *
 * Table playback engine, ported from the sibling PFM-STM32G474 project
 * -- see pfm.h's header comment for exactly what was and wasn't ported
 * (construction/builders and FEEDBACK mode were deliberately left out),
 * and for the 2026-09-08 generalization from a hardcoded 3 channels to
 * a compile-time HRTIM_NUM_CHANNELS (ctrlr_config.h).
 */

#include "pfm.h"
#include "hrtim.h"
#include "pfm_input.h"
#include <string.h>

static PFM_Step_t g_pfmTable[PFM_TABLE_SIZE];
static uint16_t g_pfmIndex = 0U;
static uint16_t g_holdCounter = 0U;

/* --------------------------------------------------------------------------
 * Diagnostic: real inter-call timing for PFM_CycleBoundaryHandler(),
 * added 2026-09-09 to test the "HRTIM1_Master's own interrupt is being
 * starved by higher-priority TIM2-5 capture ISRs" theory for the
 * real-hardware table-advance stall found this session (a real PFM
 * output holding one table entry for 140+ periods instead of the
 * table's own 10-period dwell -- see docs/changelog.txt).
 *
 * Deliberately NOT just a call counter: PFM_CycleBoundaryHandler()
 * only ever stops a shot once g_pfmIndex reaches the table's entry
 * count, so a COMPLETED shot's call count is always exactly that
 * entry count regardless of whether stalling happened along the way
 * -- it can't distinguish "the ISR ran on schedule" from "the ISR was
 * starved for a long stretch, then caught up." What actually proves
 * or disproves interrupt starvation is the REAL TIME gap between
 * consecutive calls: normal operation should show a gap of roughly
 * one real period every time (a few thousand CPU cycles at 170 MHz);
 * a genuine starvation event would show one or more far larger gaps.
 * Uses the Cortex-M4 DWT cycle counter (DWT->CYCCNT) -- free-running,
 * always available, no peripheral/timer of its own to configure,
 * enabled once in PFM_Init(). CPU clock is 170 MHz, same as
 * HRTIM_TIMER_CLK_HZ (hrtim.h) -- confirmed via main.c's
 * SystemClock_Config(), not assumed -- so cycles and HRTIM ticks are
 * directly comparable 1:1.
 * -------------------------------------------------------------------------- */
static uint32_t g_diagCallCount     = 0U;
static uint32_t g_diagMaxGapCycles  = 0U;
static uint32_t g_diagLastCallCycle = 0U;
static uint8_t  g_diagHaveLastCall  = 0U;

/* Added 2026-09-09, second round: the max-gap metric above proves
   there's no single catastrophic stall, but can't distinguish "always
   ran right on schedule" from "many small, repeated delays" -- and a
   real-hardware test (a long run of identical table entries, RAMP/
   HOLD/RAMP profile) showed the real output falling ~77 real periods
   behind where the table says it should be, with NO single gap large
   enough to explain it (PFM:DIAG? showed 112 us max, ~11x one normal
   period -- nowhere near 77 periods' worth). Logs the RAW gap (cycles)
   for every call this shot, up to PFM_DIAG_GAP_LOG_LEN of them, so a
   delay can be correlated with exactly which table entry it happened
   at instead of just a single aggregate number. Sized to this
   session's own 200-entry test tables (PFM_INPUT_MAX_PERIODS,
   pfm_input.h) -- calls beyond the log's length are still counted
   (g_diagCallCount) and still contribute to g_diagMaxGapCycles, just
   not individually logged. */
#define PFM_DIAG_GAP_LOG_LEN  (PFM_INPUT_MAX_PERIODS)
static uint32_t g_diagGapLog[PFM_DIAG_GAP_LOG_LEN];

/* Number of ENTRIES ACTUALLY BUILT in g_pfmTable[], as opposed to
   PFM_TABLE_SIZE (the fixed buffer capacity). Written only by
   PFM_TableReset() (resets to 0) and PFM_AppendStep() (increments on
   each successful append) -- see those functions, and commands.c's
   TABle:* handlers, which are the only caller today.
   PFM_CycleBoundaryHandler() must check g_pfmIndex against THIS, not
   PFM_TABLE_SIZE -- a short table would otherwise never trigger
   exhaustion, since the index would have to climb all the way to
   PFM_TABLE_SIZE first. */
static uint16_t g_pfmEntryCount = 0U;

static PFM_State_t g_pfmState = PFM_STATE_RUNNING;

/* --------------------------------------------------------------------------
 * Per-channel output enable
 *
 * Standalone state, independent of table construction/playback -- set
 * directly by whatever future command layer adds SET ENABLE/DISABLE,
 * and read by PFM_Restart() whenever a shot actually fires. Deliberately
 * NOT reset by PFM_Init() or PFM_ResetIndices() -- per the sibling
 * project's design decision (carried over here), phase enable/disable
 * is durable and only changes via an explicit call to
 * PFM_SetPhaseEnabled(). All channels default to enabled.
 *
 * Sized by HRTIM_NUM_CHANNELS (ctrlr_config.h) -- was three separate
 * named booleans (g_phaseEnabledU/V/W) before the 2026-09-08
 * generalization; PFM_PHASE_U/V/W (pfm.h) remain as named indices into
 * this same array for callers that want them.
 *
 * Left zero-initialized here (C guarantees static arrays without an
 * explicit initializer start all-zero, via the same .bss-clear the
 * startup code already performs for every other uninitialized global
 * in this project -- no dependency on GCC constructors or
 * __libc_init_array() actually running, which this startup file was
 * never confirmed to call). PFM_Init()'s one-time-only branch below
 * sets every element to 1 (enabled) on the first call, then leaves the
 * array alone on any later call -- see PFM_Init() for why it can't
 * just be a plain loop in the initializer or every call to PFM_Init()
 * would silently undo an operator's PFM_SetPhaseEnabled(), breaking
 * the durability guarantee described above. */
static uint8_t g_channelEnabled[HRTIM_NUM_CHANNELS];

uint16_t PFM_PhaseForChannel(uint16_t per, uint8_t channel)
{
    uint32_t counts = (uint32_t)per + 1U;
    return (uint16_t)(((uint32_t)channel * counts) / HRTIM_NUM_CHANNELS);
}

void PFM_Init(void)
{
    /* g_channelEnabled defaults to all-enabled on the FIRST call only
       (see that array's own comment for why it's not initialized
       inline) -- `firstCall` is itself a static, so it -- like
       g_channelEnabled -- starts at 0 via the normal .bss-clear, no
       constructor tricks needed. Every call after the first leaves
       g_channelEnabled untouched, preserving the durability guarantee
       (an operator's PFM_SetPhaseEnabled() must survive a later
       PFM_Init(), if this is ever called more than once at boot). */
    static uint8_t firstCall = 1U;
    if (firstCall != 0U)
    {
        for (uint8_t ch = 0U; ch < HRTIM_NUM_CHANNELS; ch++)
        {
            g_channelEnabled[ch] = 1U;
        }
        firstCall = 0U;
    }

    /* No table builder is wired up yet (see pfm.h) -- this just brings
       the module to a known-empty, known-quiescent state. */
    g_pfmIndex = 0U;
    g_holdCounter = 0U;
    g_pfmState = PFM_STATE_STOPPED;
    g_pfmEntryCount = 0U;

    /* Enable the DWT cycle counter for the diagnostic above -- TRCENA
       (DEMCR) must be set before DWT->CTRL's own CYCCNTENA bit will
       stick, per the Cortex-M4 debug architecture. Idempotent -- safe
       even if PFM_Init() is ever called more than once. */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

/* Last step actually written to HRTIM by PFM_ApplyCurrentStep(), used to
   skip redundant register writes -- most ISR periods in a real shot
   re-apply an unchanged step (a CONSTANT-style table is many identical
   entries), so most periods would otherwise re-clamp and rewrite every
   HRTIM register with values they already hold. Comparing per + the
   whole cmp[] array replaces that redundant apply on every unchanged
   period. Skipping is safe because the HRTIM preload (shadow) registers
   retain the last written values between update events -- rewriting an
   identical value is a pure no-op electrically.

   g_lastAppliedValid is cleared by PFM_Restart() so the first period of
   every shot always writes, regardless of what any previous shot left
   in the registers. */
static PFM_Step_t g_lastAppliedStep;
static uint8_t    g_lastAppliedValid = 0U;

void PFM_ApplyCurrentStep(void)
{
    const PFM_Step_t *pStep = &g_pfmTable[g_pfmIndex];
    uint16_t phase[HRTIM_NUM_CHANNELS];

    /* phase[] is not stored in PFM_Step_t -- it's a pure function of
       pStep->per, so the duplicate-write check below only needs to
       compare per + cmp[]: if both match, the recomputed phase will
       always match too, by construction. */
    if ((g_lastAppliedValid != 0U) &&
        (pStep->per == g_lastAppliedStep.per) &&
        (memcmp(pStep->cmp, g_lastAppliedStep.cmp, sizeof(pStep->cmp)) == 0))
    {
        return;   /* registers already hold exactly these values */
    }

    g_lastAppliedStep  = *pStep;
    g_lastAppliedValid = 1U;

    phase[0] = 0U;   /* unused -- channel 0 has no phase register, see hrtim.h */
    for (uint8_t ch = 1U; ch < HRTIM_NUM_CHANNELS; ch++)
    {
        phase[ch] = PFM_PhaseForChannel(pStep->per, ch);
    }

    HRTIM1_ApplyPfmStep(pStep->per, pStep->cmp, phase);
}

void PFM_CycleBoundaryHandler(void)
{
    /* Diagnostic instrumentation -- first thing in this function, on
       purpose, so the recorded gap reflects real ISR-to-ISR timing,
       not time spent in the rest of this handler. See this file's own
       comment on g_diagMaxGapCycles above for what this is testing. */
    {
        uint32_t now = DWT->CYCCNT;
        if (g_diagHaveLastCall != 0U)
        {
            uint32_t gap = now - g_diagLastCallCycle;   /* wraps correctly even across DWT->CYCCNT overflow */
            if (gap > g_diagMaxGapCycles)
            {
                g_diagMaxGapCycles = gap;
            }
            if (g_diagCallCount < PFM_DIAG_GAP_LOG_LEN)
            {
                g_diagGapLog[g_diagCallCount] = gap;
            }
        }
        g_diagLastCallCycle = now;
        g_diagHaveLastCall  = 1U;
        g_diagCallCount++;
    }

    /* Hardware fault check (2026-09-08, PC10/HRTIM1_FLT6 -- see
       hrtim.h). By the time this ever reads tripped, the HRTIM
       peripheral has ALREADY forced every fault-enabled channel's
       outputs to their safe level in hardware, autonomously -- this is
       bookkeeping only: stop the counters (so the master-rep ISR
       doesn't keep firing forever underneath outputs that are already
       disabled, same reasoning as the empty-table branch below) and
       get g_pfmState out of RUNNING. Deliberately does NOT clear the
       fault flag itself -- that stays latched (FAULT? keeps reporting
       it, FIRE stays rejected) until an operator explicitly sends
       FAULT:CLEAR (commands.c), matching the project decision that a
       fault silently clearing itself is a hazard, not a convenience. */
    if (HRTIM1_FaultIsTripped() != 0U)
    {
        PFM_ForceStop();
        return;
    }

    /* Defensive: with no table builder wired up (see pfm.h),
       g_pfmEntryCount is always 0 today, so this always takes the stop
       path below on the very first call after a shot starts -- applying
       g_pfmTable[0] of a never-built table would otherwise silently run
       on garbage. This is intentionally the same defensive check the
       sibling project uses, kept ready for the day a builder populates
       real entries. */
    if (g_pfmEntryCount == 0U)
    {
        PFM_ForceStop();
        return;
    }

    PFM_ApplyCurrentStep();

    g_holdCounter++;
    if (g_holdCounter >= PFM_HOLD_PERIODS)
    {
        g_holdCounter = 0U;
        g_pfmIndex++;

        /* Compare against g_pfmEntryCount (the ACTUAL built length),
           not PFM_TABLE_SIZE (the fixed buffer capacity) -- a short
           table would otherwise never be seen as exhausted here. */
        if (g_pfmIndex >= g_pfmEntryCount)
        {
            /* Last entry just completed this cycle.
               Stop outputs now, at a coherent boundary. */
            HRTIM1_PWM_Stop();
            g_pfmIndex = 0U;
            g_pfmState = PFM_STATE_STOPPED;

            /* Normal (non-fault) shot end -- bounds any still-running
               PFM_Input capture to here too. This branch stops
               outputs inline rather than via PFM_ForceStop() (see
               that function's own comment for why -- it doesn't reset
               g_pfmIndex, this branch does), so PfmInput_OnShotEnd()
               needs its own explicit call here as well. */
            PfmInput_OnShotEnd();
        }
    }
}

void PFM_Restart(void)
{
    g_pfmIndex = 0U;
    g_holdCounter = 0U;
    g_pfmState = PFM_STATE_RUNNING;

    /* Fresh diagnostic window for this shot -- see g_diagMaxGapCycles's
       own comment above. g_diagHaveLastCall = 0 means the very first
       PFM_CycleBoundaryHandler() call of this shot won't be scored
       against a call from the PREVIOUS shot (or from before FIRE was
       ever sent). */
    g_diagCallCount     = 0U;
    g_diagMaxGapCycles  = 0U;
    g_diagLastCallCycle = 0U;
    g_diagHaveLastCall  = 0U;
    memset(g_diagGapLog, 0, sizeof(g_diagGapLog));

    /* Every shot must write HRTIM on its first period, no matter what a
       previous shot left in the registers. */
    g_lastAppliedValid = 0U;

    PFM_ApplyCurrentStep();   /* preload step 0 before starting */

    /* Cold-start fix: PFM_ApplyCurrentStep() (above) wrote the step-0
       PER/CMP/phase values to HRTIM shadow registers. With preload
       enabled, those writes do not take effect until the next update
       event -- but the counters have not started yet, so there has
       been no update event, and the active registers still hold the
       stale init defaults from HRTIM1_FullInit() (100 kHz, 50 % duty,
       zero phase offset). Without the software update below, the FIRST
       period of every shot would run with those wrong init values. The
       software update forces an immediate shadow→active transfer so
       the counters, when started one line below, begin their very
       first period with the correct step-0 values. */
    HRTIM1_SoftwareUpdate();

    HRTIM1_PWM_Start(g_channelEnabled);

    /* PFM_Input capture (2026-09-08, pfm_input.h) begins here,
       deliberately the same call as HRTIM1_PWM_Start() above -- this
       is the literal mechanism that makes input capture start exactly
       when a PFM output shot starts, not some separate command run at
       an arbitrary later moment. No-op for any channel never armed
       via PfmInput_Arm(). */
    PfmInput_OnShotStart();
}

void PFM_SetPhaseEnabled(uint8_t channel, uint8_t enabled)
{
    if (channel >= HRTIM_NUM_CHANNELS)
    {
        return;
    }
    g_channelEnabled[channel] = (enabled != 0U) ? 1U : 0U;
}

uint8_t PFM_GetPhaseEnabled(uint8_t channel)
{
    if (channel >= HRTIM_NUM_CHANNELS)
    {
        return 0U;
    }
    return g_channelEnabled[channel];
}

void PFM_ForceStop(void)
{
    /* See pfm.h's doc comment -- the exact stop-and-mark-STOPPED pair
       PFM_CycleBoundaryHandler() already performs inline for its two
       own early-stop cases, extracted so a fault source outside the
       HRTIM master-repetition ISR (gate_driver.c's GateDriverStatus
       EXTI interrupt) can do the same thing and keep PFM_GetState()
       truthful. Deliberately does not touch g_pfmIndex, matching both
       existing call sites. */
    HRTIM1_PWM_Stop();
    g_pfmState = PFM_STATE_STOPPED;

    /* Bounds any still-running PFM_Input capture to the shot that's
       ending here (fault-triggered stop) -- see pfm_input.h. Any
       channel already finished on its own is a no-op. */
    PfmInput_OnShotEnd();
}

void PFM_ResetIndices(void)
{
    /* Quiescent reset: zero g_pfmIndex/g_holdCounter WITHOUT touching
       HRTIM outputs or applying any step. Distinct from PFM_Restart(),
       which is for actually beginning a shot and does start outputs. */
    g_pfmIndex = 0U;
    g_holdCounter = 0U;
    g_pfmState = PFM_STATE_STOPPED;
}

PFM_State_t PFM_GetState(void)
{
    return g_pfmState;
}

const PFM_Step_t *PFM_GetCurrentStep(void)
{
    return &g_pfmTable[g_pfmIndex];
}

const PFM_Step_t *PFM_GetStepByIndex(uint16_t index)
{
    if (index >= PFM_TABLE_SIZE)
    {
        index = 0U;
    }

    return &g_pfmTable[index];
}

void PFM_TableReset(void)
{
    g_pfmEntryCount = 0U;
}

uint8_t PFM_AppendStep(uint16_t per, const uint16_t *cmp)
{
    if (g_pfmEntryCount >= PFM_TABLE_SIZE)
    {
        return 0U;
    }

    g_pfmTable[g_pfmEntryCount].per = per;
    memcpy(g_pfmTable[g_pfmEntryCount].cmp, cmp, sizeof(g_pfmTable[g_pfmEntryCount].cmp));
    g_pfmEntryCount++;

    return 1U;
}

uint16_t PFM_GetEntryCount(void)
{
    return g_pfmEntryCount;
}

void PFM_GetDiagCounters(uint32_t *callCount, uint32_t *maxGapCycles)
{
    if (callCount != NULL)
    {
        *callCount = g_diagCallCount;
    }
    if (maxGapCycles != NULL)
    {
        *maxGapCycles = g_diagMaxGapCycles;
    }
}

const uint32_t *PFM_GetDiagGapLog(uint16_t *count)
{
    uint32_t logged = g_diagCallCount;
    if (logged > PFM_DIAG_GAP_LOG_LEN)
    {
        logged = PFM_DIAG_GAP_LOG_LEN;
    }
    if (count != NULL)
    {
        *count = (uint16_t)logged;
    }
    return g_diagGapLog;
}
