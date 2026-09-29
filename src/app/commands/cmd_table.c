/*
 * cmd_table.c
 *
 * Legacy PFM table path: TABle:*, the max-carrier-frequency
 * config that bounds it, FIRE, PFM:DIAG?/GAPLOG?.
 * Handlers are declared in commands.h and registered in command_table.c.
 */

#include "commands.h"
#include "cmd_common.h"
#include "ctrlr_config.h"
#include "pfm.h"
#include "hrtim.h"
#include "state_machine.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* --------------------------------------------------------------------------
 * PFM table upload
 *
 * TABle:BEGin -> zero or more TABle:STEP <per> <cmp0> ... <cmp(N-1)>
 * (N = HRTIM_NUM_CHANNELS, ctrlr_config.h) -> TABle:END. Construction
 * (what the actual per/cmp values for a given shot profile should be)
 * happens entirely off-controller, in python/pfm_table_upload.py --
 * this layer just accepts whatever PFM_Step_t values it's given and
 * writes them in, one at a time. See pfm.h's "ADDED" header note for
 * why that split was a deliberate project decision, not a placeholder
 * for a builder that's coming later.
 *
 * g_tableUploadActive is deliberately local to this file, not pfm.c --
 * "is an upload session open" is protocol-layer state, not table
 * data, matching how the rest of this codebase keeps that kind of
 * bookkeeping in the command layer (see cmd_boot()'s own comments for the
 * same principle applied elsewhere).
 * -------------------------------------------------------------------------- */
static uint8_t g_tableUploadActive = 0U;

void cmd_table_begin(uart_instance_t *inst, char *args)
{
    (void)args;

    PFM_TableReset();
    g_tableUploadActive = 1U;

    uart_send(inst, "OK\r\n");
}

/* per + one compare value per channel -- see ctrlr_config.h's
   HRTIM_NUM_CHANNELS. */
#define TABLE_STEP_TOKEN_COUNT  (1U + HRTIM_NUM_CHANNELS)

/* PFM_MAX_CARRIER_FREQ_HZ (ctrlr_config.h) -- RUNTIME-CONFIGURABLE as
   of 2026-09-22, direct request. Backs CONFig:MaxCarrierHz
   (below/command_table.c). Was a compile-time-only ceiling (TABLE_STEP_
   MIN_PER used to be a #define derived from it); TableStepMinPer()
   now recomputes the equivalent "smallest allowed per" live from
   g_pfmMaxCarrierFreqHz every call instead. Same formula as
   python/pfm_table_upload.py's own per_from_freq() (round(clock/freq)
   - 1); the compile-time default (100000) still divides exactly
   (170000000/100000 = 1700, no rounding to differ over) -- an
   operator-chosen value may not, same "plain integer division,
   truncates toward the SAFER (higher-per/lower-frequency) side" note
   that always applied here, not a new caveat. See ctrlr_config.h's own
   comment on PFM_MAX_CARRIER_FREQ_HZ for why this limit exists at
   all. */
static uint32_t g_pfmMaxCarrierFreqHz = PFM_MAX_CARRIER_FREQ_HZ;

/* CONFig:MaxCarrierHz's setter -- `hz` must be > 0 and large enough
   that HRTIM_TIMER_CLK_HZ/hz - 1 still fits a uint16_t (TableStepMinPer()'s
   own return type, and PER's real 16-bit hardware register width) --
   rejecting rather than silently truncating a too-small `hz` into a
   wrapped, wrong minimum. Roughly hz >= 2595 at this board's fixed
   170 MHz HRTIM clock (170000000/65536, rounded up) -- deliberately
   not hardcoded as a named constant here since it's a DERIVED bound
   from HRTIM_TIMER_CLK_HZ, not an independent design choice like
   PID_LOOP_RATE_HZ_MIN/_MAX (pid.c). */
uint8_t PFM_SetMaxCarrierFreqHz(uint32_t hz)
{
    if (hz == 0U)
    {
        return 0U;
    }
    uint32_t rawMinPer = (HRTIM_TIMER_CLK_HZ / hz);
    if ((rawMinPer == 0U) || (rawMinPer > 65536U))
    {
        return 0U;
    }
    g_pfmMaxCarrierFreqHz = hz;
    return 1U;
}

uint32_t PFM_GetMaxCarrierFreqHz(void)
{
    return g_pfmMaxCarrierFreqHz;
}

/* Smallest `per` allowed by the CURRENT g_pfmMaxCarrierFreqHz ceiling
   -- per is INVERSELY related to frequency (freq = HRTIM_TIMER_CLK_HZ
   / (per+1)), so capping frequency means a MINIMUM per, not a
   maximum. Recomputed live on every call (was TABLE_STEP_MIN_PER, a
   compile-time #define, before g_pfmMaxCarrierFreqHz existed) --
   cheap enough (one division) that caching isn't worth the
   staleness risk if CONFig:MaxCarrierHz changes between calls. */
static uint16_t TableStepMinPer(void)
{
    return (uint16_t)((HRTIM_TIMER_CLK_HZ / g_pfmMaxCarrierFreqHz) - 1UL);
}

/* Builds the "wrong token count" error text at runtime -- the count
   itself (1 + HRTIM_NUM_CHANNELS) is only known as a preprocessor
   expression, not a plain literal, so the standard #x/two-level
   stringification trick doesn't work here: it would stringify the
   unevaluated text "(1U + (4U))" rather than the computed value "5".
   snprintf() with %u sidesteps that entirely. */
static void SendTableStepCountErr(uart_instance_t *inst)
{
    char msg[64];
    snprintf(msg, sizeof(msg),
             "TABLE:STEP needs exactly %u values: per cmp0 .. cmp(N-1)",
             (unsigned int)TABLE_STEP_TOKEN_COUNT);
    SendErr(inst, ERR_TABLE_STEP_INVALID, msg);
}

void cmd_table_step(uart_instance_t *inst, char *args)
{
    char *tok;
    long  vals[TABLE_STEP_TOKEN_COUNT];
    int   n = 0;
    uint16_t cmp[HRTIM_NUM_CHANNELS];

    if (g_tableUploadActive == 0U)
    {
        SendErr(inst, ERR_TABLE_NOT_UPLOADING, "Not currently uploading -- send TABLE:BEGIN first");
        return;
    }

    if (args == NULL)
    {
        SendTableStepCountErr(inst);
        return;
    }

    for (tok = strtok(args, " \r\n"); tok != NULL; tok = strtok(NULL, " \r\n"))
    {
        long v;

        if (n >= (int)TABLE_STEP_TOKEN_COUNT)
        {
            /* One extra token showed up -- too many values, not a
               valid step. */
            n++;
            break;
        }

        v = atol(tok);
        if (v < 0L || v > 65535L)
        {
            SendErr(inst, ERR_TABLE_STEP_INVALID, "Value out of uint16 range (0-65535)");
            return;
        }

        vals[n] = v;
        n++;
    }

    if (n != (int)TABLE_STEP_TOKEN_COUNT)
    {
        SendTableStepCountErr(inst);
        return;
    }

    if (vals[0] < (long)TableStepMinPer())
    {
        char msg[96];
        snprintf(msg, sizeof(msg),
                 "per below %u implies carrier > %lu Hz (max allowed)",
                 (unsigned int)TableStepMinPer(), (unsigned long)g_pfmMaxCarrierFreqHz);
        SendErr(inst, ERR_CARRIER_TOO_HIGH, msg);
        return;
    }

    for (uint8_t ch = 0U; ch < HRTIM_NUM_CHANNELS; ch++)
    {
        cmp[ch] = (uint16_t)vals[1 + ch];
    }

    if (PFM_AppendStep((uint16_t)vals[0], cmp) == 0U)
    {
        SendErr(inst, ERR_TABLE_FULL, "Table full");
        return;
    }

    uart_send(inst, "OK\r\n");
}

/* CONFig:MaxCarrierHz <hz> / ? -- added 2026-09-22, direct request:
   PFM_MAX_CARRIER_FREQ_HZ (ctrlr_config.h) made runtime-configurable.
   See PFM_SetMaxCarrierFreqHz()'s own doc comment, above, for the
   hardware-register-fit validation applied. Legacy-path-only (only
   TABLE:STEP, above, consults this) -- has no effect on the modern
   SHOT:* path, which has its own separate, already-runtime-
   configurable output bounds (CONFig:PIDRate/TURNONHz/MAXFREQHz). */
void cmd_config_max_carrier_hz(uart_instance_t *inst, char *args)
{
    char *tok;
    long  hz;

    tok = (args != NULL) ? strtok(args, " \r\n") : NULL;
    if (tok == NULL)
    {
        SendErr(inst, ERR_INVALID_ARGS, "CONFig:MaxCarrierHz needs one argument: hz");
        return;
    }
    hz = atol(tok);
    if ((hz <= 0L) || (PFM_SetMaxCarrierFreqHz((uint32_t)hz) == 0U))
    {
        SendErr(inst, ERR_INVALID_ARGS, "hz rejected -- too small/large for the "
                           "16-bit PER register at this board's HRTIM clock");
        return;
    }
    uart_send(inst, "OK\r\n");
}

void cmd_config_max_carrier_hz_query(uart_instance_t *inst, char *args)
{
    char buf[32];
    (void)args;

    snprintf(buf, sizeof(buf), "OK %lu\r\n", (unsigned long)PFM_GetMaxCarrierFreqHz());
    uart_send(inst, buf);
}

void cmd_table_end(uart_instance_t *inst, char *args)
{
    char buf[48];
    (void)args;

    g_tableUploadActive = 0U;

    snprintf(buf, sizeof(buf), "OK %u\r\n", (unsigned int)PFM_GetEntryCount());
    uart_send(inst, buf);
}

void cmd_table_query(uart_instance_t *inst, char *args)
{
    char buf[48];
    (void)args;

    snprintf(buf, sizeof(buf), "OK %u\r\n", (unsigned int)PFM_GetEntryCount());
    uart_send(inst, buf);
}

/* --------------------------------------------------------------------------
 * FIRE
 *
 * Begins PWM output by (re)starting playback of whatever table is
 * currently uploaded, from step 0. No ARM/state-machine interlock
 * exists in this minimal firmware -- FIRE always takes effect
 * immediately, whether the controller was idle or already mid-shot
 * (PFM_Restart() is safe to call in either case: it always resets to
 * step 0 and re-applies HRTIM1_PWM_Start()). If a fuller state machine
 * (ARM/interlock, matching the sibling PFM-STM32G474 project's
 * SM_Fire()) is ever needed here, this is the call site to extend, not
 * replace. Fault gating (below) is the first, narrow piece of that --
 * not a full state machine.
 *
 * Two guards, checked in this order (both cheap, order doesn't
 * otherwise matter): a latched hardware fault (PC10/HRTIM1_FLT6, see
 * hrtim.h) is checked first -- the outputs are already safe in
 * hardware regardless, but firing straight into a still-tripped fault
 * would be a confusing "OK" that then produces no output, with no
 * indication why; then firing an empty table. PFM_CycleBoundaryHandler()
 * (the ISR-driven playback advance, see stm32g4xx_it.c's
 * HRTIM1_Master_IRQHandler()) already defends against g_pfmEntryCount
 * == 0 by stopping outputs again at the very first master-repetition
 * interrupt -- but that would happen silently, after a near-instant
 * blip on the outputs, with no error ever reported to the operator.
 * Rejecting both here instead gives a clear reason up front.
 * -------------------------------------------------------------------------- */


void cmd_fire(uart_instance_t *inst, char *args)
{
    (void)args;

    if ((AnyFaultLatched() != 0U) && (SM_GetFaultBypassEnabled() == 0U))
    {
        SendErr(inst, ERR_FAULT_LATCHED, "Fault latched -- send FAULT:CLEAR first");
        return;
    }

    /* GATED 2026-09-21: FIRE previously bypassed the state machine
       (only a fault check, no ARM/interlock) -- see this project's
       design review. Now requires ARMED, same as SHOT:STARt.
       Deliberately NOT transitioned to FIRING here: the legacy table
       path is not tracked by the SM's IDLE/ARMED/FIRING lifecycle (it
       never calls SM_Fire()/SM_NotifyShotComplete(), see state_machine.h),
       and marking it FIRING would make EnterFault()'s fault handlers
       assume pid.c's PID_Update() loop is running when it isn't. So
       ARMED is a required precondition only; a fault during playback
       still safes the legacy output (EnterFault() from ARMED ->
       PFM_ForceStop()). */
    if (RequireArmedAndEnabled(inst) == 0U)
    {
        return;
    }

    if (PFM_GetEntryCount() == 0U)
    {
        SendErr(inst, ERR_TABLE_EMPTY, "Table is empty -- upload one first (TABLE:BEGIN/STEP/END)");
        return;
    }

    PFM_Restart();

    uart_send(inst, "OK\r\n");
}

/* --------------------------------------------------------------------------
 * PFM:DIAG?
 *
 * Diagnostic, added 2026-09-09 to test the "HRTIM1_Master's own
 * interrupt is being starved by higher-priority TIM2-5 capture ISRs"
 * theory for a real-hardware table-advance stall found this session
 * (a real PFM output holding one table entry for far longer than the
 * table's own dwell length says it should -- see docs/changelog.txt
 * and pfm.c's own comment on g_diagMaxGapCycles for the full
 * writeup). Reports PFM_GetDiagCounters()'s two numbers for the
 * CURRENT/most recent shot (reset at every FIRE/PFM_Restart()):
 * how many times PFM_CycleBoundaryHandler() has run, and the largest
 * real-time gap seen between two consecutive calls, in both raw CPU
 * cycles and microseconds (170 MHz CPU clock, confirmed against
 * main.c's SystemClock_Config(), not assumed). A max gap far larger
 * than one real period's worth of time is direct evidence of ISR
 * starvation, not just "the table happened to hold one entry a
 * while." Not gated on PFM_INPUT_FEATURE_ENABLED -- this is about the
 * PFM output engine itself, unrelated to whether input capture exists.
 * -------------------------------------------------------------------------- */
void cmd_pfm_diag(uart_instance_t *inst, char *args)
{
    char buf[80];
    uint32_t callCount;
    uint32_t maxGapCycles;
    (void)args;

    PFM_GetDiagCounters(&callCount, &maxGapCycles);

    /* 170 MHz CPU clock -- see this function's own doc comment. */
    uint32_t maxGapUs = maxGapCycles / 170U;

    snprintf(buf, sizeof(buf), "OK %lu %lu %lu\r\n",
             (unsigned long)callCount, (unsigned long)maxGapCycles,
             (unsigned long)maxGapUs);
    uart_send(inst, buf);
}

/* TEMPORARY debug command, 2026-09-09 (second round) -- see pfm.h's
   own comment on PFM_GetDiagGapLog(). Sized like PFMIN:DATA?'s own
   g_pfminDataBuf (cmd_pfmin.c) -- up to PFM_INPUT_MAX_PERIODS entries,
   each up to " 4294967295" (11 chars), plus the "OK <count>" prefix
   and CRLF. Remove once the RAMP/HOLD/RAMP real-hardware lag
   investigation (docs/changelog.txt) is resolved and this diagnostic
   is no longer needed. */
static char g_pfmGapLogBuf[2600];

void cmd_pfm_gaplog(uart_instance_t *inst, char *args)
{
    uint16_t count;
    const uint32_t *log = PFM_GetDiagGapLog(&count);
    int len = 0;
    (void)args;

    len += snprintf(&g_pfmGapLogBuf[len], sizeof(g_pfmGapLogBuf) - (size_t)len,
                     "OK %u", (unsigned int)count);
    for (uint16_t i = 0U; i < count; i++)
    {
        len += snprintf(&g_pfmGapLogBuf[len], sizeof(g_pfmGapLogBuf) - (size_t)len,
                         " %lu", (unsigned long)log[i]);
    }
    snprintf(&g_pfmGapLogBuf[len], sizeof(g_pfmGapLogBuf) - (size_t)len, "\r\n");

    uart_send(inst, g_pfmGapLogBuf);
}
