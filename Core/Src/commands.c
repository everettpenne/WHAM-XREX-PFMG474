/*
 * commands.c
 *
 * Command handler implementations for WHAM-XREX-PFMG474.
 *
 * Still minimal by design -- *IDN and BOOT, on top of the ported
 * serial command architecture (uart.c + cmd_parser.c's tokenize/
 * dispatch, wired up in main.c/stm32g4xx_it.c). Add more handlers
 * here as they're needed, following cmd_parser.c's "to add a
 * command" recipe.
 */

#include "commands.h"
#include "uart.h"
#include "ctrlr_config.h"
#include "boot_jump.h"
#include "pfm.h"
#include "hrtim.h"
#include "gate_driver.h"
#include "qspi_test.h"
#include "pfm_input.h"
#include "pid.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void SendErr(uart_instance_t *inst, int code, const char *msg)
{
    char buf[128];
    snprintf(buf, sizeof(buf), "ERR %d %s\r\n", code, msg);
    uart_send(inst, buf);
}

/* --------------------------------------------------------------------------
 * Identification / system
 * -------------------------------------------------------------------------- */

void cmd_idn(uart_instance_t *inst, char *args)
{
    char buf[64];
    (void)args;

    /* Board + firmware identity in one line, matching the sibling
       PFM-STM32G474 project's *IDN convention (OK <value>, space-
       separated fields) -- see ctrlr_config.h for the constants. */
    snprintf(buf, sizeof(buf), "OK %s %s %s\r\n",
             HW_BOARD_NAME, HW_BOARD_REV, FW_VERSION_STRING);
    uart_send(inst, buf);
}

#if (BOOT_JUMP_FEATURE_ENABLED != 0)
void cmd_boot(uart_instance_t *inst, char *args)
{
    (void)args;

    /* No state-machine/Firing concept exists in this minimal firmware
       yet -- when one is added, gate this the same way the sibling
       PFM-STM32G474 project's cmd_boot() does (reject with ERR while
       Firing; resetting under load would drop outputs uncontrolled). */

    /* uart_send() is blocking (HAL_UART_Transmit with HAL_MAX_DELAY), so
       this ACK is guaranteed to be fully on the wire before
       BootJump_RequestBootloader() resets the MCU below -- the operator
       (or a flashing script) sees "OK ENTERING BOOTLOADER" before the
       link drops. */
    uart_send(inst, "OK ENTERING BOOTLOADER\r\n");

    BootJump_RequestBootloader();
    /* Never returns. */
}
#endif /* BOOT_JUMP_FEATURE_ENABLED */

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
 * bookkeeping in commands.c (see cmd_boot()'s own comments for the
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

/* Smallest `per` allowed by ctrlr_config.h's PFM_MAX_CARRIER_FREQ_HZ
   ceiling -- per is INVERSELY related to frequency (freq =
   HRTIM_TIMER_CLK_HZ / (per+1)), so capping frequency means a MINIMUM
   per, not a maximum. Same formula as python/pfm_table_upload.py's own
   per_from_freq() (round(clock/freq) - 1); plain integer division
   here reproduces that exactly for PFM_MAX_CARRIER_FREQ_HZ = 100000
   (170000000/100000 = 1700 exactly, no rounding to differ over) --
   this is a hard compile-time boundary check, not a place to silently
   accept a rounding mismatch against the host tool. See
   ctrlr_config.h's own comment on PFM_MAX_CARRIER_FREQ_HZ for why this
   limit exists. */
#define TABLE_STEP_MIN_PER \
    ((uint16_t)((HRTIM_TIMER_CLK_HZ / PFM_MAX_CARRIER_FREQ_HZ) - 1UL))

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
    SendErr(inst, 4, msg);
}

void cmd_table_step(uart_instance_t *inst, char *args)
{
    char *tok;
    long  vals[TABLE_STEP_TOKEN_COUNT];
    int   n = 0;
    uint16_t cmp[HRTIM_NUM_CHANNELS];

    if (g_tableUploadActive == 0U)
    {
        SendErr(inst, 2, "Not currently uploading -- send TABLE:BEGIN first");
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
            SendErr(inst, 4, "Value out of uint16 range (0-65535)");
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

    if (vals[0] < (long)TABLE_STEP_MIN_PER)
    {
        char msg[96];
        snprintf(msg, sizeof(msg),
                 "per below %u implies carrier > %lu Hz (max allowed)",
                 (unsigned int)TABLE_STEP_MIN_PER, (unsigned long)PFM_MAX_CARRIER_FREQ_HZ);
        SendErr(inst, 10, msg);
        return;
    }

    for (uint8_t ch = 0U; ch < HRTIM_NUM_CHANNELS; ch++)
    {
        cmp[ch] = (uint16_t)vals[1 + ch];
    }

    if (PFM_AppendStep((uint16_t)vals[0], cmp) == 0U)
    {
        SendErr(inst, 3, "Table full");
        return;
    }

    uart_send(inst, "OK\r\n");
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
/* True if EITHER fault source is latched: PC10/HRTIM1_FLT6 (native
   hardware, hrtim.h) or the GateDriverStatus EXTI interrupt
   (gate_driver.h, PE0..PE11). One combined answer for FAULT?/FIRE's
   ERR 6 gate/FAULT:CLEAR -- see cmd_fault_query()'s own comment for
   why these two structurally-independent mechanisms present as one
   fault state at the command layer. */
static uint8_t AnyFaultLatched(void)
{
    return (HRTIM1_FaultIsTripped() != 0U) || (GateDriver_FaultIsLatched() != 0U);
}

void cmd_fire(uart_instance_t *inst, char *args)
{
    (void)args;

    if (AnyFaultLatched() != 0U)
    {
        SendErr(inst, 6, "Fault latched -- send FAULT:CLEAR first");
        return;
    }

    if (PFM_GetEntryCount() == 0U)
    {
        SendErr(inst, 5, "Table is empty -- upload one first (TABLE:BEGIN/STEP/END)");
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
   g_pfminDataBuf (commands.c) -- up to PFM_INPUT_MAX_PERIODS entries,
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

/* --------------------------------------------------------------------------
 * CONFig:CHANnels?
 *
 * Reports HRTIM_NUM_CHANNELS (ctrlr_config.h) -- added alongside the
 * 2026-09-08 channel-count generalization specifically so
 * python/pfm_table_upload.py (or any other host tool) can confirm what
 * a given board was actually built for, rather than assuming a value
 * that could silently drift from reality after a rebuild with a
 * different channel count -- a TABLE:STEP wire-format mismatch that
 * would otherwise only surface as a confusing ERR 4 partway through an
 * upload.
 * -------------------------------------------------------------------------- */
void cmd_config_channels(uart_instance_t *inst, char *args)
{
    char buf[32];
    (void)args;

    snprintf(buf, sizeof(buf), "OK %u\r\n", (unsigned int)HRTIM_NUM_CHANNELS);
    uart_send(inst, buf);
}

/* --------------------------------------------------------------------------
 * FAULT? / FAULT:CLEar
 *
 * Status/clear for BOTH of this project's fault sources, combined into
 * one operator-facing fault state (AnyFaultLatched(), above):
 *   - PC10/HRTIM1_FLT6 -- native HRTIM hardware fault input (hrtim.h).
 *     Autonomous in silicon; already fully in effect by the time either
 *     of these is ever called.
 *   - GateDriverStatus_01..12 (PE0..PE11) -- software/EXTI-driven fault
 *     interrupt (gate_driver.h), added 2026-09-08. Needs the EXTI ISR
 *     to actually run (GateDriver_CheckFault()), unlike the PC10 path.
 * These two mechanisms stay structurally independent underneath (two
 * separate latches, two separate detection paths) -- combined only
 * here, at the command layer, because from an operator's perspective
 * "is there a fault, and can I FIRE" is one question with one answer,
 * not two. FAULT:CLEAR clears both latches unconditionally (clearing
 * one that was never set is a harmless no-op); FAULT? reports 1 if
 * either is latched.
 * -------------------------------------------------------------------------- */
void cmd_fault_query(uart_instance_t *inst, char *args)
{
    (void)args;

    uart_send(inst, (AnyFaultLatched() != 0U) ? "OK 1\r\n" : "OK 0\r\n");
}

void cmd_fault_clear(uart_instance_t *inst, char *args)
{
    (void)args;

    HRTIM1_FaultClear();
    GateDriver_FaultClear();

    uart_send(inst, "OK\r\n");
}

/* --------------------------------------------------------------------------
 * GDS?
 *
 * Raw HIGH/LOW snapshot of all 12 GateDriverStatus pins (PE0..PE11,
 * gate_driver.h) -- added 2026-09-08 as a diagnostic while chasing why
 * a fault wasn't being registered. Deliberately a single OK line (one
 * "NN=HIGH" or "NN=LOW" token per pin, NN = 01..12 matching the
 * GateDriverStatus_01..12 silkscreen/schematic numbering, space
 * separated, PE0 first) rather than the sibling PFM-STM32G474
 * project's multi-line/bitmask GDS? formats -- matches this project's
 * existing single-"OK <value>"-line convention (commands.h) instead of
 * introducing a new multi-line reply shape for just this one command.
 * Raw and uncached like the sibling project's GDS?: reads GPIOE->IDR
 * live at the moment of the query, no debounce, no polarity
 * interpretation, no fault-latching of its own -- this command itself
 * is read-only visibility and cannot stop PWM output by itself. (These
 * same 12 pins DO now gate PWM output, via a separate path: the
 * GateDriverStatus EXTI interrupt, gate_driver.h's
 * GateDriver_CheckFault(), added 2026-09-08 -- GDS? is unaffected by
 * and independent of that mechanism, just a raw snapshot either way.)
 * -------------------------------------------------------------------------- */
void cmd_gds_query(uart_instance_t *inst, char *args)
{
    uint16_t mask = GateDriver_Read();
    char buf[160];
    int  len = 0;
    (void)args;

    len += snprintf(&buf[len], sizeof(buf) - (size_t)len, "OK");
    for (uint8_t pin = 0U; pin < 12U; pin++)
    {
        int high = ((mask >> pin) & 1U) != 0U;
        len += snprintf(&buf[len], sizeof(buf) - (size_t)len,
                         " %02u=%s", (unsigned int)(pin + 1U), high ? "HIGH" : "LOW");
    }
    snprintf(&buf[len], sizeof(buf) - (size_t)len, "\r\n");

    uart_send(inst, buf);
}

#if (QSPI_TEST_FEATURE_ENABLED != 0)
/* --------------------------------------------------------------------------
 * QSPI:ID?
 *
 * QUADSPI connectivity test against the W25Q128JVS wired to
 * PE12-PE15/PB10-PB11 (docs/pin_mapping_v4.csv) -- issues the flash's
 * standard JEDEC Read ID instruction (0x9F, plain 1-line mode, no Quad
 * Enable required) via QspiTest_ReadId() (qspi_test.h) and reports the
 * 3 raw bytes (Manufacturer, Memory Type, Capacity) as space-separated
 * uppercase hex, e.g. "OK EF 40 18" for a healthy Winbond part --
 * compare against the W25Q128JVS datasheet's own JEDEC ID table rather
 * than trusting any specific expected value hardcoded here (none is;
 * this command reports what the chip actually says, nothing assumed).
 * ERR 7 on any HAL_QSPI command/receive failure or timeout (e.g. no
 * chip present, a wiring fault, or the bus wedged) -- see
 * QspiTest_ReadId()'s own doc comment for exactly what that call does
 * and does not verify.
 * -------------------------------------------------------------------------- */
void cmd_qspi_id(uart_instance_t *inst, char *args)
{
    uint8_t id[3];
    char buf[32];
    (void)args;

    if (QspiTest_ReadId(id) == 0U)
    {
        SendErr(inst, 7, "QUADSPI command failed or timed out");
        return;
    }

    snprintf(buf, sizeof(buf), "OK %02X %02X %02X\r\n",
             (unsigned int)id[0], (unsigned int)id[1], (unsigned int)id[2]);
    uart_send(inst, buf);
}
#endif /* QSPI_TEST_FEATURE_ENABLED */

#if (PFM_INPUT_FEATURE_ENABLED != 0)
/* --------------------------------------------------------------------------
 * PFMIN:CAPTURE, PFMIN:STATus?, PFMIN:DATA?
 *
 * See pfm_input.h and commands.h's own header comment above these
 * declarations for the shot-synchronized design -- PFMIN:CAPTURE only
 * arms (PfmInput_Arm()); the actual capture starts inside
 * PFM_Restart() (pfm.c), at the next FIRE, not here.
 * -------------------------------------------------------------------------- */
void cmd_pfmin_capture(uart_instance_t *inst, char *args)
{
    char *tok;
    long  m;

    tok = (args != NULL) ? strtok(args, " \r\n") : NULL;
    if (tok == NULL)
    {
        SendErr(inst, 9, "PFMIN:CAPTURE needs one argument: M (1-PFM_INPUT_MAX_PERIODS)");
        return;
    }

    m = atol(tok);
    if ((m < 1L) || (m > (long)PFM_INPUT_MAX_PERIODS))
    {
        SendErr(inst, 9, "M out of range (1-PFM_INPUT_MAX_PERIODS)");
        return;
    }

    PfmInput_Arm((uint16_t)m);

    uart_send(inst, "OK\r\n");
}

void cmd_pfmin_status(uart_instance_t *inst, char *args)
{
    char buf[64];
    int  len = 0;
    (void)args;

    len += snprintf(&buf[len], sizeof(buf) - (size_t)len, "OK");
    for (uint8_t ch = 0U; ch < PFM_INPUT_NUM_CHANNELS; ch++)
    {
        len += snprintf(&buf[len], sizeof(buf) - (size_t)len,
                         " %u", (unsigned int)PfmInput_GetCount(ch));
    }
    snprintf(&buf[len], sizeof(buf) - (size_t)len, "\r\n");

    uart_send(inst, buf);
}

/* TEMPORARY debug command, 2026-09-09 -- see pfm_input.h's own comment
   on PfmInput_GetDmaStartStatus(). Remove once DMA capture is
   confirmed reliable. */
void cmd_pfmin_dmastat(uart_instance_t *inst, char *args)
{
    char buf[64];
    int  len = 0;
    (void)args;

    len += snprintf(&buf[len], sizeof(buf) - (size_t)len, "OK");
    for (uint8_t ch = 0U; ch < PFM_INPUT_NUM_CHANNELS; ch++)
    {
        len += snprintf(&buf[len], sizeof(buf) - (size_t)len,
                         " %u", (unsigned int)PfmInput_GetDmaStartStatus(ch));
    }
    snprintf(&buf[len], sizeof(buf) - (size_t)len, "\r\n");

    uart_send(inst, buf);
}

/* Sized for the worst case: PFM_INPUT_MAX_PERIODS entries, each up to
   " 4294967295" (11 chars), plus the "OK <count> OVERCAP=<n>" prefix
   and CRLF -- comfortably over 200*11+32 with room to spare. static,
   not a stack local: this project's linker script reserves only
   _Min_Stack_Size (1 KB) for the stack (STM32G474QETX_FLASH.ld) --
   putting several KB on the stack here would be a real, needless risk
   even though the actual runtime stack has far more room in practice
   (see AGENTS.md's own flagged-but-deferred note on stack sizing). */
static char g_pfminDataBuf[2600];

void cmd_pfmin_data(uart_instance_t *inst, char *args)
{
    char    *tok;
    long     chArg;
    uint8_t  ch;
    uint16_t count;
    const uint32_t *periods;
    int      len = 0;

    tok = (args != NULL) ? strtok(args, " \r\n") : NULL;
    if (tok == NULL)
    {
        SendErr(inst, 8, "PFMIN:DATA? needs one argument: channel (1-6)");
        return;
    }

    chArg = atol(tok);
    if ((chArg < 1L) || (chArg > (long)PFM_INPUT_NUM_CHANNELS))
    {
        SendErr(inst, 8, "Invalid PFM_Input channel (1-6)");
        return;
    }
    ch = (uint8_t)(chArg - 1L);   /* 1-6 on the wire -> 0-5 internally */

    count   = PfmInput_GetCount(ch);
    periods = PfmInput_GetPeriods(ch);

    /* OVERCAP=<n>: a genuine data-quality indicator (see
       PfmInput_GetOvercaptureCount()'s own doc comment in
       pfm_input.c) -- nonzero means a second rising edge arrived
       before this module's ISR could read the previous one, so some
       entries below may not be trustworthy. Expected to read 0 in
       normal operation; not yet documented in
       docs/command_reference.md. */
    len += snprintf(&g_pfminDataBuf[len], sizeof(g_pfminDataBuf) - (size_t)len,
                     "OK %u OVERCAP=%u", (unsigned int)count,
                     (unsigned int)PfmInput_GetOvercaptureCount(ch));

    for (uint16_t i = 0U; i < count; i++)
    {
        len += snprintf(&g_pfminDataBuf[len], sizeof(g_pfminDataBuf) - (size_t)len,
                         " %lu", (unsigned long)periods[i]);
    }
    snprintf(&g_pfminDataBuf[len], sizeof(g_pfminDataBuf) - (size_t)len, "\r\n");

    uart_send(inst, g_pfminDataBuf);
}
#endif /* PFM_INPUT_FEATURE_ENABLED */

/* --------------------------------------------------------------------------
 * PID:* -- closed-loop control, see pid.h for the full architecture.
 * Channel numbering matches PFMIN:DATA?'s own convention: 1..N on the
 * wire, 0..N-1 internally (N = HRTIM_NUM_CHANNELS, CONFig:CHANnels?
 * reports it). Error codes 11 (invalid channel) and 12 (invalid
 * argument count/value) -- see commands.h.
 * -------------------------------------------------------------------------- */

void cmd_pid_start(uart_instance_t *inst, char *args)
{
    (void)args;
    (void)PID_Start();
    uart_send(inst, "OK\r\n");
}

void cmd_pid_stop(uart_instance_t *inst, char *args)
{
    (void)args;
    PID_Stop();
    uart_send(inst, "OK\r\n");
}

void cmd_pid_setpoint(uart_instance_t *inst, char *args)
{
    char *tok;
    long  chArg;
    long  hzArg;
    uint8_t ch;

    tok = (args != NULL) ? strtok(args, " \r\n") : NULL;
    if (tok == NULL)
    {
        SendErr(inst, 12, "PID:SETPOINT needs two arguments: channel hz");
        return;
    }
    chArg = atol(tok);

    tok = strtok(NULL, " \r\n");
    if (tok == NULL)
    {
        SendErr(inst, 12, "PID:SETPOINT needs two arguments: channel hz");
        return;
    }
    hzArg = atol(tok);

    if ((chArg < 1L) || (chArg > (long)HRTIM_NUM_CHANNELS))
    {
        SendErr(inst, 11, "Invalid PID channel");
        return;
    }
    ch = (uint8_t)(chArg - 1L);   /* 1..N on the wire -> 0..N-1 internally */

    if (hzArg < 0L)
    {
        SendErr(inst, 12, "hz must be >= 0");
        return;
    }

    /* PID_SetSetpoint() clamps into [PID_OUTPUT_MIN_HZ,
       PID_OUTPUT_MAX_HZ] itself -- an out-of-range value here is
       accepted, not rejected, matching this project's existing
       clamp-don't-reject convention for HRTIM1_ClampCompare(). */
    (void)PID_SetSetpoint(ch, (uint32_t)hzArg);
    uart_send(inst, "OK\r\n");
}

void cmd_pid_gains(uart_instance_t *inst, char *args)
{
    char *tok;
    long  chArg;
    double kp;
    double ki;
    double kd;
    uint8_t ch;

    tok = (args != NULL) ? strtok(args, " \r\n") : NULL;
    if (tok == NULL)
    {
        SendErr(inst, 12, "PID:GAINS needs four arguments: channel kp ki kd");
        return;
    }
    chArg = atol(tok);

    tok = strtok(NULL, " \r\n");
    if (tok == NULL)
    {
        SendErr(inst, 12, "PID:GAINS needs four arguments: channel kp ki kd");
        return;
    }
    kp = atof(tok);

    tok = strtok(NULL, " \r\n");
    if (tok == NULL)
    {
        SendErr(inst, 12, "PID:GAINS needs four arguments: channel kp ki kd");
        return;
    }
    ki = atof(tok);

    tok = strtok(NULL, " \r\n");
    if (tok == NULL)
    {
        SendErr(inst, 12, "PID:GAINS needs four arguments: channel kp ki kd");
        return;
    }
    kd = atof(tok);

    if ((chArg < 1L) || (chArg > (long)HRTIM_NUM_CHANNELS))
    {
        SendErr(inst, 11, "Invalid PID channel");
        return;
    }
    ch = (uint8_t)(chArg - 1L);

    (void)PID_SetGains(ch, (float)kp, (float)ki, (float)kd);
    uart_send(inst, "OK\r\n");
}

void cmd_pid_status(uart_instance_t *inst, char *args)
{
    char buf[96];
    char *tok;
    long  chArg;
    uint8_t ch;
    uint32_t setpointHz;
    uint32_t measuredHz;
    uint32_t outputHz;

    tok = (args != NULL) ? strtok(args, " \r\n") : NULL;
    if (tok == NULL)
    {
        SendErr(inst, 12, "PID:STATus? needs one argument: channel");
        return;
    }
    chArg = atol(tok);

    if ((chArg < 1L) || (chArg > (long)HRTIM_NUM_CHANNELS))
    {
        SendErr(inst, 11, "Invalid PID channel");
        return;
    }
    ch = (uint8_t)(chArg - 1L);

    (void)PID_GetStatus(ch, &setpointHz, &measuredHz, &outputHz);

    snprintf(buf, sizeof(buf), "OK %u %lu %lu %lu\r\n",
             (unsigned int)PID_IsRunning(),
             (unsigned long)setpointHz, (unsigned long)measuredHz,
             (unsigned long)outputHz);
    uart_send(inst, buf);
}
