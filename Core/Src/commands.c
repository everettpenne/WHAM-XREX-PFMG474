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
#include "state_machine.h"
#include "git_version.h"
#include "main.h"
#include "xrex_io.h"
#include "sim_transrex.h"
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
    char buf[96];
    (void)args;

    /* Board + firmware identity in one line, matching the sibling
       PFM-STM32G474 project's *IDN convention (OK <value>, space-
       separated fields) -- see ctrlr_config.h for HW_BOARD_NAME/
       HW_BOARD_REV/FW_VERSION_STRING.

       4th field, FW_GIT_COMMIT, added 2026-09-11 per direct request --
       exactly which git commit THIS firmware was actually built from,
       so a host can always tell what's really running on a board, not
       just what's supposed to be flashed. See python/gen_git_version.py
       (generates Core/Inc/git_version.h, gitignored -- see that file's
       own header comment) and python/wham_build.py (runs it
       automatically before every build; this is the canonical way to
       build this project now). A `-dirty` suffix (FW_GIT_DIRTY) flags
       a build made with uncommitted changes -- the commit hash alone
       would otherwise silently overstate how precisely this build
       matches that commit in git history. "unknown" if git_version.h
       was never generated (git unavailable, or built some other way
       entirely) -- gen_git_version.py always writes a valid, buildable
       header either way, never blocks compiling over this. */
    snprintf(buf, sizeof(buf), "OK %s %s %s %s%s\r\n",
             HW_BOARD_NAME, HW_BOARD_REV, FW_VERSION_STRING,
             FW_GIT_COMMIT, (FW_GIT_DIRTY != 0U) ? "-dirty" : "");
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

    /* SM_ClearFault() (state_machine.h, added 2026-09-13) now owns
       actually calling HRTIM1_FaultClear()/GateDriver_FaultClear() and
       re-checking both sources -- see its own doc comment. Reply stays
       unconditional "OK" either way, matching this command's existing,
       already-documented convention (a still-present condition
       re-latches immediately, before this even returns -- an operator
       checks FAULT?/STATE? afterward to see the real result, same as
       before this change). */
    (void)SM_ClearFault();

    uart_send(inst, "OK\r\n");
}

/* --------------------------------------------------------------------------
 * ARM / DISARM / STATE? -- the top-level operating-state machine
 * (state_machine.h), added 2026-09-13. See that header for the full
 * design (IDLE/ARMED/FIRING/FAULT). Bare, top-level commands (no
 * `PID:`/other namespace prefix) -- system-wide state, not specific to
 * any one subsystem, matching this project's existing bare `FIRE`
 * (pfm.c's legacy table-based output path, unrelated). Error code 13
 * (new): an invalid state-machine transition for the current state.
 * -------------------------------------------------------------------------- */

/* IDLE -> ARMED. See SM_Arm()'s own doc comment for exactly what
   "the appropriate conditions" currently checks (a stub, always
   allows arming today). */
void cmd_arm(uart_instance_t *inst, char *args)
{
    (void)args;

    if (SM_Arm() == 0U)
    {
        SendErr(inst, 13, "Can't ARM -- not currently IDLE, or arm conditions not met");
        return;
    }
    uart_send(inst, "OK\r\n");
}

/* ARMED -> IDLE, without firing -- stand down. No-op (still replies
   OK) if not currently ARMED, matching SM_Disarm()'s own convention
   and this project's general "idempotent, no error for a harmless
   no-op" style (e.g. FAULT:CLEAR clearing an already-clear latch). */
void cmd_disarm(uart_instance_t *inst, char *args)
{
    (void)args;

    SM_Disarm();
    uart_send(inst, "OK\r\n");
}

/* OK <IDLE|ARMED|FIRING|FAULT>, OK FAULT GENERAL, or OK FAULT OVERCURRENT
   <ch> (1-based, this project's usual wire convention -- see
   SM_GetFaultChannel()'s own doc comment, state_machine.h) when in
   FAULT. The trailing <ch> token is new 2026-09-15, alongside
   SM_ReportOcpFault() -- only present for OVERCURRENT, since General
   Fault is system-wide and has no single channel to report. */
void cmd_state_query(uart_instance_t *inst, char *args)
{
    char buf[32];
    const char *name;
    (void)args;

    switch (SM_GetState())
    {
        case SM_STATE_IDLE:   name = "IDLE";   break;
        case SM_STATE_ARMED:  name = "ARMED";  break;
        case SM_STATE_FIRING: name = "FIRING"; break;
        case SM_STATE_FAULT:  name = "FAULT";  break;
        default:              name = "UNKNOWN"; break;
    }

    if (SM_GetState() == SM_STATE_FAULT)
    {
        if (SM_GetFaultType() == SM_FAULT_OVERCURRENT)
        {
            snprintf(buf, sizeof(buf), "OK %s OVERCURRENT %u\r\n",
                      name, (unsigned)(SM_GetFaultChannel() + 1U));
        }
        else if (SM_GetFaultType() == SM_FAULT_EXTERNAL_ENABLE)
        {
            /* Added 2026-09-16 -- distinct from GENERAL purely for
               operator diagnostics (identical ramp-down response
               either way, see state_machine.h's own external-enable
               section). */
            snprintf(buf, sizeof(buf), "OK %s EXTERNAL_ENABLE\r\n", name);
        }
        else if (SM_GetFaultType() == SM_FAULT_EMERGENCY_STOP)
        {
            /* Added 2026-09-17 -- distinct from GENERAL for operator
               diagnostics (an immediate hard cutoff, NOT the graceful
               ramp-down every other fault type here uses -- see
               state_machine.h's own emergency-stop section). */
            snprintf(buf, sizeof(buf), "OK %s EMERGENCY_STOP\r\n", name);
        }
        else if (SM_GetFaultType() == SM_FAULT_ENABLE_OUTPUT)
        {
            /* Added 2026-09-17 -- PER-CHANNEL, like OVERCURRENT (whose
               exact response it reuses) -- distinct from GENERAL/
               EXTERNAL_ENABLE purely for operator diagnostics. Do not
               confuse with EXTERNAL_ENABLE: that's a single, system-wide
               INPUT interlock (PF13); this is a per-channel check of
               this firmware's OWN commanded ENA_OUT/CONTACT_OUT output
               state -- see state_machine.h's own SM_FAULT_ENABLE_OUTPUT
               header comment. */
            snprintf(buf, sizeof(buf), "OK %s ENABLE_OUTPUT %u\r\n",
                      name, (unsigned)(SM_GetFaultChannel() + 1U));
        }
        else if (SM_GetFaultType() == SM_FAULT_ENERPRO)
        {
            /* Added 2026-09-18 -- PER-CHANNEL, like OVERCURRENT/
               ENABLE_OUTPUT (whose exact response it reuses). Enerpro
               was RECLASSIFIED out of GENERAL this same day -- see
               state_machine.h's own SM_FAULT_ENERPRO header comment for
               the full reasoning (Transrex_Controls_Upgrade doc's fault
               table). */
            snprintf(buf, sizeof(buf), "OK %s ENERPRO %u\r\n",
                      name, (unsigned)(SM_GetFaultChannel() + 1U));
        }
        else
        {
            snprintf(buf, sizeof(buf), "OK %s GENERAL\r\n", name);
        }
    }
    else
    {
        snprintf(buf, sizeof(buf), "OK %s\r\n", name);
    }
    uart_send(inst, buf);
}

/* --------------------------------------------------------------------------
 * EXTernal:ENAble <0|1> / EXTernal:ENAble? / EXTernal:INPut?
 *
 * Added 2026-09-16, per direct request. Backing pin MOVED 2026-09-17
 * from PF15 to PF13, per direct instruction: enable and trigger are now
 * independent physical signals on separate pins, not one shared wire --
 * see EXTernal:TRIGger below and state_machine.h's own external-enable
 * section (SM_SetExternalEnableRequired() and friends) for the full
 * design. This is just the wire-command wrapper around that. Own
 * top-level `EXTernal:` namespace (not nested under `PID:` or any other
 * existing prefix) -- system-wide config, not specific to any one
 * subsystem, same reasoning as `ARM`/`DISARM` above being bare; two
 * ':'-levels here (rather than one compound word) purely so
 * "ENAble"/"INPut" each get their own independent short-form
 * abbreviation -- see cmd_parser.c's own comment on this command's
 * table entry for why. */
void cmd_ext_enable(uart_instance_t *inst, char *args)
{
    char *tok;
    long  val;

    tok = (args != NULL) ? strtok(args, " \r\n") : NULL;
    if (tok == NULL)
    {
        SendErr(inst, 12, "EXTernal:ENAble needs one argument: 0|1");
        return;
    }
    val = atol(tok);

    SM_SetExternalEnableRequired((val != 0L) ? 1U : 0U);
    uart_send(inst, "OK\r\n");
}

void cmd_ext_enable_query(uart_instance_t *inst, char *args)
{
    char buf[16];
    (void)args;

    snprintf(buf, sizeof(buf), "OK %u\r\n", (unsigned)SM_GetExternalEnableRequired());
    uart_send(inst, buf);
}

/* Raw PF13 logic level, independent of whether the interlock is even
   turned on -- lets an operator confirm real wiring/signal presence
   before relying on it, same diagnostic role PFMIN:DEBUG:RAW?/REG?
   played for the PFM_Input fiber-patching investigation
   (docs/changelog.txt, 2026-09-15). MOVED 2026-09-17 from PF15 to
   PF13, same change as cmd_ext_enable() above. */
void cmd_ext_enable_input_query(uart_instance_t *inst, char *args)
{
    char buf[16];
    (void)args;

    snprintf(buf, sizeof(buf), "OK %u\r\n", (unsigned)SM_GetExternalEnableInputRaw());
    uart_send(inst, buf);
}

/* --------------------------------------------------------------------------
 * EXTernal:TRIGger <0|1> / EXTernal:TRIGger? / EXTernal:TRIGger:INPut?
 *
 * Added 2026-09-16, per direct follow-up request. RESTRUCTURED
 * 2026-09-17: this pin (PF15, "Fiber_Enable") now backs TRIGGER ONLY --
 * the external-enable interlock moved to its own pin (PF13, above).
 * See state_machine.h's own external-trigger design comment
 * (SM_SetExternalTriggerRequired() and friends) for the full design --
 * this is just the wire-command wrapper. No longer structurally
 * coupled to EXTernal:ENAble (that dependency existed only because both
 * features read the same wire, back when this was PF15-for-both) --
 * SM_SetExternalTriggerRequired() always succeeds now, so the ERR 16
 * refusal path below is gone. ERR 16 itself is retired, not reassigned
 * -- this project's convention is error codes are never renumbered or
 * reused once assigned (see docs/command_reference.md). */
void cmd_ext_trigger(uart_instance_t *inst, char *args)
{
    char *tok;
    long  val;

    tok = (args != NULL) ? strtok(args, " \r\n") : NULL;
    if (tok == NULL)
    {
        SendErr(inst, 12, "EXTernal:TRIGger needs one argument: 0|1");
        return;
    }
    val = atol(tok);

    (void)SM_SetExternalTriggerRequired((val != 0L) ? 1U : 0U);
    uart_send(inst, "OK\r\n");
}

void cmd_ext_trigger_query(uart_instance_t *inst, char *args)
{
    char buf[16];
    (void)args;

    snprintf(buf, sizeof(buf), "OK %u\r\n", (unsigned)SM_GetExternalTriggerRequired());
    uart_send(inst, buf);
}

/* Raw PF15 logic level, independent of whether the trigger feature is
   even turned on -- added 2026-09-17 alongside the PF13/PF15 split;
   previously EXTernal:INPut? covered this same pin (it backed both
   enable and trigger, being the same wire) -- now that they're
   separate, this is trigger's own dedicated diagnostic, matching
   cmd_ext_enable_input_query()'s role for PF13. */
void cmd_ext_trigger_input_query(uart_instance_t *inst, char *args)
{
    char buf[16];
    (void)args;

    snprintf(buf, sizeof(buf), "OK %u\r\n", (unsigned)SM_GetExternalTriggerInputRaw());
    uart_send(inst, buf);
}

/* --------------------------------------------------------------------------
 * EMERGency:ENAble <0|1> / EMERGency:ENAble? / EMERGency:INPut?
 *
 * Added 2026-09-17, per direct request. See state_machine.h's own
 * emergency-stop design comment (SM_SetEmergencyStopRequired() and
 * friends) for the full design -- this is just the wire-command
 * wrapper. PG10, NOT the same pin as EXTernal:ENAble above (PF13) --
 * a completely separate fiber-optic input. Own top-level `EMERGency:`
 * namespace, same reasoning as `EXTernal:` -- system-wide config, not
 * nested under `PID:` or any other existing prefix. */
void cmd_emerg_enable(uart_instance_t *inst, char *args)
{
    char *tok;
    long  val;

    tok = (args != NULL) ? strtok(args, " \r\n") : NULL;
    if (tok == NULL)
    {
        SendErr(inst, 12, "EMERGency:ENAble needs one argument: 0|1");
        return;
    }
    val = atol(tok);

    SM_SetEmergencyStopRequired((val != 0L) ? 1U : 0U);
    uart_send(inst, "OK\r\n");
}

void cmd_emerg_enable_query(uart_instance_t *inst, char *args)
{
    char buf[16];
    (void)args;

    snprintf(buf, sizeof(buf), "OK %u\r\n", (unsigned)SM_GetEmergencyStopRequired());
    uart_send(inst, buf);
}

/* Raw PG10 logic level, independent of whether the feature is even
   turned on -- lets an operator confirm real wiring/signal presence
   before relying on it, same diagnostic role EXTernal:INPut? plays for
   PF13. */
void cmd_emerg_input_query(uart_instance_t *inst, char *args)
{
    char buf[16];
    (void)args;

    snprintf(buf, sizeof(buf), "OK %u\r\n", (unsigned)SM_GetEmergencyStopInputRaw());
    uart_send(inst, buf);
}

/* --------------------------------------------------------------------------
 * DIAGnostic:GPOut12 <0|1> / DIAGnostic:GPOut12?
 *
 * Added 2026-09-16, per direct request: a generic, software-driven
 * diagnostic output on PD1 ("GPOut_12" in the V4 column,
 * docs/pin_mapping_v4.csv -- confirmed GPO there; PF13 was proposed
 * first and corrected -- it's actually "GPInput_12", an input, in that
 * same CSV). GPIO config lives in main.c's MX_GPIO_Init(), matching
 * this project's established precedent (gate_driver.c's GateDriverStatus
 * pins, PF15 above) of plain GPIO config in main.c and the read/write
 * logic in the module that actually uses it -- there's no dedicated
 * module for this one, it's a two-line direct HAL_GPIO_WritePin()/
 * ReadPin() pair, not enough behavior to justify one.
 *
 * Immediate use, 2026-09-16: physically loop this pin to PF15
 * (Fiber_Enable) so the external-enable/external-trigger feature above
 * could be driven entirely from the serial console -- precise,
 * repeatable timing on exactly when PF15 goes HIGH/LOW, instead of a
 * hand-operated bench jumper/switch. (2026-09-17 UPDATE: PF15 now
 * backs external-TRIGGER only -- enable moved to its own pin, PF13 --
 * so this same loop now drives trigger specifically; nothing about
 * this command itself changed.) Nothing about this command is PF15-
 * specific, though -- it's a plain level output, reusable for any future
 * diagnostic
 * that needs one. */
void cmd_diag_gpout12(uart_instance_t *inst, char *args)
{
    char *tok;
    long  val;

    tok = (args != NULL) ? strtok(args, " \r\n") : NULL;
    if (tok == NULL)
    {
        SendErr(inst, 12, "DIAGnostic:GPOut12 needs one argument: 0|1");
        return;
    }
    val = atol(tok);

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, (val != 0L) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    uart_send(inst, "OK\r\n");
}

void cmd_diag_gpout12_query(uart_instance_t *inst, char *args)
{
    char buf[16];
    (void)args;

    snprintf(buf, sizeof(buf), "OK %u\r\n",
             (unsigned)(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_1) == GPIO_PIN_SET ? 1U : 0U));
    uart_send(inst, buf);
}

/* --------------------------------------------------------------------------
 * DIAGnostic:GPOut11 <0|1> / DIAGnostic:GPOut11?
 *
 * Added 2026-09-17 -- a SECOND, independent diagnostic output, on PD0
 * ("GPOut_11" in the V4 column, docs/pin_mapping_v4.csv). Direct
 * correction: PD1/DIAGnostic:GPOut12 above was initially reused for
 * testing the PG10 emergency-stop feature too, but PD1 is already the
 * pin dedicated to driving PF15 (external-trigger as of later the same
 * day -- external-enable at the time this was written) -- the user
 * caught this and asked for a genuinely separate pin for PG10 testing,
 * to remove any ambiguity about which diagnostic signal drives which real
 * input. Otherwise an exact mirror of cmd_diag_gpout12()/
 * cmd_diag_gpout12_query() above -- see that pair's own doc comment
 * for the shared reasoning (generic level output, GPIO config in
 * main.c, no dedicated module). */
void cmd_diag_gpout11(uart_instance_t *inst, char *args)
{
    char *tok;
    long  val;

    tok = (args != NULL) ? strtok(args, " \r\n") : NULL;
    if (tok == NULL)
    {
        SendErr(inst, 12, "DIAGnostic:GPOut11 needs one argument: 0|1");
        return;
    }
    val = atol(tok);

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, (val != 0L) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    uart_send(inst, "OK\r\n");
}

void cmd_diag_gpout11_query(uart_instance_t *inst, char *args)
{
    char buf[16];
    (void)args;

    snprintf(buf, sizeof(buf), "OK %u\r\n",
             (unsigned)(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_0) == GPIO_PIN_SET ? 1U : 0U));
    uart_send(inst, buf);
}

/* --------------------------------------------------------------------------
 * DIAGnostic:GPOut09 <0|1> / DIAGnostic:GPOut09?
 * DIAGnostic:GPOut10 <0|1> / DIAGnostic:GPOut10?
 *
 * Added 2026-09-18 -- a THIRD and FOURTH diagnostic output, on PG8
 * ("GPOut_09" in the V4 column, docs/pin_mapping_v4.csv) and PG9
 * ("GPOut_10"). Verified directly against the CSV before writing this:
 * the V3 column for these same two rows says "No connection", and a
 * DIFFERENT pair of pins (PD8/PD9) carried the "GPOut_09"/"GPOut_10"
 * names under V3 -- the same class of V3/V4 name-reuse trap as the
 * earlier PC14-vs-PF15 and PF13-vs-PD1 corrections, so this one was
 * checked against the CSV rather than assumed.
 *
 * Immediate use: closes the last gap in the Transrex simulator's fiber-
 * transmitter budget (docs/pin_mapping_reference.tex Section 7) -- these
 * two feed XR1_OCP/XR2_OCP on the controller (GPInput_03/PF4 and
 * GPInput_07/PF8 respectively), completing OCP fault-injection coverage
 * for all 4 channels. Otherwise an exact mirror of
 * cmd_diag_gpout12()/cmd_diag_gpout11() above -- see that pair's own
 * doc comment for the shared reasoning (generic level output, GPIO
 * config in main.c, no dedicated module). */
void cmd_diag_gpout09(uart_instance_t *inst, char *args)
{
    char *tok;
    long  val;

    tok = (args != NULL) ? strtok(args, " \r\n") : NULL;
    if (tok == NULL)
    {
        SendErr(inst, 12, "DIAGnostic:GPOut09 needs one argument: 0|1");
        return;
    }
    val = atol(tok);

    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_8, (val != 0L) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    uart_send(inst, "OK\r\n");
}

void cmd_diag_gpout09_query(uart_instance_t *inst, char *args)
{
    char buf[16];
    (void)args;

    snprintf(buf, sizeof(buf), "OK %u\r\n",
             (unsigned)(HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_8) == GPIO_PIN_SET ? 1U : 0U));
    uart_send(inst, buf);
}

void cmd_diag_gpout10(uart_instance_t *inst, char *args)
{
    char *tok;
    long  val;

    tok = (args != NULL) ? strtok(args, " \r\n") : NULL;
    if (tok == NULL)
    {
        SendErr(inst, 12, "DIAGnostic:GPOut10 needs one argument: 0|1");
        return;
    }
    val = atol(tok);

    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, (val != 0L) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    uart_send(inst, "OK\r\n");
}

void cmd_diag_gpout10_query(uart_instance_t *inst, char *args)
{
    char buf[16];
    (void)args;

    snprintf(buf, sizeof(buf), "OK %u\r\n",
             (unsigned)(HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_9) == GPIO_PIN_SET ? 1U : 0U));
    uart_send(inst, buf);
}

/* --------------------------------------------------------------------------
 * DIAGnostic:OPTBytes?
 *
 * TEMPORARY diagnostic, added 2026-09-17 -- direct request: is PB8
 * (this board's BOOT0 net, docs/pin_mapping_v4.csv) actually available
 * as a plain GPIO after boot, or does it stay committed to boot-mode
 * sampling? Reads the LIVE FLASH_OPTR register (FLASH->OPTR, loaded
 * from the option bytes at reset -- stm32g474xx.h's own bit
 * definitions, not guessed) rather than relying on memory of what the
 * G4 family "usually" does. The three bits that actually decide this,
 * per the CMSIS header:
 *   nBOOT0     (bit 27) -- the option-byte-supplied boot0 VALUE, used
 *              only when nSWBOOT0 (below) is 0.
 *   nSWBOOT0   (bit 26) -- 1 = boot0 is read from the physical pin
 *              (BOOT0/PB8, whichever this package/remap uses); 0 =
 *              boot0 is taken ENTIRELY from the nBOOT0 option-byte
 *              value above, and the physical pin is NOT sampled at
 *              all for boot purposes -- meaning if nSWBOOT0=0, PB8 is
 *              free for GPIO use regardless of any BOOT0/PB8 remap
 *              question, since nothing ever reads it at boot either
 *              way.
 *   nBOOT1     (bit 23) -- combines with the effective boot0 value
 *              (whichever source) to select the final boot target
 *              (main flash / system memory / SRAM).
 * Does NOT itself answer whether PB8 is safe to repurpose -- that
 * still depends on nSWBOOT0's value once read back for real, and
 * separately, on what PG10 (the other pin asked about, this board's
 * own "NRST"-labeled net) is actually wired to on the schematic, which
 * no register on this chip can reveal -- only the schematic can.
 * Remove once the PB8/PG10 GPIO-reuse question is settled. */
void cmd_diag_optbytes_query(uart_instance_t *inst, char *args)
{
    char buf[96];
    (void)args;

    uint32_t optr = FLASH->OPTR;

    snprintf(buf, sizeof(buf), "OK OPTR=%08lX nBOOT0=%u nSWBOOT0=%u nBOOT1=%u\r\n",
             (unsigned long)optr,
             (unsigned)((optr & FLASH_OPTR_nBOOT0) != 0U),
             (unsigned)((optr & FLASH_OPTR_nSWBOOT0) != 0U),
             (unsigned)((optr & FLASH_OPTR_nBOOT1) != 0U));
    uart_send(inst, buf);
}

/* --------------------------------------------------------------------------
 * DIAGnostic:RSTCause? / DIAGnostic:RSTCause:CLEar
 *
 * TEMPORARY diagnostic, added 2026-09-17 -- direct request: while
 * testing the new PD0->fiber TX->fiber->inverting RX->PG10 loop for the
 * emergency-stop feature, DIAGnostic:GPOut11 1 (PD0 HIGH) consistently
 * returned a single garbled 0x00 byte instead of "OK\r\n", and the pin
 * never actually reached HIGH on a follow-up query -- while the
 * unrelated DIAGnostic:GPOut12 (PD1, no fiber transmitter behind it)
 * kept working perfectly the whole time. User asked whether this could
 * be (a) some PG10-specific NRST-adjacent silicon behavior, or (b)
 * power-rail droop from the fiber TRANSMITTER's own LED current
 * actually resetting the MCU -- rather than guess, this reads the
 * REAL RCC->CSR reset-cause flags (stm32g474xx.h's own bit
 * definitions -- RCC_CSR_BORRSTF/PINRSTF/SFTRSTF/IWDGRSTF/WWDGRSTF/
 * LPWRRSTF/OBLRSTF, not guessed), which is the one thing that can
 * actually distinguish "a real reset happened, and here's why" from
 * "no reset at all, something else garbled the UART byte." Nothing
 * else in this codebase clears these flags (confirmed by grep before
 * adding this), so whatever the reset cause was persists until
 * explicitly cleared here -- no special early-boot capture needed.
 *
 * On pure silicon-fact grounds (separately answered, not guessed): NRST
 * is ALWAYS a dedicated, non-GPIO pin on every STM32 (confirmed via
 * this project's own device header -- RCC_CSR_PINRSTF is the ONLY flag
 * tied to that pin, and no flag here is tied to any GPIO port at all),
 * so PG10 cannot itself BE or trigger the chip's own NRST function at
 * the silicon level. If PINRSTF is what shows up after reproducing the
 * glitch, that would mean something is asserting the REAL NRST net
 * (a board-level path, not a PG10-is-secretly-NRST one); if BORRSTF
 * shows up instead, that directly confirms the power-rail-droop
 * hypothesis. Remove once the GPOut11/PG10 diagnostic investigation is
 * settled. */
void cmd_diag_rstcause_query(uart_instance_t *inst, char *args)
{
    char buf[160];
    int  len = 0;
    (void)args;

    uint32_t csr = RCC->CSR;

    len += snprintf(&buf[len], sizeof(buf) - (size_t)len, "OK CSR=%08lX", (unsigned long)csr);
    len += snprintf(&buf[len], sizeof(buf) - (size_t)len, " BOR=%u",   (unsigned)((csr & RCC_CSR_BORRSTF)   != 0U));
    len += snprintf(&buf[len], sizeof(buf) - (size_t)len, " PIN=%u",   (unsigned)((csr & RCC_CSR_PINRSTF)   != 0U));
    len += snprintf(&buf[len], sizeof(buf) - (size_t)len, " SFT=%u",   (unsigned)((csr & RCC_CSR_SFTRSTF)   != 0U));
    len += snprintf(&buf[len], sizeof(buf) - (size_t)len, " IWDG=%u",  (unsigned)((csr & RCC_CSR_IWDGRSTF)  != 0U));
    len += snprintf(&buf[len], sizeof(buf) - (size_t)len, " WWDG=%u",  (unsigned)((csr & RCC_CSR_WWDGRSTF)  != 0U));
    len += snprintf(&buf[len], sizeof(buf) - (size_t)len, " LPWR=%u",  (unsigned)((csr & RCC_CSR_LPWRRSTF)  != 0U));
    len += snprintf(&buf[len], sizeof(buf) - (size_t)len, " OBL=%u",   (unsigned)((csr & RCC_CSR_OBLRSTF)   != 0U));
    snprintf(&buf[len], sizeof(buf) - (size_t)len, "\r\n");

    uart_send(inst, buf);
}

void cmd_diag_rstcause_clear(uart_instance_t *inst, char *args)
{
    (void)args;
    RCC->CSR |= RCC_CSR_RMVF;   /* self-clearing bit -- resets every *RSTF
                                    flag above to 0, per the reference
                                    manual */
    uart_send(inst, "OK\r\n");
}

/* --------------------------------------------------------------------------
 * OCP:TEST:FAULT <ch>
 *
 * TEMPORARY debug/verification command, added 2026-09-15 -- same
 * removability precedent as PFMIN:DMASTAT? (pfm_input.h's own comment)
 * and qspi_test.c: exists to let real per-channel OCP behavior
 * (SM_ReportOcpFault(), state_machine.h -- immediate disable for `ch`,
 * an immediate proportional step-down for the other enabled channels,
 * then the same graceful ramp General Fault uses, see that function's
 * own doc comment) be exercised end-to-end on REAL hardware, since there
 * is no real per-channel OCP pin wired up anywhere in this codebase yet
 * (see state_machine.h's own "NOTE TO REVISIT" -- the mapping is still
 * to be defined). Software-only fault injection -- calls
 * SM_ReportOcpFault(ch - 1) directly, exactly as a real OCP pin's own
 * (not-yet-written) EXTI handler eventually will. Not gated/dangerous in
 * the operator-console sense (wham_console.py/wham_llm_console.py's
 * _is_dangerous()) -- triggering a fault only ever STOPS/reduces output,
 * never starts new output, the same "safe direction, never gated"
 * reasoning FAULT:CLEAR/PID:STOP already get.
 *
 * Remove once real OCP hardware detection exists and has its own real
 * trigger path -- keeping a software fault-injection command around
 * even after that point could still be useful for bench verification
 * without needing to actually force a real overcurrent condition, but
 * that's a decision for whoever wires the real pins, not assumed here. */
void cmd_ocp_test_fault(uart_instance_t *inst, char *args)
{
    char *tok;
    long  chArg;

    tok = (args != NULL) ? strtok(args, " \r\n") : NULL;
    if (tok == NULL)
    {
        SendErr(inst, 12, "OCP:TEST:FAULT needs one argument: channel");
        return;
    }
    chArg = atol(tok);

    if ((chArg < 1L) || (chArg > (long)HRTIM_NUM_CHANNELS))
    {
        SendErr(inst, 11, "Invalid PID channel");
        return;
    }

    SM_ReportOcpFault((uint8_t)(chArg - 1L));
    uart_send(inst, "OK\r\n");
}

/* --------------------------------------------------------------------------
 * GENERAL:TEST:FAULT
 *
 * TEMPORARY debug/verification command, added 2026-09-15 -- same
 * removability precedent as OCP:TEST:FAULT above (and PFMIN:DMASTAT?/
 * qspi_test.c before it): lets General Fault's ramp-down (see
 * state_machine.h's own header comment and PID_BeginFaultRampDown(),
 * pid.h) be exercised end-to-end on REAL hardware at a moment of the
 * operator's own choosing -- e.g. EARLY in a shot, mid-ramp-up, rather
 * than only the steady-state/flat-top case OCP:TEST:FAULT has been
 * exercised against so far -- without needing to actually trip a real
 * PC10/HRTIM1_FLT6 or GateDriverStatus condition. No channel argument:
 * General Fault is system-wide, unlike OCP. Software-only fault
 * injection -- calls SM_ReportGeneralFault() directly, exactly as
 * SM_PollFaults() itself does the instant it polls a real tripped
 * source. Same "safe direction, never gated" reasoning as
 * OCP:TEST:FAULT/FAULT:CLEAR/PID:STOP -- triggering a fault only ever
 * stops/reduces output.
 *
 * Remove once a real reason to keep it around after all fault paths are
 * otherwise well-exercised stops applying -- same "not assumed here"
 * deferral as OCP:TEST:FAULT's own comment. */
void cmd_general_test_fault(uart_instance_t *inst, char *args)
{
    (void)args;   /* no arguments -- General Fault is system-wide */
    SM_ReportGeneralFault();
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

/* --------------------------------------------------------------------------
 * XREX:CHANnel:STATus? <ch>
 *
 * Added 2026-09-17, per direct request: an XR-labeled diagnostic
 * alongside the existing GDS? (raw GateDriverStatus_01..12 readback,
 * above) -- reports one Transrex channel's own Water/Temp/Enerpro/OCP
 * pins together, by name, rather than needing to remember which of the
 * 16 underlying physical pins corresponds to which signal. Raw levels
 * (HIGH/LOW), polarity-agnostic -- same convention GDS?/EXTernal:INPut?/
 * EMERGency:INPut? already use; XR_WATER_FLT_POLARITY/etc.
 * (ctrlr_config.h) are what determine which raw level actually means
 * "faulted," not this command. See xrex_io.h's own header comment for
 * the full XR1..XR4 pin-naming design. */
void cmd_xrex_channel_status(uart_instance_t *inst, char *args)
{
    char *tok;
    long  chArg;
    char  buf[80];

    tok = (args != NULL) ? strtok(args, " \r\n") : NULL;
    if (tok == NULL)
    {
        SendErr(inst, 12, "XREX:CHANnel:STATus? needs one argument: channel");
        return;
    }
    chArg = atol(tok);

    if ((chArg < 1L) || (chArg > (long)HRTIM_NUM_CHANNELS))
    {
        SendErr(inst, 11, "Invalid PID channel");
        return;
    }

    uint8_t water = 0U, tmp = 0U, enerpro = 0U, ocp = 0U;
    (void)XrexIo_GetChannelStatus((uint8_t)(chArg - 1L), &water, &tmp, &enerpro, &ocp);

    snprintf(buf, sizeof(buf), "OK WATER=%s TMP=%s ENERPRO=%s OCP=%s\r\n",
             water ? "HIGH" : "LOW", tmp ? "HIGH" : "LOW",
             enerpro ? "HIGH" : "LOW", ocp ? "HIGH" : "LOW");
    uart_send(inst, buf);
}

/* --------------------------------------------------------------------------
 * XREX:CHANnel:ENAOut <ch> <0|1> / XREX:CHANnel:ENAOut? <ch>
 * XREX:CHANnel:CONTactOut <ch> <0|1> / XREX:CHANnel:CONTactOut? <ch>
 *
 * Added 2026-09-17, per direct request: "Enable and contactor fiber
 * outputs need to be set by a serial command, one for each supply."
 * Real per-channel outputs (PG0-PG3/PG4-PG7, docs/pin_mapping_v4.csv's
 * "XREX Pin Name" column) this firmware itself drives -- owned (pin
 * table, GPIO read/write) by xrex_io.c, matching that module's
 * established per-channel-XR-signal charter. See state_machine.h's own
 * SM_FAULT_ENABLE_OUTPUT/enable-output sections for what reads these:
 * `ARM` refuses unless every currently-enabled channel's own
 * ENA_OUT+CONTACT_OUT are BOTH already HIGH ("The controller cannot be
 * armed unless these are outputting prior to the arm signal"), and this
 * is continuously re-checked once ARMED -- asked directly and
 * confirmed, not assumed.
 *
 * 1-based channel argument, matching this project's universal wire
 * convention -- reuses ERR 11 (invalid channel)/ERR 12 (missing/invalid
 * argument), same as every other numbered-channel command; no new
 * error codes needed for this whole feature. Takes effect immediately;
 * an operator may set/clear either output at any time, including
 * mid-shot -- the continuous poll (XrexIo_PollEnableOutputFaults(),
 * xrex_io.c) is what reacts if that turns out to matter for a
 * currently-enabled channel. */
void cmd_xrex_ena_out(uart_instance_t *inst, char *args)
{
    char *tok;
    long  chArg;
    long  val;

    tok = (args != NULL) ? strtok(args, " \r\n") : NULL;
    if (tok == NULL)
    {
        SendErr(inst, 12, "XREX:CHANnel:ENAOut needs two arguments: channel, 0|1");
        return;
    }
    chArg = atol(tok);
    if ((chArg < 1L) || (chArg > (long)HRTIM_NUM_CHANNELS))
    {
        SendErr(inst, 11, "Invalid PID channel");
        return;
    }

    tok = strtok(NULL, " \r\n");
    if (tok == NULL)
    {
        SendErr(inst, 12, "XREX:CHANnel:ENAOut needs two arguments: channel, 0|1");
        return;
    }
    val = atol(tok);

    XrexIo_SetEnableOutput((uint8_t)(chArg - 1L), (val != 0L) ? 1U : 0U);
    uart_send(inst, "OK\r\n");
}

void cmd_xrex_ena_out_query(uart_instance_t *inst, char *args)
{
    char *tok;
    long  chArg;
    char  buf[16];

    tok = (args != NULL) ? strtok(args, " \r\n") : NULL;
    if (tok == NULL)
    {
        SendErr(inst, 12, "XREX:CHANnel:ENAOut? needs one argument: channel");
        return;
    }
    chArg = atol(tok);
    if ((chArg < 1L) || (chArg > (long)HRTIM_NUM_CHANNELS))
    {
        SendErr(inst, 11, "Invalid PID channel");
        return;
    }

    snprintf(buf, sizeof(buf), "OK %u\r\n", (unsigned)XrexIo_GetEnableOutput((uint8_t)(chArg - 1L)));
    uart_send(inst, buf);
}

void cmd_xrex_contact_out(uart_instance_t *inst, char *args)
{
    char *tok;
    long  chArg;
    long  val;

    tok = (args != NULL) ? strtok(args, " \r\n") : NULL;
    if (tok == NULL)
    {
        SendErr(inst, 12, "XREX:CHANnel:CONTactOut needs two arguments: channel, 0|1");
        return;
    }
    chArg = atol(tok);
    if ((chArg < 1L) || (chArg > (long)HRTIM_NUM_CHANNELS))
    {
        SendErr(inst, 11, "Invalid PID channel");
        return;
    }

    tok = strtok(NULL, " \r\n");
    if (tok == NULL)
    {
        SendErr(inst, 12, "XREX:CHANnel:CONTactOut needs two arguments: channel, 0|1");
        return;
    }
    val = atol(tok);

    XrexIo_SetContactorOutput((uint8_t)(chArg - 1L), (val != 0L) ? 1U : 0U);
    uart_send(inst, "OK\r\n");
}

void cmd_xrex_contact_out_query(uart_instance_t *inst, char *args)
{
    char *tok;
    long  chArg;
    char  buf[16];

    tok = (args != NULL) ? strtok(args, " \r\n") : NULL;
    if (tok == NULL)
    {
        SendErr(inst, 12, "XREX:CHANnel:CONTactOut? needs one argument: channel");
        return;
    }
    chArg = atol(tok);
    if ((chArg < 1L) || (chArg > (long)HRTIM_NUM_CHANNELS))
    {
        SendErr(inst, 11, "Invalid PID channel");
        return;
    }

    snprintf(buf, sizeof(buf), "OK %u\r\n", (unsigned)XrexIo_GetContactorOutput((uint8_t)(chArg - 1L)));
    uart_send(inst, buf);
}

/* --------------------------------------------------------------------------
 * GPOut:ENAble <0|1> / GPOut:ENAble?
 *
 * Added 2026-09-17, per direct request: PC13 ("GPOut_Enable_Pin" in
 * the V4 column, docs/pin_mapping_v4.csv -- confirmed GPO there),
 * default HIGH at boot (main.c's MX_GPIO_Init()), with a serial command
 * to set it either level. A plain, direct HAL_GPIO_WritePin()/
 * ReadPin() pair -- no dedicated module, matching the same "not enough
 * behavior to justify one" precedent DIAGnostic:GPOut11/GPOut12 already
 * established -- own top-level `GPOut:` namespace (not nested under
 * `DIAGnostic:`) since this is a real, specifically-named board signal
 * from the schematic, not a generic scratch diagnostic pin. */
void cmd_gpout_enable(uart_instance_t *inst, char *args)
{
    char *tok;
    long  val;

    tok = (args != NULL) ? strtok(args, " \r\n") : NULL;
    if (tok == NULL)
    {
        SendErr(inst, 12, "GPOut:ENAble needs one argument: 0|1");
        return;
    }
    val = atol(tok);

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, (val != 0L) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    uart_send(inst, "OK\r\n");
}

void cmd_gpout_enable_query(uart_instance_t *inst, char *args)
{
    char buf[16];
    (void)args;

    snprintf(buf, sizeof(buf), "OK %u\r\n",
             (unsigned)(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET ? 1U : 0U));
    uart_send(inst, buf);
}

/* --------------------------------------------------------------------------
 * PWMAlt:ENAble <0|1> / PWMAlt:ENAble?
 *
 * Added 2026-09-17, per direct request: PC15 ("PWM_Alt_Enable" in the
 * V4 column, docs/pin_mapping_v4.csv -- confirmed GPO there), default
 * HIGH at boot, with a serial command to set it either level -- exact
 * mirror of GPOut:ENAble above in every respect, see that pair's own
 * doc comment for the shared reasoning. */
void cmd_pwmalt_enable(uart_instance_t *inst, char *args)
{
    char *tok;
    long  val;

    tok = (args != NULL) ? strtok(args, " \r\n") : NULL;
    if (tok == NULL)
    {
        SendErr(inst, 12, "PWMAlt:ENAble needs one argument: 0|1");
        return;
    }
    val = atol(tok);

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, (val != 0L) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    uart_send(inst, "OK\r\n");
}

void cmd_pwmalt_enable_query(uart_instance_t *inst, char *args)
{
    char buf[16];
    (void)args;

    snprintf(buf, sizeof(buf), "OK %u\r\n",
             (unsigned)(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15) == GPIO_PIN_SET ? 1U : 0U));
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

/* --------------------------------------------------------------------------
 * PFMIN:DEBUG:RAW? <ch>
 *
 * TEMPORARY debug command, added 2026-09-15 -- same removability
 * precedent as PFMIN:DMASTAT? just above: diagnosing why PID:STATus?'s
 * measuredHz reads persistently 0 for WHAM channels 2/3/4 while
 * channel 1 works, confirmed on real hardware (each channel isolated
 * alone, driving confirmed-correct real HRTIM output over a full
 * shot). Non-destructive read of PfmInput_GetDebugRaw() (pfm_input.h)
 * -- unlike PID:STATus?'s own PfmInput_ConsumeAveragePeriod() call,
 * does NOT reset the accumulator, so repeated polling can watch
 * avgCount accumulate (or not) live during a shot without disturbing
 * the real control loop's own consumption. `ch` is 1-based (WHAM
 * channel numbering, matching PID:STATus?), internally
 * channel = ch - 1 (PFM_Input_01..04, the first 4 of the 6 physical
 * PFM_Input channels -- see pid.c's own per-channel loop). Remove once
 * the root cause is found and fixed. */
void cmd_pfmin_debug_raw(uart_instance_t *inst, char *args)
{
    char *tok;
    long  chArg;

    tok = (args != NULL) ? strtok(args, " \r\n") : NULL;
    if (tok == NULL)
    {
        SendErr(inst, 12, "PFMIN:DEBUG:RAW? needs one argument: channel");
        return;
    }
    chArg = atol(tok);

    if ((chArg < 1L) || (chArg > (long)PFM_INPUT_NUM_CHANNELS))
    {
        SendErr(inst, 8, "Invalid PFM_Input channel (1-6)");
        return;
    }

    uint8_t  continuous = 0U, running = 0U, haveFirstRise = 0U;
    uint16_t avgCount = 0U, overcaptureCount = 0U;
    uint32_t lastPeriod = 0U;
    (void)PfmInput_GetDebugRaw((uint8_t)(chArg - 1L), &continuous, &running,
                                &haveFirstRise, &avgCount, &lastPeriod, &overcaptureCount);

    char buf[96];
    snprintf(buf, sizeof(buf), "OK cont=%u run=%u firstRise=%u avgCount=%u lastPeriod=%lu overcap=%u\r\n",
             (unsigned int)continuous, (unsigned int)running, (unsigned int)haveFirstRise,
             (unsigned int)avgCount, (unsigned long)lastPeriod, (unsigned int)overcaptureCount);
    uart_send(inst, buf);
}

/* --------------------------------------------------------------------------
 * PFMIN:DEBUG:REG? <ch>
 *
 * TEMPORARY debug command, added 2026-09-15 -- see
 * PfmInput_GetDebugRegs()'s own doc comment (pfm_input.h) for the full
 * reasoning: raw TIMx register readback, bypassing all software
 * layers, while diagnosing why measuredHz reads 0 for WHAM channels
 * 2/3 despite confirmed-correct real output AND confirmed-correct
 * physical wiring. Poll twice in a row during an active shot and
 * compare CNT/CCR -- if they're not moving, the hardware itself isn't
 * seeing this channel's signal; if they ARE moving but software never
 * reports it, the bug is downstream in the DMA/ISR path instead.
 * Remove once the root cause is found and fixed. */
void cmd_pfmin_debug_reg(uart_instance_t *inst, char *args)
{
    char *tok;
    long  chArg;

    tok = (args != NULL) ? strtok(args, " \r\n") : NULL;
    if (tok == NULL)
    {
        SendErr(inst, 12, "PFMIN:DEBUG:REG? needs one argument: channel");
        return;
    }
    chArg = atol(tok);

    if ((chArg < 1L) || (chArg > (long)PFM_INPUT_NUM_CHANNELS))
    {
        SendErr(inst, 8, "Invalid PFM_Input channel (1-6)");
        return;
    }

    uint32_t cr1 = 0U, ccer = 0U, dier = 0U, sr = 0U, cnt = 0U, ccrChannel = 0U;
    (void)PfmInput_GetDebugRegs((uint8_t)(chArg - 1L), &cr1, &ccer, &dier, &sr, &cnt, &ccrChannel);
    uint32_t ccmr1 = PfmInput_GetDebugCcmr1((uint8_t)(chArg - 1L));
    uint32_t moder = 0U, afr = 0U, idr = 0U;
    (void)PfmInput_GetDebugGpio((uint8_t)(chArg - 1L), &moder, &afr, &idr);

    char buf[176];
    snprintf(buf, sizeof(buf),
             "OK CR1=%08lX CCER=%08lX DIER=%08lX SR=%08lX CNT=%08lX CCR=%08lX "
             "CCMR1=%08lX MODER=%lu AFR=%lu IDR=%lu\r\n",
             (unsigned long)cr1, (unsigned long)ccer, (unsigned long)dier,
             (unsigned long)sr, (unsigned long)cnt, (unsigned long)ccrChannel, (unsigned long)ccmr1,
             (unsigned long)moder, (unsigned long)afr, (unsigned long)idr);
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
    /* SM_Stop() (state_machine.h, added 2026-09-13) -- a manual abort,
       ARMED/FIRING -> IDLE. No-op if already IDLE; deliberately does
       NOT clear FAULT (see its own doc comment -- FAULT:CLEAR is the
       only way out of a real fault latch). */
    SM_Stop();
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

/* Added 2026-09-10, alongside python/wham_console.py -- PID:GAINS was
   write-only until now, forcing any host tool to remember what it
   itself last sent rather than being able to ask the device (see
   PID_GetGains()'s own doc comment in pid.h). */
void cmd_pid_gains_query(uart_instance_t *inst, char *args)
{
    char buf[64];
    char *tok;
    long  chArg;
    uint8_t ch;
    float kp;
    float ki;
    float kd;

    tok = (args != NULL) ? strtok(args, " \r\n") : NULL;
    if (tok == NULL)
    {
        SendErr(inst, 12, "PID:GAINS? needs one argument: channel");
        return;
    }
    chArg = atol(tok);

    if ((chArg < 1L) || (chArg > (long)HRTIM_NUM_CHANNELS))
    {
        SendErr(inst, 11, "Invalid PID channel");
        return;
    }
    ch = (uint8_t)(chArg - 1L);

    (void)PID_GetGains(ch, &kp, &ki, &kd);

    snprintf(buf, sizeof(buf), "OK %g %g %g\r\n", (double)kp, (double)ki, (double)kd);
    uart_send(inst, buf);
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

/* channel argument 0 means "log every channel at once" (PID_ArmLogAll(),
   added 2026-09-10) -- real channels are 1..N on the wire everywhere
   else in this project, leaving 0 a natural, otherwise-unused sentinel
   rather than a whole new command. See PID_ArmLogAll()'s own doc
   comment in pid.h for why this exists (a genuine simultaneous
   cross-channel comparison, not N separate single-channel runs). */
void cmd_pid_log(uart_instance_t *inst, char *args)
{
    char *tok;
    long  chArg;
    long  nArg;
    long  decimArg;

    tok = (args != NULL) ? strtok(args, " \r\n") : NULL;
    if (tok == NULL)
    {
        SendErr(inst, 12, "PID:LOG needs three arguments: channel(0=all) maxSamples decim");
        return;
    }
    chArg = atol(tok);

    tok = strtok(NULL, " \r\n");
    if (tok == NULL)
    {
        SendErr(inst, 12, "PID:LOG needs three arguments: channel(0=all) maxSamples decim");
        return;
    }
    nArg = atol(tok);

    tok = strtok(NULL, " \r\n");
    if (tok == NULL)
    {
        SendErr(inst, 12, "PID:LOG needs three arguments: channel(0=all) maxSamples decim");
        return;
    }
    decimArg = atol(tok);

    if ((chArg < 0L) || (chArg > (long)HRTIM_NUM_CHANNELS))
    {
        SendErr(inst, 11, "Invalid PID channel (0 = all channels)");
        return;
    }

    if ((nArg < 1L) || (nArg > (long)PID_LOG_MAX_SAMPLES) || (decimArg < 1L))
    {
        SendErr(inst, 12, "maxSamples must be 1-PID_LOG_MAX_SAMPLES, decim >= 1");
        return;
    }

    if (chArg == 0L)
    {
        (void)PID_ArmLogAll((uint16_t)nArg, (uint16_t)decimArg);
    }
    else
    {
        (void)PID_ArmLog((uint8_t)(chArg - 1L), (uint16_t)nArg, (uint16_t)decimArg);
    }
    uart_send(inst, "OK\r\n");
}

/* STREAMED, not built into one giant buffer -- with 3 values/sample
   (setpointHz, measuredHz, outputHz, added 2026-09-10 alongside
   PID_StartRamp()) at up to PID_LOG_MAX_SAMPLES (1000), a single
   contiguous reply buffer would need ~40 KB (worst case, all 10-digit
   values) -- real RAM this project doesn't have to spare (this MCU
   has 128 KiB total, most of it already spoken for by g_pfmTable[]
   and friends). uart_send() is just a blocking HAL_UART_Transmit() (see
   uart.c) with no minimum-call-size requirement, so there's no
   correctness reason to batch into one large string either -- this
   sends the header, then one small chunk per sample, reusing one
   small stack buffer regardless of how many samples are logged. Costs
   more individual transmit calls (~1000 for a full log) instead of
   one, but at 115200 baud the whole reply takes over a second to
   clock out either way -- the call overhead is noise against that.

   OPTIONAL channel argument, added 2026-09-10 alongside PID_ArmLogAll():
   `PID:LOGDATA?` (no argument) is UNCHANGED, exactly its original
   behavior -- valid only when a single channel is armed (PID_ArmLog()),
   fetches that channel's data, ERR 12 if all-channels mode is active
   (ambiguous without a channel to pick). `PID:LOGDATA? <ch>` works in
   EITHER mode: under all-channels mode any ch is valid; under
   single-channel mode ch must equal the one actually armed (ERR 12
   otherwise -- no data exists for any other channel this run). */
void cmd_pid_logdata(uart_instance_t *inst, char *args)
{
    char *tok;
    long  chArg;
    uint8_t ch;
    uint16_t count;
    const uint32_t *setpoint;
    const uint32_t *measured;
    const uint32_t *output;
    char chunk[48];

    tok = (args != NULL) ? strtok(args, " \r\n") : NULL;
    if (tok == NULL)
    {
        if (PID_IsLogAllChannels() != 0U)
        {
            SendErr(inst, 12, "PID:LOGDATA? needs a channel argument while "
                               "all-channels logging is armed");
            return;
        }
        ch = PID_GetLogChannel();   /* 0xFF (no channel armed) is caught below */
    }
    else
    {
        chArg = atol(tok);
        if ((chArg < 1L) || (chArg > (long)HRTIM_NUM_CHANNELS))
        {
            SendErr(inst, 11, "Invalid PID channel");
            return;
        }
        ch = (uint8_t)(chArg - 1L);
        if ((PID_IsLogAllChannels() == 0U) && (ch != PID_GetLogChannel()))
        {
            SendErr(inst, 12, "That channel isn't the one currently armed -- "
                               "see PID:LOG");
            return;
        }
    }

    setpoint = PID_GetLogSetpoint(ch);
    measured = PID_GetLogMeasured(ch);
    output   = PID_GetLogOutput(ch);
    if ((setpoint == NULL) || (measured == NULL) || (output == NULL))
    {
        SendErr(inst, 11, "Invalid PID channel");
        return;
    }

    count = PID_GetLogCount();
    snprintf(chunk, sizeof(chunk), "OK %u %lu", (unsigned int)count,
             (unsigned long)PID_GetLogSampleRateHz());
    uart_send(inst, chunk);

    for (uint16_t i = 0U; i < count; i++)
    {
        snprintf(chunk, sizeof(chunk), " %lu %lu %lu",
                 (unsigned long)setpoint[i], (unsigned long)measured[i],
                 (unsigned long)output[i]);
        uart_send(inst, chunk);
    }

    uart_send(inst, "\r\n");
}

void cmd_pid_ramp(uart_instance_t *inst, char *args)
{
    char *tok;
    long  chArg;
    long  startArg;
    long  endArg;
    long  durationArg;
    uint8_t ch;

    tok = (args != NULL) ? strtok(args, " \r\n") : NULL;
    if (tok == NULL)
    {
        SendErr(inst, 12, "PID:RAMP needs four arguments: channel startHz endHz durationMs");
        return;
    }
    chArg = atol(tok);

    tok = strtok(NULL, " \r\n");
    if (tok == NULL)
    {
        SendErr(inst, 12, "PID:RAMP needs four arguments: channel startHz endHz durationMs");
        return;
    }
    startArg = atol(tok);

    tok = strtok(NULL, " \r\n");
    if (tok == NULL)
    {
        SendErr(inst, 12, "PID:RAMP needs four arguments: channel startHz endHz durationMs");
        return;
    }
    endArg = atol(tok);

    tok = strtok(NULL, " \r\n");
    if (tok == NULL)
    {
        SendErr(inst, 12, "PID:RAMP needs four arguments: channel startHz endHz durationMs");
        return;
    }
    durationArg = atol(tok);

    if ((chArg < 1L) || (chArg > (long)HRTIM_NUM_CHANNELS))
    {
        SendErr(inst, 11, "Invalid PID channel");
        return;
    }
    ch = (uint8_t)(chArg - 1L);

    if ((startArg < 0L) || (endArg < 0L) || (durationArg < 1L))
    {
        SendErr(inst, 12, "startHz/endHz must be >= 0, durationMs >= 1");
        return;
    }

    /* PID_StartRamp() clamps startHz/endHz into [PID_OUTPUT_MIN_HZ,
       PID_OUTPUT_MAX_HZ] itself -- matches PID:SETPOINT's own
       clamp-don't-reject convention. */
    (void)PID_StartRamp(ch, (uint32_t)startArg, (uint32_t)endArg, (uint32_t)durationArg);
    uart_send(inst, "OK\r\n");
}

/* --------------------------------------------------------------------------
 * Production shot profile + open/closed-loop mode -- added 2026-09-10.
 * See pid.h's "DEMAND PROFILE"/"OPEN-LOOP MODE" doc sections.
 * -------------------------------------------------------------------------- */

/* `ch` = 0 means "every channel at once" -- added 2026-09-16, per
   direct request for a system-wide loop-mode convenience (originally
   asked for in service of the external-trigger feature, but not
   restricted to that use -- a plain global setter). Matches
   PID:LOG's own existing "0 = all channels" convention (pid.h's own
   PfmInput_ArmLogAll() precedent) rather than inventing a new command
   -- reuses this exact command/argument slot instead. Real channels
   are still 1..HRTIM_NUM_CHANNELS as always. */
void cmd_pid_loopmode(uart_instance_t *inst, char *args)
{
    char *tok;
    long  chArg;
    long  modeArg;
    uint8_t ch;
    uint8_t mode;

    tok = (args != NULL) ? strtok(args, " \r\n") : NULL;
    if (tok == NULL)
    {
        SendErr(inst, 12, "PID:LOOPMODE needs two arguments: channel(0=all) 0|1");
        return;
    }
    chArg = atol(tok);

    tok = strtok(NULL, " \r\n");
    if (tok == NULL)
    {
        SendErr(inst, 12, "PID:LOOPMODE needs two arguments: channel(0=all) 0|1");
        return;
    }
    modeArg = atol(tok);

    if ((chArg < 0L) || (chArg > (long)HRTIM_NUM_CHANNELS))
    {
        SendErr(inst, 11, "Invalid PID channel");
        return;
    }
    mode = (modeArg != 0L) ? 1U : 0U;

    if (chArg == 0L)
    {
        for (ch = 0U; ch < HRTIM_NUM_CHANNELS; ch++)
        {
            (void)PID_SetLoopMode(ch, mode);
        }
    }
    else
    {
        ch = (uint8_t)(chArg - 1L);
        (void)PID_SetLoopMode(ch, mode);
    }
    uart_send(inst, "OK\r\n");
}

/* Added 2026-09-10 -- see cmd_pid_gains_query()'s own comment on why
   (PID:LOOPMODE was write-only until now). */
void cmd_pid_loopmode_query(uart_instance_t *inst, char *args)
{
    char buf[32];
    char *tok;
    long  chArg;
    uint8_t ch;

    tok = (args != NULL) ? strtok(args, " \r\n") : NULL;
    if (tok == NULL)
    {
        SendErr(inst, 12, "PID:LOOPMODE? needs one argument: channel");
        return;
    }
    chArg = atol(tok);

    if ((chArg < 1L) || (chArg > (long)HRTIM_NUM_CHANNELS))
    {
        SendErr(inst, 11, "Invalid PID channel");
        return;
    }
    ch = (uint8_t)(chArg - 1L);

    snprintf(buf, sizeof(buf), "OK %u\r\n", (unsigned int)PID_GetLoopMode(ch));
    uart_send(inst, buf);
}

/* Added 2026-09-11, per direct request -- see PID_SetChannelEnable()'s
   own doc comment in pid.h for exactly what this does (a genuine "no
   PFM waveform at all" switch, distinct from PID:LOOPMODE). Takes
   effect immediately if the loop is already running -- see that
   function's own comment. */
void cmd_pid_channel_enable(uart_instance_t *inst, char *args)
{
    char *tok;
    long  chArg;
    long  enableArg;
    uint8_t ch;

    tok = (args != NULL) ? strtok(args, " \r\n") : NULL;
    if (tok == NULL)
    {
        SendErr(inst, 12, "PID:CHANNEL:ENABLE needs two arguments: channel 0|1");
        return;
    }
    chArg = atol(tok);

    tok = strtok(NULL, " \r\n");
    if (tok == NULL)
    {
        SendErr(inst, 12, "PID:CHANNEL:ENABLE needs two arguments: channel 0|1");
        return;
    }
    enableArg = atol(tok);

    if ((chArg < 1L) || (chArg > (long)HRTIM_NUM_CHANNELS))
    {
        SendErr(inst, 11, "Invalid PID channel");
        return;
    }
    ch = (uint8_t)(chArg - 1L);

    (void)PID_SetChannelEnable(ch, (enableArg != 0L) ? 1U : 0U);
    uart_send(inst, "OK\r\n");
}

void cmd_pid_channel_enable_query(uart_instance_t *inst, char *args)
{
    char buf[32];
    char *tok;
    long  chArg;
    uint8_t ch;

    tok = (args != NULL) ? strtok(args, " \r\n") : NULL;
    if (tok == NULL)
    {
        SendErr(inst, 12, "PID:CHANNEL:ENABLE? needs one argument: channel");
        return;
    }
    chArg = atol(tok);

    if ((chArg < 1L) || (chArg > (long)HRTIM_NUM_CHANNELS))
    {
        SendErr(inst, 11, "Invalid PID channel");
        return;
    }
    ch = (uint8_t)(chArg - 1L);

    snprintf(buf, sizeof(buf), "OK %u\r\n", (unsigned int)PID_GetChannelEnable(ch));
    uart_send(inst, buf);
}

void cmd_pid_channel_nickname(uart_instance_t *inst, char *args)
{
    char *tok;
    long  chArg;
    uint8_t ch;

    tok = (args != NULL) ? strtok(args, " \r\n") : NULL;
    if (tok == NULL)
    {
        SendErr(inst, 12, "PID:CHANNEL:NICKNAME needs two arguments: channel name");
        return;
    }
    chArg = atol(tok);

    tok = strtok(NULL, " \r\n");
    if (tok == NULL)
    {
        SendErr(inst, 12, "PID:CHANNEL:NICKNAME needs two arguments: channel name");
        return;
    }

    if ((chArg < 1L) || (chArg > (long)HRTIM_NUM_CHANNELS))
    {
        SendErr(inst, 11, "Invalid PID channel");
        return;
    }
    ch = (uint8_t)(chArg - 1L);

    if (PID_SetChannelNickname(ch, tok) == 0U)
    {
        SendErr(inst, 14, "Invalid nickname -- 1-PID_CHANNEL_NICKNAME_MAX_LEN chars, "
                          "no spaces, and not the reserved value '-'");
        return;
    }
    uart_send(inst, "OK\r\n");
}

void cmd_pid_channel_nickname_query(uart_instance_t *inst, char *args)
{
    char buf[24U + PID_CHANNEL_NICKNAME_MAX_LEN];
    char *tok;
    long  chArg;
    uint8_t ch;
    const char *name;

    tok = (args != NULL) ? strtok(args, " \r\n") : NULL;
    if (tok == NULL)
    {
        SendErr(inst, 12, "PID:CHANNEL:NICKNAME? needs one argument: channel");
        return;
    }
    chArg = atol(tok);

    if ((chArg < 1L) || (chArg > (long)HRTIM_NUM_CHANNELS))
    {
        SendErr(inst, 11, "Invalid PID channel");
        return;
    }
    ch = (uint8_t)(chArg - 1L);

    /* "-" is the wire sentinel for "no nickname set" -- see
       PID_SetChannelNickname()'s own doc comment in pid.h for why: an
       empty second token ("OK \r\n") would be ambiguous to parse on the
       client side, so this always emits exactly two space-separated
       tokens. */
    name = PID_GetChannelNickname(ch);
    if (name[0] == '\0')
    {
        name = "-";
    }

    snprintf(buf, sizeof(buf), "OK %s\r\n", name);
    uart_send(inst, buf);
}

void cmd_pid_profile_timing(uart_instance_t *inst, char *args)
{
    char *tok;
    double rampTimeS;
    double flatTopTimeS;

    tok = (args != NULL) ? strtok(args, " \r\n") : NULL;
    if (tok == NULL)
    {
        SendErr(inst, 12, "PID:PROFILE:TIMING needs two arguments: rampTimeS flatTopTimeS");
        return;
    }
    rampTimeS = atof(tok);

    tok = strtok(NULL, " \r\n");
    if (tok == NULL)
    {
        SendErr(inst, 12, "PID:PROFILE:TIMING needs two arguments: rampTimeS flatTopTimeS");
        return;
    }
    flatTopTimeS = atof(tok);

    if ((rampTimeS <= 0.0) || (flatTopTimeS <= 0.0))
    {
        SendErr(inst, 12, "rampTimeS/flatTopTimeS must be > 0");
        return;
    }

    /* Operator-facing unit is seconds (5-15s ramp, 1-15s flat-top
       nominal, per the real shot profile) -- PID_SetProfileTiming()'s
       own unit is milliseconds, matching PID:RAMP's durationMs. */
    if (PID_SetProfileTiming((uint32_t)(rampTimeS * 1000.0), (uint32_t)(flatTopTimeS * 1000.0)) == 0U)
    {
        SendErr(inst, 12, "rampTimeS/flatTopTimeS too small");
        return;
    }
    uart_send(inst, "OK\r\n");
}

/* Added 2026-09-10 -- see cmd_pid_gains_query()'s own comment on why
   (PID:PROFILE:TIMING was write-only until now). ERR 12 (not just an
   empty/zero OK reply) if timing was never successfully set -- a real,
   meaningful "not configured yet" state (PID_ProfileStart() itself
   refuses to run in it), worth a distinct reply rather than silently
   reporting 0 0 as if that were a real value. */
void cmd_pid_profile_timing_query(uart_instance_t *inst, char *args)
{
    char buf[48];
    uint32_t rampTimeMs;
    uint32_t flatTopTimeMs;
    (void)args;

    if (PID_GetProfileTiming(&rampTimeMs, &flatTopTimeMs) == 0U)
    {
        SendErr(inst, 12, "Profile timing not set -- send PID:PROFILE:TIMING first");
        return;
    }

    /* ms -> seconds, the operator-facing unit (matches the setter's
       own convention) -- %g rather than integer division so a
       sub-second value (e.g. 500 ms -> "0.5") round-trips cleanly. */
    snprintf(buf, sizeof(buf), "OK %g %g\r\n",
             (double)rampTimeMs / 1000.0, (double)flatTopTimeMs / 1000.0);
    uart_send(inst, buf);
}

void cmd_pid_profile_current(uart_instance_t *inst, char *args)
{
    char *tok;
    long  chArg;
    double currentA;
    uint8_t ch;

    tok = (args != NULL) ? strtok(args, " \r\n") : NULL;
    if (tok == NULL)
    {
        SendErr(inst, 12, "PID:PROFILE:CURRENT needs two arguments: channel demandCurrentA");
        return;
    }
    chArg = atol(tok);

    tok = strtok(NULL, " \r\n");
    if (tok == NULL)
    {
        SendErr(inst, 12, "PID:PROFILE:CURRENT needs two arguments: channel demandCurrentA");
        return;
    }
    currentA = atof(tok);

    if ((chArg < 1L) || (chArg > (long)HRTIM_NUM_CHANNELS))
    {
        SendErr(inst, 11, "Invalid PID channel");
        return;
    }
    ch = (uint8_t)(chArg - 1L);

    if (currentA < 0.0)
    {
        SendErr(inst, 12, "demandCurrentA must be >= 0");
        return;
    }

    /* PID_SetProfileCurrent() clamps into [0, this channel's own
       PFM_MAX_CURRENT_A_PER_CHANNEL entry] itself -- matches
       PID:SETPOINT's own clamp-don't-reject convention. */
    (void)PID_SetProfileCurrent(ch, (float)currentA);
    uart_send(inst, "OK\r\n");
}

/* Added 2026-09-10 -- see cmd_pid_gains_query()'s own comment on why
   (PID:PROFILE:CURRENT was write-only until now). Always succeeds for
   a valid channel (demandCurrentA defaults to 0.0f, a real value, not
   an "unset" sentinel -- unlike profile timing there's no distinct
   "never configured" state worth a separate ERR here). */
void cmd_pid_profile_current_query(uart_instance_t *inst, char *args)
{
    char buf[32];
    char *tok;
    long  chArg;
    uint8_t ch;
    float demandCurrentA;

    tok = (args != NULL) ? strtok(args, " \r\n") : NULL;
    if (tok == NULL)
    {
        SendErr(inst, 12, "PID:PROFILE:CURRENT? needs one argument: channel");
        return;
    }
    chArg = atol(tok);

    if ((chArg < 1L) || (chArg > (long)HRTIM_NUM_CHANNELS))
    {
        SendErr(inst, 11, "Invalid PID channel");
        return;
    }
    ch = (uint8_t)(chArg - 1L);

    (void)PID_GetProfileCurrent(ch, &demandCurrentA);

    snprintf(buf, sizeof(buf), "OK %g\r\n", (double)demandCurrentA);
    uart_send(inst, buf);
}

/* Now gated on the state machine (state_machine.h, added 2026-09-13) --
   per direct instruction, this only ever starts outputs from ARMED
   (ARM first, see cmd_arm()). SM_Fire() calls PID_ProfileStart()
   internally and only actually transitions to FIRING if that
   succeeds -- THREE distinct failure reasons as of 2026-09-16 (state
   mismatch, external-enable interlock not satisfied, profile timing
   never set), each reported distinctly here rather than collapsing
   them into one generic error. The external-enable check is done HERE
   (SM_ExternalEnableOk(), state_machine.h) rather than just relying on
   SM_Fire()'s own internal redundant re-check of the same thing,
   specifically so this can report a precise message instead of the
   generic "stays ARMED" SM_Fire() itself returns -- see that
   function's own comment (state_machine.c) for why it still re-checks
   anyway (a narrow race window between this check and the SM_Fire()
   call, and defense against any future caller that reaches SM_Fire()
   directly). */
void cmd_pid_profile_start(uart_instance_t *inst, char *args)
{
    (void)args;

    if (SM_GetState() != SM_STATE_ARMED)
    {
        SendErr(inst, 13, "Must ARM first -- see the ARM command");
        return;
    }

    if (SM_ExternalEnableOk() == 0U)
    {
        SendErr(inst, 15, "External enable interlock not satisfied -- PF13 reads LOW");
        return;
    }

    if (SM_Fire() == 0U)
    {
        SendErr(inst, 12, "PID:PROFILE:TIMING must be set before PID:PROFILE:START");
        return;
    }
    uart_send(inst, "OK\r\n");
}

#if defined(BUILD_TARGET_SIMULATOR)
/* --------------------------------------------------------------------------
 * SIM:FAULT:WATERTEMP <ch> <0|1> / SIM:FAULT:WATERTEMP? <ch>
 * SIM:FAULT:ENERPRO <ch> <0|1> / SIM:FAULT:ENERPRO? <ch>
 * SIM:FAULT:OCP <ch> <0|1> / SIM:FAULT:OCP? <ch>
 * SIM:MODEL:TAU <ms> / SIM:MODEL:TAU?
 * SIM:CHANnel:STATus? <ch>
 *
 * Added 2026-09-18 -- the sim_transrex.h/.c backed command namespace
 * for the Transrex simulator (see that module's own header comment for
 * the full design: DRIVE capture -> low-pass filter -> FEEDBACK
 * generation, ENA_OUT/CONTACT_OUT gating, per-channel/per-category
 * fault injection). SIMULATOR-ONLY, guarded out of a controller build
 * entirely -- there is no "simulated Transrex" concept on the real
 * controller for this namespace to mean anything about, unlike the
 * generic DIAGnostic:GPOut09-12/etc. pins which stay present on both
 * targets. 1-based wire channel, converted to this project's 0-based
 * internal convention -- same pattern as every other XREX:CHANnel:*
 * command above; ERR 11/12 reused for invalid-channel/missing-argument,
 * same convention throughout this file.
 *
 * WATERTEMP/ENERPRO/OCP drive the SAME physical pins XREX:CHANnel:
 * ENAOut/CONTactOut and DIAGnostic:GPOut09-12 can also drive directly
 * (see sim_transrex.c's own pin-role table) -- don't mix both
 * mechanisms on the same channel/category at once. */
void cmd_sim_fault_watertemp(uart_instance_t *inst, char *args)
{
    char *tok;
    long  chArg;
    long  val;

    tok = (args != NULL) ? strtok(args, " \r\n") : NULL;
    if (tok == NULL)
    {
        SendErr(inst, 12, "SIM:FAULT:WATERTEMP needs two arguments: channel, 0|1");
        return;
    }
    chArg = atol(tok);
    if ((chArg < 1L) || (chArg > (long)HRTIM_NUM_CHANNELS))
    {
        SendErr(inst, 11, "Invalid PID channel");
        return;
    }

    tok = strtok(NULL, " \r\n");
    if (tok == NULL)
    {
        SendErr(inst, 12, "SIM:FAULT:WATERTEMP needs two arguments: channel, 0|1");
        return;
    }
    val = atol(tok);

    SimTransrex_SetFaultWaterTemp((uint8_t)(chArg - 1L), (val != 0L) ? 1U : 0U);
    uart_send(inst, "OK\r\n");
}

void cmd_sim_fault_watertemp_query(uart_instance_t *inst, char *args)
{
    char *tok;
    long  chArg;
    char  buf[16];

    tok = (args != NULL) ? strtok(args, " \r\n") : NULL;
    if (tok == NULL)
    {
        SendErr(inst, 12, "SIM:FAULT:WATERTEMP? needs one argument: channel");
        return;
    }
    chArg = atol(tok);
    if ((chArg < 1L) || (chArg > (long)HRTIM_NUM_CHANNELS))
    {
        SendErr(inst, 11, "Invalid PID channel");
        return;
    }

    snprintf(buf, sizeof(buf), "OK %u\r\n",
             (unsigned)SimTransrex_GetFaultWaterTemp((uint8_t)(chArg - 1L)));
    uart_send(inst, buf);
}

void cmd_sim_fault_enerpro(uart_instance_t *inst, char *args)
{
    char *tok;
    long  chArg;
    long  val;

    tok = (args != NULL) ? strtok(args, " \r\n") : NULL;
    if (tok == NULL)
    {
        SendErr(inst, 12, "SIM:FAULT:ENERPRO needs two arguments: channel, 0|1");
        return;
    }
    chArg = atol(tok);
    if ((chArg < 1L) || (chArg > (long)HRTIM_NUM_CHANNELS))
    {
        SendErr(inst, 11, "Invalid PID channel");
        return;
    }

    tok = strtok(NULL, " \r\n");
    if (tok == NULL)
    {
        SendErr(inst, 12, "SIM:FAULT:ENERPRO needs two arguments: channel, 0|1");
        return;
    }
    val = atol(tok);

    SimTransrex_SetFaultEnerpro((uint8_t)(chArg - 1L), (val != 0L) ? 1U : 0U);
    uart_send(inst, "OK\r\n");
}

void cmd_sim_fault_enerpro_query(uart_instance_t *inst, char *args)
{
    char *tok;
    long  chArg;
    char  buf[16];

    tok = (args != NULL) ? strtok(args, " \r\n") : NULL;
    if (tok == NULL)
    {
        SendErr(inst, 12, "SIM:FAULT:ENERPRO? needs one argument: channel");
        return;
    }
    chArg = atol(tok);
    if ((chArg < 1L) || (chArg > (long)HRTIM_NUM_CHANNELS))
    {
        SendErr(inst, 11, "Invalid PID channel");
        return;
    }

    snprintf(buf, sizeof(buf), "OK %u\r\n",
             (unsigned)SimTransrex_GetFaultEnerpro((uint8_t)(chArg - 1L)));
    uart_send(inst, buf);
}

void cmd_sim_fault_ocp(uart_instance_t *inst, char *args)
{
    char *tok;
    long  chArg;
    long  val;

    tok = (args != NULL) ? strtok(args, " \r\n") : NULL;
    if (tok == NULL)
    {
        SendErr(inst, 12, "SIM:FAULT:OCP needs two arguments: channel, 0|1");
        return;
    }
    chArg = atol(tok);
    if ((chArg < 1L) || (chArg > (long)HRTIM_NUM_CHANNELS))
    {
        SendErr(inst, 11, "Invalid PID channel");
        return;
    }

    tok = strtok(NULL, " \r\n");
    if (tok == NULL)
    {
        SendErr(inst, 12, "SIM:FAULT:OCP needs two arguments: channel, 0|1");
        return;
    }
    val = atol(tok);

    SimTransrex_SetFaultOcp((uint8_t)(chArg - 1L), (val != 0L) ? 1U : 0U);
    uart_send(inst, "OK\r\n");
}

void cmd_sim_fault_ocp_query(uart_instance_t *inst, char *args)
{
    char *tok;
    long  chArg;
    char  buf[16];

    tok = (args != NULL) ? strtok(args, " \r\n") : NULL;
    if (tok == NULL)
    {
        SendErr(inst, 12, "SIM:FAULT:OCP? needs one argument: channel");
        return;
    }
    chArg = atol(tok);
    if ((chArg < 1L) || (chArg > (long)HRTIM_NUM_CHANNELS))
    {
        SendErr(inst, 11, "Invalid PID channel");
        return;
    }

    snprintf(buf, sizeof(buf), "OK %u\r\n",
             (unsigned)SimTransrex_GetFaultOcp((uint8_t)(chArg - 1L)));
    uart_send(inst, buf);
}

void cmd_sim_model_tau(uart_instance_t *inst, char *args)
{
    char *tok;
    long  val;

    tok = (args != NULL) ? strtok(args, " \r\n") : NULL;
    if (tok == NULL)
    {
        SendErr(inst, 12, "SIM:MODEL:TAU needs one argument: milliseconds");
        return;
    }
    val = atol(tok);

    if ((val <= 0L) || (SimTransrex_SetTauMs((uint32_t)val) == 0U))
    {
        SendErr(inst, 11, "Invalid tau -- must be a positive number of milliseconds");
        return;
    }
    uart_send(inst, "OK\r\n");
}

void cmd_sim_model_tau_query(uart_instance_t *inst, char *args)
{
    char buf[16];
    (void)args;

    snprintf(buf, sizeof(buf), "OK %lu\r\n", (unsigned long)SimTransrex_GetTauMs());
    uart_send(inst, buf);
}

void cmd_sim_channel_status(uart_instance_t *inst, char *args)
{
    char *tok;
    long  chArg;
    char  buf[128];
    uint32_t measuredHz = 0U, feedbackHz = 0U;
    uint8_t  enaGated = 0U, contactGated = 0U;
    uint8_t  faultWaterTemp = 0U, faultEnerpro = 0U, faultOcp = 0U;

    tok = (args != NULL) ? strtok(args, " \r\n") : NULL;
    if (tok == NULL)
    {
        SendErr(inst, 12, "SIM:CHANnel:STATus? needs one argument: channel");
        return;
    }
    chArg = atol(tok);
    if ((chArg < 1L) || (chArg > (long)HRTIM_NUM_CHANNELS))
    {
        SendErr(inst, 11, "Invalid PID channel");
        return;
    }

    (void)SimTransrex_GetChannelStatus((uint8_t)(chArg - 1L), &measuredHz, &feedbackHz,
                                        &enaGated, &contactGated,
                                        &faultWaterTemp, &faultEnerpro, &faultOcp);

    snprintf(buf, sizeof(buf),
             "OK DRIVE_HZ=%lu FEEDBACK_HZ=%lu ENA_OUT=%s CONTACT_OUT=%s "
             "WATERTEMP_FAULT=%u ENERPRO_FAULT=%u OCP_FAULT=%u\r\n",
             (unsigned long)measuredHz, (unsigned long)feedbackHz,
             enaGated ? "HIGH" : "LOW", contactGated ? "HIGH" : "LOW",
             (unsigned)faultWaterTemp, (unsigned)faultEnerpro, (unsigned)faultOcp);
    uart_send(inst, buf);
}

/* --------------------------------------------------------------------------
 * SIM:LOG <ch> <maxSamples> <minIntervalMs>
 * SIM:LOGDATA? <ch>
 *
 * Added 2026-09-18, per direct correction: host-side polling of
 * SIM:CHANnel:STATus? during a shot (the original
 * run_simulator_validation.py approach) was too coarse and added
 * serial round-trip jitter -- this is the simulator-side equivalent of
 * PID:LOG/PID:LOGDATA? (pid.h/commands.c), backed by
 * SimTransrex_ArmLog()/GetLogCount()/GetLogSample() (sim_transrex.h --
 * see that header's own comment for why each sample carries its own
 * timestamp instead of a single shared rate_hz like PID:LOGDATA? uses:
 * SimTransrex_Update() runs off the main loop, not a fixed hardware
 * tick). `ch` is 1-based on the wire, same convention as every other
 * XREX:CHANnel:/PID: command in this file. */
void cmd_sim_log(uart_instance_t *inst, char *args)
{
    char *tok;
    long  chArg;
    long  maxSamplesArg;
    long  minIntervalArg;

    tok = (args != NULL) ? strtok(args, " \r\n") : NULL;
    if (tok == NULL)
    {
        SendErr(inst, 12, "SIM:LOG needs three arguments: channel maxSamples minIntervalMs");
        return;
    }
    chArg = atol(tok);
    if ((chArg < 1L) || (chArg > (long)HRTIM_NUM_CHANNELS))
    {
        SendErr(inst, 11, "Invalid PID channel");
        return;
    }

    tok = strtok(NULL, " \r\n");
    if (tok == NULL)
    {
        SendErr(inst, 12, "SIM:LOG needs three arguments: channel maxSamples minIntervalMs");
        return;
    }
    maxSamplesArg = atol(tok);

    tok = strtok(NULL, " \r\n");
    if (tok == NULL)
    {
        SendErr(inst, 12, "SIM:LOG needs three arguments: channel maxSamples minIntervalMs");
        return;
    }
    minIntervalArg = atol(tok);

    if ((maxSamplesArg < 1L) || (maxSamplesArg > (long)SIM_LOG_MAX_SAMPLES) || (minIntervalArg < 0L))
    {
        SendErr(inst, 12, "maxSamples must be 1-SIM_LOG_MAX_SAMPLES, minIntervalMs >= 0");
        return;
    }

    (void)SimTransrex_ArmLog((uint8_t)(chArg - 1L), (uint16_t)maxSamplesArg, (uint16_t)minIntervalArg);
    uart_send(inst, "OK\r\n");
}

void cmd_sim_logdata(uart_instance_t *inst, char *args)
{
    char *tok;
    long  chArg;
    uint8_t ch;
    uint16_t count;
    char chunk[48];

    tok = (args != NULL) ? strtok(args, " \r\n") : NULL;
    if (tok == NULL)
    {
        SendErr(inst, 12, "SIM:LOGDATA? needs one argument: channel");
        return;
    }
    chArg = atol(tok);
    if ((chArg < 1L) || (chArg > (long)HRTIM_NUM_CHANNELS))
    {
        SendErr(inst, 11, "Invalid PID channel");
        return;
    }
    ch = (uint8_t)(chArg - 1L);

    count = SimTransrex_GetLogCount(ch);
    snprintf(chunk, sizeof(chunk), "OK %u", (unsigned int)count);
    uart_send(inst, chunk);

    for (uint16_t i = 0U; i < count; i++)
    {
        uint32_t timeMs = 0U, driveHz = 0U, feedbackHz = 0U;
        (void)SimTransrex_GetLogSample(ch, i, &timeMs, &driveHz, &feedbackHz);
        snprintf(chunk, sizeof(chunk), " %lu %lu %lu",
                 (unsigned long)timeMs, (unsigned long)driveHz, (unsigned long)feedbackHz);
        uart_send(inst, chunk);
    }

    uart_send(inst, "\r\n");
}
#endif /* BUILD_TARGET_SIMULATOR */
