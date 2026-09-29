/*
 * cmd_state.c
 *
 * Operating state and faults: FAULT?/FAULT:CLEar, ARM/DISARM/
 * STATE?, DEBUG:FAULT:BYPASS, OCP:TEST:FAULT, GENERAL:TEST:FAULT.
 * Handlers are declared in commands.h and registered in command_table.c.
 */

#include "commands.h"
#include "cmd_common.h"
#include "ctrlr_config.h"
#include "state_machine.h"
#include "xrex_io.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* --------------------------------------------------------------------------
 * FAULT? / FAULT:CLEar
 *
 * Status/clear for BOTH of this project's fault sources, combined into
 * one operator-facing fault state (AnyFaultLatched(), cmd_common.c):
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
 * `SOURce:`/other namespace prefix) -- system-wide state, not specific to
 * any one subsystem, matching this project's existing bare `FIRE`
 * (pfm.c's legacy table-based output path, unrelated). Error code 13
 * (new): an invalid state-machine transition for the current state.
 * -------------------------------------------------------------------------- */

/* IDLE -> ARMED. See SM_Arm()'s own doc comment for exactly what
   "the appropriate conditions" currently checks.

   REWORKED 2026-09-22, direct request: this used to collapse every
   failure reason (wrong state, external-enable interlock not
   satisfied, a specific channel's ENA_OUT/CONTACT_OUT not both set)
   into one generic "conditions not met" -- an operator with one
   misconfigured channel out of four had no way to tell which one from
   the wire protocol alone. Now checks each precondition itself, in the
   same order ArmConditionsMet() (state_machine.c) does internally, and
   reports the FIRST one that fails with its own specific message --
   matching the "distinct failure reasons, reported distinctly" pattern
   cmd_shot_start()/cmd_source_run() already established for
   SHOT:STARt/SOURce:RUN. SM_Arm() itself is still called last as
   the actual transition (and its own internal ArmConditionsMet() check
   stays as a redundant last-line-of-defense re-check, same reasoning
   as SM_Fire()'s own re-check of SM_ExternalEnableOk() -- a narrow
   race window between these checks and the call, and defense against
   any other caller that reaches SM_Arm() directly) -- its generic
   fallback message below should only ever fire on that narrow race,
   never on a normal, reproducible misconfiguration. */
void cmd_arm(uart_instance_t *inst, char *args)
{
    /* 96 bytes: comfortably fits the longest message below (82 chars)
       plus SendErr()'s own "ERR %d %s\r\n" framing, well under
       SendErr()'s own 128-byte buffer (cmd_common.c) --
       *** REAL BUG, FOUND AND FIXED 2026-09-22, confirmed on real
       hardware ***: the first cut used buf[64], which silently
       truncated this exact message mid-word ("...see XREX:CHA") since
       the full text is 120 chars -- snprintf null-terminates within
       whatever size it's given rather than overflowing, so this never
       crashed or corrupted anything, it just quietly sent a cut-off
       error message. Shortened the message text too (dropped the
       trailing "or SOURce:ENAble 0 if unused" clause) rather than
       just growing the buffer further, since the core "which channel,
       which check" information is what actually matters here. */
    char buf[96];
    uint8_t badChannel;

    (void)args;

    if (SM_GetState() != SM_STATE_IDLE)
    {
        SendErr(inst, ERR_INVALID_STATE, "Can't ARM -- not currently IDLE");
        return;
    }

    if (SM_ExternalEnableOk() == 0U)
    {
        SendErr(inst, ERR_EXT_ENABLE_LOW, "External enable interlock not satisfied -- PF13 reads LOW");
        return;
    }

    badChannel = XrexIo_FindNotReadyChannel();
    if (badChannel != 0xFFU)
    {
        snprintf(buf, sizeof(buf),
                 "Channel %u's ENA_OUT/CONTACT_OUT not both set -- see "
                 "XREX:CHANnel:ENAOut/CONTactOut", (unsigned)(badChannel + 1U));
        SendErr(inst, ERR_INVALID_STATE, buf);
        return;
    }

    if (SM_Arm() == 0U)
    {
        SendErr(inst, ERR_INVALID_STATE, "Can't ARM -- arm conditions not met");   /* narrow race only, see comment above */
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

/* DEBUG:FAULT:BYPASS <0|1> / ? -- added 2026-09-21, direct request: a
   bench-only override so a channel can be run with no real fault-
   detect signal present (e.g. testing a controller-target build with
   no simulator/Transrex physically connected to feed its Water/Temp/
   Enerpro/OCP inputs healthy -- they float, and float reads as an
   instant fault the moment a channel is enabled). See
   state_machine.h's own SM_SetFaultBypassEnabled() comment for the
   full reasoning/safety warning -- defaults OFF at every boot, RAM-
   only, never persisted. EXTENDED 2026-09-24: also lets FAULT:CLEAR
   succeed regardless of the physical cause and stops PF13-low /
   latched-fault checks from blocking ARM/FIRE -- see that same
   comment. */
void cmd_debug_fault_bypass(uart_instance_t *inst, char *args)
{
    char *tok;
    long  val;

    tok = (args != NULL) ? strtok(args, " \r\n") : NULL;
    if (tok == NULL)
    {
        SendErr(inst, ERR_INVALID_ARGS, "DEBUG:FAULT:BYPASS needs one argument: 0 or 1");
        return;
    }
    val = atol(tok);
    if ((val != 0L) && (val != 1L))
    {
        SendErr(inst, ERR_INVALID_CHANNEL, "Invalid value -- must be 0 or 1");
        return;
    }
    SM_SetFaultBypassEnabled((uint8_t)val);
    uart_send(inst, "OK\r\n");
}

void cmd_debug_fault_bypass_query(uart_instance_t *inst, char *args)
{
    char buf[16];
    (void)args;

    snprintf(buf, sizeof(buf), "OK %u\r\n", (unsigned)SM_GetFaultBypassEnabled());
    uart_send(inst, buf);
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
 * reasoning FAULT:CLEAR/SOURce:STOP already get.
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
        SendErr(inst, ERR_INVALID_ARGS, "OCP:TEST:FAULT needs one argument: channel");
        return;
    }
    chArg = atol(tok);

    if ((chArg < 1L) || (chArg > (long)HRTIM_NUM_CHANNELS))
    {
        SendErr(inst, ERR_INVALID_CHANNEL, "Invalid channel");
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
 * OCP:TEST:FAULT/FAULT:CLEAR/SOURce:STOP -- triggering a fault only ever
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
