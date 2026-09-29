/*
 * cmd_common.c -- shared pieces of the serial command layer, see
 * cmd_common.h.
 */
#include "cmd_common.h"
#include "hrtim.h"
#include "gate_driver.h"
#include "state_machine.h"
#include <stdio.h>

void SendErr(uart_instance_t *inst, int code, const char *msg)
{
    char buf[128];
    snprintf(buf, sizeof(buf), "ERR %d %s\r\n", code, msg);
    uart_send(inst, buf);
}

/* True if EITHER fault source is latched: PC10/HRTIM1_FLT6 (native
   hardware, hrtim.h) or the GateDriverStatus EXTI interrupt
   (gate_driver.h, PE0..PE11). One combined answer for FAULT?/FIRE's
   ERR 6 gate/FAULT:CLEAR -- see cmd_fault_query()'s own comment for
   why these two structurally-independent mechanisms present as one
   fault state at the command layer. */
uint8_t AnyFaultLatched(void)
{
    return (HRTIM1_FaultIsTripped() != 0U) || (GateDriver_FaultIsLatched() != 0U);
}

/* Shared by cmd_fire/cmd_source_run/cmd_shot_start -- the ARMED-state +
   external-enable-interlock precondition every fire-attempt command
   requires before actually starting output. Factored out 2026-09-22:
   found duplicated byte-for-byte in all three during a review of the
   PID: -> SOURce:/SHOT:/LOG:/CHANnel: rename -- this is safety-
   interlock logic, and three independent copies meant a future change
   to either condition could easily be applied to only some of them.
   Deliberately does NOT cover cmd_arm()'s own similar-looking check
   (IDLE-state + external-enable + per-channel-output-readiness,
   cmd_state.c's cmd_arm()) -- that one reports a DIFFERENT "not
   currently IDLE" message and has a third precondition this helper
   doesn't, so folding it in here would either lose that distinct
   messaging or bloat this helper with an ARM-only concern; kept
   separate on purpose, not missed.

   Returns 1 if the caller may proceed; 0 if an ERR has already been
   sent and the caller should return immediately without sending
   anything further. */
uint8_t RequireArmedAndEnabled(uart_instance_t *inst)
{
    if (SM_GetState() != SM_STATE_ARMED)
    {
        SendErr(inst, ERR_INVALID_STATE, "Must ARM first -- see the ARM command");
        return 0U;
    }
    if (SM_ExternalEnableOk() == 0U)
    {
        SendErr(inst, ERR_EXT_ENABLE_LOW, "External enable interlock not satisfied -- PF13 reads LOW");
        return 0U;
    }
    return 1U;
}
