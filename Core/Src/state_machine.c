/*
 * state_machine.c
 *
 * See state_machine.h for the full design writeup (states, fault
 * types, what's stubbed and what's real).
 */

#include "state_machine.h"
#include "pid.h"
#include "hrtim.h"
#include "gate_driver.h"
#include "pfm.h"

static SM_State_t     g_state     = SM_STATE_IDLE;
static SM_FaultType_t g_faultType = SM_FAULT_NONE;

/* -- fault-type handler stubs, per direct instruction: empty for now -- */

/* Called on entering SM_STATE_FAULT with g_faultType == SM_FAULT_GENERAL.
   Currently every fault this codebase can detect routes here -- see
   state_machine.h's own "NOTE TO REVISIT" on why, and SM_PollFaults()
   below. EMPTY -- populate with real General Fault handling later
   (whatever that turns out to mean: a specific recovery sequence,
   logging, an operator notification path, etc. -- not yet decided). */
static void HandleGeneralFault(void)
{
    /* TODO (2026-09-13): populate. */
}

/* Called on entering SM_STATE_FAULT with g_faultType == SM_FAULT_OVERCURRENT.
   Nothing in this codebase currently produces this fault type -- see
   state_machine.h's own "NOTE TO REVISIT". EMPTY -- populate once both
   (a) a real overcurrent-detection mechanism exists and is routed here,
   and (b) the real handling behavior (distinct from General Fault) is
   decided. */
static void HandleOvercurrentFault(void)
{
    /* TODO (2026-09-13): populate. */
}

/* Readiness gate for SM_Arm() -- STUB, per direct instruction: always
   allows arming today. Real interlock conditions (no fault active --
   already implicitly guaranteed, since IDLE and FAULT are mutually
   exclusive states, so nothing further needed for that one
   specifically; profile timing actually configured; at least one
   channel enabled; gains sane; whatever else turns out to matter) are
   explicitly NOT implemented yet -- populate later. */
static uint8_t ArmConditionsMet(void)
{
    return 1U;   /* TODO (2026-09-13): populate real conditions. */
}

/* Shared by SM_PollFaults() -- the one place a fault is actually
   latched into the state machine, regardless of which of the two
   sources tripped or which of the two call sites (main loop vs.
   PID_Update()) noticed it. */
static void EnterFault(SM_FaultType_t type)
{
    g_state     = SM_STATE_FAULT;
    g_faultType = type;

    /* Full stop across BOTH the current (pid.c) and legacy (pfm.c)
       output paths, regardless of which was actually in use -- belt
       and suspenders. Both are already idempotent/safe to call when
       nothing was running (PID_Stop(): "idempotent" per its own doc
       comment; PFM_ForceStop(): the same, matching
       HRTIM1_FaultClear()'s own established pattern) -- see
       state_machine.h's own comment on why this project's two output
       paths (old table-based FIRE, new PID:PROFile:STARt) both get
       stopped unconditionally here rather than trying to track which
       one was actually active. */
    PID_Stop();
    PFM_ForceStop();

    switch (type)
    {
        case SM_FAULT_OVERCURRENT:
            HandleOvercurrentFault();
            break;
        case SM_FAULT_GENERAL:
        case SM_FAULT_NONE:
        default:
            HandleGeneralFault();
            break;
    }
}

void SM_Init(void)
{
    g_state     = SM_STATE_IDLE;
    g_faultType = SM_FAULT_NONE;
}

SM_State_t SM_GetState(void)
{
    return g_state;
}

SM_FaultType_t SM_GetFaultType(void)
{
    return (g_state == SM_STATE_FAULT) ? g_faultType : SM_FAULT_NONE;
}

uint8_t SM_Arm(void)
{
    if (g_state != SM_STATE_IDLE)
    {
        return 0U;
    }
    if (ArmConditionsMet() == 0U)
    {
        return 0U;
    }

    g_state = SM_STATE_ARMED;
    return 1U;
}

void SM_Disarm(void)
{
    if (g_state == SM_STATE_ARMED)
    {
        g_state = SM_STATE_IDLE;
    }
}

uint8_t SM_Fire(void)
{
    if (g_state != SM_STATE_ARMED)
    {
        return 0U;
    }

    if (PID_ProfileStart() == 0U)
    {
        return 0U;   /* stays ARMED -- e.g. profile timing never set */
    }

    g_state = SM_STATE_FIRING;
    return 1U;
}

void SM_Stop(void)
{
    /* Deliberately does NOT touch FAULT -- see this function's own
       doc comment in state_machine.h. */
    if ((g_state == SM_STATE_ARMED) || (g_state == SM_STATE_FIRING))
    {
        g_state = SM_STATE_IDLE;
    }
}

void SM_NotifyShotComplete(void)
{
    if (g_state == SM_STATE_FIRING)
    {
        g_state = SM_STATE_IDLE;
    }
}

void SM_PollFaults(void)
{
    if (g_state == SM_STATE_FAULT)
    {
        return;   /* already latched -- nothing new to do */
    }

    /* Both existing sources route to SM_FAULT_GENERAL unconditionally
       today -- see state_machine.h's own "NOTE TO REVISIT" comment on
       why, and this function's own header comment for the two call
       sites (main loop + PID_Update()) this reaches from. */
    if ((HRTIM1_FaultIsTripped() != 0U) || (GateDriver_FaultIsLatched() != 0U))
    {
        EnterFault(SM_FAULT_GENERAL);
    }
}

uint8_t SM_ClearFault(void)
{
    if (g_state != SM_STATE_FAULT)
    {
        return 0U;   /* nothing to clear */
    }

    /* Both already re-validate their own source before truly clearing
       (see their own doc comments in hrtim.h/gate_driver.h) -- a
       condition that's still physically present re-latches before
       either call even returns, so SM_PollFaults()'s next check would
       just re-detect it and re-enter FAULT again anyway. Re-check
       directly here instead, for an honest return value on THIS call
       rather than making the caller poll again to find out. */
    HRTIM1_FaultClear();
    GateDriver_FaultClear();

    if ((HRTIM1_FaultIsTripped() != 0U) || (GateDriver_FaultIsLatched() != 0U))
    {
        return 0U;   /* still faulted -- stays in SM_STATE_FAULT */
    }

    g_state     = SM_STATE_IDLE;
    g_faultType = SM_FAULT_NONE;
    return 1U;
}
