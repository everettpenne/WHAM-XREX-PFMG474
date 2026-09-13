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

static SM_State_t     g_state         = SM_STATE_IDLE;
static SM_FaultType_t g_faultType     = SM_FAULT_NONE;

/* Captured by EnterFault(), before it transitions g_state to
   SM_STATE_FAULT -- lets the fault-type handlers below tell whether
   there was real output in progress (SM_STATE_FIRING) at the exact
   moment the fault was detected, without needing g_state itself (which
   is already SM_STATE_FAULT by the time either handler runs) to carry
   that information. */
static SM_State_t g_stateBeforeFault = SM_STATE_IDLE;

/* -- fault-type handlers -- */

/* Called on entering SM_STATE_FAULT with g_faultType == SM_FAULT_GENERAL.
   Currently every fault this codebase can detect routes here -- see
   state_machine.h's own "NOTE TO REVISIT" on why, and SM_PollFaults()
   below.

   POPULATED 2026-09-13, per direct instruction: if the fault hit while
   FIRING (real output in progress, ANY channel, anywhere in its own
   shot profile), every actively-outputting channel immediately begins
   an OPEN-LOOP linear ramp-down to PFM_TURNON_FREQ_HZ (0A) over
   FAULT_RAMP_DOWN_TIME_S seconds (PID_BeginFaultRampDown()/
   ProcessFaultRampDown(), pid.c) -- explicitly NOT via the normal PID
   feedback loop, per direct instruction. PID_Update() keeps ticking
   (g_running stays 1) to drive this ramp forward, one tick at a time,
   until every participating channel reaches the floor, at which point
   pid.c calls PID_Stop() itself (disconnecting every HRTIM output
   unconditionally) and the controller settles into FAULT to wait for
   FAULT:CLEAR -- exactly "disable the HRTIM output channels and wait
   in the FAULT state," per direct instruction.

   If the fault hit while IDLE or ARMED, nothing was actually
   outputting for it to have interrupted -- an immediate PID_Stop() is
   all that's needed (also covers the case where every channel happened
   to already be disabled/idle even during FIRING -- see
   PID_BeginFaultRampDown()'s own "anyParticipating" check). */
static void HandleGeneralFault(void)
{
    if (g_stateBeforeFault == SM_STATE_FIRING)
    {
        PID_BeginFaultRampDown();
    }
    else
    {
        PID_Stop();
    }
}

/* Called on entering SM_STATE_FAULT with g_faultType == SM_FAULT_OVERCURRENT.
   Nothing in this codebase currently produces this fault type -- see
   state_machine.h's own "NOTE TO REVISIT". Its own DISTINCT behavior
   (as opposed to General Fault's open-loop ramp-down, above) is still
   undecided -- an immediate, unconditional PID_Stop() is kept here as
   the safe default in the meantime (matching this function's own
   behavior before General Fault was populated) -- an "overcurrent
   protection fault" sounds, if anything, like it should be MORE
   aggressive/immediate than General Fault's graceful ramp-down, not
   less, so defaulting to the fastest possible stop until a real
   decision is made seems like the safer placeholder of the two
   options, not an oversight. */
static void HandleOvercurrentFault(void)
{
    PID_Stop();
    /* TODO (2026-09-13): populate this fault type's own distinct
       behavior, once decided -- see state_machine.h's own
       "NOTE TO REVISIT". */
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
    g_stateBeforeFault = g_state;   /* captured BEFORE transitioning --
                                        see this variable's own comment
                                        above for why the handlers need it */
    g_state     = SM_STATE_FAULT;
    g_faultType = type;

    /* Legacy (pfm.c TABLE:STEP/FIRE) output path -- always stopped
       immediately and unconditionally here, regardless of fault type
       or which state pid.c's own state machine was in. This path is
       NOT part of the new IDLE/ARMED/FIRING/FAULT model at all (the
       legacy FIRE command never calls SM_Fire()), so it has no
       ramp-down concept to preserve -- an instant stop is correct for
       it either way. Idempotent/safe to call when nothing was running
       (matching HRTIM1_FaultClear()'s own established pattern). The
       CURRENT (pid.c PID:*) output path's own stop is now each fault
       type's OWN responsibility (see HandleGeneralFault()/
       HandleOvercurrentFault(), below) -- General Fault's whole point
       (2026-09-13, per direct instruction) is NOT stopping it
       immediately here, but instead beginning a controlled ramp-down
       that keeps Master's counter running until it finishes. */
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
    /* Critical section: this function runs from BOTH the main loop
       (background context) and PID_Update() (HRTIM1_Master_IRQn, priority
       1, which can preempt the main loop at any instruction). Without a
       guard, the main loop could read g_state (not yet FAULT), get
       preempted right there by the ISR's own call to this function
       completing EnterFault() first (possibly starting a General-Fault
       ramp-down), and then resume and -- since its OWN check already
       passed, before preemption -- call EnterFault() a second, redundant
       time. That second call would see g_stateBeforeFault == SM_STATE_FAULT
       (not SM_STATE_FIRING, since the first legitimate call already
       transitioned it) and take HandleGeneralFault()'s "else" branch,
       calling PID_Stop() immediately and silently replacing the just-
       started graceful ramp-down with an abrupt cutoff. That outcome was
       initially (wrongly) judged "safety-neutral" here -- corrected
       directly: the whole point of a linear ramp instead of an instant
       stop is to avoid inductive/mechanical stress, so silently
       substituting the abrupt path IS the hazard, not a neutral one.

       __disable_irq()/__enable_irq() is this codebase's own established
       critical-section pattern (see boot_jump.c, main.c). Wrapping the
       entire read-check-transition sequence -- including EnterFault()
       itself -- makes the double-entry race structurally impossible
       rather than merely unlikely: whichever context (main loop or ISR)
       gets here first now runs this whole function to completion before
       the other context's own call can even read g_state. EnterFault()'s
       own work (PFM_ForceStop(), the fault-type dispatch,
       PID_BeginFaultRampDown()) is all fast, non-blocking register/flag
       writes with no dependency on any interrupt firing to complete, so
       it's safe to run with IRQs globally disabled. Calling this from
       inside an ISR (the PID_Update() call site) is fine too --
       __disable_irq() while already inside an ISR is idempotent for the
       brief window this takes, same as boot_jump.c's own unconditional
       use without special-casing ISR vs. main context. */
    __disable_irq();

    if (g_state == SM_STATE_FAULT)
    {
        __enable_irq();
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

    __enable_irq();
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
       rather than making the caller poll again to find out.

       Also unconditionally calls PID_Stop() (added 2026-09-13,
       alongside the General-Fault ramp-down): a General Fault caught
       while FIRING leaves g_running deliberately 1 -- and possibly a
       ramp-down actively in progress -- for as long as
       PID_BeginFaultRampDown()'s own ramp hasn't finished yet (see
       pid.c). HRTIM1_FaultClear()/GateDriver_FaultClear() above
       already force-stop HRTIM unconditionally regardless (same as
       PFM_ForceStop() in EnterFault()), so an operator clearing the
       fault mid-ramp already gets an immediate hard stop physically --
       but without this PID_Stop() call, pid.c's own g_running/
       g_faultRampActive bookkeeping would be left stale (g_running
       still 1 with nothing actually running), which is exactly the
       2026-09-11 bug (docs/changelog.txt) all over again if left
       unfixed. Idempotent/harmless if the ramp had already finished
       normally (PID_Stop() would have already been called then, by
       pid.c itself). */
    PID_Stop();

    if ((HRTIM1_FaultIsTripped() != 0U) || (GateDriver_FaultIsLatched() != 0U))
    {
        return 0U;   /* still faulted -- stays in SM_STATE_FAULT */
    }

    g_state     = SM_STATE_IDLE;
    g_faultType = SM_FAULT_NONE;
    return 1U;
}
