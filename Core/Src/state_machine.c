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

/* Meaningful only when g_faultType == SM_FAULT_OVERCURRENT -- see
   SM_GetFaultChannel()'s own doc comment (state_machine.h). 0xFF is the
   "not applicable" sentinel (never a real channel index). Added
   2026-09-15 alongside SM_ReportOcpFault(). */
static uint8_t g_faultChannel = 0xFFU;

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

/* Called on entering SM_STATE_FAULT with g_faultType == SM_FAULT_OVERCURRENT
   -- POPULATED 2026-09-15, per direct instruction. Channel-aware, unlike
   General Fault: if the fault hit while FIRING, `channel`'s own HRTIM
   output is disabled IMMEDIATELY (no ramp for it at all), every OTHER
   currently-enabled channel is stepped down by (100 * 1/N)% of its own
   current output (N = channels enabled at the fault instant, `channel`
   included), and the survivors then ramp on down to PFM_TURNON_FREQ_HZ
   exactly like General Fault -- see PID_BeginOvercurrentRampDown()'s own
   extensive doc comment (pid.h) for the full mechanism, including a real,
   deliberately-not-silently-resolved tension between "simultaneous" and
   the existing hard slew-rate clamp.

   If the fault hit while IDLE or ARMED, nothing was actually outputting
   for it to have interrupted -- PID_BeginOvercurrentRampDown() itself
   falls through to an immediate stop in that case (via the same path
   PID_BeginFaultRampDown()'s own "anyParticipating" check already
   provides), so no separate branch is needed here, unlike
   HandleGeneralFault()'s explicit if/else above. */
static void HandleOvercurrentFault(uint8_t channel)
{
    if (g_stateBeforeFault == SM_STATE_FIRING)
    {
        PID_BeginOvercurrentRampDown(channel);
    }
    else
    {
        PID_SetChannelEnable(channel, 0U);   /* still disable it, even with
                                                  nothing else to ramp */
        PID_Stop();
    }
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

/* Shared by SM_PollFaults() AND SM_ReportOcpFault() -- the one place a
   fault is actually latched into the state machine, regardless of which
   source/call site noticed it. `channel` is meaningless for anything
   other than SM_FAULT_OVERCURRENT -- pass 0xFF (SM_PollFaults()'s own
   call site does) when it doesn't apply, matching SM_GetFaultChannel()'s
   own "not applicable" sentinel. Extended 2026-09-15 to take `channel` --
   previously SM_FAULT_GENERAL was this function's only real caller. */
static void EnterFault(SM_FaultType_t type, uint8_t channel)
{
    g_stateBeforeFault = g_state;   /* captured BEFORE transitioning --
                                        see this variable's own comment
                                        above for why the handlers need it */
    g_state        = SM_STATE_FAULT;
    g_faultType    = type;
    g_faultChannel = channel;

    /* Legacy (pfm.c TABLE:STEP/FIRE) output path -- always safed here,
       regardless of fault type or which state pid.c's own state
       machine was in. This path is NOT part of the new IDLE/ARMED/
       FIRING/FAULT model at all (the legacy FIRE command never calls
       SM_Fire()), so it has no ramp-down concept to preserve -- an
       instant stop is correct for it either way. Idempotent/safe to
       call when nothing was running (matching HRTIM1_FaultClear()'s
       own established pattern). The CURRENT (pid.c PID:*) output
       path's own stop is now each fault type's OWN responsibility (see
       HandleGeneralFault()/HandleOvercurrentFault(), below) --
       General/Overcurrent Fault's whole point is NOT stopping it
       immediately here, but instead beginning a controlled ramp-down
       that keeps Master's counter running until it finishes.

       *** REAL BUG, FIXED 2026-09-15, confirmed on real hardware ***:
       this used to call the FULL PFM_ForceStop() unconditionally,
       EVERY time, before either handler even ran -- which stops the
       SAME shared HRTIM Master/channel counters a ramp-down needs to
       keep running (HRTIM1_PWM_Stop(), see PFM_ForceStopSoft()'s own
       extensive doc comment, pfm.h, for the full diagnostic). The
       comment directly above THIS one already said the point was "NOT
       stopping it immediately here" -- the code contradicted its own
       stated intent. Fixed: use the SOFT stop (no counter touch) when
       a ramp-down might actually begin (g_stateBeforeFault ==
       SM_STATE_FIRING, for EITHER fault type -- both handlers check
       this same condition before deciding to ramp vs. stop
       immediately); the FULL stop is still correct and unchanged for
       IDLE/ARMED (nothing was outputting, nothing needs the counters
       to keep running). */
    if (g_stateBeforeFault == SM_STATE_FIRING)
    {
        PFM_ForceStopSoft();
    }
    else
    {
        PFM_ForceStop();
    }

    switch (type)
    {
        case SM_FAULT_OVERCURRENT:
            HandleOvercurrentFault(channel);
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
    g_state        = SM_STATE_IDLE;
    g_faultType    = SM_FAULT_NONE;
    g_faultChannel = 0xFFU;
}

SM_State_t SM_GetState(void)
{
    return g_state;
}

SM_FaultType_t SM_GetFaultType(void)
{
    return (g_state == SM_STATE_FAULT) ? g_faultType : SM_FAULT_NONE;
}

uint8_t SM_GetFaultChannel(void)
{
    return (SM_GetFaultType() == SM_FAULT_OVERCURRENT) ? g_faultChannel : 0xFFU;
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
        EnterFault(SM_FAULT_GENERAL, 0xFFU);   /* channel N/A for a system-wide fault */
    }

    __enable_irq();
}

void SM_ReportOcpFault(uint8_t channel)
{
    if (channel >= HRTIM_NUM_CHANNELS)
    {
        return;   /* defensive -- a real, correctly-wired caller never
                      produces this */
    }

    /* Same critical-section reasoning as SM_PollFaults() above -- see
       that function's own extensive comment. This can eventually be
       called from an ISR (a real OCP pin's EXTI handler, once wired up)
       just like GateDriver_CheckFault() already is, and needs the same
       protection against the same main-loop/ISR race SM_PollFaults()
       itself was fixed for. */
    __disable_irq();

    if (g_state == SM_STATE_FAULT)
    {
        /* Already faulted (either type) -- see this header's own "ALSO
           NOTE TO REVISIT" comment (state_machine.h) on why the full
           redistribute-and-ramp sequence does NOT re-run here. Still
           always hard-disables THIS channel's own output unconditionally
           -- cheap, safe, and a real OCP condition must never be left
           connected just because some other channel's fault got here
           first. */
        (void)PID_SetChannelEnable(channel, 0U);
        __enable_irq();
        return;
    }

    EnterFault(SM_FAULT_OVERCURRENT, channel);
    __enable_irq();
}

uint8_t SM_ClearFault(void)
{
    if (g_state != SM_STATE_FAULT)
    {
        return 0U;   /* nothing to clear */
    }

    /* Unconditionally calls PID_Stop() (added 2026-09-13, alongside the
       General-Fault ramp-down): a General/Overcurrent Fault caught
       while FIRING leaves g_running deliberately 1 -- and possibly a
       ramp-down actively in progress -- for as long as
       PID_BeginFaultRampDown()'s own ramp hasn't finished yet (see
       pid.c). Called FIRST, before either FaultClear() below, so the
       shared HRTIM hardware is already fully, unconditionally stopped
       (PID_Stop() -> HRTIM1_PWM_Stop()) by the time they run -- clearing
       a fault mid-ramp is an immediate hard stop, not a resumed ramp.
       Without this call, pid.c's own g_running/g_faultRampActive
       bookkeeping would be left stale (g_running still 1 with nothing
       actually running), which is exactly the 2026-09-11 bug
       (docs/changelog.txt) all over again if left unfixed. Idempotent/
       harmless if the ramp had already finished normally (PID_Stop()
       would have already been called then, by pid.c itself). */
    PID_Stop();

    /* *** REAL BUG, FIXED 2026-09-15 *** -- this function's own
       long-standing comment (now corrected) claimed
       "HRTIM1_FaultClear()/GateDriver_FaultClear() above already
       force-stop HRTIM unconditionally," but neither was EVER actually
       called anywhere in this codebase -- confirmed by a full search,
       not an assumption. Concretely: GateDriver_FaultClear() is the
       ONLY thing that ever resets g_gdsFaultLatched back to 0
       (gate_driver.c); since nothing called it, a GateDriverStatus-
       triggered fault could NEVER actually be cleared via FAULT:CLEAR
       -- GateDriver_FaultIsLatched() below would stay 1 forever,
       forcing a full power cycle to recover from ANY such fault. PC10/
       HRTIM1_FLT6 is likely equally affected (HRTIM fault flags latch
       in hardware until explicitly cleared). Found while investigating
       the separate frozen-ramp bug above (PFM_ForceStopSoft(), pfm.h)
       -- unrelated root cause, same area, real enough to fix alongside
       it rather than leave for later now that it's found. Both
       re-validate their own source before truly clearing (see their
       own doc comments in hrtim.h/gate_driver.h) -- a condition that's
       still physically present re-latches before either call even
       returns, which is exactly what the re-check right below now
       genuinely observes, instead of unconditionally seeing the same
       stale "still latched" state every time. */
    HRTIM1_FaultClear();
    GateDriver_FaultClear();

    if ((HRTIM1_FaultIsTripped() != 0U) || (GateDriver_FaultIsLatched() != 0U))
    {
        return 0U;   /* still faulted -- stays in SM_STATE_FAULT */
    }

    g_state        = SM_STATE_IDLE;
    g_faultType    = SM_FAULT_NONE;
    g_faultChannel = 0xFFU;
    return 1U;
}
