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
#include "main.h"
#include "xrex_io.h"

static SM_State_t     g_state         = SM_STATE_IDLE;
static SM_FaultType_t g_faultType     = SM_FAULT_NONE;

/* External-enable interlock -- PF13, MOVED here 2026-09-17 from PF15
   (Fiber_Enable), per direct instruction: enable and trigger are now
   two independent physical signals on separate pins, not one shared
   wire. See this file's own header comment for the full design. OFF by
   default -- existing shots/tests are unaffected unless explicitly
   turned on. */
static uint8_t g_externalEnableRequired = 0U;

/* External trigger (rising edge on PF15 fires a shot while ARMED) --
   see this file's own header comment for the full design. OFF by
   default. UNCOUPLED from g_externalEnableRequired 2026-09-17, the
   same day enable moved to PF13 -- previously turning enable off also
   forced this off (SM_SetExternalEnableRequired()'s old comment), a
   rule that existed only because both features read the SAME physical
   wire; now that they're on separate pins, per direct instruction, the
   two config flags are fully independent. The real safety guarantee is
   unaffected either way: SM_Fire() itself (below) still unconditionally
   re-checks SM_ExternalEnableOk() before ever actually firing,
   regardless of how or when trigger was turned on. */
static uint8_t g_externalTriggerRequired = 0U;

/* Baseline PF15 level for edge detection, meaningful only while
   g_state == SM_STATE_ARMED and g_externalTriggerRequired != 0 --
   captured fresh by SM_Arm() the instant it succeeds, then kept
   current by SM_PollFaults() on every call while still ARMED. See the
   external-trigger design comment (state_machine.h) for why a fresh
   capture at arm-time (not a stale value from a previous cycle)
   matters. RENAMED (not just re-read) 2026-09-17 from
   g_lastExternalEnableLevelWhileArmed -- this was always tracking
   whichever physical pin trigger's edge detection reads, which just
   happened to be the same pin the enable interlock also used before
   today; now that enable and trigger are genuinely different pins,
   the old name would actively mislead (it would sound like it tracks
   PF13, the new enable pin, when it has always meant -- and still
   means -- the trigger pin specifically). */
static uint8_t g_lastExternalTriggerLevelWhileArmed = 0U;

/* GPIO_PULLDOWN in main.c's MX_GPIO_Init() means an unconnected/
   floating PF13 reads LOW here -- the deliberate fail-safe default,
   see this file's header comment. MOVED from PF15 2026-09-17, same
   change as the static above. */
static uint8_t ExternalEnableInputIsHigh(void)
{
    return (HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_13) == GPIO_PIN_SET) ? 1U : 0U;
}

/* PF15 (Fiber_Enable) -- trigger-only as of 2026-09-17 (previously this
   pin backed BOTH ExternalEnableInputIsHigh() above and this function,
   since enable and trigger were the same wire). GPIO_PULLDOWN in
   main.c means an unconnected/floating PF15 reads LOW -- no spurious
   rising edge from a disconnected trigger wire (see main.c's own
   comment on this pin for the fuller reasoning). */
static uint8_t ExternalTriggerInputIsHigh(void)
{
    return (HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_15) == GPIO_PIN_SET) ? 1U : 0U;
}

/* Emergency stop (PG10, a fiber-optic input -- NOT PF15) -- see this
   file's own header comment for the full design. OFF by default --
   "acts as though it does not exist" when off, per direct instruction. */
static uint8_t g_emergencyStopRequired = 0U;

/* Pure raw read -- 1 if PG10 is electrically HIGH right now, 0 if LOW.
   Deliberately polarity-agnostic (matches ExternalEnableInputIsHigh()'s
   own convention just above, and SM_GetEmergencyStopInputRaw()'s own
   doc comment in state_machine.h) -- what this LEVEL actually means
   ("asserted" vs "OK") is EMERGENCY_STOP_POLARITY's job
   (ctrlr_config.h), applied by EmergencyStopAsserted() below, not this
   function. GPIO_PULLDOWN in main.c's MX_GPIO_Init() means an
   unconnected/floating PG10 reads LOW here. */
static uint8_t EmergencyStopInputIsHigh(void)
{
    return (HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_10) == GPIO_PIN_SET) ? 1U : 0U;
}

/* Applies EMERGENCY_STOP_POLARITY (ctrlr_config.h) to the raw PG10
   level -- 1 if this represents the E-stop being ASSERTED right now,
   0 if OK. INVERTED 2026-09-17, later the same day it was first added:
   a hardware inverter was placed between the fiber-optic receiver and
   PG10, so EMERGENCY_STOP_POLARITY now defaults NORMALLY_LOW ("no
   input" -- including a genuinely floating pin -- reads as OK, a HIGH
   reading is the fault), the OPPOSITE of this feature's original
   NORMALLY_HIGH assumption. See ctrlr_config.h's own extensive comment
   on EMERGENCY_STOP_POLARITY for the full reasoning and the safety
   trade-off this reversal carries. Same one-line shape as xrex_io.c's
   own BitIsFault() helper -- not shared/reused across modules (this
   file has no xrex_io.h dependency), matching this file's own existing
   per-concern-local-helper style (ExternalEnableInputIsHigh(),
   EmergencyStopInputIsHigh(), just above). */
static uint8_t EmergencyStopAsserted(void)
{
    uint8_t bit = EmergencyStopInputIsHigh();
    return (EMERGENCY_STOP_POLARITY == FAULT_POLARITY_NORMALLY_HIGH) ? (bit == 0U) : (bit != 0U);
}

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

/* Called on entering SM_STATE_FAULT with g_faultType ==
   SM_FAULT_EMERGENCY_STOP -- added 2026-09-17, per direct instruction.
   Deliberately NOT branching on g_stateBeforeFault at all, unlike
   every other handler above -- an emergency stop means an immediate,
   unconditional hard cutoff regardless of whether the fault hit while
   FIRING or not; there is no ramp path for this fault type under any
   circumstance. PID_Stop() itself is already idempotent/safe to call
   when nothing was actually running (see its own comment, pid.c), so
   no separate "nothing to stop" branch is needed here either -- this
   is intentionally the simplest handler in this file. */
static void HandleEmergencyStopFault(void)
{
    PID_Stop();
}

/* Readiness gate for SM_Arm() -- STUB, per direct instruction: always
   allows arming today. Real interlock conditions (no fault active --
   already implicitly guaranteed, since IDLE and FAULT are mutually
   exclusive states, so nothing further needed for that one
   specifically; profile timing actually configured; at least one
   channel enabled; gains sane; whatever else turns out to matter) are
   explicitly NOT implemented yet -- populate later.

   ONE real condition now populated, 2026-09-16: the external-enable
   interlock (SM_ExternalEnableOk(), see this file's header comment) --
   still an opt-in, OFF-by-default check, but no longer purely a stub
   for this specific case. */
static uint8_t ArmConditionsMet(void)
{
    if (SM_ExternalEnableOk() == 0U)
    {
        return 0U;
    }
    /* Enable/contactor output precondition, added 2026-09-17 -- per
       direct instruction, every currently-enabled channel's own
       ENA_OUT/CONTACT_OUT must both be commanded ON before ARM
       succeeds. Pure check, no side effects, gated per-channel inside
       XrexIo_EnableOutputsReadyToArm() itself -- see state_machine.h's
       own enable-output section for the full design. */
    if (XrexIo_EnableOutputsReadyToArm() == 0U)
    {
        return 0U;
    }
    return 1U;   /* TODO (2026-09-13): populate the rest. */
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
        case SM_FAULT_ENABLE_OUTPUT:   /* identical response to OVERCURRENT
                                            (immediate hard-disable of THIS
                                            channel, survivors derated 1/N
                                            and ramped) -- see
                                            state_machine.h's own
                                            SM_FAULT_ENABLE_OUTPUT header
                                            comment for why; only the
                                            reported TYPE differs, same
                                            "distinct type, shared
                                            response" pattern
                                            SM_FAULT_EXTERNAL_ENABLE uses
                                            for HandleGeneralFault() below */
            HandleOvercurrentFault(channel);
            break;
        case SM_FAULT_EMERGENCY_STOP:   /* deliberately its OWN handler, NOT
                                            grouped with GENERAL below -- see
                                            HandleEmergencyStopFault()'s own
                                            comment: no ramp, ever, unlike
                                            every other fault type here */
            HandleEmergencyStopFault();
            break;
        case SM_FAULT_GENERAL:
        case SM_FAULT_EXTERNAL_ENABLE:   /* identical response to GENERAL,
                                             see state_machine.h's own
                                             external-enable section for why --
                                             only the reported TYPE differs */
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

    /* External trigger's edge-detection baseline, added 2026-09-16 --
       captured fresh here regardless of whether the trigger feature is
       even on right now (cheap, and avoids a stale value if it gets
       turned on later while already ARMED) -- see the external-trigger
       design comment (state_machine.h) for why arm-time capture
       matters: a signal already HIGH at the moment of arming must NOT
       look like a rising edge on the next poll. Reads PF15 via
       ExternalTriggerInputIsHigh() -- MOVED 2026-09-17 off
       ExternalEnableInputIsHigh() (now PF13) now that trigger and
       enable are separate pins. */
    g_lastExternalTriggerLevelWhileArmed = ExternalTriggerInputIsHigh();

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

    /* Redundant last-line-of-defense re-check, 2026-09-16 -- the
       command layer (cmd_pid_profile_start(), commands.c) already
       checks SM_ExternalEnableOk() itself first, for a precise error
       message; this catches PF13 (the enable pin, MOVED here 2026-09-17
       from PF15) dropping in the narrow window between that check and
       this call, or any future caller that reaches SM_Fire() directly
       without going through that command handler -- including the
       external-trigger feature's own direct SM_Fire() call, below.
       Returns the same generic 0 as the PID_ProfileStart() failure
       below -- the command layer's own upfront check is what's
       responsible for distinguishing the two reasons for an operator,
       not this function. */
    if (SM_ExternalEnableOk() == 0U)
    {
        return 0U;   /* stays ARMED */
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
    /* Emergency stop (PG10), added 2026-09-17 -- checked across ALL
       states (IDLE/ARMED/FIRING), same as the GENERAL check just
       above, NOT the FIRING-only carve-out external-enable below uses
       -- see state_machine.h's own emergency-stop section for why.
       Per direct instruction ("acts as though it does not exist" when
       off), the PG10 read itself is gated behind
       g_emergencyStopRequired first -- genuinely zero work, not just a
       zero-effect check, when the feature is off. Uses
       EmergencyStopAsserted() (EMERGENCY_STOP_POLARITY-aware), not a
       raw HIGH/LOW comparison -- see that helper's own comment for the
       2026-09-17 inverter-driven polarity reversal. */
    else if ((g_emergencyStopRequired != 0U) && (EmergencyStopAsserted() != 0U))
    {
        EnterFault(SM_FAULT_EMERGENCY_STOP, 0xFFU);   /* channel N/A, system-wide */
    }
    /* External-enable interlock (PF13, MOVED 2026-09-17 from PF15),
       added 2026-09-16 -- deliberately gated on SM_STATE_FIRING
       specifically, not IDLE/ARMED too. See state_machine.h's own
       external-enable section for the full reasoning (this is the ONLY
       continuous-monitoring gate point for this interlock; ARM/
       PID:PROFile:STARt cover IDLE/ARMED via their own one-shot checks
       instead). g_state re-read fresh here (not cached from above) so
       this naturally doesn't double-enter if the GENERAL check above
       already transitioned to FAULT this same call. */
    else if ((g_externalEnableRequired != 0U) && (g_state == SM_STATE_FIRING) &&
             (ExternalEnableInputIsHigh() == 0U))
    {
        EnterFault(SM_FAULT_EXTERNAL_ENABLE, 0xFFU);   /* channel N/A, system-wide */
    }

    /* External trigger (PF15), added 2026-09-16 -- see state_machine.h's
       own design comment for the full reasoning. Independent of the
       fault checks above (mutually exclusive in practice: this only
       ever looks at SM_STATE_ARMED, which neither fault branch above
       can leave g_state in). SM_Fire()'s own return value is
       deliberately ignored here -- if it fails (e.g. profile timing
       never set, or the enable interlock -- now PF13 -- isn't
       currently satisfied; SM_Fire() itself still unconditionally
       re-checks SM_ExternalEnableOk() regardless of how trigger got
       turned on, see this file's own SM_Fire() comment), the state
       simply stays ARMED with the baseline now recording HIGH, so this
       genuinely is edge-triggered (needs a fresh falling-then-rising
       transition to try again), not level-triggered (which would
       otherwise retry every single poll for as long as PF15 stayed
       HIGH). Reads ExternalTriggerInputIsHigh() (PF15) -- MOVED
       2026-09-17 off ExternalEnableInputIsHigh() (now PF13), same day
       enable and trigger split onto separate pins. */
    if ((g_externalTriggerRequired != 0U) && (g_state == SM_STATE_ARMED))
    {
        uint8_t level = ExternalTriggerInputIsHigh();
        if ((g_lastExternalTriggerLevelWhileArmed == 0U) && (level != 0U))
        {
            (void)SM_Fire();
        }
        g_lastExternalTriggerLevelWhileArmed = level;
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

void SM_ReportEnableOutputFault(uint8_t channel)
{
    if (channel >= HRTIM_NUM_CHANNELS)
    {
        return;   /* defensive -- a real, correctly-wired caller never
                      produces this */
    }

    /* Same critical-section reasoning as SM_ReportOcpFault() just above --
       this is called from XrexIo_PollEnableOutputFaults() (xrex_io.c) at
       the same tick cadence OCP polling gets, so it needs the same
       protection against the same main-loop/PID_Update()-tick race. */
    __disable_irq();

    if (g_state == SM_STATE_FAULT)
    {
        /* Already faulted (any type) -- same reasoning as
           SM_ReportOcpFault() above: a channel with its own
           ENA_OUT/CONTACT_OUT no longer both HIGH must never be left
           connected just because some other channel's (or this same
           channel's own, via a different fault type) condition got here
           first. */
        (void)PID_SetChannelEnable(channel, 0U);
        __enable_irq();
        return;
    }

    EnterFault(SM_FAULT_ENABLE_OUTPUT, channel);
    __enable_irq();
}

void SM_ReportGeneralFault(void)
{
    /* Same critical-section reasoning as SM_PollFaults()/SM_ReportOcpFault()
       above -- see SM_PollFaults()'s own extensive comment. */
    __disable_irq();

    if (g_state == SM_STATE_FAULT)
    {
        /* Already faulted -- matches SM_PollFaults()'s own "already
           latched -- nothing new to do" behavior. Unlike
           SM_ReportOcpFault(), there is no per-channel action to take
           here even redundantly -- General Fault has no specific
           channel target. */
        __enable_irq();
        return;
    }

    EnterFault(SM_FAULT_GENERAL, 0xFFU);   /* channel N/A for a system-wide fault,
                                                same sentinel SM_PollFaults() uses */
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
    /* External-enable interlock, added 2026-09-16 -- same "only clear if
       the condition is actually gone" philosophy as the two checks just
       above. Only actually gates anything when the interlock is turned
       on -- SM_ExternalEnableOk() itself already returns 1 unconditionally
       when g_externalEnableRequired is 0, so this is a no-op re-check
       (never blocks clearing) whenever the feature isn't in use. */
    if (SM_ExternalEnableOk() == 0U)
    {
        return 0U;   /* still faulted -- stays in SM_STATE_FAULT */
    }
    /* Emergency stop, added 2026-09-17 -- same philosophy again.
       SM_EmergencyStopOk() itself already returns 1 unconditionally
       when g_emergencyStopRequired is 0, so this is a no-op re-check
       (never blocks clearing, never reads PG10) whenever the feature
       isn't in use -- matching "acts as though it does not exist" when
       off. */
    if (SM_EmergencyStopOk() == 0U)
    {
        return 0U;   /* still faulted -- stays in SM_STATE_FAULT */
    }

    g_state        = SM_STATE_IDLE;
    g_faultType    = SM_FAULT_NONE;
    g_faultChannel = 0xFFU;
    return 1U;
}

void SM_SetExternalEnableRequired(uint8_t required)
{
    /* UNCOUPLED from g_externalTriggerRequired 2026-09-17, same day
       enable (PF13) and trigger (PF15) split onto separate pins -- see
       g_externalTriggerRequired's own comment above for why the old
       "turning this off forces trigger off too" rule no longer applies
       now that they're independent physical signals. */
    g_externalEnableRequired = (required != 0U) ? 1U : 0U;
}

uint8_t SM_GetExternalEnableRequired(void)
{
    return g_externalEnableRequired;
}

uint8_t SM_ExternalEnableOk(void)
{
    if (g_externalEnableRequired == 0U)
    {
        return 1U;   /* interlock not in use -- always "ok" */
    }
    return ExternalEnableInputIsHigh();
}

uint8_t SM_GetExternalEnableInputRaw(void)
{
    return ExternalEnableInputIsHigh();
}

uint8_t SM_SetExternalTriggerRequired(uint8_t required)
{
    /* No longer gated on g_externalEnableRequired -- UNCOUPLED
       2026-09-17, per direct instruction, now that enable (PF13) and
       trigger (PF15) are separate pins. Always succeeds now; kept a
       uint8_t return (rather than void) purely so the command layer
       (cmd_ext_trigger(), commands.c) doesn't need its own signature
       change for what's now an unconditional operation. */
    g_externalTriggerRequired = (required != 0U) ? 1U : 0U;
    return 1U;
}

uint8_t SM_GetExternalTriggerRequired(void)
{
    return g_externalTriggerRequired;
}

uint8_t SM_GetExternalTriggerInputRaw(void)
{
    return ExternalTriggerInputIsHigh();
}

void SM_SetEmergencyStopRequired(uint8_t required)
{
    g_emergencyStopRequired = (required != 0U) ? 1U : 0U;
}

uint8_t SM_GetEmergencyStopRequired(void)
{
    return g_emergencyStopRequired;
}

uint8_t SM_EmergencyStopOk(void)
{
    if (g_emergencyStopRequired == 0U)
    {
        return 1U;   /* feature not in use -- always "ok", and never reads PG10 */
    }
    return (EmergencyStopAsserted() == 0U) ? 1U : 0U;
}

uint8_t SM_GetEmergencyStopInputRaw(void)
{
    return EmergencyStopInputIsHigh();
}
