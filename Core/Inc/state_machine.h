#ifndef __STATE_MACHINE_H__
#define __STATE_MACHINE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* --------------------------------------------------------------------------
 * state_machine.c
 *
 * Added 2026-09-13, per direct request: an explicit top-level
 * operating-state machine for this controller, sitting above pid.c
 * (the control loop) / hrtim.c (the hardware) / gate_driver.c (one
 * fault source) / pfm.c (the legacy table-based output path) -- the
 * thing that ties them together into one coherent "what is this
 * controller doing right now" answer, queryable over the wire
 * (STATE?) and enforced at the command layer (PID:PROFile:STARt now
 * refuses to do anything unless the machine is actually ARMED).
 *
 * FOUR STATES:
 *
 *   IDLE    -- the normal state: entered at boot (SM_Init()), and
 *              returned to after a shot completes (SM_NotifyShotComplete(),
 *              called by pid.c) or a fault is cleared (SM_ClearFault()).
 *              Every HRTIM channel output is disabled in this state --
 *              nothing outputs a PFM waveform. Also reachable from
 *              ARMED/FIRING via SM_Stop() (an operator abort, PID:STOP)
 *              or SM_Disarm() (stand down without firing).
 *
 *   ARMED   -- a readiness gate between IDLE and FIRING, added per
 *              direct request/design discussion (2026-09-13): outputs
 *              are still disabled here, exactly like IDLE -- nothing
 *              electrical changes on entry. What ARMED buys is a
 *              deliberate, separately-confirmable "ready to fire" step
 *              before any real output begins, gated by
 *              ArmConditionsMet() (currently a STUB, see its own
 *              comment below -- always allows arming today). Entered
 *              via SM_Arm() (the `ARM` command); left via SM_Fire()
 *              (`PID:PROFile:STARt`, which only succeeds from here) or
 *              SM_Disarm() (`DISARM`, stand down) or SM_Stop()
 *              (`PID:STOP`, abort).
 *
 *   FIRING  -- entered only from ARMED, via SM_Fire(). Every currently-
 *              ENABLED channel (PID:CHANnel:ENAble -- a separate,
 *              already-existing per-channel concern, orthogonal to
 *              this system-wide state) gets its HRTIM output connected
 *              and begins the programmed ramp profile
 *              (PID_ProfileStart(), pid.c, unchanged). Returns to IDLE
 *              automatically when the shot completes
 *              (SM_NotifyShotComplete(), called from PID_Update()'s
 *              own existing shot-completion check), or on a manual
 *              SM_Stop() (`PID:STOP`) abort.
 *
 *   FAULT   -- entered from ANY other state the instant a fault
 *              condition is detected (SM_PollFaults(), see its own
 *              comment for exactly when this runs) -- per direct
 *              instruction, "no matter which state the supply is in".
 *              Forces a full stop across both the current (pid.c) and
 *              legacy (pfm.c) output paths regardless of which was
 *              actually active, then dispatches to one of two STUB
 *              fault-type handlers (see below). Left only via
 *              SM_ClearFault() (`FAULT:CLEAR`, existing command,
 *              extended) -- always back to IDLE, never directly to
 *              ARMED/FIRING; a fresh ARM+PID:PROFile:STARt is always
 *              required after any fault.
 *
 * TWO FAULT TYPES, STUBBED, NOT YET ROUTED -- per direct instruction,
 * write empty handler functions now, decide the real behavior later:
 *
 *   SM_FAULT_GENERAL       -- HandleGeneralFault() (state_machine.c),
 *                              currently empty.
 *   SM_FAULT_OVERCURRENT   -- HandleOvercurrentFault() (state_machine.c),
 *                              currently empty.
 *
 * *** NOTE TO REVISIT *** -- per direct instruction, both of this
 * project's existing fault sources (PC10/HRTIM1_FLT6 hardware input,
 * GateDriverStatus_01..12 EXTI) currently route to SM_FAULT_GENERAL
 * unconditionally (SM_PollFaults(), state_machine.c) -- there is NO
 * overcurrent-specific detection mechanism wired up anywhere in this
 * codebase yet, and no decision has been made about which real-world
 * condition(s) should ever produce SM_FAULT_OVERCURRENT instead of
 * SM_FAULT_GENERAL. Both stub handlers are empty either way today, so
 * this routing has zero behavioral effect right now -- it will matter
 * once real handling logic is written into either stub. Revisit both
 * the routing decision and the handler bodies together.
 *
 * ALSO NOTE TO REVISIT: SM_Arm()'s ArmConditionsMet() is a stub that
 * always allows arming (see its own comment) -- no real interlock
 * logic (fault-free, profile timing configured, sane gains, etc.)
 * exists yet.
 * -------------------------------------------------------------------------- */

typedef enum
{
    SM_STATE_IDLE = 0,
    SM_STATE_ARMED,
    SM_STATE_FIRING,
    SM_STATE_FAULT
} SM_State_t;

typedef enum
{
    SM_FAULT_NONE = 0,     /* meaningful only via SM_GetFaultType() when
                               SM_GetState() != SM_STATE_FAULT */
    SM_FAULT_GENERAL,
    SM_FAULT_OVERCURRENT
} SM_FaultType_t;

/* Called once at boot (main.c), after pid.c's own PID_Init(). Sets the
 * state to IDLE. Does not touch any hardware itself -- pid.c/hrtim.c's
 * own init already leaves outputs off; this just makes that the
 * explicit, queryable state from the start. */
void SM_Init(void);

SM_State_t SM_GetState(void);

/* Valid (meaningful) only while SM_GetState() == SM_STATE_FAULT --
 * returns SM_FAULT_NONE otherwise. */
SM_FaultType_t SM_GetFaultType(void);

/* IDLE -> ARMED. Backs the `ARM` command (commands.c). Returns 1 on
 * success, 0 if the current state isn't IDLE (already ARMED/FIRING/
 * FAULT) or if ArmConditionsMet() (state_machine.c, currently a stub
 * that always returns 1) refuses. */
uint8_t SM_Arm(void);

/* ARMED -> IDLE, WITHOUT firing -- stand down. Backs the `DISARM`
 * command (commands.c) -- added alongside SM_Arm() as its natural
 * companion (arming is reversible without requiring either a fire or
 * a fault); flag if this isn't wanted. No-op (not an error) if not
 * currently ARMED. */
void SM_Disarm(void);

/* ARMED -> FIRING. Backs PID:PROFile:STARt (commands.c's
 * cmd_pid_profile_start(), now gated on this) -- calls
 * PID_ProfileStart() (pid.c, unchanged) internally and only actually
 * transitions to FIRING if that succeeds. Returns 1 on success, 0 if
 * the current state isn't ARMED, or if PID_ProfileStart() itself
 * fails (e.g. profile timing was never set) -- state stays ARMED in
 * that case, not FIRING. */
uint8_t SM_Fire(void);

/* ARMED or FIRING -> IDLE, a manual abort -- backs PID:STOP
 * (commands.c's cmd_pid_stop(), now also calling this) alongside its
 * existing PID_Stop() call. No-op (not an error) if already IDLE, and
 * deliberately does NOT clear FAULT (see SM_ClearFault() for that --
 * a plain STOP must never be a backdoor out of a real fault latch). */
void SM_Stop(void);

/* FIRING -> IDLE, called by pid.c's PID_Update() at the exact point a
 * profile naturally completes (the shared shot clock reaching its
 * total duration) -- the automatic, no-fault, no-operator-action shot
 * end. Do not call this for a manual abort -- that's SM_Stop(). */
void SM_NotifyShotComplete(void);

/* Checks BOTH existing fault sources (HRTIM1_FaultIsTripped(),
 * GateDriver_FaultIsLatched()) and transitions IDLE/ARMED/FIRING ->
 * FAULT the instant either is set -- per direct instruction, "no
 * matter which state the supply is in". No-op if already in FAULT
 * (nothing new to latch) or if neither source is tripped.
 *
 * Called from TWO places, deliberately, so detection genuinely doesn't
 * depend on what state the machine is in:
 *   1. main.c's main loop, every iteration, unconditionally -- covers
 *      IDLE/ARMED, where the HRTIM Master counter (and so pid.c's own
 *      per-tick PID_Update()) isn't even running yet, so nothing else
 *      would ever notice a fault at all before an operator tried to
 *      fire into one.
 *   2. pid.c's PID_Update(), once per Master tick, for FIRING -- the
 *      same call, just also reached at the higher, deterministic rate
 *      an active shot already runs at, rather than waiting for the
 *      main loop's own (currently unbounded, uart_process()-gated)
 *      iteration cadence.
 * *** NOTE TO REVISIT ***: the main-loop polling rate is whatever the
 * loop's own cadence happens to be (currently just uart_process() next
 * to it) -- not a bounded, guaranteed interval. Fine for now (both
 * underlying fault sources already protect themselves independently
 * of this -- PC10 in silicon, GateDriverStatus via its own EXTI edge
 * regardless of when this function next runs), but worth a harder
 * look (a dedicated periodic timer?) if SM_GetState() itself needs a
 * tighter latency guarantee later. */
void SM_PollFaults(void);

/* FAULT -> IDLE, ONLY if the underlying condition is actually gone --
 * backs FAULT:CLEAR (commands.c's existing cmd_fault_clear(), now also
 * calling this) alongside its existing HRTIM1_FaultClear()/
 * GateDriver_FaultClear() calls (both of which already re-validate
 * their own source before truly clearing -- see their own doc
 * comments). Returns 1 if now clear (state -> IDLE), 0 if still
 * faulted (state stays FAULT, matching GateDriver_FaultClear()'s own
 * "re-latches before this even returns" behavior for a still-present
 * condition) or if not currently in FAULT at all (nothing to clear). */
uint8_t SM_ClearFault(void);

#ifdef __cplusplus
}
#endif

#endif /* __STATE_MACHINE_H__ */
