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
 *              Always stops the legacy (pfm.c TABLE:STEP/FIRE) output
 *              path immediately, unconditionally (it isn't part of
 *              this state machine at all -- the legacy FIRE command
 *              never calls SM_Fire() -- so it has no ramp-down concept
 *              to preserve). The CURRENT (pid.c PID:*) output path's
 *              own stop is each fault type's OWN responsibility -- see
 *              the two handlers below, only one of which is still a
 *              pure stub. Left only via SM_ClearFault() (`FAULT:CLEAR`,
 *              existing command, extended) -- always back to IDLE,
 *              never directly to ARMED/FIRING; a fresh
 *              ARM+PID:PROFile:STARt is always required after any
 *              fault.
 *
 * TWO FAULT TYPES:
 *
 *   SM_FAULT_GENERAL       -- HandleGeneralFault() (state_machine.c).
 *                              POPULATED 2026-09-13, per direct
 *                              instruction: if the fault hit while
 *                              FIRING, every channel that was actively
 *                              outputting immediately begins an
 *                              OPEN-LOOP linear ramp-down to
 *                              PFM_TURNON_FREQ_HZ (0A) -- no PID/
 *                              feedback correction at all -- over
 *                              FAULT_RAMP_DOWN_TIME_S (ctrlr_config.h)
 *                              seconds, from wherever it actually was
 *                              in its own shot profile (see
 *                              PID_BeginFaultRampDown()/
 *                              ProcessFaultRampDown(), pid.c). Once
 *                              every participating channel reaches the
 *                              floor, its output is disconnected
 *                              (PID_Stop(), unconditional across every
 *                              channel) and the controller settles
 *                              into FAULT to wait for FAULT:CLEAR. If
 *                              the fault hit while IDLE/ARMED (nothing
 *                              was actually outputting), stops
 *                              immediately instead -- nothing to ramp.
 *   SM_FAULT_OVERCURRENT   -- HandleOvercurrentFault() (state_machine.c).
 *                              POPULATED 2026-09-15, per direct
 *                              instruction: unlike General Fault above,
 *                              this is a PER-CHANNEL condition, reported
 *                              for exactly one channel at a time via the
 *                              new SM_ReportOcpFault(channel) (below),
 *                              not detected by SM_PollFaults(). That
 *                              channel's own HRTIM output is disabled
 *                              IMMEDIATELY -- no ramp for it at all, the
 *                              opposite of General Fault's treatment of
 *                              every channel -- while every OTHER
 *                              currently-enabled channel is stepped down
 *                              by (100 * 1/N)% of its own current output
 *                              (N = channels enabled at the fault
 *                              instant, the faulted one included), THEN
 *                              ramps on down to PFM_TURNON_FREQ_HZ over
 *                              FAULT_RAMP_DOWN_TIME_S exactly like
 *                              General Fault -- see PID_BeginOvercurrentRampDown()'s
 *                              own extensive doc comment (pid.h) for the
 *                              full 3-step mechanism and a real, flagged
 *                              tension between "simultaneous" and the
 *                              existing hard slew-rate clamp that was
 *                              deliberately NOT bypassed for this. If the
 *                              fault hit while IDLE/ARMED, or the faulted
 *                              channel was the only one enabled, falls
 *                              through to the same "nothing to ramp, stop
 *                              immediately" handling General Fault
 *                              already has.
 *
 * *** NOTE TO REVISIT *** -- per direct instruction, "we will define the
 * mapping here at a later time": SM_ReportOcpFault(channel) is a real,
 * callable, fully-implemented entry point (and OCP:TEST:FAULT, commands.c,
 * exercises it end-to-end on real hardware without needing real pins --
 * see that command's own doc comment), but there is NO actual per-channel
 * OCP hardware detection wired up anywhere in this codebase yet -- no
 * GPIO/EXTI configuration, no pin table, nothing in main.c's
 * MX_GPIO_Init(). Deliberately NOT guessed/stubbed with placeholder pin
 * numbers (this codebase's own established convention -- see e.g.
 * pin_mapping_v4.csv-sourced decisions elsewhere -- is to work from real
 * pin data, never invent it) -- whatever real per-channel OCP pins this
 * board ends up with, wiring them is a self-contained follow-up: an EXTI
 * ISR (or a new poll-based per-channel source, mirroring gate_driver.c's
 * own shape, whichever the real hardware needs) that ultimately calls
 * SM_ReportOcpFault(channel) once the condition is detected -- none of
 * the logic built here needs to change to accommodate either shape.
 * Separately, both of this project's EXISTING fault sources (PC10/
 * HRTIM1_FLT6, GateDriverStatus_01..12) still route to SM_FAULT_GENERAL
 * unconditionally (SM_PollFaults()) -- nothing about this update changes
 * that; those two remain General Fault sources, not OCP ones, unless a
 * future decision says otherwise.
 *
 * ALSO NOTE TO REVISIT, added 2026-09-15: SM_ReportOcpFault() called a
 * SECOND time (a different channel) while already in SM_STATE_FAULT
 * (from either fault type) does NOT re-run the redistribute-and-ramp
 * sequence -- that only makes sense once, at the instant a fault is
 * first detected (it captures a fresh N and a fresh ramp-start snapshot
 * for whichever channels are still enabled at THAT moment). A second
 * report still ALWAYS immediately hard-disables its own channel
 * unconditionally (a real OCP condition must never be left connected
 * just because some other channel's fault got there first) -- but true
 * simultaneous/overlapping multi-channel OCP, and whether the survivors'
 * ramp should be recomputed mid-flight for a newly-reduced N, is
 * genuinely unresolved. The single-channel case this was actually
 * specified for (direct instruction: "channel 1's OCP fault pin") is
 * fully and correctly handled either way.
 *
 * ALSO NOTE TO REVISIT: SM_Arm()'s ArmConditionsMet() is a stub that
 * always allows arming (see its own comment) -- no real interlock
 * logic (fault-free, profile timing configured, sane gains, etc.)
 * exists yet. AND: FAULT_RAMP_DOWN_TIME_S (ctrlr_config.h, currently
 * 1.0s) is a first guess, not derived from any real Transrex/magnet
 * requirement -- see that macro's own comment.
 *
 * FIXED 2026-09-13 (same day, later still): g_state is read/written from
 * BOTH main.c's main loop AND PID_Update() (HRTIM1_Master_IRQn, priority
 * 1 -- can preempt the main loop at any instruction). This used to be a
 * genuine race: the main loop could read g_state (not yet FAULT), get
 * preempted by Master's own regular tick right there, that ISR's own
 * SM_PollFaults() call would detect the same real fault and complete
 * EnterFault() first (possibly starting a General-Fault ramp-down), and
 * THEN the main loop would resume and -- since ITS OWN check already
 * passed, before preemption -- call EnterFault() a second, redundant
 * time, which would see g_stateBeforeFault == SM_STATE_FAULT (not
 * SM_STATE_FIRING) and call PID_Stop() immediately, silently replacing
 * a just-started graceful ramp-down with an abrupt cutoff. This was
 * first (wrongly) judged "safety-neutral" here on the theory that the
 * physical end state ("output off") was unchanged either way -- direct
 * correction: the whole point of a linear ramp instead of an instant
 * stop is to avoid inductive/mechanical stress, so silently substituting
 * the abrupt path for the graceful one IS the hazard, not a neutral
 * outcome. Fixed properly, not just documented: SM_PollFaults() (.c)
 * now wraps its entire read-check-transition sequence -- including the
 * EnterFault() call itself -- in __disable_irq()/__enable_irq() (this
 * codebase's own established critical-section pattern, see boot_jump.c),
 * so whichever context (main loop or ISR) gets there first now runs the
 * whole check-and-transition to completion before the other context's
 * own call can even read g_state. This makes the double-entry race
 * structurally impossible, not merely unlikely -- see SM_PollFaults()'s
 * own comment in state_machine.c for the full reasoning.
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

/* The channel (0-based) an OCP fault was reported for -- valid
 * (meaningful) only when SM_GetFaultType() == SM_FAULT_OVERCURRENT.
 * Returns 0xFF (never a real channel index) otherwise, including for
 * SM_FAULT_GENERAL/SM_FAULT_NONE -- General Fault is system-wide, not
 * per-channel, so it has no single channel to report here. Added
 * 2026-09-15 alongside SM_ReportOcpFault(), below. */
uint8_t SM_GetFaultChannel(void);

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

/* Reports an OCP (per-channel overcurrent) fault for `channel` (0-based,
 * 0..HRTIM_NUM_CHANNELS-1) -- added 2026-09-15, per direct instruction.
 * Unlike SM_PollFaults() above (which POLLS two system-wide sources),
 * this is a direct REPORT -- the entry point whatever eventually detects
 * a real per-channel OCP condition (an EXTI ISR once real pins are
 * defined, see state_machine.h's own header comment "NOTE TO REVISIT";
 * OCP:TEST:FAULT, commands.c, meanwhile, for verifying this end-to-end
 * on real hardware without needing them) is meant to call, the instant
 * the condition is detected. `channel` out of range is a silent no-op
 * (defensive -- a caller with a real, wired-up pin should never produce
 * one).
 *
 * From IDLE/ARMED/FIRING: transitions straight to SM_STATE_FAULT with
 * SM_FAULT_OVERCURRENT, exactly like SM_PollFaults()'s own EnterFault()
 * path for SM_FAULT_GENERAL, then runs HandleOvercurrentFault()'s
 * channel-aware response -- see state_machine.h's own header comment and
 * PID_BeginOvercurrentRampDown()'s doc comment (pid.h) for the full
 * 3-step behavior (immediate disable for THIS channel, an immediate
 * proportional step-down for the others, then the same graceful ramp
 * General Fault uses).
 *
 * From SM_STATE_FAULT (already faulted, either type): does NOT re-run
 * that sequence -- see this header's own "ALSO NOTE TO REVISIT" comment
 * on why (it only makes sense once) -- but STILL always immediately
 * hard-disables `channel`'s own HRTIM output unconditionally, a cheap,
 * safe action regardless of what else is already in progress.
 *
 * Same __disable_irq()/__enable_irq() critical-section pattern as
 * SM_PollFaults() (state_machine.c), for the same reason: this may
 * eventually be called from ISR context (a real OCP pin's EXTI handler)
 * and needs to be safe against the same main-loop/ISR race that function
 * itself was fixed for. */
void SM_ReportOcpFault(uint8_t channel);

/* FAULT -> IDLE, ONLY if the underlying condition is actually gone --
 * backs FAULT:CLEAR (commands.c's existing cmd_fault_clear(), now also
 * calling this) alongside its existing HRTIM1_FaultClear()/
 * GateDriver_FaultClear() calls (both of which already re-validate
 * their own source before truly clearing -- see their own doc
 * comments). Returns 1 if now clear (state -> IDLE), 0 if still
 * faulted (state stays FAULT, matching GateDriver_FaultClear()'s own
 * "re-latches before this even returns" behavior for a still-present
 * condition) or if not currently in FAULT at all (nothing to clear).
 *
 * *** REAL GAP, added 2026-09-15, not silently left implicit ***: this
 * only re-validates HRTIM1_FaultIsTripped()/GateDriver_FaultIsLatched()
 * -- there is no third check for SM_FAULT_OVERCURRENT, because (see this
 * header's own "NOTE TO REVISIT" above) no real OCP hardware detection
 * exists yet to re-validate against. Concretely: an OCP fault reported
 * via SM_ReportOcpFault() will ALWAYS successfully clear on the very
 * next FAULT:CLEAR, unconditionally, regardless of whether a real,
 * future OCP condition on that channel is still physically present --
 * unlike General Fault, which genuinely re-latches immediately if its
 * sources are still bad. Once real per-channel OCP detection exists,
 * this function needs a matching re-check added alongside the two it
 * already has (mirroring GateDriver_FaultIsLatched()'s own shape,
 * whichever way OCP's real detection ends up presenting a "still
 * latched" query) -- do not assume this file's job here is finished
 * just because SM_ReportOcpFault()/the ramp-down behavior are. */
uint8_t SM_ClearFault(void);

#ifdef __cplusplus
}
#endif

#endif /* __STATE_MACHINE_H__ */
