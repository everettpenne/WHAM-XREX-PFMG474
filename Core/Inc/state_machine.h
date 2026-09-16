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
 *                              full 3-step mechanism. The (100*1/N)% step
 *                              itself deliberately BYPASSES the hard
 *                              slew-rate clamp used by every other output
 *                              write in this codebase (decided 2026-09-15
 *                              after real DSLogic data showed the clamped
 *                              version never actually reached the literal
 *                              percentage) -- see ClampOutputRangeOnly()'s
 *                              comment in pid.c for the full justification.
 *                              If the fault hit while IDLE/ARMED, or the faulted
 *                              channel was the only one enabled, falls
 *                              through to the same "nothing to ramp, stop
 *                              immediately" handling General Fault
 *                              already has.
 *
 *   SM_FAULT_EXTERNAL_ENABLE -- ADDED 2026-09-16, per direct request: a
 *                              real, physical operator/facility
 *                              permissive signal (PF15, "Fiber_Enable"),
 *                              not a hardware fault condition in the
 *                              PC10/GateDriverStatus sense. System-wide,
 *                              same shape as General Fault -- reuses
 *                              HandleGeneralFault() directly (EnterFault()'s
 *                              switch) for an IDENTICAL open-loop
 *                              ramp-down response, reported as its own
 *                              distinct type purely so an operator can
 *                              tell "the interlock dropped" apart from
 *                              "a real HRTIM/gate-driver fault" via
 *                              STATE?. See the external-enable section further
 *                              down this file for the full design
 *                              (opt-in, ARM/PID:PROFile:STARt gating,
 *                              FIRING-only continuous monitoring,
 *                              FAULT:CLEAR re-validation, the deliberate
 *                              GPIO_PULLDOWN fail-safe choice).
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
    SM_FAULT_OVERCURRENT,
    SM_FAULT_EXTERNAL_ENABLE   /* added 2026-09-16 -- see the external-enable
                                   section below and this file's own
                                   header comment for the full design */
} SM_FaultType_t;

/* Called once at boot (main.c), after pid.c's own PID_Init() AND after
 * main.c's own MX_GPIO_Init() (which now also configures PF15,
 * Fiber_Enable -- see the external-enable interlock's own doc comment below). Sets the
 * state to IDLE. Does not touch any hardware itself -- pid.c/hrtim.c's
 * own init already leaves outputs off, and PF15's GPIO config lives in
 * main.c's MX_GPIO_Init() (matching gate_driver.c's own precedent for
 * plain-input pins: GPIO config in main.c, the read/fault logic in the
 * owning module) -- this just makes IDLE the explicit, queryable state
 * from the start. */
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

/* Reports a General (system-wide) fault directly -- added 2026-09-15,
 * per direct request for a software way to test the General Fault
 * ramp-down (a shot in progress, an EARLY fault -- e.g. mid-ramp-up,
 * before any channel has reached its plateau -- not just the
 * steady-state/flat-top case OCP:TEST:FAULT has been exercised
 * against so far). Backs the new GENERAL:TEST:FAULT command
 * (commands.c), same software-fault-injection precedent as
 * OCP:TEST:FAULT/SM_ReportOcpFault() above -- there is still no real
 * software path to this otherwise: SM_PollFaults() only ever reaches
 * SM_FAULT_GENERAL by POLLING the two real hardware sources (PC10/
 * HRTIM1_FLT6, GateDriverStatus), and neither can be triggered from a
 * SCPI command.
 *
 * Unlike SM_ReportOcpFault(), takes no channel argument -- General
 * Fault is system-wide by design, exactly like SM_PollFaults()'s own
 * EnterFault(SM_FAULT_GENERAL, 0xFFU) call. A no-op if already
 * SM_STATE_FAULT (matches SM_PollFaults()'s own "already latched --
 * nothing new to do" behavior; there is no OCP-style "still hard-
 * disable something" fallback needed here since General Fault has no
 * per-channel target). Same __disable_irq()/__enable_irq() critical-
 * section pattern as SM_PollFaults()/SM_ReportOcpFault(), for the same
 * main-loop/ISR race protection. */
void SM_ReportGeneralFault(void);

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

/* --------------------------------------------------------------------------
 * External-enable interlock (PF15, "Fiber_Enable" -- docs/pin_mapping_v4.csv),
 * added 2026-09-16, per direct request.
 *
 * A pin the operator reads, not drives -- confirmed GPI in the pin
 * mapping doc (PC14, the pin first proposed for this, was rejected:
 * it's documented GPO there, "STM_Enable_Pin" -- the STM32 DRIVES that
 * one outward, the opposite direction needed here; PF15 is the correct
 * pin). Opt-in: OFF by default at boot (SM_SetExternalEnableRequired()
 * was never called, or was last called with 0) -- existing shots/tests
 * are completely unaffected unless this is explicitly turned on.
 *
 * When ON:
 *   1. `ARM` refuses (ArmConditionsMet(), state_machine.c) unless PF15
 *      currently reads HIGH -- folded into that function's existing
 *      generic "arm conditions not met" failure (ERR 13), no new error
 *      code needed for this case.
 *   2. `PID:PROFile:STARt` ALSO re-checks PF15 immediately before
 *      firing (SM_ExternalEnableOk(), below, checked by the command
 *      handler for a precise error -- see cmd_pid_profile_start()'s own
 *      comment -- AND redundantly inside SM_Fire() itself as a last-
 *      line-of-defense, in case some other future caller ever bypasses
 *      the command handler's own check). Closes the real gap where PF15
 *      could drop in the window between a successful ARM and the
 *      eventual START -- ARM alone is NOT enough to guarantee this.
 *   3. While FIRING, SM_PollFaults() (below) ALSO checks PF15 every
 *      time it runs (the main loop, AND every real PID_Update() tick --
 *      the same cadence General Fault's own two hardware sources
 *      already get). If PF15 reads LOW while FIRING, enters
 *      SM_STATE_FAULT with SM_FAULT_EXTERNAL_ENABLE -- identical
 *      open-loop ramp-down response to General Fault (HandleGeneralFault()
 *      is reused directly, see EnterFault()'s switch, state_machine.c)
 *      -- reported as a DISTINCT fault type purely for operator
 *      diagnostics (STATE? -> "OK FAULT EXTERNAL_ENABLE"), not a
 *      different physical response. Per direct instruction, this is
 *      checked only while actually FIRING ("if that input is lost
 *      during a shot") -- losing PF15 while merely ARMED (nothing
 *      outputting yet) is NOT itself treated as a fault here; the next
 *      PID:PROFile:STARt attempt will simply fail its own re-check
 *      (item 2 above) instead. Deliberately not decided either way
 *      whether ARMED should also actively fault on PF15 loss -- not
 *      asked for, flagged rather than silently added.
 *   4. `FAULT:CLEAR` (SM_ClearFault(), above) re-validates PF15 is back
 *      HIGH before actually clearing an EXTERNAL_ENABLE fault, same
 *      "only clear if the condition is actually gone" philosophy
 *      already applied to PC10/GateDriverStatus.
 *
 * GPIO_PULLDOWN (main.c's MX_GPIO_Init(), not this project's usual
 * GPIO_NOPULL for actively-driven inputs) is a DELIBERATE fail-safe
 * choice specific to this one signal: an unconnected/floating PF15
 * must read LOW (no permission), never an undefined level that could
 * accidentally read HIGH and silently permit firing. GateDriverStatus's
 * own NOPULL is fine for that pin because floating-read-as-fault is
 * already the safe direction there; the same reasoning would be UNSAFE
 * here, where floating-read-as-enabled would be the dangerous one.
 *
 * *** As a FAULT source, NOT re-checked while IDLE or ARMED other than
 * at the two explicit gate points above (ARM, PID:PROFile:STARt) --
 * only genuinely continuously monitored for FAULT purposes while
 * FIRING. If tighter, always-on FAULT monitoring is ever wanted,
 * that's a real, separate decision to make explicitly, not something
 * to assume was already covered here. (PF15's raw LEVEL is, separately,
 * read continuously while ARMED too, as of 2026-09-16 -- see the
 * external-trigger feature directly below -- but a drop while ARMED
 * still does not itself enter FAULT; only the trigger-edge logic
 * reacts to it there.) *** */

/* Turns the interlock above on (1) or off (0). Persists across shots
 * until changed again or the board reboots (RAM state, like every
 * other runtime config in this project -- PID:PROFILE:CURRENT,
 * PID:LOOPMODE, etc.) -- NOT saved to non-volatile storage. Backs
 * `EXTernal:ENAble <0|1>` (commands.c). Takes effect immediately: if turned
 * ON while already FIRING, the very next SM_PollFaults() call (at most
 * one PID_Update() tick away) starts checking PF15.
 *
 * Turning this OFF also forces the external-trigger feature (below)
 * off, if it was on -- added 2026-09-16, when that feature was built:
 * triggering depends structurally on this same interlock (its own
 * "lose the signal mid-shot -> fault" protection IS this mechanism),
 * so an operator can never end up with triggering enabled while the
 * enable check itself is silently off. See SM_SetExternalTriggerRequired()
 * below for the matching one-way dependency enforced when turning
 * triggering ON. */
void SM_SetExternalEnableRequired(uint8_t required);

/* Current mode, as last set by SM_SetExternalEnableRequired() (0 by
 * default at boot). Backs `EXTernal:ENAble?` (commands.c). */
uint8_t SM_GetExternalEnableRequired(void);

/* 1 if the interlock isn't required at all (SM_GetExternalEnableRequired()
 * == 0 -- always "ok" in that case, matching this feature's opt-in
 * design), OR it IS required and PF15 currently reads HIGH. 0 only when
 * required AND PF15 currently reads LOW. This is the single source of
 * truth `ArmConditionsMet()`/`SM_Fire()` (state_machine.c) both check
 * internally -- also exposed publicly so the command layer
 * (cmd_pid_profile_start(), commands.c) can give a precise, distinct
 * error message instead of a generic failure, matching the same
 * "distinct failure reasons, reported distinctly" precedent that
 * function's own comment already established for
 * PID:PROFILE:TIMING-not-set. */
uint8_t SM_ExternalEnableOk(void);

/* Raw PF15 logic level right now (1 = HIGH, 0 = LOW) -- independent of
 * whether the interlock is even turned on. Diagnostic: lets an operator
 * confirm real wiring/signal presence (`EXTernal:INPut?`, commands.c)
 * before actually turning the interlock on, the same "verify before you
 * rely on it" role PFMIN:DEBUG:RAW?/REG? played for the PF_Input fiber-
 * patching investigation (docs/changelog.txt, 2026-09-15). */
uint8_t SM_GetExternalEnableInputRaw(void);

/* --------------------------------------------------------------------------
 * External trigger (rising edge on PF15 fires a shot), added 2026-09-16,
 * per direct follow-up request.
 *
 * Reuses the SAME PF15 signal as the external-enable interlock above --
 * NOT a second pin. Opt-in, OFF by default, and structurally DEPENDS on
 * external-enable also being on: SM_SetExternalTriggerRequired(1)
 * REFUSES (returns 0) unless SM_GetExternalEnableRequired() is already
 * 1, and SM_SetExternalEnableRequired(0) forces this back off too (see
 * that function's own updated comment above) -- an operator can never
 * end up with triggering active while the "lose the signal -> fault"
 * protection it relies on is silently disabled. Per direct
 * instruction, "as usual" losing the signal mid-shot means a fault --
 * that IS the existing SM_FAULT_EXTERNAL_ENABLE mechanism above,
 * reused as-is, not a second fault path.
 *
 * When ON, and the state machine is currently SM_STATE_ARMED: a
 * LOW-to-HIGH transition on PF15 calls SM_Fire() directly -- the exact
 * same function `PID:PROFile:STARt` itself calls, so every one of that
 * path's own guarantees (the redundant external-enable re-check right
 * before firing, the profile-timing-configured check, the resulting
 * FIRING-state fault monitoring) apply identically whether a shot
 * started by operator command or by this trigger. Per direct
 * instruction ("this is the same behavior as running PID:PROF:STAR"),
 * there is no second/different start path for a triggered shot -- see
 * this file's own investigation into whether a distinct "open-loop
 * start call" exists (it doesn't: open- vs. closed-loop has always
 * been the PER-CHANNEL PID:LOOPMODE flag, checked inside the SAME
 * PID_Update() loop both PID:START and PID:PROFile:STARt already
 * share -- there was never a second, structurally-open-loop start
 * mechanism to call here, aside from the unrelated legacy pfm.c
 * TABLE:*-and-FIRE path, which this feature does NOT touch).
 *
 * Edge detection: a baseline PF15 level is captured fresh the instant
 * SM_Arm() succeeds (SM_Fire() -- state_machine.c -- also updates it on
 * every SM_PollFaults() call while still ARMED and this feature is on)
 * specifically so a signal that's ALREADY HIGH at the moment of arming
 * does not look like a rising edge on the very next poll -- only a
 * genuine LOW-then-HIGH transition AFTER arming counts. If SM_Fire()
 * itself fails for some other reason (e.g. profile timing never set),
 * the state simply stays ARMED with the baseline now recording HIGH --
 * a real falling-then-rising edge is needed to try again, not just
 * PF15 remaining HIGH; this is an edge-triggered mechanism, not a
 * level-triggered one.
 *
 * Checked from INSIDE SM_PollFaults() (not a separate poll function) --
 * same reasoning as the FIRING-state fault check above: shares that
 * function's already-established call-site guarantee (main loop +
 * every real PID_Update() tick) rather than requiring every caller to
 * also remember a second poll entry point. While merely ARMED (not yet
 * FIRING), the only caller actually reaching SM_PollFaults() is the
 * main loop -- its own iteration rate is unbounded/undocumented (see
 * this file's pre-existing "NOTE TO REVISIT" on that), so trigger
 * latency while ARMED inherits that same, already-flagged limitation;
 * not a new one introduced here. */

/* Turns the trigger feature on (1) or off (0). Returns 1 on success, 0
 * if refused -- ONLY possible refusal: `required` is nonzero (turning
 * ON) while SM_GetExternalEnableRequired() is currently 0. Backs
 * `EXTernal:TRIGger <0|1>` (commands.c), which reports a distinct error
 * for that refusal rather than a generic one. RAM-only, resets to 0 on
 * reboot, same as every other runtime config in this project. */
uint8_t SM_SetExternalTriggerRequired(uint8_t required);

/* Current mode, as last set by SM_SetExternalTriggerRequired() (0 by
 * default at boot, and forced back to 0 if external-enable is ever
 * turned off -- see SM_SetExternalEnableRequired()'s own comment).
 * Backs `EXTernal:TRIGger?` (commands.c). */
uint8_t SM_GetExternalTriggerRequired(void);

#ifdef __cplusplus
}
#endif

#endif /* __STATE_MACHINE_H__ */
