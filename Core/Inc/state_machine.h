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
 *                              permissive signal -- PF13 as of
 *                              2026-09-17 (MOVED off PF15,
 *                              "Fiber_Enable", the same day PF15 became
 *                              trigger-only -- see the external-enable
 *                              section further down this file), not a
 *                              hardware fault condition in the
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
 *   SM_FAULT_EMERGENCY_STOP  -- ADDED 2026-09-17, per direct request: a
 *                              real, physical emergency-stop input
 *                              (PG10, a fiber-optic receiver input --
 *                              docs/pin_mapping_v4.csv mislabels this
 *                              net "NRST", a stale/incorrect name, NOT
 *                              this MCU's own reset function; confirmed
 *                              directly with the user). Polarity per
 *                              EMERGENCY_STOP_POLARITY (ctrlr_config.h)
 *                              -- INVERTED same day, later, once a
 *                              hardware inverter was added between the
 *                              fiber receiver and PG10: now NORMALLY_LOW
 *                              ("no input" reads OK, a HIGH reading is
 *                              the fault), the opposite of this
 *                              feature's original NORMALLY_HIGH
 *                              assumption -- see this section further
 *                              down this file and ctrlr_config.h's own
 *                              comment for the full reasoning and the
 *                              safety trade-off it carries. UNLIKE every
 *                              other fault type
 *                              above, deliberately NOT a graceful
 *                              ramp-down -- HandleEmergencyStopFault()
 *                              (state_machine.c) calls PID_Stop()
 *                              directly and unconditionally, with no
 *                              g_stateBeforeFault branch at all (every
 *                              other handler ramps if the fault hit
 *                              while FIRING; this one never does,
 *                              regardless of state) -- an immediate,
 *                              unconditional hard cutoff of every
 *                              enabled channel is the explicit point of
 *                              an EMERGENCY stop, not a softer
 *                              "eventually stops" response. Checked
 *                              continuously in SM_PollFaults() across
 *                              ALL states (IDLE/ARMED/FIRING alike),
 *                              matching General Fault's own PC10/
 *                              GateDriverStatus precedent -- NOT the
 *                              FIRING-only carve-out External-Enable
 *                              above deliberately uses; there is no
 *                              ARM/PID:PROFile:STARt-time gating for
 *                              this one either, not asked for and not
 *                              added. Fully opt-in
 *                              (EMERGency:ENAble/EMERGency:ENAble?) --
 *                              per direct instruction, "when the E-stop
 *                              feature is disabled, we act as though it
 *                              does not exist and the firmware
 *                              functions like it did before we
 *                              implemented it": every check below
 *                              (SM_PollFaults(), SM_ClearFault()) is
 *                              gated on the enable flag FIRST, with
 *                              zero behavior change of any kind
 *                              (including no PG10 read at all) when
 *                              it's off. `FAULT:CLEAR` re-validates
 *                              PG10 is back OK (per current polarity)
 *                              before actually clearing, same
 *                              philosophy as every other fault type.
 *                              `GPIO_PULLDOWN` (main.c) is UNCHANGED by
 *                              the 2026-09-17 polarity inversion --
 *                              floating still reads LOW at the pin
 *                              either way; only the MEANING assigned to
 *                              that level (ctrlr_config.h's
 *                              EMERGENCY_STOP_POLARITY) flipped. Under
 *                              the CURRENT (NORMALLY_LOW) polarity this
 *                              means a floating/disconnected PG10 now
 *                              reads as OK, not asserted -- a real,
 *                              deliberate, directly-confirmed reversal
 *                              of this feature's original fail-safe
 *                              assumption, not an oversight.
 *
 *   SM_FAULT_ENABLE_OUTPUT   -- ADDED 2026-09-17, per direct request: each
 *                              Transrex channel's own ENA_OUT/CONTACT_OUT
 *                              fiber OUTPUTS (PG0-PG3/PG4-PG7 --
 *                              XR1-XR4_ENA_OUT/_CONTACT_OUT,
 *                              docs/pin_mapping_v4.csv's "XREX Pin Name"
 *                              column) must both be commanded ON before
 *                              `ARM` will succeed for that channel, and
 *                              staying ON is continuously required once
 *                              ARMED (not just while FIRING -- a
 *                              deliberately WIDER window than
 *                              SM_FAULT_EXTERNAL_ENABLE's FIRING-only
 *                              carve-out above, confirmed directly by the
 *                              user). PER-CHANNEL, unlike
 *                              SM_FAULT_EXTERNAL_ENABLE/EMERGENCY_STOP
 *                              (system-wide, single PF13/PG10 signal) --
 *                              only currently-enabled channels
 *                              (`PID_GetChannelEnable()`) are checked,
 *                              matching the same per-channel-gating
 *                              philosophy already established for
 *                              Water/Temp/Enerpro/OCP (xrex_io.h). NOT
 *                              a hardware-read fault like those, though
 *                              -- ENA_OUT/CONTACT_OUT are OUTPUTS this
 *                              firmware itself drives via
 *                              `XREX:CHANnel:ENAOut`/`CONTactOut`
 *                              (commands.c); this fault type exists
 *                              because an operator could set one, then
 *                              clear it again, while a channel is
 *                              already ARMED/FIRING.
 *
 *                              Reuses HandleOvercurrentFault(channel)
 *                              DIRECTLY for its response (EnterFault()'s
 *                              switch) -- immediate hard-disable of the
 *                              affected channel, survivors derated
 *                              1/N and ramped down exactly like a real
 *                              OCP fault -- reported as its own DISTINCT
 *                              type purely for `STATE?` diagnostics
 *                              (same "distinct type, shared response"
 *                              pattern SM_FAULT_EXTERNAL_ENABLE already
 *                              established for HandleGeneralFault()).
 *                              Do not confuse the NAME with
 *                              SM_FAULT_EXTERNAL_ENABLE -- that is a
 *                              single, system-wide INPUT interlock
 *                              (PF13, an operator/facility permissive
 *                              this firmware reads); this is a
 *                              PER-CHANNEL check of this firmware's OWN
 *                              commanded OUTPUT state. See the
 *                              enable-output section further down this
 *                              file for the full design
 *                              (`XrexIo_EnableOutputsReadyToArm()`,
 *                              `XrexIo_PollEnableOutputFaults()`,
 *                              `SM_ReportEnableOutputFault()`).
 *
 * *** NOTE TO REVISIT, RESOLVED 2026-09-17 *** -- originally: "we will
 * define the mapping here at a later time" for SM_ReportOcpFault(channel)'s
 * real hardware trigger. Now real: XrexIo_PollOcpFaults() (xrex_io.h/.c)
 * polls PF4/PF5/PF8/PF12 (per-channel, per-channel-gated) every
 * PID_Update() tick and once at boot, calling SM_ReportOcpFault(channel)
 * on a real fault -- no EXTI (a real SYSCFG_EXTICR line-sharing conflict
 * with the existing GateDriverStatus EXTI setup ruled that out, see
 * xrex_io.h's own header comment). Water/Temp/Enerpro (the existing
 * GateDriverStatus_01..12 EXTI source) still route to SM_FAULT_GENERAL,
 * per-channel-gated the same way via XrexIo_EvaluateGateDriverFault() --
 * see docs/command_reference.md's XREX:CHANnel:STATus? section for the
 * full architecture.
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
 * ALSO NOTE TO REVISIT, PARTIALLY RESOLVED: SM_Arm()'s ArmConditionsMet()
 * is no longer a pure stub -- it now checks the external-enable interlock
 * (SM_ExternalEnableOk(), 2026-09-16) and the enable-output/contactor-
 * output precondition (XrexIo_EnableOutputsReadyToArm(), 2026-09-17, see
 * this file's own enable-output section further down). Still no gain-
 * sanity or other interlock logic beyond those two. AND:
 * FAULT_RAMP_DOWN_TIME_S (ctrlr_config.h, currently 1.0s) is a first
 * guess, not derived from any real Transrex/magnet requirement -- see
 * that macro's own comment.
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
    SM_FAULT_EXTERNAL_ENABLE,  /* added 2026-09-16 -- see the external-enable
                                   section below and this file's own
                                   header comment for the full design */
    SM_FAULT_EMERGENCY_STOP,   /* added 2026-09-17 -- see the emergency-stop
                                   section below and this file's own
                                   header comment for the full design */
    SM_FAULT_ENABLE_OUTPUT     /* added 2026-09-17 -- see the enable-output
                                   section below and this file's own
                                   header comment for the full design.
                                   PER-CHANNEL (like SM_FAULT_OVERCURRENT) --
                                   NOT the same thing as
                                   SM_FAULT_EXTERNAL_ENABLE (system-wide,
                                   a different signal entirely) */
} SM_FaultType_t;

/* Called once at boot (main.c), after pid.c's own PID_Init() AND after
 * main.c's own MX_GPIO_Init() (which now also configures PF13,
 * external-enable, and PF15, external-trigger -- see those interlocks'
 * own doc comments below). Sets the state to IDLE. Does not touch any
 * hardware itself -- pid.c/hrtim.c's own init already leaves outputs
 * off, and PF13/PF15's GPIO config lives in main.c's MX_GPIO_Init()
 * (matching gate_driver.c's own precedent for plain-input pins: GPIO
 * config in main.c, the read/fault logic in the owning module) -- this
 * just makes IDLE the explicit, queryable state from the start. */
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
 * External-enable interlock (PF13 -- docs/pin_mapping_v4.csv, documented
 * "GPInput_12"), added 2026-09-16, MOVED here 2026-09-17 from PF15
 * (Fiber_Enable), per direct instruction: enable and trigger are now two
 * independent physical signals on separate pins, not one shared wire --
 * PF15 keeps ONLY the trigger role from here on (see the external-trigger
 * section directly below). Every behavior/gating rule described here is
 * otherwise UNCHANGED from the original PF15-based design; only the pin
 * moved.
 *
 * A pin the operator reads, not drives -- confirmed GPI in the pin
 * mapping doc (PC14, the pin first proposed for this feature, was
 * rejected: it's documented GPO there, "STM_Enable_Pin" -- the STM32
 * DRIVES that one outward, the opposite direction needed here; PF15 was
 * the correct pin then, PF13 is the correct pin now). Opt-in: OFF by
 * default at boot (SM_SetExternalEnableRequired() was never called, or
 * was last called with 0) -- existing shots/tests are completely
 * unaffected unless this is explicitly turned on.
 *
 * When ON:
 *   1. `ARM` refuses (ArmConditionsMet(), state_machine.c) unless PF13
 *      currently reads HIGH -- folded into that function's existing
 *      generic "arm conditions not met" failure (ERR 13), no new error
 *      code needed for this case.
 *   2. `PID:PROFile:STARt` ALSO re-checks PF13 immediately before
 *      firing (SM_ExternalEnableOk(), below, checked by the command
 *      handler for a precise error -- see cmd_pid_profile_start()'s own
 *      comment -- AND redundantly inside SM_Fire() itself as a last-
 *      line-of-defense, in case some other future caller ever bypasses
 *      the command handler's own check, INCLUDING the external-trigger
 *      feature below, which calls SM_Fire() directly). Closes the real
 *      gap where PF13 could drop in the window between a successful ARM
 *      and the eventual START -- ARM alone is NOT enough to guarantee
 *      this.
 *   3. While FIRING, SM_PollFaults() (below) ALSO checks PF13 every
 *      time it runs (the main loop, AND every real PID_Update() tick --
 *      the same cadence General Fault's own two hardware sources
 *      already get). If PF13 reads LOW while FIRING, enters
 *      SM_STATE_FAULT with SM_FAULT_EXTERNAL_ENABLE -- identical
 *      open-loop ramp-down response to General Fault (HandleGeneralFault()
 *      is reused directly, see EnterFault()'s switch, state_machine.c)
 *      -- reported as a DISTINCT fault type purely for operator
 *      diagnostics (STATE? -> "OK FAULT EXTERNAL_ENABLE"), not a
 *      different physical response. Per direct instruction, this is
 *      checked only while actually FIRING ("if that input is lost
 *      during a shot") -- losing PF13 while merely ARMED (nothing
 *      outputting yet) is NOT itself treated as a fault here; the next
 *      PID:PROFile:STARt attempt will simply fail its own re-check
 *      (item 2 above) instead. Deliberately not decided either way
 *      whether ARMED should also actively fault on PF13 loss -- not
 *      asked for, flagged rather than silently added.
 *   4. `FAULT:CLEAR` (SM_ClearFault(), above) re-validates PF13 is back
 *      HIGH before actually clearing an EXTERNAL_ENABLE fault, same
 *      "only clear if the condition is actually gone" philosophy
 *      already applied to PC10/GateDriverStatus.
 *
 * GPIO_PULLDOWN (main.c's MX_GPIO_Init(), not this project's usual
 * GPIO_NOPULL for actively-driven inputs) is a DELIBERATE fail-safe
 * choice specific to this one signal: an unconnected/floating PF13
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
 * to assume was already covered here. *** */

/* Turns the interlock above on (1) or off (0). Persists across shots
 * until changed again or the board reboots (RAM state, like every
 * other runtime config in this project -- PID:PROFILE:CURRENT,
 * PID:LOOPMODE, etc.) -- NOT saved to non-volatile storage. Backs
 * `EXTernal:ENAble <0|1>` (commands.c). Takes effect immediately: if turned
 * ON while already FIRING, the very next SM_PollFaults() call (at most
 * one PID_Update() tick away) starts checking PF13.
 *
 * UNCOUPLED from the external-trigger feature (below) 2026-09-17, the
 * same day enable and trigger split onto separate pins (PF13/PF15) --
 * previously turning this off also forced trigger off, since both
 * features read the same physical wire; that coupling no longer applies
 * now that they're independent signals, per direct instruction. The
 * real safety guarantee is unaffected: SM_Fire() itself always
 * re-checks this interlock before actually firing, regardless of how
 * trigger was configured. */
void SM_SetExternalEnableRequired(uint8_t required);

/* Current mode, as last set by SM_SetExternalEnableRequired() (0 by
 * default at boot). Backs `EXTernal:ENAble?` (commands.c). */
uint8_t SM_GetExternalEnableRequired(void);

/* 1 if the interlock isn't required at all (SM_GetExternalEnableRequired()
 * == 0 -- always "ok" in that case, matching this feature's opt-in
 * design), OR it IS required and PF13 currently reads HIGH. 0 only when
 * required AND PF13 currently reads LOW. This is the single source of
 * truth `ArmConditionsMet()`/`SM_Fire()` (state_machine.c) both check
 * internally -- also exposed publicly so the command layer
 * (cmd_pid_profile_start(), commands.c) can give a precise, distinct
 * error message instead of a generic failure, matching the same
 * "distinct failure reasons, reported distinctly" precedent that
 * function's own comment already established for
 * PID:PROFILE:TIMING-not-set. */
uint8_t SM_ExternalEnableOk(void);

/* Raw PF13 logic level right now (1 = HIGH, 0 = LOW) -- independent of
 * whether the interlock is even turned on. Diagnostic: lets an operator
 * confirm real wiring/signal presence (`EXTernal:INPut?`, commands.c)
 * before actually turning the interlock on, the same "verify before you
 * rely on it" role PFMIN:DEBUG:RAW?/REG? played for the PF_Input fiber-
 * patching investigation (docs/changelog.txt, 2026-09-15). */
uint8_t SM_GetExternalEnableInputRaw(void);

/* --------------------------------------------------------------------------
 * External trigger (rising edge on PF15, "Fiber_Enable", fires a shot),
 * added 2026-09-16, per direct follow-up request. RESTRUCTURED
 * 2026-09-17, per direct instruction: PF15 now backs TRIGGER ONLY --
 * the external-enable interlock above moved to its own separate pin
 * (PF13). Trigger and enable are independent physical signals from here
 * on, not two facets of one shared wire.
 *
 * Opt-in, OFF by default. UNCOUPLED from the external-enable interlock
 * 2026-09-17 -- SM_SetExternalTriggerRequired() no longer refuses
 * regardless of SM_GetExternalEnableRequired()'s state (previously it
 * did, back when both features read the same wire and "trigger without
 * enable" was structurally meaningless). This does NOT weaken the real
 * safety guarantee: SM_Fire() (below) still unconditionally re-checks
 * SM_ExternalEnableOk() (PF13) every time it's called, including when
 * called from this trigger's own edge-detection path -- a rising edge
 * on PF15 while PF13 doesn't currently read HIGH (and the enable
 * interlock is on) simply fails to fire, exactly as a manual
 * `PID:PROFile:STARt` would.
 *
 * When ON, and the state machine is currently SM_STATE_ARMED: a
 * LOW-to-HIGH transition on PF15 calls SM_Fire() directly -- the exact
 * same function `PID:PROFile:STARt` itself calls, so every one of that
 * path's own guarantees (the external-enable re-check, the
 * profile-timing-configured check, the resulting FIRING-state fault
 * monitoring) apply identically whether a shot started by operator
 * command or by this trigger. Per direct instruction ("this is the same
 * behavior as running PID:PROF:STAR"), there is no second/different
 * start path for a triggered shot -- see this file's own investigation
 * into whether a distinct "open-loop start call" exists (it doesn't:
 * open- vs. closed-loop has always been the PER-CHANNEL PID:LOOPMODE
 * flag, checked inside the SAME PID_Update() loop both PID:START and
 * PID:PROFile:STARt already share -- there was never a second,
 * structurally-open-loop start mechanism to call here, aside from the
 * unrelated legacy pfm.c TABLE:*-and-FIRE path, which this feature does
 * NOT touch). And, per direct instruction 2026-09-17: this only ever
 * fires from SM_STATE_ARMED -- `if (g_state == SM_STATE_ARMED)` in
 * SM_PollFaults() is the ONLY place this trigger can act, no other
 * state.
 *
 * Edge detection: a baseline PF15 level is captured fresh the instant
 * SM_Arm() succeeds (SM_Fire() -- state_machine.c -- also updates it on
 * every SM_PollFaults() call while still ARMED and this feature is on)
 * specifically so a signal that's ALREADY HIGH at the moment of arming
 * does not look like a rising edge on the very next poll -- only a
 * genuine LOW-then-HIGH transition AFTER arming counts. If SM_Fire()
 * itself fails for some other reason (e.g. profile timing never set,
 * or PF13/enable isn't currently satisfied), the state simply stays
 * ARMED with the baseline now recording HIGH -- a real falling-then-
 * rising edge is needed to try again, not just PF15 remaining HIGH;
 * this is an edge-triggered mechanism, not a level-triggered one.
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

/* Turns the trigger feature on (1) or off (0). Returns 1 always, as of
 * 2026-09-17 (previously could return 0/refuse -- see this feature's
 * own header comment above for why that coupling to external-enable
 * was removed). Kept a uint8_t return rather than void purely so the
 * command layer (cmd_ext_trigger(), commands.c) didn't need its own
 * signature change. Backs `EXTernal:TRIGger <0|1>` (commands.c).
 * RAM-only, resets to 0 on reboot, same as every other runtime config
 * in this project. */
uint8_t SM_SetExternalTriggerRequired(uint8_t required);

/* Current mode, as last set by SM_SetExternalTriggerRequired() (0 by
 * default at boot). Backs `EXTernal:TRIGger?` (commands.c). */
uint8_t SM_GetExternalTriggerRequired(void);

/* Raw PF15 logic level right now (1 = HIGH, 0 = LOW) -- independent of
 * whether the trigger feature is even turned on. Added 2026-09-17
 * alongside the PF13/PF15 split -- previously EXTernal:INPut? covered
 * this same pin (it backed both enable and trigger, being the same
 * wire); now that they're separate, this is trigger's own dedicated
 * diagnostic, matching SM_GetExternalEnableInputRaw()'s role for PF13.
 * Backs `EXTernal:TRIGger:INPut?` (commands.c). */
uint8_t SM_GetExternalTriggerInputRaw(void);

/* --------------------------------------------------------------------------
 * Emergency stop (PG10, a fiber-optic receiver input -- NOT the same
 * pin/signal as PF15 above), added 2026-09-17, per direct request.
 *
 * docs/pin_mapping_v4.csv labels this net "NRST" -- confirmed directly
 * with the user this is a stale/incorrect label, not this MCU's own
 * reset function; the pin genuinely carries a fiber-optic E-stop input
 * (with a 100nF cap to ground on the net, fine/beneficial for a
 * deliberately slow-changing safety signal -- irrelevant to the fast
 * switching PFM_Input capture pins worry about, not a concern here).
 * PB8 (this board's BOOT0 net) was considered and REJECTED for this
 * purpose first -- confirmed via a live FLASH_OPTR register read
 * (nSWBOOT0=1) that PB8 is genuinely sampled for boot-mode selection
 * on every reset, not just at first power-on; an active-LOW E-stop's
 * own idle (non-emergency) HIGH state is exactly the "boot into the
 * ROM bootloader instead of the application" condition on this chip,
 * meaning any ordinary reset during normal (non-emergency) operation
 * could silently skip the application entirely. See
 * cmd_diag_optbytes_query()'s own doc comment (commands.c) for the
 * full register-level finding.
 *
 * Polarity per EMERGENCY_STOP_POLARITY (ctrlr_config.h) -- originally
 * NORMALLY_HIGH (PG10 read LOW meant asserted) when this feature was
 * first built; INVERTED 2026-09-17, later the same day, once a
 * hardware inverter was added between the fiber-optic receiver and
 * PG10 itself. The CURRENT default is NORMALLY_LOW: "no input" --
 * including PG10's own GPIO_PULLDOWN floating default -- reads OK, and
 * a HIGH reading is the fault. See ctrlr_config.h's own comment on
 * EMERGENCY_STOP_POLARITY for the full reasoning, including the
 * explicit safety-trade-off note (a floating/disconnected pin, or an
 * unpowered inverter, now reads as OK rather than asserted -- a real,
 * deliberate reversal of this feature's original fail-safe assumption,
 * confirmed directly, not an oversight). EmergencyStopAsserted()
 * (state_machine.c) is the one place this polarity is actually applied
 * -- EmergencyStopInputIsHigh() itself stays a plain, polarity-agnostic
 * raw reader, matching SM_GetEmergencyStopInputRaw()'s own convention
 * below.
 *
 * Deliberately NOT the graceful ramp-down every other fault type above
 * uses -- HandleEmergencyStopFault() (state_machine.c) calls
 * PID_Stop() directly, unconditionally, with no g_stateBeforeFault
 * branch at all -- an immediate, hard cutoff of every enabled channel,
 * regardless of what state the fault hit in, is the explicit point of
 * an EMERGENCY stop. (Unaffected by the polarity inversion above --
 * once EmergencyStopAsserted() is true, the response is identical
 * either way.)
 *
 * Checked continuously in SM_PollFaults() across ALL states (IDLE/
 * ARMED/FIRING), matching General Fault's own PC10/GateDriverStatus
 * precedent -- NOT the FIRING-only carve-out external-enable
 * deliberately uses above. There is no ARM/PID:PROFile:STARt-time
 * gating for this feature -- not asked for, not added; this is purely
 * a continuously-monitored fault source with an immediate-cutoff
 * response, nothing else.
 *
 * Fully opt-in, OFF by default. Per direct instruction, "when the
 * E-stop feature is disabled, we act as though it does not exist and
 * the firmware functions like it did before we implemented it" --
 * every single check this feature adds (SM_PollFaults(),
 * SM_ClearFault()) is gated on SM_GetEmergencyStopRequired() FIRST,
 * with genuinely zero behavior difference (including no PG10 GPIO read
 * at all) when it's off -- not merely "the fault never latches," but
 * the check itself never runs.
 *
 * `FAULT:CLEAR` re-validates PG10 is back OK (per EMERGENCY_STOP_POLARITY,
 * currently NORMALLY_LOW) before actually clearing an EMERGENCY_STOP
 * fault (when the feature is on -- a no-op re-check, same as every
 * other such check in this file, when it's off), same "only clear if
 * the condition is actually gone" philosophy as every other fault
 * type.
 *
 * `GPIO_PULLDOWN` (main.c's MX_GPIO_Init()) is UNCHANGED by the
 * 2026-09-17 polarity inversion -- an unconnected/floating PG10 still
 * reads LOW at the pin either way; only EMERGENCY_STOP_POLARITY's
 * interpretation of that level flipped (see ctrlr_config.h's own
 * comment on that constant). Under the CURRENT polarity, floating
 * reads as OK, not asserted -- the opposite of this feature's original
 * fail-safe design, a deliberate and directly-confirmed choice driven
 * by the new hardware inverter, not an oversight. */

/* Turns the emergency-stop feature on (1) or off (0). RAM-only, resets
 * to 0 on reboot, same as every other runtime config in this project.
 * Backs `EMERGency:ENAble <0|1>` (commands.c). */
void SM_SetEmergencyStopRequired(uint8_t required);

/* Current mode, as last set by SM_SetEmergencyStopRequired() (0 by
 * default at boot). Backs `EMERGency:ENAble?` (commands.c). */
uint8_t SM_GetEmergencyStopRequired(void);

/* 1 if the feature isn't required at all (SM_GetEmergencyStopRequired()
 * == 0 -- always "ok" in that case, matching this feature's opt-in,
 * "acts as though it does not exist" design), OR it IS required and
 * PG10's CURRENT polarity (EMERGENCY_STOP_POLARITY, ctrlr_config.h)
 * says it's not asserted. 0 only when required AND the pin reads as
 * asserted under that polarity. Used internally by SM_ClearFault();
 * also exposed publicly for any diagnostic/command-layer use that
 * wants it. */
uint8_t SM_EmergencyStopOk(void);

/* Raw PG10 logic level right now (1 = electrically HIGH, 0 =
 * electrically LOW) -- independent of whether the feature is even
 * turned on, and deliberately POLARITY-AGNOSTIC: this is the literal
 * pin state, not an "asserted"/"OK" interpretation (that's
 * SM_EmergencyStopOk()'s job, via EMERGENCY_STOP_POLARITY,
 * ctrlr_config.h) -- same raw-vs-interpreted split as
 * SM_GetExternalEnableInputRaw()/EXTernal:INPut? for PF13, and
 * XREX:CHANnel:STATus? for the XREX fault pins. Diagnostic: lets an
 * operator confirm real wiring/signal presence before relying on it.
 * Backs `EMERGency:INPut?` (commands.c). */
uint8_t SM_GetEmergencyStopInputRaw(void);

/* --------------------------------------------------------------------------
 * Enable/contactor output precondition (PG0-PG3/PG4-PG7 -- XR1-XR4's own
 * ENA_OUT/CONTACT_OUT fiber outputs, docs/pin_mapping_v4.csv's "XREX Pin
 * Name" column), added 2026-09-17, per direct request.
 *
 * Two commanded OUTPUTS per Transrex channel -- set via
 * `XREX:CHANnel:ENAOut <ch> <0|1>` / `XREX:CHANnel:CONTactOut <ch> <0|1>`
 * (commands.c), owned (pin table, GPIO read/write) by xrex_io.c matching
 * that module's established per-channel-XR-signal charter. Direct
 * instruction: "The controller cannot be armed unless these are
 * outputting prior to the arm signal, and similarly we cannot transition
 * to the ARM state i[f] these are not output."
 *
 * PER-CHANNEL, asked and confirmed directly: only currently-enabled
 * channels (`PID_GetChannelEnable()`) are checked -- running XR1 alone
 * never requires XR2/3/4's ENA_OUT/CONTACT_OUT to be set, matching the
 * exact same per-channel-gating philosophy already established for
 * Water/Temp/Enerpro/OCP (xrex_io.h). A channel's own ENA_OUT AND
 * CONTACT_OUT must BOTH read HIGH to count as "outputting" for that
 * channel -- treated as one combined per-channel condition, not two
 * independently-faultable ones (matches how the request groups them:
 * "these are outputting"/"these are not output", never asking to
 * distinguish which one dropped).
 *
 * 1. **`ARM`** refuses (folded into the existing generic `ERR 13`, same
 *    as the external-enable interlock above) unless EVERY currently-
 *    enabled channel's ENA_OUT+CONTACT_OUT are both HIGH --
 *    `XrexIo_EnableOutputsReadyToArm()` (xrex_io.h), a pure check, no
 *    side effects, called from `ArmConditionsMet()`.
 * 2. **Continuously monitored once ARMED** -- asked directly and
 *    confirmed: unlike SM_FAULT_EXTERNAL_ENABLE's FIRING-only carve-out,
 *    this is checked in BOTH `SM_STATE_ARMED` and `SM_STATE_FIRING` (NOT
 *    `SM_STATE_IDLE` -- nothing is armed yet there, matching the
 *    "precondition for arming" framing, not a general always-on safety
 *    interlock like emergency-stop). `XrexIo_PollEnableOutputFaults()`
 *    (xrex_io.h) does this gating itself (checks `SM_GetState()` before
 *    doing any per-channel work at all) and is called from the SAME
 *    tick cadence `XrexIo_PollOcpFaults()` already gets (main.c's boot +
 *    main loop, and every real `PID_Update()` tick, pid.c) -- reuses
 *    that same call-site pattern rather than inventing a new one.
 * 3. On a fault, `SM_ReportEnableOutputFault(channel)` (below) is called
 *    -- SAME shape as `SM_ReportOcpFault()` (critical-section-protected,
 *    "already faulted -> just hard-disable this channel" short-circuit),
 *    entering `SM_STATE_FAULT` with `SM_FAULT_ENABLE_OUTPUT` and reusing
 *    `HandleOvercurrentFault(channel)` DIRECTLY for the response --
 *    immediate hard-disable of the affected channel, survivors derated
 *    1/N and ramped down exactly like a real OCP fault. See this file's
 *    own header comment on `SM_FAULT_ENABLE_OUTPUT` for why this fault
 *    type is deliberately named/kept distinct from
 *    `SM_FAULT_EXTERNAL_ENABLE` despite the similar name -- they are
 *    unrelated mechanisms (this one: this firmware's own per-channel
 *    OUTPUT state; that one: a single system-wide INPUT interlock, PF13).
 * 4. **`FAULT:CLEAR`** needs NO dedicated re-validation for this fault
 *    type -- matching `SM_FAULT_OVERCURRENT`'s own established precedent
 *    (also no re-check in `SM_ClearFault()`), not an oversight: the
 *    affected channel is already hard-disabled
 *    (`PID_SetChannelEnable(channel, 0)`) by the time `FAULT:CLEAR` is
 *    even considered, so there is nothing still-missing left to block.
 *    The real gate is `ArmConditionsMet()` (item 1 above) at the NEXT
 *    `ARM` attempt -- if the operator re-enables this channel before its
 *    ENA_OUT/CONTACT_OUT are actually restored, arming will correctly
 *    refuse again then.
 *
 * GPIO config (main.c's `MX_GPIO_Init()`): push-pull outputs, driven LOW
 * before being enabled (never glitches HIGH on boot), matching this
 * project's established diagnostic-output convention (`DIAGnostic:GPOut11`/
 * `GPOut12`) -- these ARE the real, permanent per-channel outputs
 * themselves, not a diagnostic stand-in for them. No pull resistor
 * question applies (these are outputs, not inputs). */

/* Entry point called by XrexIo_PollEnableOutputFaults() (xrex_io.c) the
 * moment `channel`'s own ENA_OUT/CONTACT_OUT are found not both HIGH
 * while ARMED or FIRING. Same shape as SM_ReportOcpFault() above --
 * critical-section-protected; if already SM_STATE_FAULT (either type),
 * unconditionally hard-disables `channel`'s own output and returns
 * without re-entering (a real dropped output must never be left
 * connected just because some other channel's fault got here first);
 * otherwise calls EnterFault(SM_FAULT_ENABLE_OUTPUT, channel), which
 * routes to HandleOvercurrentFault(channel) (state_machine.c) for the
 * actual response -- see this file's own SM_FAULT_ENABLE_OUTPUT header
 * comment for why that specific handler is reused. `channel` must be
 * 0-based and < HRTIM_NUM_CHANNELS (out-of-range is a defensive no-op,
 * matching SM_ReportOcpFault()'s own guard). */
void SM_ReportEnableOutputFault(uint8_t channel);

#ifdef __cplusplus
}
#endif

#endif /* __STATE_MACHINE_H__ */
