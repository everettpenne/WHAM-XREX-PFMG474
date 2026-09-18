#ifndef __XREX_IO_H__
#define __XREX_IO_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* --------------------------------------------------------------------------
 * xrex_io.c
 *
 * Added 2026-09-17, per direct request. docs/pin_mapping_v4.csv gained
 * a new "XREX Pin Name" column giving real, per-Transrex-channel
 * semantic labels to pins this project previously only knew by generic
 * names ("GateDriverStatus_NN", "GPInput_NN") inherited from the
 * sibling PFM-STM32G474 (limiter/H-bridge HVPS) project, where those
 * generic names made sense but don't clearly convey this project's own
 * per-channel Water/Temp/Enerpro/OCP fault structure. This module is
 * the home for that XR-semantic interpretation going forward --
 * direct instruction: leave gate_driver.c's own naming/module identity
 * alone (it's still an accurate low-level description of what it does
 * -- reads and latches PE0..PE11 via EXTI), rather than force a wider
 * rename onto already-working code. New work should be clearly XREX-
 * native instead.
 *
 * Per Transrex channel (XR1..XR4, 1:1 with this project's existing
 * WHAM/HRTIM channel numbering -- confirmed via XR1_DRIVE=PA8/
 * HRTIM1_CHA1 etc. matching the already-established Phase U/V/W/X
 * assignment):
 *
 *   XRn_WATER_FLT, XRn_TMP_FLT, XRn_ENERPRO_FLT -- the SAME 12 physical
 *     pins (PE0..PE11) gate_driver.c already reads/latches as
 *     "GateDriverStatus_01..12" -- this module does not duplicate that
 *     EXTI/GPIO/latch plumbing, it re-INTERPRETS the same raw bits
 *     (GateDriver_Read()) with XR-channel-aware semantics:
 *       - independently configurable polarity per category
 *         (ctrlr_config.h's XR_WATER_FLT_POLARITY/XR_TMP_FLT_POLARITY/
 *         XR_ENERPRO_FLT_POLARITY -- default NORMALLY_HIGH, confirmed
 *         directly, DELIBERATELY OPPOSITE the old shared
 *         GDS_FAULT_POLARITY this replaces -- see that constant's own
 *         removal comment, ctrlr_config.h, for the full reasoning);
 *       - per-channel GATING: a channel's own Water/Temp/Enerpro pins
 *         only count toward a fault if that channel is currently
 *         enabled (PID_GetChannelEnable()) -- per direct instruction,
 *         running XR1 alone must not register a low reading on XR2/3/4's
 *         fault pins as a reason to stop or prevent output.
 *     Water+Temp route to SM_FAULT_GENERAL (state_machine.h) -- per
 *     direct instruction, "Water, Temperature, and Enerpro faults are
 *     considered general faults." Enerpro was RECLASSIFIED 2026-09-18
 *     (see SM_FAULT_ENERPRO's own comment, state_machine.h) after
 *     docs/Transrex/Transrex_Controls_Upgrade (1).pdf's own fault table
 *     was found to give Enerpro the SAME response as Overcurrent
 *     (reduce surviving channels' DEMAND, keep running), NOT Water/
 *     Temp's full-stop -- the original instruction above is now
 *     superseded for Enerpro specifically, Water+Temp unchanged.
 *     XrexIo_EvaluateGateDriverFault() is gate_driver.c's OWN fault
 *     decision now (GateDriver_CheckFault() delegates to it) for the
 *     Water+Temp/SM_FAULT_GENERAL path -- this module doesn't
 *     re-implement the EXTI/latch/PFM_ForceStop mechanics, gate_driver.c
 *     still owns those. Enerpro's own per-channel check
 *     (XrexIo_PollEnerproFaults(), below) is a SEPARATE function, called
 *     from the same GateDriver_CheckFault() EXTI context but reporting
 *     directly via SM_ReportEnerproFault(channel) -- no shared latch/
 *     PFM_ForceStop involvement, matching SM_FAULT_OVERCURRENT's own
 *     "individual response" precedent instead.
 *
 *   XRn_OCP -- four BRAND NEW pins (PF4/PF5/PF8/PF12), no prior
 *     detection existed anywhere in this codebase (SM_ReportOcpFault()
 *     was previously only ever reachable via the software-injection
 *     OCP:TEST:FAULT command). POLLED, not EXTI-driven -- a real
 *     hardware conflict, not a preference: EXTI0..EXTI15 are 16 SHARED
 *     lines (one GPIO port per line number, project-wide, via
 *     SYSCFG_EXTICR), and 3 of these 4 pins' line numbers (EXTI4,
 *     EXTI5, EXTI8) are already claimed by the EXISTING GateDriverStatus
 *     EXTI setup on PE4/PE5/PE8 -- PF4/PF5/PF8 genuinely cannot ALSO be
 *     interrupt-driven simultaneously. Rather than split the 4 OCP pins
 *     across two different detection mechanisms (EXTI for the one that
 *     doesn't conflict, PF12, polling for the other three), all 4 are
 *     polled uniformly -- XrexIo_PollOcpFaults(), called from the SAME
 *     call sites (main.c's main loop, pid.c's PID_Update()) that
 *     already call state_machine.c's SM_PollFaults() at the same
 *     cadence (~1kHz while FIRING, whatever the main loop's own
 *     iteration rate is otherwise) -- same accepted latency this
 *     project already uses for the external-enable/external-trigger/
 *     emergency-stop features (state_machine.c). Checked in ALL states
 *     (not just FIRING), matching General Fault's own always-on
 *     philosophy -- an OCP condition should both PREVENT a shot from
 *     starting and STOP one already running, per direct instruction.
 *     Also per-channel GATED, same reasoning as Water/Temp/Enerpro
 *     above. Calls the EXISTING SM_ReportOcpFault(channel) directly --
 *     this module doesn't reimplement OCP's own response (the
 *     immediate per-channel disable + proportional derate + ramp is
 *     entirely PID_BeginOvercurrentRampDown()'s job, pid.c, unchanged);
 *     it only supplies the real hardware TRIGGER that function's own
 *     doc comment always anticipated ("whatever eventually detects a
 *     real per-channel OCP condition").
 *
 *   XRn_FEEDBACK -- just a new, clearer name for the existing
 *     PFM_Input_01..04 pins (pfm_input.c) -- XR1=PFM_Input_01(PA15),
 *     XR2=PFM_Input_02(PD4), XR3=PFM_Input_03(PB2), XR4=PFM_Input_04(PC12).
 *     No functional change; this module does not touch pfm_input.c.
 *
 *   XRn_DRIVE -- just a new, clearer name for the existing HRTIM phase
 *     output pins (hrtim.c) -- XR1=PHASE_U/HRTIM1_CHA1,
 *     XR2=PHASE_V/HRTIM1_CHB1, XR3=PHASE_W/HRTIM1_CHC1,
 *     XR4=PHASE_X/HRTIM1_CHD1. No functional change; this module does
 *     not touch hrtim.c. (Per-channel output gating -- "don't drive an
 *     output to XR2/3/4 when only XR1 is enabled" -- is already fully
 *     handled by the existing PID:CHANnel:ENAble mechanism, pid.c;
 *     nothing new needed here.)
 *
 *   XRn_ENA_OUT, XRn_CONTACT_OUT -- IMPLEMENTED 2026-09-17 (previously
 *     deferred -- see this file's own git history for the original
 *     "NOTE TO REVISIT" this replaces). Eight real outputs this
 *     firmware itself drives (PG0-PG3, PG4-PG7 -- GPO per
 *     docs/pin_mapping_v4.csv), one ENA_OUT + one CONTACT_OUT per
 *     Transrex channel, set via `XREX:CHANnel:ENAOut`/`CONTactOut`
 *     (commands.c). Direct instruction: `ARM` must refuse unless every
 *     currently-enabled channel's own ENA_OUT+CONTACT_OUT are BOTH
 *     already HIGH, and this must stay true continuously once ARMED
 *     (not just at the ARM instant) -- see state_machine.h's own
 *     SM_FAULT_ENABLE_OUTPUT/enable-output sections for the full fault-
 *     type design (reuses HandleOvercurrentFault()'s exact per-channel
 *     disable+derate+ramp response). Per-channel GATED, same
 *     philosophy as every other XR-signal above -- only currently-
 *     enabled channels are checked, in either direction (ARM gate or
 *     continuous poll). Unlike Water/Temp/Enerpro/OCP, these are
 *     OUTPUTS this firmware commands, not hardware-driven INPUTS it
 *     reads -- "checking" them is a readback of this firmware's own
 *     last-written GPIO state (HAL_GPIO_ReadPin() on a push-pull
 *     output correctly reflects ODR), not an interpretation of
 *     external signal polarity -- no XR_*_POLARITY-style config needed
 *     here, "HIGH" unambiguously means "commanded on."
 * -------------------------------------------------------------------------- */

/* Called from gate_driver.c's GateDriver_CheckFault() with the raw
 * 12-bit PE0..PE11 read (GateDriver_Read()) -- returns 1 if, after
 * per-channel gating and each category's own polarity setting, this
 * represents a real Water OR Temp fault (Enerpro EXCLUDED as of
 * 2026-09-18 -- see this file's own header comment on the
 * reclassification; use XrexIo_PollEnerproFaults() below for Enerpro);
 * 0 otherwise. Pure decision function -- does not itself latch anything
 * or touch hardware; gate_driver.c's own existing latch/PFM_ForceStop/
 * EnterFault(SM_FAULT_GENERAL) mechanics are unchanged, just now gated
 * on this function's answer instead of the old single-polarity
 * badBits check. */
uint8_t XrexIo_EvaluateGateDriverFault(uint16_t raw12);

/* Called from gate_driver.c's GateDriver_CheckFault() with the SAME raw
 * 12-bit PE0..PE11 read passed to XrexIo_EvaluateGateDriverFault() above
 * (no redundant GPIO re-read) -- checks each enabled channel's own
 * Enerpro bit (per XR_ENERPRO_FLT_POLARITY, ctrlr_config.h) and calls
 * SM_ReportEnerproFault(channel) DIRECTLY for any that read faulted.
 * Unlike XrexIo_EvaluateGateDriverFault() (a pure decision function),
 * this one has the side effect itself -- matching XrexIo_PollOcpFaults()'s
 * own "evaluate and report" shape, not the Water+Temp "evaluate, let the
 * caller latch/stop" shape, because Enerpro's real response
 * (HandleOvercurrentFault(), via SM_ReportEnerproFault()) is a per-
 * channel derate-and-ramp, not a shared full-stop -- there is no single
 * "the fault" to hand back to a caller, each affected channel's own
 * fault is independent. Added 2026-09-18, see this file's own header
 * comment for the reclassification. */
void XrexIo_PollEnerproFaults(uint16_t raw12);

/* Polls all 4 XRn_OCP pins (PF4/PF5/PF8/PF12) and calls
 * SM_ReportOcpFault(channel) (state_machine.h) directly for any
 * enabled channel whose pin currently reads as faulted (per
 * XR_OCP_FLT_POLARITY, ctrlr_config.h). Call from the same cadence
 * SM_PollFaults() already gets (main.c's main loop, pid.c's
 * PID_Update()) -- see this file's own header comment for why polling,
 * not EXTI. Safe/idempotent to call repeatedly while a fault is
 * already latched -- SM_ReportOcpFault() itself already handles that
 * case (state_machine.c). */
void XrexIo_PollOcpFaults(void);

/* Raw levels (1 = HIGH, 0 = LOW; polarity-agnostic, same "raw
 * readback" convention GDS?/EXTernal:INPut?/EMERGency:INPut? already
 * use) for one channel's own Water/Temp/Enerpro/OCP pins -- backs
 * XREX:CHANnel:STATus? (commands.c). `channel` is 0-based
 * (0..HRTIM_NUM_CHANNELS-1). Any output pointer may be NULL if that
 * value isn't wanted. Returns 0 (leaves outputs untouched) if `channel`
 * is out of range, 1 otherwise. */
uint8_t XrexIo_GetChannelStatus(uint8_t channel, uint8_t *water, uint8_t *tmp,
                                 uint8_t *enerpro, uint8_t *ocp);

/* Sets XRn_ENA_OUT for `channel` (0-based) HIGH (`on` != 0) or LOW.
 * Backs `XREX:CHANnel:ENAOut <ch> <0|1>` (commands.c). Out-of-range
 * `channel` is a silent no-op, matching this file's other setters.
 * Takes effect immediately -- does not itself check or touch ARM/FAULT
 * state; XrexIo_EnableOutputsReadyToArm()/XrexIo_PollEnableOutputFaults()
 * below are what react to the result. */
void XrexIo_SetEnableOutput(uint8_t channel, uint8_t on);

/* Raw current level of XRn_ENA_OUT for `channel` (1 = HIGH/commanded
 * on, 0 = LOW). Backs `XREX:CHANnel:ENAOut? <ch>` (commands.c). Returns
 * 0 for an out-of-range `channel` (same "can't tell, so say no" as
 * every other boolean query in this file). */
uint8_t XrexIo_GetEnableOutput(uint8_t channel);

/* Sets XRn_CONTACT_OUT for `channel` (0-based) HIGH (`on` != 0) or LOW.
 * Backs `XREX:CHANnel:CONTactOut <ch> <0|1>` (commands.c). Same
 * conventions as XrexIo_SetEnableOutput() above. */
void XrexIo_SetContactorOutput(uint8_t channel, uint8_t on);

/* Raw current level of XRn_CONTACT_OUT for `channel`. Backs
 * `XREX:CHANnel:CONTactOut? <ch>` (commands.c). Same conventions as
 * XrexIo_GetEnableOutput() above. */
uint8_t XrexIo_GetContactorOutput(uint8_t channel);

/* 1 if EVERY currently-enabled channel (PID_GetChannelEnable()) has
 * both its ENA_OUT and CONTACT_OUT currently HIGH; 0 if any enabled
 * channel is missing either one. A channel with no participating
 * output at all (disabled) is simply skipped -- not checked, not a
 * reason to refuse. Pure check, no side effects -- called from
 * ArmConditionsMet() (state_machine.c); see state_machine.h's own
 * enable-output section for the full design. */
uint8_t XrexIo_EnableOutputsReadyToArm(void);

/* Continuous version of the check above -- called from the SAME tick
 * cadence XrexIo_PollOcpFaults() already gets (main.c's boot + main
 * loop, pid.c's PID_Update()). Does nothing at all (not even a per-
 * channel loop) unless SM_GetState() is currently SM_STATE_ARMED or
 * SM_STATE_FIRING -- per direct instruction, this precondition is only
 * meaningful once actually armed, unlike Water/Temp/Enerpro/OCP's
 * always-on checking. Calls SM_ReportEnableOutputFault(channel)
 * (state_machine.h) for the first currently-enabled channel found with
 * either output no longer HIGH. */
void XrexIo_PollEnableOutputFaults(void);

#ifdef __cplusplus
}
#endif

#endif /* __XREX_IO_H__ */
