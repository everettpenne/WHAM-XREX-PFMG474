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
 *     All three route to SM_FAULT_GENERAL (state_machine.h) -- per
 *     direct instruction, "Water, Temperature, and Enerpro faults are
 *     considered general faults" -- no sub-type distinction at the
 *     fault-TYPE level (STATE? still just says GENERAL), only at the
 *     diagnostic-query level (XREX:CHANnel:STATus?, commands.c, below).
 *     XrexIo_EvaluateGateDriverFault() is gate_driver.c's OWN fault
 *     decision now (GateDriver_CheckFault() delegates to it) -- this
 *     module doesn't re-implement the EXTI/latch/PFM_ForceStop
 *     mechanics, gate_driver.c still owns those.
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
 * *** NOTE TO REVISIT *** -- XRn_ENA_OUT (PG0/PG1/PG2/PG3) and
 * XRn_CONTACT_OUT (PG4/PG5/PG6/PG7) are real outputs docs/pin_mapping_v4.csv's
 * new column also names, explicitly DEFERRED per direct instruction
 * ("Once we figure all this out, we will configure the outputs that I
 * suggested. Make a note of those and remind me later.") -- NOT
 * configured anywhere in this codebase yet, not even as plain GPIO
 * outputs. Remind the user about these before considering this feature
 * area complete.
 * -------------------------------------------------------------------------- */

/* Called from gate_driver.c's GateDriver_CheckFault() with the raw
 * 12-bit PE0..PE11 read (GateDriver_Read()) -- returns 1 if, after
 * per-channel gating and each category's own polarity setting, this
 * represents a real Water/Temp/Enerpro fault; 0 otherwise. Pure
 * decision function -- does not itself latch anything or touch
 * hardware; gate_driver.c's own existing latch/PFM_ForceStop/
 * EnterFault(SM_FAULT_GENERAL) mechanics are unchanged, just now gated
 * on this function's answer instead of the old single-polarity
 * badBits check. */
uint8_t XrexIo_EvaluateGateDriverFault(uint16_t raw12);

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

#ifdef __cplusplus
}
#endif

#endif /* __XREX_IO_H__ */
