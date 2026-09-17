#ifndef __GATE_DRIVER_H__
#define __GATE_DRIVER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* --------------------------------------------------------------------------
 * gate_driver.c
 *
 * The 12 GateDriverStatus_01..12 pins (PE0..PE11, GPIOE,
 * docs/pin_mapping_v4.csv). Two independent pieces of functionality:
 *
 *  1. Raw diagnostic readback -- GateDriver_Read(), backing commands.c's
 *     GDS? command. ADDED 2026-09-08 while investigating why a fault
 *     wasn't being registered: these pins were previously never even
 *     configured as GPIO inputs (MX_GPIO_Init() only enabled GPIOA).
 *     Polarity-agnostic: just the live IDR bits, no interpretation.
 *
 *  2. EXTI-driven fault protection -- GateDriver_CheckFault(), called
 *     from the 7 EXTI IRQ handlers (stm32g4xx_it.c) that PE0..PE11's
 *     interrupt lines span (EXTI0..EXTI4 individually, EXTI9_5 and
 *     EXTI15_10 shared). Re-reads all 12 pins on every edge and, as of
 *     2026-09-17, delegates the actual fault DECISION to
 *     XrexIo_EvaluateGateDriverFault() (xrex_io.h/.c) -- these 12 pins
 *     are docs/pin_mapping_v4.csv's XR1..XR4 _WATER_FLT/_TMP_FLT/
 *     _ENERPRO_FLT signals, each category independently polarity-
 *     configurable and gated so a disabled channel's own pins don't
 *     count (see xrex_io.h's own extensive header comment for the full
 *     reasoning) -- REPLACES the old single shared GDS_FAULT_POLARITY
 *     check this function used to do directly. If that decision comes
 *     back true, immediately calls PFM_ForceStop() or PFM_ForceStopSoft()
 *     (pfm.h -- see GateDriver_CheckFault()'s own .c comment for which,
 *     and why: PFM_ForceStopSoft() when pid.c currently has an active
 *     shot, so the shared HRTIM Master/channel counters stay running
 *     for state_machine.c's own very next SM_PollFaults() call to hand
 *     off to a General-Fault ramp-down, added 2026-09-15 after a real-
 *     hardware bug confirmed the full stop was killing that ramp-down
 *     before it could ever run) and latches a software fault, queried/
 *     cleared via GateDriver_FaultIsLatched()/GateDriver_FaultClear()
 *     -- unified with the PC10/HRTIM1_FLT6 native hardware fault
 *     (hrtim.h) at the commands.c level (FAULT?/FAULT:CLEAR/FIRE's
 *     ERR 6 check both sources; see commands.c). Pin/EXTI/NVIC
 *     configuration itself lives in main.c's MX_GPIO_Init(), matching
 *     where the pins were already configured as plain inputs before
 *     this.
 *
 * Deliberately narrow, unlike the sibling PFM-STM32G474 project's
 * gate_driver.c/fault_pins.c: no debounce, no fault logging -- one
 * shared latch for whatever xrex_io.c decides counts as a fault among
 * these 12 pins (per-pin polarity/gating now lives there, not here --
 * see this file's own point 2 above). These 12 pins remain distinct
 * from, and independent of, the PC10/HRTIM1_FLT6 native hardware fault
 * mechanism: that one protects autonomously in silicon even if the CPU
 * is hung; this one needs the EXTI interrupt to actually run.
 * -------------------------------------------------------------------------- */

/* Raw, instantaneous read of PE0..PE11 -- bit i = PE(i)'s live IDR
   state (1 = pin high, 0 = pin low), i = 0..11. Not cached, not
   debounced, not polarity-adjusted: two calls back-to-back can
   legitimately differ if a pin is toggling. */
uint16_t GateDriver_Read(void);

/* Re-reads all 12 pins and asks xrex_io.c's XrexIo_EvaluateGateDriverFault()
   whether that represents a real fault (per-channel gated, per-category
   polarity -- see xrex_io.h). If so, immediately force-stops HRTIM
   output (PFM_ForceStop()) and latches the fault. Called from the 7
   EXTI IRQ handlers that share PE0..PE11's interrupt lines
   (stm32g4xx_it.c) -- safe to call from ISR context, and safe (a
   harmless no-op re-stop) to call again while already latched. */
void GateDriver_CheckFault(void);

uint8_t GateDriver_FaultIsLatched(void);

/* Force-stops HRTIM output again (defensive, matching
   HRTIM1_FaultClear()'s own idempotent-safety pattern -- see hrtim.h),
   clears the latch, then immediately re-validates by calling
   GateDriver_CheckFault() again -- unlike PC10/HRTIM1_FLT6's own
   hardware fault latch (which re-trips on its own if its condition is
   still physically present), a GateDriverStatus pin that's still bad
   produces no new EXTI edge by itself, so the clear-then-recheck here
   is what keeps a still-faulted pin from silently reporting FAULT? OK
   0 just because it was cleared. Net effect: clearing a fault that is
   still physically present re-latches it immediately, before this
   function even returns. Deliberately does NOT reconnect/restart
   outputs on a successful clear -- that's the next explicit FIRE's
   job, same convention as HRTIM1_FaultClear(). */
void GateDriver_FaultClear(void);

#ifdef __cplusplus
}
#endif

#endif /* __GATE_DRIVER_H__ */
