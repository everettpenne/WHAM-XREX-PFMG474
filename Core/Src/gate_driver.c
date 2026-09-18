/*
 * gate_driver.c
 *
 * See gate_driver.h for scope. GPIOE clock enable + pin/EXTI/NVIC
 * config (input, interrupt on both edges, matching the sibling
 * PFM-STM32G474 project's gpio.c precedent for these same 12 pins as
 * far as the plain-input part goes) lives in main.c's MX_GPIO_Init()
 * -- this file is just the read and the fault evaluation/latch.
 */

#include "gate_driver.h"
#include "ctrlr_config.h"
#include "hrtim.h"
#include "pfm.h"
#include "pid.h"
#include "main.h"
#include "xrex_io.h"

#define GDS_PIN_MASK   (GPIO_PIN_0  | GPIO_PIN_1  | GPIO_PIN_2  | GPIO_PIN_3  | \
                        GPIO_PIN_4  | GPIO_PIN_5  | GPIO_PIN_6  | GPIO_PIN_7  | \
                        GPIO_PIN_8  | GPIO_PIN_9  | GPIO_PIN_10 | GPIO_PIN_11)

/* Set only from GateDriver_CheckFault() (ISR context, EXTI0..EXTI4,
   EXTI9_5, EXTI15_10 -- see stm32g4xx_it.c), cleared only from
   GateDriver_FaultClear() (main-loop context, commands.c's
   FAULT:CLEAR). volatile: written from ISR, read from the main loop
   via GateDriver_FaultIsLatched(). */
static volatile uint8_t g_gdsFaultLatched = 0U;

uint16_t GateDriver_Read(void)
{
    return (uint16_t)(GPIOE->IDR & GDS_PIN_MASK);
}

void GateDriver_CheckFault(void)
{
    uint16_t raw = GateDriver_Read();

    /* Enerpro's own per-channel check, added 2026-09-18 -- see
       xrex_io.h's own header comment on the reclassification (Enerpro
       now gets an individual derate-and-ramp response, like OCP, not
       the shared full-stop below). Deliberately called BEFORE the
       Water+Temp check, on the SAME raw read (no redundant GPIO
       access) -- independent of, and not gated by, whatever the
       Water+Temp check below decides. */
    XrexIo_PollEnerproFaults(raw);

    /* Fault decision delegated to xrex_io.c, 2026-09-17 -- see that
       file's own header comment for the full reasoning: these 12 pins
       are now interpreted with per-Transrex-channel semantics
       (XRn_WATER_FLT/_TMP_FLT, docs/pin_mapping_v4.csv's new "XREX Pin
       Name" column -- Enerpro handled separately above as of
       2026-09-18), each category independently polarity-configurable
       (ctrlr_config.h's XR_WATER_FLT_POLARITY/XR_TMP_FLT_POLARITY --
       REPLACES the old single shared GDS_FAULT_POLARITY this function
       used to check directly), and gated so a disabled channel's own
       pins never count toward a fault. This function's own EXTI-
       trigger/latch/PFM_ForceStop(Soft) mechanics below are otherwise
       UNCHANGED -- still the same "any real Water/Temp fault among
       these pins" response, just a smarter decision of what counts as
       one. */
    if (XrexIo_EvaluateGateDriverFault(raw) != 0U)
    {
        /* *** REAL BUG, FIXED 2026-09-15, confirmed on real hardware ***
           -- see PFM_ForceStopSoft()'s own extensive doc comment
           (pfm.h) for the full diagnostic. This used to call the FULL
           PFM_ForceStop() unconditionally, here, from the EXTI ISR, the
           INSTANT a real GateDriverStatus pin trips -- BEFORE
           state_machine.c's SM_PollFaults()/EnterFault() ever get a
           chance to run and decide whether a General-Fault ramp-down
           should begin. That full stop kills the shared HRTIM Master/
           channel counters the ramp-down needs, so even after
           EnterFault()'s OWN fix (same date, state_machine.c), a REAL
           GateDriverStatus-triggered fault would still have arrived
           with the hardware already dead.

           Fixed: soft-stop (no counter touch) when pid.c currently has
           an active shot (PID_IsRunning()) -- state_machine.c's very
           next SM_PollFaults() call (main loop, every iteration, or
           PID_Update() itself -- whichever reaches it first, within at
           most one ~1ms Master tick) will pick this fault up and decide
           ramp-down vs. immediate stop from there, needing these same
           counters. When PID is NOT running (the legacy pfm.c
           TABLE:FIRE path is what's actually active, or nothing is),
           the full stop is still correct and unchanged -- there is no
           OTHER mechanism that will ever safe that hardware otherwise,
           since the legacy path has no ramp-down concept at all. */
        if (PID_IsRunning() != 0U)
        {
            PFM_ForceStopSoft();
        }
        else
        {
            PFM_ForceStop();
        }
        g_gdsFaultLatched = 1U;
    }
}

uint8_t GateDriver_FaultIsLatched(void)
{
    return g_gdsFaultLatched;
}

void GateDriver_FaultClear(void)
{
    PFM_ForceStop();
    g_gdsFaultLatched = 0U;

    /* Re-validate immediately, rather than trusting the clear to mean
       the physical condition is actually gone. This matters because
       GateDriver_CheckFault() is only ever invoked by an EXTI EDGE
       (or the one explicit boot-time call in main.c) -- unlike
       PC10/HRTIM1_FLT6's own hardware fault latch, which re-trips on
       its own if its condition is still physically present, a
       cleared-but-still-bad GateDriverStatus pin produces no new edge
       by itself and would otherwise sit silently unlatched (FAULT?
       reporting OK 0) until something eventually toggled it again.
       Calling CheckFault() here closes that gap: if any pin is still
       in its fault state, this immediately re-latches (and re-calls
       PFM_ForceStop(), a harmless repeat) before FAULT:CLEAR's OK
       reply is even sent -- an operator can never clear away a fault
       that is still physically present. */
    GateDriver_CheckFault();
}
