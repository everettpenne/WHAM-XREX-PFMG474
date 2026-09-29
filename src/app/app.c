/*
 * app.c -- application start-up and the main loop's task list, called from
 * main.c's USER CODE blocks so the CubeMX-generated file stays generated
 * init plus a few calls (see app.h for the call order).
 */
#include "app.h"
#include "tasks.h"
#include <stdio.h>
#include "uart.h"
#include "mcu.h"
#include "flash_bank.h"
#include "boot_diag.h"
#include "hrtim.h"
#include "pfm.h"
#include "gate_driver.h"
#include "qspi_test.h"
#include "pfm_input.h"
#include "pid.h"
#include "state_machine.h"
#include "xrex_io.h"
#include "telemetry.h"
#include "sim_transrex.h"
#include "ctrlr_config.h"
#include "git_version.h"

/* Every module's init, in dependency order, then the USART2 command link.
   Runs once from main() after the CubeMX peripheral init. */
void App_Init(void)
{
    /* Brings the PFM table module to a known-empty, known-stopped state
       before anything else can touch it (a TABLE:* command over UART, or
       a FIRE). Does not start HRTIM outputs -- see PFM_Init()'s own
       comment in pfm.c. */
    PFM_Init();

    /* One explicit GateDriver_CheckFault() call, here at boot, before
       relying on the EXTI interrupt (gate_driver.c, wired up in
       board_io.c's BoardIo_Init()) for everything from here on. EXTI is
       edge-triggered: a pin that is ALREADY in its fault state (per
       GDS_FAULT_POLARITY, ctrlr_config.h) at the moment PE0..PE11 get
       configured for interrupt mode produces no edge of its own -- the
       ISR would simply never fire for it, silently, until something
       eventually toggles that pin. Confirmed relevant on this exact
       board: a GDS? snapshot taken earlier the same day this was added
       showed GateDriverStatus_03 (PE2) already HIGH, which is a fault
       under this build's NORMALLY_LOW polarity. This call catches
       exactly that case -- any fault already present at boot -- instead
       of depending on a future transition that might never come. */
    GateDriver_CheckFault();

    /* QUADSPI bring-up (PE12-PE15/PB10-PB11, W25Q128JVS) -- see
       qspi_test.h for scope. No-op when QSPI_TEST_FEATURE_ENABLED is 0,
       matching the same always-call/resolves-to-something-or-nothing
       pattern already used for BootJump_CheckAndEnter(). */
    QspiTest_Init();

    /* PFM_Input period/duty capture (PA15/PD4/PB2/PC12/PB4/PD12, TIM2/
       TIM3/TIM4/TIM5) -- see pfm_input.h. Configures the timers/GPIO
       only; does not arm or start any capture (that's PfmInput_Arm()
       via PFMIN:CAPTURE, and PfmInput_OnShotStart(), called from
       PFM_Restart() in pfm.c). No-op when PFM_INPUT_FEATURE_ENABLED is
       0. */
    PfmInput_Init();

    /* Closed-loop PID controller (pid.h) -- brings every channel's PID
       state to a known, safe-inert default (all gains 0). Does not touch
       HRTIM or PFM_Input hardware -- that's PID_Start(), via a new
       serial command (not yet added, see docs/changelog.txt). */
    PID_Init();

    /* Telemetry event stream (telemetry.h) -- Phase 1 of docs/telemetry.md.
       Just zeroes the event ring + flight recorder + sets the !EVT gate
       to ON; the actual state/fault events are pushed by state_machine.c
       and emitted each main-loop iteration by Telemetry_PollEmit() (App_Poll()).

       *** REAL BUG, FOUND AND FIXED 2026-09-23 ***: this used to run
       AFTER the SM_Init()/SM_PollFaults()/XrexIo_Poll*Faults() block
       below -- which, per that block's OWN comment, exists specifically
       to catch a fault that's already latched at power-on. When that
       happened, EnterFault() (state_machine.c) correctly pushed a FAULT
       event into both the live ring AND the flight recorder -- and this
       call, running right after, unconditionally zeroed both, silently
       erasing the one event the flight recorder (SYS:EVLOG?) exists
       specifically to survive for post-mortem review. No later call ever
       re-pushes it (every fault-report path dedupes on g_state already
       being SM_STATE_FAULT), so it was gone permanently, not delayed --
       defeating the flight recorder's own stated purpose for exactly the
       case it matters most. Moved before the boot-time fault poll below
       so any pre-existing fault's event survives it. */
    Telemetry_Init();

    /* Top-level operating-state machine (state_machine.h), added
       2026-09-13 -- see that header for the full design (IDLE/ARMED/
       FIRING/FAULT). SM_Init() alone would set IDLE unconditionally,
       which would be WRONG if the GateDriver_CheckFault() call above
       (deliberately earlier -- boot-time GateDriverStatus
       check, before this state machine even existed) already found a
       real pre-existing fault: an immediate SM_PollFaults() right after
       SM_Init() picks that up, so a board that boots with a fault
       already present correctly starts in FAULT, not IDLE. XrexIo_PollOcpFaults()
       (xrex_io.h, added 2026-09-17) is called right alongside it for the
       same reason -- a board that boots with a real OCP condition already
       present should also start in FAULT, not IDLE. */
    SM_Init();
    SM_PollFaults();
    XrexIo_PollOcpFaults();
    XrexIo_PollEnableOutputFaults();   /* no-op here -- SM_Init() just set
                                           IDLE, and this check only ever
                                           does anything while ARMED/FIRING
                                           (see xrex_io.h) -- called anyway
                                           for the same "same cadence as
                                           everything else" consistency */

    uart_start(&uart2);   /* bound to USART2 by main.c (uart_bind()) */

    /* The HRTIM master-repetition interrupt must be enabled now, at
       boot, even though outputs are not yet running: PFM_CycleBoundaryHandler()
       needs to be wired up and ready before the first FIRE, not armed
       reactively at fire time. The ISR itself is a no-op with respect to
       actual switching until HRTIM1_PWM_Start() has been called (by
       cmd_fire() -> PFM_Restart()). Ported from the sibling
       PFM-STM32G474 project's main.c, same placement/rationale. */
    HRTIM1_EnableMasterInterrupt();

    /* Transrex simulator logic (sim_transrex.h) -- SIMULATOR-ONLY, added
       2026-09-18. Starts this board's own HRTIM output + PFM_Input
       capture unconditionally (not tied to this board's own ARM/FIRE
       state -- see sim_transrex.h's own comment for why) and drives
       every fault-injection transmitter to its healthy default. No
       controller-target equivalent -- this board plays a genuinely
       different physical role. */
#if defined(BUILD_TARGET_SIMULATOR)
    SimTransrex_Init();
#endif
}

/* Unsolicited boot banner, added 2026-09-25 for a firmware-update
   regression test: FWUPdate:SWAP/ROLLback reboot the board, and this
   line is the exact, unambiguous signal (on the host side, just poll
   the link for it) that a reboot has completed and USART2 is live
   again -- cheaper and more precise than polling *IDN? in a loop.
   Sent exactly once, from main() after App_Init() has run but
   before the main loop starts -- so BANK/BFB2/STATE below reflect
   the true post-init state, not a boot-time snapshot from earlier.
   "!BOOT" (not "OK"/"ERR"/"!EVT") so host tooling can never confuse
   it with a command reply or a state_machine.c telemetry event.
   Plain uart_send(), not telemetry.c -- this must go out even if
   Telemetry_Init() (App_Init()) or the event ring/gate is ever
   changed; a boot signal that could be silently gated off is not
   a signal a host can rely on. */
void App_SendBootBanner(void)
{
    char banner[200];
    uint8_t bank    = FlashBank_Active();
    uint8_t bfb2    = FlashBank_Bfb2();
    const char *stateName;
    switch (SM_GetState())
    {
        case SM_STATE_IDLE:   stateName = "IDLE";   break;
        case SM_STATE_ARMED:  stateName = "ARMED";  break;
        case SM_STATE_FIRING: stateName = "FIRING"; break;
        case SM_STATE_FAULT:  stateName = "FAULT";  break;
        default:              stateName = "UNKNOWN"; break;
    }
    snprintf(banner, sizeof(banner),
             "!BOOT %s %s %s%s BANK=%u BFB2=%u STATE=%s tick=%lu\r\n",
             HW_BOARD_NAME, FW_VERSION_STRING, FW_GIT_COMMIT,
             (FW_GIT_DIRTY != 0U) ? "-dirty" : "",
             (unsigned)bank, (unsigned)bfb2, stateName,
             (unsigned long)Mcu_GetTickMs());
    uart_send(&uart2, banner);

    /* Previous boot's record and this boot's reset cause (boot_diag.c). */
    BootDiag_FormatReport(banner, sizeof(banner));
    uart_send(&uart2, banner);
    uart_send(&uart2, "!BOOT Rise and shine, controller's awake and ready to work \xF0\x9F\x8C\x9E\r\n");
}

/* One main-loop iteration. Order matters: fault detection first, the
   command link last. */
void App_Poll(void)
{
    TaskFaults_Poll();

    /* Emit any pending telemetry events (telemetry.h) as unsolicited
       !EVT lines. Bounded (at most a few events per call) and only ever
       runs at thread priority, so it cannot disturb the 1 kHz loop or any
       fault ISR. */
    Telemetry_PollEmit(&uart2);

#if defined(BUILD_TARGET_SIMULATOR)
    SimTransrex_Update();   /* added 2026-09-18 -- SIMULATOR-ONLY, same
                                "regardless of state" main-loop cadence;
                                see sim_transrex.h for why this can't
                                run from PID_Update() instead (that only
                                ticks while THIS board's own HRTIM
                                Master is active, i.e. only during a
                                FIRE on this board -- irrelevant here) */
#endif

    TaskScpi_Poll();
}
