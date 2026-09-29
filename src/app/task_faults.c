/*
 * task_faults.c -- fault detection that must run regardless of state.
 */
#include "tasks.h"
#include "state_machine.h"
#include "xrex_io.h"

void TaskFaults_Poll(void)
{
    /* See state_machine.h's own SM_PollFaults() comment for why this needs
       to run here too, not just from PID_Update() (which only runs while
       FIRING). Cheap: both underlying reads are simple flag checks, not
       full re-scans. XrexIo_PollOcpFaults() (xrex_io.h, added 2026-09-17)
       runs at this same cadence for the same reason -- OCP is polled, not
       EXTI-driven (see xrex_io.h's own header comment for why), so it
       needs this same "regardless of state" call site to work at all. */
    SM_PollFaults();
    XrexIo_PollOcpFaults();
    XrexIo_PollEnableOutputFaults();   /* added 2026-09-17 -- same
                                           "regardless of state" call site
                                           reasoning, but itself only acts
                                           while ARMED/FIRING (xrex_io.h) */
}
