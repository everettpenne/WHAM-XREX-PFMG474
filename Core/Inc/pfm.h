#ifndef __PFM_H__
#define __PFM_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "ctrlr_config.h"
#include <stdint.h>

/*
 * pfm.h / pfm.c
 *
 * Ported from the sibling PFM-STM32G474 project's pfm.c -- the table
 * PLAYBACK engine only, per project decision (2026-08-31):
 *
 *   PORTED:   PFM_Step_t, the table itself, and everything that walks
 *             it and writes HRTIM registers each cycle (PFM_Init(),
 *             PFM_ApplyCurrentStep(), PFM_CycleBoundaryHandler(),
 *             PFM_Restart(), PFM_ResetIndices(), PFM_GetState(),
 *             PFM_GetCurrentStep(), PFM_SetPhaseEnabled()/
 *             PFM_GetPhaseEnabled(), PFM_Phase120()/PFM_Phase240()).
 *
 *   NOT PORTED: the sibling project's SWEEP segment/track builder
 *             (PFM_BuildTable(), PFM_FreqInit/Seg, PFM_DutyInit_x/
 *             Seg_x, the interpolation math) and everything
 *             FEEDBACK-mode related. No on-controller construction
 *             logic exists, and none is planned -- see below.
 *
 *   ADDED (2026-09-04), NOT a port: PFM_TableReset()/PFM_AppendStep()/
 *             PFM_GetEntryCount() -- minimal, generic primitives so a
 *             serial command layer (commands.c's TABle:* commands) can
 *             write a complete table built entirely off-controller, by
 *             a host-side Python script (python/pfm_table_upload.py).
 *             Per project decision: table CONSTRUCTION lives on the
 *             host, not the firmware -- these three functions do no
 *             validation of what a step "means" (no frequency/duty
 *             math, no bounds tied to a supply type, none of that
 *             exists here), only bounds-checking that the table itself
 *             isn't overrun. commands.c's cmd_table_step() owns
 *             whatever validation is appropriate at the protocol
 *             layer. If a real on-controller builder is ever added
 *             later, it would use this exact same API.
 *
 *   GENERALIZED (2026-09-08): PFM_Step_t's per-channel compare values
 *             (previously named cmpA/cmpB/cmpC, hardcoded to 3
 *             channels) are now a `cmp[HRTIM_NUM_CHANNELS]` array
 *             sized by the compile-time channel count in
 *             ctrlr_config.h -- see that file and hrtim.h for the full
 *             explanation. PFM_Phase120()/PFM_Phase240() (3-channel-
 *             only) became PFM_PhaseForChannel() (any channel, any N).
 *             PFM_Phase_t/PFM_SetPhaseEnabled()/PFM_GetPhaseEnabled()
 *             switched from a 3-member enum to a plain 0..N-1 channel
 *             index -- PFM_PHASE_U/V/W remain as named conveniences
 *             for callers that want them, valid whenever N is large
 *             enough.
 *
 * Also not ported: PFM_SetStartupFreq()/Duty()/PulseLength() and their
 * g_startup* state -- those exist in the sibling project purely to feed
 * the table builder (SET FREQ/DUTY/PULSE commands write them, and the
 * builder falls back to them), so they have no purpose here without a
 * builder to feed. Same reasoning for supply_config.h's ACTIVE_*
 * default macros -- this project doesn't have a SUPPLY_TYPE concept at
 * all yet.
 */

/* --------------------------------------------------------------------------
 * Hold periods
 * Number of PWM cycles to hold each table entry before stepping to next
   1 = update every period
   N = update every N periods.
 * -------------------------------------------------------------------------- */
#define PFM_HOLD_PERIODS                         (1U)

/* --------------------------------------------------------------------------
 * Table size. The sibling project derives this from its own supply_config.h
 * (SUPPLY_PFM_TABLE_SIZE, currently 5000 there, chosen for a specific RAM
 * budget history tied to features not present here) -- defined directly
 * here instead, at the same value, since there's no equivalent config
 * source yet. Revisit if/when this project grows its own RAM-budget
 * pressure worth tracking. PFM_Step_t's size now depends on
 * HRTIM_NUM_CHANNELS (2 + 2*N bytes -- see ctrlr_config.h), so total
 * static RAM for the table is PFM_TABLE_SIZE * (2 + 2*HRTIM_NUM_CHANNELS)
 * bytes, not a fixed 8 bytes/entry regardless of channel count anymore.
 * At N=5 and PFM_TABLE_SIZE=5000 that's 60,000 B -- confirmed against a
 * real .map file to comfortably fit this MCU's 128 KiB SRAM alongside
 * everything else (see docs/changelog.txt, 2026-09-08).
 * -------------------------------------------------------------------------- */
#define PFM_TABLE_SIZE                           (5000U)

/* --------------------------------------------------------------------------
 * PFM_Step_t -- one playback entry. `cmp[]` holds one compare value per
 * active channel (HRTIM_NUM_CHANNELS of them, see ctrlr_config.h) --
 * sized exactly for whatever channel count this build is configured
 * for, no waste. See the sibling project's pfm.h for the RAM-budget
 * history behind the original 3-channel layout this generalizes
 * (phaseB/phaseC/freqHz were removed there in favor of recomputing
 * them from `per` at apply time; the same principle applies here --
 * phase is still never stored, only per-channel duty).
 * -------------------------------------------------------------------------- */
typedef struct
{
    uint16_t per;
    uint16_t cmp[HRTIM_NUM_CHANNELS];
} PFM_Step_t;

/* Pure function of `per` and a channel index: the Master-timer phase
 * offset (in HRTIM counts) for that channel at this period, evenly
 * spaced across HRTIM_NUM_CHANNELS channels. `channel` 0 has no phase
 * register of its own (it's the Master-PER reference channel, 0 deg by
 * definition) -- only call this for `channel` in 1..HRTIM_NUM_CHANNELS-1. */
uint16_t PFM_PhaseForChannel(uint16_t per, uint8_t channel);

typedef enum
{
    PFM_STATE_RUNNING = 0U,
    PFM_STATE_STOPPED
} PFM_State_t;

/* Named conveniences for the first three channels -- still meaningful
 * whenever HRTIM_NUM_CHANNELS >= 3 (true for every profile this
 * project has used so far). PFM_SetPhaseEnabled()/PFM_GetPhaseEnabled()
 * take a plain uint8_t channel index (0..HRTIM_NUM_CHANNELS-1), not
 * this enum type, so callers with more than 3 channels configured can
 * still address channel 3, 4, etc. directly by number. */
typedef enum
{
    PFM_PHASE_U = 0U,
    PFM_PHASE_V,
    PFM_PHASE_W
} PFM_Phase_t;

void PFM_Init(void);
void PFM_ApplyCurrentStep(void);
void PFM_CycleBoundaryHandler(void);
void PFM_Restart(void);          /* re-arm from external trigger: resets indices, applies step 0, starts outputs */
void PFM_ResetIndices(void);     /* quiescent index reset only, no register/output writes (e.g. fault clear) */
PFM_State_t PFM_GetState(void);  /* optional, for debug/status */

/* Immediately stops HRTIM output (HRTIM1_PWM_Stop()) and marks
 * PFM_GetState() STOPPED -- the exact pair of actions
 * PFM_CycleBoundaryHandler() already performs, from ISR context, when
 * it notices a latched HRTIM1_FLT6 fault (hrtim.h) or an empty table.
 * Extracted as its own function (2026-09-08) so any OTHER fault source
 * -- e.g. gate_driver.c's GateDriver_CheckFault(), called from the
 * GateDriverStatus EXTI interrupts, which run independently of the
 * HRTIM master-repetition ISR -- can force the same clean stop and
 * keep PFM_GetState() truthful, instead of calling HRTIM1_PWM_Stop()
 * directly and leaving g_pfmState stale at RUNNING. Does NOT reset
 * g_pfmIndex (matching the two existing early-stop call sites inside
 * PFM_CycleBoundaryHandler(), which never did either) and does NOT
 * clear any fault latch itself -- callers own their own latch. Safe to
 * call from ISR context; safe to call repeatedly (HRTIM1_PWM_Stop() is
 * idempotent). */
void PFM_ForceStop(void);

/* Same as PFM_ForceStop() -- SAME g_pfmState/PfmInput_OnShotEnd()
 * bookkeeping -- WITHOUT the HRTIM1_PWM_Stop() call, i.e. does NOT
 * touch the shared HRTIM Master/channel counters. Added 2026-09-15,
 * found and confirmed on real hardware while verifying the new OCP
 * fault handling (state_machine.h):
 *
 * PFM_ForceStop()'s HRTIM1_PWM_Stop() call stops the Master counter
 * (by design -- see that function's own doc comment: "leaving the
 * counters running... means the Master repetition ISR keeps firing
 * forever"). That's correct when nothing else needs the Master ISR
 * afterward. But state_machine.c's EnterFault() called the FULL
 * PFM_ForceStop() UNCONDITIONALLY, for every fault, BEFORE either fault
 * handler even runs -- including the case where a PID fault ramp-down
 * (General OR Overcurrent, whichever hit while FIRING) is about to
 * begin, and NEEDS that same Master counter to keep running for up to
 * FAULT_RAMP_DOWN_TIME_S more seconds (PID_Update() is what drives
 * ProcessFaultRampDown()/the OCP equivalent, one tick at a time, and
 * ONLY fires from the Master's own repetition interrupt). Symptom,
 * confirmed on real hardware with a new debug query: the ramp's
 * SOFTWARE state (lastOutputHz/setpointHz) correctly interpolates every
 * call, and the derated/ramped values get correctly WRITTEN to each
 * channel's shadow registers -- but since the counter that would ever
 * PROMOTE a shadow write into the active register never rolls over
 * again, PID_Update() itself is never called a second time at all
 * (elapsed ticks measured stuck at 0, indefinitely) -- the REAL
 * PHYSICAL output freezes at whatever static level the counter
 * happened to be at, for the entire ramp window, not a smooth ramp to
 * zero. gate_driver.c's GateDriver_CheckFault() has the same issue for
 * its own PFM_ForceStop() call, run from the EXTI ISR the instant a
 * real GateDriverStatus pin trips, BEFORE state_machine.c's own
 * EnterFault() even runs.
 *
 * Used at exactly those two call sites, conditionally (only when a PID
 * ramp-down might actually need the counters -- otherwise the full
 * PFM_ForceStop() is still correct and unchanged) -- see EnterFault()
 * (state_machine.c) and GateDriver_CheckFault() (gate_driver.c) for the
 * condition each one checks. NOT used anywhere else -- every OTHER
 * existing PFM_ForceStop() caller (pfm.c's own two internal early-stop
 * cases, GateDriver_FaultClear()) is unaffected by this addition. */
void PFM_ForceStopSoft(void);

/* Per-channel output enable -- standalone, mode-independent, durable
 * state. Not reset by PFM_Init() or PFM_ResetIndices() -- only changes
 * via an explicit call to PFM_SetPhaseEnabled(). Defaults to all
 * enabled. `channel` out of range (>= HRTIM_NUM_CHANNELS) is a no-op
 * for Set and returns 0 for Get. */
void PFM_SetPhaseEnabled(uint8_t channel, uint8_t enabled);
uint8_t PFM_GetPhaseEnabled(uint8_t channel);

const PFM_Step_t *PFM_GetCurrentStep(void);
const PFM_Step_t *PFM_GetStepByIndex(uint16_t index);

/* --------------------------------------------------------------------------
 * Table upload primitives -- see the ADDED note in this file's header
 * comment. Generic on purpose: no notion of "upload session" state
 * (whether a TABLE:BEGIN was sent) lives here -- that's commands.c's
 * job, matching how the rest of this codebase keeps protocol-layer
 * state (e.g. cmd_boot()'s checks) out of the modules it calls into.
 * -------------------------------------------------------------------------- */

/* Empties the table (g_pfmEntryCount = 0) without touching HRTIM or
 * PFM_GetState() -- purely a data-structure reset, safe to call at any
 * time. Does NOT reset g_pfmIndex/g_holdCounter (PFM_ResetIndices()
 * already does that, separately, for the fault-clear/quiescent case;
 * this function is deliberately narrower). */
void PFM_TableReset(void);

/* Appends one entry at g_pfmTable[g_pfmEntryCount], then increments
 * g_pfmEntryCount. `cmp` must point to exactly HRTIM_NUM_CHANNELS
 * values -- the caller (commands.c's cmd_table_step()) is responsible
 * for having parsed that many. Returns 1 on success, 0 if the table is
 * already at PFM_TABLE_SIZE capacity (entry NOT written in that case --
 * the caller's count of "how many actually got in" should stop
 * advancing on a 0 return, not silently keep calling). No
 * range-checking on the values themselves (per/cmp accepted as given --
 * the eventual apply-time HRTIM1_ClampCompare() in hrtim.c is the last
 * line of defense there, but callers should validate before this, not
 * rely on that clamp as anything other than a backstop). */
uint8_t PFM_AppendStep(uint16_t per, const uint16_t *cmp);

/* Current g_pfmEntryCount -- how many entries PFM_AppendStep() has
 * actually written since the last PFM_TableReset(). */
uint16_t PFM_GetEntryCount(void);

/* Diagnostic, added 2026-09-09 (see pfm.c's own comment on
 * g_diagMaxGapCycles) -- real inter-call timing for
 * PFM_CycleBoundaryHandler() during the CURRENT/most recent shot,
 * reset at every PFM_Restart(). `*callCount` is how many times the
 * handler has run since that reset; `*maxGapCycles` is the largest
 * gap seen between two consecutive calls, in CPU cycles (170 MHz,
 * same clock as HRTIM_TIMER_CLK_HZ -- hrtim.h -- so directly
 * comparable to HRTIM tick counts). A gap far larger than one real
 * period's worth of cycles is direct evidence the ISR was starved for
 * that stretch, not just that the table happened to hold one entry
 * for a while. Either pointer may be NULL to skip that output. */
void PFM_GetDiagCounters(uint32_t *callCount, uint32_t *maxGapCycles);

/* Diagnostic, added 2026-09-09 (second round -- see pfm.c's own
 * comment on g_diagGapLog): the RAW inter-call gap, in CPU cycles, for
 * every PFM_CycleBoundaryHandler() call this shot, up to
 * PFM_INPUT_MAX_PERIODS (pfm_input.h) of them -- unlike
 * PFM_GetDiagCounters()'s single max-gap number, this lets a caller
 * see EXACTLY which table entries had delays, not just whether the
 * single worst one was large. Returns a pointer to the internal log
 * array (valid until the next PFM_Restart()) and writes the number of
 * valid entries in it (== min(call count, PFM_INPUT_MAX_PERIODS)) to
 * *count if non-NULL. */
const uint32_t *PFM_GetDiagGapLog(uint16_t *count);

#ifdef __cplusplus
}
#endif

#endif /* __PFM_H__ */
