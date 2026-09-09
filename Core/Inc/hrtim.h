#ifndef __HRTIM_H__
#define __HRTIM_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "ctrlr_config.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_hrtim.h"
#include <stdint.h>

/*
 * hrtim.c
 *
 * Ported from the sibling PFM-STM32G474 project's hrtim.c -- 3-phase
 * (Timers A/B/C) complementary PWM generation on HRTIM1, wired to the
 * same U/V/W phases that project drives, plus (2026-09-04) Timers D/E/F
 * initialized the same way -- dead time enabled, complementary outputs
 * -- as pin/electrical reservations for this board's 3 additional
 * HRTIM channels (see docs/pin_mapping_v4.csv).
 *
 * GENERALIZED (2026-09-08): the channel count actually driven,
 * phase-locked, is now HRTIM_NUM_CHANNELS (ctrlr_config.h), 1-5, not a
 * hardcoded 3. Channels 0..HRTIM_NUM_CHANNELS-1 (A.. up to E) are
 * phase-locked to the Master timer (channel 0 via MASTER_PER, channels
 * 1..N-1 via MASTER_CMP1..MASTER_CMP(N-1)); channels
 * HRTIM_NUM_CHANNELS..5 (up through F) stay initialized -- dead time
 * enabled, pins reserved -- but free-running/unlocked and never
 * started by HRTIM1_PWM_Start(), exactly as D/E/F behaved before this
 * config existed for the N=3 default. A 6th phase-locked channel (F)
 * is NOT supported by this scheme -- see ctrlr_config.h's
 * HRTIM_NUM_CHANNELS comment for why and what would be needed.
 *
 * FAULT INPUT (2026-09-08): PC10 / HRTIM1_FLT6 (docs/pin_mapping_v4.csv)
 * is wired up as a REAL HARDWARE fault input, not a software-polled
 * one -- see HRTIM1_FullInit()'s fault-config block and
 * HRTIM1_FaultIsTripped()/HRTIM1_FaultClear() below. Deliberately NOT
 * the sibling V3 project's HRTIM1_EmergencyStop()/fault_pins.c
 * approach (a GPIO fault-sensing pin polled in software, which then
 * forces outputs off): this pin is one of HRTIM1's own FLTx inputs, so
 * the peripheral forces every fault-enabled channel's outputs to their
 * safe (INACTIVE) level autonomously, in silicon, the instant the pin
 * trips -- no CPU, no interrupt, no polling latency, and it keeps
 * working even if the CPU is hung. What software DOES still need to do
 * (PFM_CycleBoundaryHandler(), pfm.c) is just bookkeeping: stop the
 * counters so the master-rep ISR doesn't spin forever, and reflect the
 * trip in PFM_GetState() -- the actual output-safing has already
 * happened by the time any of that runs.
 *
 * NOT ported (out of scope, per project decision, 2026-08-31):
 *   - Anything FEEDBACK-mode related (that lived in pfm.c/feedback.c,
 *     not hrtim.c, and was never in this file to begin with).
 */

extern HRTIM_HandleTypeDef hhrtim1;

/* Core timing -- see the sibling project's hrtim.h for the full
 * derivation (170 MHz HRTIM clock from PLL math, confirmed against the
 * CONFIG command's live SYSCLK readout there; 17 counts of dead time at
 * 170 MHz = 100 ns). Carried over unchanged; this board's clock tree is
 * the same STM32G474 PLL configuration. */
#define HRTIM_TIMER_CLK_HZ      170000000U
#define HRTIM_DEADTIME_NS       100U
#define HRTIM_DEADTIME_COUNTS   17U

/* Master timebase for the PID control-loop heartbeat -- added
 * 2026-09-09, see ctrlr_config.h's PID_LOOP_RATE_HZ for the full
 * reasoning and hrtim.c's HRTIM1_FullInit() for where these are
 * applied. /4 prescale chosen as the smallest (fastest, finest-grained)
 * prescale that still fits PID_LOOP_RATE_HZ's default (1 kHz) inside
 * HRTIM's 16-bit PER register -- /1 or /2 would overflow (170 MHz or
 * 85 MHz respectively, divided by 1 kHz, both exceed 65535); /4
 * (42.5 MHz) gives exactly 42500 counts at 1 kHz, with headroom to go
 * slower (down to ~649 Hz at /4 before needing a coarser prescale) if
 * PID_LOOP_RATE_HZ is ever lowered further. Raise the divider (DIV8,
 * DIV16, ...) if PID_LOOP_RATE_HZ is ever set below that floor -- this
 * is NOT auto-selected, matching this project's existing
 * "compile-time constant, not auto-derived" philosophy elsewhere
 * (HRTIM_NUM_CHANNELS, HRTIM_MAX_CARRIER_FREQ_HZ). */
#define HRTIM_MASTER_PID_PRESCALE   HRTIM_PRESCALERRATIO_DIV4
#define HRTIM_MASTER_PID_PRESCALE_DIV  4U
#define HRTIM_MASTER_PID_PERIOD    ((uint16_t)((HRTIM_TIMER_CLK_HZ / HRTIM_MASTER_PID_PRESCALE_DIV / PID_LOOP_RATE_HZ) - 1U))

/* Safe compare clamping margin */
#define HRTIM_COMPARE_MIN           ((uint16_t)2U)

/* PC10 / HRTIM1_FLT6 (docs/pin_mapping_v4.csv), active-low (confirmed
 * against the actual fault-sensing circuit, 2026-09-08 -- do not
 * assume this generalizes to some other polarity without checking the
 * hardware again). See HRTIM1_FullInit()'s fault-config block. */
#define HRTIM_FAULT_CHANNEL     HRTIM_FAULT_6
#define HRTIM_FAULT_FLAG        HRTIM_FLAG_FLT6

void HRTIM1_FullInit(void);

/* Enables the HRTIM1 Master-repetition interrupt (HRTIM1_Master_IRQn),
 * which drives PFM_CycleBoundaryHandler() (stm32g4xx_it.c's
 * HRTIM1_Master_IRQHandler()) once per PWM period while the Master
 * counter is running. Ported from the sibling PFM-STM32G474 project's
 * HRTIM1_EnableMasterInterrupt() (main.c there) -- same priority
 * scheme (SysTick=0 highest, HRTIM1_Master=1, USART2=2), same
 * call-once-at-boot placement, so it's ready and waiting before the
 * first FIRE rather than being armed reactively at fire time. The ISR
 * itself is a no-op with respect to actual switching until
 * HRTIM1_PWM_Start() has been called (by cmd_fire() -> PFM_Restart()).
 *
 * Must be called after HAL_Init() has set up the NVIC and after
 * FixSysTickPriority() (main.c) has raised SysTick off its HAL default
 * of priority 15 -- see that function's own doc comment for the
 * priority-inversion window this ordering avoids. */
void HRTIM1_EnableMasterInterrupt(void);

/* Starts HRTIM outputs, selectively enabling only the channels
 * requested. `channelEnabled` must point to exactly HRTIM_NUM_CHANNELS
 * uint8_t flags (index 0..N-1, matching PFM_Step_t's `cmp[]` ordering).
 * A channel passed as 0 (disabled) never has its outputs enabled --
 * its HRTIM timer counter still runs (kept synchronized with the
 * Master for coherent-update correctness if the channel is re-enabled
 * later), but no output pin for that channel is driven. */
void HRTIM1_PWM_Start(const uint8_t *channelEnabled);
void HRTIM1_PWM_Stop(void);

/* Reads the HRTIM1_FLT6 status flag directly from hardware (ISR
 * register) -- NOT a software-tracked/debounced copy. Since the fault
 * channel is configured (HRTIM1_FullInit()) to force every
 * fault-enabled channel's outputs to their safe INACTIVE level the
 * instant the pin trips, by the time this ever reads 1 the outputs
 * are already safe -- this function exists for status reporting
 * (commands.c's FAULT?) and to let PFM_CycleBoundaryHandler() notice
 * and stop cleanly, not to trigger the protection itself. */
uint8_t HRTIM1_FaultIsTripped(void);

/* Stops the Master/channel counters and disconnects outputs
 * (HRTIM1_PWM_Stop() -- a fault mid-shot must not leave the master-rep
 * ISR spinning forever, same reasoning as that function's own doc
 * comment), then clears the latched HRTIM1_FLT6 flag. Leaves the
 * peripheral in exactly the same clean-stopped state a normal shot's
 * table exhaustion already leaves it in -- does NOT reconnect/restart
 * outputs itself (that's HRTIM1_PWM_Start()'s job, via a fresh,
 * explicit FIRE -> PFM_Restart(), not duplicated here). */
void HRTIM1_FaultClear(void);

/* Force an immediate transfer of shadow (preload) registers into
 * active registers for the Master timer and all HRTIM_NUM_CHANNELS
 * active slave timers.
 *
 * Called at the start of every shot (by PFM_Restart(), before counters
 * begin) to ensure the first PWM period runs with the correct PER/CMP/
 * phase values rather than the stale init defaults from HRTIM1_FullInit().
 * During the shot, the normal repetition-event preload transfer handles
 * subsequent updates without any help; this is a cold-start fix only.
 *
 * Uses the HRTIM_CR2 global software-update bits (MSWU for Master,
 * TxSWU for each slave).  A write to CR2 bypasses the HAL state machine
 * and lock -- safe here because no HRTIM operation is in flight when this
 * is called (counters are stopped, no HAL calls are active). */
void HRTIM1_SoftwareUpdate(void);

/* Writes a new period (and a recomputed 50%-duty CMP1, see this
 * function's own comment in hrtim.c for why that's not a fixed tick
 * count) into ONE channel's shadow registers -- pid.c's per-heartbeat
 * output write, Possibility 3's core primitive. Always safe to call,
 * at any time, regardless of what that channel's counter is currently
 * doing -- the hardware promotes shadow to active at THAT channel's
 * own next roll-over (ResetUpdate=ENABLED, see HRTIM1_FullInit()), not
 * at the moment this function runs. `channel` out of range
 * (>= HRTIM_NUM_CHANNELS) is a no-op. Replaces the old
 * HRTIM1_ApplyPfmStep() (one shared `per` + a phase-offset array
 * across all active channels) -- that function assumed a phase-locked
 * multi-channel carrier this project no longer drives; see git history
 * (WHAM-PFMG474-V4, this project's own base) if that's ever needed
 * again. */
void HRTIM1_SetChannelPeriod(uint8_t channel, uint16_t per);

/* COMPATIBILITY SHIM, 2026-09-09 -- kept only so pfm.c's inherited
 * table-engine (TABLE:STEP/FIRE, not this project's point, not yet
 * removed) still links and does something sane on real hardware, not
 * this project's actual output path (pid.c's per-heartbeat
 * HRTIM1_SetChannelPeriod() calls are). `cmp`/`phase` are IGNORED --
 * see the .c file's own comment for why neither means anything under
 * Possibility 3 -- every active channel just gets the same `per`,
 * independently, no interleave. */
void HRTIM1_ApplyPfmStep(uint16_t per, const uint16_t *cmp, const uint16_t *phase);

/* Register access helpers -- indexed by channel (0..HRTIM_NUM_CHANNELS-1
 * for the per-timer getters) rather than one named function per
 * channel, so callers don't need to grow with N. Master's own PER
 * register is channel-independent; its CMPx registers are indexed
 * 1..4 (matching MCMP1R-MCMP4R, used for channels 1..4's phase). */
volatile uint32_t *HRTIM1_GetMasterPerRegAddress(void);
volatile uint32_t *HRTIM1_GetMasterCmpRegAddress(uint8_t masterCompareUnit);

volatile uint32_t *HRTIM1_GetTimerPerRegAddress(uint8_t channel);
volatile uint32_t *HRTIM1_GetTimerCmp1RegAddress(uint8_t channel);

#ifdef __cplusplus
}
#endif

#endif /* __HRTIM_H__ */
