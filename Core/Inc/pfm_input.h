#ifndef INC_PFM_INPUT_H_
#define INC_PFM_INPUT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "stm32g4xx_hal.h"

/*
 * pfm_input.h
 *
 * Per-period PERIOD measurement (rising-edge-to-rising-edge only, no
 * duty cycle) on the 6 PFM_Input_01..06 pins (docs/pin_mapping_v4.csv
 * -- PA15/TIM2_CH1, PD4/TIM2_CH2, PB2/TIM5_CH1, PC12/TIM5_CH2,
 * PB4/TIM3_CH1, PD12/TIM4_CH1), added 2026-09-08. Each channel is
 * measured completely independently -- different PFM_Input pins can
 * be watching entirely unrelated signals -- via plain
 * TIM_ICPOLARITY_RISING input capture, armed once at init and never
 * changed again (see pfm_input.c).
 *
 * DECISION, 2026-09-08: this module originally also measured duty
 * cycle (time-high per period), via TIM_ICPOLARITY_BOTHEDGE and later
 * an alternating-single-polarity technique -- both real-hardware
 * attempts consistently produced data offset by exactly one real
 * period, for reasons never root-caused (see pfm_input.c's top
 * comment and docs/changelog.txt for the full diagnostic history).
 * Duty-cycle measurement was dropped rather than continue chasing that
 * bug: period-only, rising-edge-only capture never changes polarity
 * inside the ISR at all, which removes the one mechanism every failed
 * attempt had in common.
 *
 * Capture is synchronized to PFM shot lifetime, not a standalone
 * blocking command: PfmInput_Arm() only sets up a target period count
 * per channel; the actual hardware capture starts inside
 * PFM_Restart() (pfm.c) -- the same function that starts HRTIM output
 * -- via PfmInput_OnShotStart(), and stops (with whatever partial
 * count was reached) wherever pfm.c stops a shot, via
 * PfmInput_OnShotEnd(). A capture can therefore never run longer than
 * the shot that started it -- see AGENTS.md/docs/changelog.txt for the
 * full design writeup, including what got corrected from this
 * feature's first draft plan (originally a blocking PFMIN:CAPTURE with
 * no tie to FIRE at all).
 *
 * This module is self-contained and removable: PFM_INPUT_FEATURE_ENABLED
 * below is the single point of control, matching boot_jump.h/
 * qspi_test.h's established pattern. Disabled: every function below
 * becomes a no-op/returns-nothing (see pfm_input.c's #else branch);
 * pfm.c's calls into PfmInput_OnShotStart()/OnShotEnd() stay
 * unconditional either way (always resolve to something or nothing),
 * same as every other removable module's call sites in this project.
 */

#ifndef PFM_INPUT_FEATURE_ENABLED
#define PFM_INPUT_FEATURE_ENABLED (1)
#endif

/* One entry per PFM_Input_01..06 -- channel index 0..5 = PFM_Input_01..06. */
#define PFM_INPUT_NUM_CHANNELS  (6U)

/* --------------------------------------------------------------------------
 * Active-channel mask (compile-time, hard limit)
 *
 * Added 2026-09-09, alongside the frequency-ramp capture investigation
 * (see docs/changelog.txt): only PFM_Input_01 (bit 0, PA15/TIM2_CH1)
 * is physically wired to a real signal (phase U) on this bench right
 * now. The other 5 channels were being fully initialized and armed on
 * every run for no benefit -- 3 extra timer peripherals (TIM3/TIM4/
 * TIM5) clocked and interrupt-enabled, 2 extra channels configured on
 * TIM2, all doing real (if idle) ISR/NVIC work -- adding CPU/interrupt
 * load for zero signal, and load is exactly what's under suspicion in
 * the still-open frequency-transition investigation (see
 * ctrlr_config.h's PFM_MAX_CARRIER_FREQ_HZ comment). Disabled here to
 * both save that compute and shrink the variables in play while that
 * investigation continues.
 *
 * Bit i (0-5) = PFM_Input_0(i+1) active. A timer with NO active
 * channel is left completely untouched by PfmInput_Init() -- no
 * HAL_TIM_IC_Init(), no clock enable, no NVIC IRQ enable, no GPIO AF
 * config -- not just "armed with M=0"; see pfm_input.c. Re-enable a
 * channel by setting its bit here; nothing else needs to change.
 *
 * Set to 0x3F (all 6 channels), 2026-09-09, step 6 (final) of the
 * DMA-based capture mechanism's incremental re-test (see this file's
 * own top comment and pfm_input.c's capture-technique history for the
 * full story). Steps 1-5 all confirmed clean, PFM:DIAG? max gap
 * growing gently and roughly linearly with channel count (about 17,
 * 42, 57, 74, 91 us) -- nowhere near the old interrupt-driven design's
 * 734 us stall, which broke at just 2 channels. This is the same
 * all-6 configuration that caused a real board lockup under the old
 * design. */
#define PFM_INPUT_ACTIVE_CHANNEL_MASK  (0x3FU)  /* all 6 channels */

/* Compile-time cap on periods storable per channel per run, sizing the
 * RAM arrays (6 x this x 2 x 4 bytes) -- see pfm_input.c. 200 is 10x
 * the example M=20 in the original request; cheap against this MCU's
 * 128 KiB SRAM and this project's existing 40 KB g_pfmTable[]
 * (pfm.c). Raise if a real need for more periods per run shows up. */
#define PFM_INPUT_MAX_PERIODS   (200U)

/* Exposed so stm32g4xx_it.c's TIM2/TIM3/TIM4/TIM5_IRQHandler()s can
 * call HAL_TIM_IRQHandler() directly, matching hrtim.h's own `extern
 * HRTIM_HandleTypeDef hhrtim1` precedent for the same reason. Not
 * meant to be touched directly by anything else -- go through the
 * functions below. Guarded the same as pfm_input.c's own definitions
 * (absent, not just unused, when this feature is disabled) --
 * stm32g4xx_it.c's 4 IRQHandler()s are guarded identically, so these
 * either both exist together or neither does; the vector table falls
 * back to the startup file's harmless weak Default_Handler for these
 * 4 entries when disabled (never a problem in practice, since the
 * NVIC for them is never enabled either -- PfmInput_Init() is a no-op
 * when disabled). */
#if (PFM_INPUT_FEATURE_ENABLED != 0)
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;

/* Exposed for the same reason as htim2..htim5 above -- stm32g4xx_it.c's
 * 6 DMA1_ChannelN_IRQHandler()s (2026-09-09, the DMA-based capture
 * redesign -- see pfm_input.c's capture-technique history) need to
 * call HAL_DMA_IRQHandler() directly. Indexed the same way as
 * kDesc[]/g_state[] in pfm_input.c (0 = PFM_Input_01, ..., 5 =
 * PFM_Input_06), regardless of which channels PFM_INPUT_ACTIVE_
 * CHANNEL_MASK actually enables -- an inactive channel's entry here
 * is simply never initialized/linked, harmless to declare either way. */
extern DMA_HandleTypeDef pfmInputDma[PFM_INPUT_NUM_CHANNELS];
#endif

/* Called once at boot (main.c, alongside the other peripheral bring-up
 * calls) -- configures rising-edge input capture, GPIO AF pins, AND
 * (2026-09-09) the DMA channel that streams captured ticks into RAM
 * with no per-edge CPU involvement, for every channel enabled in
 * PFM_INPUT_ACTIVE_CHANNEL_MASK (above); a timer with no active
 * channel is left completely uninitialized (no clock, no NVIC, no
 * GPIO, no DMA) rather than merely unarmed. Does not arm or start any
 * capture. No-op when PFM_INPUT_FEATURE_ENABLED is 0. */
void PfmInput_Init(void);

/* Arms every ACTIVE channel (PFM_INPUT_ACTIVE_CHANNEL_MASK, above) to
 * capture up to `m` periods each on the NEXT shot (PFM_Restart(),
 * pfm.c) -- pure bookkeeping, touches no TIMx hardware, safe to call
 * whether or not a shot is currently running. An inactive channel is
 * left permanently at count 0, whatever `m` is passed. Single-shot:
 * consumed by the next PfmInput_OnShotStart() call, not sticky across
 * multiple shots -- arm again before every FIRE that should capture.
 * `m` is clamped to PFM_INPUT_MAX_PERIODS. No-op when disabled. */
void PfmInput_Arm(uint16_t m);

/* Called from PFM_Restart() (pfm.c), right alongside HRTIM1_PWM_Start()
 * -- starts HAL_TIM_IC_Start_DMA() for every channel with a nonzero
 * armed target (see PfmInput_Arm()), and clears that channel's arm
 * (single-shot). A channel never armed is left untouched. This is what
 * makes capture begin exactly when a PFM output shot begins. No-op
 * when disabled. */
void PfmInput_OnShotStart(void);

/* Called from every place pfm.c stops a shot (PFM_ForceStop() and the
 * normal end-of-table-exhaustion path in PFM_CycleBoundaryHandler()) --
 * stops (HAL_TIM_IC_Stop_DMA()) any channel still capturing, leaving
 * its partial count intact. Bounds every capture's runtime to the shot
 * that started it; a channel already finished on its own is a no-op
 * here. No-op when disabled. */
void PfmInput_OnShotEnd(void);

/* Current captured-period count for `channel` (0..5), valid whether or
 * not a capture is armed/running/finished. Returns 0 for an
 * out-of-range channel or when disabled. */
uint16_t PfmInput_GetCount(uint8_t channel);

/* Raw tick array for `channel` (0..5) -- period[i] (rising-to-rising,
 * in timer ticks) for i = 0..PfmInput_GetCount(channel)-1, in capture
 * order. Returns NULL for an out-of-range channel or when disabled; a
 * valid channel always returns a non-NULL pointer (into a fixed static
 * array), regardless of how many entries are actually valid --
 * callers must use PfmInput_GetCount() to know how many entries to
 * read. */
const uint32_t *PfmInput_GetPeriods(uint8_t channel);

/* Diagnostic: number of times a hardware overcapture (CCxOF) was
 * observed for `channel` since the last PfmInput_Arm() -- a nonzero
 * count means at least one edge was serviced too late (a later edge's
 * timestamp silently overwrote an earlier one in hardware before it
 * could be read), so some entries in this channel's period[] array may
 * not be trustworthy. Added 2026-09-08 while diagnosing real data
 * corruption on real hardware -- see docs/changelog.txt. As of the
 * 2026-09-09 DMA-based capture redesign (pfm_input.c), this is checked
 * once per DMA half-buffer batch rather than once per edge -- coarser
 * (can't say which edge in a batch), but still meaningful, and
 * expected to be even less likely to read nonzero than before (DMA
 * services CCRx far faster than the old software ISR did). Returns 0
 * for an out-of-range channel or when disabled. */
uint16_t PfmInput_GetOvercaptureCount(uint8_t channel);

/* TEMPORARY debug aid, 2026-09-09 (see pfm_input.c's own comment on
 * g_dmaStartStatus) -- HAL_StatusTypeDef from the last
 * HAL_TIM_IC_Start_DMA() call for `channel` (0=HAL_OK, 1=HAL_ERROR,
 * 2=HAL_BUSY, 3=HAL_TIMEOUT), or 0xFF if never armed/started or the
 * channel is out of range. Remove once DMA capture is confirmed
 * reliable. */
uint8_t PfmInput_GetDmaStartStatus(uint8_t channel);

/* --------------------------------------------------------------------------
 * Continuous ("free-running") capture -- added 2026-09-09 for pid.c's
 * closed-loop feedback read, a genuinely different consumer than the
 * bounded bench-diagnostic capture above (PfmInput_Arm()/OnShotStart()/
 * OnShotEnd(), the PFMIN:CAPTURE/STATus?/DATA? wire commands). That
 * path captures exactly M periods into an array, then stops -- exactly
 * right for "pull a bench snapshot and analyze it," wrong for a live
 * controller that just wants "whatever the most recently measured
 * period is," indefinitely, with no target count and no array to
 * overflow past PFM_INPUT_MAX_PERIODS. This is that second, simpler
 * mode: no accumulation, just one continuously-overwritten scalar per
 * channel.
 *
 * Independent of the shot-lifecycle machinery above -- does not touch
 * PfmInput_Arm()'s target-count state, is not started by
 * PFM_Restart()/PfmInput_OnShotStart(), and is not stopped by
 * PfmInput_OnShotEnd(). Runs until explicitly stopped. Mutually
 * exclusive with the bench-capture path PER CHANNEL (both ultimately
 * own the same DMA/timer resource) -- starting one while the other is
 * already running on the same channel fails cleanly (returns 0 /
 * no-op) rather than corrupting shared DMA state; a channel not in use
 * by the PID loop remains fully available for PFMIN:CAPTURE as before.
 * -------------------------------------------------------------------------- */

/* Starts free-running capture on `channel` (0..5). Returns 1 on
 * success, 0 if `channel` is out of range, inactive
 * (PFM_INPUT_ACTIVE_CHANNEL_MASK), already running (continuous OR an
 * armed bench capture), or the DMA start itself failed. No-op
 * (returns 0) when disabled. */
uint8_t PfmInput_StartContinuous(uint8_t channel);

/* Stops free-running capture started by PfmInput_StartContinuous().
 * Harmless if `channel` is out of range or wasn't running in
 * continuous mode (a bench capture running on this channel is left
 * alone -- this only ever stops what PfmInput_StartContinuous()
 * itself started). No-op when disabled. */
void PfmInput_StopContinuous(uint8_t channel);

/* Most recently measured period (raw timer ticks, rising-to-rising)
 * for `channel` -- 0 before at least 2 rising edges have been seen
 * since capture started (continuous or bench), or if `channel` is out
 * of range or disabled. Updated by EITHER capture mode (whichever is
 * currently running on this channel), so it stays meaningful
 * regardless of which one is active -- only continuous capture runs
 * indefinitely, though; a bench capture's last-measured value goes
 * stale once that capture finishes and stops. */
uint32_t PfmInput_GetLatestPeriod(uint8_t channel);

/* --------------------------------------------------------------------------
 * Windowed average -- added 2026-09-10, per direct project decision:
 * pid.c's closed-loop feedback should be the AVERAGE frequency over
 * everything PFM_Input measured during the just-completed HRTIM
 * Master period, not just the single most-recently-captured edge
 * (PfmInput_GetLatestPeriod(), above) -- a real Transrex output can
 * be tens of periods per 1 ms Master tick, and averaging over all of
 * them (standard reciprocal-frequency-counting practice: average the
 * raw PERIODS, then convert once to frequency, per direct
 * confirmation -- NOT averaging already-converted frequencies, a
 * different, not-chosen number) gives a real noise-rejecting
 * measurement instead of one arbitrary sample.
 *
 * Window boundary is intentionally NOT hardware-timestamped to the
 * exact 1 ms Master tick -- per direct confirmation, "doesn't need to
 * be exact." Implemented the simple way: accumulate every complete
 * period since the last PfmInput_ConsumeAveragePeriod() call
 * (continuous mode only -- see ProcessDmaChunk()'s own comment),
 * THEN RESET on read. Called once per PID_Update() tick, this
 * naturally yields "the average over very close to one Master
 * period" without needing a second, independently-timed window
 * mechanism -- any edge still in flight when the window closes just
 * carries into the NEXT window's count instead of being split, which
 * is the normal, correct way to handle a boundary that doesn't need
 * to be exact.
 * -------------------------------------------------------------------------- */

/* Consumes (reads AND resets) `channel`'s accumulated average since
 * the last call. Returns 1 and fills `*avgPeriodTicks`/`*sampleCount`
 * if at least one complete period was accumulated; returns 0 (leaves
 * both outputs untouched) if the accumulator was empty -- e.g. the
 * signal's own period exceeds one Master period (an genuinely
 * possible condition down near PFM_TURNON_FREQ_HZ, ctrlr_config.h,
 * where a period can approach 1 ms), or nothing is physically
 * connected. `*sampleCount` is worth keeping, not just the average
 * itself: measurement confidence scales with how many periods went
 * into it -- a handful of samples near the frequency floor is a much
 * noisier estimate than the tens of samples typical near the top of
 * the range, worth surfacing (e.g. in a log) rather than only ever
 * reporting the averaged number with no sense of how solid it is. */
uint8_t PfmInput_ConsumeAveragePeriod(uint8_t channel, uint32_t *avgPeriodTicks, uint16_t *sampleCount);

/* TEMPORARY debug aid, added 2026-09-15 -- diagnosing why measuredHz
 * (PID:STATus?, backed by PfmInput_ConsumeAveragePeriod() above) reads
 * persistently 0 for WHAM channels 2/3/4 while channel 1 works
 * correctly, confirmed via real hardware (each channel isolated alone,
 * driving confirmed-correct real HRTIM output, over a full shot).
 * Non-destructive read of continuous-mode raw internal state --
 * unlike PfmInput_ConsumeAveragePeriod(), does NOT reset avgSum/
 * avgCount, so repeated polling can watch it accumulate (or not) live
 * during a shot without disturbing PID_Update()'s own real consumption.
 * `channel` out of range returns 0 with all outputs left untouched.
 * Remove once the root cause is found and fixed. */
uint8_t PfmInput_GetDebugRaw(uint8_t channel, uint8_t *continuous, uint8_t *running,
                              uint8_t *haveFirstRise, uint16_t *avgCount,
                              uint32_t *lastPeriod, uint16_t *overcaptureCount);

/* TEMPORARY debug aid, added 2026-09-15 -- raw TIMx peripheral register
 * readback for `channel`'s underlying timer/tim-channel, bypassing the
 * DMA/ISR software layers entirely to check the hardware's own ground
 * truth directly: is the counter (CR1.CEN) running, is the capture
 * channel actually enabled at the hardware level (CCER's CCxE bit --
 * this is what HAL_TIM_IC_Start_DMA()'s TIM_CCxChannelCmd() call is
 * supposed to set), is the DMA request enabled (DIER's CCxDE bit), and
 * does CNT/CCRx move between two successive calls (direct proof a real
 * edge is landing in hardware, independent of whether software ever
 * sees it). Added while diagnosing why WHAM channels 2/3's measuredHz
 * reads persistently 0 despite confirmed-correct real HRTIM output AND
 * confirmed-correct physical bench wiring -- see PfmInput_GetDebugRaw()
 * above for the software-side half of this same investigation. Remove
 * once the root cause is found and fixed. */
uint8_t PfmInput_GetDebugRegs(uint8_t channel, uint32_t *cr1, uint32_t *ccer,
                               uint32_t *dier, uint32_t *sr, uint32_t *cnt,
                               uint32_t *ccrChannel);

/* TEMPORARY debug aid, added 2026-09-15, same investigation as
 * PfmInput_GetDebugRegs() above -- CCMR1 holds the IC1S/IC2S input-
 * selection bits (is this channel's capture unit actually routed to
 * its OWN TI input, or accidentally to the wrong one/disconnected) as
 * well as the input filter/prescaler bits, none of which
 * PfmInput_GetDebugRegs() exposes. Remove once the root cause is
 * found and fixed. */
uint32_t PfmInput_GetDebugCcmr1(uint8_t channel);

/* TEMPORARY debug aid, added 2026-09-15, same investigation: raw GPIO
 * config for `channel`'s own pin -- MODER (is it actually in Alternate
 * Function mode, 0b10, not left in some other mode), AFR (which AF
 * number is actually selected, cross-checked against kDesc[]'s
 * intended value), and IDR (the pin's live logic level right now, a
 * single instantaneous sample -- polled repeatedly during a shot, a
 * genuinely toggling signal should show both 0 and 1 across samples;
 * always-0 or always-1 is a real clue). Remove once the root cause is
 * found and fixed. */
uint8_t PfmInput_GetDebugGpio(uint8_t channel, uint32_t *moder, uint32_t *afr,
                               uint32_t *idr);

#ifdef __cplusplus
}
#endif

#endif /* INC_PFM_INPUT_H_ */
