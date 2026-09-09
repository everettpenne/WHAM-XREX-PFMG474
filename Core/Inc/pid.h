#ifndef INC_PID_H_
#define INC_PID_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "ctrlr_config.h"

/*
 * pid.h / pid.c
 *
 * Closed-loop PID control for up to HRTIM_NUM_CHANNELS (ctrlr_config.h,
 * currently 4) independent Transrex channels -- this project's actual
 * reason for existing. See docs/changelog.txt's 2026-09-09
 * design-decision entry for the full architecture writeup this
 * implements; the short version:
 *
 *   - HRTIM side: Possibility 3
 *     (docs/Transrex/hrtim_divergent_period_timing.pdf) -- each active
 *     HRTIM channel is configured (hrtim.c's HRTIM1_FullInit())
 *     ResetTrigger=NONE/UpdateTrigger=NONE/ResetUpdate=ENABLED, so a
 *     PER/CMP1 write via HRTIM1_SetChannelPeriod() is always safe, at
 *     any time, and the hardware promotes it to active at THAT
 *     CHANNEL'S OWN next roll-over -- never mid-cycle, no relationship
 *     required between channels' frequencies.
 *
 *   - Timing side: the HRTIM Master repetition interrupt
 *     (HRTIM1_Master_IRQn, stm32g4xx_it.c) is kept as a FIXED-RATE
 *     heartbeat (PID_LOOP_RATE_HZ, ctrlr_config.h -- reprogrammed onto
 *     Master's own timebase by hrtim.c's HRTIM1_FullInit()),
 *     deliberately decoupled from any channel's own carrier frequency.
 *     This was a real, reconsidered design decision, not the obvious
 *     first idea -- a self-clocked alternative (each channel's own
 *     roll-over triggers its own PID recompute) was considered and
 *     rejected: it ties that channel's control-loop sample interval to
 *     the very frequency the PID is adjusting, which is both bad
 *     control practice (discrete PID assumes a fixed Delta-t; a
 *     drifting one shifts the effective I/D gains as the loop
 *     converges) and a category error (the V-to-F carrier's own
 *     frequency has no inherent relationship to the control bandwidth
 *     a magnet supply's real electrical time constant calls for). Do
 *     not "simplify" this back to self-clocked without re-reading that
 *     changelog entry first.
 *
 *   - PID_Update() (this module) runs once per Master heartbeat,
 *     looping over all HRTIM_NUM_CHANNELS channels: read channel i's
 *     latest measured period (pfm_input.c's PfmInput_GetLatestPeriod(),
 *     continuous/free-running mode -- PfmInput_StartContinuous()),
 *     convert to Hz, run that channel's own PID against its own
 *     setpoint with a FIXED Delta-t (= 1/PID_LOOP_RATE_HZ), clamp, write
 *     the new output frequency (converted back to a HRTIM `per`) via
 *     HRTIM1_SetChannelPeriod(). Each channel: fully independent state
 *     (setpoint, Kp/Ki/Kd, integrator) -- no shared table, no
 *     relationship to any other channel's own values.
 *
 * Units: setpoint, gains, and every reported value here are in Hz, not
 * raw timer ticks -- deliberately, per docs/changelog.txt's own note:
 * the V-to-F encoding this whole project serves is linear in
 * FREQUENCY, not period, so PID math needs to run in frequency space
 * for the loop to behave linearly/predictably across HRTIM's wide
 * dynamic range. The tick<->Hz conversion happens once per channel per
 * update, inside PID_Update() and HRTIM1_SetChannelPeriod().
 *
 * PER/CMP coherency (per the Possibility 3 reference document's own
 * caveat): HRTIM1_SetChannelPeriod() writes PER then CMP1 as two
 * separate register writes; a roll-over landing between them would
 * apply new-PER with old-CMP for one cycle. Not specifically guarded
 * against here -- CMP1 is always recomputed as per/2 (a clean 50%
 * duty, see that function's own comment), so the worst case is one
 * single cycle at a slightly-off duty, immaterial to a V-to-F receiver
 * that only cares about frequency. Revisit (HRTIM's UPDGAT, or a
 * DMA-burst transfer to make the pair atomic) only if real bench
 * testing ever shows this matters.
 */

/* Called once at boot (main.c) -- zeroes every channel's PID state and
 * sets a safe, inert default (all gains 0, so PID_Update() commands
 * PID_OUTPUT_MIN_HZ regardless of feedback, per project convention:
 * explicit, no implicit magic -- an operator must send real PID:GAINS
 * before this loop does anything meaningful). Does not touch HRTIM or
 * PFM_Input hardware -- see PID_Start() for that. Safe to call more
 * than once; each call re-zeroes everything (unlike pfm.c's
 * PFM_Init(), there is no durable per-channel state here worth
 * preserving across a re-init). */
void PID_Init(void);

/* Begins closed-loop operation: starts free-running PFM_Input capture
 * (PfmInput_StartContinuous()) on channels 0..HRTIM_NUM_CHANNELS-1 for
 * feedback, resets every channel's integrator/derivative history, and
 * starts HRTIM output (HRTIM1_PWM_Start()) with every active channel
 * enabled. A channel whose feedback signal isn't physically connected
 * (PfmInput_StartContinuous() still "succeeds" in the sense that DMA
 * capture starts -- it just never sees an edge) is not treated as an
 * error here: PID_Update() safely holds that channel's last output
 * rather than dividing by a period of 0, so a single unconnected
 * channel doesn't block the other 3. Returns 1 (always succeeds, or is
 * already running) -- kept non-void for symmetry with the rest of this
 * project's start/stop pairs and in case a real failure path is added
 * later. Idempotent -- calling while already running is a harmless
 * no-op. */
uint8_t PID_Start(void);

/* Stops closed-loop operation: stops HRTIM output (HRTIM1_PWM_Stop())
 * and every channel's continuous PFM_Input capture
 * (PfmInput_StopContinuous()). Idempotent -- harmless if not running.
 * Does NOT reset setpoints/gains (those are durable operator state,
 * same convention as pfm.c's g_channelEnabled) -- only PID_Start()
 * resets the transient integrator/derivative-history state. */
void PID_Stop(void);

/* The control loop itself -- called from HRTIM1_Master_IRQHandler()
 * (stm32g4xx_it.c) once per PID_LOOP_RATE_HZ heartbeat. A cheap no-op
 * when not running (PID_Start() never called, or PID_Stop() since).
 * See this file's own header comment for the full per-channel
 * algorithm. Not meant to be called from anywhere else. */
void PID_Update(void);

uint8_t PID_IsRunning(void);

/* Sets channel `channel`'s (0..HRTIM_NUM_CHANNELS-1) target output
 * frequency, clamped to [PID_OUTPUT_MIN_HZ, PID_OUTPUT_MAX_HZ]
 * (ctrlr_config.h). Returns 1 on success, 0 if `channel` is out of
 * range. Durable -- survives PID_Start()/PID_Stop(), same as pfm.c's
 * g_channelEnabled. Takes effect on this channel's very next
 * PID_Update() tick, whether or not the loop is currently running (a
 * setpoint set while stopped is simply what the loop will chase once
 * started). */
uint8_t PID_SetSetpoint(uint8_t channel, uint32_t setpointHz);

/* Sets channel `channel`'s PID gains and resets its integrator to 0 --
 * deliberately, to avoid a discontinuous output jump from an
 * integrator value accumulated under the OLD gains suddenly being
 * multiplied by new ones. Returns 1 on success, 0 if `channel` is out
 * of range. No sign/range validation on kp/ki/kd -- a badly-chosen
 * gain is an operator/tuning error to observe via PID:STATus?, not
 * something this function tries to guess is wrong. */
uint8_t PID_SetGains(uint8_t channel, float kp, float ki, float kd);

/* Current setpoint/measured/output (Hz) for `channel`, via `*setpointHz`/
 * `*measuredHz`/`*outputHz` (any may be NULL to skip that one) --
 * valid whether or not the loop is running (measured/output simply
 * hold whatever they last were). Returns 1 on success, 0 if `channel`
 * is out of range (outputs left unwritten in that case). */
uint8_t PID_GetStatus(uint8_t channel, uint32_t *setpointHz,
                      uint32_t *measuredHz, uint32_t *outputHz);

#ifdef __cplusplus
}
#endif

#endif /* INC_PID_H_ */
