#ifndef INC_SIM_TRANSREX_H_
#define INC_SIM_TRANSREX_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*
 * sim_transrex.h
 *
 * SIMULATOR-ONLY module (BUILD_TARGET_SIMULATOR, ctrlr_config.h/
 * build_target.h) -- the actual "act like a Transrex" logic for the
 * second-board bench simulator (docs/pin_mapping_reference.tex Section
 * 7, /Users/everettpenne/.claude/plans/cuddly-nibbling-octopus.md).
 * Everything up to this point (DIAGnostic:GPOut09-12, XREX:CHANnel:
 * ENAOut/CONTactOut, the raw fiber wiring itself) is just physical
 * connectivity -- this module is what turns that wiring into a
 * plausible closed loop: capture the controller's real commanded DRIVE
 * frequency, low-pass filter it, and drive a FEEDBACK signal back out,
 * gated on the controller's own ENA_OUT/CONTACT_OUT signals actually
 * being asserted (mimicking a real Transrex only responding once
 * genuinely connected/enabled) -- plus per-channel/per-category fault
 * injection over the same fiber transmitters used for connectivity
 * testing.
 *
 * Owns 4 things, one per Transrex channel (0..HRTIM_NUM_CHANNELS-1,
 * 0-based internal convention, matching every other per-channel module
 * in this project):
 *
 *   1. DRIVE capture -- reuses pfm_input.c's existing continuous-mode
 *      capture (PfmInput_StartContinuous()/ConsumeAveragePeriod()),
 *      unchanged, on PFM_Input_01..04 (PA15/PD4/PB2/PC12 -- this
 *      board's XR1..4_FEEDBACK pins, which receive the CONTROLLER's
 *      own XRn_DRIVE output).
 *   2. A single-pole low-pass filter on that measured frequency
 *      (SimTransrex_SetTauMs()/GetTauMs(), default
 *      SIM_TRANSREX_DEFAULT_TAU_MS) -- filtering in Hz directly, not
 *      Amps-then-reconverting, since AmpsToHz() (pid.c) is linear; see
 *      the plan document's own note on this assumption.
 *   3. FEEDBACK generation -- writes the filtered frequency out via
 *      this board's own XRn_DRIVE HRTIM channels (the same physical
 *      HRTIM1_SetChannelPeriod() primitive pid.c uses on the
 *      controller), which the CONTROLLER receives as ITS
 *      XRn_FEEDBACK. Gated per channel on ENA_OUT+CONTACT_OUT both
 *      being asserted (read from the controller, via GateDriver_Read()'s
 *      PE0..PE7 -- NOT XrexIo_GetEnableOutput()/GetContactorOutput(),
 *      which read THIS board's own PG0..PG7 transmit pins, a different,
 *      unrelated pair of signals on the simulator's own role -- see
 *      this file's own .c comment for the full pin-role table).
 *   4. Per-channel/per-category fault injection (Water+Temp combined,
 *      Enerpro, OCP -- SimTransrex_SetFault*()) over the same 12
 *      GPOut_01..12 fiber transmitters DIAGnostic:GPOut09-12 and
 *      XREX:CHANnel:ENAOut/CONTactOut already drive at the raw-pin
 *      level -- this module's setters are the SIM:FAULT: namespace's
 *      backing implementation, and own the actual polarity translation
 *      (FAULT_POLARITY_NORMALLY_HIGH, ctrlr_config.h -- "faulted" means
 *      drive the pin LOW, "healthy" means HIGH) so the SIM: command
 *      layer and DIAGnostic:GPOut09-12/XREX:CHANnel:ENAOut/CONTactOut
 *      can coexist without fighting -- see this file's own .c comment.
 *
 * *** KNOWN LIMITATION, flagged directly rather than silently worked
 * around: this simulator build still runs the FULL controller firmware
 * unconditionally -- its own state_machine.c/pid.c/gate_driver.c are
 * all still live and watching the exact same physical pins this module
 * uses for a completely different purpose (PE0..PE7, repurposed here
 * as ENA_OUT/CONTACT_OUT gating input, are ALSO the simulator's own
 * XR1..4_WATER_FLT/_TMP_FLT fault-detection inputs under its own,
 * unmodified gate_driver.c). This is harmless AS LONG AS the
 * simulator's own PID channels are never PID:CHANnel:ENAble'd on --
 * gate_driver.c's fault evaluation is gated per-channel-enabled
 * (xrex_io.h), so a disabled channel's pins are never evaluated as a
 * real fault regardless of what's actually being received on them.
 * Do not enable the simulator's own PID channels while using it as a
 * fault-injection/feedback device. Differentiating main.c's own
 * state-machine/PID polling to be controller-only (so this caveat goes
 * away entirely) is flagged as a follow-up, not yet done -- see
 * AGENTS.md/docs/changelog.txt. */

/* Default filter time constant, milliseconds -- see this file's own
 * top comment, point 2. A first, reasonable-sounding guess (real
 * magnet/supply time constants are unknown from here); revisit once
 * real bench step-response data exists, same "PLACEHOLDER, revisit"
 * status as ctrlr_config.h's own Amps<->Hz calibration constants. */
#define SIM_TRANSREX_DEFAULT_TAU_MS   (100U)

/* Called once at boot (main.c, BUILD_TARGET_SIMULATOR only) after
 * HRTIM1_FullInit()/HRTIM1_EnableMasterInterrupt() and PfmInput_Init()
 * have already run (both are unconditional, shared boot steps -- see
 * main.c). Starts continuous PFM_Input capture on channels 0..3 and
 * starts this board's own HRTIM Master+slave counters running
 * UNCONDITIONALLY -- deliberately NOT tied to this board's own
 * ARM/FIRE state (see this file's own top comment: a real Transrex's
 * response is gated by ENA_OUT/CONTACT_OUT, not by its own internal
 * "shot" concept, which doesn't apply to a simulator playing that
 * role) -- but every channel's actual OUTPUT starts DISCONNECTED
 * (stays LOW), per direct instruction: a channel with no ENA_OUT
 * asserted, or no CONTACT_OUT asserted (independently, either one
 * alone), must produce no PFM output at all, not a floored idle
 * frequency -- see SimTransrex_Update()'s own comment for where the
 * live per-channel connect/disconnect actually happens. Also drives
 * every fault-injection transmitter pin to its HEALTHY level (HIGH,
 * FAULT_POLARITY_NORMALLY_HIGH).
 *
 * *** DELIBERATE EXCEPTION to this project's usual "every new output
 * defaults LOW at boot, never glitch HIGH" convention (DIAGnostic:
 * GPOut09-12 etc.) *** -- per the confirmed scope decision ("Default
 * healthy, inject all 4 categories"), the whole point of this
 * simulator is to NOT present the floating/LOW-reads-as-fault state a
 * disconnected real Transrex's fault pins do on the bench today. The
 * pins are briefly LOW (their own GPIO block's own boot default, set
 * earlier in MX_GPIO_Init()) before this function then drives them
 * HIGH -- a one-time post-boot correction to this module's own real
 * steady-state default, not a glitch in the pins' own output. */
void SimTransrex_Init(void);

/* Called every main-loop iteration (main.c's `while(1)`, alongside
 * SM_PollFaults()/XrexIo_PollOcpFaults(), BUILD_TARGET_SIMULATOR only)
 * -- NOT from PID_Update() (which only runs while THIS board's own
 * HRTIM Master counter is active, i.e. only during a FIRE on this
 * board -- irrelevant here, see this file's top comment). Computes its
 * own dt from HAL_GetTick() (millisecond resolution; main-loop cadence
 * is not exactly regular, same "doesn't need to be exact" precedent as
 * pfm_input.h's own windowed-average design). For each channel,
 * independently (4 separate channels, not one shared gate):
 *   - reads ENA_OUT+CONTACT_OUT gating (GateDriver_Read(), PE0..PE7);
 *   - on a gating EDGE, connects/disconnects that channel's actual
 *     HRTIM output pins (HRTIM1_SetChannelOutputEnable()) to match --
 *     gated off means the pin physically stays LOW (this channel's
 *     configured HRTIM idle level), not a floored idle frequency, per
 *     direct instruction; gated on reconnects it;
 *   - if gated on: consumes the latest averaged DRIVE measurement
 *     (PfmInput_ConsumeAveragePeriod()), converts to Hz
 *     (HRTIM_TIMER_CLK_HZ / avgPeriodTicks, same formula pid.c uses),
 *     low-pass filters it, and writes the result out via
 *     HRTIM1_SetChannelPeriod();
 *   - if gated off: the filter's own internal target still floors
 *     toward PFM_TURNON_FREQ_HZ (0 A) even though the output is
 *     physically disconnected -- purely so a later re-gate resumes
 *     from near the floor instead of some stale mid-shot value, not
 *     because anything reaches the pin while disconnected. */
void SimTransrex_Update(void);

/* Water+Temp fault injection for `channel` (0..HRTIM_NUM_CHANNELS-1),
 * combined into one call/one physical transmitter (GPOut_01..04 --
 * these two categories share a single fiber+splitter per channel on
 * the real wiring, docs/pin_mapping_reference.tex Section 7, so they
 * cannot be independently faulted -- an honest single command instead
 * of two same-pin-aliased ones). `faulted != 0` drives the pin to its
 * FAULT level (LOW, FAULT_POLARITY_NORMALLY_HIGH); 0 drives HEALTHY
 * (HIGH). Out-of-range `channel` is a no-op. */
void SimTransrex_SetFaultWaterTemp(uint8_t channel, uint8_t faulted);
uint8_t SimTransrex_GetFaultWaterTemp(uint8_t channel);

/* Enerpro fault injection for `channel`, independent per channel
 * (GPOut_05..08, one dedicated transmitter each). Same polarity/
 * out-of-range convention as SimTransrex_SetFaultWaterTemp() above. */
void SimTransrex_SetFaultEnerpro(uint8_t channel, uint8_t faulted);
uint8_t SimTransrex_GetFaultEnerpro(uint8_t channel);

/* OCP fault injection for `channel`, independent per channel
 * (GPOut_09..12, one dedicated transmitter each -- XR1..4 in that
 * order, matching the controller's own irregular kOcpPin[] order, see
 * docs/pin_mapping_reference.tex Section 7). Same polarity/out-of-
 * range convention as SimTransrex_SetFaultWaterTemp() above. */
void SimTransrex_SetFaultOcp(uint8_t channel, uint8_t faulted);
uint8_t SimTransrex_GetFaultOcp(uint8_t channel);

/* Low-pass filter time constant, milliseconds -- SIM:MODEL:TAU
 * (commands.c). Applies to every channel identically (one shared
 * filter constant, not per-channel -- no evidence yet that it should
 * differ per channel). `ms` of 0 is rejected (returns 0, config left
 * unchanged) -- a zero time constant is a divide-by-zero in the filter
 * math (SimTransrex_Update()'s own alpha = dt/(tau+dt)), not a
 * meaningful "instant response" request; use a very small nonzero
 * value instead if that's genuinely wanted. */
uint8_t SimTransrex_SetTauMs(uint32_t ms);
uint32_t SimTransrex_GetTauMs(void);

/* Diagnostic snapshot for `channel` (0..HRTIM_NUM_CHANNELS-1) -- backs
 * SIM:CHANnel:STATus? (commands.c). Returns 1 and fills every output on
 * success; returns 0 (outputs left untouched) for an out-of-range
 * channel. `measuredDriveHz` is the last value SimTransrex_Update()
 * actually consumed from PFM_Input (0 if never yet measured or the
 * channel is gated off); `filteredFeedbackHz` is this module's own
 * current filter state (what it's actually driving out via HRTIM,
 * PFM_TURNON_FREQ_HZ while gated off); `enaOutGated`/`contactOutGated`
 * are the live, raw ENA_OUT/CONTACT_OUT gating bits (not latched -- a
 * fresh GateDriver_Read() each call); the three fault outputs are this
 * module's own INJECTED state (SimTransrex_SetFault*()'s last-set
 * value), not a raw pin read. */
uint8_t SimTransrex_GetChannelStatus(uint8_t channel,
                                      uint32_t *measuredDriveHz,
                                      uint32_t *filteredFeedbackHz,
                                      uint8_t *enaOutGated,
                                      uint8_t *contactOutGated,
                                      uint8_t *faultWaterTemp,
                                      uint8_t *faultEnerpro,
                                      uint8_t *faultOcp);

/* --------------------------------------------------------------------------
 * Waveform log -- added 2026-09-18, per direct correction: polling
 * SIM:CHANnel:STATus? at a fixed host-side interval during a shot (the
 * original run_simulator_validation.py approach) is coarse (~10
 * samples/sec, serial round-trip jitter) and doesn't match this
 * project's own established "arm a log, run the shot, retrieve and
 * plot afterward" pattern already used for the CONTROLLER side
 * (pid.h's PID_ArmLog()/PID:LOGDATA?). This is that same pattern for
 * the simulator: SimTransrex_Update() itself appends a sample every
 * time it actually runs (main-loop cadence, no polling from the host
 * at all), so resolution is limited only by how fast the main loop
 * itself runs, not by a host-side query round-trip.
 *
 * UNLIKE PID_ArmLog() (pid.h), which decimates against a REAL fixed
 * hardware tick (PID_Update() runs on the HRTIM Master's 1kHz
 * interrupt, so "every Nth tick" is a precise, regular rate) --
 * SimTransrex_Update() runs off the main loop, which has NO fixed
 * rate at all. So instead of a tick-count decimation, each armed
 * channel gets its own MINIMUM time-between-samples throttle
 * (`minIntervalMs`), and each logged sample carries its OWN elapsed-
 * time-since-arm timestamp explicitly (SIM_LOGDATA_TIMESTAMPS below)
 * rather than assuming a shared rate a host-side plotter could
 * reconstruct from a sample index alone. Single-channel-at-a-time,
 * like PID_ArmLog() (not PID_ArmLogAll()'s every-channel variant) --
 * this project's simulator scenarios only ever care about one gated
 * channel per test. */

/* 1000, not 2000 -- an initial 2000 (24000 bytes: 1000*3*4) overflowed
 * this MCU's RAM by 7200 bytes on a simulator build (this module's log
 * arrays are on top of pid.c's own 48000-byte g_logSetpoint/Measured/
 * Output, pfm.c's 40000-byte g_pfmTable[], and everything else already
 * resident) -- reduced to 1000, matching PID_LOG_MAX_SAMPLES's own
 * value, confirmed to link clean. */
#define SIM_LOG_MAX_SAMPLES  (1000U)

/* Arms logging for `channel` (0..HRTIM_NUM_CHANNELS-1): resets any
 * previous log for this or any other channel (single-channel-at-a-
 * time, arming a new one discards the old), clamps `maxSamples` to
 * SIM_LOG_MAX_SAMPLES, and records `minIntervalMs` as the minimum
 * real time between two logged samples (0 = log every single
 * SimTransrex_Update() call, unthrottled -- true native main-loop
 * resolution). Returns 1 on success, 0 for an out-of-range channel. */
uint8_t SimTransrex_ArmLog(uint8_t channel, uint16_t maxSamples, uint16_t minIntervalMs);

/* How many samples have been logged so far for the currently-armed
 * channel (0 if nothing is armed, or the armed channel doesn't match
 * `channel`). Keeps growing (up to its own armed maxSamples) as
 * SimTransrex_Update() runs; does not itself stop anything once full
 * -- matching PID_GetLogSampleCount()'s own "keeps reporting the
 * final count once full" behavior. */
uint16_t SimTransrex_GetLogCount(uint8_t channel);

/* One logged sample (0-based `index` < SimTransrex_GetLogCount(channel)):
 * `timeMs` is elapsed milliseconds since SimTransrex_ArmLog() was
 * called (NOT since the channel was gated on, or since sample 0 --
 * always since the arm call itself, so a host-side plotter's time
 * axis starts at 0 the instant logging was armed, matching PID:LOG's
 * own "time axis starts when armed" convention); `driveHz`/`feedbackHz`
 * are that sample's DRIVE_HZ/FEEDBACK_HZ, same values
 * SimTransrex_GetChannelStatus() would have reported at that instant.
 * Returns 1 on success, 0 for an out-of-range channel/index or a
 * channel that isn't the currently-armed one. */
uint8_t SimTransrex_GetLogSample(uint8_t channel, uint16_t index,
                                  uint32_t *timeMs, uint32_t *driveHz, uint32_t *feedbackHz);

#ifdef __cplusplus
}
#endif

#endif /* INC_SIM_TRANSREX_H_ */
