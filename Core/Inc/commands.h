#ifndef __COMMANDS_H__
#define __COMMANDS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "uart.h"
#include "boot_jump.h"
#include "ctrlr_config.h"
#include "qspi_test.h"
#include "pfm_input.h"

/* --------------------------------------------------------------------------
 * Command handler implementations for WHAM-XREX-PFMG474.
 *
 * Response conventions (ported from the sibling PFM-STM32G474 project,
 * per project decision):
 *   OK\r\n              - accepted, no data
 *   OK <value>\r\n      - accepted, with return value
 *   ERR <n> <msg>\r\n   - rejected; error codes are stable across versions
 *
 * Error codes
 * ------------
 *   1   Unknown command
 *   2   Not currently uploading a table -- send TABle:BEGin first
 *   3   Table full (PFM_TABLE_SIZE entries already appended)
 *   4   Invalid TABle:STEP arguments (wrong count, or a value outside
 *       uint16 range 0-65535) -- count is 1 + HRTIM_NUM_CHANNELS
 *       (per + one compare value per channel), not a fixed 4; see
 *       CONFig:CHANnels? to discover HRTIM_NUM_CHANNELS for a given
 *       board without assuming it.
 *   5   Table is empty -- FIRE has nothing to play back
 *   6   Fault latched -- either PC10/HRTIM1_FLT6 (native hardware) or
 *       the GateDriverStatus_01..12 EXTI interrupt (PE0..PE11,
 *       gate_driver.h) -- FAULT:CLEAR required before FIRE will work
 *       again
 *   7   QUADSPI command failed or timed out (see QSPI:ID?, qspi_test.h)
 *   8   Invalid PFM_Input channel -- must be 1-6 (see PFMIN:DATA?)
 *   9   PFMIN:CAPTURE's M out of range -- must be 1-PFM_INPUT_MAX_PERIODS
 *       (pfm_input.h)
 *   10  TABLE:STEP's per implies a carrier frequency above
 *       PFM_MAX_CARRIER_FREQ_HZ (ctrlr_config.h) -- see that file's own
 *       comment for why this hard limit exists
 *   11  Invalid PID channel -- must be 1-HRTIM_NUM_CHANNELS (see
 *       CONFig:CHANnels?, pid.h)
 *   12  Invalid PID:* command arguments (wrong count, or a
 *       non-numeric/out-of-range value)
 *   13  Invalid state-machine transition for the current state (ARM/
 *       DISARM/PID:PROFile:STARt) -- see state_machine.h
 *   14  Invalid PID:CHANnel:NICKname -- 1-PID_CHANNEL_NICKNAME_MAX_LEN
 *       chars, no whitespace, and not the reserved value "-" (pid.h)
 *   15  PID:PROFile:STARt refused -- the external-enable interlock
 *       (EXTernal:ENAble) is on and PF13 currently reads LOW
 *       (state_machine.h)
 *   16  RETIRED 2026-09-17 -- previously "EXTernal:TRIGger refused --
 *       EXTernal:ENAble must be turned on first," back when enable and
 *       trigger shared one pin (PF15). No longer generated (enable
 *       moved to PF13, trigger stayed on PF15, the two features are no
 *       longer coupled) -- kept here, not reassigned, per this
 *       project's "never renumber/reuse" error-code convention
 *
 * Mnemonics are SCPI-style hierarchical patterns matched by
 * cmd_parser.c's scpi_match() -- see that file's header for the
 * short/long-form and multi-layer (':') matching rules.
 *
 * To add a command: declare its handler here, implement it in
 * commands.c, and add a row to cmd_parser.c's command_table[].
 * -------------------------------------------------------------------------- */

/* Identification / system */
void cmd_idn(uart_instance_t *inst, char *args); /* *IDN? -- board + firmware
                                                      identification, see
                                                      ctrlr_config.h */

/* Serial-bootloader entry. Gated on BOOT_JUMP_FEATURE_ENABLED
 * (boot_jump.h) -- entirely absent, including cmd_parser.c's "BOOT"
 * table row, when that module is disabled. */
#if (BOOT_JUMP_FEATURE_ENABLED != 0)
void cmd_boot(uart_instance_t *inst, char *args); /* BOOT -- reset into the
                                                       ROM serial bootloader,
                                                       see boot_jump.c */
#endif

/* PFM table upload -- a complete shot profile built entirely
 * off-controller (see python/pfm_table_upload.py) and streamed in one
 * step at a time. See pfm.h's "ADDED" header note and
 * docs/command_reference.md for the full protocol and why
 * construction deliberately lives on the host, not here. */
void cmd_table_begin(uart_instance_t *inst, char *args); /* TABle:BEGin -- clears
                                                              the table, opens an
                                                              upload session */
void cmd_table_step(uart_instance_t *inst, char *args);  /* TABle:STEP <per> <cmp0>
                                                              ... <cmp(N-1)> -- appends
                                                              one entry, N = HRTIM_NUM_CHANNELS */
void cmd_table_end(uart_instance_t *inst, char *args);   /* TABle:END -- closes the
                                                              upload session, reports
                                                              the final entry count */
void cmd_table_query(uart_instance_t *inst, char *args); /* TABle? -- reports the
                                                              current entry count */

/* Begins PWM output: (re)starts playback of the currently-uploaded PFM
 * table from step 0 (PFM_Restart(), which also enables the HRTIM
 * channels for output -- see hrtim.c's HRTIM1_PWM_Start()). No ARM/
 * state-machine interlock exists in this minimal firmware -- FIRE
 * always takes effect immediately, whether idle or already mid-shot.
 * Rejects with ERR 5 if the table is empty, ERR 6 if either fault
 * source is latched (see commands.c's AnyFaultLatched()). */
void cmd_fire(uart_instance_t *inst, char *args); /* FIRE -- start PWM output */

/* Diagnostic (added 2026-09-09) -- real inter-call timing for the PFM
 * table-advance ISR during the current/most recent shot. See
 * commands.c's own header comment on cmd_pfm_diag() for the full
 * writeup and pfm.h's PFM_GetDiagCounters(). */
void cmd_pfm_diag(uart_instance_t *inst, char *args); /* PFM:DIAG? -- OK
                                                           <callCount> <maxGapCycles>
                                                           <maxGapUs> */

/* TEMPORARY debug command (2026-09-09, second round) -- dumps the raw
 * per-call inter-call gap log behind PFM_GetDiagGapLog() (pfm.h), for
 * the RAMP/HOLD/RAMP real-hardware lag investigation (see
 * docs/changelog.txt). Remove once that investigation is resolved. */
void cmd_pfm_gaplog(uart_instance_t *inst, char *args); /* PFM:GAPLOG? -- OK
                                                             <count> <gap1> <gap2> ... */

/* Status/clear for BOTH of this project's fault sources, combined --
 * PC10/HRTIM1_FLT6 native hardware (hrtim.h, autonomous in silicon)
 * and the GateDriverStatus_01..12 EXTI interrupt (gate_driver.h,
 * PE0..PE11, software/interrupt-driven). See commands.c's
 * AnyFaultLatched() and its own header comment. Latched: FAULT:CLEAR
 * must be sent explicitly before FIRE works again, even after the
 * physical fault condition itself has gone away -- a fault silently
 * clearing on its own, with output resuming unnoticed, is the hazard
 * this avoids. */
void cmd_fault_query(uart_instance_t *inst, char *args); /* FAULT? -- OK 0|1 */
void cmd_fault_clear(uart_instance_t *inst, char *args);  /* FAULT:CLEar -- OK */

/* Top-level operating-state machine (state_machine.h), added
 * 2026-09-13 -- IDLE/ARMED/FIRING/FAULT. Bare, top-level commands
 * (system-wide state, not one subsystem), matching this project's
 * existing bare FIRE. ERR 13 (new): invalid state-machine transition
 * for the current state. */
void cmd_arm(uart_instance_t *inst, char *args);          /* ARM -- OK, IDLE -> ARMED */
void cmd_disarm(uart_instance_t *inst, char *args);       /* DISARM -- OK, ARMED -> IDLE,
                                                                no-op if not ARMED */
void cmd_state_query(uart_instance_t *inst, char *args);  /* STATE? -- OK <IDLE|ARMED|
                                                                FIRING|FAULT>, or
                                                                OK FAULT GENERAL, or
                                                                OK FAULT OVERCURRENT <ch>
                                                                (1-based), or
                                                                OK FAULT EXTERNAL_ENABLE, or
                                                                OK FAULT EMERGENCY_STOP, or
                                                                OK FAULT ENABLE_OUTPUT <ch>
                                                                (1-based) */

/* External-enable interlock (PF13), added 2026-09-16, MOVED here
 * 2026-09-17 from PF15 -- see state_machine.h's own external-enable
 * section for the full design. Bare top-level namespace, same
 * reasoning as ARM/DISARM above. ERR 15 (new): PID:PROFile:STARt
 * refused because the interlock is on and PF13 currently reads LOW. */
void cmd_ext_enable(uart_instance_t *inst, char *args);             /* EXTernal:ENAble <0|1> --
                                                                         OK */
void cmd_ext_enable_query(uart_instance_t *inst, char *args);       /* EXTernal:ENAble? -- OK
                                                                         <0|1> */
void cmd_ext_enable_input_query(uart_instance_t *inst, char *args); /* EXTernal:INPut? --
                                                                         OK <0|1>, raw PF13
                                                                         level, independent
                                                                         of whether the
                                                                         interlock is on */

/* External trigger (rising edge on PF15 fires a shot while ARMED, and
 * ONLY from ARMED), added 2026-09-16 -- see state_machine.h's own
 * design comment (SM_SetExternalTriggerRequired() and friends).
 * RESTRUCTURED 2026-09-17: PF15 now backs trigger only (external-enable
 * moved to PF13, above) -- no longer requires EXTernal:ENAble on first,
 * that coupling was removed the same day the pins split. ERR 16 is
 * retired (no longer generated by this command) but not reassigned --
 * see docs/command_reference.md's error-code table. */
void cmd_ext_trigger(uart_instance_t *inst, char *args);       /* EXTernal:TRIGger <0|1> --
                                                                    OK */
void cmd_ext_trigger_query(uart_instance_t *inst, char *args); /* EXTernal:TRIGger? -- OK
                                                                    <0|1> */
void cmd_ext_trigger_input_query(uart_instance_t *inst, char *args); /* EXTernal:TRIGger:INPut?
                                                                          -- OK <0|1>, raw PF15
                                                                          level, independent of
                                                                          whether the trigger
                                                                          feature is on */

/* Emergency stop (PG10, a fiber-optic input -- NOT PF15), added
 * 2026-09-17 -- see state_machine.h's own design comment
 * (SM_SetEmergencyStopRequired() and friends). Own top-level
 * `EMERGency:` namespace. Immediate, unconditional hard cutoff (no
 * ramp) when asserted (LOW) and this is on -- see
 * HandleEmergencyStopFault() (state_machine.c). "Acts as though it
 * does not exist" when off, per direct instruction -- no PG10 read at
 * all in that case. */
void cmd_emerg_enable(uart_instance_t *inst, char *args);       /* EMERGency:ENAble <0|1> --
                                                                     OK */
void cmd_emerg_enable_query(uart_instance_t *inst, char *args); /* EMERGency:ENAble? -- OK
                                                                     <0|1> */
void cmd_emerg_input_query(uart_instance_t *inst, char *args);  /* EMERGency:INPut? -- OK
                                                                     <0|1>, raw PG10 level,
                                                                     independent of whether
                                                                     the feature is on */

/* Generic diagnostic output on PD1 ("GPOut_12" in pin_mapping_v4.csv's
 * V4 column), added 2026-09-16 -- see cmd_diag_gpout12()'s own doc
 * comment (commands.c) for the full reasoning. Not gated behind a
 * TEMPORARY-removal marker like the OCP/GENERAL test-fault commands --
 * this is a generic, reusable diagnostic pin, not scaffolding tied to
 * one investigation. */
void cmd_diag_gpout12(uart_instance_t *inst, char *args);       /* DIAGnostic:GPOut12 <0|1>
                                                                     -- OK */
void cmd_diag_gpout12_query(uart_instance_t *inst, char *args); /* DIAGnostic:GPOut12? --
                                                                     OK <0|1> */

/* Generic diagnostic output on PD0 ("GPOut_11" in pin_mapping_v4.csv's
 * V4 column), added 2026-09-17 -- a SECOND, independent diagnostic
 * output, distinct from PD1/DIAGnostic:GPOut12 above. Direct
 * correction: PD1 was initially double-used for both PF15 (external-
 * enable) AND PG10 (emergency-stop) testing, which the user caught and
 * asked to be split onto genuinely separate pins -- see
 * cmd_diag_gpout11()'s own doc comment (commands.c). */
void cmd_diag_gpout11(uart_instance_t *inst, char *args);       /* DIAGnostic:GPOut11 <0|1>
                                                                     -- OK */
void cmd_diag_gpout11_query(uart_instance_t *inst, char *args); /* DIAGnostic:GPOut11? --
                                                                     OK <0|1> */

/* Generic diagnostic outputs on PG8/PG9 ("GPOut_09"/"GPOut_10" in
 * pin_mapping_v4.csv's V4 column -- verified against the CSV directly,
 * NOT the same pins as PD8/PD9 which carried those names under V3),
 * added 2026-09-18 -- feed the Transrex simulator's XR1_OCP/XR2_OCP
 * fiber transmitters, completing OCP coverage for all 4 channels. See
 * cmd_diag_gpout09()/cmd_diag_gpout10()'s own doc comment (commands.c)
 * for the full reasoning. */
void cmd_diag_gpout09(uart_instance_t *inst, char *args);       /* DIAGnostic:GPOut09 <0|1>
                                                                     -- OK */
void cmd_diag_gpout09_query(uart_instance_t *inst, char *args); /* DIAGnostic:GPOut09? --
                                                                     OK <0|1> */
void cmd_diag_gpout10(uart_instance_t *inst, char *args);       /* DIAGnostic:GPOut10 <0|1>
                                                                     -- OK */
void cmd_diag_gpout10_query(uart_instance_t *inst, char *args); /* DIAGnostic:GPOut10? --
                                                                     OK <0|1> */

/* TEMPORARY diagnostic, added 2026-09-17 -- see cmd_diag_optbytes_query()'s
 * own doc comment (commands.c) for the full reasoning: reads the live
 * FLASH_OPTR register to determine whether PB8 (BOOT0) is actually
 * available for GPIO reuse post-boot. Remove once the PB8/PG10
 * GPIO-reuse question is settled. */
void cmd_diag_optbytes_query(uart_instance_t *inst, char *args); /* DIAGnostic:OPTBytes? --
                                                                      OK OPTR=.. nBOOT0=..
                                                                      nSWBOOT0=.. nBOOT1=.. */

/* TEMPORARY diagnostic, added 2026-09-17 -- see cmd_diag_rstcause_query()'s
 * own doc comment (commands.c) for the full reasoning: reads the real
 * RCC->CSR reset-cause flags to test whether DIAGnostic:GPOut11 1's
 * garbled response is a genuine MCU reset (and if so, why), rather
 * than guessing. Remove once the GPOut11/PG10 investigation is
 * settled. */
void cmd_diag_rstcause_query(uart_instance_t *inst, char *args); /* DIAGnostic:RSTCause? --
                                                                      OK CSR=.. BOR=.. PIN=..
                                                                      SFT=.. IWDG=.. WWDG=..
                                                                      LPWR=.. OBL=.. */
void cmd_diag_rstcause_clear(uart_instance_t *inst, char *args); /* DIAGnostic:RSTCause:CLEar
                                                                      -- OK, clears all flags
                                                                      above via RCC_CSR_RMVF */

/* TEMPORARY debug/verification command, added 2026-09-15 -- software
 * fault injection for SM_ReportOcpFault() (state_machine.h), since no
 * real per-channel OCP pin is wired up anywhere yet (mapping still TBD
 * -- see that header's own "NOTE TO REVISIT"). See cmd_ocp_test_fault()'s
 * own doc comment (commands.c) for the full reasoning and removability
 * precedent (same pattern as PFMIN:DMASTAT?/qspi_test.c). */
void cmd_ocp_test_fault(uart_instance_t *inst, char *args);  /* OCP:TEST:FAULT <ch> -- OK,
                                                                  triggers SM_ReportOcpFault(ch-1) */

/* TEMPORARY debug/verification command, added 2026-09-15 -- software
 * fault injection for SM_ReportGeneralFault() (state_machine.h), the
 * General-Fault counterpart to OCP:TEST:FAULT above. See
 * cmd_general_test_fault()'s own doc comment (commands.c) for the full
 * reasoning and removability precedent. */
void cmd_general_test_fault(uart_instance_t *inst, char *args);  /* GENERAL:TEST:FAULT -- OK,
                                                                      triggers SM_ReportGeneralFault() */

/* Reports HRTIM_NUM_CHANNELS (ctrlr_config.h), the compile-time HRTIM
 * channel count this specific firmware build was configured for --
 * lets host tooling (python/pfm_table_upload.py) confirm what a board
 * actually is instead of silently assuming a value that might not
 * match, e.g. after a rebuild with a different channel count. */
void cmd_config_channels(uart_instance_t *inst, char *args); /* CONFig:CHANnels? -- OK <N> */

/* Raw GateDriverStatus_01..12 (PE0..PE11, see gate_driver.h) readback
 * -- a pure diagnostic snapshot, unaffected by and independent of
 * GDS_FAULT_POLARITY/the GateDriverStatus fault interrupt (same 12
 * pins, but a separate mechanism -- see gate_driver.h). One HIGH/LOW
 * state per pin, on a single OK line -- see commands.c for the exact
 * format. */
void cmd_gds_query(uart_instance_t *inst, char *args); /* GDS? -- OK 01=HIGH|LOW ... 12=HIGH|LOW */

/* Per-Transrex-channel fault-pin readback, added 2026-09-17 alongside
 * the new xrex_io.c module -- reports one channel's own Water/Temp/
 * Enerpro/OCP pins together, by name, rather than needing to remember
 * which of GDS?'s 12 raw pins (or the 4 new OCP pins) maps to which
 * signal for a given Transrex. Raw HIGH/LOW levels only, same
 * polarity-agnostic convention as GDS?/EXTernal:INPut?/
 * EMERGency:INPut? -- ctrlr_config.h's XR_WATER_FLT_POLARITY/etc. are
 * what decide which level actually means "faulted," not this command.
 * 1-based channel argument, matching this project's universal wire
 * convention (ERR 11 if out of range, ERR 12 if missing). See
 * xrex_io.h for the full pin-naming/gating design. */
void cmd_xrex_channel_status(uart_instance_t *inst, char *args); /* XREX:CHANnel:STATus? <ch> --
                                                                      OK WATER=HIGH|LOW TMP=HIGH|LOW
                                                                      ENERPRO=HIGH|LOW OCP=HIGH|LOW */

/* Real per-channel ENA_OUT/CONTACT_OUT fiber outputs (PG0-PG3/PG4-PG7),
 * added 2026-09-17, per direct request: "Enable and contactor fiber
 * outputs need to be set by a serial command, one for each supply."
 * See state_machine.h's own SM_FAULT_ENABLE_OUTPUT/enable-output
 * sections for what reads these -- ARM refuses unless every currently-
 * enabled channel's own pair is HIGH, and this is continuously
 * re-checked once ARMED. 1-based channel argument (ERR 11 if out of
 * range, ERR 12 if an argument is missing), same convention as every
 * other numbered-channel command. See xrex_io.h for the pin table. */
void cmd_xrex_ena_out(uart_instance_t *inst, char *args);          /* XREX:CHANnel:ENAOut <ch>
                                                                        <0|1> -- OK */
void cmd_xrex_ena_out_query(uart_instance_t *inst, char *args);    /* XREX:CHANnel:ENAOut?
                                                                        <ch> -- OK <0|1> */
void cmd_xrex_contact_out(uart_instance_t *inst, char *args);      /* XREX:CHANnel:CONTactOut
                                                                        <ch> <0|1> -- OK */
void cmd_xrex_contact_out_query(uart_instance_t *inst, char *args);/* XREX:CHANnel:CONTactOut?
                                                                        <ch> -- OK <0|1> */

/* PC13 ("GPOut_Enable_Pin" in pin_mapping_v4.csv's V4 column), added
 * 2026-09-17, per direct request -- default HIGH at boot (main.c). See
 * cmd_gpout_enable()'s own doc comment (commands.c). */
void cmd_gpout_enable(uart_instance_t *inst, char *args);       /* GPOut:ENAble <0|1> -- OK */
void cmd_gpout_enable_query(uart_instance_t *inst, char *args); /* GPOut:ENAble? -- OK <0|1> */

/* PC15 ("PWM_Alt_Enable" in pin_mapping_v4.csv's V4 column), added
 * 2026-09-17, per direct request -- default HIGH at boot (main.c),
 * exact mirror of GPOut:ENAble above. See cmd_pwmalt_enable()'s own
 * doc comment (commands.c). */
void cmd_pwmalt_enable(uart_instance_t *inst, char *args);       /* PWMAlt:ENAble <0|1> -- OK */
void cmd_pwmalt_enable_query(uart_instance_t *inst, char *args); /* PWMAlt:ENAble? -- OK <0|1> */

/* QUADSPI connectivity test (PE12-PE15/PB10-PB11, W25Q128JVS) -- see
 * qspi_test.h. Issues the flash's standard JEDEC Read ID instruction
 * and reports the 3-byte ID; nothing else (no program/erase, no
 * memory-mapped access). Gated on QSPI_TEST_FEATURE_ENABLED
 * (qspi_test.h), same removability pattern as cmd_boot() above --
 * entirely absent, including cmd_parser.c's "QSPI:ID?" table row, when
 * that module is disabled. */
#if (QSPI_TEST_FEATURE_ENABLED != 0)
void cmd_qspi_id(uart_instance_t *inst, char *args); /* QSPI:ID? -- OK <MFR> <TYPE> <CAP>, hex bytes */
#endif

/* PFM_Input period/duty capture (PA15/PD4/PB2/PC12/PB4/PD12, TIM2/TIM3/
 * TIM4/TIM5) -- see pfm_input.h. Capture is synchronized to PFM shot
 * lifetime, not these commands directly: PFMIN:CAPTURE only arms a
 * target period count per channel for the NEXT FIRE (PfmInput_Arm());
 * the actual hardware capture starts inside PFM_Restart() (pfm.c) and
 * stops wherever pfm.c stops the shot, bounding a capture's runtime to
 * the shot that started it. Gated on PFM_INPUT_FEATURE_ENABLED
 * (pfm_input.h), same removability pattern as cmd_boot()/cmd_qspi_id()
 * above. */
#if (PFM_INPUT_FEATURE_ENABLED != 0)
void cmd_pfmin_capture(uart_instance_t *inst, char *args); /* PFMIN:CAPTURE <M> -- OK,
                                                                arms all 6 channels for
                                                                the next FIRE */
void cmd_pfmin_status(uart_instance_t *inst, char *args);  /* PFMIN:STATus? -- OK <n1>
                                                                .. <n6>, current
                                                                captured-period counts */
void cmd_pfmin_data(uart_instance_t *inst, char *args);    /* PFMIN:DATA? <ch> -- OK
                                                                <count> <per1> <per2> ...,
                                                                raw ticks (period only,
                                                                no duty), ch = 1-6 */

/* TEMPORARY debug command, 2026-09-09 -- see pfm_input.h's own
 * comment on PfmInput_GetDmaStartStatus(). Remove once DMA capture is
 * confirmed reliable. */
void cmd_pfmin_dmastat(uart_instance_t *inst, char *args); /* PFMIN:DMASTAT? -- OK
                                                                <s1> .. <s6>, last
                                                                HAL_TIM_IC_Start_DMA()
                                                                return code per channel */

/* TEMPORARY debug command, added 2026-09-15 -- see
 * cmd_pfmin_debug_raw()'s own doc comment (commands.c) and
 * PfmInput_GetDebugRaw()'s (pfm_input.h) for the full reasoning:
 * diagnosing why measuredHz reads 0 for WHAM channels 2/3/4. Remove
 * once the root cause is found and fixed. */
void cmd_pfmin_debug_raw(uart_instance_t *inst, char *args); /* PFMIN:DEBUG:RAW? <ch> --
                                                                  OK cont=.. run=..
                                                                  firstRise=.. avgCount=..
                                                                  lastPeriod=.. overcap=.. */

/* TEMPORARY debug command, added 2026-09-15 -- see
 * cmd_pfmin_debug_reg()'s own doc comment (commands.c) and
 * PfmInput_GetDebugRegs()'s (pfm_input.h) for the full reasoning: raw
 * TIMx register readback. Remove once the root cause is found and
 * fixed. */
void cmd_pfmin_debug_reg(uart_instance_t *inst, char *args); /* PFMIN:DEBUG:REG? <ch> --
                                                                  OK CR1=.. CCER=.. DIER=..
                                                                  SR=.. CNT=.. CCR=.. */
#endif

/* --------------------------------------------------------------------------
 * PID:* -- closed-loop control, added 2026-09-09. See pid.h for the
 * full architecture (Possibility 3 + fixed-rate Master heartbeat) this
 * project exists to implement, and docs/changelog.txt's design-decision
 * entry for the reasoning. Channel numbering matches PFMIN:DATA?'s own
 * convention: 1..N on the wire (N = HRTIM_NUM_CHANNELS, CONFig:CHANnels?
 * reports it), 0..N-1 internally. Not gated on a feature-enable flag --
 * this project's whole point, unlike PFM_Input/QUADSPI/BOOT's opt-in
 * modules. */
void cmd_pid_start(uart_instance_t *inst, char *args);    /* PID:START -- OK, begins
                                                               closed-loop operation on
                                                               every channel */
void cmd_pid_stop(uart_instance_t *inst, char *args);     /* PID:STOP -- OK, stops
                                                               output + feedback capture */
void cmd_pid_setpoint(uart_instance_t *inst, char *args); /* PID:SETPOINT <ch> <hz> --
                                                               OK, sets channel ch's
                                                               target output frequency */
void cmd_pid_gains(uart_instance_t *inst, char *args);    /* PID:GAINS <ch> <kp> <ki>
                                                               <kd> -- OK, sets channel
                                                               ch's PID gains, resets
                                                               its integrator */
void cmd_pid_gains_query(uart_instance_t *inst, char *args); /* PID:GAINS? <ch> -- OK
                                                                  <kp> <ki> <kd> -- added
                                                                  2026-09-10, see
                                                                  PID_GetGains() */
void cmd_pid_status(uart_instance_t *inst, char *args);   /* PID:STATus? <ch> -- OK
                                                               <running> <setpointHz>
                                                               <measuredHz> <outputHz> */

/* Waveform logging, added 2026-09-10 -- see pid.h's own comment block
   on PID_ArmLog() for the full design (why polling PID:STATus? isn't
   enough to actually plot a waveform). <ch>=0 means "log every
   channel at once" (PID_ArmLogAll(), see its own pid.h comment) --
   added the same day for a genuine simultaneous cross-channel
   comparison. */
void cmd_pid_log(uart_instance_t *inst, char *args);       /* PID:LOG <ch(0=all)>
                                                                <maxSamples> <decim> -- OK,
                                                                arms waveform logging */
void cmd_pid_logdata(uart_instance_t *inst, char *args);   /* PID:LOGDATA? [ch] -- OK <count>
                                                                <rateHz> s1 m1 o1 s2 m2 o2
                                                                ..., setpoint/measured/
                                                                output Hz triples. [ch]
                                                                optional if a single
                                                                channel is armed
                                                                (unchanged, original
                                                                behavior); required (and
                                                                any channel valid) under
                                                                all-channels mode */

/* Linear setpoint ramp, added 2026-09-10 for trajectory-tracking bench
   tests -- see pid.h's own comment on PID_StartRamp(). */
void cmd_pid_ramp(uart_instance_t *inst, char *args);       /* PID:RAMP <ch> <startHz>
                                                                 <endHz> <durationMs> --
                                                                 OK, begins a linear
                                                                 setpoint ramp */

/* Production shot profile + open/closed-loop mode, added 2026-09-10 --
   see pid.h's "DEMAND PROFILE"/"OPEN-LOOP MODE" doc sections and
   PID_SetLoopMode()/PID_SetProfileTiming()/PID_SetProfileCurrent()/
   PID_ProfileStart()'s own comments for the full design. */
void cmd_pid_loopmode(uart_instance_t *inst, char *args);       /* PID:LOOPMODE <ch>
                                                                     <0|1> -- OK, 0 =
                                                                     open-loop, 1 =
                                                                     closed-loop
                                                                     (default) */
void cmd_pid_loopmode_query(uart_instance_t *inst, char *args); /* PID:LOOPMODE? <ch>
                                                                     -- OK <0|1> --
                                                                     added 2026-09-10,
                                                                     see PID_GetLoopMode() */

/* Per-channel output enable/disable, added 2026-09-11 per direct
   request -- see PID_SetChannelEnable()'s own doc comment in pid.h
   for exactly what this does and doesn't do (distinct from
   PID:LOOPMODE -- this is "no PFM waveform at all", not "uncorrected
   PFM waveform"). */
void cmd_pid_channel_enable(uart_instance_t *inst, char *args);       /* PID:CHANnel:ENAble
                                                                           <ch> <0|1> -- OK,
                                                                           0 = output fully
                                                                           disabled (no PFM
                                                                           waveform at all),
                                                                           1 = enabled
                                                                           (default) */
void cmd_pid_channel_enable_query(uart_instance_t *inst, char *args); /* PID:CHANnel:ENAble?
                                                                           <ch> -- OK <0|1> */

/* Per-channel human-readable nickname, added 2026-09-13 per direct
   request -- purely a label, see PID_SetChannelNickname()'s own doc
   comment in pid.h for the full story (max length, reserved "-"
   sentinel, why it can't contain spaces). */
void cmd_pid_channel_nickname(uart_instance_t *inst, char *args);       /* PID:CHANnel:NICKname
                                                                             <ch> <name> -- OK,
                                                                             or ERR 14 if name
                                                                             is invalid */
void cmd_pid_channel_nickname_query(uart_instance_t *inst, char *args); /* PID:CHANnel:NICKname?
                                                                             <ch> -- OK <name>,
                                                                             or OK - if none set */

void cmd_pid_profile_timing(uart_instance_t *inst, char *args);  /* PID:PROFILE:TIMING
                                                                      <rampTimeS>
                                                                      <flatTopTimeS> --
                                                                      OK, sets the SHARED
                                                                      shot timing (every
                                                                      channel) */
void cmd_pid_profile_timing_query(uart_instance_t *inst, char *args); /* PID:PROFILE:TIMING?
                                                                           -- OK <rampTimeS>
                                                                           <flatTopTimeS> --
                                                                           added 2026-09-10,
                                                                           see
                                                                           PID_GetProfileTiming();
                                                                           ERR 12 if never set */
void cmd_pid_profile_current(uart_instance_t *inst, char *args); /* PID:PROFILE:CURRENT
                                                                      <ch> <demandCurrentA>
                                                                      -- OK, sets channel
                                                                      ch's peak current
                                                                      for the next shot */
void cmd_pid_profile_current_query(uart_instance_t *inst, char *args); /* PID:PROFILE:CURRENT?
                                                                            <ch> -- OK
                                                                            <demandCurrentA>
                                                                            -- added 2026-09-10,
                                                                            see
                                                                            PID_GetProfileCurrent() */
void cmd_pid_profile_start(uart_instance_t *inst, char *args);   /* PID:PROFILE:START --
                                                                      OK, begins a
                                                                      profiled shot on
                                                                      every channel at
                                                                      once */

/* SIM: namespace -- sim_transrex.h backed, SIMULATOR-ONLY, added
 * 2026-09-18. See commands.c's own header comment on
 * cmd_sim_fault_watertemp() for the full reasoning. Guarded out of a
 * controller build entirely -- these declarations, their definitions,
 * and their cmd_parser.c registration all share the same
 * BUILD_TARGET_SIMULATOR guard. */
#if defined(BUILD_TARGET_SIMULATOR)
void cmd_sim_fault_watertemp(uart_instance_t *inst, char *args);       /* SIM:FAULT:WATERTEMP
                                                                            <ch> <0|1> -- OK */
void cmd_sim_fault_watertemp_query(uart_instance_t *inst, char *args); /* SIM:FAULT:WATERTEMP?
                                                                            <ch> -- OK <0|1> */
void cmd_sim_fault_enerpro(uart_instance_t *inst, char *args);         /* SIM:FAULT:ENERPRO
                                                                            <ch> <0|1> -- OK */
void cmd_sim_fault_enerpro_query(uart_instance_t *inst, char *args);   /* SIM:FAULT:ENERPRO?
                                                                            <ch> -- OK <0|1> */
void cmd_sim_fault_ocp(uart_instance_t *inst, char *args);             /* SIM:FAULT:OCP <ch>
                                                                            <0|1> -- OK */
void cmd_sim_fault_ocp_query(uart_instance_t *inst, char *args);       /* SIM:FAULT:OCP? <ch>
                                                                            -- OK <0|1> */
void cmd_sim_model_tau(uart_instance_t *inst, char *args);             /* SIM:MODEL:TAU <ms>
                                                                            -- OK */
void cmd_sim_model_tau_query(uart_instance_t *inst, char *args);       /* SIM:MODEL:TAU? --
                                                                            OK <ms> */
void cmd_sim_channel_status(uart_instance_t *inst, char *args);        /* SIM:CHANnel:STATus?
                                                                            <ch> -- OK
                                                                            DRIVE_HZ=... */
void cmd_sim_log(uart_instance_t *inst, char *args);                   /* SIM:LOG <ch>
                                                                            <maxSamples>
                                                                            <minIntervalMs>
                                                                            -- OK */
void cmd_sim_logdata(uart_instance_t *inst, char *args);               /* SIM:LOGDATA? <ch>
                                                                            -- OK <count>
                                                                            <t0> <drive0>
                                                                            <feedback0> ... */
#endif /* BUILD_TARGET_SIMULATOR */

#ifdef __cplusplus
}
#endif

#endif /* __COMMANDS_H__ */
