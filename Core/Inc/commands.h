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
#endif

#ifdef __cplusplus
}
#endif

#endif /* __COMMANDS_H__ */
