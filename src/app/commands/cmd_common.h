/*
 * cmd_common.h -- shared pieces of the serial command layer: the error
 * codes of the wire protocol and the reply helper.
 *
 * Replies are "OK\r\n", "OK <value>\r\n" or "ERR <n> <msg>\r\n". The
 * numbers below are a frozen wire contract (docs/command_reference.md,
 * "Error codes"): never renumber or reuse one.
 */
#ifndef CMD_COMMON_H
#define CMD_COMMON_H

#include <stdint.h>
#include "uart.h"

typedef enum
{
    ERR_UNKNOWN_COMMAND     = 1,   /* no command_table[] pattern matched */
    ERR_TABLE_NOT_UPLOADING = 2,   /* TABle:STEP/END without TABle:BEGin */
    ERR_TABLE_FULL          = 3,   /* PFM_TABLE_SIZE entries already appended */
    ERR_TABLE_STEP_INVALID  = 4,   /* TABle:STEP: wrong count (1 + HRTIM_NUM_CHANNELS)
                                      or a value outside uint16 */
    ERR_TABLE_EMPTY         = 5,   /* FIRE with nothing uploaded */
    ERR_FAULT_LATCHED       = 6,   /* PC10/HRTIM1_FLT6 or GateDriverStatus latch --
                                      FAULT:CLEar first */
    ERR_QSPI_FAILED         = 7,   /* QSPI:ID? command failed or timed out */
    ERR_PFMIN_BAD_CHANNEL   = 8,   /* PFM_Input channel not 1-6 */
    ERR_PFMIN_BAD_COUNT     = 9,   /* PFMIN:CAPTURE M out of 1-PFM_INPUT_MAX_PERIODS */
    ERR_CARRIER_TOO_HIGH    = 10,  /* TABle:STEP per above the max carrier frequency */
    ERR_INVALID_CHANNEL     = 11,  /* channel not 1-HRTIM_NUM_CHANNELS. Also sent,
                                      inconsistently, for a few non-channel values
                                      (0/1 flags, SIM:MODEL:TAU) */
    ERR_INVALID_ARGS        = 12,  /* wrong argument count, or non-numeric/out-of-range */
    ERR_INVALID_STATE       = 13,  /* not allowed in the current state machine or
                                      firmware-update state */
    ERR_INVALID_NICKNAME    = 14,  /* CHANnel:NICKname rules, see pid.h */
    ERR_EXT_ENABLE_LOW      = 15,  /* EXTernal:ENAble on and PF13 reads LOW */
    /* 16: retired 2026-09-17 (old EXTernal:TRIGger coupling) -- do not reuse */
    ERR_FWUPDATE            = 17   /* FWUPdate:* flash/CRC/image/option-byte failure */
} cmd_err_t;

/* Send "ERR <code> <msg>\r\n". code is an int, not cmd_err_t, so every call
   site compiles exactly as it did with literal numbers. */
void SendErr(uart_instance_t *inst, int code, const char *msg);

/* True if either fault source (PC10/HRTIM1_FLT6 or GateDriverStatus) is
   latched -- FAULT? and FIRE's ERR 6 gate. */
uint8_t AnyFaultLatched(void);

/* Fire-attempt precondition shared by FIRE, SOURce:RUN and SHOT:STARt:
   ARMED and the external-enable interlock satisfied. Returns 1 to proceed;
   0 means an ERR has already been sent. */
uint8_t RequireArmedAndEnabled(uart_instance_t *inst);

#endif /* CMD_COMMON_H */
