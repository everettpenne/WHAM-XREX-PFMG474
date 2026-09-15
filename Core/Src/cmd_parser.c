/*
 * cmd_parser.c
 *
 * Tokenises received lines and dispatches to command handlers, using
 * SCPI-style hierarchical mnemonics (IEEE 488.2 / SCPI-1999 subset).
 *
 * To add a command
 * -----------------
 *  1. Implement handler in commands.c
 *  2. Declare in commands.h
 *  3. Add a { "PATTern:MNEMonic?", handler } row to command_table[]
 *     below. Nothing else changes -- the table is flat (no separate
 *     tree structure to build), so adding a new leaf or a whole new
 *     subsystem is always just one more row.
 *
 * Pattern syntax
 * --------------
 *  - Colon-separated hierarchy levels, e.g. "SOURce:VOLTage:LIMit".
 *    A leading colon on the *input* is tolerated (stripped) but never
 *    required, since there is no notion of a "current path" here --
 *    every command is matched against the full table from the root,
 *    same as sending one command per line with no compound (';')
 *    commands. (Compound commands aren't supported; add them later by
 *    splitting the line on ';' before tokenising if ever needed.)
 *  - Per-level SCPI short/long form: in each pattern token, the
 *    leading run of UPPERCASE letters is the mandatory short form;
 *    any lowercase letters after it are the optional long-form
 *    suffix. An input token must match the mandatory part
 *    case-insensitively, and if it's longer than that, must match the
 *    *entire* token (short+long) case-insensitively -- there is no
 *    such thing as a partial-long-form match (see scpi_token_match()).
 *    Example: pattern "VERSion" accepts "VERS", "VERSION", "version",
 *    "Version" -- but not "VERSI" or "VERSIO".
 *  - A trailing '?' marks a query and must match exactly: a pattern
 *    ending in '?' only matches an input also ending in '?', and vice
 *    versa. Common (IEEE 488.2) commands like "*IDN?" or "*RST" are
 *    just zero-colon patterns -- the same matcher handles them with
 *    no special-casing.
 *
 * See scpi_match()/scpi_token_match() below for the implementation;
 * both are self-contained and were verified against a table of
 * matching/non-matching cases (short form, long form, mixed, wrong
 * depth, wrong query suffix, leading colon) before being wired in
 * here.
 */

#include "cmd_parser.h"
#include "commands.h"
#include "uart.h"
#include "boot_jump.h"
#include "qspi_test.h"
#include "pfm_input.h"
#include <string.h>
#include <strings.h>
#include <ctype.h>

typedef void (*cmd_handler_t)(uart_instance_t *inst, char *args);

typedef struct {
    const char    *pattern;
    cmd_handler_t  handler;
} command_t;

static const command_t command_table[] = {
    /* Common commands (IEEE 488.2) */
    { "*IDN?",              cmd_idn          },

    /* Serial-bootloader entry -- excluded entirely (not just an
       unreachable row) when BOOT_JUMP_FEATURE_ENABLED is 0, so BOOT
       falls through to "ERR 1 Unknown command" like any other
       unrecognized mnemonic. See boot_jump.h. */
#if (BOOT_JUMP_FEATURE_ENABLED != 0)
    { "BOOT",                cmd_boot         },
#endif

    /* PFM table upload -- see commands.c's own header comment on
       these four handlers. */
    { "TABle:BEGin",         cmd_table_begin  },
    { "TABle:STEP",          cmd_table_step   },
    { "TABle:END",           cmd_table_end    },
    { "TABle?",              cmd_table_query  },

    /* Begins PWM output -- see commands.c's own header comment on
       cmd_fire(). */
    { "FIRE",                cmd_fire         },

    /* PFM table-advance ISR timing diagnostic -- see commands.c's own
       header comment on cmd_pfm_diag(). */
    { "PFM:DIAG?",           cmd_pfm_diag     },

    /* TEMPORARY, see commands.h -- raw per-call gap log behind
       PFM:DIAG?'s aggregate max-gap number. */
    { "PFM:GAPLOG?",         cmd_pfm_gaplog   },

    /* Reports the compile-time HRTIM channel count -- see commands.c's
       own header comment on cmd_config_channels(). */
    { "CONFig:CHANnels?",    cmd_config_channels },

    /* PC10/HRTIM1_FLT6 hardware fault status/clear -- see commands.c's
       own header comment on cmd_fault_query()/cmd_fault_clear(). */
    { "FAULT?",              cmd_fault_query  },
    { "FAULT:CLEar",         cmd_fault_clear  },

    /* Top-level operating-state machine (IDLE/ARMED/FIRING/FAULT) --
       see commands.c's own header comment on cmd_arm()/cmd_disarm()/
       cmd_state_query() and state_machine.h for the full design. */
    { "ARM",                 cmd_arm          },
    { "DISARM",              cmd_disarm       },
    { "STATE?",              cmd_state_query  },

    /* TEMPORARY debug/verification command -- software OCP fault
       injection, see commands.c's own header comment on
       cmd_ocp_test_fault(). */
    { "OCP:TEST:FAULT",      cmd_ocp_test_fault },

    /* Raw GateDriverStatus_01..12 (PE0..PE11) diagnostic readback --
       see commands.c's own header comment on cmd_gds_query(). */
    { "GDS?",                cmd_gds_query    },

    /* QUADSPI connectivity test (W25Q128JVS) -- excluded entirely when
       QSPI_TEST_FEATURE_ENABLED is 0, same removability pattern as
       BOOT above. See commands.c's own header comment on
       cmd_qspi_id(). */
#if (QSPI_TEST_FEATURE_ENABLED != 0)
    { "QSPI:ID?",            cmd_qspi_id      },
#endif

    /* PFM_Input period/duty capture -- excluded entirely when
       PFM_INPUT_FEATURE_ENABLED is 0, same removability pattern as
       BOOT/QSPI:ID? above. See commands.c's own header comment on
       cmd_pfmin_capture()/cmd_pfmin_status()/cmd_pfmin_data(). */
#if (PFM_INPUT_FEATURE_ENABLED != 0)
    { "PFMIN:CAPTURE",       cmd_pfmin_capture },
    { "PFMIN:STATus?",       cmd_pfmin_status  },
    { "PFMIN:DMASTAT?",      cmd_pfmin_dmastat },  /* TEMPORARY, see commands.h */
    { "PFMIN:DATA?",         cmd_pfmin_data    },
#endif

    /* Closed-loop PID control -- this project's whole point, see
       commands.c's own header comment on the cmd_pid_*() handlers and
       pid.h for the architecture. Not gated on a feature-enable flag,
       unlike the modules above. */
    { "PID:START",           cmd_pid_start     },
    { "PID:STOP",            cmd_pid_stop      },
    { "PID:SETPOINT",        cmd_pid_setpoint  },
    { "PID:GAINS",           cmd_pid_gains     },
    { "PID:GAINS?",          cmd_pid_gains_query },
    { "PID:STATus?",         cmd_pid_status    },
    { "PID:LOG",             cmd_pid_log       },
    { "PID:LOGDATA?",        cmd_pid_logdata   },
    { "PID:RAMP",            cmd_pid_ramp      },

    /* Production shot profile + open/closed-loop mode, added
       2026-09-10 -- see commands.h's own header comment on these
       handlers and pid.h's "DEMAND PROFILE"/"OPEN-LOOP MODE" doc
       sections for the full design. */
    { "PID:LOOPMODE",        cmd_pid_loopmode        },
    { "PID:LOOPMODE?",       cmd_pid_loopmode_query        },
    { "PID:CHANnel:ENAble",  cmd_pid_channel_enable        },
    { "PID:CHANnel:ENAble?", cmd_pid_channel_enable_query  },
    { "PID:CHANnel:NICKname",  cmd_pid_channel_nickname        },
    { "PID:CHANnel:NICKname?", cmd_pid_channel_nickname_query  },
    { "PID:PROFile:TIMing",  cmd_pid_profile_timing  },
    { "PID:PROFile:TIMing?", cmd_pid_profile_timing_query  },
    { "PID:PROFile:CURRent", cmd_pid_profile_current },
    { "PID:PROFile:CURRent?", cmd_pid_profile_current_query },
    { "PID:PROFile:STARt",   cmd_pid_profile_start   },
};

#define NUM_COMMANDS  (sizeof(command_table) / sizeof(command_table[0]))

/*
 * scpi_token_match()
 *
 * Matches one ':'-level of an input mnemonic against one level of a
 * table pattern, per the short/long-form rule described in the file
 * header. `pattern` is one of our own table strings (trusted, always
 * NUL-terminated); `input` is operator-supplied.
 */
static int scpi_token_match(const char *pattern, const char *input)
{
    size_t mandatory_len = 0;
    while (pattern[mandatory_len] != '\0' &&
           isupper((unsigned char)pattern[mandatory_len])) {
        mandatory_len++;
    }
    size_t pattern_len = strlen(pattern);
    size_t input_len   = strlen(input);

    if (input_len < mandatory_len) {
        return 0;
    }
    if (strncasecmp(pattern, input, mandatory_len) != 0) {
        return 0;
    }
    if (input_len == mandatory_len) {
        return 1; /* short form used */
    }
    if (input_len != pattern_len) {
        return 0; /* neither short nor exactly the full long form */
    }
    return strncasecmp(pattern, input, pattern_len) == 0; /* long form used */
}

/*
 * scpi_match()
 *
 * Matches a full mnemonic (the whitespace-delimited first word of the
 * command line, e.g. "SOUR:VOLT:LIM?") against one command_table[]
 * pattern (e.g. "SOURce:VOLTage:LIMit?"), level by level.
 *
 * Copies both strings into fixed local buffers before calling
 * strtok_r() on them, since the two tokenisations are interleaved
 * (one level of pattern, then one level of input, repeat) and plain
 * strtok()'s single hidden save-pointer can't do that -- and because
 * `cmd` here is a pointer into uart.c's shared line buffer, which
 * this function must not mutate.
 */
static int scpi_match(const char *pattern, const char *cmd)
{
    size_t plen = strlen(pattern);
    size_t clen = strlen(cmd);

    int p_query = (plen > 0 && pattern[plen - 1] == '?');
    int c_query = (clen > 0 && cmd[clen - 1] == '?');
    if (p_query != c_query) {
        return 0;
    }

    char pbuf[UART_RX_BUF_SIZE];
    char cbuf[UART_RX_BUF_SIZE];
    if (plen >= sizeof(pbuf) || clen >= sizeof(cbuf)) {
        return 0; /* can't happen for our own patterns; defends cmd length */
    }

    memcpy(pbuf, pattern, plen - (size_t)p_query);
    pbuf[plen - (size_t)p_query] = '\0';
    memcpy(cbuf, cmd, clen - (size_t)c_query);
    cbuf[clen - (size_t)c_query] = '\0';

    char *cin = cbuf;
    if (*cin == ':') {
        cin++; /* tolerate (don't require) a leading colon on input */
    }

    char *psave = NULL;
    char *csave = NULL;
    char *ptok = strtok_r(pbuf, ":", &psave);
    char *ctok = strtok_r(cin,  ":", &csave);

    while (ptok != NULL && ctok != NULL) {
        if (!scpi_token_match(ptok, ctok)) {
            return 0;
        }
        ptok = strtok_r(NULL, ":", &psave);
        ctok = strtok_r(NULL, ":", &csave);
    }
    return (ptok == NULL && ctok == NULL); /* both exhausted = same depth */
}

void dispatch_command(uart_instance_t *inst, char *buf)
{
    char *cmd  = strtok(buf, " \r\n");
    char *args = strtok(NULL, "\r\n");

    if (cmd == NULL) return;

    for (int i = 0; i < (int)NUM_COMMANDS; i++) {
        if (scpi_match(command_table[i].pattern, cmd)) {
            command_table[i].handler(inst, args);
            return;
        }
    }

    uart_send(inst, "ERR 1 Unknown command\r\n");
}
