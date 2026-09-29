/*
 * scpi_parser.c
 *
 * Splits a received line into mnemonic + arguments and dispatches it to
 * the first matching entry of a caller-supplied command table, using
 * SCPI-style hierarchical mnemonics (IEEE 488.2 / SCPI-1999 subset).
 * Knows nothing about the product's commands -- the table lives in
 * src/app/commands/command_table.c -- so it can be unit-tested on a PC
 * (tests/test_scpi_parser.c).
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
 * tests/test_scpi_parser.c covers the matching rules (short form, long
 * form, mixed, wrong depth, wrong query suffix, leading colon).
 */

#include "scpi_parser.h"
#include <string.h>
#include <strings.h>
#include <ctype.h>

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
 * command line, e.g. "SOUR:VOLT:LIM?") against one table
 * pattern (e.g. "SOURce:VOLTage:LIMit?"), level by level.
 *
 * Copies both strings into fixed local buffers before calling
 * strtok_r() on them, since the two tokenisations are interleaved
 * (one level of pattern, then one level of input, repeat) and plain
 * strtok()'s single hidden save-pointer can't do that -- and because
 * `cmd` here is a pointer into the serial link's shared line buffer,
 * which this function must not mutate.
 */
int scpi_match(const char *pattern, const char *cmd)
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

scpi_result_t scpi_dispatch(const scpi_command_t *table, size_t count,
                            uart_instance_t *inst, char *line)
{
    char *cmd  = strtok(line, " \r\n");
    char *args = strtok(NULL, "\r\n");

    if (cmd == NULL) return SCPI_EMPTY;

    for (int i = 0; i < (int)count; i++) {
        if (scpi_match(table[i].pattern, cmd)) {
            table[i].handler(inst, args);
            return SCPI_HANDLED;
        }
    }
    return SCPI_UNKNOWN;
}
