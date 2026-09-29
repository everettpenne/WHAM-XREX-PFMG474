/*
 * scpi_parser.h -- SCPI-style mnemonic matching and table dispatch. The
 * matching rules are described in scpi_parser.c.
 */
#ifndef SCPI_PARSER_H
#define SCPI_PARSER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include "uart.h"

typedef void (*scpi_handler_t)(uart_instance_t *inst, char *args);

typedef struct {
    const char     *pattern;   /* e.g. "SOURce:SETpoint", "*IDN?" */
    scpi_handler_t  handler;
} scpi_command_t;

typedef enum {
    SCPI_HANDLED,   /* a table entry matched and its handler ran */
    SCPI_EMPTY,     /* blank line -- nothing to do, no reply */
    SCPI_UNKNOWN    /* no entry matched -- the caller replies */
} scpi_result_t;

/* 1 if mnemonic `cmd` (no arguments) matches table pattern `pattern`. */
int scpi_match(const char *pattern, const char *cmd);

/* Split `line` (modified in place) into mnemonic + argument string and run
   the first matching handler in `table`. */
scpi_result_t scpi_dispatch(const scpi_command_t *table, size_t count,
                            uart_instance_t *inst, char *line);

#ifdef __cplusplus
}
#endif

#endif /* SCPI_PARSER_H */
