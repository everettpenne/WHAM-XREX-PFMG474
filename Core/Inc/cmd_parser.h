#ifndef __CMD_PARSER_H__
#define __CMD_PARSER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "uart.h"

/* Tokenises a received line and dispatches to the matching command
   handler from command_table[] in cmd_parser.c. Called from
   uart_process() once a full line has been accumulated. */
void dispatch_command(uart_instance_t *inst, char *buf);

#ifdef __cplusplus
}
#endif

#endif /* __CMD_PARSER_H__ */
