/*
 * task_scpi.c -- the serial command link: hands each completed line to the
 * command table (commands.h, command_table.c).
 */
#include "tasks.h"
#include "uart.h"
#include "commands.h"
#include <stddef.h>

void TaskScpi_Poll(void)
{
    char *line = uart_take_line(&uart2);
    if (line != NULL)
    {
        Commands_Dispatch(&uart2, line);
    }
}
