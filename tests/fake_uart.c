/*
 * fake_uart.c -- host stand-in for the serial link (drivers/uart.h):
 * uart_send() appends to a buffer the tests can inspect.
 */
#include "uart.h"
#include "fake_uart.h"
#include <string.h>

struct uart_instance { int unused; };
uart_instance_t uart2;

char fake_uart_out[4096];

void fake_uart_clear(void)
{
    fake_uart_out[0] = '\0';
}

void uart_send(uart_instance_t *inst, const char *str)
{
    (void)inst;
    strncat(fake_uart_out, str, sizeof(fake_uart_out) - strlen(fake_uart_out) - 1);
}

void uart_start(uart_instance_t *inst) { (void)inst; }
char *uart_take_line(uart_instance_t *inst) { (void)inst; return NULL; }
