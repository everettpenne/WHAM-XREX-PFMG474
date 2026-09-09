/*
 * uart.h
 *
 * Ported from the sibling PFM-STM32G474 project's serial command
 * architecture (uart.c/cmd_parser.c/commands.c split) -- see that
 * project's AGENTS.md for the full design rationale.
 *
 * Single-byte interrupt-driven UART receive with line-buffer accumulation.
 * Transmit is blocking (HAL_UART_Transmit).
 *
 * One uart_instance_t per UART peripheral.  All state is inside the struct,
 * so multiple instances are independent.
 */

#ifndef INC_UART_H_
#define INC_UART_H_

#include "main.h"

#define UART_RX_BUF_SIZE    128

typedef struct {
    UART_HandleTypeDef *huart;
    char                rx_line[UART_RX_BUF_SIZE];  /* accumulation buffer   */
    char                line_buf[UART_RX_BUF_SIZE];  /* stable copy for parse */
    volatile uint8_t    rx_byte;                      /* single-byte DMA/IT    */
    uint8_t             rx_index;
    volatile uint8_t    line_ready;
} uart_instance_t;

/* Global instances — one per peripheral */
extern uart_instance_t uart2;

void uart_init(uart_instance_t *inst, UART_HandleTypeDef *huart);
void uart_send(uart_instance_t *inst, const char *str);
void uart_process(uart_instance_t *inst);
void uart_rx_callback(uart_instance_t *inst);

#endif /* INC_UART_H_ */
