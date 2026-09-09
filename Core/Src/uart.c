/*
 * uart.c
 *
 * Ported verbatim from the sibling PFM-STM32G474 project's serial
 * command architecture -- pin/board-agnostic (buffer/line-accumulation
 * logic only), so it needed no adaptation for this project's pinout.
 *
 * Single-byte interrupt-driven receive with line-buffer accumulation.
 * Each call to HAL_UART_Receive_IT() arms for exactly one byte; the ISR
 * fires uart_rx_callback(), which accumulates into rx_line until a line
 * terminator is seen, then copies to line_buf and sets line_ready.
 *
 * uart_process() is polled from the main loop; it calls dispatch_command()
 * when a complete line is ready.  No RTOS or DMA required.
 */

#include "uart.h"
#include "cmd_parser.h"
#include <string.h>

/* Global UART instance for USART2 */
uart_instance_t uart2;

void uart_init(uart_instance_t *inst, UART_HandleTypeDef *huart)
{
    inst->huart      = huart;
    inst->rx_index   = 0;
    inst->line_ready = 0;

    /* Arm the first receive interrupt */
    if (HAL_UART_Receive_IT(inst->huart, (uint8_t *)&inst->rx_byte, 1) != HAL_OK) {
        /* Handle error - maybe add debug output */
        char err[] = "UART RX IT Failed!\r\n";
        HAL_UART_Transmit(inst->huart, (uint8_t*)err, strlen(err), 100);
    }
}

void uart_send(uart_instance_t *inst, const char *str)
{
    HAL_UART_Transmit(inst->huart, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
}

/*
 * uart_rx_callback()
 *
 * Called from HAL_UART_RxCpltCallback (stm32g4xx_it.c) after each received
 * byte. Must re-arm the interrupt before returning.
 */
void uart_rx_callback(uart_instance_t *inst)
{
    char c = (char)inst->rx_byte;

    /* Simple accumulation */
    if (c == '\n' || c == '\r') {
        /* End of line */
        if (inst->rx_index > 0) {
            inst->rx_line[inst->rx_index] = '\0';
            strcpy(inst->line_buf, inst->rx_line);
            inst->rx_index = 0;
            inst->line_ready = 1;
        }
    } else if (inst->rx_index < UART_RX_BUF_SIZE - 1) {
        inst->rx_line[inst->rx_index++] = c;
    }

    HAL_UART_Receive_IT(inst->huart, (uint8_t *)&inst->rx_byte, 1);
}


void uart_process(uart_instance_t *inst)
{
    if (inst->line_ready) {
        inst->line_ready = 0;
        dispatch_command(inst, inst->line_buf);
    }
}
