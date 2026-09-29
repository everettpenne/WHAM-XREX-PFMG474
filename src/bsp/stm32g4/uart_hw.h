/*
 * uart_hw.h -- BSP-private part of the serial link: binding an instance to
 * its HAL handle, and the receive-complete hook the HAL callback calls
 * (stm32g4xx_it.c). Public interface: src/drivers/uart.h.
 */
#ifndef UART_HW_H
#define UART_HW_H

#include "uart.h"
#include "main.h"

/* Attach an instance to its CubeMX-initialized UART handle. No hardware
   effect -- reception starts at uart_start(). */
void uart_bind(uart_instance_t *inst, UART_HandleTypeDef *huart);

/* Called from HAL_UART_RxCpltCallback() after each received byte. */
void uart_rx_callback(uart_instance_t *inst);

#endif /* UART_HW_H */
