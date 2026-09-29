/*
 * app.h -- application start-up and main loop, called from main.c.
 *
 * main() call order (each in its CubeMX USER CODE block):
 *   BootJump_CheckAndEnter(), BootDiag_Begin()    USER CODE 1, before HAL_Init()
 *   Mcu_SetSysTickHighestPriority()               USER CODE Init, after HAL_Init()
 *   BoardIo_Init()                                inside MX_GPIO_Init()
 *   uart_bind(&uart2, &huart2)                    USER CODE 2 (BSP, uart_hw.h)
 *   App_Init(), App_SendBootBanner()              USER CODE 2
 *   App_Poll()                                    every main-loop iteration
 */
#ifndef APP_H
#define APP_H

/* Initializes every application module and starts the command link. */
void App_Init(void);

/* Sends the unsolicited "!BOOT" lines -- once, just before the main loop. */
void App_SendBootBanner(void);

/* One main-loop iteration: runs each task in tasks.h once. */
void App_Poll(void);

#endif /* APP_H */
