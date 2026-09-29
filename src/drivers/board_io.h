/*
 * board_io.h -- this board's discrete digital signals, by name. The pin
 * behind each name (docs/pin_mapping_v4.csv) and its electrical setup live
 * in src/bsp/<chip>/board_io.c; nothing outside the BSP needs a port or pin
 * number.
 *
 * Per-channel families are consecutive, so channel ch (0-based) of a family
 * is e.g. BOARD_SIG_XR_OCP(ch). Channel 0..3 = Transrex XR1..XR4.
 */
#ifndef BOARD_IO_H
#define BOARD_IO_H

#include <stdint.h>

#define BOARD_XR_CHANNELS   4U   /* Transrex channels wired on this board */

typedef enum
{
    /* Inputs */
    BOARD_SIG_EXT_ENABLE,        /* PF13 GPInput_12: external-enable interlock (pull-down) */
    BOARD_SIG_EXT_TRIGGER,       /* PF15 Fiber_Enable: external trigger (pull-down) */
    BOARD_SIG_XR1_OCP,           /* PF4  (pull-down) */
    BOARD_SIG_XR2_OCP,           /* PF8 */
    BOARD_SIG_XR3_OCP,           /* PF12 */
    BOARD_SIG_XR4_OCP,           /* PF5 */

    /* Outputs (reading one returns the pin's actual level) */
    BOARD_SIG_XR1_ENA_OUT,       /* PG0..PG3 */
    BOARD_SIG_XR2_ENA_OUT,
    BOARD_SIG_XR3_ENA_OUT,
    BOARD_SIG_XR4_ENA_OUT,
    BOARD_SIG_XR1_CONTACT_OUT,   /* PG4..PG7 */
    BOARD_SIG_XR2_CONTACT_OUT,
    BOARD_SIG_XR3_CONTACT_OUT,
    BOARD_SIG_XR4_CONTACT_OUT,
    BOARD_SIG_GPOUT_09,          /* PG8 */
    BOARD_SIG_GPOUT_10,          /* PG9 */
    BOARD_SIG_GPOUT_11,          /* PD0 */
    BOARD_SIG_GPOUT_12,          /* PD1 */
    BOARD_SIG_GPOUT_ENABLE,      /* PC13 GPOut_Enable_Pin */
    BOARD_SIG_PWMALT_ENABLE,     /* PC15 PWM_Alt_Enable */

    BOARD_SIG_COUNT
} board_signal_t;

#define BOARD_SIG_XR_OCP(ch)          ((board_signal_t)(BOARD_SIG_XR1_OCP + (ch)))
#define BOARD_SIG_XR_ENA_OUT(ch)      ((board_signal_t)(BOARD_SIG_XR1_ENA_OUT + (ch)))
#define BOARD_SIG_XR_CONTACT_OUT(ch)  ((board_signal_t)(BOARD_SIG_XR1_CONTACT_OUT + (ch)))

/* Configures every board pin CubeMX's MX_GPIO_Init() doesn't: the
   GateDriverStatus fault inputs (EXTI), the signals above. Outputs are
   driven to their default level before being enabled, so none glitches at
   boot. Called from MX_GPIO_Init()'s USER CODE block. */
void BoardIo_Init(void);

/* 1 = pin HIGH, 0 = LOW. Out-of-range signal reads 0. */
uint8_t BoardIo_Read(board_signal_t sig);

/* Drive an output HIGH (high != 0) or LOW. No-op for inputs/out of range. */
void BoardIo_Write(board_signal_t sig, uint8_t high);

/* The 12 GateDriverStatus inputs (PE0..PE11) as one word, bit n = pin PEn:
   XR1-4 Water (bits 0-3), Temp (4-7), Enerpro (8-11). Raw levels, no
   polarity applied. */
uint16_t BoardIo_ReadGateDriverStatus(void);

#endif /* BOARD_IO_H */
