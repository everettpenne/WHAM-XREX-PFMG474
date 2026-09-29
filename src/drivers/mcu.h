/*
 * mcu.h -- core MCU services for code outside the BSP: time, interrupt
 * masking, the cycle counter, and the few chip facts the diagnostics
 * report. Implemented in src/bsp/<chip>/mcu.c.
 */
#ifndef MCU_H
#define MCU_H

#include <stdint.h>

/* Milliseconds since boot (the HAL tick, SysTick-driven). */
uint32_t Mcu_GetTickMs(void);

/* Busy-wait. Needs SysTick running. */
void Mcu_DelayMs(uint32_t ms);

/* Interrupt masking. Mcu_IrqSave()/Mcu_IrqRestore() nest safely (restore the
   previous PRIMASK); Mcu_IrqDisable()/Mcu_IrqEnable() are the plain
   unconditional pair, for code that is never entered with interrupts
   already masked. */
uint32_t Mcu_IrqSave(void);
void     Mcu_IrqRestore(uint32_t saved);
void     Mcu_IrqDisable(void);
void     Mcu_IrqEnable(void);

/* Free-running CPU cycle counter (170 MHz on this board). Start once;
   Mcu_CycleCount() wraps, so take differences as uint32_t. */
void     Mcu_CycleCounterStart(void);
uint32_t Mcu_CycleCount(void);

/* Raises SysTick to the highest interrupt priority. Call right after
   HAL_Init() (main.c) -- see mcu.c for why. */
void Mcu_SetSysTickHighestPriority(void);

/* Boot-related option bits (DIAGnostic:OPTBytes?). */
typedef struct
{
    uint32_t raw;          /* the whole option register */
    uint8_t  nBoot0;
    uint8_t  nSwBoot0;
    uint8_t  nBoot1;
} mcu_option_bytes_t;
void Mcu_ReadOptionBytes(mcu_option_bytes_t *ob);

/* Reset-cause flags since they were last cleared (DIAGnostic:RSTCause?). */
typedef struct
{
    uint32_t raw;          /* the whole reset-status register */
    uint8_t  bor, pin, sft, iwdg, wwdg, lpwr, obl;
} mcu_reset_flags_t;
void Mcu_ReadResetFlags(mcu_reset_flags_t *f);
void Mcu_ClearResetFlags(void);

#endif /* MCU_H */
