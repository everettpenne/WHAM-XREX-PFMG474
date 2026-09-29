/*
 * boot_diag.h -- reset-surviving boot diagnostics, added 2026-09-25.
 *
 * Investigates an intermittent post-FWUPdate:SWAP hang (board silent until
 * something resets it again). Lives in .noinit RAM, so it survives every
 * reset (NRST, option-byte reload, software, fault) but not a power cycle.
 * Each boot reports the PREVIOUS boot's record in its !BOOT banner:
 *
 *   - bootCount: +1 per boot that reaches main(). If the recovery boot shows
 *     a jump of 1 since the boot that issued SWAP, no firmware ran in between
 *     (held in reset, or sitting in the ROM bootloader). A jump of 2 means a
 *     boot ran and got stuck.
 *   - lastStage: how far the previous boot got (BD_STAGE_*).
 *   - faultCount/errCount/nmiCount + CFSR/HFSR/ECCR + stage at the event:
 *     whether that boot died in HardFault_Handler, Error_Handler or
 *     NMI_Handler, all of which spin forever.
 *
 * RCC->CSR (reset-cause flags) is also captured and cleared at every boot,
 * so each banner shows only the reset sources since the previous boot.
 */
#ifndef BOOT_DIAG_H
#define BOOT_DIAG_H

#include <stddef.h>
#include <stdint.h>

#define BOOT_DIAG_MAGIC         0xB00DD1A6UL

#define BD_STAGE_MAIN           1U   /* main() entered, past BootJump_CheckAndEnter() */
#define BD_STAGE_HAL_INIT       2U
#define BD_STAGE_CLOCK          3U   /* SystemClock_Config() returned */
#define BD_STAGE_GPIO           4U
#define BD_STAGE_HRTIM          5U
#define BD_STAGE_USART          6U
#define BD_STAGE_APP_INIT       7U   /* App_Init() done (main.c USER CODE 2) */
#define BD_STAGE_MAIN_LOOP      8U   /* banner sent, entering while(1) */
#define BD_STAGE_OB_LAUNCH      9U   /* FWUPdate:SWAP/ROLLback about to reload option bytes */

typedef struct
{
    uint32_t magic;
    uint32_t bootCount;
    uint32_t lastStage;
    uint32_t faultCount;
    uint32_t faultStage;
    uint32_t faultCfsr;
    uint32_t faultHfsr;
    uint32_t errCount;
    uint32_t errStage;
    uint32_t nmiCount;       /* NMI: on this chip, e.g. a flash ECC double error */
    uint32_t nmiStage;
    uint32_t nmiEccr;        /* FLASH->ECCR at the NMI */
} BootDiag_t;

extern volatile BootDiag_t g_bootDiag;

#define BOOT_DIAG_STAGE(s)  do { g_bootDiag.lastStage = (s); } while (0)

/* Called once from main(), straight after BootJump_CheckAndEnter(): keeps
   the previous boot's record for the banner and starts this boot's. */
void BootDiag_Begin(void);

/* Formats the "!BOOT diag ..." banner line (CRLF-terminated) into buf. */
void BootDiag_FormatReport(char *buf, size_t len);

#endif /* BOOT_DIAG_H */
