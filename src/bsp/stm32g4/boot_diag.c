/*
 * boot_diag.c -- reset-surviving boot diagnostics, see boot_diag.h.
 */
#include "boot_diag.h"
#include "main.h"
#include <stdio.h>

/* .noinit, so the startup code neither copies nor zeroes it: it holds the
   previous boot's record until BootDiag_Begin() snapshots it and starts
   this boot's own. */
__attribute__((section(".noinit"))) volatile BootDiag_t g_bootDiag;
static BootDiag_t s_prevBoot;      /* previous boot's record, for the banner */
static uint32_t   s_resetCsr;      /* RCC->CSR as found at this boot */

/* Snapshot the previous boot's record, then start this one. Invalid magic
   = first boot since power-up (RAM content is random) -- start from zero.
   Reset-cause flags are captured and cleared here, so each boot's banner
   shows only what reset it. Called from main() straight after
   BootJump_CheckAndEnter(), before HAL_Init(). */
void BootDiag_Begin(void)
{
    uint32_t i;
    if (g_bootDiag.magic != BOOT_DIAG_MAGIC)
    {
        volatile uint32_t *w = (volatile uint32_t *)&g_bootDiag;
        for (i = 0U; i < (sizeof(BootDiag_t) / sizeof(uint32_t)); i++)
        {
            w[i] = 0U;
        }
        g_bootDiag.magic = BOOT_DIAG_MAGIC;
    }
    s_prevBoot.bootCount  = g_bootDiag.bootCount;
    s_prevBoot.lastStage  = g_bootDiag.lastStage;
    s_prevBoot.faultCount = g_bootDiag.faultCount;
    s_prevBoot.faultStage = g_bootDiag.faultStage;
    s_prevBoot.faultCfsr  = g_bootDiag.faultCfsr;
    s_prevBoot.faultHfsr  = g_bootDiag.faultHfsr;
    s_prevBoot.errCount   = g_bootDiag.errCount;
    s_prevBoot.errStage   = g_bootDiag.errStage;
    s_prevBoot.nmiCount   = g_bootDiag.nmiCount;
    s_prevBoot.nmiStage   = g_bootDiag.nmiStage;
    s_prevBoot.nmiEccr    = g_bootDiag.nmiEccr;

    g_bootDiag.bootCount++;
    g_bootDiag.nmiCount   = 0U;
    g_bootDiag.nmiStage   = 0U;
    g_bootDiag.nmiEccr    = 0U;
    g_bootDiag.faultCount = 0U;
    g_bootDiag.faultStage = 0U;
    g_bootDiag.faultCfsr  = 0U;
    g_bootDiag.faultHfsr  = 0U;
    g_bootDiag.errCount   = 0U;
    g_bootDiag.errStage   = 0U;
    BOOT_DIAG_STAGE(BD_STAGE_MAIN);

    s_resetCsr = RCC->CSR;
    RCC->CSR |= RCC_CSR_RMVF;
}

/* Format the "!BOOT diag" line: this boot's count plus the previous
   boot's record. "prev" is the boot BEFORE this one; rst= is what reset
   the chip since that boot cleared the flags (OBL option-byte reload, PIN
   NRST, BOR brown-out/power-on, SFT software, IWDG/WWDG watchdog, LPWR
   low-power). */
void BootDiag_FormatReport(char *buf, size_t len)
{
    snprintf(buf, len,
             "!BOOT diag boot=%lu prev: stage=%lu fault=%lu@%lu cfsr=%08lX hfsr=%08lX err=%lu@%lu"
             " nmi=%lu@%lu eccr=%08lX rst=%s%s%s%s%s%s%s\r\n",
             (unsigned long)g_bootDiag.bootCount,
             (unsigned long)s_prevBoot.lastStage,
             (unsigned long)s_prevBoot.faultCount, (unsigned long)s_prevBoot.faultStage,
             (unsigned long)s_prevBoot.faultCfsr, (unsigned long)s_prevBoot.faultHfsr,
             (unsigned long)s_prevBoot.errCount, (unsigned long)s_prevBoot.errStage,
             (unsigned long)s_prevBoot.nmiCount, (unsigned long)s_prevBoot.nmiStage,
             (unsigned long)s_prevBoot.nmiEccr,
             ((s_resetCsr & RCC_CSR_OBLRSTF)  != 0U) ? "OBL,"  : "",
             ((s_resetCsr & RCC_CSR_PINRSTF)  != 0U) ? "PIN,"  : "",
             ((s_resetCsr & RCC_CSR_BORRSTF)  != 0U) ? "BOR,"  : "",
             ((s_resetCsr & RCC_CSR_SFTRSTF)  != 0U) ? "SFT,"  : "",
             ((s_resetCsr & RCC_CSR_IWDGRSTF) != 0U) ? "IWDG," : "",
             ((s_resetCsr & RCC_CSR_WWDGRSTF) != 0U) ? "WWDG," : "",
             ((s_resetCsr & RCC_CSR_LPWRRSTF) != 0U) ? "LPWR," : "");
}
