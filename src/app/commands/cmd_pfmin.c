/*
 * cmd_pfmin.c
 *
 * PFMIN:* -- PFM_Input period capture and its debug queries.
 * Handlers are declared in commands.h and registered in command_table.c.
 */

#include "commands.h"
#include "cmd_common.h"
#include "pfm_input.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if (PFM_INPUT_FEATURE_ENABLED != 0)
/* --------------------------------------------------------------------------
 * PFMIN:CAPTURE, PFMIN:STATus?, PFMIN:DATA?
 *
 * See pfm_input.h and commands.h's own header comment above these
 * declarations for the shot-synchronized design -- PFMIN:CAPTURE only
 * arms (PfmInput_Arm()); the actual capture starts inside
 * PFM_Restart() (pfm.c), at the next FIRE, not here.
 * -------------------------------------------------------------------------- */
void cmd_pfmin_capture(uart_instance_t *inst, char *args)
{
    char *tok;
    long  m;

    tok = (args != NULL) ? strtok(args, " \r\n") : NULL;
    if (tok == NULL)
    {
        SendErr(inst, ERR_PFMIN_BAD_COUNT, "PFMIN:CAPTURE needs one argument: M (1-PFM_INPUT_MAX_PERIODS)");
        return;
    }

    m = atol(tok);
    if ((m < 1L) || (m > (long)PFM_INPUT_MAX_PERIODS))
    {
        SendErr(inst, ERR_PFMIN_BAD_COUNT, "M out of range (1-PFM_INPUT_MAX_PERIODS)");
        return;
    }

    PfmInput_Arm((uint16_t)m);

    uart_send(inst, "OK\r\n");
}

void cmd_pfmin_status(uart_instance_t *inst, char *args)
{
    char buf[64];
    int  len = 0;
    (void)args;

    len += snprintf(&buf[len], sizeof(buf) - (size_t)len, "OK");
    for (uint8_t ch = 0U; ch < PFM_INPUT_NUM_CHANNELS; ch++)
    {
        len += snprintf(&buf[len], sizeof(buf) - (size_t)len,
                         " %u", (unsigned int)PfmInput_GetCount(ch));
    }
    snprintf(&buf[len], sizeof(buf) - (size_t)len, "\r\n");

    uart_send(inst, buf);
}

/* TEMPORARY debug command, 2026-09-09 -- see pfm_input.h's own comment
   on PfmInput_GetDmaStartStatus(). Remove once DMA capture is
   confirmed reliable. */
void cmd_pfmin_dmastat(uart_instance_t *inst, char *args)
{
    char buf[64];
    int  len = 0;
    (void)args;

    len += snprintf(&buf[len], sizeof(buf) - (size_t)len, "OK");
    for (uint8_t ch = 0U; ch < PFM_INPUT_NUM_CHANNELS; ch++)
    {
        len += snprintf(&buf[len], sizeof(buf) - (size_t)len,
                         " %u", (unsigned int)PfmInput_GetDmaStartStatus(ch));
    }
    snprintf(&buf[len], sizeof(buf) - (size_t)len, "\r\n");

    uart_send(inst, buf);
}

/* --------------------------------------------------------------------------
 * PFMIN:DEBUG:RAW? <ch>
 *
 * TEMPORARY debug command, added 2026-09-15 -- same removability
 * precedent as PFMIN:DMASTAT? just above: diagnosing why SOURce:STATus?'s
 * measuredHz reads persistently 0 for WHAM channels 2/3/4 while
 * channel 1 works, confirmed on real hardware (each channel isolated
 * alone, driving confirmed-correct real HRTIM output over a full
 * shot). Non-destructive read of PfmInput_GetDebugRaw() (pfm_input.h)
 * -- unlike SOURce:STATus?'s own PfmInput_ConsumeAveragePeriod() call,
 * does NOT reset the accumulator, so repeated polling can watch
 * avgCount accumulate (or not) live during a shot without disturbing
 * the real control loop's own consumption. `ch` is 1-based (WHAM
 * channel numbering, matching SOURce:STATus?), internally
 * channel = ch - 1 (PFM_Input_01..04, the first 4 of the 6 physical
 * PFM_Input channels -- see pid.c's own per-channel loop). Remove once
 * the root cause is found and fixed. */
void cmd_pfmin_debug_raw(uart_instance_t *inst, char *args)
{
    char *tok;
    long  chArg;

    tok = (args != NULL) ? strtok(args, " \r\n") : NULL;
    if (tok == NULL)
    {
        SendErr(inst, ERR_INVALID_ARGS, "PFMIN:DEBUG:RAW? needs one argument: channel");
        return;
    }
    chArg = atol(tok);

    if ((chArg < 1L) || (chArg > (long)PFM_INPUT_NUM_CHANNELS))
    {
        SendErr(inst, ERR_PFMIN_BAD_CHANNEL, "Invalid PFM_Input channel (1-6)");
        return;
    }

    uint8_t  continuous = 0U, running = 0U, haveFirstRise = 0U;
    uint16_t avgCount = 0U, overcaptureCount = 0U;
    uint32_t lastPeriod = 0U;
    (void)PfmInput_GetDebugRaw((uint8_t)(chArg - 1L), &continuous, &running,
                                &haveFirstRise, &avgCount, &lastPeriod, &overcaptureCount);

    char buf[96];
    snprintf(buf, sizeof(buf), "OK cont=%u run=%u firstRise=%u avgCount=%u lastPeriod=%lu overcap=%u\r\n",
             (unsigned int)continuous, (unsigned int)running, (unsigned int)haveFirstRise,
             (unsigned int)avgCount, (unsigned long)lastPeriod, (unsigned int)overcaptureCount);
    uart_send(inst, buf);
}

/* --------------------------------------------------------------------------
 * PFMIN:DEBUG:REG? <ch>
 *
 * TEMPORARY debug command, added 2026-09-15 -- see
 * PfmInput_GetDebugRegs()'s own doc comment (pfm_input.h) for the full
 * reasoning: raw TIMx register readback, bypassing all software
 * layers, while diagnosing why measuredHz reads 0 for WHAM channels
 * 2/3 despite confirmed-correct real output AND confirmed-correct
 * physical wiring. Poll twice in a row during an active shot and
 * compare CNT/CCR -- if they're not moving, the hardware itself isn't
 * seeing this channel's signal; if they ARE moving but software never
 * reports it, the bug is downstream in the DMA/ISR path instead.
 * Remove once the root cause is found and fixed. */
void cmd_pfmin_debug_reg(uart_instance_t *inst, char *args)
{
    char *tok;
    long  chArg;

    tok = (args != NULL) ? strtok(args, " \r\n") : NULL;
    if (tok == NULL)
    {
        SendErr(inst, ERR_INVALID_ARGS, "PFMIN:DEBUG:REG? needs one argument: channel");
        return;
    }
    chArg = atol(tok);

    if ((chArg < 1L) || (chArg > (long)PFM_INPUT_NUM_CHANNELS))
    {
        SendErr(inst, ERR_PFMIN_BAD_CHANNEL, "Invalid PFM_Input channel (1-6)");
        return;
    }

    uint32_t cr1 = 0U, ccer = 0U, dier = 0U, sr = 0U, cnt = 0U, ccrChannel = 0U;
    (void)PfmInput_GetDebugRegs((uint8_t)(chArg - 1L), &cr1, &ccer, &dier, &sr, &cnt, &ccrChannel);
    uint32_t ccmr1 = PfmInput_GetDebugCcmr1((uint8_t)(chArg - 1L));
    uint32_t moder = 0U, afr = 0U, idr = 0U;
    (void)PfmInput_GetDebugGpio((uint8_t)(chArg - 1L), &moder, &afr, &idr);

    char buf[176];
    snprintf(buf, sizeof(buf),
             "OK CR1=%08lX CCER=%08lX DIER=%08lX SR=%08lX CNT=%08lX CCR=%08lX "
             "CCMR1=%08lX MODER=%lu AFR=%lu IDR=%lu\r\n",
             (unsigned long)cr1, (unsigned long)ccer, (unsigned long)dier,
             (unsigned long)sr, (unsigned long)cnt, (unsigned long)ccrChannel, (unsigned long)ccmr1,
             (unsigned long)moder, (unsigned long)afr, (unsigned long)idr);
    uart_send(inst, buf);
}

/* Sized for the worst case: PFM_INPUT_MAX_PERIODS entries, each up to
   " 4294967295" (11 chars), plus the "OK <count> OVERCAP=<n>" prefix
   and CRLF -- comfortably over 200*11+32 with room to spare. static,
   not a stack local: this project's linker script reserves only
   _Min_Stack_Size (1 KB) for the stack (STM32G474QETX_FLASH.ld) --
   putting several KB on the stack here would be a real, needless risk
   even though the actual runtime stack has far more room in practice
   (see AGENTS.md's own flagged-but-deferred note on stack sizing). */
static char g_pfminDataBuf[2600];

void cmd_pfmin_data(uart_instance_t *inst, char *args)
{
    char    *tok;
    long     chArg;
    uint8_t  ch;
    uint16_t count;
    const uint32_t *periods;
    int      len = 0;

    tok = (args != NULL) ? strtok(args, " \r\n") : NULL;
    if (tok == NULL)
    {
        SendErr(inst, ERR_PFMIN_BAD_CHANNEL, "PFMIN:DATA? needs one argument: channel (1-6)");
        return;
    }

    chArg = atol(tok);
    if ((chArg < 1L) || (chArg > (long)PFM_INPUT_NUM_CHANNELS))
    {
        SendErr(inst, ERR_PFMIN_BAD_CHANNEL, "Invalid PFM_Input channel (1-6)");
        return;
    }
    ch = (uint8_t)(chArg - 1L);   /* 1-6 on the wire -> 0-5 internally */

    count   = PfmInput_GetCount(ch);
    periods = PfmInput_GetPeriods(ch);

    /* OVERCAP=<n>: a genuine data-quality indicator (see
       PfmInput_GetOvercaptureCount()'s own doc comment in
       pfm_input.c) -- nonzero means a second rising edge arrived
       before this module's ISR could read the previous one, so some
       entries below may not be trustworthy. Expected to read 0 in
       normal operation; not yet documented in
       docs/command_reference.md. */
    len += snprintf(&g_pfminDataBuf[len], sizeof(g_pfminDataBuf) - (size_t)len,
                     "OK %u OVERCAP=%u", (unsigned int)count,
                     (unsigned int)PfmInput_GetOvercaptureCount(ch));

    for (uint16_t i = 0U; i < count; i++)
    {
        len += snprintf(&g_pfminDataBuf[len], sizeof(g_pfminDataBuf) - (size_t)len,
                         " %lu", (unsigned long)periods[i]);
    }
    snprintf(&g_pfminDataBuf[len], sizeof(g_pfminDataBuf) - (size_t)len, "\r\n");

    uart_send(inst, g_pfminDataBuf);
}
#endif /* PFM_INPUT_FEATURE_ENABLED */
