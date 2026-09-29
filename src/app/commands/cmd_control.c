/*
 * cmd_control.c
 *
 * Closed-loop control: SOURce:*, PID:GAINS, LOG:*.
 * Handlers are declared in commands.h and registered in command_table.c.
 */

#include "commands.h"
#include "cmd_common.h"
#include "ctrlr_config.h"
#include "pid.h"
#include "state_machine.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* --------------------------------------------------------------------------
 * Closed-loop control + demand-output / shot-profile / log / channel
 * namespaces, see pid.h for the full architecture. Channel numbering
 * matches PFMIN:DATA?'s own convention: 1..N on the wire, 0..N-1
 * internally (N = HRTIM_NUM_CHANNELS, CONFig:CHANnels? reports it).
 * Error codes 11 (invalid channel) and 12 (invalid argument count/value)
 * -- see commands.h. RENAMED 2026-09-22 (PID: -> SOURce:/SHOT:/LOG:/
 * CHANnel:), clean cut -- the pid.c API underneath is unchanged.
 * -------------------------------------------------------------------------- */

void cmd_source_run(uart_instance_t *inst, char *args)
{
    (void)args;

    /* GATED 2026-09-21: SOURce:RUN previously bypassed the state machine
       (PID_Start() directly, no ARM/interlock) -- see this project's
       design review. Now mirrors cmd_shot_start()'s own gate (shared
       via RequireArmedAndEnabled(), cmd_common.c): must be ARMED, and the
       external-enable interlock must be satisfied. SM_StartPlain()
       (state_machine.h/.c) is the ARMED -> FIRING transition that
       actually calls PID_Start(); SOURce:STOP (SM_Stop()) returns it
       to IDLE -- this loop has no shot clock, so there is no auto-
       completion path. */
    if (RequireArmedAndEnabled(inst) == 0U)
    {
        return;
    }

    (void)SM_StartPlain();
    uart_send(inst, "OK\r\n");
}

void cmd_source_stop(uart_instance_t *inst, char *args)
{
    (void)args;
    PID_Stop();
    /* SM_Stop() (state_machine.h, added 2026-09-13) -- a manual abort,
       ARMED/FIRING -> IDLE. No-op if already IDLE; deliberately does
       NOT clear FAULT (see its own doc comment -- FAULT:CLEAR is the
       only way out of a real fault latch). */
    SM_Stop();
    uart_send(inst, "OK\r\n");
}

void cmd_source_setpoint(uart_instance_t *inst, char *args)
{
    char *tok;
    long  chArg;
    long  hzArg;
    uint8_t ch;

    tok = (args != NULL) ? strtok(args, " \r\n") : NULL;
    if (tok == NULL)
    {
        SendErr(inst, ERR_INVALID_ARGS, "SOURce:SETpoint needs two arguments: channel hz");
        return;
    }
    chArg = atol(tok);

    tok = strtok(NULL, " \r\n");
    if (tok == NULL)
    {
        SendErr(inst, ERR_INVALID_ARGS, "SOURce:SETpoint needs two arguments: channel hz");
        return;
    }
    hzArg = atol(tok);

    if ((chArg < 1L) || (chArg > (long)HRTIM_NUM_CHANNELS))
    {
        SendErr(inst, ERR_INVALID_CHANNEL, "Invalid channel");
        return;
    }
    ch = (uint8_t)(chArg - 1L);   /* 1..N on the wire -> 0..N-1 internally */

    if (hzArg < 0L)
    {
        SendErr(inst, ERR_INVALID_ARGS, "hz must be >= 0");
        return;
    }

    /* PID_SetSetpoint() clamps into [PID_OUTPUT_MIN_HZ,
       PID_OUTPUT_MAX_HZ] itself -- an out-of-range value here is
       accepted, not rejected, matching this project's existing
       clamp-don't-reject convention for HRTIM1_ClampCompare(). */
    (void)PID_SetSetpoint(ch, (uint32_t)hzArg);
    uart_send(inst, "OK\r\n");
}

void cmd_pid_gains(uart_instance_t *inst, char *args)
{
    char *tok;
    long  chArg;
    double kp;
    double ki;
    double kd;
    uint8_t ch;

    tok = (args != NULL) ? strtok(args, " \r\n") : NULL;
    if (tok == NULL)
    {
        SendErr(inst, ERR_INVALID_ARGS, "PID:GAINS needs four arguments: channel kp ki kd");
        return;
    }
    chArg = atol(tok);

    tok = strtok(NULL, " \r\n");
    if (tok == NULL)
    {
        SendErr(inst, ERR_INVALID_ARGS, "PID:GAINS needs four arguments: channel kp ki kd");
        return;
    }
    kp = atof(tok);

    tok = strtok(NULL, " \r\n");
    if (tok == NULL)
    {
        SendErr(inst, ERR_INVALID_ARGS, "PID:GAINS needs four arguments: channel kp ki kd");
        return;
    }
    ki = atof(tok);

    tok = strtok(NULL, " \r\n");
    if (tok == NULL)
    {
        SendErr(inst, ERR_INVALID_ARGS, "PID:GAINS needs four arguments: channel kp ki kd");
        return;
    }
    kd = atof(tok);

    if ((chArg < 1L) || (chArg > (long)HRTIM_NUM_CHANNELS))
    {
        SendErr(inst, ERR_INVALID_CHANNEL, "Invalid channel");
        return;
    }
    ch = (uint8_t)(chArg - 1L);

    (void)PID_SetGains(ch, (float)kp, (float)ki, (float)kd);
    uart_send(inst, "OK\r\n");
}

/* Added 2026-09-10, alongside python/wham_console.py -- PID:GAINS was
   write-only until now, forcing any host tool to remember what it
   itself last sent rather than being able to ask the device (see
   PID_GetGains()'s own doc comment in pid.h). */
void cmd_pid_gains_query(uart_instance_t *inst, char *args)
{
    char buf[64];
    char *tok;
    long  chArg;
    uint8_t ch;
    float kp;
    float ki;
    float kd;

    tok = (args != NULL) ? strtok(args, " \r\n") : NULL;
    if (tok == NULL)
    {
        SendErr(inst, ERR_INVALID_ARGS, "PID:GAINS? needs one argument: channel");
        return;
    }
    chArg = atol(tok);

    if ((chArg < 1L) || (chArg > (long)HRTIM_NUM_CHANNELS))
    {
        SendErr(inst, ERR_INVALID_CHANNEL, "Invalid channel");
        return;
    }
    ch = (uint8_t)(chArg - 1L);

    (void)PID_GetGains(ch, &kp, &ki, &kd);

    snprintf(buf, sizeof(buf), "OK %g %g %g\r\n", (double)kp, (double)ki, (double)kd);
    uart_send(inst, buf);
}

void cmd_source_status(uart_instance_t *inst, char *args)
{
    char buf[96];
    char *tok;
    long  chArg;
    uint8_t ch;
    uint32_t setpointHz;
    uint32_t measuredHz;
    uint32_t outputHz;

    tok = (args != NULL) ? strtok(args, " \r\n") : NULL;
    if (tok == NULL)
    {
        SendErr(inst, ERR_INVALID_ARGS, "SOURce:STATus? needs one argument: channel");
        return;
    }
    chArg = atol(tok);

    if ((chArg < 1L) || (chArg > (long)HRTIM_NUM_CHANNELS))
    {
        SendErr(inst, ERR_INVALID_CHANNEL, "Invalid channel");
        return;
    }
    ch = (uint8_t)(chArg - 1L);

    (void)PID_GetStatus(ch, &setpointHz, &measuredHz, &outputHz);

    snprintf(buf, sizeof(buf), "OK %u %lu %lu %lu\r\n",
             (unsigned int)PID_IsRunning(),
             (unsigned long)setpointHz, (unsigned long)measuredHz,
             (unsigned long)outputHz);
    uart_send(inst, buf);
}

/* channel argument 0 means "log every channel at once" (PID_ArmLogAll(),
   added 2026-09-10) -- real channels are 1..N on the wire everywhere
   else in this project, leaving 0 a natural, otherwise-unused sentinel
   rather than a whole new command. See PID_ArmLogAll()'s own doc
   comment in pid.h for why this exists (a genuine simultaneous
   cross-channel comparison, not N separate single-channel runs). */
void cmd_log_arm(uart_instance_t *inst, char *args)
{
    char *tok;
    long  chArg;
    long  nArg;
    long  decimArg;

    tok = (args != NULL) ? strtok(args, " \r\n") : NULL;
    if (tok == NULL)
    {
        SendErr(inst, ERR_INVALID_ARGS, "LOG:ARM needs three arguments: channel(0=all) maxSamples decim");
        return;
    }
    chArg = atol(tok);

    tok = strtok(NULL, " \r\n");
    if (tok == NULL)
    {
        SendErr(inst, ERR_INVALID_ARGS, "LOG:ARM needs three arguments: channel(0=all) maxSamples decim");
        return;
    }
    nArg = atol(tok);

    tok = strtok(NULL, " \r\n");
    if (tok == NULL)
    {
        SendErr(inst, ERR_INVALID_ARGS, "LOG:ARM needs three arguments: channel(0=all) maxSamples decim");
        return;
    }
    decimArg = atol(tok);

    if ((chArg < 0L) || (chArg > (long)HRTIM_NUM_CHANNELS))
    {
        SendErr(inst, ERR_INVALID_CHANNEL, "Invalid channel (0 = all channels)");
        return;
    }

    if ((nArg < 1L) || (nArg > (long)PID_LOG_MAX_SAMPLES) || (decimArg < 1L))
    {
        SendErr(inst, ERR_INVALID_ARGS, "maxSamples must be 1-PID_LOG_MAX_SAMPLES, decim >= 1");
        return;
    }

    if (chArg == 0L)
    {
        (void)PID_ArmLogAll((uint16_t)nArg, (uint16_t)decimArg);
    }
    else
    {
        (void)PID_ArmLog((uint8_t)(chArg - 1L), (uint16_t)nArg, (uint16_t)decimArg);
    }
    uart_send(inst, "OK\r\n");
}

/* STREAMED, not built into one giant buffer -- with 3 values/sample
   (setpointHz, measuredHz, outputHz, added 2026-09-10 alongside
   PID_StartRamp()) at up to PID_LOG_MAX_SAMPLES (1000), a single
   contiguous reply buffer would need ~40 KB (worst case, all 10-digit
   values) -- real RAM this project doesn't have to spare (this MCU
   has 128 KiB total, most of it already spoken for by g_pfmTable[]
   and friends). uart_send() is just a blocking HAL_UART_Transmit() (see
   uart.c) with no minimum-call-size requirement, so there's no
   correctness reason to batch into one large string either -- this
   sends the header, then one small chunk per sample, reusing one
   small stack buffer regardless of how many samples are logged. Costs
   more individual transmit calls (~1000 for a full log) instead of
   one, but at 115200 baud the whole reply takes over a second to
   clock out either way -- the call overhead is noise against that.

   OPTIONAL channel argument, added 2026-09-10 alongside PID_ArmLogAll():
   `LOG:DATA?` (no argument) is UNCHANGED, exactly its original
   behavior -- valid only when a single channel is armed (PID_ArmLog()),
   fetches that channel's data, ERR 12 if all-channels mode is active
   (ambiguous without a channel to pick). `LOG:DATA? <ch>` works in
   EITHER mode: under all-channels mode any ch is valid; under
   single-channel mode ch must equal the one actually armed (ERR 12
   otherwise -- no data exists for any other channel this run). */
void cmd_log_data(uart_instance_t *inst, char *args)
{
    char *tok;
    long  chArg;
    uint8_t ch;
    uint16_t count;
    const uint32_t *setpoint;
    const uint32_t *measured;
    const uint32_t *output;
    char chunk[48];

    tok = (args != NULL) ? strtok(args, " \r\n") : NULL;
    if (tok == NULL)
    {
        if (PID_IsLogAllChannels() != 0U)
        {
            SendErr(inst, ERR_INVALID_ARGS, "LOG:DATA? needs a channel argument while "
                               "all-channels logging is armed");
            return;
        }
        ch = PID_GetLogChannel();   /* 0xFF (no channel armed) is caught below */
    }
    else
    {
        chArg = atol(tok);
        if ((chArg < 1L) || (chArg > (long)HRTIM_NUM_CHANNELS))
        {
            SendErr(inst, ERR_INVALID_CHANNEL, "Invalid channel");
            return;
        }
        ch = (uint8_t)(chArg - 1L);
        if ((PID_IsLogAllChannels() == 0U) && (ch != PID_GetLogChannel()))
        {
            SendErr(inst, ERR_INVALID_ARGS, "That channel isn't the one currently armed -- "
                               "see LOG:ARM");
            return;
        }
    }

    setpoint = PID_GetLogSetpoint(ch);
    measured = PID_GetLogMeasured(ch);
    output   = PID_GetLogOutput(ch);
    if ((setpoint == NULL) || (measured == NULL) || (output == NULL))
    {
        SendErr(inst, ERR_INVALID_CHANNEL, "Invalid channel");
        return;
    }

    count = PID_GetLogCount();
    snprintf(chunk, sizeof(chunk), "OK %u %lu", (unsigned int)count,
             (unsigned long)PID_GetLogSampleRateHz());
    uart_send(inst, chunk);

    for (uint16_t i = 0U; i < count; i++)
    {
        snprintf(chunk, sizeof(chunk), " %lu %lu %lu",
                 (unsigned long)setpoint[i], (unsigned long)measured[i],
                 (unsigned long)output[i]);
        uart_send(inst, chunk);
    }

    uart_send(inst, "\r\n");
}

void cmd_source_ramp(uart_instance_t *inst, char *args)
{
    char *tok;
    long  chArg;
    long  startArg;
    long  endArg;
    long  durationArg;
    uint8_t ch;

    tok = (args != NULL) ? strtok(args, " \r\n") : NULL;
    if (tok == NULL)
    {
        SendErr(inst, ERR_INVALID_ARGS, "SOURce:RAMP needs four arguments: channel startHz endHz durationMs");
        return;
    }
    chArg = atol(tok);

    tok = strtok(NULL, " \r\n");
    if (tok == NULL)
    {
        SendErr(inst, ERR_INVALID_ARGS, "SOURce:RAMP needs four arguments: channel startHz endHz durationMs");
        return;
    }
    startArg = atol(tok);

    tok = strtok(NULL, " \r\n");
    if (tok == NULL)
    {
        SendErr(inst, ERR_INVALID_ARGS, "SOURce:RAMP needs four arguments: channel startHz endHz durationMs");
        return;
    }
    endArg = atol(tok);

    tok = strtok(NULL, " \r\n");
    if (tok == NULL)
    {
        SendErr(inst, ERR_INVALID_ARGS, "SOURce:RAMP needs four arguments: channel startHz endHz durationMs");
        return;
    }
    durationArg = atol(tok);

    if ((chArg < 1L) || (chArg > (long)HRTIM_NUM_CHANNELS))
    {
        SendErr(inst, ERR_INVALID_CHANNEL, "Invalid channel");
        return;
    }
    ch = (uint8_t)(chArg - 1L);

    if ((startArg < 0L) || (endArg < 0L) || (durationArg < 1L))
    {
        SendErr(inst, ERR_INVALID_ARGS, "startHz/endHz must be >= 0, durationMs >= 1");
        return;
    }

    /* PID_StartRamp() clamps startHz/endHz into [PID_OUTPUT_MIN_HZ,
       PID_OUTPUT_MAX_HZ] itself -- matches SOURce:SETpoint's own
       clamp-don't-reject convention. */
    (void)PID_StartRamp(ch, (uint32_t)startArg, (uint32_t)endArg, (uint32_t)durationArg);
    uart_send(inst, "OK\r\n");
}
