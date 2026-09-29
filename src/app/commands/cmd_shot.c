/*
 * cmd_shot.c
 *
 * Production shot profile: PID:LOOPMODE, SOURce:ENAble,
 * CHANnel:NICKname, SHOT:TIMing/CURRent/STARt.
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
 * Production shot profile + open/closed-loop mode -- added 2026-09-10.
 * See pid.h's "DEMAND PROFILE"/"OPEN-LOOP MODE" doc sections.
 * -------------------------------------------------------------------------- */

/* `ch` = 0 means "every channel at once" -- added 2026-09-16, per
   direct request for a system-wide loop-mode convenience (originally
   asked for in service of the external-trigger feature, but not
   restricted to that use -- a plain global setter). Matches
   LOG:ARM's own existing "0 = all channels" convention (pid.h's own
   PfmInput_ArmLogAll() precedent) rather than inventing a new command
   -- reuses this exact command/argument slot instead. Real channels
   are still 1..HRTIM_NUM_CHANNELS as always. */
void cmd_pid_loopmode(uart_instance_t *inst, char *args)
{
    char *tok;
    long  chArg;
    long  modeArg;
    uint8_t ch;
    uint8_t mode;

    tok = (args != NULL) ? strtok(args, " \r\n") : NULL;
    if (tok == NULL)
    {
        SendErr(inst, ERR_INVALID_ARGS, "PID:LOOPMODE needs two arguments: channel(0=all) 0|1");
        return;
    }
    chArg = atol(tok);

    tok = strtok(NULL, " \r\n");
    if (tok == NULL)
    {
        SendErr(inst, ERR_INVALID_ARGS, "PID:LOOPMODE needs two arguments: channel(0=all) 0|1");
        return;
    }
    modeArg = atol(tok);

    if ((chArg < 0L) || (chArg > (long)HRTIM_NUM_CHANNELS))
    {
        SendErr(inst, ERR_INVALID_CHANNEL, "Invalid channel");
        return;
    }
    mode = (modeArg != 0L) ? 1U : 0U;

    if (chArg == 0L)
    {
        for (ch = 0U; ch < HRTIM_NUM_CHANNELS; ch++)
        {
            (void)PID_SetLoopMode(ch, mode);
        }
    }
    else
    {
        ch = (uint8_t)(chArg - 1L);
        (void)PID_SetLoopMode(ch, mode);
    }
    uart_send(inst, "OK\r\n");
}

/* Added 2026-09-10 -- see cmd_pid_gains_query()'s own comment on why
   (PID:LOOPMODE was write-only until now). */
void cmd_pid_loopmode_query(uart_instance_t *inst, char *args)
{
    char buf[32];
    char *tok;
    long  chArg;
    uint8_t ch;

    tok = (args != NULL) ? strtok(args, " \r\n") : NULL;
    if (tok == NULL)
    {
        SendErr(inst, ERR_INVALID_ARGS, "PID:LOOPMODE? needs one argument: channel");
        return;
    }
    chArg = atol(tok);

    if ((chArg < 1L) || (chArg > (long)HRTIM_NUM_CHANNELS))
    {
        SendErr(inst, ERR_INVALID_CHANNEL, "Invalid channel");
        return;
    }
    ch = (uint8_t)(chArg - 1L);

    snprintf(buf, sizeof(buf), "OK %u\r\n", (unsigned int)PID_GetLoopMode(ch));
    uart_send(inst, buf);
}

/* Added 2026-09-11, per direct request -- see PID_SetChannelEnable()'s
   own doc comment in pid.h for exactly what this does (a genuine "no
   PFM waveform at all" switch, distinct from PID:LOOPMODE). Takes
   effect immediately if the loop is already running -- see that
   function's own comment. */
void cmd_source_enable(uart_instance_t *inst, char *args)
{
    char *tok;
    long  chArg;
    long  enableArg;
    uint8_t ch;

    tok = (args != NULL) ? strtok(args, " \r\n") : NULL;
    if (tok == NULL)
    {
        SendErr(inst, ERR_INVALID_ARGS, "SOURce:ENAble needs two arguments: channel 0|1");
        return;
    }
    chArg = atol(tok);

    tok = strtok(NULL, " \r\n");
    if (tok == NULL)
    {
        SendErr(inst, ERR_INVALID_ARGS, "SOURce:ENAble needs two arguments: channel 0|1");
        return;
    }
    enableArg = atol(tok);

    if ((chArg < 1L) || (chArg > (long)HRTIM_NUM_CHANNELS))
    {
        SendErr(inst, ERR_INVALID_CHANNEL, "Invalid channel");
        return;
    }
    ch = (uint8_t)(chArg - 1L);

    (void)PID_SetChannelEnable(ch, (enableArg != 0L) ? 1U : 0U);
    uart_send(inst, "OK\r\n");
}

void cmd_source_enable_query(uart_instance_t *inst, char *args)
{
    char buf[32];
    char *tok;
    long  chArg;
    uint8_t ch;

    tok = (args != NULL) ? strtok(args, " \r\n") : NULL;
    if (tok == NULL)
    {
        SendErr(inst, ERR_INVALID_ARGS, "SOURce:ENAble? needs one argument: channel");
        return;
    }
    chArg = atol(tok);

    if ((chArg < 1L) || (chArg > (long)HRTIM_NUM_CHANNELS))
    {
        SendErr(inst, ERR_INVALID_CHANNEL, "Invalid channel");
        return;
    }
    ch = (uint8_t)(chArg - 1L);

    snprintf(buf, sizeof(buf), "OK %u\r\n", (unsigned int)PID_GetChannelEnable(ch));
    uart_send(inst, buf);
}

void cmd_chan_nickname(uart_instance_t *inst, char *args)
{
    char *tok;
    long  chArg;
    uint8_t ch;

    tok = (args != NULL) ? strtok(args, " \r\n") : NULL;
    if (tok == NULL)
    {
        SendErr(inst, ERR_INVALID_ARGS, "CHANnel:NICKname needs two arguments: channel name");
        return;
    }
    chArg = atol(tok);

    tok = strtok(NULL, " \r\n");
    if (tok == NULL)
    {
        SendErr(inst, ERR_INVALID_ARGS, "CHANnel:NICKname needs two arguments: channel name");
        return;
    }

    if ((chArg < 1L) || (chArg > (long)HRTIM_NUM_CHANNELS))
    {
        SendErr(inst, ERR_INVALID_CHANNEL, "Invalid channel");
        return;
    }
    ch = (uint8_t)(chArg - 1L);

    if (PID_SetChannelNickname(ch, tok) == 0U)
    {
        SendErr(inst, ERR_INVALID_NICKNAME, "Invalid nickname -- 1-PID_CHANNEL_NICKNAME_MAX_LEN chars, "
                          "no spaces, and not the reserved value '-'");
        return;
    }
    uart_send(inst, "OK\r\n");
}

void cmd_chan_nickname_query(uart_instance_t *inst, char *args)
{
    char buf[24U + PID_CHANNEL_NICKNAME_MAX_LEN];
    char *tok;
    long  chArg;
    uint8_t ch;
    const char *name;

    tok = (args != NULL) ? strtok(args, " \r\n") : NULL;
    if (tok == NULL)
    {
        SendErr(inst, ERR_INVALID_ARGS, "CHANnel:NICKname? needs one argument: channel");
        return;
    }
    chArg = atol(tok);

    if ((chArg < 1L) || (chArg > (long)HRTIM_NUM_CHANNELS))
    {
        SendErr(inst, ERR_INVALID_CHANNEL, "Invalid channel");
        return;
    }
    ch = (uint8_t)(chArg - 1L);

    /* "-" is the wire sentinel for "no nickname set" -- see
       PID_SetChannelNickname()'s own doc comment in pid.h for why: an
       empty second token ("OK \r\n") would be ambiguous to parse on the
       client side, so this always emits exactly two space-separated
       tokens. */
    name = PID_GetChannelNickname(ch);
    if (name[0] == '\0')
    {
        name = "-";
    }

    snprintf(buf, sizeof(buf), "OK %s\r\n", name);
    uart_send(inst, buf);
}

void cmd_shot_timing(uart_instance_t *inst, char *args)
{
    char *tok;
    double rampUpTimeS;
    double flatTopTimeS;
    double rampDownTimeS;

    /* Three arguments now -- rampUpTimeS/rampDownTimeS independently
       configurable, added 2026-09-22 per direct request (was two
       arguments, rampTimeS shared for both directions, before this).
       A caller still sending the old two-argument form gets the
       ordinary "needs three arguments" ERR 12 below, same as any
       other malformed call -- no silent old-format fallback. */
    tok = (args != NULL) ? strtok(args, " \r\n") : NULL;
    if (tok == NULL)
    {
        SendErr(inst, ERR_INVALID_ARGS, "SHOT:TIMing needs three arguments: "
                           "rampUpTimeS flatTopTimeS rampDownTimeS");
        return;
    }
    rampUpTimeS = atof(tok);

    tok = strtok(NULL, " \r\n");
    if (tok == NULL)
    {
        SendErr(inst, ERR_INVALID_ARGS, "SHOT:TIMing needs three arguments: "
                           "rampUpTimeS flatTopTimeS rampDownTimeS");
        return;
    }
    flatTopTimeS = atof(tok);

    tok = strtok(NULL, " \r\n");
    if (tok == NULL)
    {
        SendErr(inst, ERR_INVALID_ARGS, "SHOT:TIMing needs three arguments: "
                           "rampUpTimeS flatTopTimeS rampDownTimeS");
        return;
    }
    rampDownTimeS = atof(tok);

    if ((rampUpTimeS <= 0.0) || (flatTopTimeS <= 0.0) || (rampDownTimeS <= 0.0))
    {
        SendErr(inst, ERR_INVALID_ARGS, "rampUpTimeS/flatTopTimeS/rampDownTimeS must be > 0");
        return;
    }

    /* Operator-facing unit is seconds (5-15s ramp, 1-15s flat-top
       nominal, per the real shot profile) -- PID_SetProfileTiming()'s
       own unit is milliseconds, matching SOURce:RAMP's durationMs. */
    if (PID_SetProfileTiming((uint32_t)(rampUpTimeS * 1000.0), (uint32_t)(flatTopTimeS * 1000.0),
                              (uint32_t)(rampDownTimeS * 1000.0)) == 0U)
    {
        SendErr(inst, ERR_INVALID_ARGS, "rampUpTimeS/flatTopTimeS/rampDownTimeS too small");
        return;
    }
    uart_send(inst, "OK\r\n");
}

/* Added 2026-09-10 -- see cmd_pid_gains_query()'s own comment on why
   (SHOT:TIMing was write-only until now). ERR 12 (not just an
   empty/zero OK reply) if timing was never successfully set -- a real,
   meaningful "not configured yet" state (PID_ProfileStart() itself
   refuses to run in it), worth a distinct reply rather than silently
   reporting 0 0 0 as if that were a real value. Extended 2026-09-22 to
   report rampDownTimeS as a third value, matching the setter's own
   independently-configurable ramp-up/ramp-down. */
void cmd_shot_timing_query(uart_instance_t *inst, char *args)
{
    char buf[64];
    uint32_t rampUpTimeMs;
    uint32_t flatTopTimeMs;
    uint32_t rampDownTimeMs;
    (void)args;

    if (PID_GetProfileTiming(&rampUpTimeMs, &flatTopTimeMs, &rampDownTimeMs) == 0U)
    {
        SendErr(inst, ERR_INVALID_ARGS, "Profile timing not set -- send SHOT:TIMing first");
        return;
    }

    /* ms -> seconds, the operator-facing unit (matches the setter's
       own convention) -- %g rather than integer division so a
       sub-second value (e.g. 500 ms -> "0.5") round-trips cleanly. */
    snprintf(buf, sizeof(buf), "OK %g %g %g\r\n",
             (double)rampUpTimeMs / 1000.0, (double)flatTopTimeMs / 1000.0,
             (double)rampDownTimeMs / 1000.0);
    uart_send(inst, buf);
}

void cmd_shot_current(uart_instance_t *inst, char *args)
{
    char *tok;
    long  chArg;
    double currentA;
    uint8_t ch;

    tok = (args != NULL) ? strtok(args, " \r\n") : NULL;
    if (tok == NULL)
    {
        SendErr(inst, ERR_INVALID_ARGS, "SHOT:CURRent needs two arguments: channel demandCurrentA");
        return;
    }
    chArg = atol(tok);

    tok = strtok(NULL, " \r\n");
    if (tok == NULL)
    {
        SendErr(inst, ERR_INVALID_ARGS, "SHOT:CURRent needs two arguments: channel demandCurrentA");
        return;
    }
    currentA = atof(tok);

    if ((chArg < 1L) || (chArg > (long)HRTIM_NUM_CHANNELS))
    {
        SendErr(inst, ERR_INVALID_CHANNEL, "Invalid channel");
        return;
    }
    ch = (uint8_t)(chArg - 1L);

    if (currentA < 0.0)
    {
        SendErr(inst, ERR_INVALID_ARGS, "demandCurrentA must be >= 0");
        return;
    }

    /* PID_SetProfileCurrent() clamps into [0, this channel's own
       PFM_MAX_CURRENT_A_PER_CHANNEL entry] itself -- matches
       SOURce:SETpoint's own clamp-don't-reject convention. */
    (void)PID_SetProfileCurrent(ch, (float)currentA);
    uart_send(inst, "OK\r\n");
}

/* Added 2026-09-10 -- see cmd_pid_gains_query()'s own comment on why
   (SHOT:CURRent was write-only until now). Always succeeds for
   a valid channel (demandCurrentA defaults to 0.0f, a real value, not
   an "unset" sentinel -- unlike profile timing there's no distinct
   "never configured" state worth a separate ERR here). */
void cmd_shot_current_query(uart_instance_t *inst, char *args)
{
    char buf[32];
    char *tok;
    long  chArg;
    uint8_t ch;
    float demandCurrentA;

    tok = (args != NULL) ? strtok(args, " \r\n") : NULL;
    if (tok == NULL)
    {
        SendErr(inst, ERR_INVALID_ARGS, "SHOT:CURRent? needs one argument: channel");
        return;
    }
    chArg = atol(tok);

    if ((chArg < 1L) || (chArg > (long)HRTIM_NUM_CHANNELS))
    {
        SendErr(inst, ERR_INVALID_CHANNEL, "Invalid channel");
        return;
    }
    ch = (uint8_t)(chArg - 1L);

    (void)PID_GetProfileCurrent(ch, &demandCurrentA);

    snprintf(buf, sizeof(buf), "OK %g\r\n", (double)demandCurrentA);
    uart_send(inst, buf);
}

/* Now gated on the state machine (state_machine.h, added 2026-09-13) --
   per direct instruction, this only ever starts outputs from ARMED
   (ARM first, see cmd_arm()). SM_Fire() calls PID_ProfileStart()
   internally and only actually transitions to FIRING if that
   succeeds -- THREE distinct failure reasons as of 2026-09-16 (state
   mismatch, external-enable interlock not satisfied, profile timing
   never set), each reported distinctly here rather than collapsing
   them into one generic error. The external-enable check is done HERE
   (SM_ExternalEnableOk(), state_machine.h) rather than just relying on
   SM_Fire()'s own internal redundant re-check of the same thing,
   specifically so this can report a precise message instead of the
   generic "stays ARMED" SM_Fire() itself returns -- see that
   function's own comment (state_machine.c) for why it still re-checks
   anyway (a narrow race window between this check and the SM_Fire()
   call, and defense against any future caller that reaches SM_Fire()
   directly). */
void cmd_shot_start(uart_instance_t *inst, char *args)
{
    (void)args;

    if (RequireArmedAndEnabled(inst) == 0U)
    {
        return;
    }

    if (SM_Fire() == 0U)
    {
        SendErr(inst, ERR_INVALID_ARGS, "SHOT:TIMing must be set before SHOT:STARt");
        return;
    }
    uart_send(inst, "OK\r\n");
}
