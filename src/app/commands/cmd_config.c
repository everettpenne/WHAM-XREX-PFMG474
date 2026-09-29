/*
 * cmd_config.c
 *
 * CONFig:* -- channel count, loop rate, frequency/current limits,
 * slew rate, fault ramp time, fault-input polarities.
 * Handlers are declared in commands.h and registered in command_table.c.
 */

#include "commands.h"
#include "cmd_common.h"
#include "ctrlr_config.h"
#include "pid.h"
#include "xrex_io.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* --------------------------------------------------------------------------
 * CONFig:CHANnels?
 *
 * Reports HRTIM_NUM_CHANNELS (ctrlr_config.h) -- added alongside the
 * 2026-09-08 channel-count generalization specifically so
 * python/pfm_table_upload.py (or any other host tool) can confirm what
 * a given board was actually built for, rather than assuming a value
 * that could silently drift from reality after a rebuild with a
 * different channel count -- a TABLE:STEP wire-format mismatch that
 * would otherwise only surface as a confusing ERR 4 partway through an
 * upload.
 * -------------------------------------------------------------------------- */
void cmd_config_channels(uart_instance_t *inst, char *args)
{
    char buf[32];
    (void)args;

    snprintf(buf, sizeof(buf), "OK %u\r\n", (unsigned int)HRTIM_NUM_CHANNELS);
    uart_send(inst, buf);
}

/* CONFig:PIDRate <hz> / ? -- added 2026-09-22, direct request:
   PID_LOOP_RATE_HZ (ctrlr_config.h) made runtime-configurable. See
   PID_SetLoopRateHz()'s own doc comment (pid.c) for the range check
   and the refuses-while-running policy -- ERR 12 covers both a
   malformed argument and a rejected value (out of range, hardware
   register check failed, or a shot currently running), since none of
   those need a more specific error code than "this command's
   argument was rejected." */
void cmd_config_pid_rate(uart_instance_t *inst, char *args)
{
    char *tok;
    long  hz;

    tok = (args != NULL) ? strtok(args, " \r\n") : NULL;
    if (tok == NULL)
    {
        SendErr(inst, ERR_INVALID_ARGS, "CONFig:PIDRate needs one argument: hz");
        return;
    }
    hz = atol(tok);
    if ((hz <= 0L) || (PID_SetLoopRateHz((uint32_t)hz) == 0U))
    {
        SendErr(inst, ERR_INVALID_ARGS, "hz rejected -- out of range, hardware register "
                           "check failed, or a shot is currently running");
        return;
    }
    uart_send(inst, "OK\r\n");
}

void cmd_config_pid_rate_query(uart_instance_t *inst, char *args)
{
    char buf[32];
    (void)args;

    snprintf(buf, sizeof(buf), "OK %lu\r\n", (unsigned long)PID_GetLoopRateHz());
    uart_send(inst, buf);
}

/* CONFig:TURNONHz <hz> / ? -- added 2026-09-22, direct request:
   PFM_TURNON_FREQ_HZ (ctrlr_config.h) made runtime-configurable. See
   PID_SetTurnonFreqHz()'s own doc comment (pid.c) for the cross-
   validation applied. */
void cmd_config_turnon_hz(uart_instance_t *inst, char *args)
{
    char *tok;
    long  hz;

    tok = (args != NULL) ? strtok(args, " \r\n") : NULL;
    if (tok == NULL)
    {
        SendErr(inst, ERR_INVALID_ARGS, "CONFig:TURNONHz needs one argument: hz");
        return;
    }
    hz = atol(tok);
    if ((hz <= 0L) || (PID_SetTurnonFreqHz((uint32_t)hz) == 0U))
    {
        SendErr(inst, ERR_INVALID_ARGS, "hz rejected -- must be >= PID_OUTPUT_MIN_HZ and "
                           "strictly below the current CONFig:MAXFREQHz");
        return;
    }
    uart_send(inst, "OK\r\n");
}

void cmd_config_turnon_hz_query(uart_instance_t *inst, char *args)
{
    char buf[32];
    (void)args;

    snprintf(buf, sizeof(buf), "OK %lu\r\n", (unsigned long)PID_GetTurnonFreqHz());
    uart_send(inst, buf);
}

/* CONFig:MAXFREQHz <hz> / ? -- added 2026-09-22, direct request:
   PFM_MAX_FREQ_HZ (ctrlr_config.h) made runtime-configurable. See
   PID_SetMaxFreqHz()'s own doc comment (pid.c) for the cross-
   validation applied. */
void cmd_config_max_freq_hz(uart_instance_t *inst, char *args)
{
    char *tok;
    long  hz;

    tok = (args != NULL) ? strtok(args, " \r\n") : NULL;
    if (tok == NULL)
    {
        SendErr(inst, ERR_INVALID_ARGS, "CONFig:MAXFREQHz needs one argument: hz");
        return;
    }
    hz = atol(tok);
    if ((hz <= 0L) || (PID_SetMaxFreqHz((uint32_t)hz) == 0U))
    {
        SendErr(inst, ERR_INVALID_ARGS, "hz rejected -- must be <= PID_OUTPUT_MAX_HZ and "
                           "strictly above the current CONFig:TURNONHz");
        return;
    }
    uart_send(inst, "OK\r\n");
}

void cmd_config_max_freq_hz_query(uart_instance_t *inst, char *args)
{
    char buf[32];
    (void)args;

    snprintf(buf, sizeof(buf), "OK %lu\r\n", (unsigned long)PID_GetMaxFreqHz());
    uart_send(inst, buf);
}

/* CONFig:MAXCURRent <ch> <amps> / <ch> -- added 2026-09-22, direct
   request: PFM_MAX_CURRENT_A_PER_CHANNEL[ch] (ctrlr_config.h) made
   runtime-configurable -- the real per-channel calibration that
   constant's own "MUST BE CALIBRATED BEFORE FINAL DEPLOYMENT" comment
   describes. See PID_SetMaxCurrentA()'s own doc comment (pid.c).
   1-based channel on the wire, matching every other SOURce:* or
   CONFig:* per-channel command in this file. */
void cmd_config_max_current(uart_instance_t *inst, char *args)
{
    char  *tok;
    long   chArg;
    double amps;
    uint8_t ch;

    tok = (args != NULL) ? strtok(args, " \r\n") : NULL;
    if (tok == NULL)
    {
        SendErr(inst, ERR_INVALID_ARGS, "CONFig:MAXCURRent needs two arguments: channel amps");
        return;
    }
    chArg = atol(tok);

    tok = strtok(NULL, " \r\n");
    if (tok == NULL)
    {
        SendErr(inst, ERR_INVALID_ARGS, "CONFig:MAXCURRent needs two arguments: channel amps");
        return;
    }
    amps = atof(tok);

    if ((chArg < 1L) || (chArg > (long)HRTIM_NUM_CHANNELS))
    {
        SendErr(inst, ERR_INVALID_CHANNEL, "Invalid channel");
        return;
    }
    ch = (uint8_t)(chArg - 1L);

    if (PID_SetMaxCurrentA(ch, (float)amps) == 0U)
    {
        SendErr(inst, ERR_INVALID_ARGS, "amps rejected -- must be > 0");
        return;
    }
    uart_send(inst, "OK\r\n");
}

void cmd_config_max_current_query(uart_instance_t *inst, char *args)
{
    char   buf[32];
    char  *tok;
    long   chArg;
    uint8_t ch;
    float  amps;

    tok = (args != NULL) ? strtok(args, " \r\n") : NULL;
    if (tok == NULL)
    {
        SendErr(inst, ERR_INVALID_ARGS, "CONFig:MAXCURRent? needs one argument: channel");
        return;
    }
    chArg = atol(tok);
    if ((chArg < 1L) || (chArg > (long)HRTIM_NUM_CHANNELS))
    {
        SendErr(inst, ERR_INVALID_CHANNEL, "Invalid channel");
        return;
    }
    ch = (uint8_t)(chArg - 1L);

    (void)PID_GetMaxCurrentA(ch, &amps);
    snprintf(buf, sizeof(buf), "OK %g\r\n", (double)amps);
    uart_send(inst, buf);
}

/* CONFig:SLEWRate <hzPerTick> / ? -- added 2026-09-22, direct request:
   PID_OUTPUT_MAX_SLEW_HZ_PER_TICK (ctrlr_config.h) made runtime-
   configurable. See PID_SetSlewRateHzPerTick()'s own doc comment
   (pid.c) -- this is a real hardware-safety clamp, only enforced > 0
   here, no upper bound. */
void cmd_config_slew_rate(uart_instance_t *inst, char *args)
{
    char   *tok;
    double  hzPerTick;

    tok = (args != NULL) ? strtok(args, " \r\n") : NULL;
    if (tok == NULL)
    {
        SendErr(inst, ERR_INVALID_ARGS, "CONFig:SLEWRate needs one argument: hzPerTick");
        return;
    }
    hzPerTick = atof(tok);
    if (PID_SetSlewRateHzPerTick((float)hzPerTick) == 0U)
    {
        SendErr(inst, ERR_INVALID_ARGS, "hzPerTick rejected -- must be > 0");
        return;
    }
    uart_send(inst, "OK\r\n");
}

void cmd_config_slew_rate_query(uart_instance_t *inst, char *args)
{
    char buf[32];
    (void)args;

    snprintf(buf, sizeof(buf), "OK %g\r\n", (double)PID_GetSlewRateHzPerTick());
    uart_send(inst, buf);
}

/* CONFig:FaultRampTime <s> / ? -- added 2026-09-22, direct request:
   FAULT_RAMP_DOWN_TIME_S (ctrlr_config.h) made runtime-configurable.
   See PID_SetFaultRampDownTimeS()'s own doc comment (pid.c) -- only
   takes effect on the NEXT fault, not one already in progress. */
void cmd_config_fault_ramp_time(uart_instance_t *inst, char *args)
{
    char   *tok;
    double  s;

    tok = (args != NULL) ? strtok(args, " \r\n") : NULL;
    if (tok == NULL)
    {
        SendErr(inst, ERR_INVALID_ARGS, "CONFig:FaultRampTime needs one argument: seconds");
        return;
    }
    s = atof(tok);
    if (PID_SetFaultRampDownTimeS((float)s) == 0U)
    {
        SendErr(inst, ERR_INVALID_ARGS, "seconds rejected -- must be > 0");
        return;
    }
    uart_send(inst, "OK\r\n");
}

void cmd_config_fault_ramp_time_query(uart_instance_t *inst, char *args)
{
    char buf[32];
    (void)args;

    snprintf(buf, sizeof(buf), "OK %g\r\n", (double)PID_GetFaultRampDownTimeS());
    uart_send(inst, buf);
}

/* CONFig:FaultPolarity:WATER/:TEMP/:ENERPRO/:OCP <0|1> / ? -- added
   2026-09-22, direct request: XR_WATER_FLT_POLARITY/XR_TMP_FLT_POLARITY/
   XR_ENERPRO_FLT_POLARITY/XR_OCP_FLT_POLARITY (ctrlr_config.h) made
   runtime-configurable. <0|1> matches FAULT_POLARITY_NORMALLY_HIGH(0)/
   _LOW(1)'s own encoding (ctrlr_config.h) directly -- see
   XrexIo_SetFaultPolarity*()'s own doc comment (xrex_io.h) for the
   validation applied. Four independent commands, not one with a
   category argument, matching this project's existing SIM:FAULT:*
   per-category convention and the fact these ARE four independent
   settings on real hardware (ctrlr_config.h's own comment on why). */
static void ConfigFaultPolaritySet(uart_instance_t *inst, char *args, const char *name,
                                    uint8_t (*setter)(uint32_t))
{
    char *tok;
    long  val;

    tok = (args != NULL) ? strtok(args, " \r\n") : NULL;
    if (tok == NULL)
    {
        SendErr(inst, ERR_INVALID_ARGS, "needs one argument: 0 (normally high) or 1 (normally low)");
        (void)name;
        return;
    }
    val = atol(tok);
    if ((val != 0L) && (val != 1L))
    {
        SendErr(inst, ERR_INVALID_ARGS, "must be 0 (normally high) or 1 (normally low)");
        return;
    }
    (void)setter((uint32_t)val);   /* can't actually fail -- val is already
                                       checked to be exactly 0 or 1 above,
                                       the same two values the setter itself
                                       accepts (see XrexIo_SetFaultPolarity*()'s
                                       own doc comment) */
    uart_send(inst, "OK\r\n");
}

static void ConfigFaultPolarityQuery(uart_instance_t *inst, char *args, uint32_t (*getter)(void))
{
    char buf[32];
    (void)args;

    snprintf(buf, sizeof(buf), "OK %lu\r\n", (unsigned long)getter());
    uart_send(inst, buf);
}

void cmd_config_fault_polarity_water(uart_instance_t *inst, char *args)
{
    ConfigFaultPolaritySet(inst, args, "WATER", XrexIo_SetFaultPolarityWater);
}
void cmd_config_fault_polarity_water_query(uart_instance_t *inst, char *args)
{
    ConfigFaultPolarityQuery(inst, args, XrexIo_GetFaultPolarityWater);
}

void cmd_config_fault_polarity_temp(uart_instance_t *inst, char *args)
{
    ConfigFaultPolaritySet(inst, args, "TEMP", XrexIo_SetFaultPolarityTemp);
}
void cmd_config_fault_polarity_temp_query(uart_instance_t *inst, char *args)
{
    ConfigFaultPolarityQuery(inst, args, XrexIo_GetFaultPolarityTemp);
}

void cmd_config_fault_polarity_enerpro(uart_instance_t *inst, char *args)
{
    ConfigFaultPolaritySet(inst, args, "ENERPRO", XrexIo_SetFaultPolarityEnerpro);
}
void cmd_config_fault_polarity_enerpro_query(uart_instance_t *inst, char *args)
{
    ConfigFaultPolarityQuery(inst, args, XrexIo_GetFaultPolarityEnerpro);
}

void cmd_config_fault_polarity_ocp(uart_instance_t *inst, char *args)
{
    ConfigFaultPolaritySet(inst, args, "OCP", XrexIo_SetFaultPolarityOcp);
}
void cmd_config_fault_polarity_ocp_query(uart_instance_t *inst, char *args)
{
    ConfigFaultPolarityQuery(inst, args, XrexIo_GetFaultPolarityOcp);
}
