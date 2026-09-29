/*
 * cmd_sim.c
 *
 * Simulator-only SIM:* commands (BUILD_TARGET_SIMULATOR builds).
 * Handlers are declared in commands.h and registered in command_table.c.
 */

#include "commands.h"
#include "cmd_common.h"
#include "ctrlr_config.h"
#include "sim_transrex.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if defined(BUILD_TARGET_SIMULATOR)
/* --------------------------------------------------------------------------
 * SIM:FAULT:WATERTEMP <ch> <0|1> / SIM:FAULT:WATERTEMP? <ch>
 * SIM:FAULT:ENERPRO <ch> <0|1> / SIM:FAULT:ENERPRO? <ch>
 * SIM:FAULT:OCP <ch> <0|1> / SIM:FAULT:OCP? <ch>
 * SIM:MODEL:TAU <ms> / SIM:MODEL:TAU?
 * SIM:CHANnel:STATus? <ch>
 *
 * Added 2026-09-18 -- the sim_transrex.h/.c backed command namespace
 * for the Transrex simulator (see that module's own header comment for
 * the full design: DRIVE capture -> low-pass filter -> FEEDBACK
 * generation, ENA_OUT/CONTACT_OUT gating, per-channel/per-category
 * fault injection). SIMULATOR-ONLY, guarded out of a controller build
 * entirely -- there is no "simulated Transrex" concept on the real
 * controller for this namespace to mean anything about, unlike the
 * generic DIAGnostic:GPOut09-12/etc. pins which stay present on both
 * targets. 1-based wire channel, converted to this project's 0-based
 * internal convention -- same pattern as every other XREX:CHANnel:*
 * command above; ERR 11/12 reused for invalid-channel/missing-argument,
 * same convention throughout this file.
 *
 * WATERTEMP/ENERPRO/OCP drive the SAME physical pins XREX:CHANnel:
 * ENAOut/CONTactOut and DIAGnostic:GPOut09-12 can also drive directly
 * (see sim_transrex.c's own pin-role table) -- don't mix both
 * mechanisms on the same channel/category at once. */
void cmd_sim_fault_watertemp(uart_instance_t *inst, char *args)
{
    char *tok;
    long  chArg;
    long  val;

    tok = (args != NULL) ? strtok(args, " \r\n") : NULL;
    if (tok == NULL)
    {
        SendErr(inst, ERR_INVALID_ARGS, "SIM:FAULT:WATERTEMP needs two arguments: channel, 0|1");
        return;
    }
    chArg = atol(tok);
    if ((chArg < 1L) || (chArg > (long)HRTIM_NUM_CHANNELS))
    {
        SendErr(inst, ERR_INVALID_CHANNEL, "Invalid channel");
        return;
    }

    tok = strtok(NULL, " \r\n");
    if (tok == NULL)
    {
        SendErr(inst, ERR_INVALID_ARGS, "SIM:FAULT:WATERTEMP needs two arguments: channel, 0|1");
        return;
    }
    val = atol(tok);

    SimTransrex_SetFaultWaterTemp((uint8_t)(chArg - 1L), (val != 0L) ? 1U : 0U);
    uart_send(inst, "OK\r\n");
}

void cmd_sim_fault_watertemp_query(uart_instance_t *inst, char *args)
{
    char *tok;
    long  chArg;
    char  buf[16];

    tok = (args != NULL) ? strtok(args, " \r\n") : NULL;
    if (tok == NULL)
    {
        SendErr(inst, ERR_INVALID_ARGS, "SIM:FAULT:WATERTEMP? needs one argument: channel");
        return;
    }
    chArg = atol(tok);
    if ((chArg < 1L) || (chArg > (long)HRTIM_NUM_CHANNELS))
    {
        SendErr(inst, ERR_INVALID_CHANNEL, "Invalid channel");
        return;
    }

    snprintf(buf, sizeof(buf), "OK %u\r\n",
             (unsigned)SimTransrex_GetFaultWaterTemp((uint8_t)(chArg - 1L)));
    uart_send(inst, buf);
}

void cmd_sim_fault_enerpro(uart_instance_t *inst, char *args)
{
    char *tok;
    long  chArg;
    long  val;

    tok = (args != NULL) ? strtok(args, " \r\n") : NULL;
    if (tok == NULL)
    {
        SendErr(inst, ERR_INVALID_ARGS, "SIM:FAULT:ENERPRO needs two arguments: channel, 0|1");
        return;
    }
    chArg = atol(tok);
    if ((chArg < 1L) || (chArg > (long)HRTIM_NUM_CHANNELS))
    {
        SendErr(inst, ERR_INVALID_CHANNEL, "Invalid channel");
        return;
    }

    tok = strtok(NULL, " \r\n");
    if (tok == NULL)
    {
        SendErr(inst, ERR_INVALID_ARGS, "SIM:FAULT:ENERPRO needs two arguments: channel, 0|1");
        return;
    }
    val = atol(tok);

    SimTransrex_SetFaultEnerpro((uint8_t)(chArg - 1L), (val != 0L) ? 1U : 0U);
    uart_send(inst, "OK\r\n");
}

void cmd_sim_fault_enerpro_query(uart_instance_t *inst, char *args)
{
    char *tok;
    long  chArg;
    char  buf[16];

    tok = (args != NULL) ? strtok(args, " \r\n") : NULL;
    if (tok == NULL)
    {
        SendErr(inst, ERR_INVALID_ARGS, "SIM:FAULT:ENERPRO? needs one argument: channel");
        return;
    }
    chArg = atol(tok);
    if ((chArg < 1L) || (chArg > (long)HRTIM_NUM_CHANNELS))
    {
        SendErr(inst, ERR_INVALID_CHANNEL, "Invalid channel");
        return;
    }

    snprintf(buf, sizeof(buf), "OK %u\r\n",
             (unsigned)SimTransrex_GetFaultEnerpro((uint8_t)(chArg - 1L)));
    uart_send(inst, buf);
}

void cmd_sim_fault_ocp(uart_instance_t *inst, char *args)
{
    char *tok;
    long  chArg;
    long  val;

    tok = (args != NULL) ? strtok(args, " \r\n") : NULL;
    if (tok == NULL)
    {
        SendErr(inst, ERR_INVALID_ARGS, "SIM:FAULT:OCP needs two arguments: channel, 0|1");
        return;
    }
    chArg = atol(tok);
    if ((chArg < 1L) || (chArg > (long)HRTIM_NUM_CHANNELS))
    {
        SendErr(inst, ERR_INVALID_CHANNEL, "Invalid channel");
        return;
    }

    tok = strtok(NULL, " \r\n");
    if (tok == NULL)
    {
        SendErr(inst, ERR_INVALID_ARGS, "SIM:FAULT:OCP needs two arguments: channel, 0|1");
        return;
    }
    val = atol(tok);

    SimTransrex_SetFaultOcp((uint8_t)(chArg - 1L), (val != 0L) ? 1U : 0U);
    uart_send(inst, "OK\r\n");
}

void cmd_sim_fault_ocp_query(uart_instance_t *inst, char *args)
{
    char *tok;
    long  chArg;
    char  buf[16];

    tok = (args != NULL) ? strtok(args, " \r\n") : NULL;
    if (tok == NULL)
    {
        SendErr(inst, ERR_INVALID_ARGS, "SIM:FAULT:OCP? needs one argument: channel");
        return;
    }
    chArg = atol(tok);
    if ((chArg < 1L) || (chArg > (long)HRTIM_NUM_CHANNELS))
    {
        SendErr(inst, ERR_INVALID_CHANNEL, "Invalid channel");
        return;
    }

    snprintf(buf, sizeof(buf), "OK %u\r\n",
             (unsigned)SimTransrex_GetFaultOcp((uint8_t)(chArg - 1L)));
    uart_send(inst, buf);
}

void cmd_sim_model_tau(uart_instance_t *inst, char *args)
{
    char *tok;
    long  val;

    tok = (args != NULL) ? strtok(args, " \r\n") : NULL;
    if (tok == NULL)
    {
        SendErr(inst, ERR_INVALID_ARGS, "SIM:MODEL:TAU needs one argument: milliseconds");
        return;
    }
    val = atol(tok);

    if ((val <= 0L) || (SimTransrex_SetTauMs((uint32_t)val) == 0U))
    {
        SendErr(inst, ERR_INVALID_CHANNEL, "Invalid tau -- must be a positive number of milliseconds");
        return;
    }
    uart_send(inst, "OK\r\n");
}

void cmd_sim_model_tau_query(uart_instance_t *inst, char *args)
{
    char buf[16];
    (void)args;

    snprintf(buf, sizeof(buf), "OK %lu\r\n", (unsigned long)SimTransrex_GetTauMs());
    uart_send(inst, buf);
}

/* SIM:DIAGnostic:IDLETONE <0|1> / ? -- added 2026-09-21, direct request:
   board-wide diagnostic idle tone, see sim_transrex.h's own comment.
   Not per-channel -- there's no real use case for lighting only some of
   the 4 transmitters during a bench visual check. */
void cmd_sim_diag_idletone(uart_instance_t *inst, char *args)
{
    char *tok;
    long  val;

    tok = (args != NULL) ? strtok(args, " \r\n") : NULL;
    if (tok == NULL)
    {
        SendErr(inst, ERR_INVALID_ARGS, "SIM:DIAGnostic:IDLETONE needs one argument: 0 or 1");
        return;
    }
    val = atol(tok);
    if ((val != 0L) && (val != 1L))
    {
        SendErr(inst, ERR_INVALID_CHANNEL, "Invalid value -- must be 0 or 1");
        return;
    }
    SimTransrex_SetIdleToneEnabled((uint8_t)val);
    uart_send(inst, "OK\r\n");
}

void cmd_sim_diag_idletone_query(uart_instance_t *inst, char *args)
{
    char buf[16];
    (void)args;

    snprintf(buf, sizeof(buf), "OK %u\r\n", (unsigned)SimTransrex_GetIdleToneEnabled());
    uart_send(inst, buf);
}

void cmd_sim_channel_status(uart_instance_t *inst, char *args)
{
    char *tok;
    long  chArg;
    char  buf[128];
    uint32_t measuredHz = 0U, feedbackHz = 0U;
    uint8_t  enaGated = 0U, contactGated = 0U;
    uint8_t  faultWaterTemp = 0U, faultEnerpro = 0U, faultOcp = 0U;

    tok = (args != NULL) ? strtok(args, " \r\n") : NULL;
    if (tok == NULL)
    {
        SendErr(inst, ERR_INVALID_ARGS, "SIM:CHANnel:STATus? needs one argument: channel");
        return;
    }
    chArg = atol(tok);
    if ((chArg < 1L) || (chArg > (long)HRTIM_NUM_CHANNELS))
    {
        SendErr(inst, ERR_INVALID_CHANNEL, "Invalid channel");
        return;
    }

    (void)SimTransrex_GetChannelStatus((uint8_t)(chArg - 1L), &measuredHz, &feedbackHz,
                                        &enaGated, &contactGated,
                                        &faultWaterTemp, &faultEnerpro, &faultOcp);

    snprintf(buf, sizeof(buf),
             "OK DRIVE_HZ=%lu FEEDBACK_HZ=%lu ENA_OUT=%s CONTACT_OUT=%s "
             "WATERTEMP_FAULT=%u ENERPRO_FAULT=%u OCP_FAULT=%u\r\n",
             (unsigned long)measuredHz, (unsigned long)feedbackHz,
             enaGated ? "HIGH" : "LOW", contactGated ? "HIGH" : "LOW",
             (unsigned)faultWaterTemp, (unsigned)faultEnerpro, (unsigned)faultOcp);
    uart_send(inst, buf);
}

/* --------------------------------------------------------------------------
 * SIM:LOG <ch> <maxSamples> <minIntervalMs>
 * SIM:LOGDATA? <ch>
 *
 * Added 2026-09-18, per direct correction: host-side polling of
 * SIM:CHANnel:STATus? during a shot (the original
 * run_simulator_validation.py approach) was too coarse and added
 * serial round-trip jitter -- this is the simulator-side equivalent of
 * LOG:ARM/LOG:DATA? (pid.h/cmd_control.c), backed by
 * SimTransrex_ArmLog()/GetLogCount()/GetLogSample() (sim_transrex.h --
 * see that header's own comment for why each sample carries its own
 * timestamp instead of a single shared rate_hz like LOG:DATA? uses:
 * SimTransrex_Update() runs off the main loop, not a fixed hardware
 * tick). `ch` is 1-based on the wire, same convention as every other
 * XREX:CHANnel:/SOURce: command in this file. */
void cmd_sim_log(uart_instance_t *inst, char *args)
{
    char *tok;
    long  chArg;
    long  maxSamplesArg;
    long  minIntervalArg;

    tok = (args != NULL) ? strtok(args, " \r\n") : NULL;
    if (tok == NULL)
    {
        SendErr(inst, ERR_INVALID_ARGS, "SIM:LOG needs three arguments: channel maxSamples minIntervalMs");
        return;
    }
    chArg = atol(tok);
    if ((chArg < 1L) || (chArg > (long)HRTIM_NUM_CHANNELS))
    {
        SendErr(inst, ERR_INVALID_CHANNEL, "Invalid channel");
        return;
    }

    tok = strtok(NULL, " \r\n");
    if (tok == NULL)
    {
        SendErr(inst, ERR_INVALID_ARGS, "SIM:LOG needs three arguments: channel maxSamples minIntervalMs");
        return;
    }
    maxSamplesArg = atol(tok);

    tok = strtok(NULL, " \r\n");
    if (tok == NULL)
    {
        SendErr(inst, ERR_INVALID_ARGS, "SIM:LOG needs three arguments: channel maxSamples minIntervalMs");
        return;
    }
    minIntervalArg = atol(tok);

    if ((maxSamplesArg < 1L) || (maxSamplesArg > (long)SIM_LOG_MAX_SAMPLES) || (minIntervalArg < 0L))
    {
        SendErr(inst, ERR_INVALID_ARGS, "maxSamples must be 1-SIM_LOG_MAX_SAMPLES, minIntervalMs >= 0");
        return;
    }

    (void)SimTransrex_ArmLog((uint8_t)(chArg - 1L), (uint16_t)maxSamplesArg, (uint16_t)minIntervalArg);
    uart_send(inst, "OK\r\n");
}

void cmd_sim_logdata(uart_instance_t *inst, char *args)
{
    char *tok;
    long  chArg;
    uint8_t ch;
    uint16_t count;
    char chunk[48];

    tok = (args != NULL) ? strtok(args, " \r\n") : NULL;
    if (tok == NULL)
    {
        SendErr(inst, ERR_INVALID_ARGS, "SIM:LOGDATA? needs one argument: channel");
        return;
    }
    chArg = atol(tok);
    if ((chArg < 1L) || (chArg > (long)HRTIM_NUM_CHANNELS))
    {
        SendErr(inst, ERR_INVALID_CHANNEL, "Invalid channel");
        return;
    }
    ch = (uint8_t)(chArg - 1L);

    count = SimTransrex_GetLogCount(ch);
    snprintf(chunk, sizeof(chunk), "OK %u", (unsigned int)count);
    uart_send(inst, chunk);

    for (uint16_t i = 0U; i < count; i++)
    {
        uint32_t timeMs = 0U, driveHz = 0U, feedbackHz = 0U;
        (void)SimTransrex_GetLogSample(ch, i, &timeMs, &driveHz, &feedbackHz);
        snprintf(chunk, sizeof(chunk), " %lu %lu %lu",
                 (unsigned long)timeMs, (unsigned long)driveHz, (unsigned long)feedbackHz);
        uart_send(inst, chunk);
    }

    uart_send(inst, "\r\n");
}
#endif /* BUILD_TARGET_SIMULATOR */
