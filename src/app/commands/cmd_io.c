/*
 * cmd_io.c
 *
 * Board I/O: EXTernal:ENAble/TRIGger, DIAGnostic:GPOut09-12,
 * GDS?, XREX:CHANnel:*, GPOut:ENAble, PWMAlt:ENAble.
 * Handlers are declared in commands.h and registered in command_table.c.
 */

#include "commands.h"
#include "cmd_common.h"
#include "ctrlr_config.h"
#include "gate_driver.h"
#include "state_machine.h"
#include "board_io.h"
#include "xrex_io.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* --------------------------------------------------------------------------
 * EXTernal:ENAble <0|1> / EXTernal:ENAble? / EXTernal:INPut?
 *
 * Added 2026-09-16, per direct request. Backing pin MOVED 2026-09-17
 * from PF15 to PF13, per direct instruction: enable and trigger are now
 * independent physical signals on separate pins, not one shared wire --
 * see EXTernal:TRIGger below and state_machine.h's own external-enable
 * section (SM_SetExternalEnableRequired() and friends) for the full
 * design. This is just the wire-command wrapper around that. Own
 * top-level `EXTernal:` namespace (not nested under `SOURce:` or any other
 * existing prefix) -- system-wide config, not specific to any one
 * subsystem, same reasoning as `ARM`/`DISARM` above being bare; two
 * ':'-levels here (rather than one compound word) purely so
 * "ENAble"/"INPut" each get their own independent short-form
 * abbreviation -- see command_table.c's own comment on this command's
 * table entry for why. */
void cmd_ext_enable(uart_instance_t *inst, char *args)
{
    char *tok;
    long  val;

    tok = (args != NULL) ? strtok(args, " \r\n") : NULL;
    if (tok == NULL)
    {
        SendErr(inst, ERR_INVALID_ARGS, "EXTernal:ENAble needs one argument: 0|1");
        return;
    }
    val = atol(tok);

    SM_SetExternalEnableRequired((val != 0L) ? 1U : 0U);
    uart_send(inst, "OK\r\n");
}

void cmd_ext_enable_query(uart_instance_t *inst, char *args)
{
    char buf[16];
    (void)args;

    snprintf(buf, sizeof(buf), "OK %u\r\n", (unsigned)SM_GetExternalEnableRequired());
    uart_send(inst, buf);
}

/* Raw PF13 logic level, independent of whether the interlock is even
   turned on -- lets an operator confirm real wiring/signal presence
   before relying on it, same diagnostic role PFMIN:DEBUG:RAW?/REG?
   played for the PFM_Input fiber-patching investigation
   (docs/changelog.txt, 2026-09-15). MOVED 2026-09-17 from PF15 to
   PF13, same change as cmd_ext_enable() above. */
void cmd_ext_enable_input_query(uart_instance_t *inst, char *args)
{
    char buf[16];
    (void)args;

    snprintf(buf, sizeof(buf), "OK %u\r\n", (unsigned)SM_GetExternalEnableInputRaw());
    uart_send(inst, buf);
}

/* --------------------------------------------------------------------------
 * EXTernal:TRIGger <0|1> / EXTernal:TRIGger? / EXTernal:TRIGger:INPut?
 *
 * Added 2026-09-16, per direct follow-up request. RESTRUCTURED
 * 2026-09-17: this pin (PF15, "Fiber_Enable") now backs TRIGGER ONLY --
 * the external-enable interlock moved to its own pin (PF13, above).
 * See state_machine.h's own external-trigger design comment
 * (SM_SetExternalTriggerRequired() and friends) for the full design --
 * this is just the wire-command wrapper. No longer structurally
 * coupled to EXTernal:ENAble (that dependency existed only because both
 * features read the same wire, back when this was PF15-for-both) --
 * SM_SetExternalTriggerRequired() always succeeds now, so the ERR 16
 * refusal path below is gone. ERR 16 itself is retired, not reassigned
 * -- this project's convention is error codes are never renumbered or
 * reused once assigned (see docs/command_reference.md). */
void cmd_ext_trigger(uart_instance_t *inst, char *args)
{
    char *tok;
    long  val;

    tok = (args != NULL) ? strtok(args, " \r\n") : NULL;
    if (tok == NULL)
    {
        SendErr(inst, ERR_INVALID_ARGS, "EXTernal:TRIGger needs one argument: 0|1");
        return;
    }
    val = atol(tok);

    (void)SM_SetExternalTriggerRequired((val != 0L) ? 1U : 0U);
    uart_send(inst, "OK\r\n");
}

void cmd_ext_trigger_query(uart_instance_t *inst, char *args)
{
    char buf[16];
    (void)args;

    snprintf(buf, sizeof(buf), "OK %u\r\n", (unsigned)SM_GetExternalTriggerRequired());
    uart_send(inst, buf);
}

/* Raw PF15 logic level, independent of whether the trigger feature is
   even turned on -- added 2026-09-17 alongside the PF13/PF15 split;
   previously EXTernal:INPut? covered this same pin (it backed both
   enable and trigger, being the same wire) -- now that they're
   separate, this is trigger's own dedicated diagnostic, matching
   cmd_ext_enable_input_query()'s role for PF13. */
void cmd_ext_trigger_input_query(uart_instance_t *inst, char *args)
{
    char buf[16];
    (void)args;

    snprintf(buf, sizeof(buf), "OK %u\r\n", (unsigned)SM_GetExternalTriggerInputRaw());
    uart_send(inst, buf);
}

/* --------------------------------------------------------------------------
 * DIAGnostic:GPOut12 <0|1> / DIAGnostic:GPOut12?
 *
 * Added 2026-09-16, per direct request: a generic, software-driven
 * diagnostic output on PD1 ("GPOut_12" in the V4 column,
 * docs/pin_mapping_v4.csv -- confirmed GPO there; PF13 was proposed
 * first and corrected -- it's actually "GPInput_12", an input, in that
 * same CSV). GPIO config lives in board_io.c's BoardIo_Init(), matching
 * this project's established precedent (gate_driver.c's GateDriverStatus
 * pins, PF15 above) of plain GPIO config in board_io.c and the read/write
 * logic in the module that actually uses it -- there's no dedicated
 * module for this one, it's a two-line direct HAL_GPIO_WritePin()/
 * ReadPin() pair, not enough behavior to justify one.
 *
 * Immediate use, 2026-09-16: physically loop this pin to PF15
 * (Fiber_Enable) so the external-enable/external-trigger feature above
 * could be driven entirely from the serial console -- precise,
 * repeatable timing on exactly when PF15 goes HIGH/LOW, instead of a
 * hand-operated bench jumper/switch. (2026-09-17 UPDATE: PF15 now
 * backs external-TRIGGER only -- enable moved to its own pin, PF13 --
 * so this same loop now drives trigger specifically; nothing about
 * this command itself changed.) Nothing about this command is PF15-
 * specific, though -- it's a plain level output, reusable for any future
 * diagnostic
 * that needs one. */
void cmd_diag_gpout12(uart_instance_t *inst, char *args)
{
    char *tok;
    long  val;

    tok = (args != NULL) ? strtok(args, " \r\n") : NULL;
    if (tok == NULL)
    {
        SendErr(inst, ERR_INVALID_ARGS, "DIAGnostic:GPOut12 needs one argument: 0|1");
        return;
    }
    val = atol(tok);

    BoardIo_Write(BOARD_SIG_GPOUT_12, (val != 0L) ? 1U : 0U);
    uart_send(inst, "OK\r\n");
}

void cmd_diag_gpout12_query(uart_instance_t *inst, char *args)
{
    char buf[16];
    (void)args;

    snprintf(buf, sizeof(buf), "OK %u\r\n",
             (unsigned)BoardIo_Read(BOARD_SIG_GPOUT_12));
    uart_send(inst, buf);
}

/* --------------------------------------------------------------------------
 * DIAGnostic:GPOut11 <0|1> / DIAGnostic:GPOut11?
 *
 * Added 2026-09-17 -- a SECOND, independent diagnostic output, on PD0
 * ("GPOut_11" in the V4 column, docs/pin_mapping_v4.csv). Direct
 * correction: PD1/DIAGnostic:GPOut12 above was initially reused for
 * testing the PG10 emergency-stop feature too, but PD1 is already the
 * pin dedicated to driving PF15 (external-trigger as of later the same
 * day -- external-enable at the time this was written) -- the user
 * caught this and asked for a genuinely separate pin for PG10 testing,
 * to remove any ambiguity about which diagnostic signal drives which real
 * input. Otherwise an exact mirror of cmd_diag_gpout12()/
 * cmd_diag_gpout12_query() above -- see that pair's own doc comment
 * for the shared reasoning (generic level output, GPIO config in
 * board_io.c, no dedicated module). */
void cmd_diag_gpout11(uart_instance_t *inst, char *args)
{
    char *tok;
    long  val;

    tok = (args != NULL) ? strtok(args, " \r\n") : NULL;
    if (tok == NULL)
    {
        SendErr(inst, ERR_INVALID_ARGS, "DIAGnostic:GPOut11 needs one argument: 0|1");
        return;
    }
    val = atol(tok);

    BoardIo_Write(BOARD_SIG_GPOUT_11, (val != 0L) ? 1U : 0U);
    uart_send(inst, "OK\r\n");
}

void cmd_diag_gpout11_query(uart_instance_t *inst, char *args)
{
    char buf[16];
    (void)args;

    snprintf(buf, sizeof(buf), "OK %u\r\n",
             (unsigned)BoardIo_Read(BOARD_SIG_GPOUT_11));
    uart_send(inst, buf);
}

/* --------------------------------------------------------------------------
 * DIAGnostic:GPOut09 <0|1> / DIAGnostic:GPOut09?
 * DIAGnostic:GPOut10 <0|1> / DIAGnostic:GPOut10?
 *
 * Added 2026-09-18 -- a THIRD and FOURTH diagnostic output, on PG8
 * ("GPOut_09" in the V4 column, docs/pin_mapping_v4.csv) and PG9
 * ("GPOut_10"). Verified directly against the CSV before writing this:
 * the V3 column for these same two rows says "No connection", and a
 * DIFFERENT pair of pins (PD8/PD9) carried the "GPOut_09"/"GPOut_10"
 * names under V3 -- the same class of V3/V4 name-reuse trap as the
 * earlier PC14-vs-PF15 and PF13-vs-PD1 corrections, so this one was
 * checked against the CSV rather than assumed.
 *
 * Immediate use: closes the last gap in the Transrex simulator's fiber-
 * transmitter budget (docs/pin_mapping_reference.tex Section 7) -- these
 * two feed XR1_OCP/XR2_OCP on the controller (GPInput_03/PF4 and
 * GPInput_07/PF8 respectively), completing OCP fault-injection coverage
 * for all 4 channels. Otherwise an exact mirror of
 * cmd_diag_gpout12()/cmd_diag_gpout11() above -- see that pair's own
 * doc comment for the shared reasoning (generic level output, GPIO
 * config in board_io.c, no dedicated module). */
void cmd_diag_gpout09(uart_instance_t *inst, char *args)
{
    char *tok;
    long  val;

    tok = (args != NULL) ? strtok(args, " \r\n") : NULL;
    if (tok == NULL)
    {
        SendErr(inst, ERR_INVALID_ARGS, "DIAGnostic:GPOut09 needs one argument: 0|1");
        return;
    }
    val = atol(tok);

    BoardIo_Write(BOARD_SIG_GPOUT_09, (val != 0L) ? 1U : 0U);
    uart_send(inst, "OK\r\n");
}

void cmd_diag_gpout09_query(uart_instance_t *inst, char *args)
{
    char buf[16];
    (void)args;

    snprintf(buf, sizeof(buf), "OK %u\r\n",
             (unsigned)BoardIo_Read(BOARD_SIG_GPOUT_09));
    uart_send(inst, buf);
}

void cmd_diag_gpout10(uart_instance_t *inst, char *args)
{
    char *tok;
    long  val;

    tok = (args != NULL) ? strtok(args, " \r\n") : NULL;
    if (tok == NULL)
    {
        SendErr(inst, ERR_INVALID_ARGS, "DIAGnostic:GPOut10 needs one argument: 0|1");
        return;
    }
    val = atol(tok);

    BoardIo_Write(BOARD_SIG_GPOUT_10, (val != 0L) ? 1U : 0U);
    uart_send(inst, "OK\r\n");
}

void cmd_diag_gpout10_query(uart_instance_t *inst, char *args)
{
    char buf[16];
    (void)args;

    snprintf(buf, sizeof(buf), "OK %u\r\n",
             (unsigned)BoardIo_Read(BOARD_SIG_GPOUT_10));
    uart_send(inst, buf);
}

/* --------------------------------------------------------------------------
 * GDS?
 *
 * Raw HIGH/LOW snapshot of all 12 GateDriverStatus pins (PE0..PE11,
 * gate_driver.h) -- added 2026-09-08 as a diagnostic while chasing why
 * a fault wasn't being registered. Deliberately a single OK line (one
 * "NN=HIGH" or "NN=LOW" token per pin, NN = 01..12 matching the
 * GateDriverStatus_01..12 silkscreen/schematic numbering, space
 * separated, PE0 first) rather than the sibling PFM-STM32G474
 * project's multi-line/bitmask GDS? formats -- matches this project's
 * existing single-"OK <value>"-line convention (commands.h) instead of
 * introducing a new multi-line reply shape for just this one command.
 * Raw and uncached like the sibling project's GDS?: reads GPIOE->IDR
 * live at the moment of the query, no debounce, no polarity
 * interpretation, no fault-latching of its own -- this command itself
 * is read-only visibility and cannot stop PWM output by itself. (These
 * same 12 pins DO now gate PWM output, via a separate path: the
 * GateDriverStatus EXTI interrupt, gate_driver.h's
 * GateDriver_CheckFault(), added 2026-09-08 -- GDS? is unaffected by
 * and independent of that mechanism, just a raw snapshot either way.)
 * -------------------------------------------------------------------------- */
void cmd_gds_query(uart_instance_t *inst, char *args)
{
    uint16_t mask = GateDriver_Read();
    char buf[160];
    int  len = 0;
    (void)args;

    len += snprintf(&buf[len], sizeof(buf) - (size_t)len, "OK");
    for (uint8_t pin = 0U; pin < 12U; pin++)
    {
        int high = ((mask >> pin) & 1U) != 0U;
        len += snprintf(&buf[len], sizeof(buf) - (size_t)len,
                         " %02u=%s", (unsigned int)(pin + 1U), high ? "HIGH" : "LOW");
    }
    snprintf(&buf[len], sizeof(buf) - (size_t)len, "\r\n");

    uart_send(inst, buf);
}

/* --------------------------------------------------------------------------
 * XREX:CHANnel:STATus? <ch>
 *
 * Added 2026-09-17, per direct request: an XR-labeled diagnostic
 * alongside the existing GDS? (raw GateDriverStatus_01..12 readback,
 * above) -- reports one Transrex channel's own Water/Temp/Enerpro/OCP
 * pins together, by name, rather than needing to remember which of the
 * 16 underlying physical pins corresponds to which signal. Raw levels
 * (HIGH/LOW), polarity-agnostic -- same convention GDS?/EXTernal:INPut?
 * already use; XR_WATER_FLT_POLARITY/etc.
 * (ctrlr_config.h) are what determine which raw level actually means
 * "faulted," not this command. See xrex_io.h's own header comment for
 * the full XR1..XR4 pin-naming design. */
void cmd_xrex_channel_status(uart_instance_t *inst, char *args)
{
    char *tok;
    long  chArg;
    char  buf[80];

    tok = (args != NULL) ? strtok(args, " \r\n") : NULL;
    if (tok == NULL)
    {
        SendErr(inst, ERR_INVALID_ARGS, "XREX:CHANnel:STATus? needs one argument: channel");
        return;
    }
    chArg = atol(tok);

    if ((chArg < 1L) || (chArg > (long)HRTIM_NUM_CHANNELS))
    {
        SendErr(inst, ERR_INVALID_CHANNEL, "Invalid channel");
        return;
    }

    uint8_t water = 0U, tmp = 0U, enerpro = 0U, ocp = 0U;
    (void)XrexIo_GetChannelStatus((uint8_t)(chArg - 1L), &water, &tmp, &enerpro, &ocp);

    snprintf(buf, sizeof(buf), "OK WATER=%s TMP=%s ENERPRO=%s OCP=%s\r\n",
             water ? "HIGH" : "LOW", tmp ? "HIGH" : "LOW",
             enerpro ? "HIGH" : "LOW", ocp ? "HIGH" : "LOW");
    uart_send(inst, buf);
}

/* --------------------------------------------------------------------------
 * XREX:CHANnel:ENAOut <ch> <0|1> / XREX:CHANnel:ENAOut? <ch>
 * XREX:CHANnel:CONTactOut <ch> <0|1> / XREX:CHANnel:CONTactOut? <ch>
 *
 * Added 2026-09-17, per direct request: "Enable and contactor fiber
 * outputs need to be set by a serial command, one for each supply."
 * Real per-channel outputs (PG0-PG3/PG4-PG7, docs/pin_mapping_v4.csv's
 * "XREX Pin Name" column) this firmware itself drives -- owned (pin
 * table, GPIO read/write) by xrex_io.c, matching that module's
 * established per-channel-XR-signal charter. See state_machine.h's own
 * SM_FAULT_ENABLE_OUTPUT/enable-output sections for what reads these:
 * `ARM` refuses unless every currently-enabled channel's own
 * ENA_OUT+CONTACT_OUT are BOTH already HIGH ("The controller cannot be
 * armed unless these are outputting prior to the arm signal"), and this
 * is continuously re-checked once ARMED -- asked directly and
 * confirmed, not assumed.
 *
 * 1-based channel argument, matching this project's universal wire
 * convention -- reuses ERR 11 (invalid channel)/ERR 12 (missing/invalid
 * argument), same as every other numbered-channel command; no new
 * error codes needed for this whole feature. Takes effect immediately;
 * an operator may set/clear either output at any time, including
 * mid-shot -- the continuous poll (XrexIo_PollEnableOutputFaults(),
 * xrex_io.c) is what reacts if that turns out to matter for a
 * currently-enabled channel. */
void cmd_xrex_ena_out(uart_instance_t *inst, char *args)
{
    char *tok;
    long  chArg;
    long  val;

    tok = (args != NULL) ? strtok(args, " \r\n") : NULL;
    if (tok == NULL)
    {
        SendErr(inst, ERR_INVALID_ARGS, "XREX:CHANnel:ENAOut needs two arguments: channel, 0|1");
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
        SendErr(inst, ERR_INVALID_ARGS, "XREX:CHANnel:ENAOut needs two arguments: channel, 0|1");
        return;
    }
    val = atol(tok);

    XrexIo_SetEnableOutput((uint8_t)(chArg - 1L), (val != 0L) ? 1U : 0U);
    uart_send(inst, "OK\r\n");
}

void cmd_xrex_ena_out_query(uart_instance_t *inst, char *args)
{
    char *tok;
    long  chArg;
    char  buf[16];

    tok = (args != NULL) ? strtok(args, " \r\n") : NULL;
    if (tok == NULL)
    {
        SendErr(inst, ERR_INVALID_ARGS, "XREX:CHANnel:ENAOut? needs one argument: channel");
        return;
    }
    chArg = atol(tok);
    if ((chArg < 1L) || (chArg > (long)HRTIM_NUM_CHANNELS))
    {
        SendErr(inst, ERR_INVALID_CHANNEL, "Invalid channel");
        return;
    }

    snprintf(buf, sizeof(buf), "OK %u\r\n", (unsigned)XrexIo_GetEnableOutput((uint8_t)(chArg - 1L)));
    uart_send(inst, buf);
}

void cmd_xrex_contact_out(uart_instance_t *inst, char *args)
{
    char *tok;
    long  chArg;
    long  val;

    tok = (args != NULL) ? strtok(args, " \r\n") : NULL;
    if (tok == NULL)
    {
        SendErr(inst, ERR_INVALID_ARGS, "XREX:CHANnel:CONTactOut needs two arguments: channel, 0|1");
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
        SendErr(inst, ERR_INVALID_ARGS, "XREX:CHANnel:CONTactOut needs two arguments: channel, 0|1");
        return;
    }
    val = atol(tok);

    XrexIo_SetContactorOutput((uint8_t)(chArg - 1L), (val != 0L) ? 1U : 0U);
    uart_send(inst, "OK\r\n");
}

void cmd_xrex_contact_out_query(uart_instance_t *inst, char *args)
{
    char *tok;
    long  chArg;
    char  buf[16];

    tok = (args != NULL) ? strtok(args, " \r\n") : NULL;
    if (tok == NULL)
    {
        SendErr(inst, ERR_INVALID_ARGS, "XREX:CHANnel:CONTactOut? needs one argument: channel");
        return;
    }
    chArg = atol(tok);
    if ((chArg < 1L) || (chArg > (long)HRTIM_NUM_CHANNELS))
    {
        SendErr(inst, ERR_INVALID_CHANNEL, "Invalid channel");
        return;
    }

    snprintf(buf, sizeof(buf), "OK %u\r\n", (unsigned)XrexIo_GetContactorOutput((uint8_t)(chArg - 1L)));
    uart_send(inst, buf);
}

/* --------------------------------------------------------------------------
 * GPOut:ENAble <0|1> / GPOut:ENAble?
 *
 * Added 2026-09-17, per direct request: PC13 ("GPOut_Enable_Pin" in
 * the V4 column, docs/pin_mapping_v4.csv -- confirmed GPO there),
 * default HIGH at boot (board_io.c's BoardIo_Init()), with a serial command
 * to set it either level. A plain, direct HAL_GPIO_WritePin()/
 * ReadPin() pair -- no dedicated module, matching the same "not enough
 * behavior to justify one" precedent DIAGnostic:GPOut11/GPOut12 already
 * established -- own top-level `GPOut:` namespace (not nested under
 * `DIAGnostic:`) since this is a real, specifically-named board signal
 * from the schematic, not a generic scratch diagnostic pin. */
void cmd_gpout_enable(uart_instance_t *inst, char *args)
{
    char *tok;
    long  val;

    tok = (args != NULL) ? strtok(args, " \r\n") : NULL;
    if (tok == NULL)
    {
        SendErr(inst, ERR_INVALID_ARGS, "GPOut:ENAble needs one argument: 0|1");
        return;
    }
    val = atol(tok);

    BoardIo_Write(BOARD_SIG_GPOUT_ENABLE, (val != 0L) ? 1U : 0U);
    uart_send(inst, "OK\r\n");
}

void cmd_gpout_enable_query(uart_instance_t *inst, char *args)
{
    char buf[16];
    (void)args;

    snprintf(buf, sizeof(buf), "OK %u\r\n",
             (unsigned)BoardIo_Read(BOARD_SIG_GPOUT_ENABLE));
    uart_send(inst, buf);
}

/* --------------------------------------------------------------------------
 * PWMAlt:ENAble <0|1> / PWMAlt:ENAble?
 *
 * Added 2026-09-17, per direct request: PC15 ("PWM_Alt_Enable" in the
 * V4 column, docs/pin_mapping_v4.csv -- confirmed GPO there), default
 * HIGH at boot, with a serial command to set it either level -- exact
 * mirror of GPOut:ENAble above in every respect, see that pair's own
 * doc comment for the shared reasoning. */
void cmd_pwmalt_enable(uart_instance_t *inst, char *args)
{
    char *tok;
    long  val;

    tok = (args != NULL) ? strtok(args, " \r\n") : NULL;
    if (tok == NULL)
    {
        SendErr(inst, ERR_INVALID_ARGS, "PWMAlt:ENAble needs one argument: 0|1");
        return;
    }
    val = atol(tok);

    BoardIo_Write(BOARD_SIG_PWMALT_ENABLE, (val != 0L) ? 1U : 0U);
    uart_send(inst, "OK\r\n");
}

void cmd_pwmalt_enable_query(uart_instance_t *inst, char *args)
{
    char buf[16];
    (void)args;

    snprintf(buf, sizeof(buf), "OK %u\r\n",
             (unsigned)BoardIo_Read(BOARD_SIG_PWMALT_ENABLE));
    uart_send(inst, buf);
}
