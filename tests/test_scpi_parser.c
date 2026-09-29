/*
 * test_scpi_parser.c -- the SCPI matching and dispatch rules
 * (src/middleware/scpi/scpi_parser.c), on the host.
 */
#include "scpi_parser.h"
#include "fake_uart.h"
#include "test_util.h"
#include <string.h>

static char last_args[256];
static int  last_handler;

static void record(int which, char *args)
{
    last_handler = which;
    strcpy(last_args, (args != NULL) ? args : "<null>");
}
static void h_idn(uart_instance_t *i, char *a)      { (void)i; record(1, a); }
static void h_setpoint(uart_instance_t *i, char *a) { (void)i; record(2, a); }
static void h_status_q(uart_instance_t *i, char *a) { (void)i; record(3, a); }
static void h_status(uart_instance_t *i, char *a)   { (void)i; record(4, a); }
static void h_boot(uart_instance_t *i, char *a)     { (void)i; record(5, a); }

static const scpi_command_t table[] = {
    { "*IDN?",              h_idn      },
    { "SOURce:SETpoint",    h_setpoint },
    { "SOURce:STATus?",     h_status_q },
    { "SOURce:STATus",      h_status   },
    { "BOOT",               h_boot     },
};
#define N (sizeof(table) / sizeof(table[0]))

static scpi_result_t run(const char *line)
{
    char buf[256];
    strcpy(buf, line);
    last_handler = 0;
    last_args[0] = '\0';
    return scpi_dispatch(table, N, &uart2, buf);
}

int main(void)
{
    /* Short form, long form, any case */
    CHECK(scpi_match("SOURce:SETpoint", "SOUR:SET"));
    CHECK(scpi_match("SOURce:SETpoint", "SOURCE:SETPOINT"));
    CHECK(scpi_match("SOURce:SETpoint", "source:setpoint"));
    CHECK(scpi_match("SOURce:SETpoint", "Sour:SetPoint"));
    CHECK(scpi_match("SOURce:SETpoint", "SOUR:SETPOINT"));

    /* A partial long form is not a match */
    CHECK(!scpi_match("SOURce:SETpoint", "SOURC:SET"));
    CHECK(!scpi_match("SOURce:SETpoint", "SOUR:SETP"));      /* short form is SET */
    CHECK(!scpi_match("SOURce:SETpoint", "SOU:SET"));

    /* Depth must match */
    CHECK(!scpi_match("SOURce:SETpoint", "SOUR"));
    CHECK(!scpi_match("SOURce:SETpoint", "SOUR:SET:X"));

    /* Query suffix must match both ways */
    CHECK(scpi_match("SOURce:STATus?", "SOUR:STAT?"));
    CHECK(!scpi_match("SOURce:STATus?", "SOUR:STAT"));
    CHECK(!scpi_match("SOURce:STATus", "SOUR:STAT?"));

    /* Leading colon tolerated on input, common commands need no colon */
    CHECK(scpi_match("SOURce:SETpoint", ":SOUR:SET"));
    CHECK(scpi_match("*IDN?", "*idn?"));
    CHECK(!scpi_match("*IDN?", "*IDN"));

    /* Dispatch: arguments are everything after the first space */
    CHECK(run("SOUR:SET 1 20000") == SCPI_HANDLED);
    CHECK(last_handler == 2);
    CHECK_STR(last_args, "1 20000");

    CHECK(run("sour:stat? 3") == SCPI_HANDLED);
    CHECK(last_handler == 3);
    CHECK_STR(last_args, "3");

    CHECK(run("SOURCE:STATUS") == SCPI_HANDLED);
    CHECK(last_handler == 4);
    CHECK_STR(last_args, "<null>");

    CHECK(run("*IDN?\r\n") == SCPI_HANDLED);
    CHECK(last_handler == 1);

    CHECK(run("boot") == SCPI_HANDLED);
    CHECK(last_handler == 5);

    /* Blank and unknown lines run nothing */
    CHECK(run("") == SCPI_EMPTY);
    CHECK(run("   ") == SCPI_EMPTY);
    CHECK(run("\r\n") == SCPI_EMPTY);
    CHECK(run("NOPE:SUCH") == SCPI_UNKNOWN);
    CHECK(run("SOUR:SETPO 1") == SCPI_UNKNOWN);
    CHECK(last_handler == 0);

    /* The parser never replies on its own -- the caller owns ERR 1 */
    fake_uart_clear();
    run("NOPE");
    CHECK_STR(fake_uart_out, "");

    return test_report("test_scpi_parser");
}
