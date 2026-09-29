/*
 * cmd_fwupdate.c
 *
 * FWUPdate:* -- in-application firmware update over the ordinary serial
 * command link (added 2026-09-24). Works through any 8N1 transport --
 * including the ethernet serial bridge, which cannot carry the ROM
 * bootloader's 8E1. The flash mechanism (dual-bank, BFB2 boot) is
 * described in drivers/flash_bank.h; this file is the transfer protocol.
 *
 * Wire protocol (strict stop-and-wait -- the serial link has a single line
 * buffer, so the host must wait for each reply before sending the next
 * line):
 *   FWUPdate:BEGin <size> <crc32hex>   erase inactive bank (needs IDLE)
 *   FWUPdate:DATA <offsethex> <hex>    program 8..48 bytes, offsets in order
 *   FWUPdate:END                       CRC + image sanity check
 *   FWUPdate:SWAP                      set BFB2 to boot the new image, reset
 *   FWUPdate:ROLLback                  boot the other bank's existing image
 *   FWUPdate:ABORt                     forget a transfer in progress
 *   FWUPdate:STATus?                   bank/BFB2/transfer state
 * <size> is the image length padded to a multiple of 8 with 0xFF; the CRC
 * is standard CRC-32 (zlib.crc32) over those padded bytes.
 */
#include "commands.h"
#include "cmd_common.h"
#include "flash_bank.h"
#include "state_machine.h"
#include "boot_diag.h"
#include "mcu.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define FWUP_MAX_CHUNK       48U

typedef enum { FWUP_IDLE = 0, FWUP_RECEIVING, FWUP_VERIFIED } fwup_state_t;

static fwup_state_t s_state = FWUP_IDLE;
static uint32_t     s_size;
static uint32_t     s_crc;
static uint32_t     s_next;

static int HexNibble(char c)
{
    if ((c >= '0') && (c <= '9')) { return c - '0'; }
    if ((c >= 'a') && (c <= 'f')) { return c - 'a' + 10; }
    if ((c >= 'A') && (c <= 'F')) { return c - 'A' + 10; }
    return -1;
}

static int ParseU32(const char *tok, int base, uint32_t *out)
{
    char *end;
    if (tok == NULL) { return 0; }
    *out = (uint32_t)strtoul(tok, &end, base);
    return (end != tok) && (*end == '\0');
}

/* Program OPTR.BFB2 so the next boot starts the currently-inactive bank,
   reply, then reload option bytes (a full system reset). Only returns on
   failure. */
static void BootInactiveBank(uart_instance_t *inst)
{
    char buf[64];

    if (FlashBank_SelectInactiveForBoot() != FLASH_BANK_OK)
    {
        SendErr(inst, ERR_FWUPDATE, "option byte (BFB2) programming failed -- boot bank unchanged");
        return;
    }

    snprintf(buf, sizeof(buf), "OK SWAPPING -- rebooting into bank %u\r\n",
             (unsigned)((FlashBank_Active() == 2U) ? 1U : 2U));
    uart_send(inst, buf);
    Mcu_DelayMs(50U);
    BOOT_DIAG_STAGE(BD_STAGE_OB_LAUNCH);
    FlashBank_ReloadOptionBytes();   /* system reset; does not return */

    SendErr(inst, ERR_FWUPDATE, "option byte reload did not reset");
}

void cmd_fwup_begin(uart_instance_t *inst, char *args)
{
    char *tokSize = (args != NULL) ? strtok(args, " ") : NULL;
    char *tokCrc  = strtok(NULL, " ");
    uint32_t size;
    uint32_t crc;
    uint32_t pages = 0U;
    uint8_t  bank = 0U;
    uint32_t detail = 0U;
    char buf[64];

    if (!ParseU32(tokSize, 0, &size) || !ParseU32(tokCrc, 16, &crc))
    {
        SendErr(inst, ERR_INVALID_ARGS, "usage: FWUPdate:BEGin <size> <crc32hex>");
        return;
    }
    if ((size == 0U) || ((size & 7U) != 0U) || (size > FlashBank_Size()))
    {
        SendErr(inst, ERR_INVALID_ARGS, "size must be a nonzero multiple of 8, at most one bank (262144)");
        return;
    }
    if (SM_GetState() != SM_STATE_IDLE)
    {
        SendErr(inst, ERR_INVALID_STATE, "firmware update needs STATE IDLE");
        return;
    }
    if (FlashBank_DualBank() == 0U)
    {
        SendErr(inst, ERR_FWUPDATE, "flash is not in dual-bank mode (OPTR.DBANK=0)");
        return;
    }

    s_state = FWUP_IDLE;
    switch (FlashBank_EraseInactive(size, &pages, &bank, &detail))
    {
        case FLASH_BANK_OK:
            break;
        case FLASH_BANK_NOT_BLANK:
            snprintf(buf, sizeof(buf), "inactive bank not blank after erase at +0x%05lX",
                     (unsigned long)detail);
            SendErr(inst, ERR_FWUPDATE, buf);
            return;
        default:
            snprintf(buf, sizeof(buf), "erase failed at page %lu", (unsigned long)detail);
            SendErr(inst, ERR_FWUPDATE, buf);
            return;
    }

    s_size  = size;
    s_crc   = crc;
    s_next  = 0U;
    s_state = FWUP_RECEIVING;
    snprintf(buf, sizeof(buf), "OK ERASED %lu PAGES BANK %u\r\n",
             (unsigned long)pages, (unsigned)bank);
    uart_send(inst, buf);
}

void cmd_fwup_data(uart_instance_t *inst, char *args)
{
    char *tokOff = (args != NULL) ? strtok(args, " ") : NULL;
    char *tokHex = strtok(NULL, " ");
    uint32_t off;
    uint8_t  data[FWUP_MAX_CHUNK];
    uint32_t n;
    uint32_t failOff = 0U;
    char buf[64];

    if (s_state != FWUP_RECEIVING)
    {
        SendErr(inst, ERR_INVALID_STATE, "no transfer in progress -- send FWUPdate:BEGin first");
        return;
    }
    if (!ParseU32(tokOff, 16, &off) || (tokHex == NULL))
    {
        SendErr(inst, ERR_INVALID_ARGS, "usage: FWUPdate:DATA <offsethex> <hexbytes>");
        return;
    }
    if (off != s_next)
    {
        snprintf(buf, sizeof(buf), "expected offset %lX", (unsigned long)s_next);
        SendErr(inst, ERR_INVALID_ARGS, buf);
        return;
    }
    n = (uint32_t)strlen(tokHex) / 2U;
    if (((strlen(tokHex) & 1U) != 0U) || (n == 0U) || (n > FWUP_MAX_CHUNK) ||
        ((n & 7U) != 0U) || ((s_next + n) > s_size))
    {
        SendErr(inst, ERR_INVALID_ARGS, "data must be 8..48 bytes, a multiple of 8, within size");
        return;
    }
    for (uint32_t i = 0U; i < n; i++)
    {
        int hi = HexNibble(tokHex[2U * i]);
        int lo = HexNibble(tokHex[(2U * i) + 1U]);
        if ((hi < 0) || (lo < 0))
        {
            SendErr(inst, ERR_INVALID_ARGS, "bad hex digit");
            return;
        }
        data[i] = (uint8_t)((hi << 4) | lo);
    }

    if (FlashBank_ProgramInactive(s_next, data, n, &failOff) != FLASH_BANK_OK)
    {
        s_state = FWUP_IDLE;
        snprintf(buf, sizeof(buf), "program failed at +0x%05lX -- transfer aborted",
                 (unsigned long)failOff);
        SendErr(inst, ERR_FWUPDATE, buf);
        return;
    }
    s_next += n;
    uart_send(inst, "OK\r\n");
}

void cmd_fwup_end(uart_instance_t *inst, char *args)
{
    char buf[80];
    const char *problem;
    uint32_t crc;
    (void)args;

    if ((s_state != FWUP_RECEIVING) || (s_next != s_size))
    {
        SendErr(inst, ERR_INVALID_STATE, "transfer not complete");
        return;
    }
    crc = FlashBank_InactiveCrc32(s_size);
    if (crc != s_crc)
    {
        s_state = FWUP_IDLE;
        snprintf(buf, sizeof(buf), "CRC mismatch: got %08lX expected %08lX",
                 (unsigned long)crc, (unsigned long)s_crc);
        SendErr(inst, ERR_FWUPDATE, buf);
        return;
    }
    problem = FlashBank_InactiveImageProblem(s_size);
    if (problem != NULL)
    {
        s_state = FWUP_IDLE;
        SendErr(inst, ERR_FWUPDATE, problem);
        return;
    }
    s_state = FWUP_VERIFIED;
    snprintf(buf, sizeof(buf), "OK VERIFIED CRC=%08lX\r\n", (unsigned long)crc);
    uart_send(inst, buf);
}

void cmd_fwup_swap(uart_instance_t *inst, char *args)
{
    (void)args;
    if (s_state != FWUP_VERIFIED)
    {
        SendErr(inst, ERR_INVALID_STATE, "no verified image -- complete FWUPdate:END first");
        return;
    }
    if (SM_GetState() != SM_STATE_IDLE)
    {
        SendErr(inst, ERR_INVALID_STATE, "firmware update needs STATE IDLE");
        return;
    }
    BootInactiveBank(inst);
}

void cmd_fwup_rollback(uart_instance_t *inst, char *args)
{
    const char *problem;
    (void)args;

    if (s_state == FWUP_RECEIVING)
    {
        SendErr(inst, ERR_INVALID_STATE, "transfer in progress -- the other bank is partly written");
        return;
    }
    if (SM_GetState() != SM_STATE_IDLE)
    {
        SendErr(inst, ERR_INVALID_STATE, "firmware update needs STATE IDLE");
        return;
    }
    problem = FlashBank_InactiveImageProblem(FlashBank_Size());
    if (problem != NULL)
    {
        SendErr(inst, ERR_FWUPDATE, problem);
        return;
    }
    BootInactiveBank(inst);
}

void cmd_fwup_abort(uart_instance_t *inst, char *args)
{
    (void)args;
    s_state = FWUP_IDLE;
    uart_send(inst, "OK\r\n");
}

void cmd_fwup_status(uart_instance_t *inst, char *args)
{
    static const char *const names[] = { "IDLE", "RECEIVING", "VERIFIED" };
    char buf[96];
    (void)args;

    snprintf(buf, sizeof(buf), "OK BANK=%u BFB2=%u DBANK=%u STATE=%s RX=%lu/%lu\r\n",
             (unsigned)FlashBank_Active(),
             (unsigned)FlashBank_Bfb2(),
             (unsigned)FlashBank_DualBank(),
             names[s_state],
             (unsigned long)s_next, (unsigned long)s_size);
    uart_send(inst, buf);
}
