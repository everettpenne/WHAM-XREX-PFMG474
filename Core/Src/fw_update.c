/*
 * fw_update.c -- in-application dual-bank firmware update, see fw_update.h
 * for the mechanism and wire protocol.
 *
 * Bank terminology: "physical" bank 1/2 is what FLASH_CR.BKER and HAL's
 * FLASH_BANK_1/2 refer to; SYSCFG->MEMRMP.FB_MODE decides which physical
 * bank is mapped at 0x08000000 (HAL's own flash_ex.c treats them the same
 * way). The running bank is always at 0x08000000, so the inactive one is
 * always at FWUP_INACTIVE_BASE regardless of swap state -- only the
 * physical bank number used for ERASE has to be derived from FB_MODE.
 * Programming and reading are by address and follow the mapping.
 */
#include "fw_update.h"
#include "main.h"
#include "ctrlr_config.h"
#include "state_machine.h"
#include "boot_diag.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define FWUP_INACTIVE_BASE   (FLASH_BASE + FLASH_BANK_SIZE)
#define FWUP_MAX_CHUNK       48U
#define FWUP_SRAM_TOP        (SRAM_BASE + 0x20000UL)   /* _estack, STM32G474QETX_FLASH.ld */

typedef enum { FWUP_IDLE = 0, FWUP_RECEIVING, FWUP_VERIFIED } fwup_state_t;

static fwup_state_t s_state = FWUP_IDLE;
static uint32_t     s_size;
static uint32_t     s_crc;
static uint32_t     s_next;

static void SendErr(uart_instance_t *inst, int code, const char *msg)
{
    char buf[128];
    snprintf(buf, sizeof(buf), "ERR %d %s\r\n", code, msg);
    uart_send(inst, buf);
}

static uint8_t BanksSwapped(void)
{
    return (READ_BIT(SYSCFG->MEMRMP, SYSCFG_MEMRMP_FB_MODE) != 0U) ? 1U : 0U;
}

static uint32_t InactivePhysicalBank(void)
{
    return (BanksSwapped() != 0U) ? FLASH_BANK_1 : FLASH_BANK_2;
}

static uint8_t Bfb2Set(void)
{
    return (READ_BIT(FLASH->OPTR, FLASH_OPTR_BFB2) != 0U) ? 1U : 0U;
}

/* The inactive bank was just erased/programmed behind the data cache's
   back -- drop any stale lines before reading it for CRC/sanity checks. */
static void FlushDataCache(void)
{
    if (READ_BIT(FLASH->ACR, FLASH_ACR_DCEN) != 0U)
    {
        __HAL_FLASH_DATA_CACHE_DISABLE();
        __HAL_FLASH_DATA_CACHE_RESET();
        __HAL_FLASH_DATA_CACHE_ENABLE();
    }
}

static uint32_t Crc32(const volatile uint8_t *p, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFFUL;
    while (len-- != 0U)
    {
        crc ^= *p++;
        for (uint32_t k = 0U; k < 8U; k++)
        {
            crc = (crc >> 1) ^ (0xEDB88320UL & (0UL - (crc & 1UL)));
        }
    }
    return ~crc;
}

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

/* Returns NULL if the image at `base` looks bootable by THIS build's
   product, else a short reason. Checks what the ROM bootloader's dual-bank
   boot and our own reset path depend on, plus the NUL-terminated product
   name so a simulator image can't be loaded onto the controller (or vice
   versa) -- "WHAM-XREX-PFMG474\0" is not a substring of
   "WHAM-XREX-PFMG474-SIM\0". Cannot prove the code is correct, only that
   it is plausibly the right kind of image. */
static const char *ImageProblem(uint32_t base, uint32_t len)
{
    const volatile uint32_t *vec = (const volatile uint32_t *)base;
    uint32_t sp = vec[0];
    uint32_t rv = vec[1];
    static const char name[] = HW_BOARD_NAME;
    const uint32_t nlen = (uint32_t)sizeof(name);   /* includes the NUL */
    const volatile uint8_t *img = (const volatile uint8_t *)base;

    if ((sp < SRAM_BASE) || (sp > FWUP_SRAM_TOP) || ((sp & 3U) != 0U))
    {
        return "bad initial stack pointer";
    }
    if (((rv & 1U) == 0U) || ((rv & ~1UL) < FLASH_BASE) || ((rv & ~1UL) >= (FLASH_BASE + len)))
    {
        return "bad reset vector";
    }
    for (uint32_t i = 0U; (i + nlen) <= len; i++)
    {
        uint32_t j = 0U;
        while ((j < nlen) && (img[i + j] == (uint8_t)name[j]))
        {
            j++;
        }
        if (j == nlen)
        {
            return NULL;
        }
    }
    return "product name " HW_BOARD_NAME " not found (wrong target image?)";
}

/* Program OPTR.BFB2 so the next boot starts the currently-inactive bank,
   reply, then reload option bytes (a full system reset). Only returns on
   failure. */
static void BootInactiveBank(uart_instance_t *inst)
{
    FLASH_OBProgramInitTypeDef ob;
    char buf[64];
    uint32_t target = (BanksSwapped() != 0U) ? OB_BFB2_DISABLE : OB_BFB2_ENABLE;

    memset(&ob, 0, sizeof(ob));
    ob.OptionType = OPTIONBYTE_USER;
    ob.USERType   = OB_USER_BFB2;
    ob.USERConfig = target;

    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
    HAL_FLASH_OB_Unlock();
    if (HAL_FLASHEx_OBProgram(&ob) != HAL_OK)
    {
        HAL_FLASH_OB_Lock();
        HAL_FLASH_Lock();
        SendErr(inst, 17, "option byte (BFB2) programming failed -- boot bank unchanged");
        return;
    }

    snprintf(buf, sizeof(buf), "OK SWAPPING -- rebooting into bank %u\r\n",
             (unsigned)((BanksSwapped() != 0U) ? 1U : 2U));
    uart_send(inst, buf);
    HAL_Delay(50U);
    BOOT_DIAG_STAGE(BD_STAGE_OB_LAUNCH);
    HAL_FLASH_OB_Launch();   /* system reset; does not return */

    HAL_FLASH_OB_Lock();
    HAL_FLASH_Lock();
    SendErr(inst, 17, "option byte reload did not reset");
}

void cmd_fwup_begin(uart_instance_t *inst, char *args)
{
    char *tokSize = (args != NULL) ? strtok(args, " ") : NULL;
    char *tokCrc  = strtok(NULL, " ");
    uint32_t size;
    uint32_t crc;
    FLASH_EraseInitTypeDef er;
    uint32_t pageErr = 0U;
    char buf[64];

    if (!ParseU32(tokSize, 0, &size) || !ParseU32(tokCrc, 16, &crc))
    {
        SendErr(inst, 12, "usage: FWUPdate:BEGin <size> <crc32hex>");
        return;
    }
    if ((size == 0U) || ((size & 7U) != 0U) || (size > FLASH_BANK_SIZE))
    {
        SendErr(inst, 12, "size must be a nonzero multiple of 8, at most one bank (262144)");
        return;
    }
    if (SM_GetState() != SM_STATE_IDLE)
    {
        SendErr(inst, 13, "firmware update needs STATE IDLE");
        return;
    }
    if (READ_BIT(FLASH->OPTR, FLASH_OPTR_DBANK) == 0U)
    {
        SendErr(inst, 17, "flash is not in dual-bank mode (OPTR.DBANK=0)");
        return;
    }

    s_state = FWUP_IDLE;
    memset(&er, 0, sizeof(er));
    er.TypeErase = FLASH_TYPEERASE_PAGES;
    er.Banks     = InactivePhysicalBank();
    er.Page      = 0U;
    er.NbPages   = (size + FLASH_PAGE_SIZE - 1U) / FLASH_PAGE_SIZE;

    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
    HAL_StatusTypeDef st = HAL_FLASHEx_Erase(&er, &pageErr);
    HAL_FLASH_Lock();
    if (st != HAL_OK)
    {
        snprintf(buf, sizeof(buf), "erase failed at page %lu", (unsigned long)pageErr);
        SendErr(inst, 17, buf);
        return;
    }

    FlushDataCache();
    for (uint32_t off = 0U; off < size; off += 4U)
    {
        if (*(const volatile uint32_t *)(FWUP_INACTIVE_BASE + off) != 0xFFFFFFFFUL)
        {
            snprintf(buf, sizeof(buf), "inactive bank not blank after erase at +0x%05lX",
                     (unsigned long)off);
            SendErr(inst, 17, buf);
            return;
        }
    }

    s_size  = size;
    s_crc   = crc;
    s_next  = 0U;
    s_state = FWUP_RECEIVING;
    snprintf(buf, sizeof(buf), "OK ERASED %lu PAGES BANK %u\r\n",
             (unsigned long)er.NbPages, (unsigned)((er.Banks == FLASH_BANK_1) ? 1U : 2U));
    uart_send(inst, buf);
}

void cmd_fwup_data(uart_instance_t *inst, char *args)
{
    char *tokOff = (args != NULL) ? strtok(args, " ") : NULL;
    char *tokHex = strtok(NULL, " ");
    uint32_t off;
    uint8_t  data[FWUP_MAX_CHUNK];
    uint32_t n;
    char buf[64];

    if (s_state != FWUP_RECEIVING)
    {
        SendErr(inst, 13, "no transfer in progress -- send FWUPdate:BEGin first");
        return;
    }
    if (!ParseU32(tokOff, 16, &off) || (tokHex == NULL))
    {
        SendErr(inst, 12, "usage: FWUPdate:DATA <offsethex> <hexbytes>");
        return;
    }
    if (off != s_next)
    {
        snprintf(buf, sizeof(buf), "expected offset %lX", (unsigned long)s_next);
        SendErr(inst, 12, buf);
        return;
    }
    n = (uint32_t)strlen(tokHex) / 2U;
    if (((strlen(tokHex) & 1U) != 0U) || (n == 0U) || (n > FWUP_MAX_CHUNK) ||
        ((n & 7U) != 0U) || ((s_next + n) > s_size))
    {
        SendErr(inst, 12, "data must be 8..48 bytes, a multiple of 8, within size");
        return;
    }
    for (uint32_t i = 0U; i < n; i++)
    {
        int hi = HexNibble(tokHex[2U * i]);
        int lo = HexNibble(tokHex[(2U * i) + 1U]);
        if ((hi < 0) || (lo < 0))
        {
            SendErr(inst, 12, "bad hex digit");
            return;
        }
        data[i] = (uint8_t)((hi << 4) | lo);
    }

    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
    for (uint32_t i = 0U; i < n; i += 8U)
    {
        uint64_t dw;
        memcpy(&dw, &data[i], sizeof(dw));
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, FWUP_INACTIVE_BASE + s_next + i, dw) != HAL_OK)
        {
            HAL_FLASH_Lock();
            s_state = FWUP_IDLE;
            snprintf(buf, sizeof(buf), "program failed at +0x%05lX -- transfer aborted",
                     (unsigned long)(s_next + i));
            SendErr(inst, 17, buf);
            return;
        }
    }
    HAL_FLASH_Lock();
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
        SendErr(inst, 13, "transfer not complete");
        return;
    }
    FlushDataCache();
    crc = Crc32((const volatile uint8_t *)FWUP_INACTIVE_BASE, s_size);
    if (crc != s_crc)
    {
        s_state = FWUP_IDLE;
        snprintf(buf, sizeof(buf), "CRC mismatch: got %08lX expected %08lX",
                 (unsigned long)crc, (unsigned long)s_crc);
        SendErr(inst, 17, buf);
        return;
    }
    problem = ImageProblem(FWUP_INACTIVE_BASE, s_size);
    if (problem != NULL)
    {
        s_state = FWUP_IDLE;
        SendErr(inst, 17, problem);
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
        SendErr(inst, 13, "no verified image -- complete FWUPdate:END first");
        return;
    }
    if (SM_GetState() != SM_STATE_IDLE)
    {
        SendErr(inst, 13, "firmware update needs STATE IDLE");
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
        SendErr(inst, 13, "transfer in progress -- the other bank is partly written");
        return;
    }
    if (SM_GetState() != SM_STATE_IDLE)
    {
        SendErr(inst, 13, "firmware update needs STATE IDLE");
        return;
    }
    FlushDataCache();
    problem = ImageProblem(FWUP_INACTIVE_BASE, FLASH_BANK_SIZE);
    if (problem != NULL)
    {
        SendErr(inst, 17, problem);
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
             (unsigned)((BanksSwapped() != 0U) ? 2U : 1U),
             (unsigned)Bfb2Set(),
             (unsigned)((READ_BIT(FLASH->OPTR, FLASH_OPTR_DBANK) != 0U) ? 1U : 0U),
             names[s_state],
             (unsigned long)s_next, (unsigned long)s_size);
    uart_send(inst, buf);
}
