/*
 * flash_bank.c -- STM32G4 implementation of drivers/flash_bank.h.
 *
 * Bank terminology: "physical" bank 1/2 is what FLASH_CR.BKER and HAL's
 * FLASH_BANK_1/2 refer to; SYSCFG->MEMRMP.FB_MODE decides which physical
 * bank is mapped at 0x08000000 (HAL's own flash_ex.c treats them the same
 * way). The running bank is always at 0x08000000, so the inactive one is
 * always at INACTIVE_BASE regardless of swap state -- only the physical
 * bank number used for ERASE has to be derived from FB_MODE. Programming
 * and reading are by address and follow the mapping.
 */
#include "flash_bank.h"
#include "main.h"
#include "ctrlr_config.h"
#include <string.h>

#define INACTIVE_BASE   (FLASH_BASE + FLASH_BANK_SIZE)
#define SRAM_TOP        (SRAM_BASE + 0x20000UL)   /* _estack, STM32G474QETX_FLASH.ld */

static uint8_t BanksSwapped(void)
{
    return (READ_BIT(SYSCFG->MEMRMP, SYSCFG_MEMRMP_FB_MODE) != 0U) ? 1U : 0U;
}

static uint32_t InactivePhysicalBank(void)
{
    return (BanksSwapped() != 0U) ? FLASH_BANK_1 : FLASH_BANK_2;
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

uint32_t FlashBank_Size(void)
{
    return FLASH_BANK_SIZE;
}

uint8_t FlashBank_Active(void)
{
    return (BanksSwapped() != 0U) ? 2U : 1U;
}

uint8_t FlashBank_Bfb2(void)
{
    return (READ_BIT(FLASH->OPTR, FLASH_OPTR_BFB2) != 0U) ? 1U : 0U;
}

uint8_t FlashBank_DualBank(void)
{
    return (READ_BIT(FLASH->OPTR, FLASH_OPTR_DBANK) != 0U) ? 1U : 0U;
}

flash_bank_status_t FlashBank_EraseInactive(uint32_t size, uint32_t *pages, uint8_t *physBank,
                                            uint32_t *detail)
{
    FLASH_EraseInitTypeDef er;
    uint32_t pageErr = 0U;

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
        *detail = pageErr;
        return FLASH_BANK_ERASE_FAILED;
    }

    FlushDataCache();
    for (uint32_t off = 0U; off < size; off += 4U)
    {
        if (*(const volatile uint32_t *)(INACTIVE_BASE + off) != 0xFFFFFFFFUL)
        {
            *detail = off;
            return FLASH_BANK_NOT_BLANK;
        }
    }

    *pages    = er.NbPages;
    *physBank = (er.Banks == FLASH_BANK_1) ? 1U : 2U;
    return FLASH_BANK_OK;
}

flash_bank_status_t FlashBank_ProgramInactive(uint32_t offset, const uint8_t *data, uint32_t n,
                                              uint32_t *detail)
{
    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
    for (uint32_t i = 0U; i < n; i += 8U)
    {
        uint64_t dw;
        memcpy(&dw, &data[i], sizeof(dw));
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, INACTIVE_BASE + offset + i, dw) != HAL_OK)
        {
            HAL_FLASH_Lock();
            *detail = offset + i;
            return FLASH_BANK_PROGRAM_FAILED;
        }
    }
    HAL_FLASH_Lock();
    return FLASH_BANK_OK;
}

uint32_t FlashBank_InactiveCrc32(uint32_t len)
{
    const volatile uint8_t *p = (const volatile uint8_t *)INACTIVE_BASE;
    uint32_t crc = 0xFFFFFFFFUL;

    FlushDataCache();
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

/* Checks what the ROM bootloader's dual-bank boot and our own reset path
   depend on, plus the NUL-terminated product name so a simulator image
   can't be loaded onto the controller (or vice versa) --
   "WHAM-XREX-PFMG474\0" is not a substring of "WHAM-XREX-PFMG474-SIM\0". */
const char *FlashBank_InactiveImageProblem(uint32_t len)
{
    const volatile uint32_t *vec = (const volatile uint32_t *)INACTIVE_BASE;
    static const char name[] = HW_BOARD_NAME;
    const uint32_t nlen = (uint32_t)sizeof(name);   /* includes the NUL */
    const volatile uint8_t *img = (const volatile uint8_t *)INACTIVE_BASE;

    FlushDataCache();
    uint32_t sp = vec[0];
    uint32_t rv = vec[1];

    if ((sp < SRAM_BASE) || (sp > SRAM_TOP) || ((sp & 3U) != 0U))
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

flash_bank_status_t FlashBank_SelectInactiveForBoot(void)
{
    FLASH_OBProgramInitTypeDef ob;

    memset(&ob, 0, sizeof(ob));
    ob.OptionType = OPTIONBYTE_USER;
    ob.USERType   = OB_USER_BFB2;
    ob.USERConfig = (BanksSwapped() != 0U) ? OB_BFB2_DISABLE : OB_BFB2_ENABLE;

    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
    HAL_FLASH_OB_Unlock();
    if (HAL_FLASHEx_OBProgram(&ob) != HAL_OK)
    {
        HAL_FLASH_OB_Lock();
        HAL_FLASH_Lock();
        return FLASH_BANK_OB_FAILED;
    }
    return FLASH_BANK_OK;
}

void FlashBank_ReloadOptionBytes(void)
{
    HAL_FLASH_OB_Launch();   /* system reset; does not return */

    HAL_FLASH_OB_Lock();
    HAL_FLASH_Lock();
}
