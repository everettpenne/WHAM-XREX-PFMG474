/*
 * flash_bank.h -- dual-bank flash operations behind the in-application
 * firmware update (FWUPdate:*, src/app/commands/cmd_fwupdate.c).
 * Implemented in src/bsp/<chip>/flash_bank.c.
 *
 * Mechanism: this STM32G474 runs in dual-bank mode (OPTR.DBANK=1, 2 x
 * 256 KB). Whichever bank is running is mapped at 0x08000000; the other
 * ("inactive") bank is mapped at 0x08040000 (SYSCFG->MEMRMP.FB_MODE says
 * which physical bank is which). A new image -- the SAME binary that
 * would be flashed at 0x08000000, no relinking -- is streamed into the
 * inactive bank, CRC-checked, sanity-checked, and then booted by setting
 * OPTR.BFB2 so the ROM bootloader's dual-bank boot starts the other bank
 * (with the banks swapped, so it again runs at 0x08000000). The previous
 * image stays intact in the now-inactive bank; FWUPdate:ROLLback boots it
 * again.
 *
 * Offsets below are from the start of the inactive bank.
 */
#ifndef FLASH_BANK_H
#define FLASH_BANK_H

#include <stdint.h>

/* Bank size in bytes (the most an image can be). */
uint32_t FlashBank_Size(void);

/* Physical bank (1 or 2) currently running at 0x08000000. */
uint8_t FlashBank_Active(void);

/* 1 if OPTR.BFB2 is set (boot from bank 2). */
uint8_t FlashBank_Bfb2(void);

/* 1 if the flash is in dual-bank mode (OPTR.DBANK). */
uint8_t FlashBank_DualBank(void);

typedef enum
{
    FLASH_BANK_OK = 0,
    FLASH_BANK_ERASE_FAILED,   /* *detail = failing page */
    FLASH_BANK_NOT_BLANK,      /* *detail = first non-blank offset */
    FLASH_BANK_PROGRAM_FAILED, /* *detail = failing offset */
    FLASH_BANK_OB_FAILED       /* option-byte programming failed */
} flash_bank_status_t;

/* Erase enough pages of the inactive bank for `size` bytes, then check they
   read back blank. On success *pages = pages erased, *physBank = the
   physical bank (1/2) erased. */
flash_bank_status_t FlashBank_EraseInactive(uint32_t size, uint32_t *pages, uint8_t *physBank,
                                            uint32_t *detail);

/* Program n bytes (a multiple of 8) at `offset`. */
flash_bank_status_t FlashBank_ProgramInactive(uint32_t offset, const uint8_t *data, uint32_t n,
                                              uint32_t *detail);

/* CRC-32 (zlib) of the first len bytes of the inactive bank. */
uint32_t FlashBank_InactiveCrc32(uint32_t len);

/* NULL if the first len bytes of the inactive bank look like a bootable
   image of THIS product (stack pointer, reset vector, product name), else
   a short reason. Cannot prove the code correct, only plausibly the right
   kind of image. */
const char *FlashBank_InactiveImageProblem(uint32_t len);

/* Boot the inactive bank next: program OPTR.BFB2 accordingly. On success
   the flash and option bytes are left unlocked for
   FlashBank_ReloadOptionBytes(); on failure they are locked again. */
flash_bank_status_t FlashBank_SelectInactiveForBoot(void);

/* Reload option bytes -- a full system reset. Returns only if the reset
   did not happen (flash and option bytes are then locked again). */
void FlashBank_ReloadOptionBytes(void);

#endif /* FLASH_BANK_H */
