/*
 * fw_update.h
 *
 * In-application firmware update over the ordinary serial command link
 * (added 2026-09-24). Works through any 8N1 transport -- including the
 * ethernet serial bridge, which cannot carry the ROM bootloader's 8E1.
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
 * Wire protocol (strict stop-and-wait -- uart.c has a single line buffer,
 * so the host must wait for each reply before sending the next line):
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
#ifndef FW_UPDATE_H
#define FW_UPDATE_H

#include "uart.h"

void cmd_fwup_begin(uart_instance_t *inst, char *args);
void cmd_fwup_data(uart_instance_t *inst, char *args);
void cmd_fwup_end(uart_instance_t *inst, char *args);
void cmd_fwup_swap(uart_instance_t *inst, char *args);
void cmd_fwup_rollback(uart_instance_t *inst, char *args);
void cmd_fwup_abort(uart_instance_t *inst, char *args);
void cmd_fwup_status(uart_instance_t *inst, char *args);

#endif /* FW_UPDATE_H */
