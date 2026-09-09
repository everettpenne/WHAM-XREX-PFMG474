/*
 * boot_jump.h
 *
 * Software entry into the STM32G474 ROM (System memory) bootloader over
 * the existing USART2 link -- no physical BOOT0/NRST access required.
 * Ported from the sibling PFM-STM32G474 project; see boot_jump.c for
 * the full mechanism.
 *
 * This module is self-contained and removable: BOOT_JUMP_FEATURE_ENABLED
 * below is the single point of control. Set it to 0 (or override it in
 * a build config before this header is first included) and:
 *   - boot_jump.c's real implementation compiles to two empty no-op
 *     functions (see its #else branch) -- no .noinit sentinel logic,
 *     no SYSCFG/ROM-address code.
 *   - commands.c's cmd_boot() and its declaration here both disappear
 *     (both are gated on this same macro).
 *   - cmd_parser.c's "BOOT" table row disappears, so BOOT becomes an
 *     ordinary "ERR 1 Unknown command" like any other unrecognized
 *     mnemonic.
 * main.c's call to BootJump_CheckAndEnter() is intentionally left
 * unconditional either way -- it always resolves to *something*
 * (real jump or no-op), so main.c never needs its own #if for this.
 *
 * STM32G474QETX_FLASH.ld's .noinit section stays defined either way
 * (plain GNU ld scripts have no C-preprocessor conditional here) --
 * but it only reserves space for what's actually placed in it. With
 * this flag off, nothing declares a .noinit variable, so the section
 * is empty (verified: 0 bytes) rather than a fixed cost.
 *
 * Physically deleting boot_jump.c/boot_jump.h instead of disabling
 * them via this flag is NOT supported without also editing main.c and
 * commands.h/commands.c/cmd_parser.c -- this flag is the intended,
 * single-point way to remove the feature.
 */

#ifndef INC_BOOT_JUMP_H_
#define INC_BOOT_JUMP_H_

#ifdef __cplusplus
extern "C" {
#endif

#ifndef BOOT_JUMP_FEATURE_ENABLED
#define BOOT_JUMP_FEATURE_ENABLED (1)
#endif

/* Called from cmd_boot() (commands.c) once the ACK has been sent on the
 * wire. Arms the reset-surviving sentinel and resets the MCU. Never
 * returns. No-op when BOOT_JUMP_FEATURE_ENABLED is 0. */
void BootJump_RequestBootloader(void);

/* Called as the FIRST statement in main(), before HAL_Init(). If the
 * sentinel from BootJump_RequestBootloader() is set, clears it and jumps
 * into the ROM bootloader from this clean, just-reset state; otherwise
 * returns immediately and main() proceeds with normal startup. Always
 * returns immediately when BOOT_JUMP_FEATURE_ENABLED is 0. */
void BootJump_CheckAndEnter(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_BOOT_JUMP_H_ */
