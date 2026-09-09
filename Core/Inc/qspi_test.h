#ifndef INC_QSPI_TEST_H_
#define INC_QSPI_TEST_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*
 * qspi_test.h
 *
 * Bring-up connectivity test for the 6-pin QUADSPI bus this board wires
 * out (PE12-PE15 = BK1_IO0-3, PB10 = CLK, PB11 = BK1_NCS -- all AF10,
 * docs/pin_mapping_v4.csv) to a Winbond W25Q128JVS NOR flash, added
 * 2026-09-08 in response to a direct request for an easily removable
 * test module -- NOT a general QUADSPI driver or a flash filesystem/
 * storage layer. Scope, per direct confirmation: one command, one
 * question -- "is the chip there and talking" -- via the W25Q128's
 * standard JEDEC Read ID instruction (0x9F), issued in plain 1-line
 * (single-SPI-compatible) mode. No quad-mode reads, no program/erase,
 * no memory-mapped mode. Extend deliberately, not by accident, if this
 * ever needs to grow past that.
 *
 * This module is self-contained and removable: QSPI_TEST_FEATURE_ENABLED
 * below is the single point of control, matching boot_jump.h's own
 * "self-contained and removable" pattern exactly (see that header for
 * the fuller explanation of what "removable" means here). Set it to 0
 * (or override it in a build config before this header is first
 * included) and:
 *   - qspi_test.c's real implementation (QUADSPI init, GPIO AF10
 *     config, HAL_QSPI_MspInit/MspDeInit, the actual Read ID
 *     transaction) compiles to two empty no-op functions (see its
 *     #else branch).
 *   - commands.c's cmd_qspi_id() and its declaration here both
 *     disappear (both gated on this same macro), same as
 *     BOOT_JUMP_FEATURE_ENABLED gates cmd_boot().
 *   - cmd_parser.c's "QSPI:ID?" table row disappears, so QSPI:ID?
 *     becomes an ordinary "ERR 1 Unknown command" like any other
 *     unrecognized mnemonic.
 * main.c's call to QspiTest_Init() is intentionally left unconditional
 * either way -- it always resolves to *something* (real init or
 * no-op), so main.c never needs its own #if for this.
 *
 * What this flag does NOT remove: HAL_QSPI_MODULE_ENABLED
 * (stm32g4xx_hal_conf.h) and the stm32g4xx_hal_qspi.c/.h files it pulls
 * in stay compiled in either way -- an inert, unused object when this
 * flag is 0, same as every other always-on HAL module in this project
 * (UART, GPIO, HRTIM, ...) has no per-feature removability flag of its
 * own. Matches boot_jump.h's own stated definition of "removable":
 * this flag is the intended, single-point way to remove the FEATURE,
 * not a instruction to physically delete every file it touches.
 */

#ifndef QSPI_TEST_FEATURE_ENABLED
#define QSPI_TEST_FEATURE_ENABLED (1)
#endif

/* Called once at boot (main.c, after MX_GPIO_Init()/MX_HRTIM1_Init(),
 * alongside this project's other peripheral bring-up) -- configures
 * PE12-PE15/PB10-PB11 for QUADSPI AF10 and initializes the QUADSPI
 * peripheral itself (HAL_QSPI_Init()). Does not talk to the flash chip
 * itself; that's QspiTest_ReadId(). No-op when
 * QSPI_TEST_FEATURE_ENABLED is 0. */
void QspiTest_Init(void);

/* Issues the W25Q128JVS's standard JEDEC Read ID instruction (0x9F, 1
 * instruction line / 1 data line, no address, no dummy cycles) and
 * reads back the 3-byte ID (Manufacturer, Memory Type, Capacity) into
 * `id`, which must point to at least 3 bytes. Returns 1 on success, 0
 * on any HAL_QSPI error or timeout (id[] contents are undefined in
 * that case). Blocking/polling -- no DMA, no interrupts, matching this
 * being a one-shot diagnostic triggered from a serial command, not a
 * throughput-sensitive transfer. Always returns 0 (and leaves id[]
 * untouched) when QSPI_TEST_FEATURE_ENABLED is 0. */
uint8_t QspiTest_ReadId(uint8_t id[3]);

#ifdef __cplusplus
}
#endif

#endif /* INC_QSPI_TEST_H_ */
