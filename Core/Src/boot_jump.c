/*
 * boot_jump.c
 *
 * Software jump into the STM32G474's ROM (System memory) bootloader,
 * driven entirely over the normal USART2 command link -- no physical
 * BOOT0 jumper or NRST access required. Ported verbatim from the
 * sibling PFM-STM32G474 project (pin/board-agnostic mechanism, so it
 * needed no adaptation), with one addition: everything below is gated
 * on BOOT_JUMP_FEATURE_ENABLED (boot_jump.h) so the module is
 * removable from a single line -- see that header for what "removed"
 * means exactly.
 *
 * Mechanism (when enabled)
 * -------------------------
 * A jump straight into the ROM bootloader from deep inside a *running*
 * application (clocks reconfigured, peripherals live) leaves the
 * bootloader unable to bring up its own interfaces -- it never ACKs the
 * AN3155 sync byte. This is the standard two-step pattern that avoids
 * that:
 *
 *   1. BootJump_RequestBootloader() (called from cmd_boot(), after the
 *      "OK ENTERING BOOTLOADER" reply has gone out) writes a magic
 *      sentinel to a RAM word that lives in the .noinit linker section
 *      (STM32G474QETX_FLASH.ld) -- deliberately excluded from the
 *      startup code's .bss zero-fill loop, so it survives a warm
 *      NVIC_SystemReset() (SRAM content is only undefined after a
 *      *power-cycle*, not a reset).
 *
 *   2. BootJump_CheckAndEnter() is called as the literal first statement
 *      in main(), before HAL_Init()/SystemClock_Config()/any peripheral
 *      init. It sees the sentinel, clears it (so a later reset boots the
 *      app normally instead of looping back into the bootloader), and
 *      performs the ROM jump from this clean, just-reset, default-HSI16
 *      state -- which is what lets the bootloader ACK reliably.
 *
 * Bootloader entry address
 * -------------------------
 * RM0440 (STM32G4 series reference manual), Flash memory information
 * block: System memory occupies 0x1FFF0000-0x1FFF6FFF (28 Kbytes) on
 * this device. AN2606 documents the standard jump sequence: remap
 * System memory to 0x0 (so any VTOR-relative access the bootloader
 * makes resolves correctly), load the main stack pointer from word 0 of
 * that region, then branch to the reset vector at word 1.
 */

#include "boot_jump.h"
#include "main.h"

#if (BOOT_JUMP_FEATURE_ENABLED != 0)

#define BOOT_JUMP_MAGIC        (0xB007B007UL)
#define BOOT_JUMP_SYSMEM_ADDR  (0x1FFF0000UL)

/* Reset-surviving request flag. Lives in .noinit (see
 * STM32G474QETX_FLASH.ld) -- NOT zeroed by the startup code, and holds
 * UNDEFINED content after a cold power-up (which is fine: it only ever
 * needs to be recognized after a *warm* reset that BootJump_
 * RequestBootloader() itself triggered). The distinctive 32-bit magic,
 * rather than a plain flag/bool, makes a false-positive match against
 * whatever garbage happens to be in RAM at cold boot vanishingly
 * unlikely. */
__attribute__((section(".noinit")))
static volatile uint32_t s_bootRequest;

void BootJump_RequestBootloader(void)
{
    s_bootRequest = BOOT_JUMP_MAGIC;

    /* NVIC_SystemReset() never returns. Everything the application has
     * configured (GPIO, UART, ...) is torn down by the reset itself --
     * cmd_boot() has already rejected this call in whatever states
     * would make an uncontrolled reset unsafe, so nothing further to
     * guard here. */
    NVIC_SystemReset();
}

void BootJump_CheckAndEnter(void)
{
    if (s_bootRequest != BOOT_JUMP_MAGIC)
    {
        /* Not a bootloader-entry request -- but unconditionally restore
         * the CPU's vector table pointer and the main-Flash memory
         * remap before proceeding to HAL_Init().
         *
         * BUGFIX (confirmed on real hardware, two-part):
         *
         * On a true power-on/NRST reset, SCB->VTOR resets to its
         * hardware default (0x00000000, which resolves to Flash start
         * via SYSCFG's own POR-default remap) -- so both lines below
         * are a harmless no-op there. But the ROM bootloader's "Go"
         * command -- what stm32flash's -g issues after writing a new
         * .bin -- does NOT perform a real chip reset; it just branches
         * here. Two things can be left stale from the bootloader's own
         * session (which needs a working vector table for ITS OWN
         * USART/DMA interrupts while it speaks AN3155):
         *   1. SCB->VTOR may still point at the ROM bootloader's own
         *      vector table. This project never sets VTOR itself --
         *      system_stm32g4xx.c's SystemInit() only does so under
         *      USER_VECT_TAB_ADDRESS, which is NOT defined here -- so
         *      nothing else corrects this on the Go-jump path.
         *   2. If the *previous* boot got here via the jump-INTO-the-
         *      bootloader path below (the normal BOOT-command flow),
         *      SYSCFG->MEMRMP is still set to System memory from that
         *      call, and stays that way across the bootloader's Go.
         * VTOR is the authoritative vector table pointer on this core
         * -- it is not re-derived from the address-0 remap alias on
         * every interrupt, only at reset -- so #1 alone is sufficient
         * to explain the symptom, and MEMRMP was normally already
         * correct by the time this runs. Restoring both anyway costs
         * nothing and removes any doubt: with either stale, EVERY
         * interrupt (including the USART2 RX interrupt uart.c's whole
         * command/response path depends on) vectors into the ROM
         * bootloader's table instead of ours -- the app runs, but is
         * silently deaf to every interrupt-driven peripheral. Symptom
         * observed: *IDN? got no reply immediately after a serial
         * reflash's Go-jump, and only started working after a manual
         * power cycle (a real reset, which fixes both on its own).
         * Fixing it here removes that manual step, which matters a lot
         * for the whole point of this feature: a *remote* reflash with
         * nobody there to power cycle anything. */
        SCB->VTOR = FLASH_BASE;
        __HAL_RCC_SYSCFG_CLK_ENABLE();
        __HAL_SYSCFG_REMAPMEMORY_FLASH();
        return; /* Normal boot -- main() proceeds with HAL_Init() etc. */
    }

    /* Consume the request now, before the jump, so that if the operator
     * later resets the board (power-cycle, NRST, or the bootloader
     * session's own exit) while still in the ROM bootloader, the next
     * boot after that starts the application instead of jumping right
     * back in. */
    s_bootRequest = 0U;

    __disable_irq();

    /* Nothing in main() has run yet at this point (we are called before
     * HAL_Init()), so SysTick should already be in its reset-default
     * disabled state -- cleared here anyway, defensively, since a stray
     * SysTick IRQ firing mid-jump (before __disable_irq() above takes
     * effect at the instruction boundary) would be hard to diagnose. */
    SysTick->CTRL = 0U;

    /* Remap System memory to 0x00000000, per AN2606's documented jump
     * sequence -- required for interrupts/anything VTOR-relative inside
     * the bootloader to resolve against the right vector table. */
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_SYSCFG_REMAPMEMORY_SYSTEMFLASH();

    {
        typedef void (*pFunction)(void);

        volatile uint32_t *sysMem = (volatile uint32_t *)BOOT_JUMP_SYSMEM_ADDR;
        uint32_t   jumpSp = sysMem[0]; /* word 0: initial MSP            */
        pFunction  jumpFn = (pFunction)sysMem[1]; /* word 1: reset vector */

        __set_MSP(jumpSp);
        jumpFn();
    }

    /* Never reached -- jumpFn() does not return. */
    for (;;)
    {
    }
}

#else /* BOOT_JUMP_FEATURE_ENABLED == 0 */

void BootJump_RequestBootloader(void)
{
    /* Feature disabled -- deliberately does nothing. main.c's call
     * site never needs its own #if because of this. */
}

void BootJump_CheckAndEnter(void)
{
    /* Feature disabled -- deliberately does nothing (no sentinel to
     * check, no jump to perform). */
}

#endif /* BOOT_JUMP_FEATURE_ENABLED */
