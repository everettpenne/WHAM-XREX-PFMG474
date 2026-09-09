# WHAM-XREX-PFMG474 — Serial Reflash Guide

How to re-flash the controller over its USART2 serial link using the
open-source `stm32flash`, with **no** STM32CubeIDE, ST-Link, or physical
BOOT0/NRST access involved in the flashing step itself. Ported from the
sibling PFM-STM32G474 project's guide, adjusted for this project's baud
(115200, raised from 9600 on 2026-09-04 — see `docs/changelog.txt`) and
build artifact name (`WHAM-XREX-PFMG474.bin`), and with a real bugfix
history specific to this project (see "The Go-jump bug" below).

**Verified working end-to-end on real hardware** (2026-08-31): flash a
new firmware version, query `*IDN?` immediately afterward with no manual
reset, get the new version back — confirmed twice, back to back.

## 1. Prerequisites (one-time)

```bash
brew install stm32flash        # the flasher (macOS/Homebrew)
pip install pyserial           # used by wham_serial_flash.py
```

- `stm32flash` implements ST's AN3155 USART bootloader protocol.
- The ROM bootloader always uses **8E1** (even parity) at an auto-bauded
  rate (57600 by default here); `stm32flash` handles this — you don't
  configure it, and it's independent of the *application's* 115200 8N1.
- USART2 wiring: adapter **TX → PA3** (USART2_RX), adapter **RX → PA2**
  (USART2_TX), **GND ↔ GND**.

## 2. The easy path — `wham_serial_flash.py`

From the project directory, with the board powered and running current
firmware:

```bash
python3 python/wham_serial_flash.py --port /dev/cu.usbserial-XXXXX
```

(Auto-detection of the port is intentionally conservative: on macOS a
single USB-serial adapter shows up as both a `cu.*` and `tty.*` device
node, so the script sees 2 candidates and refuses to guess rather than
risk the wrong one. Pass `--port` explicitly. Prefer the `cu.*` node.)

That will:
1. Send `BOOT` at 115200 8N1; the app replies `OK ENTERING BOOTLOADER` and
   resets into the ROM bootloader.
2. Run `stm32flash` to write + verify `Debug/WHAM-XREX-PFMG474.bin`.
3. Issue a Go so the new firmware starts immediately (no reset pin
   needed) — see the bugfix note below for why this is now reliable.

Useful options:

```bash
python3 python/wham_serial_flash.py --bin firmware/WHAM-XREX-PFMG474.bin --port ...
python3 python/wham_serial_flash.py --no-boot --port ...   # board already in bootloader (BOOT0)
python3 python/wham_serial_flash.py --no-run --port ...    # leave it in the bootloader after flashing
```

By default it flashes `Debug/WHAM-XREX-PFMG474.bin`. **This project's
`.cproject` does not have the "Convert to binary file" post-build step
enabled**, so that `.bin` doesn't appear automatically from a normal
build — generate it by hand after building:

```bash
arm-none-eabi-objcopy -O binary Debug/WHAM-XREX-PFMG474.elf Debug/WHAM-XREX-PFMG474.bin
```

## 3. The `BOOT` command (mechanism)

`BOOT` is a normal command in the USART2 protocol, alongside `*IDN?`.
Sending `BOOT\r\n`:

1. The handler ([`cmd_boot`](../Core/Src/commands.c)) replies
   `OK ENTERING BOOTLOADER`.
2. [`BootJump_RequestBootloader`](../Core/Src/boot_jump.c) writes a
   sentinel to a reset-surviving RAM word (a `.noinit` linker section
   that the startup code does not zero) and triggers a real chip reset
   (`NVIC_SystemReset()`).
3. On the next boot, `BootJump_CheckAndEnter()` — the **first**
   statement in `main()`, before `HAL_Init()` — sees the sentinel and
   jumps into the ROM bootloader from that clean, just-reset state.

Doing the jump immediately after a real reset (not from deep inside a
running application, with clocks/peripherals already configured) is
what makes the ROM bootloader able to answer — this was diagnosed during
the sibling project's own bring-up and carried forward here unchanged.

## 4. The Go-jump bug (found and fixed in this project)

The mechanism above covers *entering* the bootloader reliably. *Leaving*
it — via `stm32flash -g`, which issues the ROM bootloader's own `Go`
command after writing the new image — is a different story, and turned
out to have a real bug:

**`Go` is not a chip reset.** It branches to the given address, but does
not reset peripherals or core registers. If the *previous* boot reached
the bootloader via the normal `BOOT`-command path above,
`SCB->VTOR` (the CPU's vector table pointer) and `SYSCFG->MEMRMP` (the
address-0 memory remap) can both be left pointing at the ROM
bootloader's own vector table — because the bootloader needs a working
vector table for *its own* USART/DMA interrupts while it speaks AN3155,
and this firmware never explicitly sets `SCB->VTOR` itself
(`USER_VECT_TAB_ADDRESS` is undefined in `system_stm32g4xx.c`, the
CubeMX default).

**Symptom observed on real hardware**: after a `wham_serial_flash.py`
run reported success (write + verify both passed), the new firmware was
completely unresponsive over serial — `*IDN?` got no reply — until the
board was power-cycled, at which point it worked immediately and
correctly reported the new version. The flash itself was never the
problem; the *running* firmware was deaf to every interrupt-driven
peripheral, including the USART2 RX interrupt the whole command parser
depends on, because interrupts were still vectoring into the ROM
bootloader's table.

**Fix**: `BootJump_CheckAndEnter()`'s normal-boot path now
unconditionally restores both `SCB->VTOR` and `SYSCFG->MEMRMP` before
falling through to `HAL_Init()`. On a true power-on/NRST reset this is a
harmless no-op (both are already at their hardware-default values,
which are the correct ones). On the `Go`-jump path, it's what makes the
new firmware come up already working. See `Core/Src/boot_jump.c`'s doc
comment on that function for the full explanation, and
`docs/changelog.txt` for the debugging history.

This bug (and fix) is specific to this project as of this writing — the
sibling PFM-STM32G474/-V4 projects carry the same unfixed `boot_jump.c`
they were ported from, untested on real hardware for this exact failure
mode. Not yet ported back; see `AGENTS.md`'s "Known gaps."

## 5. Manual `stm32flash` (no script)

### After a `BOOT` command
Send `BOOT` (via the script's step above, `scpi.py`, or any terminal),
wait ~1.5 s, then:

```bash
stm32flash -w Debug/WHAM-XREX-PFMG474.bin -v -g 0x08000000 /dev/cu.usbserial-XXXXX
```

- `-w` write, `-v` verify, `-g 0x08000000` start the app afterward.
  (`0x08000000` — the literal Flash base address, not `0x0` — is
  deliberate: if the memory remap were ever left pointing somewhere
  else, `-g 0x0` would jump through *that* alias instead of the real
  application. `0x08000000` always means physical Flash regardless of
  remap state.)

### Hardware BOOT0 (bench fallback)
1. Hold **BOOT0 high**, pulse **NRST** (or power-cycle) — a *clean*
   reset matters here too.
2. Verify the link (prints chip info, no write):
   ```bash
   stm32flash /dev/cu.usbserial-XXXXX
   ```
3. Flash as above.
4. Set **BOOT0 low** for normal operation.

## 6. Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `Failed to init device, timeout.` on `stm32flash`'s own probe (before any write) | Not a firmware issue — this happens independent of what's running on the chip. Check power, wiring (TX/RX not crossed, common GND), and that nothing else has the port open. |
| `BOOT` gets no reply, but the physical-layer checks above are fine | The *currently running* firmware doesn't recognize `BOOT` — either it predates `boot_jump.c` (needs one bootstrap flash via ST-Link/CubeIDE first) or it was itself flashed via a `Go`-jump and never reset (see Section 4 — power-cycle it once to recover, then retry). |
| Flash reports success, but the new firmware doesn't respond afterward | Should be fixed as of the Go-jump bugfix (Section 4). If you still see this, the fix may have regressed — check `boot_jump.c`'s `BootJump_CheckAndEnter()` for the `SCB->VTOR`/`__HAL_SYSCFG_REMAPMEMORY_FLASH()` calls. |
| Works once, fails on retry | The bootloader locks its baud on the first sync byte; re-enter it (fresh `BOOT` or reset) before each `stm32flash` run. |
