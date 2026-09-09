#!/usr/bin/env python3
"""
wham_serial_flash.py -- one-command serial reflash for WHAM-XREX-PFMG474.

Ported from the sibling PFM-STM32G474 project's pfm_serial_flash.py --
same mechanism, adjusted for this project's build artifact name
(WHAM-XREX-PFMG474.bin). Application baud is 115200 (raised from 9600 on
2026-09-04 -- see AGENTS.md and docs/changelog.txt; WHAM-XREX-PFMG474-only,
the sibling project still uses 9600). The ROM bootloader's own baud
(--flash-baud, default 57600) is unrelated and unaffected.

It performs the full remote-friendly firmware update over the ordinary
USART2 link, with no physical BOOT0 / NRST access required:

  1. Opens the serial port at the application baud (115200 8N1) and sends
     the firmware's `BOOT` command. The running application acknowledges,
     sets a reset-surviving flag, and resets into the STM32 ROM
     bootloader (see Core/Src/boot_jump.c). No-op if that module was
     built with BOOT_JUMP_FEATURE_ENABLED=0 -- BOOT won't be
     acknowledged, and this script will say so and bail out.
  2. Runs `stm32flash` to write and verify the .bin over the same line
     (the ROM bootloader speaks AN3155 at 8E1), then optionally issues a Go
     so the new firmware starts immediately -- again with no reset pin.

If the board is *already* in the bootloader (e.g. you entered it with a
physical BOOT0 jumper), pass --no-boot to skip step 1.

Prerequisites:
  - stm32flash on PATH            (macOS:  brew install stm32flash)
  - pyserial                      (pip install pyserial)

Typical use (from anywhere -- paths below default relative to this
script's own location, in python/, not the current directory):
  python3 python/wham_serial_flash.py                 # auto-detect port, flash Debug/WHAM-XREX-PFMG474.bin
  python3 python/wham_serial_flash.py --bin firmware/WHAM-XREX-PFMG474.bin
  python3 python/wham_serial_flash.py --port /dev/cu.usbserial-130
  python3 python/wham_serial_flash.py --no-boot        # board already in bootloader (BOOT0)
"""

import argparse
import glob
import os
import shutil
import subprocess
import sys
import time

try:
    import serial  # pyserial
except ImportError:
    sys.exit("error: pyserial not installed. Run:  pip install pyserial")

# --- Defaults -------------------------------------------------------------

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.dirname(SCRIPT_DIR)  # this script lives in python/

# Where CubeIDE drops the build artifact. Point --bin elsewhere (e.g. a local
# 'firmware/' folder you copy releases into) if you prefer.
DEFAULT_BIN = os.path.join(PROJECT_DIR, "Debug", "WHAM-XREX-PFMG474.bin")

FLASH_BASE_ADDR = "0x08000000"   # application origin (matches the linker script)
APP_BAUD_DEFAULT = 115200        # firmware command protocol: 115200 8N1
FLASH_BAUD_DEFAULT = 57600       # stm32flash <-> ROM bootloader (auto-bauded, 8E1)

BOOT_ACK_TOKEN = b"BOOTLOADER"   # substring expected in the BOOT reply


def find_default_port():
    """Return the single obvious USB-serial device, or None if ambiguous."""
    candidates = sorted(
        glob.glob("/dev/cu.usbserial*")
        + glob.glob("/dev/tty.usbserial*")
        + glob.glob("/dev/ttyUSB*")
        + glob.glob("/dev/ttyACM*")
    )
    return candidates[0] if len(candidates) == 1 else None


def request_bootloader(port, app_baud):
    """Send the firmware's BOOT command; return True if it acknowledged."""
    print(f"[boot] opening {port} at {app_baud} 8N1 and sending 'BOOT'...")
    try:
        with serial.Serial(port, app_baud, bytesize=serial.EIGHTBITS,
                           parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
                           timeout=1.0) as ser:
            ser.reset_input_buffer()
            ser.write(b"BOOT\r\n")
            ser.flush()
            # Read whatever the app sends back before it resets.
            deadline = time.time() + 2.0
            reply = b""
            while time.time() < deadline:
                chunk = ser.read(64)
                if chunk:
                    reply += chunk
                    if b"\n" in reply:
                        break
            reply = reply.strip()
            if reply:
                print(f"[boot] device replied: {reply!r}")
            else:
                print("[boot] no reply (device may already be in the bootloader,")
                print("       or was built with BOOT_JUMP_FEATURE_ENABLED=0)")
            return BOOT_ACK_TOKEN in reply.upper()
    except serial.SerialException as exc:
        print(f"[boot] serial error: {exc}")
        return False


def run_stm32flash(stm32flash, port, bin_path, flash_baud, go):
    cmd = [stm32flash, "-b", str(flash_baud), "-w", bin_path, "-v"]
    if go:
        cmd += ["-g", FLASH_BASE_ADDR]
    cmd += [port]
    print(f"[flash] {' '.join(cmd)}")
    return subprocess.run(cmd).returncode


def main():
    ap = argparse.ArgumentParser(description="Serial reflash for WHAM-XREX-PFMG474")
    ap.add_argument("--port", help="serial device (auto-detected if omitted)")
    ap.add_argument("--bin", default=DEFAULT_BIN,
                    help=f"firmware .bin to flash (default: {DEFAULT_BIN})")
    ap.add_argument("--app-baud", type=int, default=APP_BAUD_DEFAULT,
                    help=f"application command baud (default: {APP_BAUD_DEFAULT})")
    ap.add_argument("--flash-baud", type=int, default=FLASH_BAUD_DEFAULT,
                    help=f"stm32flash baud (default: {FLASH_BAUD_DEFAULT})")
    ap.add_argument("--no-boot", action="store_true",
                    help="skip the BOOT command (board already in bootloader)")
    ap.add_argument("--no-run", action="store_true",
                    help="do not start the app after flashing (omit the Go)")
    ap.add_argument("--stm32flash", default=shutil.which("stm32flash"),
                    help="path to stm32flash (default: found on PATH)")
    args = ap.parse_args()

    if not args.stm32flash:
        sys.exit("error: stm32flash not found on PATH. macOS: brew install stm32flash")
    if not os.path.isfile(args.bin):
        sys.exit(f"error: firmware not found: {args.bin}\n"
                 f"       build it in STM32CubeIDE or pass --bin <path>")

    port = args.port or find_default_port()
    if not port:
        sys.exit("error: could not auto-detect a serial port; pass --port /dev/cu.usbserial-XXX")

    size = os.path.getsize(args.bin)
    print(f"=== WHAM-XREX-PFMG474 serial reflash ===")
    print(f"    port : {port}")
    print(f"    bin  : {args.bin} ({size} bytes)")
    print()

    if not args.no_boot:
        acked = request_bootloader(port, args.app_baud)
        if not acked:
            print("[boot] proceeding anyway -- if this fails, the app may not be")
            print("       running (or BOOT_JUMP_FEATURE_ENABLED=0); use --no-boot")
            print("       with a physical BOOT0 entry instead.")
        # Give the MCU time to reset and the ROM bootloader to come up.
        time.sleep(1.5)

    rc = run_stm32flash(args.stm32flash, port, args.bin, args.flash_baud,
                        go=not args.no_run)
    if rc == 0:
        print("\n[done] firmware written and verified.")
        if args.no_run:
            print("       (no Go issued -- reset or power-cycle to run it.)")
    else:
        print(f"\n[fail] stm32flash exited with code {rc}.")
        print("       - Is the board in the bootloader? (BOOT command acked, or BOOT0 high)")
        print("       - Try again: entry can miss if the reset wasn't clean.")
    sys.exit(rc)


if __name__ == "__main__":
    main()
