#!/usr/bin/env python3
"""net_flash.py -- flash WHAM-XREX-PFMG474 firmware over the serial-over-TCP link.

Same mechanism as wham_serial_flash.py (send BOOT, then talk to the STM32 ROM
bootloader, AN3155), but spoken directly over a pyserial URL so it works
through the ethernet bridge instead of a local /dev/cu.* port.

    ssh -N -o ServerAliveInterval=30 -L 5000:<controller-ip>:5000 <jump-host>   # tunnel
    python3 python/net_flash.py                                   # controller .bin
    python3 python/net_flash.py --target simulator
    python3 python/net_flash.py --url rfc2217://localhost:5000    # if the bridge speaks RFC2217

IMPORTANT -- stop python/net_broker.py / net_terminal.py first: the bridge
accepts one client at a time.

THE BRIDGE MUST BE ABLE TO CHANGE PARITY (verified on hardware 2026-09-24).
The application talks 115200 8N1 but the ROM bootloader talks 8E1 (11-bit
frames). Through a bridge fixed at 8N1 (10-bit frames) the single sync byte
0x7F happens to get through and is ACKed, but every following byte is
misframed: the bootloader's USART2 showed PE (parity error) and FE (framing
error) set and NACKed the first real command (GET ID). It cannot be worked
around from the host: an 8N1 sender's parity position is always a 1, so any
data byte with an even number of 1-bits fails the check. A bridge that
supports RFC 2217 (telnet com-port control) would let this script switch
parity on the fly (--url rfc2217://...); the bridge tested on the bench does
NOT, so use the ST-Link there. A failed attempt leaves the board in the ROM
bootloader (nothing erased); reset it with the ST-Link or a power cycle.

If a flash fails after the erase starts, the board is left in the ROM
bootloader with no valid application. A reset/power cycle does not restore the
old firmware -- re-run this script (--no-boot if the board is already in the
bootloader).
"""
import argparse
import os
import sys
import time

import serial

ACK, NACK = 0x79, 0x1F
FLASH_BASE = 0x08000000
PAGE = 2048
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


class Boot:
    def __init__(self, ser):
        self.s = ser

    def read(self, n, what):
        d = self.s.read(n)
        if len(d) != n:
            raise SystemExit(f"[fail] timeout waiting for {what} (got {len(d)}/{n} bytes)")
        return d

    def ack(self, what):
        b = self.read(1, f"ACK after {what}")[0]
        if b != ACK:
            raise SystemExit(f"[fail] {what}: bootloader answered 0x{b:02X} (NACK)" if b == NACK
                             else f"[fail] {what}: unexpected byte 0x{b:02X}")

    def cmd(self, c, what):
        self.s.write(bytes([c, c ^ 0xFF]))
        self.ack(what)

    @staticmethod
    def with_xor(data):
        x = 0
        for b in data:
            x ^= b
        return bytes(data) + bytes([x])

    def sync(self):
        self.s.reset_input_buffer()
        self.s.write(b"\x7f")
        b = self.s.read(1)
        if not b:
            return False
        return b[0] in (ACK, NACK)   # NACK = already synchronised

    def get_id(self):
        self.cmd(0x02, "GET ID")
        n = self.read(1, "GET ID length")[0]
        pid = self.read(n + 1, "GET ID data")
        self.ack("GET ID end")
        return int.from_bytes(pid, "big")

    def read_mem(self, addr, n):
        self.cmd(0x11, "READ MEMORY")
        self.s.write(self.with_xor(addr.to_bytes(4, "big")))
        self.ack("READ address")
        self.s.write(bytes([n - 1, (n - 1) ^ 0xFF]))
        self.ack("READ length")
        return self.read(n, "memory data")

    def write_mem(self, addr, data):
        self.cmd(0x31, "WRITE MEMORY")
        self.s.write(self.with_xor(addr.to_bytes(4, "big")))
        self.ack("WRITE address")
        self.s.write(self.with_xor(bytes([len(data) - 1]) + bytes(data)))
        self.ack("WRITE data")

    def erase_pages(self, npages):
        self.cmd(0x44, "EXTENDED ERASE")
        payload = (npages - 1).to_bytes(2, "big") + b"".join(p.to_bytes(2, "big") for p in range(npages))
        self.s.write(self.with_xor(payload))
        old = self.s.timeout
        self.s.timeout = 60
        try:
            self.ack("erase")
        finally:
            self.s.timeout = old

    def go(self, addr):
        self.cmd(0x21, "GO")
        self.s.write(self.with_xor(addr.to_bytes(4, "big")))
        self.ack("GO address")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--url", default="socket://localhost:5000", help="pyserial URL (socket:// or rfc2217://)")
    ap.add_argument("--bin", help="firmware .bin (default: Debug/ build for --target)")
    ap.add_argument("--target", choices=["controller", "simulator"], default="controller")
    ap.add_argument("--app-baud", type=int, default=115200)
    ap.add_argument("--boot-baud", type=int, default=57600, help="ROM bootloader baud (8E1)")
    ap.add_argument("--no-boot", action="store_true", help="board is already in the bootloader")
    ap.add_argument("--no-go", action="store_true", help="do not start the application afterwards")
    ap.add_argument("--yes", action="store_true", help="skip the confirmation prompt")
    args = ap.parse_args()

    binpath = args.bin or os.path.join(
        ROOT, "Debug", "WHAM-XREX-PFMG474-SIM.bin" if args.target == "simulator" else "WHAM-XREX-PFMG474.bin")
    fw = open(binpath, "rb").read()
    fw += b"\xff" * (-len(fw) % 4)
    npages = -(-len(fw) // PAGE)
    print(f"target={args.target}  image={binpath}  {len(fw)} bytes  ({npages} pages)  via {args.url}")
    if not args.yes and input("Flash this board? Type 'yes': ").strip().lower() != "yes":
        sys.exit("aborted")

    ser = serial.serial_for_url(args.url, baudrate=args.app_baud, timeout=2)
    if not args.no_boot:
        ser.reset_input_buffer()
        ser.write(b"BOOT\r\n")
        reply = ser.read(64)
        print(f"BOOT reply: {reply!r}")
        if b"OK" not in reply:
            sys.exit("[fail] BOOT not acknowledged (wrong board, or BOOT feature disabled). Nothing changed.")
        time.sleep(1.0)   # board resets into the ROM bootloader
    ser.baudrate = args.boot_baud
    ser.parity = serial.PARITY_EVEN
    ser.timeout = 2
    time.sleep(0.2)

    b = Boot(ser)
    if not any(b.sync() for _ in range(3)):
        sys.exit("[fail] bootloader did not answer 0x7F. The bridge is probably not at 8E1 -- see this "
                 "script's docstring (try --url rfc2217://...). Nothing was erased.")
    pid = b.get_id()
    print(f"bootloader answered, chip ID 0x{pid:04X}")
    if pid != 0x469:
        sys.exit(f"[fail] unexpected chip ID 0x{pid:04X} (STM32G474 is 0x0469). Nothing was erased.")

    print(f"erasing {npages} pages ...")
    b.erase_pages(npages)
    print("writing ...")
    for off in range(0, len(fw), 256):
        b.write_mem(FLASH_BASE + off, fw[off:off + 256])
        if (off // 256) % 32 == 0:
            print(f"  {off * 100 // len(fw):3d}%", end="\r", flush=True)
    print("verifying ...")
    for off in range(0, len(fw), 256):
        chunk = fw[off:off + 256]
        if b.read_mem(FLASH_BASE + off, len(chunk)) != chunk:
            sys.exit(f"[fail] verify mismatch at 0x{FLASH_BASE + off:08X}")
    print("[ok] written and verified")
    if not args.no_go:
        b.go(FLASH_BASE)
        print("[ok] Go issued -- application starting; reconnect at 115200 8N1 and check *IDN?")
    ser.close()


if __name__ == "__main__":
    main()
