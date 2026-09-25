#!/usr/bin/env python3
"""fw_update.py -- update WHAM-XREX-PFMG474 firmware over the normal serial
command link (115200 8N1), using the firmware's own FWUPdate:* commands.

Unlike wham_serial_flash.py / net_flash.py this never enters the ROM
bootloader's AN3155 protocol, so it works through the ethernet serial bridge.
The new image goes into the flash bank that is NOT running; the running image
is untouched until the final swap, and stays in the other bank afterwards
(`--rollback` boots it again). See Core/Inc/fw_update.h.

    ssh -N -o ServerAliveInterval=30 -L 5000:<controller-ip>:5000 <jump-host>   # tunnel
    python3 python/fw_update.py --status
    python3 python/fw_update.py                      # controller image, then swap
    python3 python/fw_update.py --target simulator --url socket://localhost:<port>
    python3 python/fw_update.py --no-swap            # load + verify only
    python3 python/fw_update.py --rollback           # boot the previous image

The bridge takes one client. Either stop net_broker.py / net_terminal.py
first, or go through the broker so others can watch:
    python3 python/fw_update.py --url socket://localhost:5001 --name fw_update
Works with a local port too: --url /dev/cu.usbserial-1130
"""
import argparse
import os
import sys
import time
import zlib

import serial

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BANK_SIZE = 256 * 1024
CHUNK = 48
NAMES = {"controller": b"WHAM-XREX-PFMG474", "simulator": b"WHAM-XREX-PFMG474-SIM"}


class Link:
    def __init__(self, url):
        self.s = serial.serial_for_url(url, baudrate=115200, timeout=0.1)
        self.buf = b""
        self.boot_lines = []      # (time, text) of every !BOOT line seen

    def line(self, timeout):
        """Next reply line. !EVT telemetry and net_broker.py chatter
        ("--- x joined ---", "[name] > cmd") are skipped; !BOOT banner lines
        are printed and recorded, not returned."""
        end = time.time() + timeout
        while time.time() < end:
            if b"\n" in self.buf:
                ln, self.buf = self.buf.split(b"\n", 1)
                ln = ln.strip().decode(errors="replace")
                if "!BOOT" in ln:
                    # the reset glitches TX, so the first banner line can
                    # arrive with a garbage byte in front of it
                    ln = ln[ln.index("!BOOT"):]
                    self.boot_lines.append((time.time(), ln))
                    print(f"  {ln}")
                    continue
                if ln and not ln.startswith(("!EVT", "---", "[")):
                    return ln
                continue
            self.buf += self.s.read(max(1, self.s.in_waiting))
        return None

    def cmd(self, text, timeout=3.0):
        self.s.write(text.encode() + b"\r\n")
        return self.line(timeout)

    def sync(self):
        self.s.write(b"\r\n")
        time.sleep(0.3)
        self.s.reset_input_buffer()
        self.buf = b""


def fail(msg):
    sys.exit(f"[fail] {msg}")


def idn(link):
    r = link.cmd("*IDN?")
    if not r or not r.startswith("OK "):
        fail(f"no *IDN? reply ({r!r}) -- is the tunnel up and nothing else connected?")
    return r


def running_target(idn_reply):
    return "simulator" if idn_reply.split()[1] == NAMES["simulator"].decode() else "controller"


MAX_RESYNCS = 10


def received_count(link):
    """RX byte count from FWUPdate:STATus? while a transfer is in progress,
    or None. Retried, since the reply itself can be the one that gets hit."""
    for _ in range(3):
        st = link.cmd("FWUP:STAT?", timeout=2)
        if st and "STATE=RECEIVING" in st:
            for tok in st.split():
                if tok.startswith("RX="):
                    return int(tok[3:].split("/")[0])
        if st and "STATE=" in st:
            return None
    return None


def bank_of(status):
    for tok in (status or "").split():
        if tok.startswith("BANK="):
            return tok[5:]
    return None


def wait_for_reboot(link, timeout=30.0):
    """Wait for the reboot after SWAP/ROLLback. Firmware with the !BOOT
    banner announces itself the moment it is up; older firmware is found by
    polling *IDN?."""
    t0 = time.time()
    n0 = len(link.boot_lines)
    while time.time() - t0 < min(timeout, 20.0) and len(link.boot_lines) == n0:
        link.line(0.2)
    if len(link.boot_lines) > n0:
        print(f"  (banner {link.boot_lines[n0][0] - t0:.2f}s after the swap ack)")
    else:
        print("  (no !BOOT banner seen -- falling back to *IDN? polling)")
    end = time.time() + timeout
    while time.time() < end:
        r = link.cmd("*IDN?", timeout=1.5)
        if r and r.startswith("OK "):
            return r
        link.sync()
    return None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--url", default="socket://localhost:5000")
    ap.add_argument("--target", choices=list(NAMES), default="controller")
    ap.add_argument("--bin", help="image to load (default: Debug/ build for --target)")
    ap.add_argument("--status", action="store_true", help="just print FWUPdate:STATus? and exit")
    ap.add_argument("--no-swap", action="store_true", help="load and verify, but keep running the old image")
    ap.add_argument("--rollback", action="store_true", help="boot the image already in the other bank")
    ap.add_argument("--yes", action="store_true", help="skip the confirmation prompt")
    ap.add_argument("--name", help="connecting through net_broker.py (port 5001): name to show "
                                   "other clients, e.g. --url socket://localhost:5001 --name fw_update")
    args = ap.parse_args()

    link = Link(args.url)
    if args.name:
        link.s.write(f"@name {args.name}\r\n".encode())
    link.sync()
    who = idn(link)
    print(f"connected: {who}")
    print(f"status:    {link.cmd('FWUP:STAT?')}")
    if args.status:
        return
    if running_target(who) != args.target:
        fail(f"the connected board is the {running_target(who)}, not the {args.target} -- nothing changed")

    if args.rollback:
        if not args.yes and input("Reboot into the image in the other bank? Type 'yes': ").strip() != "yes":
            sys.exit("aborted")
        r = link.cmd("FWUP:ROLL", timeout=5)
        print(r)
        if not r or not r.startswith("OK SWAPPING"):
            fail("rollback refused -- nothing changed")
        after = wait_for_reboot(link)
        print(f"after reboot: {after}\nstatus:       {link.cmd('FWUP:STAT?')}")
        return

    binpath = args.bin or os.path.join(
        ROOT, "Debug", "WHAM-XREX-PFMG474-SIM.bin" if args.target == "simulator" else "WHAM-XREX-PFMG474.bin")
    img = open(binpath, "rb").read()
    img += b"\xff" * (-len(img) % 8)
    crc = zlib.crc32(img) & 0xFFFFFFFF
    if len(img) > BANK_SIZE:
        fail(f"image is {len(img)} bytes, larger than one flash bank ({BANK_SIZE})")
    if NAMES[args.target] + b"\0" not in img:
        fail(f"{binpath} does not contain {NAMES[args.target].decode()} -- wrong image for --target")
    print(f"image:     {binpath}  {len(img)} bytes  crc32 {crc:08X}")
    if not args.yes and input("Load this image into the inactive bank? Type 'yes': ").strip() != "yes":
        sys.exit("aborted")

    r = link.cmd(f"FWUP:BEG {len(img)} {crc:08X}", timeout=20)
    print(r)
    if not r or not r.startswith("OK ERASED"):
        fail("FWUPdate:BEGin refused -- the running image is untouched")

    t0 = time.time()
    off = 0
    resyncs = 0
    next_progress = 0
    while off < len(img):
        chunk = img[off:off + CHUNK]
        r = link.cmd(f"FWUP:DATA {off:X} {chunk.hex().upper()}", timeout=3)
        if r == "OK":
            off += len(chunk)
        else:
            # A byte lost either way on the link (seen on the ethernet bridge:
            # an "OK" reply arriving as "O") leaves us unsure whether this
            # chunk was written. Ask the firmware how far it got and resume
            # from there -- it only accepts the exact next offset, and the
            # final CRC still guards the whole image.
            done = received_count(link)
            resyncs += 1
            if done not in (off, off + len(chunk)) or resyncs > MAX_RESYNCS:
                link.cmd("FWUP:ABOR")
                fail(f"at offset 0x{off:X}: {r!r} (firmware has {done}) -- transfer aborted, "
                     "the running image is untouched")
            print(f"\n  resync at 0x{off:X}: got {r!r}, firmware has {done} bytes -- resuming")
            off = done
        if off >= next_progress:
            next_progress += CHUNK * 100
            rate = off / max(time.time() - t0, 1e-3)
            print(f"  {off * 100 // len(img):3d}%  {rate / 1024:5.1f} KB/s", end="\r", flush=True)
    print(f"  100%  {len(img) / (time.time() - t0) / 1024:5.1f} KB/s   "
          + (f"({resyncs} resync{'s' if resyncs != 1 else ''})" if resyncs else ""))

    r = link.cmd("FWUP:END", timeout=10)
    print(r)
    if not r or not r.startswith("OK VERIFIED"):
        fail("verification failed -- the running image is untouched")
    if args.no_swap:
        print("[ok] image loaded and verified in the inactive bank; not swapping (--no-swap)")
        return

    bank_before = bank_of(link.cmd("FWUP:STAT?"))
    r = link.cmd("FWUP:SWAP", timeout=5)
    print(r)
    if not r or not r.startswith("OK SWAPPING"):
        fail("swap refused -- still running the old image")
    after = wait_for_reboot(link)
    if not after:
        fail("board did not answer *IDN? after the swap. Recover with the ST-Link "
             "(see docs/command_reference.md, FWUPdate)")
    status = link.cmd("FWUP:STAT?")
    print(f"after reboot: {after}\nstatus:       {status}")
    if bank_of(status) == bank_before:
        fail(f"board came back on bank {bank_before} (the OLD image) -- the swap did not take")
    version, githash = after.split()[3], after.split()[4].split("-")[0]
    if version.encode() + b"\0" not in img or githash.encode() not in img:
        fail(f"running firmware reports {version} {githash}, which is not in {binpath}")
    print(f"[ok] now running {version} {githash} from bank {bank_of(status)}")


if __name__ == "__main__":
    main()
