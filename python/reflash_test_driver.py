#!/usr/bin/env python3
"""
reflash_test_driver.py -- run N network reflash tests on the simulator,
bumping FW_VERSION_STRING each iteration (v0.8b, v0.8c, ...). Each iteration:
bump version -> build simulator -> verify image -> flash over the network
through the broker -> confirm the swap self-booted. Stops on the first
failure and logs a summary to logs/reflash_tests.log.

Run from the repo root (or anywhere -- paths are resolved from this file's
own location). The broker (net_broker.py) and tunnel must already be up;
this drives fw_update.py against socket://localhost:5001.
"""
import os
import re
import subprocess
import sys
import time
import zlib

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CTRLR = os.path.join(ROOT, "Core", "Inc", "ctrlr_config.h")
BIN = os.path.join(ROOT, "Debug", "WHAM-XREX-PFMG474-SIM.bin")
TOOLS = ("/Applications/STM32CubeIDE.app/Contents/Eclipse/plugins/"
         "com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32."
         "13.3.rel1.macos64_1.0.0.202411102158/tools/bin")
LOG = os.path.join(ROOT, "logs", "reflash_tests.log")
VERSIONS = ["v0.8b", "v0.8c", "v0.8d", "v0.8e", "v0.8f"]
KEY_TOKENS = ("ERASED", "VERIFIED", "SWAPPING", "banner", "after reboot",
              "now running", "fail", "resync", "aborted", "refused")


def log(msg):
    line = f"[{time.strftime('%H:%M:%S')}] {msg}"
    print(line, flush=True)
    with open(LOG, "a") as f:
        f.write(line + "\n")


def bump(ver):
    s = open(CTRLR).read()
    m = re.search(r'#define FW_VERSION_STRING\s+"[^"]+"', s)
    if not m:
        raise SystemExit("FW_VERSION_STRING not found in ctrlr_config.h")
    new = f'#define FW_VERSION_STRING "{ver}"'
    open(CTRLR, "w").write(s[:m.start()] + new + s[m.end():])
    log(f"bumped FW_VERSION_STRING -> {ver}")


def build():
    env = dict(os.environ)
    env["PATH"] = TOOLS + ":" + env.get("PATH", "")
    r = subprocess.run([sys.executable, "python/wham_build.py", "--target", "simulator"],
                       cwd=ROOT, env=env,
                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    if r.returncode != 0:
        log("BUILD FAILED:\n" + r.stdout[-1500:])
        return False
    return True


def verify(ver):
    img = open(BIN, "rb").read()
    ok = (ver.encode() + b"\0") in img and b"WHAM-XREX-PFMG474-SIM\0" in img
    crc = zlib.crc32(img) & 0xFFFFFFFF
    log(f"image {len(img)} bytes crc={crc:08X} version_ok={ok}")
    return ok


def flash(ver):
    r = subprocess.run([sys.executable, "python/fw_update.py", "--target", "simulator",
                        "--url", "socket://localhost:5001", "--name", "fw_update", "--yes"],
                       cwd=ROOT,
                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    out = r.stdout
    ok = (r.returncode == 0) and (f"now running {ver}" in out)
    log(f"flash exit={r.returncode} ok={ok}")
    for line in out.splitlines():
        if any(k in line for k in KEY_TOKENS):
            log("  " + line)
    return ok


def main():
    os.makedirs(os.path.dirname(LOG), exist_ok=True)
    log(f"=== starting {len(VERSIONS)} reflash tests: {VERSIONS} ===")
    for i, ver in enumerate(VERSIONS, 1):
        log(f"--- test {i}/{len(VERSIONS)}: {ver} ---")
        bump(ver)
        if not build():
            log(f"RESULT {ver}: BUILD FAILED -- aborting")
            return 1
        if not verify(ver):
            log(f"RESULT {ver}: IMAGE VERIFY FAILED -- aborting")
            return 1
        if not flash(ver):
            log(f"RESULT {ver}: FLASH FAILED -- aborting")
            return 1
        log(f"RESULT {ver}: PASSED")
        time.sleep(2)
    log(f"=== ALL {len(VERSIONS)} TESTS PASSED ===")
    return 0


if __name__ == "__main__":
    sys.exit(main())
