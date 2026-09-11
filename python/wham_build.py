#!/usr/bin/env python3
"""
wham_build.py -- one-command build for WHAM-XREX-PFMG474: regenerates
Core/Inc/git_version.h (gen_git_version.py) so *IDN? reports exactly
which commit this build is from, runs `make` in Debug/, then
regenerates Debug/WHAM-XREX-PFMG474.bin.

That last step matters on its own: this project's .cproject does NOT
have CubeIDE's "Convert to binary file (.bin)" post-build step enabled
(see AGENTS.md), so `make` alone never touches the .bin at all -- a
real, confirmed-on-hardware gotcha (docs/changelog.txt, 2026-09-10):
running wham_serial_flash.py against a stale .bin flashes it
"successfully" (verified even) while it silently does NOT contain your
latest changes. This script makes the three steps genuinely one
command, so there's no gap in the sequence to forget.

Usage:
  python3 python/wham_build.py                # regenerate git_version.h, make, .bin
  python3 python/wham_build.py --no-git-version  # skip the git_version.h step
                                                  # (rare -- e.g. debugging the
                                                  # build itself with a fixed header)
  python3 python/wham_build.py --clean         # `make clean` first

After this, flash with:
  python3 python/wham_serial_flash.py
"""

import argparse
import os
import subprocess
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.dirname(SCRIPT_DIR)  # this script lives in python/
DEBUG_DIR = os.path.join(PROJECT_DIR, "Debug")
ELF_NAME = "WHAM-XREX-PFMG474.elf"
BIN_NAME = "WHAM-XREX-PFMG474.bin"


def run(cmd, cwd=None, description=""):
    print(f"\n--- {description or ' '.join(cmd)} ---")
    result = subprocess.run(cmd, cwd=cwd)
    if result.returncode != 0:
        sys.exit(f"error: {description or cmd[0]} failed (exit {result.returncode})")


def main():
    ap = argparse.ArgumentParser(description="One-command build for WHAM-XREX-PFMG474")
    ap.add_argument("--no-git-version", action="store_true",
                     help="skip regenerating Core/Inc/git_version.h")
    ap.add_argument("--clean", action="store_true", help="`make clean` before building")
    ap.add_argument("--jobs", "-j", type=int, default=4, help="make -j parallelism (default 4)")
    ap.add_argument("--objcopy", default="arm-none-eabi-objcopy",
                     help="objcopy binary (default: arm-none-eabi-objcopy, must be on PATH)")
    args = ap.parse_args()

    if not os.path.isdir(DEBUG_DIR):
        sys.exit(f"error: {DEBUG_DIR} not found -- run this from a normal checkout "
                  "(this script lives in python/, one level under the project root)")

    if not args.no_git_version:
        run([sys.executable, os.path.join(SCRIPT_DIR, "gen_git_version.py")],
            description="regenerating Core/Inc/git_version.h")
    else:
        print("\n--- skipping git_version.h regeneration (--no-git-version) ---")

    if args.clean:
        run(["make", "clean"], cwd=DEBUG_DIR, description="make clean")

    run(["make", f"-j{args.jobs}", "all"], cwd=DEBUG_DIR, description="make all")

    elf_path = os.path.join(DEBUG_DIR, ELF_NAME)
    bin_path = os.path.join(DEBUG_DIR, BIN_NAME)
    if not os.path.isfile(elf_path):
        sys.exit(f"error: {elf_path} wasn't produced -- build must have failed silently")
    run([args.objcopy, "-O", "binary", elf_path, bin_path],
        description=f"objcopy -> {BIN_NAME}")

    size = os.path.getsize(bin_path)
    print(f"\n[done] {bin_path} ({size} bytes). Flash with:\n"
          f"       python3 python/wham_serial_flash.py")


if __name__ == "__main__":
    main()
