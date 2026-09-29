#!/usr/bin/env python3
"""
wham_build.py -- one-command build for WHAM-XREX-PFMG474: regenerates
build/generated/git_version.h (gen_git_version.py) so *IDN? reports exactly
which commit this build is from, regenerates build/generated/build_target.h
(gen_build_target.py) to select CONTROLLER vs SIMULATOR firmware, checks
the Debug/ makefiles still match the source tree (sync_build_sources.py
--check), checks the layer rules (check_layout.py), runs the off-target
unit tests (tests/), runs `make` in Debug/, regenerates Debug/WHAM-XREX-PFMG474.bin,
and publishes the .elf/.bin/.map to build/ under the target's own name:

    build/WHAM-XREX-PFMG474.{elf,bin,map}       controller
    build/WHAM-XREX-PFMG474-SIM.{elf,bin,map}   simulator

build/ is what the flashing tools read by default and what git tracks;
Debug/ is CubeIDE's working folder (its .elf is what the IDE debugger
loads). Since each target publishes under its own names, a simulator
build can never leave simulator firmware behind under the controller's
filename in build/.

That .bin step matters on its own: this project's .cproject does NOT
have CubeIDE's "Convert to binary file (.bin)" post-build step enabled
(see AGENTS.md), so `make` alone never touches the .bin at all -- a
real, confirmed-on-hardware gotcha (docs/changelog.txt, 2026-09-10):
running wham_serial_flash.py against a stale .bin flashes it
"successfully" (verified even) while it silently does NOT contain your
latest changes. This script makes the three steps genuinely one
command, so there's no gap in the sequence to forget.

Usage:
  python3 python/wham_build.py                       # controller (default)
  python3 python/wham_build.py --target simulator     # Transrex simulator
  python3 python/wham_build.py --no-git-version  # skip the git_version.h step
                                                  # (rare -- e.g. debugging the
                                                  # build itself with a fixed header)
  python3 python/wham_build.py --clean         # `make clean` first

--target defaults to "controller" -- every existing invocation/workflow
this project already has keeps working unchanged. A --target simulator
build's output .bin/.elf are additionally copied to distinctly-named
Debug/WHAM-XREX-PFMG474-SIM.{bin,elf} (the underlying make-driven build
still produces the same WHAM-XREX-PFMG474.{bin,elf} filenames either
way -- this project's Debug/ Makefile is CubeIDE-generated and
deliberately NOT hand-edited to rename its own output, see AGENTS.md's
"Adding a new source file" section for the exact same class of
fragility).

*** REAL BUG CAUGHT during Phase-0 verification, 2026-09-17, before any
flash was attempted: a --target simulator build's own `make all` step
silently OVERWRITES the plain Debug/WHAM-XREX-PFMG474.{elf,bin} (the
exact filenames wham_serial_flash.py's own DEFAULT --target controller
picks) with SIMULATOR content, since both targets share those
filenames -- caught by comparing the embedded *IDN? string between two
builds, not assumed correct. This is precisely the failure mode
flagged as critical: an unqualified wham_serial_flash.py run right
after a simulator build would flash the CONTROLLER-named default file,
but it would actually contain simulator firmware. Fixed by DELETING
the plain-named .elf/.bin after copying them to the SIM-named files on
a --target simulator build -- the plain filenames must never exist in
an ambiguous state; a subsequent plain `wham_serial_flash.py` (no
--target) will correctly fail loudly (file not found) rather than
silently flash the wrong firmware, forcing an explicit
`--target controller` rebuild before the controller can be flashed
again. ***

After this, flash with:
  python3 python/wham_serial_flash.py                       # controller
  python3 python/wham_serial_flash.py --target simulator    # simulator
"""

import argparse
import os
import shutil
import subprocess
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.dirname(SCRIPT_DIR)  # this script lives in python/
DEBUG_DIR = os.path.join(PROJECT_DIR, "Debug")
BUILD_DIR = os.path.join(PROJECT_DIR, "build")   # published, tracked images (see header)
ELF_NAME = "WHAM-XREX-PFMG474.elf"
BIN_NAME = "WHAM-XREX-PFMG474.bin"
SIM_ELF_NAME = "WHAM-XREX-PFMG474-SIM.elf"
SIM_BIN_NAME = "WHAM-XREX-PFMG474-SIM.bin"


def run(cmd, cwd=None, description=""):
    print(f"\n--- {description or ' '.join(cmd)} ---")
    result = subprocess.run(cmd, cwd=cwd)
    if result.returncode != 0:
        sys.exit(f"error: {description or cmd[0]} failed (exit {result.returncode})")


def main():
    ap = argparse.ArgumentParser(description="One-command build for WHAM-XREX-PFMG474")
    ap.add_argument("--target", choices=["controller", "simulator"], default="controller",
                     help="which firmware to build (default: controller -- every existing "
                          "workflow keeps working unchanged)")
    ap.add_argument("--no-git-version", action="store_true",
                     help="skip regenerating build/generated/git_version.h")
    ap.add_argument("--clean", action="store_true", help="`make clean` before building")
    ap.add_argument("--skip-tests", action="store_true",
                     help="don't run the off-target unit tests (tests/, needs a host C compiler)")
    ap.add_argument("--jobs", "-j", type=int, default=4, help="make -j parallelism (default 4)")
    ap.add_argument("--objcopy", default="arm-none-eabi-objcopy",
                     help="objcopy binary (default: arm-none-eabi-objcopy, must be on PATH)")
    args = ap.parse_args()

    if not os.path.isdir(DEBUG_DIR):
        sys.exit(f"error: {DEBUG_DIR} not found -- run this from a normal checkout "
                  "(this script lives in python/, one level under the project root)")

    run([sys.executable, os.path.join(SCRIPT_DIR, "gen_build_target.py"), "--target", args.target],
        description=f"regenerating build/generated/build_target.h (--target {args.target})")

    if not args.no_git_version:
        run([sys.executable, os.path.join(SCRIPT_DIR, "gen_git_version.py")],
            description="regenerating build/generated/git_version.h")
    else:
        print("\n--- skipping git_version.h regeneration (--no-git-version) ---")

    # The source tree and the build must agree before make runs: a file
    # added, moved or renamed without re-syncing would otherwise be silently
    # left out of the link (or fail with an undefined reference).
    run([sys.executable, os.path.join(SCRIPT_DIR, "sync_build_sources.py"), "--check"],
        description="checking Debug/ makefiles and .cproject match the source tree "
                    "(if this fails: python3 python/sync_build_sources.py)")

    # The source layout rules (AGENTS.md, "Source layout") -- a violation
    # fails the build, the same as a compile error would.
    run([sys.executable, os.path.join(SCRIPT_DIR, "check_layout.py")],
        description="checking the source layout (python/check_layout.py)")

    if not args.skip_tests:
        run(["make", "-C", os.path.join(PROJECT_DIR, "tests")],
            description="off-target unit tests (tests/)")

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

    # Publish to build/ under the target's own names. build/ is the tracked
    # record of what was built; Debug/ is CubeIDE's working folder.
    os.makedirs(BUILD_DIR, exist_ok=True)
    stem = "WHAM-XREX-PFMG474-SIM" if args.target == "simulator" else "WHAM-XREX-PFMG474"
    for ext in ("elf", "bin", "map"):
        shutil.copyfile(os.path.join(DEBUG_DIR, f"WHAM-XREX-PFMG474.{ext}"),
                        os.path.join(BUILD_DIR, f"{stem}.{ext}"))
    print(f"\n--- published build/{stem}.elf/.bin/.map ---")

    if args.target == "simulator":
        sim_elf_path = os.path.join(DEBUG_DIR, SIM_ELF_NAME)
        sim_bin_path = os.path.join(DEBUG_DIR, SIM_BIN_NAME)
        shutil.copyfile(elf_path, sim_elf_path)
        shutil.copyfile(bin_path, sim_bin_path)
        # Delete the plain-named files -- see this script's own header
        # comment for the real bug this prevents: they must never be
        # left sitting around containing simulator content under the
        # same filename a controller-target flash would default to.
        os.remove(elf_path)
        os.remove(bin_path)
        print(f"\n[done] {sim_bin_path} ({size} bytes) -- copied from the build's own "
              f"{BIN_NAME}/{ELF_NAME}, which were then DELETED (not left behind under "
              f"the controller's own default filename). Flash with:\n"
              f"       python3 python/wham_serial_flash.py --target simulator")
    else:
        print(f"\n[done] {bin_path} ({size} bytes). Flash with:\n"
              f"       python3 python/wham_serial_flash.py")


if __name__ == "__main__":
    main()
