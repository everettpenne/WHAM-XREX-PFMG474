#!/usr/bin/env python3
"""
memory_report.py -- flash/RAM footprint snapshot for WHAM-XREX-PFMG474.

Reads a built .elf (default: Debug/WHAM-XREX-PFMG474.elf) with
arm-none-eabi-size and arm-none-eabi-nm, and reports:
  - total FLASH used / total (512 KiB, STM32G474QETX_FLASH.ld) and
    RAM used / total (128 KiB)
  - a breakdown of that usage by source file/module, split into
    flash (code + rodata + .data's flash copy) and RAM (.data + .bss)

Two subcommands:
  snapshot   Print a snapshot (human-readable, or --json) for the elf
             given by --elf (or found via --commit, see below).
  history    Append one row to docs/memory_history.csv for the CURRENT
             git commit (HEAD) using --elf, or backfill history for
             every past commit that has a Debug/WHAM-XREX-PFMG474.elf
             checked in (--backfill) by extracting each commit's own
             tracked elf with `git show` -- no rebuilding needed, since
             this project tracks Debug/ build output in git.

Per-module attribution uses arm-none-eabi-nm --print-size -l (DWARF
line info from this project's -g3 debug builds) rather than the
linker .map file: with -ffunction-sections/-fdata-sections and
--gc-sections (both used by this project's Makefile), a raw per-.o
`size` sums in code the linker later discarded as unreachable --
wildly overcounting HAL driver modules where only a handful of
functions per file actually get pulled into the final image. Reading
attribution off the LINKED elf's own retained symbol table avoids
that: every symbol counted here genuinely made it into the binary.

One real wrinkle handled explicitly: this MCU's interrupt vector
table aliases ~80 unimplemented IRQ handler names onto the same
Default_Handler address (weak symbols, all sharing one definition) --
nm lists all ~80 names at that one address, so summing naively would
count that handler's few bytes ~80 times over. Deduplicated by
(address, size) before summing.

Usage:
  python3 memory_report.py snapshot [--elf PATH] [--json]
  python3 memory_report.py history  [--elf PATH]
  python3 memory_report.py history  --backfill
"""

import argparse
import csv
import json
import re
import shutil
import subprocess
import sys
import tempfile
from collections import defaultdict
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_ELF = REPO_ROOT / "Debug" / "WHAM-XREX-PFMG474.elf"
HISTORY_CSV = REPO_ROOT / "docs" / "memory_history.csv"

# STM32G474QETX_FLASH.ld MEMORY block -- kept as plain constants here
# rather than parsed from the .ld file, since they've been fixed since
# this project's first commit and re-parsing linker script syntax for
# two numbers isn't worth it; revisit if the part or memory map ever
# changes.
FLASH_TOTAL = 512 * 1024
RAM_TOTAL = 128 * 1024

# Toolchain location -- STM32CubeIDE's bundled arm-none-eabi-gcc, same
# one this project's Makefile/AGENTS.md already point to. Overridable
# via TOOLCHAIN_BIN for anyone with a different install layout.
import os

TOOLCHAIN_BIN = os.environ.get(
    "TOOLCHAIN_BIN",
    "/Applications/STM32CubeIDE.app/Contents/Eclipse/plugins/"
    "com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32."
    "13.3.rel1.macos64_1.0.0.202411102158/tools/bin",
)


def tool(name):
    candidate = Path(TOOLCHAIN_BIN) / name
    if candidate.exists():
        return str(candidate)
    found = shutil.which(name)
    if found:
        return found
    sys.exit(
        f"error: {name} not found (looked in {TOOLCHAIN_BIN} and PATH) -- "
        f"set TOOLCHAIN_BIN to your arm-none-eabi toolchain's bin/ dir"
    )


# --------------------------------------------------------------------------
# Module bucketing -- coarse, source-tree-shaped categories a reader can
# scan at a glance. Order matters: first matching prefix wins.
# --------------------------------------------------------------------------
MODULE_RULES = [
    ("Core/Startup", "Startup/vector table"),
    ("Core/Src/pfm_input.c", "PFM_Input capture"),
    ("Core/Src/pfm.c", "PFM table engine"),
    ("Core/Src/hrtim.c", "HRTIM driver"),
    ("Core/Src/commands.c", "Serial commands"),
    ("Core/Src/cmd_parser.c", "Serial commands"),
    ("Core/Src/uart.c", "Serial commands"),
    ("Core/Src/gate_driver.c", "Gate driver / faults"),
    ("Core/Src/qspi_test.c", "QUADSPI test"),
    ("Core/Src/boot_jump.c", "Bootloader jump"),
    ("Core/Src/stm32g4xx_it.c", "IRQ vector dispatch"),
    ("Core/Src/stm32g4xx_hal_msp.c", "HAL MSP config"),
    ("Core/Src/main.c", "main() / init"),
    ("Core/Src/system_stm32g4xx.c", "CMSIS system init"),
    ("Core/Src/syscalls.c", "Newlib syscalls"),
    ("Core/Src/sysmem.c", "Newlib syscalls"),
    ("Drivers/STM32G4xx_HAL_Driver", "ST HAL library"),
    ("Drivers/CMSIS", "CMSIS"),
]


def bucket_for(path):
    if not path:
        return "Toolchain (libc/libm/startup)"
    for prefix, label in MODULE_RULES:
        if prefix in path:
            return label
    return "Other"


# --------------------------------------------------------------------------
# arm-none-eabi-size -- overall totals
# --------------------------------------------------------------------------
def read_totals(elf_path):
    out = subprocess.run(
        [tool("arm-none-eabi-size"), str(elf_path)],
        capture_output=True, text=True, check=True,
    ).stdout
    # "   text    data     bss     dec     hex filename"
    line = out.strip().splitlines()[-1]
    text, data, bss = (int(x) for x in line.split()[:3])
    flash_used = text + data
    ram_used = data + bss
    return {
        "text": text, "data": data, "bss": bss,
        "flash_used": flash_used, "flash_total": FLASH_TOTAL,
        "ram_used": ram_used, "ram_total": RAM_TOTAL,
    }


# --------------------------------------------------------------------------
# arm-none-eabi-nm -- per-symbol attribution
# --------------------------------------------------------------------------
NM_LINE_RE = re.compile(
    r"^([0-9a-fA-F]+)\s+(?:([0-9a-fA-F]+)\s+)?(\S)\s+(\S+)(?:\t(.*))?$"
)

# Symbol types that occupy FLASH (code + read-only data + the flash-
# resident init image of .data). Weak (W/w) symbols in this project
# are exclusively the unimplemented-IRQ vector stubs -- real code.
FLASH_TYPES = set("TtRrWwDd")
# Symbol types that occupy RAM at runtime (.data lives in both flash
# AND ram; .bss lives only in ram).
RAM_TYPES = set("BbDd")


def read_symbols(elf_path):
    out = subprocess.run(
        [tool("arm-none-eabi-nm"), "--print-size", "-l", str(elf_path)],
        capture_output=True, text=True, check=True,
    ).stdout

    seen_addr_size = set()  # dedupe weak-alias IRQ handlers etc.
    flash_by_module = defaultdict(int)
    ram_by_module = defaultdict(int)

    for line in out.splitlines():
        m = NM_LINE_RE.match(line)
        if not m:
            continue
        addr, size_hex, typ, name, fileinfo = m.groups()
        if size_hex is None:
            continue  # linker marker symbol, no size (e.g. _sdata)
        size = int(size_hex, 16)
        if size == 0:
            continue

        key = (addr, size_hex, typ)
        if key in seen_addr_size:
            continue  # same address+size+type already counted (alias)
        seen_addr_size.add(key)

        path = None
        if fileinfo:
            path = fileinfo.rsplit(":", 1)[0]
            # Normalize the "Debug/../Core/Src/x.c" noise from -l's
            # absolute paths down to a repo-relative-looking form.
            idx = path.find("/Core/")
            if idx == -1:
                idx = path.find("/Drivers/")
            if idx != -1:
                path = path[idx + 1:]

        module = bucket_for(path)

        if typ in FLASH_TYPES:
            flash_by_module[module] += size
        if typ in RAM_TYPES:
            ram_by_module[module] += size

    return flash_by_module, ram_by_module


PADDING_LABEL = "Padding / unattributed (alignment, vector table, anon symbols)"


def snapshot(elf_path):
    totals = read_totals(elf_path)
    flash_by_module, ram_by_module = read_symbols(elf_path)

    # nm's symbol table doesn't cover every byte `size` counts --
    # section alignment padding, the raw .isr_vector table, and a
    # handful of anonymous/mergeable symbols (string-literal pools)
    # are real bytes with no single attributable symbol. Roll the gap
    # into an explicit bucket so a stacked chart's total still equals
    # the true, authoritative `size` total instead of quietly
    # undercounting.
    flash_gap = totals["flash_used"] - sum(flash_by_module.values())
    ram_gap = totals["ram_used"] - sum(ram_by_module.values())
    if flash_gap > 0:
        flash_by_module[PADDING_LABEL] = flash_gap
    if ram_gap > 0:
        ram_by_module[PADDING_LABEL] = ram_gap

    return {
        "totals": totals,
        "flash_by_module": dict(sorted(
            flash_by_module.items(), key=lambda kv: -kv[1])),
        "ram_by_module": dict(sorted(
            ram_by_module.items(), key=lambda kv: -kv[1])),
    }


def print_snapshot_human(snap):
    t = snap["totals"]
    print(f"FLASH: {t['flash_used']:>7,} / {t['flash_total']:,} bytes "
          f"({100 * t['flash_used'] / t['flash_total']:.1f}%)")
    print(f"RAM:   {t['ram_used']:>7,} / {t['ram_total']:,} bytes "
          f"({100 * t['ram_used'] / t['ram_total']:.1f}%)")
    print()
    print("Flash by module:")
    for mod, size in snap["flash_by_module"].items():
        print(f"  {size:>7,}  {mod}")
    print()
    print("RAM by module:")
    for mod, size in snap["ram_by_module"].items():
        print(f"  {size:>7,}  {mod}")


# --------------------------------------------------------------------------
# git plumbing -- current commit, and pulling a past commit's own
# tracked Debug/WHAM-XREX-PFMG474.elf without touching the working tree.
# --------------------------------------------------------------------------
def git(*args):
    return subprocess.run(
        ["git", *args], cwd=REPO_ROOT, capture_output=True, text=True, check=True
    ).stdout.strip()


def commit_date(rev):
    return git("show", "-s", "--format=%cI", rev)


def commit_subject(rev):
    return git("show", "-s", "--format=%s", rev)


def extract_elf_at(rev, dest_dir):
    dest = Path(dest_dir) / f"{rev}.elf"
    data = subprocess.run(
        ["git", "show", f"{rev}:Debug/WHAM-XREX-PFMG474.elf"],
        cwd=REPO_ROOT, capture_output=True, check=True,
    ).stdout
    dest.write_bytes(data)
    return dest


def commits_with_tracked_elf():
    out = git("log", "--format=%H", "--", "Debug/WHAM-XREX-PFMG474.elf")
    return [line for line in out.splitlines() if line]


# --------------------------------------------------------------------------
# History CSV -- one row per (commit, snapshot). Wide format: fixed
# totals columns + one column per module bucket for each of flash/ram,
# NaN-filled ("") for modules that didn't exist yet at a given commit.
# Column set can only grow over time (new modules append new columns);
# a plotting/reading script should treat a missing column as 0, not
# an error.
# --------------------------------------------------------------------------
def history_row(rev, subject, date, snap):
    row = {
        "commit": rev[:10],
        "date": date,
        "subject": subject,
        "flash_used": snap["totals"]["flash_used"],
        "flash_total": snap["totals"]["flash_total"],
        "ram_used": snap["totals"]["ram_used"],
        "ram_total": snap["totals"]["ram_total"],
    }
    for mod, size in snap["flash_by_module"].items():
        row[f"flash:{mod}"] = size
    for mod, size in snap["ram_by_module"].items():
        row[f"ram:{mod}"] = size
    return row


def load_history():
    if not HISTORY_CSV.exists():
        return []
    with HISTORY_CSV.open(newline="") as f:
        return list(csv.DictReader(f))


def write_history(rows):
    fieldnames = ["commit", "date", "subject",
                  "flash_used", "flash_total", "ram_used", "ram_total"]
    extra = sorted({k for row in rows for k in row if k not in fieldnames})
    fieldnames += extra
    HISTORY_CSV.parent.mkdir(parents=True, exist_ok=True)
    with HISTORY_CSV.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames, restval="")
        w.writeheader()
        for row in rows:
            w.writerow(row)


def append_history_for_current(elf_path):
    rev = git("rev-parse", "HEAD")
    dirty = bool(git("status", "--porcelain", "--", "Debug/"))
    subject = commit_subject(rev)
    date = commit_date(rev)
    snap = snapshot(elf_path)
    row = history_row(rev, subject, date, snap)
    if dirty:
        row["commit"] += "-dirty"

    rows = load_history()
    rows = [r for r in rows if r["commit"] != row["commit"]]  # replace, not dup
    rows.append(row)
    rows.sort(key=lambda r: r["date"])
    write_history(rows)
    print(f"[history] appended {row['commit']} "
          f"(flash {row['flash_used']}, ram {row['ram_used']}) "
          f"-> {HISTORY_CSV.relative_to(REPO_ROOT)}")


def backfill_history():
    revs = commits_with_tracked_elf()
    if not revs:
        sys.exit("error: no commit in history tracks Debug/WHAM-XREX-PFMG474.elf")

    rows = []
    with tempfile.TemporaryDirectory() as tmp:
        for rev in revs:
            try:
                elf_path = extract_elf_at(rev, tmp)
            except subprocess.CalledProcessError:
                print(f"[skip] {rev[:10]}: no tracked elf at this commit")
                continue
            snap = snapshot(elf_path)
            subject = commit_subject(rev)
            date = commit_date(rev)
            rows.append(history_row(rev, subject, date, snap))
            print(f"[backfill] {rev[:10]}  {date}  "
                  f"flash={snap['totals']['flash_used']:,}  "
                  f"ram={snap['totals']['ram_used']:,}  -- {subject}")

    rows.sort(key=lambda r: r["date"])
    write_history(rows)
    print(f"\n[backfill] wrote {len(rows)} rows -> "
          f"{HISTORY_CSV.relative_to(REPO_ROOT)}")


# --------------------------------------------------------------------------
# HTML report -- reads docs/memory_history.csv (already committed history,
# see `history` above) and renders docs/memory_report.html: a self-
# contained, no-external-dependency (beyond Google Fonts) dashboard --
# safe to open straight from a checkout (file://) or publish as-is.
# Template lives alongside this script as memory_report_template.html;
# data is embedded as one inline JSON <script> block, rendered client-
# side with vanilla JS/SVG (crosshair tooltip on the trend chart,
# per-bar tooltips on the breakdown charts) -- no build step, no CDN.
# --------------------------------------------------------------------------
REPORT_TEMPLATE = Path(__file__).resolve().parent / "memory_report_template.html"
REPORT_HTML = REPO_ROOT / "docs" / "memory_report.html"


def _module_breakdown_from_row(row, prefix):
    out = []
    for key, value in row.items():
        if key.startswith(prefix) and value not in ("", None):
            out.append({"name": key[len(prefix):], "bytes": int(value)})
    out.sort(key=lambda m: -m["bytes"])
    return out


def render_report():
    rows = load_history()
    if not rows:
        sys.exit(f"error: {HISTORY_CSV.relative_to(REPO_ROOT)} is empty or "
                  f"missing -- run `history --backfill` (or `history`) first")

    history_json = [
        {
            "commit": r["commit"],
            "date": r["date"],
            "subject": r["subject"],
            "flash_used": int(r["flash_used"]),
            "flash_total": int(r["flash_total"]),
            "ram_used": int(r["ram_used"]),
            "ram_total": int(r["ram_total"]),
        }
        for r in rows
    ]
    latest = rows[-1]
    data = {
        "history": history_json,
        "flash_modules": _module_breakdown_from_row(latest, "flash:"),
        "ram_modules": _module_breakdown_from_row(latest, "ram:"),
    }

    flash_used, flash_total = int(latest["flash_used"]), int(latest["flash_total"])
    ram_used, ram_total = int(latest["ram_used"]), int(latest["ram_total"])

    template = REPORT_TEMPLATE.read_text()
    substitutions = {
        "__LATEST_COMMIT__": latest["commit"],
        "__LATEST_SUBJECT__": latest["subject"],
        "__FIRST_DATE__": rows[0]["date"][:10],
        "__LAST_DATE__": rows[-1]["date"][:10],
        "__COMMIT_COUNT__": str(len(rows)),
        "__FLASH_PCT__": f"{100 * flash_used / flash_total:.1f}",
        "__FLASH_USED_KB__": f"{flash_used / 1024:.1f}",
        "__FLASH_USED_B__": f"{flash_used:,}",
        "__FLASH_TOTAL_B__": f"{flash_total:,}",
        "__FLASH_REMAIN_KB__": f"{(flash_total - flash_used) / 1024:.1f}",
        "__RAM_PCT__": f"{100 * ram_used / ram_total:.1f}",
        "__RAM_USED_KB__": f"{ram_used / 1024:.1f}",
        "__RAM_USED_B__": f"{ram_used:,}",
        "__RAM_TOTAL_B__": f"{ram_total:,}",
        "__RAM_REMAIN_KB__": f"{(ram_total - ram_used) / 1024:.1f}",
        "__DATA_JSON__": json.dumps(data),
    }
    html = template
    for token, value in substitutions.items():
        html = html.replace(token, value)

    REPORT_HTML.parent.mkdir(parents=True, exist_ok=True)
    REPORT_HTML.write_text(html)
    print(f"[report] wrote {REPORT_HTML.relative_to(REPO_ROOT)} "
          f"({len(rows)} history rows, "
          f"{len(data['flash_modules'])} flash / {len(data['ram_modules'])} ram modules)")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    sp_snap = sub.add_parser("snapshot", help="print a flash/RAM snapshot")
    sp_snap.add_argument("--elf", default=str(DEFAULT_ELF))
    sp_snap.add_argument("--json", action="store_true")

    sp_hist = sub.add_parser("history", help="append/backfill docs/memory_history.csv")
    sp_hist.add_argument("--elf", default=str(DEFAULT_ELF))
    sp_hist.add_argument("--backfill", action="store_true",
                          help="rebuild history from every commit with a "
                               "tracked Debug/WHAM-XREX-PFMG474.elf, instead "
                               "of appending one row for HEAD")

    sub.add_parser("report", help="render docs/memory_report.html from "
                                   "docs/memory_history.csv")

    args = ap.parse_args()

    if args.cmd == "snapshot":
        elf_path = Path(args.elf)
        if not elf_path.exists():
            sys.exit(f"error: {elf_path} not found -- build first")
        snap = snapshot(elf_path)
        if args.json:
            print(json.dumps(snap, indent=2))
        else:
            print_snapshot_human(snap)

    elif args.cmd == "history":
        if args.backfill:
            backfill_history()
        else:
            elf_path = Path(args.elf)
            if not elf_path.exists():
                sys.exit(f"error: {elf_path} not found -- build first")
            append_history_for_current(elf_path)

    elif args.cmd == "report":
        render_report()


if __name__ == "__main__":
    main()
