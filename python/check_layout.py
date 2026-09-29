#!/usr/bin/env python3
"""check_layout.py -- enforce the firmware source layout (AGENTS.md, "Source
layout"). Run by wham_build.py before every build; exits 1 on a violation.

    python3 python/check_layout.py

Layers and what each may include:

  Core/                CubeMX-generated files only (allowlist below). The
                       composition root: main() and the IRQ handlers may call
                       into any layer.
  src/config/          project-wide compile-time configuration. Headers only;
                       may include std headers and build/generated/ only.
  src/drivers/         hardware-independent interfaces. Headers only; std,
                       config, other drivers headers.
  src/bsp/<chip>/      the only code that touches the MCU: HAL, registers,
                       interrupts. May include HAL/CMSIS, Core/Inc, drivers,
                       config, its own headers. Never app or middleware.
  src/middleware/      hardware- and application-independent logic (the SCPI
                       parser). std, config, drivers, middleware.
  src/app/             the product: control, protection, commands, tasks.
                       std, config, drivers, middleware, app.

Outside src/bsp and Core, code (comments and strings excluded) must not use
HAL calls or types, register access (RCC->, GPIOE->, ...), GPIO pin/port
names, IRQ numbers, or interrupt/barrier intrinsics -- go through a
src/drivers interface instead.
"""
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

CORE_ALLOWED = {
    "Core/Src/main.c", "Core/Src/stm32g4xx_hal_msp.c", "Core/Src/stm32g4xx_it.c",
    "Core/Src/syscalls.c", "Core/Src/sysmem.c", "Core/Src/system_stm32g4xx.c",
    "Core/Inc/main.h", "Core/Inc/stm32g4xx_hal_conf.h", "Core/Inc/stm32g4xx_it.h",
    "Core/Startup/startup_stm32g474qetx.s",
}
SRC_TOP = {"app", "config", "drivers", "middleware", "bsp"}
HEADERS_ONLY = {"config", "drivers"}

MAY_INCLUDE = {
    "config": {"std", "config", "generated"},
    "drivers": {"std", "config", "generated", "drivers"},
    "middleware": {"std", "config", "generated", "drivers", "middleware"},
    "app": {"std", "config", "generated", "drivers", "middleware", "app"},
    "bsp": {"std", "config", "generated", "drivers", "bsp", "hal", "core"},
    "core": None,   # composition root -- anything
}

HW_TOKENS = [
    (r"\b(?:__)?HAL_\w+", "HAL call/macro"),
    (r"\b\w+_HandleTypeDef\b", "HAL handle type"),
    (r"\b\w+_TypeDef\b", "CMSIS register type"),
    (r"\bGPIO_(?:PIN|MODE|PULL|SPEED|AF)\w*", "GPIO pin/mode constant"),
    (r"\bGPIO[A-G]\b", "GPIO port"),
    (r"\b[A-Z][A-Z0-9_]*\s*->\s*[A-Z][A-Z0-9_]*\b", "register access"),
    (r"\b__(?:disable_irq|enable_irq|get_PRIMASK|set_PRIMASK|DSB|ISB|DMB|NOP|WFI|WFE)\b",
     "interrupt/barrier intrinsic"),
    (r"\b\w+_IRQn\b", "IRQ number"),
]


def strip_c(text):
    """Blank out comments and string/char literals, keeping line numbers."""
    out, i, n = [], 0, len(text)
    while i < n:
        c = text[i]
        if text.startswith("/*", i):
            j = text.find("*/", i + 2)
            j = n if j < 0 else j + 2
            out.append(re.sub(r"[^\n]", " ", text[i:j])); i = j
        elif text.startswith("//", i):
            j = text.find("\n", i)
            j = n if j < 0 else j
            out.append(" " * (j - i)); i = j
        elif c in "\"'":
            j = i + 1
            while j < n and text[j] != c:
                j += 2 if text[j] == "\\" else 1
            out.append(c + " " * (j - i - 1) + c); i = j + 1
        else:
            out.append(c); i += 1
    return "".join(out)


def layer_of(rel):
    if rel.startswith("Core/"):
        return "core"
    if rel.startswith("build/generated/"):
        return "generated"
    if rel.startswith("Drivers/"):
        return "hal"
    if rel.startswith("src/"):
        return rel.split("/")[1]
    return None


def header_index():
    idx = {}
    for top in ("Core/Inc", "src", "build/generated"):
        for base, _, files in os.walk(os.path.join(ROOT, top)):
            for f in files:
                if f.endswith(".h"):
                    idx.setdefault(f, []).append(os.path.relpath(os.path.join(base, f), ROOT))
    for base, _, files in os.walk(os.path.join(ROOT, "Drivers")):
        for f in files:
            if f.endswith(".h"):
                idx.setdefault(f, []).append(os.path.relpath(os.path.join(base, f), ROOT))
    return idx


def include_layer(name, idx):
    if name in ("git_version.h", "build_target.h"):
        return "generated"
    paths = idx.get(os.path.basename(name))
    if not paths:
        return "std"
    return layer_of(paths[0])


def main():
    problems = []
    idx = header_index()

    for top in ("Core",):
        for base, _, files in os.walk(os.path.join(ROOT, top)):
            for f in files:
                rel = os.path.relpath(os.path.join(base, f), ROOT)
                if not f.startswith(".") and rel not in CORE_ALLOWED:
                    problems.append(f"{rel}: not a CubeMX-generated file -- hand-written code "
                                    "belongs under src/ (AGENTS.md, Source layout)")

    for name, paths in idx.items():
        own = [p for p in paths if not p.startswith("Drivers/")]
        if len(own) > 1:
            problems.append(f"{', '.join(own)}: duplicate header name (includes are by bare name)")

    src = os.path.join(ROOT, "src")
    for entry in sorted(os.listdir(src)):
        if os.path.isdir(os.path.join(src, entry)) and entry not in SRC_TOP:
            problems.append(f"src/{entry}: unknown top-level folder (allowed: {sorted(SRC_TOP)})")

    for base, _, files in os.walk(src):
        for f in sorted(files):
            if not f.endswith((".c", ".h")):
                continue
            rel = os.path.relpath(os.path.join(base, f), ROOT)
            layer = layer_of(rel)
            if layer in HEADERS_ONLY and f.endswith(".c"):
                problems.append(f"{rel}: src/{layer}/ holds headers only")
            if layer == "bsp" and len(rel.split("/")) < 4:
                problems.append(f"{rel}: BSP code goes in src/bsp/<chip>/")
            raw = open(os.path.join(ROOT, rel), encoding="utf-8", errors="replace").read()
            code = strip_c(raw)
            allowed = MAY_INCLUDE.get(layer)
            for m in re.finditer(r'^[ \t]*#[ \t]*include[ \t]*([<"])([^>"]+)[>"]', raw, re.M):
                line = raw.count("\n", 0, m.start()) + 1
                if code.split("\n")[line - 1].strip() == "":
                    continue            # inside a comment
                name = m.group(2)
                inc = "hal" if re.match(r"(stm32|core_cm|cmsis)", os.path.basename(name)) \
                    else include_layer(name, idx)
                if allowed is not None and inc not in allowed:
                    problems.append(f"{rel}:{line}: src/{layer}/ may not include {name} ({inc})")
            if layer == "bsp":
                continue
            for lineno, text in enumerate(code.split("\n"), 1):
                if text.lstrip().startswith("#"):
                    continue
                for rx, what in HW_TOKENS:
                    m = re.search(rx, text)
                    if m:
                        problems.append(f"{rel}:{lineno}: {what} '{m.group(0)}' outside src/bsp "
                                        "-- use a src/drivers interface")
                        break

    for p in problems:
        print(p)
    print(f"check_layout: {len(problems)} problem(s)" if problems else "check_layout: OK")
    sys.exit(1 if problems else 0)


if __name__ == "__main__":
    main()
