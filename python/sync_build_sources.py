#!/usr/bin/env python3
"""sync_build_sources.py -- keep the build in step with the source tree.

The firmware sources live in Core/ (CubeMX-generated files only) and src/
(everything hand-written, see AGENTS.md "Source layout"). CubeIDE's
generated Debug/ makefiles and .cproject are not refreshed by a plain
`make`, so adding, moving or renaming a .c/.h file outside the IDE used to
mean hand-editing several files. This script derives everything from the
tree instead:

  - source folders: Core/Src plus every folder under src/ holding a .c file
  - include folders: Core/Inc, every folder under src/ holding a .h file,
    build/generated (git_version.h, build_target.h), then the HAL/CMSIS ones

and rewrites, from those lists:

  - Debug/<folder>/subdir.mk for every source folder (created or removed as
    folders come and go), and the -I list in every compile rule
  - Debug/sources.mk (SUBDIRS), Debug/makefile (-include lines),
    Debug/objects.list (link list)
  - .cproject: C include paths and source folders, both configurations, so
    the IDE build matches the command-line build

    python3 python/sync_build_sources.py          # rewrite what is out of date
    python3 python/sync_build_sources.py --check  # exit 1 if anything is

Core/Startup and Drivers/ are left as CubeIDE generated them, apart from
the -I list.
"""
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEBUG = os.path.join(ROOT, "Debug")
FIXED_INCLUDES = ["Drivers/STM32G4xx_HAL_Driver/Inc", "Drivers/STM32G4xx_HAL_Driver/Inc/Legacy",
                  "Drivers/CMSIS/Device/ST/STM32G4xx/Include", "Drivers/CMSIS/Include"]
GENERATED_INCLUDE = "build/generated"
FIXED_SUBDIRS = ["Core/Startup", "Drivers/STM32G4xx_HAL_Driver/Src"]   # CubeIDE-owned, untouched lists
TEMPLATE_DIR = "Core/Src"      # its compile rule supplies the flags for every generated subdir.mk


def src_folders(ext):
    found = set()
    for base, dirs, files in os.walk(os.path.join(ROOT, "src")):
        dirs.sort()
        if any(f.endswith(ext) for f in files):
            found.add(os.path.relpath(base, ROOT))
    return sorted(found)


def source_dirs():
    return ["Core/Src"] + src_folders(".c")


def include_dirs():
    return ["Core/Inc"] + src_folders(".h") + [GENERATED_INCLUDE] + FIXED_INCLUDES


def mangle(d):
    return d.replace("/", "-2f-")


def c_files(d):
    return sorted(f[:-2] for f in os.listdir(os.path.join(ROOT, d)) if f.endswith(".c"))


def with_includes(rule_line, incs):
    """Replace the run of -I flags in a compile command with incs."""
    flags = " ".join(f"-I../{i}" for i in incs)
    return re.sub(r"(?: -I\S+)+", " " + flags, rule_line, count=1)


def template_flags():
    text = open(os.path.join(DEBUG, TEMPLATE_DIR, "subdir.mk")).read()
    m = re.search(r"^\tarm-none-eabi-gcc \"\$<\".*$", text, re.M)
    assert m, "compile rule not found in Debug/Core/Src/subdir.mk"
    return m.group(0)


def subdir_mk(d, rule):
    names = c_files(d)
    def block(var, fmt):
        return f"{var} += \\\n" + " \\\n".join(fmt.format(n) for n in names) + "\n"
    clean = " ".join(f"./{d}/{n}.{e}" for n in names for e in ("cyclo", "d", "o", "su"))
    return ("################################################################################\n"
            "# Automatically-generated file. Do not edit!\n"
            "# Toolchain: GNU Tools for STM32 (13.3.rel1)\n"
            "################################################################################\n\n"
            "# Add inputs and outputs from these tool invocations to the build variables \n"
            + block("C_SRCS", "../" + d + "/{}.c") + "\n"
            + block("OBJS", "./" + d + "/{}.o") + "\n"
            + block("C_DEPS", "./" + d + "/{}.d") + "\n\n"
            "# Each subdirectory must supply rules for building sources it contributes\n"
            f"{d}/%.o {d}/%.su {d}/%.cyclo: ../{d}/%.c {d}/subdir.mk\n"
            f"{rule}\n\n"
            f"clean: clean-{mangle(d)}\n\n"
            f"clean-{mangle(d)}:\n"
            f"\t-$(RM) {clean}\n\n"
            f".PHONY: clean-{mangle(d)}\n\n")


def plan():
    """Return {path: new_text} for every managed file, plus paths to delete."""
    out, delete = {}, []
    srcs, incs = source_dirs(), include_dirs()
    rule = with_includes(template_flags(), incs)

    for d in srcs:
        out[os.path.join(DEBUG, d, "subdir.mk")] = subdir_mk(d, rule)
    for base, _, files in os.walk(os.path.join(DEBUG, "src")):
        d = os.path.relpath(base, DEBUG)
        if "subdir.mk" in files and d not in srcs:
            delete.append(os.path.join(base, "subdir.mk"))
    for d in FIXED_SUBDIRS:
        p = os.path.join(DEBUG, d, "subdir.mk")
        text = open(p).read()
        out[p] = re.sub(r"^\tarm-none-eabi-gcc \"\$<\".*$", lambda m: with_includes(m.group(0), incs),
                        text, flags=re.M)

    p = os.path.join(DEBUG, "sources.mk")
    subdirs = srcs[:1] + FIXED_SUBDIRS + srcs[1:]
    out[p] = re.sub(r"^SUBDIRS := \\\n(?:.*\\\n)*\n",
                    "SUBDIRS := \\\n" + "".join(f"{d} \\\n" for d in subdirs) + "\n",
                    open(p).read(), count=1, flags=re.M)

    p = os.path.join(DEBUG, "makefile")
    text = open(p).read()
    lines = "".join(f"-include {d}/subdir.mk\n" for d in reversed(subdirs))
    out[p] = re.sub(r"(-include sources\.mk\n)(?:-include \S+/subdir\.mk\n)*", lambda m: m.group(1) + lines,
                    text, count=1)

    objs = [f'"./{d}/{n}.o"' for d in srcs for n in c_files(d)]
    p = os.path.join(DEBUG, "objects.list")
    old = open(p).read().splitlines() if os.path.exists(p) else []
    fixed = [l for l in old if any(l.startswith(f'"./{d}/') for d in FIXED_SUBDIRS)]
    out[p] = "\n".join(objs + fixed) + "\n"

    p = os.path.join(ROOT, ".cproject")
    text = open(p).read()
    def inc_block(m):
        indent = re.search(r"\n(\t+)<listOptionValue", m.group(0)).group(1)
        body = "".join(f'\n{indent}<listOptionValue builtIn="false" value="../{i}"/>' for i in incs)
        return m.group(1) + body + m.group(3)
    text = re.sub(r'(<option [^>]*c\.compiler\.option\.includepaths[^>]*>)((?:\s*<listOptionValue[^>]*/>)+)(\s*</option>)',
                  inc_block, text)
    def src_block(m):
        indent = m.group(2)
        return m.group(1) + "".join(
            f'{indent}<entry flags="VALUE_WORKSPACE_PATH|RESOLVED" kind="sourcePath" name="{n}"/>'
            for n in ("Core", "Drivers", "src")) + m.group(3)
    text = re.sub(r"(<sourceEntries>)((?:\s*)(?=<entry))(?:\s*<entry [^>]*/>)+(\s*</sourceEntries>)",
                  src_block, text)
    out[p] = text
    return out, delete


def main():
    check = "--check" in sys.argv
    out, delete = plan()
    stale = [p for p, t in out.items() if not os.path.exists(p) or open(p).read() != t] + delete
    if check:
        for p in stale:
            print("out of date:", os.path.relpath(p, ROOT))
        print("up to date" if not stale else f"{len(stale)} file(s) out of date")
        sys.exit(1 if stale else 0)
    for p in stale:
        if p in delete:
            os.remove(p)
            continue
        os.makedirs(os.path.dirname(p), exist_ok=True)
        open(p, "w").write(out[p])
    print(f"{len(source_dirs())} source folders, {len(include_dirs())} include folders; "
          + (f"updated {len(stale)} file(s)" if stale else "already up to date"))


if __name__ == "__main__":
    main()
