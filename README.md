# WHAM-XREX-PFMG474

Closed-loop PID controller firmware for the WHAM Transrex ISR-2126 magnet
power supplies (four independent units), on an STM32G474QET6.

Reseeded from [WHAM-PFMG474-V4](https://github.com/everettpenne/WHAM-PFMG474-V4)
(2026-09-09) -- same board, same pin mapping, same STM32G474QET6 -- as an
independent repository, not a GitHub fork. See
[`AGENTS.md`](AGENTS.md) for the full orientation (what's inherited vs.
new, why it's a separate repo, the architecture this project is being
built around) and [`docs/changelog.txt`](docs/changelog.txt) for the
dated decision record, starting with the design discussion that shaped
this project before any code existed.

The requirements driving this project live in [`docs/Transrex/`](docs/Transrex/).

## Layout

```
Core/        CubeMX-generated init only (main.c, IRQ handlers, startup)
src/app/     the product: control, protection, commands, main-loop tasks
src/middleware/scpi/   SCPI parser (no hardware)
src/drivers/ hardware-independent interfaces (headers)
src/bsp/stm32g4/       the only code that touches the MCU
src/config/  compile-time configuration
tests/       off-target unit tests (run on the PC)
build/       built images (from wham_build.py)
```

The rules that keep it this way are in [`AGENTS.md`](AGENTS.md), "Source
layout"; `python/check_layout.py` enforces them on every build.

```bash
python3 python/wham_build.py                     # controller -> build/WHAM-XREX-PFMG474.bin
python3 python/wham_build.py --target simulator  # simulator  -> build/WHAM-XREX-PFMG474-SIM.bin
make -C tests                                    # unit tests only
```
