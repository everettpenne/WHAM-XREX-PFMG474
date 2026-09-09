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
