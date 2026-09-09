#ifndef INC_CTRLR_CONFIG_H_
#define INC_CTRLR_CONFIG_H_

/*
 * ctrlr_config.h
 *
 * The project's one compile-time config file -- board/firmware
 * IDENTITY (HW_BOARD_NAME/REV, FW_VERSION_STRING) and general
 * controller-BEHAVIOR settings (HRTIM_NUM_CHANNELS, GDS_FAULT_POLARITY)
 * together, matching the sibling PFM-STM32G474 project's single
 * supply_config.h. This was briefly split (2026-09-08: identity stayed
 * in version.h, behavior moved here) on the reasoning that identity and
 * behavior are different KINDS of thing -- re-condensed the same day,
 * per project decision, back to one file. version.h no longer exists;
 * don't recreate it.
 *
 * (Module-local, self-contained flags are a further, older exception --
 * e.g. BOOT_JUMP_FEATURE_ENABLED lives in boot_jump.h itself, because
 * that module's whole design point is being independently removable in
 * one place. Don't move flags like that here.)
 */

/* --------------------------------------------------------------------------
 * Board/product identity. HW_BOARD_REV is a placeholder -- set it to
 * match this build's actual PCB silkscreen revision and bump it on
 * every hardware respin so *IDN's report always matches what's
 * physically in hand.
 * -------------------------------------------------------------------------- */
#define HW_BOARD_NAME     "WHAM-XREX-PFMG474"
#define HW_BOARD_REV      "REVA"

/* Firmware identity. Bump on every release the way the sibling
 * PFM-STM32G474 project does (FW_VERSION_STRING in its
 * supply_config.h). Reset to v0.1 at this project's own start
 * (2026-09-09, reseeded from WHAM-PFMG474-V4 at commit de83324 -- see
 * AGENTS.md and docs/changelog.txt) -- not a continuation of that
 * project's v0.6 lineage, a fresh one for this one. */
#define FW_VERSION_STRING "v0.1"

/* --------------------------------------------------------------------------
 * HRTIM channel count
 *
 * docs/pin_mapping_v4.csv wires out all 6 HRTIM1 channel pairs (A-F /
 * U,V,W,X,Y,Z) on this board. HRTIM_NUM_CHANNELS selects how many of
 * them hrtim.c/pfm.c actually drive, phase-locked and evenly spaced
 * around a shared carrier, as a real compile-time setting
 * (2026-09-08) -- fix it here, rebuild, reflash. It is NOT modifiable
 * at runtime, by design: no variable backs this anywhere, only this
 * #define, so "changing it after the firmware is flashed" isn't a
 * thing that can happen short of building and flashing a different
 * firmware image.
 *
 * Range is 1-5, not 1-6: the HRTIM Master timer has exactly 4 compare
 * registers (MCMP1R-MCMP4R), giving 5 total trigger points (its own
 * PER event + 4 CMPs) -- enough for channels A-E phase-locked to the
 * Master, but not a 6th (F) without a genuinely different (cross-timer)
 * sync scheme. That scheme is NOT implemented here -- the HAL does
 * expose the register support for it (HRTIM_TIMRESETTRIGGER_OTHERx_
 * CMPy, letting one slave timer's reset trigger chain off another
 * slave's compare event instead of the Master's), but it needs new
 * phase math with no existing pattern in this codebase to build from,
 * and was deliberately left as a follow-up rather than bundled into
 * this change -- see AGENTS.md.
 *
 * Channels beyond HRTIM_NUM_CHANNELS (up through Timer F) stay
 * pin/dead-time-reserved but unlocked (ResetTrigger = NONE) and are
 * never started by HRTIM1_PWM_Start() -- exactly how Timers D/E/F
 * behaved before this config existed for the N=3 default. See
 * HRTIM1_FullInit()'s own comments in hrtim.c for the exact mechanism.
 *
 * Raising this value automatically extends the 2026-09-09 HRTIM
 * SET/RESET-collision fix (hrtim.c's HRTIM1_FullInit(), the CMP3-based
 * output SET source) to whichever channels newly become active --
 * verified, not assumed: every place that fix touches (the CMP3
 * compare-register config loop, the per-channel SetSource selector,
 * and HRTIM1_PWM_Start()'s cold-start reforceActive handling) is keyed
 * off this constant, not a hardcoded channel count. No further code
 * changes are needed here when raising N; just re-verify the new
 * channel(s) on real hardware the same way U/V/W were (frequency-ramp
 * table + DSLogic capture -- see docs/changelog.txt), since this fix
 * was derived from real-hardware evidence, not proven in general.
 * -------------------------------------------------------------------------- */
#define HRTIM_NUM_CHANNELS  (3U)

#if (HRTIM_NUM_CHANNELS < 1U) || (HRTIM_NUM_CHANNELS > 5U)
#error "HRTIM_NUM_CHANNELS must be 1-5 -- see the comment above it in " \
       "ctrlr_config.h for why 6 isn't a simple extension of this scheme."
#endif

/* --------------------------------------------------------------------------
 * GateDriverStatus fault polarity (compile-time)
 *
 * GDS_NORMALLY_HIGH -- pins read HIGH in good operation;
 *                      a LOW reading is a fault.
 * GDS_NORMALLY_LOW  -- pins read LOW in good operation;
 *                      a HIGH reading is a fault.
 *
 * Governs both gate_driver.c's GateDriver_CheckFault() (the EXTI-driven
 * fault interrupt on PE0..PE11, see gate_driver.h) and nothing else --
 * GDS? (commands.c) is a raw, polarity-agnostic HIGH/LOW readback and
 * does not consult this value.
 *
 * Set to NORMALLY_LOW per direct confirmation against this board's
 * actual gate driver ICs, 2026-09-08 -- matching the sibling
 * PFM-STM32G474 project's own configured default (GDS_FAULT_POLARITY =
 * GDS_NORMALLY_LOW in supply_config.h), even though that project's own
 * comment calls NORMALLY_HIGH "the safest" choice in the abstract (a
 * disconnected pin reads LOW and immediately trips) -- the actual wiring
 * on real hardware is the deciding fact here, not the abstract argument.
 *
 * Known consequence, confirmed before choosing this value, not
 * discovered by surprise: a GDS? snapshot taken earlier this session
 * showed GateDriverStatus_03 (PE2) already reading HIGH while the other
 * 11 pins read LOW. Under NORMALLY_LOW, that pin is *already* a fault
 * condition -- expect GateDriver_CheckFault() to latch a fault on it as
 * soon as this interrupt is live (very possibly immediately at boot, if
 * that pin is still HIGH by then). That may well be surfacing a real,
 * previously-invisible fault condition, which is the whole point of
 * this feature -- see docs/changelog.txt.
 *
 * Single source of truth, unlike the sibling project's own documented
 * history (supply_config.h's own comment describes GDS_FAULT_POLARITY
 * once being defined twice -- once there, once in gate_driver.h -- with
 * gate_driver.h's #ifndef silently winning the race every time,
 * making the copy in supply_config.h dead). Not a risk here structurally:
 * gate_driver.h includes THIS file rather than defining anything
 * itself, and this is the only place GDS_NORMALLY_HIGH/LOW/
 * GDS_FAULT_POLARITY are ever defined.
 * -------------------------------------------------------------------------- */
#define GDS_NORMALLY_HIGH    (0U)
#define GDS_NORMALLY_LOW     (1U)

#define GDS_FAULT_POLARITY   GDS_NORMALLY_LOW

/* --------------------------------------------------------------------------
 * PFM carrier frequency ceiling (compile-time, hard limit)
 *
 * Added 2026-09-09 after a real-hardware frequency-ramp capture test
 * (see docs/changelog.txt) found that this board's PWM output can
 * intermittently glitch on a frequency-CHANGING table -- an occasional
 * genuinely missing real pulse on the wire, confirmed via DSLogic, not
 * just a capture-side artifact -- starting somewhere around 85 kHz and
 * getting more frequent up toward 140 kHz. Root cause is an open
 * investigation (a suspected HRTIM shadow/preload-register race in how
 * PFM_CycleBoundaryHandler applies each new table entry -- see
 * docs/changelog.txt's 2026-09-09 entries). Until that's actually
 * fixed and re-verified, every TABLE:STEP entry's implied carrier
 * frequency is capped here and rejected with ERR 10
 * (commands.c's cmd_table_step()) rather than silently accepted and
 * possibly glitching.
 *
 * 100 kHz is the documented, tested boundary -- every constant-100 kHz
 * test this project has run, going back to the very first PWM
 * bring-up, has been clean, and the ramp test's first several steps
 * (all below 85 kHz) never glitched either. This is NOT a claim that
 * 100 kHz is provably safe in general, just the boundary this project
 * has actual evidence for -- raise it only after the shadow/preload
 * fix lands and is itself re-verified on real hardware at whatever new
 * ceiling is proposed.
 * -------------------------------------------------------------------------- */
#define PFM_MAX_CARRIER_FREQ_HZ  (100000UL)

#endif /* INC_CTRLR_CONFIG_H_ */
