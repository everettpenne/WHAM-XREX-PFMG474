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
 * REPURPOSED, 2026-09-09, for the closed-loop PID architecture (see
 * pid.h and docs/changelog.txt's design-decision entry): this used to
 * mean "how many channels are phase-locked and evenly spaced around a
 * shared carrier" (WHAM-PFMG474-V4's switching-supply meaning, and
 * still literally true of the code this was reseeded from). It now
 * means "how many of the 4 Transrex channels are active" --
 * HRTIM_NUM_CHANNELS independent PFM output channels (Possibility 3:
 * per-channel HRTIM update-on-own-rollover, ResetTrigger=NONE,
 * UpdateTrigger=NONE, ResetUpdate=ENABLED), each running its own PID
 * loop against its own PFM_Input feedback channel, with NO phase
 * relationship between them at all. docs/pin_mapping_v4.csv wires out
 * all 6 HRTIM1 channel pairs (A-F / U,V,W,X,Y,Z) on this board; this
 * selects how many of them PID.c actually drives, as a real
 * compile-time setting -- fix it here, rebuild, reflash, same
 * not-runtime-modifiable reasoning as before.
 *
 * 4, not the sibling project's default of 3 -- the Transrex spec (this
 * project's whole reason for existing, docs/Transrex/) is 4
 * independent units. Range kept at 1-5 (unchanged from before): the
 * HRTIM Master timer's own repetition event is what drives the PID
 * heartbeat now (see pid.h), so the OLD 1-5 constraint (Master's 4
 * compare registers, MCMP1R-MCMP4R, phase-locking channels A-E) no
 * longer actually applies here -- Master has no per-channel phase
 * relationship left to run out of registers for. Left at 5 anyway
 * (not widened to 6) simply because nobody has re-verified a 6th
 * independent channel on this board yet, not because of the old
 * register-count reason -- raise it if a real need for a 5th/6th
 * Transrex channel ever shows up, after checking HRTIM1_FullInit()'s
 * unconditional channels-0..5 loops still make sense at N=6.
 *
 * Channels beyond HRTIM_NUM_CHANNELS (up through Timer F) stay
 * pin/dead-time-reserved but unlocked and are never started -- exactly
 * as before this repurposing, see HRTIM1_FullInit()'s own comments.
 * -------------------------------------------------------------------------- */
#define HRTIM_NUM_CHANNELS  (4U)

#if (HRTIM_NUM_CHANNELS < 1U) || (HRTIM_NUM_CHANNELS > 5U)
#error "HRTIM_NUM_CHANNELS must be 1-5 -- see the comment above it in " \
       "ctrlr_config.h."
#endif

/* --------------------------------------------------------------------------
 * PID control-loop heartbeat rate (compile-time)
 *
 * Added 2026-09-09 alongside the closed-loop PID architecture (pid.c,
 * docs/changelog.txt's design-decision entry) -- the HRTIM Master
 * repetition interrupt's rate while PID mode is running, DELIBERATELY
 * decoupled from any channel's own carrier/demand frequency (see that
 * changelog entry for the full reasoning: fixed PID sample time,
 * control bandwidth != V-to-F encoding rate). 1 kHz is a first,
 * conservative starting point -- tens of Hz to low kHz is the normal
 * range for a magnet-supply current loop; this has NOT been tuned
 * against real Transrex/magnet electrical time constants yet, revisit
 * once real bench data exists. Must yield a Master PER that fits
 * HRTIM's 16-bit register at the prescaler pid.c actually configures
 * (see HRTIM1_ConfigPidHeartbeat() in hrtim.c) -- 1 kHz at /4 prescale
 * is exactly 42500 counts (170 MHz / 4 / 1000), comfortably inside
 * 16 bits with headroom to go slower still.
 * -------------------------------------------------------------------------- */
#define PID_LOOP_RATE_HZ   (1000UL)

/* --------------------------------------------------------------------------
 * PID output frequency bounds (compile-time, hard limit)
 *
 * Added 2026-09-09 alongside the closed-loop PID architecture (pid.c)
 * -- clamps every channel's commanded output frequency (and, via
 * PID_SetSetpoint(), its setpoint) to a range HRTIM can actually
 * represent on this hardware, at the /1 prescale each channel's own
 * timer uses (hrtim.c's HRTIM1_FullInit()).
 *
 * PID_OUTPUT_MIN_HZ is a REAL hardware floor, not a tuning choice: at
 * /1 prescale, PER = HRTIM_TIMER_CLK_HZ/freq - 1 must fit in HRTIM's
 * 16-bit PER register (max 65535) -- solving for freq gives
 * 170000000/65536 = ~2594.9 Hz as the absolute lowest representable
 * frequency. Set to 3000 Hz, comfortably clear of that floor (not
 * flirting with an off-by-one at the exact boundary).
 *
 * PID_OUTPUT_MAX_HZ is NOT a hardware limit at this end (170 kHz would
 * still fit, PER=999) -- 150 kHz is a conservative starting ceiling,
 * chosen to sit within the range this exact codebase has already
 * proven clean on real hardware (its own PFM_MAX_CARRIER_FREQ_HZ,
 * below, documents 100 kHz tested clean; this is somewhat above that,
 * unverified for THIS use case -- revisit once real Transrex/magnet
 * operating-point data exists, per docs/changelog.txt's own
 * "explicitly NOT yet resolved" list on this exact point).
 * -------------------------------------------------------------------------- */
#define PID_OUTPUT_MIN_HZ  (3000UL)
#define PID_OUTPUT_MAX_HZ  (150000UL)

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
