/*
 * board_io.c -- STM32G4 implementation of drivers/board_io.h: this board's
 * hand-written GPIO configuration (every pin CubeMX's MX_GPIO_Init()
 * doesn't generate -- fault inputs, fiber outputs, interlock and
 * diagnostic pins) and the named-signal accessors. BoardIo_Init() is
 * called from MX_GPIO_Init()'s USER CODE block, so a CubeMX regeneration
 * leaves it alone. Pin roles: docs/pin_mapping_v4.csv.
 */
#include "board_io.h"
#include "main.h"
#include "ctrlr_config.h"

typedef struct
{
    GPIO_TypeDef *port;
    uint16_t      pin;
} board_pin_t;

/* Indexed by board_signal_t -- keep in the enum's order. OCP order is the
   board's own irregular one (XR1..XR4 = PF4, PF8, PF12, PF5), verified
   against docs/pin_mapping_v4.csv's "XREX Pin Name" column, 2026-09-17. */
static const board_pin_t kPins[BOARD_SIG_COUNT] =
{
    [BOARD_SIG_EXT_ENABLE]        = { GPIOF, GPIO_PIN_13 },
    [BOARD_SIG_EXT_TRIGGER]       = { GPIOF, GPIO_PIN_15 },
    [BOARD_SIG_XR1_OCP]           = { GPIOF, GPIO_PIN_4  },
    [BOARD_SIG_XR2_OCP]           = { GPIOF, GPIO_PIN_8  },
    [BOARD_SIG_XR3_OCP]           = { GPIOF, GPIO_PIN_12 },
    [BOARD_SIG_XR4_OCP]           = { GPIOF, GPIO_PIN_5  },
    [BOARD_SIG_XR1_ENA_OUT]       = { GPIOG, GPIO_PIN_0  },
    [BOARD_SIG_XR2_ENA_OUT]       = { GPIOG, GPIO_PIN_1  },
    [BOARD_SIG_XR3_ENA_OUT]       = { GPIOG, GPIO_PIN_2  },
    [BOARD_SIG_XR4_ENA_OUT]       = { GPIOG, GPIO_PIN_3  },
    [BOARD_SIG_XR1_CONTACT_OUT]   = { GPIOG, GPIO_PIN_4  },
    [BOARD_SIG_XR2_CONTACT_OUT]   = { GPIOG, GPIO_PIN_5  },
    [BOARD_SIG_XR3_CONTACT_OUT]   = { GPIOG, GPIO_PIN_6  },
    [BOARD_SIG_XR4_CONTACT_OUT]   = { GPIOG, GPIO_PIN_7  },
    [BOARD_SIG_GPOUT_09]          = { GPIOG, GPIO_PIN_8  },
    [BOARD_SIG_GPOUT_10]          = { GPIOG, GPIO_PIN_9  },
    [BOARD_SIG_GPOUT_11]          = { GPIOD, GPIO_PIN_0  },
    [BOARD_SIG_GPOUT_12]          = { GPIOD, GPIO_PIN_1  },
    [BOARD_SIG_GPOUT_ENABLE]      = { GPIOC, GPIO_PIN_13 },
    [BOARD_SIG_PWMALT_ENABLE]     = { GPIOC, GPIO_PIN_15 },
};

/* GateDriverStatus_01..12 = PE0..PE11 */
#define GDS_PIN_MASK   (GPIO_PIN_0  | GPIO_PIN_1  | GPIO_PIN_2  | GPIO_PIN_3  | \
                        GPIO_PIN_4  | GPIO_PIN_5  | GPIO_PIN_6  | GPIO_PIN_7  | \
                        GPIO_PIN_8  | GPIO_PIN_9  | GPIO_PIN_10 | GPIO_PIN_11)

uint8_t BoardIo_Read(board_signal_t sig)
{
    if ((unsigned)sig >= (unsigned)BOARD_SIG_COUNT)
    {
        return 0U;
    }
    return (HAL_GPIO_ReadPin(kPins[sig].port, kPins[sig].pin) == GPIO_PIN_SET) ? 1U : 0U;
}

void BoardIo_Write(board_signal_t sig, uint8_t high)
{
    if (((unsigned)sig >= (unsigned)BOARD_SIG_COUNT) || (sig < BOARD_SIG_XR1_ENA_OUT))
    {
        return;
    }
    HAL_GPIO_WritePin(kPins[sig].port, kPins[sig].pin, (high != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

uint16_t BoardIo_ReadGateDriverStatus(void)
{
    return (uint16_t)(GPIOE->IDR & GDS_PIN_MASK);
}

void BoardIo_Init(void)
{
    /* GateDriverStatus_01..12 (PE0..PE11, docs/pin_mapping_v4.csv) --
       interrupt-capable digital inputs, no pull. Originally added
       2026-09-08 as plain GPIO_MODE_INPUT alongside the GDS? diagnostic
       command (cmd_io.c); upgraded the same day to
       GPIO_MODE_IT_RISING_FALLING to back a real fault interrupt
       (gate_driver.c's GateDriver_CheckFault()) -- GDS? still works
       identically either way, a plain IDR read. These pins were never
       configured at all before the first of those two changes, so a
       floating/undriven pin would have read an arbitrary level. Pull
       matches the sibling PFM-STM32G474 project's gpio.c config for
       these same 12 pins (GPIO_NOPULL -- gate-driver-IC status outputs,
       actively driven, no internal pull needed).

       Both edges (not just the one GDS_FAULT_POLARITY, ctrlr_config.h,
       currently cares about) so a fault is caught regardless of which
       direction a pin moves -- the actual fault/healthy determination
       happens in GateDriver_CheckFault(), against that compile-time
       setting, not by picking rising-only or falling-only here; that
       keeps this config correct even if GDS_FAULT_POLARITY is ever
       flipped without also revisiting this block.

       __HAL_RCC_SYSCFG_CLK_ENABLE() is required before HAL_GPIO_Init()
       can actually route these pins' EXTI lines (SYSCFG->EXTICR) -- easy
       to omit and get a config that silently never fires. */
    {
        GPIO_InitTypeDef gdsInit = {0};

        __HAL_RCC_GPIOE_CLK_ENABLE();
        __HAL_RCC_SYSCFG_CLK_ENABLE();

        gdsInit.Pin   = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2  | GPIO_PIN_3  |
                         GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6  | GPIO_PIN_7  |
                         GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11;
        gdsInit.Mode  = GPIO_MODE_IT_RISING_FALLING;
        gdsInit.Pull  = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOE, &gdsInit);

        /* EXTI0..EXTI4 are individual NVIC vectors; EXTI5..9 share
           EXTI9_5_IRQn; EXTI10..15 share EXTI15_10_IRQn -- PE0..PE11
           spans all three groups, 7 vectors total (see stm32g4xx_it.c).
           Priority tied with HRTIM1_Master_IRQn (1,0) -- both are
           output-safety-critical paths, and since neither ISR runs long
           (a register read/compare, occasionally a HRTIM1_PWM_Stop()
           call), a bounded, occasional deferral between the two at equal
           priority is an acceptable tradeoff, not a real latency risk.
           Below USART2 (2,0) -- fault detection preempts serial I/O, not
           the other way around. Safe to configure NVIC priority/enable
           here: this runs after Mcu_SetSysTickHighestPriority() (main(), USER
           CODE Init) and after HAL_Init()'s own NVIC setup, the same ordering
           constraint HRTIM1_EnableMasterInterrupt() documents for
           itself. */
        HAL_NVIC_SetPriority(EXTI0_IRQn, 1U, 0U);
        HAL_NVIC_EnableIRQ(EXTI0_IRQn);
        HAL_NVIC_SetPriority(EXTI1_IRQn, 1U, 0U);
        HAL_NVIC_EnableIRQ(EXTI1_IRQn);
        HAL_NVIC_SetPriority(EXTI2_IRQn, 1U, 0U);
        HAL_NVIC_EnableIRQ(EXTI2_IRQn);
        HAL_NVIC_SetPriority(EXTI3_IRQn, 1U, 0U);
        HAL_NVIC_EnableIRQ(EXTI3_IRQn);
        HAL_NVIC_SetPriority(EXTI4_IRQn, 1U, 0U);
        HAL_NVIC_EnableIRQ(EXTI4_IRQn);
        HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1U, 0U);
        HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
        HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1U, 0U);
        HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
    }

    /* PF13 (docs/pin_mapping_v4.csv -- documented "GPInput_12", confirmed
       GPI, unused elsewhere) -- the external-enable interlock, MOVED here
       2026-09-17 from PF15 (Fiber_Enable) per direct instruction: enable
       and trigger are now two independent physical signals, not one
       shared wire. state_machine.c's own external-enable section has the
       full design (ARM/SHOT:STARt gating, FIRING-only continuous
       monitoring, SM_FAULT_EXTERNAL_ENABLE on loss). Plain polled input,
       no EXTI -- same reasoning as before the move: state_machine.c's
       SM_PollFaults() already checks this at the same cadence (main loop
       + every real PID_Update() tick, ~1kHz while FIRING) General Fault's
       own two hardware sources get.

       GPIO_PULLDOWN, NOT this project's usual GPIO_NOPULL for actively-
       driven inputs (GateDriverStatus above, PFM_Input, QUADSPI) --
       deliberate, carried over unchanged from PF15's own original
       reasoning: an unconnected/floating PF13 must read LOW (no
       permission granted), never an undefined level that could
       accidentally read HIGH and silently permit firing. */
    {
        GPIO_InitTypeDef extEnableInit = {0};

        __HAL_RCC_GPIOF_CLK_ENABLE();

        extEnableInit.Pin  = GPIO_PIN_13;
        extEnableInit.Mode = GPIO_MODE_INPUT;
        extEnableInit.Pull = GPIO_PULLDOWN;
        HAL_GPIO_Init(GPIOF, &extEnableInit);
    }

    /* PF15 (Fiber_Enable, docs/pin_mapping_v4.csv -- confirmed GPI there)
       -- external TRIGGER only, as of 2026-09-17 (previously this pin
       also carried the external-enable role removed above; a rising
       edge here while ARMED fires a shot -- state_machine.c's own
       external-trigger section has the full design). Plain polled input,
       no EXTI -- same reasoning as PF13 above, and as this pin's own
       prior enable role: state_machine.c's SM_PollFaults() polls this at
       the same ~1kHz-while-FIRING cadence.

       GPIO_PULLDOWN carried over unchanged -- an unconnected/floating
       PF15 reading LOW means no spurious rising edge is ever seen from a
       disconnected trigger wire (edge detection needs an actual LOW-to-
       HIGH transition; a pin parked at a stable floating LOW produces
       none). Unlike PF13 above, there's no "wrong direction" concern
       here either way -- a floating trigger pin that never fires is the
       safe failure mode regardless of which level it floats to, but
       PULLDOWN keeps it deterministic and consistent with every other
       pin in this file. */
    {
        GPIO_InitTypeDef extTriggerInit = {0};

        __HAL_RCC_GPIOF_CLK_ENABLE();

        extTriggerInit.Pin  = GPIO_PIN_15;
        extTriggerInit.Mode = GPIO_MODE_INPUT;
        extTriggerInit.Pull = GPIO_PULLDOWN;
        HAL_GPIO_Init(GPIOF, &extTriggerInit);
    }

    /* *** PG10 GPIO CONFIG REMOVED 2026-09-17 ***
       Originally added the same day for the emergency-stop feature
       (state_machine.c's own emergency-stop section), based on the
       user's own direct confirmation at the time that pin_mapping_v4.csv's
       "NRST" label for this net was stale/incorrect and PG10 was a plain
       fiber-optic GPIO input, unrelated to the MCU's real reset function.

       THAT CONFIRMATION WAS WRONG, corrected the same day via the actual
       schematic: PG10 IS electrically tied to this MCU's real, dedicated
       NRST pin -- both land on the same net, which also runs to the
       ST-Link/Molex debug connector. Discovered by direct real-hardware
       evidence, not inspection: driving a new fiber transmitter (PD0,
       DIAGnostic:GPOut11) into an inverting receiver wired to "PG10"
       caused a genuine MCU reset every time it went HIGH -- confirmed via
       RCC->CSR (a new temporary DIAGnostic:RSTCause? command,
       cmd_system.c): PINRSTF set, BORRSTF clear, ruling out a power-rail-
       droop theory and directly proving a real NRST-pin assertion, not
       mere GPIO-level signal corruption.

       This means configuring this pin as a GPIO peripheral input AT ALL
       (regardless of EMERGency:ENAble's state) was unsound the entire
       time -- GPIO_PULLDOWN was a weak pull-down actively fighting NRST's
       own internal pull-up on the literal reset/debug net on every boot,
       independent of whether the emergency-stop feature was ever enabled.
       Removed entirely, per direct instruction ("shelve E-stop entirely
       again... pull the GPIO_PULLDOWN config off PG10 specifically") --
       this pin is not a usable GPIO on this board and must not be
       reconfigured as one again. The emergency-stop SOFTWARE
       (SM_FAULT_EMERGENCY_STOP and friends) was then REMOVED 2026-09-21
       (see docs/changelog.txt) rather than left dormant -- a genuinely
       free pin from the schematic is needed before any future E-stop
       feature can be re-pointed at real hardware. See
       [[pending-hardware-calibration]] (session memory) for the full
       writeup. */

    /* XR1_OCP/XR2_OCP/XR3_OCP/XR4_OCP (PF4/PF8/PF12/PF5,
       docs/pin_mapping_v4.csv's new "XREX Pin Name" column), added
       2026-09-17 -- real per-channel overcurrent-protect fault inputs,
       the first real hardware trigger for SM_ReportOcpFault()
       (state_machine.h) to ever exist in this codebase (previously only
       reachable via the software-injection OCP:TEST:FAULT command). See
       xrex_io.h's own extensive header comment for the full design --
       xrex_io.c's XrexIo_PollOcpFaults() reads these.

       POLLED, not EXTI-driven -- a real hardware conflict, not a
       preference: 3 of these 4 pins' EXTI line numbers (EXTI4, EXTI5,
       EXTI8) are already claimed by the EXISTING GateDriverStatus EXTI
       setup on PE4/PE5/PE8 just above (STM32's 16 EXTI lines are shared
       project-wide, one GPIO port per line number via SYSCFG_EXTICR --
       PF4 and PE4 genuinely cannot both be interrupt sources
       simultaneously). Rather than split these 4 pins across two
       different detection mechanisms, all 4 are polled uniformly,
       alongside state_machine.c's own SM_PollFaults() calls (main loop +
       PID_Update()) -- see xrex_io.h for the full reasoning.

       GPIO_PULLDOWN, NOT this project's usual GPIO_NOPULL for the
       EXISTING GateDriverStatus pins (which ARE the same class of
       gate-driver-IC-sourced signal) -- deliberate, matching the fail-
       safe reasoning already applied to PF13/PF15/PG10 above: with
       XR_OCP_FLT_POLARITY (ctrlr_config.h) defaulting NORMALLY_HIGH, an
       unconnected/floating OCP pin must read LOW -- fault asserted, the
       safe default -- rather than an undefined level that could
       accidentally read HIGH and falsely look healthy. */
    {
        GPIO_InitTypeDef ocpInit = {0};

        __HAL_RCC_GPIOF_CLK_ENABLE();

        ocpInit.Pin  = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_8 | GPIO_PIN_12;
        ocpInit.Mode = GPIO_MODE_INPUT;
        ocpInit.Pull = GPIO_PULLDOWN;
        HAL_GPIO_Init(GPIOF, &ocpInit);
    }

    /* XR1-4_ENA_OUT (PG0-PG3) + XR1-4_CONTACT_OUT (PG4-PG7,
       docs/pin_mapping_v4.csv's new "XREX Pin Name" column), added
       2026-09-17, per direct request: real per-channel fiber outputs
       this firmware itself drives, set via
       XREX:CHANnel:ENAOut/CONTactOut (cmd_io.c), owned (pin table,
       GPIO read/write) by xrex_io.c -- see xrex_io.h and
       state_machine.h's own SM_FAULT_ENABLE_OUTPUT/enable-output
       sections for the full design (ARM refuses unless every currently-
       enabled channel's own pair is HIGH, and this is continuously
       re-checked once ARMED).

       Push-pull OUTPUTS (not inputs, unlike every other XR-pin block in
       this function) -- driven LOW BEFORE HAL_GPIO_Init() enables them,
       matching this project's established "never glitch HIGH on boot"
       convention for every other software-driven output
       (DIAGnostic:GPOut11/GPOut12 above) -- a fresh boot must never
       present an accidental "outputting" state to whatever real hardware
       these are wired to. GPIO_NOPULL (irrelevant for a push-pull
       output, same as every other output block in this file). */
    {
        GPIO_InitTypeDef enaContactInit = {0};

        __HAL_RCC_GPIOG_CLK_ENABLE();

        HAL_GPIO_WritePin(GPIOG,
                           GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 |
                           GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7,
                           GPIO_PIN_RESET);   /* set level BEFORE enabling the
                                                  output, so it never glitches
                                                  HIGH first */

        enaContactInit.Pin   = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 |
                                GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
        enaContactInit.Mode  = GPIO_MODE_OUTPUT_PP;
        enaContactInit.Pull  = GPIO_NOPULL;
        enaContactInit.Speed = GPIO_SPEED_FREQ_LOW;   /* level outputs, not
                                                           fast signals -- no
                                                           reason for a
                                                           faster slew */
        HAL_GPIO_Init(GPIOG, &enaContactInit);
    }

    /* PC13 ("GPOut_Enable_Pin" in the V4 column, docs/pin_mapping_v4.csv --
       confirmed GPO there, unused elsewhere), added 2026-09-17 per direct
       request. Push-pull output, driven to its configured default level
       BEFORE HAL_GPIO_Init() enables it -- default level is now a named
       compile-time config, GPOUT_ENABLE_DEFAULT_HIGH (ctrlr_config.h,
       added 2026-09-18 per direct follow-up instruction, applies to
       EVERY build of this board -- controller and simulator alike),
       currently `1` (HIGH) matching the ORIGINAL 2026-09-17 instruction
       ("By default, keep it HIGH") -- the OPPOSITE default of every
       other software-driven output in this file (DIAGnostic:GPOut11/12,
       ENA_OUT/CONTACT_OUT above, all deliberately LOW by default) -- a
       fresh boot must present this pin's real intended default, not an
       incidental LOW that happens to match everything else here.
       GPIO_NOPULL (irrelevant for a push-pull output, same as every
       other output block in this file). Driven at RUNTIME by
       GPOut:ENAble (cmd_io.c) -- a separate, coexisting mechanism from
       this boot-time default, see GPOUT_ENABLE_DEFAULT_HIGH's own
       comment for why both exist. */
    {
        GPIO_InitTypeDef gpOutEnableInit = {0};

        __HAL_RCC_GPIOC_CLK_ENABLE();

        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13,
                           (GPOUT_ENABLE_DEFAULT_HIGH != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
                           /* set level BEFORE enabling the output, so
                              it's never briefly the opposite level first */
        gpOutEnableInit.Pin   = GPIO_PIN_13;
        gpOutEnableInit.Mode  = GPIO_MODE_OUTPUT_PP;
        gpOutEnableInit.Pull  = GPIO_NOPULL;
        gpOutEnableInit.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(GPIOC, &gpOutEnableInit);
    }

    /* PC15 ("PWM_Alt_Enable" in the V4 column, docs/pin_mapping_v4.csv --
       confirmed GPO there, unused elsewhere), added 2026-09-17 per direct
       request -- same reasoning as PC13 just above (default level from
       PWMALT_ENABLE_DEFAULT_HIGH, ctrlr_config.h, currently HIGH; driven
       before enable; GPIO_NOPULL). Driven at runtime by PWMAlt:ENAble
       (cmd_io.c). */
    {
        GPIO_InitTypeDef pwmAltEnableInit = {0};

        __HAL_RCC_GPIOC_CLK_ENABLE();

        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15,
                           (PWMALT_ENABLE_DEFAULT_HIGH != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
                           /* set level BEFORE enabling the output */
        pwmAltEnableInit.Pin   = GPIO_PIN_15;
        pwmAltEnableInit.Mode  = GPIO_MODE_OUTPUT_PP;
        pwmAltEnableInit.Pull  = GPIO_NOPULL;
        pwmAltEnableInit.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(GPIOC, &pwmAltEnableInit);
    }

    /* PD1 ("GPOut_12" in the V4 column, docs/pin_mapping_v4.csv --
       confirmed GPO there), added 2026-09-16 as a generic, software-
       driven diagnostic output. Direct request; PF13 was proposed first
       and corrected -- PF13 is actually documented "GPInput_12" (an
       INPUT) in the same CSV, a different pin from the one actually
       named "GPOut_12" (PD1), same class of name/pin mismatch as the
       earlier PC14-vs-PF15 correction. Immediate use, 2026-09-16: driven
       by DIAGnostic:GPOut12 (cmd_io.c) and physically looped to PF15
       (Fiber_Enable) by the operator, letting the external-enable/
       external-trigger feature above be exercised entirely from the
       serial console -- precise, repeatable control over PF15's level
       at exactly the right moments -- rather than needing a hand-
       operated bench jumper/switch. (2026-09-17 UPDATE: PF15 now backs
       external-TRIGGER only -- enable moved to its own pin, PF13, not
       looped to this diagnostic output -- so this loop now exercises
       trigger specifically.) Not tied to that use case in the pin config
       itself, just today's reason for wanting it: a generic level output,
       nothing PF15-specific baked in here.

       Initial state LOW (Pull left at default/NOPULL -- irrelevant for
       a push-pull output, the pin is actively driven the instant this
       runs) -- starts deasserted so a fresh boot never presents an
       accidental HIGH to whatever it's connected to. */
    {
        GPIO_InitTypeDef diagOutInit = {0};

        __HAL_RCC_GPIOD_CLK_ENABLE();

        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET);   /* set level
                                                                     BEFORE
                                                                     enabling
                                                                     the output,
                                                                     so it never
                                                                     glitches
                                                                     HIGH first */
        diagOutInit.Pin   = GPIO_PIN_1;
        diagOutInit.Mode  = GPIO_MODE_OUTPUT_PP;
        diagOutInit.Pull  = GPIO_NOPULL;
        diagOutInit.Speed = GPIO_SPEED_FREQ_LOW;   /* a diagnostic level
                                                        output, not a fast
                                                        signal -- no reason
                                                        for a faster slew */
        HAL_GPIO_Init(GPIOD, &diagOutInit);
    }

    /* PD0 ("GPOut_11" in the V4 column, docs/pin_mapping_v4.csv --
       confirmed GPO there, same row-pattern as PD1/"GPOut_12" just
       above), added 2026-09-17 -- a SECOND, independent diagnostic
       output. Direct correction: PD1 above was initially reused for
       testing the PG10 emergency-stop feature too (a second fiber looped
       from the same PD1 pin), but PD1 is already the pin dedicated to
       driving PF15 (external-trigger as of later the same day --
       external-enable at the time this was written) -- the user caught
       this mix-up and asked for a genuinely separate pin for PG10 testing
       instead, to
       remove any ambiguity about which diagnostic signal is driving
       which real input. Otherwise identical in every respect to PD1's
       own diagnostic-output config just above -- generic, software-
       driven level output, nothing PG10-specific baked in here, driven
       by DIAGnostic:GPOut11 (cmd_io.c). Same initial-state-LOW-before-
       enable reasoning as PD1 -- never glitches HIGH on boot. */
    {
        GPIO_InitTypeDef diagOutInit2 = {0};

        __HAL_RCC_GPIOD_CLK_ENABLE();

        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);   /* set level
                                                                     BEFORE
                                                                     enabling
                                                                     the output,
                                                                     so it never
                                                                     glitches
                                                                     HIGH first */
        diagOutInit2.Pin   = GPIO_PIN_0;
        diagOutInit2.Mode  = GPIO_MODE_OUTPUT_PP;
        diagOutInit2.Pull  = GPIO_NOPULL;
        diagOutInit2.Speed = GPIO_SPEED_FREQ_LOW;   /* a diagnostic level
                                                        output, not a fast
                                                        signal -- no reason
                                                        for a faster slew */
        HAL_GPIO_Init(GPIOD, &diagOutInit2);
    }

    /* PG8/PG9 ("GPOut_09"/"GPOut_10" in the V4 column, docs/pin_mapping_v4.csv
       -- confirmed GPO there; note the V3 column for these two rows is "No
       connection" and a DIFFERENT pair of pins, PD8/PD9, carried the
       "GPOut_09"/"GPOut_10" names in V3 -- verified against the CSV directly
       before writing this, so as not to repeat the earlier PC14-vs-PF15/
       PF13-vs-PD1 V3/V4 name-reuse mistakes), added 2026-09-18 -- a THIRD and
       FOURTH generic, software-driven diagnostic output, same class as
       PD0/PD1 ("GPOut_11"/"GPOut_12") just above. Immediate use: the Transrex
       simulator's fiber-transmitter budget assigns these two to XR1_OCP/
       XR2_OCP (see docs/pin_mapping_reference.tex Section 7) -- until now
       they had no GPIO config or command on either board, so those two OCP
       channels were untestable over fiber. Otherwise identical in every
       respect to PD0/PD1's own diagnostic-output config above -- generic
       level outputs, nothing OCP-specific baked in here, driven by
       DIAGnostic:GPOut09/GPOut10 (cmd_io.c). Same initial-state-LOW-
       before-enable reasoning -- never glitches HIGH on boot. */
    {
        GPIO_InitTypeDef diagOutInit3 = {0};

        __HAL_RCC_GPIOG_CLK_ENABLE();

        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_RESET);
                           /* set level BEFORE enabling the output, so it
                              never glitches HIGH first */
        diagOutInit3.Pin   = GPIO_PIN_8 | GPIO_PIN_9;
        diagOutInit3.Mode  = GPIO_MODE_OUTPUT_PP;
        diagOutInit3.Pull  = GPIO_NOPULL;
        diagOutInit3.Speed = GPIO_SPEED_FREQ_LOW;   /* diagnostic level
                                                         outputs, not fast
                                                         signals -- no reason
                                                         for a faster slew */
        HAL_GPIO_Init(GPIOG, &diagOutInit3);
    }

}
