/*
 * xrex_io.c
 *
 * See xrex_io.h for full scope/design.
 */

#include "xrex_io.h"
#include "ctrlr_config.h"
#include "gate_driver.h"
#include "state_machine.h"
#include "pid.h"
#include "main.h"

/* Base bit position (within GateDriver_Read()'s 12-bit PE0..PE11
   result) for each fault category's XR1 entry -- channel index ch
   (0-based) uses bit (base + ch). Verified programmatically (not by
   eye) against docs/pin_mapping_v4.csv's "XREX Pin Name" column,
   2026-09-17:
     XR1_WATER_FLT=PE0(bit0)   XR2=PE1(bit1)  XR3=PE2(bit2)  XR4=PE3(bit3)
     XR1_TMP_FLT=PE4(bit4)     XR2=PE5(bit5)  XR3=PE6(bit6)  XR4=PE7(bit7)
     XR1_ENERPRO_FLT=PE8(bit8) XR2=PE9(bit9)  XR3=PE10(bit10) XR4=PE11(bit11)
   A genuinely regular pattern (category_base + channel_index), not a
   coincidence to be suspicious of -- these are 12 consecutive pins on
   one port, assigned in exactly this per-category block order. */
#define XR_WATER_FLT_BIT_BASE     (0U)
#define XR_TMP_FLT_BIT_BASE       (4U)
#define XR_ENERPRO_FLT_BIT_BASE   (8U)

/* XRn_OCP pins (PF4/PF5/PF8/PF12) -- NOT a regular pattern like the
   water/temp/enerpro bases above, so an explicit per-channel table,
   not arithmetic, matching this project's own established precedent
   (pfm_input.c's kDesc[]) for genuinely irregular pin assignments.
   Verified against docs/pin_mapping_v4.csv's "XREX Pin Name" column,
   2026-09-17: XR1_OCP=PF4, XR2_OCP=PF8, XR3_OCP=PF12, XR4_OCP=PF5. */
static const uint16_t kOcpPin[HRTIM_NUM_CHANNELS] =
{
    GPIO_PIN_4,    /* XR1_OCP */
    GPIO_PIN_8,    /* XR2_OCP */
    GPIO_PIN_12,   /* XR3_OCP */
    GPIO_PIN_5,    /* XR4_OCP */
};

/* XR_WATER_FLT_POLARITY/XR_TMP_FLT_POLARITY/XR_ENERPRO_FLT_POLARITY/
   XR_OCP_FLT_POLARITY (ctrlr_config.h) -- RUNTIME-CONFIGURABLE as of
   2026-09-22, direct request. Back CONFig:FaultPolarity:WATER/:TEMP/
   :ENERPRO/:OCP (commands.c) via XrexIo_SetFaultPolarity*() (below).
   Declared OUTSIDE the BUILD_TARGET_SIMULATOR guard below (unlike
   BitIsFault() itself) so the setters/getters compile and work on
   BOTH targets, same as DEBUG:FAULT:BYPASS's own "not build-target-
   guarded" precedent -- harmless on the simulator build, which never
   actually reads these (its own SIM:FAULT:* fault injection is a
   completely separate mechanism, sim_transrex.c). */
static uint32_t g_xrWaterFltPolarity   = XR_WATER_FLT_POLARITY;
static uint32_t g_xrTmpFltPolarity     = XR_TMP_FLT_POLARITY;
static uint32_t g_xrEnerproFltPolarity = XR_ENERPRO_FLT_POLARITY;
static uint32_t g_xrOcpFltPolarity     = XR_OCP_FLT_POLARITY;

/* Shared setter body -- `polarity` must be exactly FAULT_POLARITY_
   NORMALLY_HIGH (0) or FAULT_POLARITY_NORMALLY_LOW (1), matching
   these two named constants' own encoding (ctrlr_config.h); anything
   else is rejected rather than silently stored as some third,
   undefined polarity. */
static uint8_t SetFaultPolarity(uint32_t *dest, uint32_t polarity)
{
    if ((polarity != FAULT_POLARITY_NORMALLY_HIGH) && (polarity != FAULT_POLARITY_NORMALLY_LOW))
    {
        return 0U;
    }
    *dest = polarity;
    return 1U;
}

uint8_t XrexIo_SetFaultPolarityWater(uint32_t polarity)
{
    return SetFaultPolarity(&g_xrWaterFltPolarity, polarity);
}
uint32_t XrexIo_GetFaultPolarityWater(void)
{
    return g_xrWaterFltPolarity;
}

uint8_t XrexIo_SetFaultPolarityTemp(uint32_t polarity)
{
    return SetFaultPolarity(&g_xrTmpFltPolarity, polarity);
}
uint32_t XrexIo_GetFaultPolarityTemp(void)
{
    return g_xrTmpFltPolarity;
}

uint8_t XrexIo_SetFaultPolarityEnerpro(uint32_t polarity)
{
    return SetFaultPolarity(&g_xrEnerproFltPolarity, polarity);
}
uint32_t XrexIo_GetFaultPolarityEnerpro(void)
{
    return g_xrEnerproFltPolarity;
}

uint8_t XrexIo_SetFaultPolarityOcp(uint32_t polarity)
{
    return SetFaultPolarity(&g_xrOcpFltPolarity, polarity);
}
uint32_t XrexIo_GetFaultPolarityOcp(void)
{
    return g_xrOcpFltPolarity;
}

#if !defined(BUILD_TARGET_SIMULATOR)
/* Shared polarity-interpretation helper -- `bit` is a raw HIGH(1)/LOW(0)
   level, `polarity` is one of ctrlr_config.h's FAULT_POLARITY_NORMALLY_*
   constants. Returns 1 if this level represents a fault under that
   polarity. Only used by the controller-target bodies of the four fault
   pollers below (2026-09-21 -- see their own comments on why they're
   no-ops on BUILD_TARGET_SIMULATOR) -- guarded out here too, otherwise
   unused on that build target. */
static uint8_t BitIsFault(uint8_t bit, uint32_t polarity)
{
    return (polarity == FAULT_POLARITY_NORMALLY_HIGH) ? (bit == 0U) : (bit != 0U);
}
#endif /* !BUILD_TARGET_SIMULATOR */

uint8_t XrexIo_EvaluateGateDriverFault(uint16_t raw12)
{
#if defined(BUILD_TARGET_SIMULATOR)
    /* REAL BUG, found and fixed 2026-09-21: on the simulator, PE0..PE11
       (GateDriver_Read(), what `raw12` is) are REPURPOSED as the
       ENA_OUT/CONTACT_OUT gating INPUT (see sim_transrex.c's own pin-
       role table) -- legitimate signals from the controller's normal
       gating activity, not real Water/Temp fault levels. This
       unmodified controller code was reading those same bits as if
       they meant Water/Temp fault status the WHOLE session, and
       reporting a spurious fault (hard-disabling that channel's own
       HRTIM output, killing its FEEDBACK transmission) essentially
       every time the controller gated a channel on/off during normal
       testing -- root cause of a day-long "controller receives
       nothing, seemingly randomly" investigation (see docs/
       changelog.txt's own extensive entry). No real meaning on this
       build target -- always report healthy. */
    (void)raw12;
    return 0U;
#else
    for (uint8_t ch = 0U; ch < HRTIM_NUM_CHANNELS; ch++)
    {
        if (PID_GetChannelEnable(ch) == 0U)
        {
            continue;   /* gated: a disabled channel's own Water/Temp
                           pins never count toward a fault */
        }

        uint8_t waterBit = (uint8_t)((raw12 >> (XR_WATER_FLT_BIT_BASE + ch)) & 1U);
        uint8_t tmpBit   = (uint8_t)((raw12 >> (XR_TMP_FLT_BIT_BASE + ch)) & 1U);

        /* Enerpro deliberately excluded here as of 2026-09-18 -- see
           this file's own header comment and XrexIo_PollEnerproFaults()
           below, which now owns Enerpro's own (differently-routed)
           fault check. */
        if ((BitIsFault(waterBit, g_xrWaterFltPolarity) != 0U) ||
            (BitIsFault(tmpBit, g_xrTmpFltPolarity) != 0U))
        {
            return 1U;
        }
    }
    return 0U;
#endif /* BUILD_TARGET_SIMULATOR */
}

void XrexIo_PollEnerproFaults(uint16_t raw12)
{
#if defined(BUILD_TARGET_SIMULATOR)
    /* Same real bug as XrexIo_EvaluateGateDriverFault() above -- these
       bits are the simulator's own ENA_OUT/CONTACT_OUT gating input on
       this build target, not real Enerpro fault levels. No-op here. */
    (void)raw12;
    return;
#else
    for (uint8_t ch = 0U; ch < HRTIM_NUM_CHANNELS; ch++)
    {
        if (PID_GetChannelEnable(ch) == 0U)
        {
            continue;   /* gated -- see this file's own header comment */
        }

        uint8_t enerproBit = (uint8_t)((raw12 >> (XR_ENERPRO_FLT_BIT_BASE + ch)) & 1U);
        if (BitIsFault(enerproBit, g_xrEnerproFltPolarity) != 0U)
        {
            SM_ReportEnerproFault(ch);
        }
    }
#endif /* BUILD_TARGET_SIMULATOR */
}

void XrexIo_PollOcpFaults(void)
{
#if defined(BUILD_TARGET_SIMULATOR)
    /* REAL BUG, found and fixed 2026-09-21: kOcpPin[] (PF4/PF8/PF12/PF5)
       has no real meaning on this build target -- genuinely floating/
       unconnected, no documented repurposing role (unlike PE0..PE11).
       Reading them as real OCP fault levels tripped a spurious,
       essentially random per-channel fault (hard-disabling that
       channel's own HRTIM output -- killing FEEDBACK transmission)
       purely from electrical noise on an unconnected pin, active from
       the moment the simulator booted (PID_GetChannelEnable() defaults
       to enabled for every channel at boot -- see PID_Init()). Root
       cause of a day-long "controller receives nothing, seemingly
       randomly" investigation -- see docs/changelog.txt's own
       extensive entry. No-op here. */
    return;
#else
    for (uint8_t ch = 0U; ch < HRTIM_NUM_CHANNELS; ch++)
    {
        if (PID_GetChannelEnable(ch) == 0U)
        {
            continue;   /* gated -- see this file's own header comment */
        }

        uint8_t bit = (HAL_GPIO_ReadPin(GPIOF, kOcpPin[ch]) == GPIO_PIN_SET) ? 1U : 0U;
        if (BitIsFault(bit, g_xrOcpFltPolarity) != 0U)
        {
            SM_ReportOcpFault(ch);
        }
    }
#endif /* BUILD_TARGET_SIMULATOR */
}

uint8_t XrexIo_GetChannelStatus(uint8_t channel, uint8_t *water, uint8_t *tmp,
                                 uint8_t *enerpro, uint8_t *ocp)
{
    if (channel >= HRTIM_NUM_CHANNELS)
    {
        return 0U;
    }

    uint16_t raw = GateDriver_Read();

    if (water != NULL)   { *water   = (uint8_t)((raw >> (XR_WATER_FLT_BIT_BASE + channel)) & 1U); }
    if (tmp != NULL)     { *tmp     = (uint8_t)((raw >> (XR_TMP_FLT_BIT_BASE + channel)) & 1U); }
    if (enerpro != NULL) { *enerpro = (uint8_t)((raw >> (XR_ENERPRO_FLT_BIT_BASE + channel)) & 1U); }
    if (ocp != NULL)     { *ocp = (HAL_GPIO_ReadPin(GPIOF, kOcpPin[channel]) == GPIO_PIN_SET) ? 1U : 0U; }

    return 1U;
}

/* XRn_ENA_OUT/XRn_CONTACT_OUT (PG0-PG3/PG4-PG7) -- a REGULAR pattern
   (base + channel_index), same as the Water/Temp/Enerpro bit bases
   above, unlike XRn_OCP's irregular table -- verified programmatically
   against docs/pin_mapping_v4.csv's "XREX Pin Name" column, 2026-09-17:
     XR1_ENA_OUT=PG0(bit0)     XR2=PG1(bit1)  XR3=PG2(bit2)  XR4=PG3(bit3)
     XR1_CONTACT_OUT=PG4(bit4) XR2=PG5(bit5)  XR3=PG6(bit6)  XR4=PG7(bit7)
   `1U << (base + ch)` produces exactly the same numeric value as the
   HAL's own GPIO_PIN_n constants for n = 0..7 (they're plain bit-shift
   literals), so this doubles as the pin's own GPIO_PIN_x mask -- no
   separate lookup table needed, matching the Water/Temp/Enerpro
   approach over the OCP one. */
#define XR_ENA_OUT_PIN_BASE       (0U)
#define XR_CONTACT_OUT_PIN_BASE   (4U)

void XrexIo_SetEnableOutput(uint8_t channel, uint8_t on)
{
    if (channel >= HRTIM_NUM_CHANNELS)
    {
        return;
    }
    HAL_GPIO_WritePin(GPIOG, (uint16_t)(1U << (XR_ENA_OUT_PIN_BASE + channel)),
                       (on != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

uint8_t XrexIo_GetEnableOutput(uint8_t channel)
{
    if (channel >= HRTIM_NUM_CHANNELS)
    {
        return 0U;
    }
    return (HAL_GPIO_ReadPin(GPIOG, (uint16_t)(1U << (XR_ENA_OUT_PIN_BASE + channel))) == GPIO_PIN_SET) ? 1U : 0U;
}

void XrexIo_SetContactorOutput(uint8_t channel, uint8_t on)
{
    if (channel >= HRTIM_NUM_CHANNELS)
    {
        return;
    }
    HAL_GPIO_WritePin(GPIOG, (uint16_t)(1U << (XR_CONTACT_OUT_PIN_BASE + channel)),
                       (on != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

uint8_t XrexIo_GetContactorOutput(uint8_t channel)
{
    if (channel >= HRTIM_NUM_CHANNELS)
    {
        return 0U;
    }
    return (HAL_GPIO_ReadPin(GPIOG, (uint16_t)(1U << (XR_CONTACT_OUT_PIN_BASE + channel))) == GPIO_PIN_SET) ? 1U : 0U;
}

/* Shared by XrexIo_EnableOutputsReadyToArm()/XrexIo_PollEnableOutputFaults()
   below -- 1 if `channel`'s own ENA_OUT AND CONTACT_OUT are BOTH
   currently HIGH, 0 otherwise. Treated as one combined per-channel
   condition throughout this feature, not two independently-faultable
   ones -- see state_machine.h's own enable-output section for why. */
static uint8_t EnableOutputsOk(uint8_t channel)
{
    return (uint8_t)((XrexIo_GetEnableOutput(channel) != 0U) &&
                      (XrexIo_GetContactorOutput(channel) != 0U));
}

uint8_t XrexIo_EnableOutputsReadyToArm(void)
{
    return (XrexIo_FindNotReadyChannel() == 0xFFU) ? 1U : 0U;
}

/* Added 2026-09-22, direct request: ARM used to report only a generic
   "conditions not met" on failure -- an operator with one misconfigured
   channel out of four had no way to tell which one from the wire
   protocol alone. Same scan as XrexIo_EnableOutputsReadyToArm() above
   (now implemented in terms of this function instead of duplicating
   the loop), but returns the first offending 0-based channel instead
   of collapsing to a bool -- 0xFF is the "all channels ready, nothing
   to report" sentinel, matching SM_GetFaultChannel()'s own established
   not-applicable convention. Backs cmd_arm()'s (commands.c) specific
   "Channel N's ENA_OUT/CONTACT_OUT..." error text. */
uint8_t XrexIo_FindNotReadyChannel(void)
{
    for (uint8_t ch = 0U; ch < HRTIM_NUM_CHANNELS; ch++)
    {
        if (PID_GetChannelEnable(ch) == 0U)
        {
            continue;   /* gated -- see this file's own header comment */
        }
        if (EnableOutputsOk(ch) == 0U)
        {
            return ch;
        }
    }
    return 0xFFU;
}

void XrexIo_PollEnableOutputFaults(void)
{
#if defined(BUILD_TARGET_SIMULATOR)
    /* Same class of bug as the other three fault pollers in this file
       (found 2026-09-21, see their own comments) -- EnableOutputsOk()
       reads PG0..PG7, which on this build target are the simulator's
       own Water+Temp/Enerpro fault-INJECTION transmitters (sim_transrex.c),
       not a real "am I connected" signal from a Transrex. Deliberately
       injecting a Water+Temp/Enerpro fault via SIM:FAULT:* would drive
       these same pins LOW, which this unmodified check would misread as
       ENA_OUT/CONTACT_OUT missing and spuriously hard-disable the
       channel. No-op here. */
    return;
#else
    /* Per direct instruction, this precondition is only meaningful once
       actually armed -- unlike Water/Temp/Enerpro/OCP's always-on
       checking, nothing here runs at all while IDLE (or FAULT, though
       SM_ReportEnableOutputFault() itself also short-circuits that
       case defensively). */
    SM_State_t state = SM_GetState();
    if ((state != SM_STATE_ARMED) && (state != SM_STATE_FIRING))
    {
        return;
    }

    for (uint8_t ch = 0U; ch < HRTIM_NUM_CHANNELS; ch++)
    {
        if (PID_GetChannelEnable(ch) == 0U)
        {
            continue;   /* gated -- see this file's own header comment */
        }
        if (EnableOutputsOk(ch) == 0U)
        {
            SM_ReportEnableOutputFault(ch);
        }
    }
#endif /* BUILD_TARGET_SIMULATOR */
}
