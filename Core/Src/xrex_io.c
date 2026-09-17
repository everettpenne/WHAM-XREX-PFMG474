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

/* Shared polarity-interpretation helper -- `bit` is a raw HIGH(1)/LOW(0)
   level, `polarity` is one of ctrlr_config.h's FAULT_POLARITY_NORMALLY_*
   constants. Returns 1 if this level represents a fault under that
   polarity. */
static uint8_t BitIsFault(uint8_t bit, uint32_t polarity)
{
    return (polarity == FAULT_POLARITY_NORMALLY_HIGH) ? (bit == 0U) : (bit != 0U);
}

uint8_t XrexIo_EvaluateGateDriverFault(uint16_t raw12)
{
    for (uint8_t ch = 0U; ch < HRTIM_NUM_CHANNELS; ch++)
    {
        if (PID_GetChannelEnable(ch) == 0U)
        {
            continue;   /* gated: a disabled channel's own Water/Temp/
                           Enerpro pins never count toward a fault */
        }

        uint8_t waterBit   = (uint8_t)((raw12 >> (XR_WATER_FLT_BIT_BASE + ch)) & 1U);
        uint8_t tmpBit     = (uint8_t)((raw12 >> (XR_TMP_FLT_BIT_BASE + ch)) & 1U);
        uint8_t enerproBit = (uint8_t)((raw12 >> (XR_ENERPRO_FLT_BIT_BASE + ch)) & 1U);

        if ((BitIsFault(waterBit, XR_WATER_FLT_POLARITY) != 0U) ||
            (BitIsFault(tmpBit, XR_TMP_FLT_POLARITY) != 0U) ||
            (BitIsFault(enerproBit, XR_ENERPRO_FLT_POLARITY) != 0U))
        {
            return 1U;
        }
    }
    return 0U;
}

void XrexIo_PollOcpFaults(void)
{
    for (uint8_t ch = 0U; ch < HRTIM_NUM_CHANNELS; ch++)
    {
        if (PID_GetChannelEnable(ch) == 0U)
        {
            continue;   /* gated -- see this file's own header comment */
        }

        uint8_t bit = (HAL_GPIO_ReadPin(GPIOF, kOcpPin[ch]) == GPIO_PIN_SET) ? 1U : 0U;
        if (BitIsFault(bit, XR_OCP_FLT_POLARITY) != 0U)
        {
            SM_ReportOcpFault(ch);
        }
    }
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
    for (uint8_t ch = 0U; ch < HRTIM_NUM_CHANNELS; ch++)
    {
        if (PID_GetChannelEnable(ch) == 0U)
        {
            continue;   /* gated -- see this file's own header comment */
        }
        if (EnableOutputsOk(ch) == 0U)
        {
            return 0U;
        }
    }
    return 1U;
}

void XrexIo_PollEnableOutputFaults(void)
{
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
}
