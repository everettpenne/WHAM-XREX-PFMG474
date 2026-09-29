/*
 * hrtim_hw.h -- BSP-private part of the HRTIM driver: the HAL handle,
 * HAL-valued settings and raw register helpers. The public, HAL-free
 * interface is src/drivers/hrtim.h. Only src/bsp and Core/ include this.
 */
#ifndef HRTIM_HW_H
#define HRTIM_HW_H

#include "hrtim.h"
#include "main.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_hrtim.h"

extern HRTIM_HandleTypeDef hhrtim1;

/* Prescaler setting for the PID heartbeat Master timebase; must match
 * HRTIM_MASTER_PID_PRESCALE_DIV (hrtim.h, where the choice is explained). */
#define HRTIM_MASTER_PID_PRESCALE   HRTIM_PRESCALERRATIO_DIV4

/* PC10 / HRTIM1_FLT6 (docs/pin_mapping_v4.csv), active-low (confirmed
 * against the actual fault-sensing circuit, 2026-09-08 -- do not
 * assume this generalizes to some other polarity without checking the
 * hardware again). See HRTIM1_FullInit()'s fault-config block. */
#define HRTIM_FAULT_CHANNEL     HRTIM_FAULT_6
#define HRTIM_FAULT_FLAG        HRTIM_FLAG_FLT6

/* Register access helpers -- indexed by channel (0..HRTIM_NUM_CHANNELS-1
 * for the per-timer getters) rather than one named function per
 * channel, so callers don't need to grow with N. Master's own PER
 * register is channel-independent; its CMPx registers are indexed
 * 1..4 (matching MCMP1R-MCMP4R, used for channels 1..4's phase). */
volatile uint32_t *HRTIM1_GetMasterPerRegAddress(void);
volatile uint32_t *HRTIM1_GetMasterCmpRegAddress(uint8_t masterCompareUnit);

volatile uint32_t *HRTIM1_GetTimerPerRegAddress(uint8_t channel);
volatile uint32_t *HRTIM1_GetTimerCmp1RegAddress(uint8_t channel);

#endif /* HRTIM_HW_H */
