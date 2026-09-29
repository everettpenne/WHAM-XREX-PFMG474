/*
 * pfm_input_hw.h -- BSP-private part of the PFM_Input driver (the HAL
 * handles stm32g4xx_it.c needs). Public interface: src/drivers/pfm_input.h.
 */
#ifndef PFM_INPUT_HW_H
#define PFM_INPUT_HW_H

#include "pfm_input.h"
#include "stm32g4xx_hal.h"

/* Exposed so stm32g4xx_it.c's TIM2/TIM3/TIM4/TIM5_IRQHandler()s can
 * call HAL_TIM_IRQHandler() directly, matching hrtim_hw.h's own `extern
 * HRTIM_HandleTypeDef hhrtim1` precedent for the same reason. Not
 * meant to be touched directly by anything else -- go through the
 * functions below. Guarded the same as pfm_input.c's own definitions
 * (absent, not just unused, when this feature is disabled) --
 * stm32g4xx_it.c's 4 IRQHandler()s are guarded identically, so these
 * either both exist together or neither does; the vector table falls
 * back to the startup file's harmless weak Default_Handler for these
 * 4 entries when disabled (never a problem in practice, since the
 * NVIC for them is never enabled either -- PfmInput_Init() is a no-op
 * when disabled). */
#if (PFM_INPUT_FEATURE_ENABLED != 0)
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;

/* Exposed for the same reason as htim2..htim5 above -- stm32g4xx_it.c's
 * 6 DMA1_ChannelN_IRQHandler()s (2026-09-09, the DMA-based capture
 * redesign -- see pfm_input.c's capture-technique history) need to
 * call HAL_DMA_IRQHandler() directly. Indexed the same way as
 * kDesc[]/g_state[] in pfm_input.c (0 = PFM_Input_01, ..., 5 =
 * PFM_Input_06), regardless of which channels PFM_INPUT_ACTIVE_
 * CHANNEL_MASK actually enables -- an inactive channel's entry here
 * is simply never initialized/linked, harmless to declare either way. */
extern DMA_HandleTypeDef pfmInputDma[PFM_INPUT_NUM_CHANNELS];
#endif

#endif /* PFM_INPUT_HW_H */
