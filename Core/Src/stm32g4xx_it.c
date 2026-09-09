/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32g4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32g4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "uart.h"
#include "hrtim.h"
#include "pfm.h"
#include "gate_driver.h"
#include "pfm_input.h"
#include "pid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
extern UART_HandleTypeDef huart2;   /* defined in main.c; same local-extern
                                        convention used in the sibling
                                        PFM-STM32G474 project rather than a
                                        shared declaration in main.h */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32G4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g4xx.s).                    */
/******************************************************************************/

/* USER CODE BEGIN 1 */

/**
  * @brief  Rx Transfer completed callback.
  *
  * Called by the HAL after each single-byte interrupt-driven UART
  * receive armed by uart_init()/uart_rx_callback() in uart.c. Routes
  * to uart_rx_callback(), which accumulates the byte into the line
  * buffer and re-arms the next single-byte receive.
  * @param  huart  Pointer to the UART handle that completed reception.
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        uart_rx_callback(&uart2);
    }
}

/**
  * @brief  This function handles USART2 global interrupt.
  *
  * Drives uart.c's interrupt-driven single-byte receive. HAL_UART_IRQHandler
  * internally invokes HAL_UART_RxCpltCallback() (defined above) once a byte
  * has been received.
  */
void USART2_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart2);
}

/**
  * @brief  This function handles the HRTIM1 Master timer global interrupt.
  *
  * Not a CubeMX-generated handler -- HRTIM1_Master_IRQn is enabled and
  * primed for this one flag by HRTIM1_EnableMasterInterrupt() (hrtim.c),
  * called once at boot from main.c.
  *
  * REWIRED, 2026-09-09, closed-loop PID architecture (see
  * docs/changelog.txt's design-decision entry and pid.h's own header
  * comment): Master's repetition event is no longer a shared PWM
  * carrier's own period boundary (WHAM-PFMG474-V4's switching-supply
  * meaning, and PFM_CycleBoundaryHandler()'s own reason for existing --
  * that function is still here, inherited, still callable, just no
  * longer wired to this ISR) -- it's now the PID control loop's
  * fixed-rate heartbeat, reprogrammed onto Master's own timebase to
  * PID_LOOP_RATE_HZ (ctrlr_config.h) by hrtim.c's HRTIM1_FullInit().
  * Fires at that fixed rate while the Master counter is running, i.e.
  * only during/after PID_Start() -- never before, and never again
  * after PID_Stop() stops the counters (HRTIM1_PWM_Stop(), same
  * reasoning as the old comment here: leaving the counters running
  * after outputs stop would starve the main loop with this ISR firing
  * forever).
  */
void HRTIM1_Master_IRQHandler(void)
{
    if (__HAL_HRTIM_MASTER_GET_FLAG(&hhrtim1, HRTIM_MASTER_FLAG_MREP) != RESET)
    {
        __HAL_HRTIM_MASTER_CLEAR_IT(&hhrtim1, HRTIM_MASTER_IT_MREP);
        PID_Update();
    }
}

/**
  * @brief  GateDriverStatus_01..12 (PE0..PE11) fault interrupt --
  *         EXTI0..EXTI4, EXTI9_5, and EXTI15_10 global interrupts.
  *
  * Not CubeMX-generated -- PE0..PE11 (main.c's MX_GPIO_Init()) are
  * configured GPIO_MODE_IT_RISING_FALLING, so any edge on any of these
  * 12 pins lands in one of these 7 vectors (EXTI0..EXTI4 are individual
  * lines; EXTI9_5 covers PE5..PE9; EXTI15_10 covers PE10..PE11 here --
  * lines 12..15 are not this group's and are not checked/cleared by
  * these handlers). Each handler clears only its own pin's pending bit
  * (__HAL_GPIO_EXTI_CLEAR_IT keys off the pin/line number, not the
  * port -- fine here since nothing else in this project uses EXTI0..15
  * on any other port), then calls GateDriver_CheckFault()
  * (gate_driver.c), which re-reads and evaluates ALL 12 pins itself --
  * so it doesn't matter here which specific pin's edge woke us up, only
  * that at least one did. See gate_driver.h for what CheckFault() does
  * on an actual fault (immediate PFM_ForceStop() + latch).
  */
void EXTI0_IRQHandler(void)
{
    if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_0) != 0U)
    {
        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_0);
        GateDriver_CheckFault();
    }
}

void EXTI1_IRQHandler(void)
{
    if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_1) != 0U)
    {
        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_1);
        GateDriver_CheckFault();
    }
}

void EXTI2_IRQHandler(void)
{
    if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_2) != 0U)
    {
        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_2);
        GateDriver_CheckFault();
    }
}

void EXTI3_IRQHandler(void)
{
    if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_3) != 0U)
    {
        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_3);
        GateDriver_CheckFault();
    }
}

void EXTI4_IRQHandler(void)
{
    if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_4) != 0U)
    {
        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_4);
        GateDriver_CheckFault();
    }
}

void EXTI9_5_IRQHandler(void)
{
    uint16_t pin;
    uint8_t  fired = 0U;

    for (pin = GPIO_PIN_5; pin <= GPIO_PIN_9; pin <<= 1U)
    {
        if (__HAL_GPIO_EXTI_GET_IT(pin) != 0U)
        {
            __HAL_GPIO_EXTI_CLEAR_IT(pin);
            fired = 1U;
        }
    }

    if (fired != 0U)
    {
        GateDriver_CheckFault();
    }
}

void EXTI15_10_IRQHandler(void)
{
    uint16_t pin;
    uint8_t  fired = 0U;

    /* Only PE10/PE11 belong to GateDriverStatus -- lines 12..15 are
       not checked/cleared here (nothing else in this project uses
       them, and PE12 is unconnected on this board, see
       docs/pin_mapping_v4.csv). */
    for (pin = GPIO_PIN_10; pin <= GPIO_PIN_11; pin <<= 1U)
    {
        if (__HAL_GPIO_EXTI_GET_IT(pin) != 0U)
        {
            __HAL_GPIO_EXTI_CLEAR_IT(pin);
            fired = 1U;
        }
    }

    if (fired != 0U)
    {
        GateDriver_CheckFault();
    }
}

/**
  * @brief  These four functions handle the TIM2/TIM3/TIM4/TIM5 global
  *         interrupts driving pfm_input.c's PFM_Input capture
  *         (PA15/PD4/PB2/PC12/PB4/PD12).
  *
  * Not CubeMX-generated -- these 4 timers were never enabled through
  * CubeMX here. Uses the generic HAL_TIM_IRQHandler() + overridden
  * HAL_TIM_IC_CaptureCallback() dispatch (pfm_input.c), the same style
  * already used for USART2 (HAL_UART_IRQHandler() +
  * HAL_UART_RxCpltCallback(), uart.c) -- a better fit here than
  * HRTIM1_Master_IRQHandler()'s hand-rolled raw-flag style, since this
  * is 4 instances x up to 2 channels each, exactly what the HAL's own
  * per-instance dispatch is designed for. htim2/htim3/htim4/htim5 are
  * pfm_input.c's own handles, exposed via pfm_input.h the same way
  * hrtim.h exposes hhrtim1.
  *
  * Guarded on PFM_INPUT_FEATURE_ENABLED, matching pfm_input.h's own
  * guard on the htim2..htim5 declarations these reference -- when
  * disabled, these 4 functions don't exist at all (rather than
  * existing and referencing undefined externs), and the vector table
  * falls back to the startup file's weak Default_Handler for these
  * entries, harmlessly, since the NVIC for them is never enabled
  * either when this feature is off.
  */
#if (PFM_INPUT_FEATURE_ENABLED != 0)
void TIM2_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim2);
}

void TIM3_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim3);
}

void TIM4_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim4);
}

void TIM5_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim5);
}
#endif /* PFM_INPUT_FEATURE_ENABLED */

/* DMA1_Channel1..6_IRQHandler() -- pfm_input.c's DMA-based capture
 * redesign (2026-09-09, see that file's own capture-technique
 * history). One DMA channel per PFM_Input_01..06 (kDesc[]'s own
 * dmaInstance/dmaIrqn table), each simply dispatching to
 * HAL_DMA_IRQHandler() for its own pfmInputDma[] handle (exposed via
 * pfm_input.h the same way htim2..htim5 are) -- the standard HAL
 * pattern, same style as the TIM2-5 handlers just above. Nothing else
 * in this project uses any DMA channel (confirmed before choosing
 * these), so there is no conflict to guard against. Guarded on
 * PFM_INPUT_FEATURE_ENABLED for the same reason as the TIM2-5
 * handlers -- when disabled, these don't exist at all, and the vector
 * table falls back to the weak Default_Handler, harmlessly, since the
 * NVIC for them is never enabled either. */
#if (PFM_INPUT_FEATURE_ENABLED != 0)
void DMA1_Channel1_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&pfmInputDma[0]);
}

void DMA1_Channel2_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&pfmInputDma[1]);
}

void DMA1_Channel3_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&pfmInputDma[2]);
}

void DMA1_Channel4_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&pfmInputDma[3]);
}

void DMA1_Channel5_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&pfmInputDma[4]);
}

void DMA1_Channel6_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&pfmInputDma[5]);
}
#endif /* PFM_INPUT_FEATURE_ENABLED */

/* USER CODE END 1 */
