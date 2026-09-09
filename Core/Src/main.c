/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "uart.h"
#include "cmd_parser.h"
#include "boot_jump.h"
#include "hrtim.h"
#include "pfm.h"
#include "gate_driver.h"
#include "qspi_test.h"
#include "pfm_input.h"
#include "pid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_HRTIM1_Init(void);
/* USER CODE BEGIN PFP */
static void FixSysTickPriority(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* Must be the very first thing that runs -- before HAL_Init() and
     therefore before any clock/peripheral configuration. See
     boot_jump.c: if cmd_boot() (over USART2) requested a bootloader
     entry on the last reset, this diverts into the ROM bootloader from
     this still-clean, just-reset state and never returns. Otherwise it
     returns immediately (always the case when BOOT_JUMP_FEATURE_ENABLED
     is 0) and startup proceeds normally below -- unconditional on
     purpose, so this call site never needs its own #if. */
  BootJump_CheckAndEnter();

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* Must run immediately after HAL_Init() -- HAL_InitTick() (called
     inside HAL_Init()) sets SysTick to its default TICK_INT_PRIORITY
     (15, the lowest possible), which this overrides. See
     FixSysTickPriority()'s own doc comment (below) for the full
     priority-inversion window this closes -- placed here, as early as
     possible, so nothing between here and HRTIM1_EnableMasterInterrupt()
     (which sets HRTIM1_Master_IRQn's priority, later in USER CODE 2)
     can be exposed to it, however briefly. Ported from the sibling
     PFM-STM32G474 project's main.c, same placement. */
  FixSysTickPriority();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_HRTIM1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  /* Brings the PFM table module to a known-empty, known-stopped state
     before anything else can touch it (a TABLE:* command over UART, or
     a FIRE). Does not start HRTIM outputs -- see PFM_Init()'s own
     comment in pfm.c. */
  PFM_Init();

  /* One explicit GateDriver_CheckFault() call, here at boot, before
     relying on the EXTI interrupt (gate_driver.c, wired up in
     main.c's MX_GPIO_Init()) for everything from here on. EXTI is
     edge-triggered: a pin that is ALREADY in its fault state (per
     GDS_FAULT_POLARITY, ctrlr_config.h) at the moment PE0..PE11 get
     configured for interrupt mode produces no edge of its own -- the
     ISR would simply never fire for it, silently, until something
     eventually toggles that pin. Confirmed relevant on this exact
     board: a GDS? snapshot taken earlier the same day this was added
     showed GateDriverStatus_03 (PE2) already HIGH, which is a fault
     under this build's NORMALLY_LOW polarity. This call catches
     exactly that case -- any fault already present at boot -- instead
     of depending on a future transition that might never come. */
  GateDriver_CheckFault();

  /* QUADSPI bring-up (PE12-PE15/PB10-PB11, W25Q128JVS) -- see
     qspi_test.h for scope. No-op when QSPI_TEST_FEATURE_ENABLED is 0,
     matching the same always-call/resolves-to-something-or-nothing
     pattern already used for BootJump_CheckAndEnter(). */
  QspiTest_Init();

  /* PFM_Input period/duty capture (PA15/PD4/PB2/PC12/PB4/PD12, TIM2/
     TIM3/TIM4/TIM5) -- see pfm_input.h. Configures the timers/GPIO
     only; does not arm or start any capture (that's PfmInput_Arm()
     via PFMIN:CAPTURE, and PfmInput_OnShotStart(), called from
     PFM_Restart() in pfm.c). No-op when PFM_INPUT_FEATURE_ENABLED is
     0. */
  PfmInput_Init();

  /* Closed-loop PID controller (pid.h) -- brings every channel's PID
     state to a known, safe-inert default (all gains 0). Does not touch
     HRTIM or PFM_Input hardware -- that's PID_Start(), via a new
     serial command (not yet added, see docs/changelog.txt). */
  PID_Init();

  uart_init(&uart2, &huart2);

  /* The HRTIM master-repetition interrupt must be enabled now, at
     boot, even though outputs are not yet running: PFM_CycleBoundaryHandler()
     needs to be wired up and ready before the first FIRE, not armed
     reactively at fire time. The ISR itself is a no-op with respect to
     actual switching until HRTIM1_PWM_Start() has been called (by
     cmd_fire() -> PFM_Restart()). Ported from the sibling
     PFM-STM32G474 project's main.c, same placement/rationale. */
  HRTIM1_EnableMasterInterrupt();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* Polls for a completed serial command line and dispatches it. */
    uart_process(&uart2);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /* Copied directly from the sibling PFM-STM32G474 project's
     SystemClock_Config() (170 MHz HSI/PLL config), not derived via
     CubeMX's own auto-resolve -- see AGENTS.md / docs/changelog.txt for
     why: enabling HRTIM1 through the CubeMX GUI on 2026-09-04 triggered
     its "resolve clock issues" auto-fix, which silently picked its own
     unrelated 104 MHz PLL config (PLLN=13) instead. hrtim.c's whole
     timing model -- HRTIM_TIMER_CLK_HZ, the hardcoded Period=1699 for
     100 kHz, the 17-count/100 ns dead time -- assumes exactly this
     170 MHz derivation (HSI 16 MHz / PLLM 4 * PLLN 85 / PLLR 2), matching
     V3's real, hardware-verified config. Voltage scale BOOST and
     FLASH_LATENCY_8 are both required at this SYSCLK -- don't drop
     either while "simplifying" this. */

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_8) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief HRTIM1 Initialization Function
  * @param None
  * @retval None
  *
  * Deliberately NOT auto-generated content -- copied verbatim from the
  * sibling PFM-STM32G474 project's own MX_HRTIM1_Init(), which itself
  * was cut down to this one-line delegate after that project hit the
  * exact same CubeMX-vs-hand-code conflict this project hit on
  * 2026-09-04 (see AGENTS.md). CubeMX still owns this function's
  * existence and call site/ordering (HRTIM1 must init before ADC, if
  * ADC is ever added -- it needs hhrtim1 for trigger config); the
  * actual peripheral configuration is entirely hand-written in
  * HRTIM1_FullInit() (hrtim.c) instead, matching V3/hrtim1's dead-time
  * and Master-sync setup exactly.
  *
  * NOT REGEN-SAFE: this line sits outside any USER CODE marker (same
  * as in the sibling project). If HRTIM1's Mode/Configuration is ever
  * reopened in CubeMX's Pinout & Configuration tool and "Generate
  * Code" is run again, this body WILL be overwritten with CubeMX's own
  * generated calls, and hhrtim1/HAL_HRTIM_MspInit/MspDeInit WILL
  * collide with hrtim.c again (multiple-definition link errors --
  * confirmed by hitting this for real, see docs/changelog.txt). Do not
  * touch HRTIM1's own Mode/Configuration panel in CubeMX again; other
  * peripherals' pinout can still be edited freely.
  */
static void MX_HRTIM1_Init(void)
{
  /* USER CODE BEGIN HRTIM1_Init 0 */
  /* USER CODE END HRTIM1_Init 0 */
  /* USER CODE BEGIN HRTIM1_Init 1 */
  /* USER CODE END HRTIM1_Init 1 */
  HRTIM1_FullInit();
  /* USER CODE BEGIN HRTIM1_Init 2 */
  /* USER CODE END HRTIM1_Init 2 */
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */
  /* huart2.Init.BaudRate below is hand-set to 115200 -- raised from
     9600 on 2026-09-04 after a real-hardware experiment (9600 -> clean
     at 115200 -> garbled/mismatched at 921600, see docs/changelog.txt)
     specifically to speed up TABLE:STEP uploads (~20s -> ~3.3s for a
     500-entry table). This is a deliberate WHAM-XREX-PFMG474-only
     divergence from the sibling PFM-STM32G474 project, which still
     uses 9600 -- do not "fix" this to match V3 without checking
     docs/changelog.txt first. python/wham_serial_flash.py and
     python/pfm_table_upload.py's own APP_BAUD constants were updated
     to match; scpi.py takes baud on its own command line.

     This line lives in CubeMX-generated code, OUTSIDE any USER CODE
     marker -- a "Generate Code" from the .ioc (which has no explicit
     baud rate of its own) would reset it back to HAL's default. As it
     happens HAL's default IS 115200 right now, so a regen wouldn't be
     visibly wrong today -- but that's a coincidence, not a guarantee:
     if this value ever needs to change again, re-apply it here
     explicitly rather than trusting a regen to land on the right
     number by chance. */
  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
  /* Required for uart.c's interrupt-driven receive (HAL_UART_Receive_IT)
     to actually fire HAL_UART_RxCpltCallback() -- without this, RX
     interrupts never reach the NVIC and the serial command parser
     never receives anything. Ported from the sibling PFM-STM32G474
     project, which hit exactly this failure mode.

     Priority 3 (was 2), moved down the same day HRTIM1_Master_IRQn
     moved from 1 to 2 (see that function's own doc comment in
     hrtim.c for why) -- keeps USART2 strictly below every other
     interrupt in this project's scheme, unchanged in relative
     position, just renumbered to make room. */
  HAL_NVIC_SetPriority(USART2_IRQn, 3U, 0U);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* GateDriverStatus_01..12 (PE0..PE11, docs/pin_mapping_v4.csv) --
     interrupt-capable digital inputs, no pull. Originally added
     2026-09-08 as plain GPIO_MODE_INPUT alongside the GDS? diagnostic
     command (commands.c); upgraded the same day to
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
         here: this runs after FixSysTickPriority() (main(), USER CODE
         Init) and after HAL_Init()'s own NVIC setup, the same ordering
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

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
  * @brief  Raises SysTick's NVIC priority off HAL's default lowest
  *         value, before anything else can run at an intermediate
  *         priority.
  *
  * Ported verbatim from the sibling PFM-STM32G474 project's main.c.
  * HAL_InitTick() (called from HAL_Init(), which must run before this)
  * leaves SysTick_IRQn at TICK_INT_PRIORITY (15, the lowest possible
  * priority on this Cortex-M4's 4-bit-preempt NVIC grouping). This
  * project's interrupt priority scheme needs SysTick to be the
  * *highest*-priority interrupt instead, at 0 -- ahead of both
  * HRTIM1_Master_IRQn (1, see HRTIM1_EnableMasterInterrupt() in
  * hrtim.c) and USART2_IRQn (2, see MX_USART2_UART_Init() above) --
  * so that HAL_Delay()/HAL_GetTick() (both driven by SysTick, and used
  * by ordinary HAL driver calls such as HAL_UART_Init() during
  * startup) can never be starved by either of those interrupts firing
  * back-to-back. Left at the HAL default, a sufficiently busy
  * HRTIM1_Master_IRQn or USART2_IRQn could indefinitely delay a
  * HAL_Delay()-based timeout inside some future HAL call, which would
  * look like an unexplained hang rather than a priority bug.
  */
static void FixSysTickPriority(void)
{
    HAL_NVIC_SetPriority(SysTick_IRQn, 0U, 0U);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
