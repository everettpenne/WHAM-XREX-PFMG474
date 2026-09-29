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
#include "app.h"
#include "boot_jump.h"
#include "boot_diag.h"
#include "board_io.h"
#include "hrtim_hw.h"
#include "mcu.h"
#include "uart_hw.h"
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

  /* Boot diagnostics (boot_diag.h): keep the previous boot's record for
     the !BOOT banner and start this boot's. */
  BootDiag_Begin();

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* Must run immediately after HAL_Init() -- HAL_InitTick() (called
     inside HAL_Init()) sets SysTick to its default TICK_INT_PRIORITY
     (15, the lowest possible), which this overrides. See
     Mcu_SetSysTickHighestPriority()'s own doc comment (mcu.c) for the full
     priority-inversion window this closes -- placed here, as early as
     possible, so nothing between here and HRTIM1_EnableMasterInterrupt()
     (which sets HRTIM1_Master_IRQn's priority, later in USER CODE 2)
     can be exposed to it, however briefly. Ported from the sibling
     PFM-STM32G474 project's main.c, same placement. */
  Mcu_SetSysTickHighestPriority();
  BOOT_DIAG_STAGE(BD_STAGE_HAL_INIT);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  BOOT_DIAG_STAGE(BD_STAGE_CLOCK);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_HRTIM1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  /* Module init in dependency order, then the USART2 command link (app.c). */
  uart_bind(&uart2, &huart2);
  App_Init();
  BOOT_DIAG_STAGE(BD_STAGE_APP_INIT);
  App_SendBootBanner();
  BOOT_DIAG_STAGE(BD_STAGE_MAIN_LOOP);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    App_Poll();
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
  BOOT_DIAG_STAGE(BD_STAGE_HRTIM);
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
  BOOT_DIAG_STAGE(BD_STAGE_USART);
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
  BoardIo_Init();
  BOOT_DIAG_STAGE(BD_STAGE_GPIO);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
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
  g_bootDiag.errCount++;
  g_bootDiag.errStage = g_bootDiag.lastStage;
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
