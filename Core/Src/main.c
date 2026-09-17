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
#include "state_machine.h"
#include "xrex_io.h"
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

  /* Top-level operating-state machine (state_machine.h), added
     2026-09-13 -- see that header for the full design (IDLE/ARMED/
     FIRING/FAULT). SM_Init() alone would set IDLE unconditionally,
     which would be WRONG if the GateDriver_CheckFault() call above
     (line ~141, deliberately earlier -- boot-time GateDriverStatus
     check, before this state machine even existed) already found a
     real pre-existing fault: an immediate SM_PollFaults() right after
     SM_Init() picks that up, so a board that boots with a fault
     already present correctly starts in FAULT, not IDLE. XrexIo_PollOcpFaults()
     (xrex_io.h, added 2026-09-17) is called right alongside it for the
     same reason -- a board that boots with a real OCP condition already
     present should also start in FAULT, not IDLE. */
  SM_Init();
  SM_PollFaults();
  XrexIo_PollOcpFaults();

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
    /* Fault detection that must work regardless of state -- see
       state_machine.h's own SM_PollFaults() comment for why this needs
       to run here too, not just from PID_Update() (which only runs
       while FIRING). Cheap: both underlying reads are simple flag
       checks, not full re-scans. XrexIo_PollOcpFaults() (xrex_io.h,
       added 2026-09-17) runs at this same cadence for the same reason
       -- OCP is polled, not EXTI-driven (see xrex_io.h's own header
       comment for why), so it needs this same "regardless of state"
       call site to work at all. */
    SM_PollFaults();
    XrexIo_PollOcpFaults();
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

  /* PF15 (Fiber_Enable, docs/pin_mapping_v4.csv -- confirmed GPI there),
     added 2026-09-16 for the external-enable interlock
     (state_machine.c's own external-enable section has the full design).
     Plain polled input, no EXTI -- state_machine.c's SM_PollFaults()
     already checks this at the same cadence (main loop + every real
     PID_Update() tick, ~1kHz while FIRING) General Fault's own two
     hardware sources get; an interrupt-driven path wasn't judged
     necessary for this signal and wasn't built -- if tighter latency
     ever matters, that's a real decision to make explicitly, not
     something this comment claims was already covered.

     GPIO_PULLDOWN, NOT this project's usual GPIO_NOPULL for actively-
     driven inputs (GateDriverStatus above, PFM_Input, QUADSPI) --
     deliberate: an unconnected/floating PF15 must read LOW (no
     permission granted), never an undefined level that could
     accidentally read HIGH and silently permit firing. GateDriverStatus's
     NOPULL is fine on its own pins because floating-reads-as-fault is
     already the safe direction there; the same reasoning would be
     UNSAFE for this one, where floating-reads-as-enabled would be the
     dangerous direction instead. */
  {
      GPIO_InitTypeDef extEnableInit = {0};

      __HAL_RCC_GPIOF_CLK_ENABLE();

      extEnableInit.Pin  = GPIO_PIN_15;
      extEnableInit.Mode = GPIO_MODE_INPUT;
      extEnableInit.Pull = GPIO_PULLDOWN;
      HAL_GPIO_Init(GPIOF, &extEnableInit);
  }

  /* PG10 -- a fiber-optic emergency-stop input, added 2026-09-17 for
     the emergency-stop feature (state_machine.c's own emergency-stop
     section has the full design). docs/pin_mapping_v4.csv labels this
     net "NRST" -- confirmed directly with the user this is a stale/
     incorrect label, NOT this MCU's own reset function (that's a
     separate, dedicated silicon pin, not part of any GPIO port). A
     100nF cap to ground already exists on this net (per the user) --
     fine/beneficial mild filtering for a deliberately slow-changing
     safety signal, not a concern the way it would be for a fast-
     switching PFM_Input capture pin.

     PB8 (this board's BOOT0 net) was considered FIRST for this purpose
     and REJECTED -- a live FLASH_OPTR register read
     (DIAGnostic:OPTBytes?, commands.c) confirmed nSWBOOT0=1 on this
     chip, meaning PB8 is genuinely sampled for boot-mode selection on
     EVERY reset, not just first power-on; an active-LOW E-stop's own
     idle (non-emergency) HIGH state is exactly the "boot into the ROM
     bootloader instead of the application" condition here, which would
     silently break normal boot on any ordinary reset during non-
     emergency operation. PG10, an ordinary GPIO pin with no boot-time
     role at all, has none of that risk.

     Plain polled input, no EXTI -- same reasoning as PF15 just above:
     state_machine.c's SM_PollFaults() already checks this at the same
     cadence (main loop + every real PID_Update() tick) General Fault's
     own two hardware sources get.

     GPIO_PULLDOWN, matching PF15's own fail-safe reasoning (even
     though the SAFE direction happens to be the same polarity by
     coincidence here, not because the reasoning is identical): an
     unconnected/floating PG10 must read LOW -- E-stop ASSERTED, the
     safe default for a stop function (a broken/disconnected E-stop
     wire should read as "stop," never as "all clear"). */
  {
      GPIO_InitTypeDef eStopInit = {0};

      __HAL_RCC_GPIOG_CLK_ENABLE();

      eStopInit.Pin  = GPIO_PIN_10;
      eStopInit.Mode = GPIO_MODE_INPUT;
      eStopInit.Pull = GPIO_PULLDOWN;
      HAL_GPIO_Init(GPIOG, &eStopInit);
  }

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
     safe reasoning already applied to PF15/PG10 above: with
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

  /* PD1 ("GPOut_12" in the V4 column, docs/pin_mapping_v4.csv --
     confirmed GPO there), added 2026-09-16 as a generic, software-
     driven diagnostic output. Direct request; PF13 was proposed first
     and corrected -- PF13 is actually documented "GPInput_12" (an
     INPUT) in the same CSV, a different pin from the one actually
     named "GPOut_12" (PD1), same class of name/pin mismatch as the
     earlier PC14-vs-PF15 correction. Immediate use: driven by
     DIAGnostic:GPOut12 (commands.c) and physically looped to PF15
     (Fiber_Enable) by the operator, letting the external-enable/
     external-trigger feature above be exercised entirely from the
     serial console -- precise, repeatable control over PF15's level
     at exactly the right moments -- rather than needing a hand-
     operated bench jumper/switch. Not tied to that use case in the
     pin config itself, just today's reason for wanting it: a generic
     level output, nothing PF15-specific baked in here.

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
