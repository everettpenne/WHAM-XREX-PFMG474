/*
 * qspi_test.c
 *
 * See qspi_test.h for scope (JEDEC Read ID only, against a
 * W25Q128JVS) and for exactly what QSPI_TEST_FEATURE_ENABLED removes
 * and does not remove.
 *
 * MspInit/MspDeInit live here, not in stm32g4xx_hal_msp.c, matching
 * this project's established precedent for hand-added peripherals
 * whose pin config isn't CubeMX-generated -- see hrtim.c's
 * HAL_HRTIM_MspInit() and its own comment on why (AGENTS.md's
 * "NOT REGEN-SAFE" warning about the HRTIM1/CubeMX collision this
 * project hit once already). QUADSPI was never enabled via CubeMX
 * here either, so the same reasoning applies: keeping this
 * self-contained avoids ever colliding with a CubeMX-regenerated
 * stm32g4xx_hal_msp.c, and keeps "remove this file" == "remove the
 * feature," no edits needed elsewhere in the Msp layer.
 */

#include "qspi_test.h"
#include "main.h"

#if (QSPI_TEST_FEATURE_ENABLED != 0)

#include "stm32g4xx_hal_qspi.h"

/* JEDEC Read ID / Read Identification -- 0x9F. Universal across
 * essentially every SPI-NOR flash, W25Q128JVS included (Winbond
 * datasheet: "Read JEDEC ID (9Fh)", 1-1-1, no address, no dummy
 * cycles, 3 response bytes: Manufacturer ID (Winbond = 0xEF),
 * Memory Type, Capacity). Deliberately issued in plain 1-line mode --
 * this instruction works identically whether or not the chip's Quad
 * Enable (QE) status-register bit has ever been set, so it is the
 * correct FIRST test for new hardware: it can't be broken by a QE
 * config step that hasn't happened yet. */
#define W25Q_CMD_READ_JEDEC_ID  (0x9FU)
#define W25Q_JEDEC_ID_LEN       (3U)

/* Kept short (not HAL_QSPI_TIMEOUT_DEFAULT_VALUE's 5 s) -- this is a
 * tiny, one-shot diagnostic transaction issued synchronously from a
 * serial command handler (commands.c); a genuinely wedged QUADSPI bus
 * should fail fast, not hang the whole command link for 5 seconds. */
#define QSPI_TEST_TIMEOUT_MS    (100U)

/* AHB (SYSCLK, see main.c's SystemClock_Config()) = 170 MHz on this
 * project's clock tree -- QspiTest_Init() explicitly selects SYSCLK as
 * the QUADSPI kernel clock source (HAL_RCCEx_PeriphCLKConfig(),
 * RCC_QSPICLKSOURCE_SYSCLK) rather than relying on it already being
 * the POR-default (it is, but this project prefers explicit config
 * over "verify no other reset value" -- see hrtim.h/hrtim.c's own
 * clock-derivation comments for the same philosophy applied to HRTIM).
 * ClockPrescaler = 33 -> QSPI_CLK = 170 MHz / (33+1) = 5 MHz -- a
 * deliberately conservative first-bring-up rate (W25Q128JVS's normal
 * READ command is rated up to 50 MHz, FAST_READ variants higher still),
 * chosen for signal-integrity margin on newly-fabricated hardware, not
 * because 5 MHz is any kind of hard limit. Raise it once basic
 * connectivity is confirmed, if throughput ever matters here. */
#define QSPI_CLOCK_PRESCALER    (33U)

/* W25Q128JVS = 128 Mbit = 16 MiB = 2^24 bytes -> HAL's FlashSize field
 * is (address bits - 1), i.e. 24 - 1 = 23. Not actually consulted by
 * a Read-ID transaction (no address phase), but set correctly anyway
 * so this struct is ready for a real memory command later without a
 * silent trap for whoever adds one. */
#define QSPI_FLASH_SIZE_W25Q128 (23U)

static QSPI_HandleTypeDef hqspi;

void QspiTest_Init(void)
{
    RCC_PeriphCLKInitTypeDef qspiClkInit = {0};

    qspiClkInit.PeriphClockSelection = RCC_PERIPHCLK_QSPI;
    qspiClkInit.QspiClockSelection   = RCC_QSPICLKSOURCE_SYSCLK;
    if (HAL_RCCEx_PeriphCLKConfig(&qspiClkInit) != HAL_OK)
    {
        Error_Handler();
    }

    hqspi.Instance = QUADSPI;
    hqspi.Init.ClockPrescaler     = QSPI_CLOCK_PRESCALER;
    hqspi.Init.FifoThreshold      = 4U;
    /* BUGFIX (confirmed on real hardware): QSPI_SAMPLE_SHIFTING_HALFCYCLE
       -- the usual recommended default, meant to compensate for signal
       propagation delay -- produced a stable, repeatable QSPI:ID?
       result of "DE 80 30" against this exact W25Q128JVS, instead of
       the chip's real EF 40 18. That is not noise: EF4018 << 1 (as one
       continuous 24-bit stream, not per-byte) == DE8030 exactly -- the
       whole response was sampled exactly one bit late. Switching to
       QSPI_SAMPLE_SHIFTING_NONE fixed it immediately (EF 40 18,
       confirmed repeatable). Root cause not chased further than that
       (a plausible story: at this deliberately conservative 5 MHz and
       whatever trace length this board's QSPI net actually is, a half
       cycle of extra sample delay overshoots past the valid data
       window rather than centering in it) -- if the clock rate is ever
       raised well past 5 MHz, this may need revisiting rather than
       assumed to still be correct. */
    hqspi.Init.SampleShifting     = QSPI_SAMPLE_SHIFTING_NONE;
    hqspi.Init.FlashSize          = QSPI_FLASH_SIZE_W25Q128;
    hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;
    hqspi.Init.ClockMode          = QSPI_CLOCK_MODE_0;
    hqspi.Init.FlashID            = QSPI_FLASH_ID_1;
    hqspi.Init.DualFlash          = QSPI_DUALFLASH_DISABLE;

    if (HAL_QSPI_Init(&hqspi) != HAL_OK)
    {
        Error_Handler();
    }
}

uint8_t QspiTest_ReadId(uint8_t id[3])
{
    QSPI_CommandTypeDef cmd = {0};

    cmd.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
    cmd.Instruction       = W25Q_CMD_READ_JEDEC_ID;
    cmd.AddressMode       = QSPI_ADDRESS_NONE;
    cmd.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    cmd.DummyCycles       = 0U;
    cmd.DataMode          = QSPI_DATA_1_LINE;
    cmd.NbData            = W25Q_JEDEC_ID_LEN;
    cmd.DdrMode           = QSPI_DDR_MODE_DISABLE;
    cmd.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

    if (HAL_QSPI_Command(&hqspi, &cmd, QSPI_TEST_TIMEOUT_MS) != HAL_OK)
    {
        return 0U;
    }

    if (HAL_QSPI_Receive(&hqspi, id, QSPI_TEST_TIMEOUT_MS) != HAL_OK)
    {
        return 0U;
    }

    return 1U;
}

/* Called by HAL_QSPI_Init() (above). Pin config: PE12-PE15 = BK1_IO0-3,
 * PB10 = CLK, PB11 = BK1_NCS, all AF10 (GPIO_AF10_QUADSPI) -- confirmed
 * via the same datasheet column-position method already trusted in
 * this project (Table 13, cross-checked against the already-verified
 * PC10/HRTIM1_FLT6 = AF13 result as a calibration point before
 * trusting this one), not assumed from the HAL header's existence
 * alone. NCS gets GPIO_PULLUP (a flagged defensive-default assumption,
 * same reasoning as PC10's fault input elsewhere in this project: no
 * external pull confirmed, and an idle-high/deselected NCS before this
 * peripheral drives it is the safe default for a chip-select line) --
 * CLK/IO0-3 get GPIO_NOPULL, matching how every other actively-driven
 * bus pin in this project is configured. */
void HAL_QSPI_MspInit(QSPI_HandleTypeDef *hqspiHandle)
{
    GPIO_InitTypeDef gpioInit = {0};
    (void)hqspiHandle;

    __HAL_RCC_QSPI_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    /* PB10 (CLK), PB11 (NCS) */
    gpioInit.Mode      = GPIO_MODE_AF_PP;
    gpioInit.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gpioInit.Alternate = GPIO_AF10_QUADSPI;

    gpioInit.Pin  = GPIO_PIN_10;
    gpioInit.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &gpioInit);

    gpioInit.Pin  = GPIO_PIN_11;
    gpioInit.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &gpioInit);

    /* PE12-PE15 (IO0-IO3) */
    gpioInit.Pin  = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    gpioInit.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOE, &gpioInit);
}

void HAL_QSPI_MspDeInit(QSPI_HandleTypeDef *hqspiHandle)
{
    (void)hqspiHandle;

    __HAL_RCC_QSPI_FORCE_RESET();
    __HAL_RCC_QSPI_RELEASE_RESET();

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10 | GPIO_PIN_11);
    HAL_GPIO_DeInit(GPIOE, GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15);
}

#else /* QSPI_TEST_FEATURE_ENABLED == 0 */

void QspiTest_Init(void)
{
    /* Feature disabled -- deliberately does nothing. main.c's call
       site never needs its own #if because of this. */
}

uint8_t QspiTest_ReadId(uint8_t id[3])
{
    /* Feature disabled -- deliberately does nothing, id[] left
       untouched. */
    (void)id;
    return 0U;
}

#endif /* QSPI_TEST_FEATURE_ENABLED */
