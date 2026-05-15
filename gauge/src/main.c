/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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

#include "blinky.h"
#include "box_to_box.h"
#include "bsp.h"
#include "cli.h"
#include "director.h"
#include "posted_signals.h"
#include "qpc.h"
#include "reset.h"
#include "shared_i2c.h"
#include "shared_i2c_events.h"
#include "usb.h"
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// QP Priorities for Active Objects must be unique
// Lower number is lower priority
// 0 is reserved, lowest available is 1
typedef enum
{
    AO_RESERVED = 0U,
    AO_PRIO_BLINKY,
    AO_PRIO_APP_CLI,
    AO_PRIO_DIRECTOR,
    // AO_PRIO_SHARED_I2C2,
    AO_PRIO_BOX_TO_BOX,
    AO_PRIO_USB,
} AO_Priority_T;

extern const QActive *AO_SharedI2C2;

typedef struct
{
    union
    {
        QEvt base_event;
        FloatEvent_T float_event;
    } small_messages;
} SmallMessageUnion_T;
typedef struct
{
    union
    {
        QEvt someMultipleQEvt[4];
        DebugForceFaultEvent_T fault_event;
    } medium_messages;
} MediumMessageUnion_T;
typedef struct
{
    union
    {
        QEvt base_event;
        FaultGeneratedEvent_T fault_event;
        CAN_Message_Received_Event_T can_event;
    } large_messages;
} LongMessageUnion_T;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac1;
DAC_HandleTypeDef hdac3;

FDCAN_HandleTypeDef hfdcan2;

I2C_HandleTypeDef hi2c2;

OPAMP_HandleTypeDef hopamp1;

TIM_HandleTypeDef htim8;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USB_PCD_Init(void);
static void MX_TIM8_Init(void);
static void MX_DAC1_Init(void);
static void MX_DAC3_Init(void);
static void MX_FDCAN2_Init(void);
static void MX_I2C2_Init(void);
static void MX_OPAMP1_Init(void);
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
    Reset_Init();
    QF_init(); // initialize the framework and the underlying RT kernel
    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    // IN CUBEMX, BE SURE TO SET INTERRUPT PRIORITY FOR ALL "QP AWARE" INTERRUPTS TO AT LEAST 4
    // SINCE QF_AWARE_ISR_CMSIS_PRI is 3

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_USB_PCD_Init();
    MX_TIM8_Init();
    MX_DAC1_Init();
    MX_DAC3_Init();
    MX_FDCAN2_Init();
    MX_I2C2_Init();
    MX_OPAMP1_Init();
    /* USER CODE BEGIN 2 */

    uint16_t priority = QF_AWARE_ISR_CMSIS_PRI;
    (void) priority; // be sure to set your QP aware interrupt priority in Cube MX to at least this

    BSP_Init();

    size_t smallsize  = sizeof(SmallMessageUnion_T);
    size_t mediumsize = sizeof(MediumMessageUnion_T);
    size_t largesize  = sizeof(LongMessageUnion_T);
    (void) smallsize;
    (void) mediumsize;
    (void) largesize;

    // initialize event pools
    static QF_MPOOL_EL(SmallMessageUnion_T) smlPoolSto[10];
    QF_poolInit(smlPoolSto, sizeof(smlPoolSto), sizeof(smlPoolSto[0]));

    static QF_MPOOL_EL(MediumMessageUnion_T) mediumPoolSto[20];
    QF_poolInit(mediumPoolSto, sizeof(mediumPoolSto), sizeof(mediumPoolSto[0]));

    static QF_MPOOL_EL(LongMessageUnion_T) longPoolSto[20];
    QF_poolInit(longPoolSto, sizeof(longPoolSto), sizeof(longPoolSto[0]));

    // initialize publish-subscribe
    static QSubscrList subscrSto[PUBSUB_MAX_SIG];
    QActive_psInit(subscrSto, Q_DIM(subscrSto));

    // instantiate and start AOs/threads...

    static QEvt const *blinkyQueueSto[10];
    Blinky_ctor();
    QACTIVE_START(
        AO_Blinky,
        AO_PRIO_BLINKY,        // QP prio. of the AO
        blinkyQueueSto,        // event queue storage
        Q_DIM(blinkyQueueSto), // queue length [events]
        (void *) 0,
        0U,          // no stack storage
        (void *) 0); // no initialization param

    static QEvt const *app_cli_QueueSto[10];
    AppCLI_ctor(BSP_Get_Serial_IO_Interface_USB0());
    QACTIVE_START(
        AO_AppCLI,
        AO_PRIO_APP_CLI,         // QP prio. of the AO
        app_cli_QueueSto,        // event queue storage
        Q_DIM(app_cli_QueueSto), // queue length [events]
        (void *) 0,              // stack storage (not used in QK)
        0U,                      // stack size [bytes] (not used in QK)
        (void *) 0);             // no initialization param

    static QEvt const *box_to_box_QueueSto[20];
    Box_To_Box_ctor();
    QACTIVE_START(
        AO_BOX_TO_BOX,
        AO_PRIO_BOX_TO_BOX,         // QP prio. of the AO
        box_to_box_QueueSto,        // event queue storage
        Q_DIM(box_to_box_QueueSto), // queue length [events]
        (void *) 0,                 // stack storage (not used in QK)
        0U,                         // stack size [bytes] (not used in QK)
        (void *) 0);                // no initialization param

    static QEvt const *directorQueueSto[10];
    Director_ctor();
    QACTIVE_START(
        AO_DIRECTOR,
        AO_PRIO_DIRECTOR,        // QP prio. of the AO
        directorQueueSto,        // event queue storage
        Q_DIM(directorQueueSto), // queue length [events]
        (void *) 0,
        0U,          // no stack storage
        (void *) 0); // no initialization param

    static QEvt const *usb_QueueSto[10];
    USB_ctor();
    QACTIVE_START(
        AO_USB,
        AO_PRIO_USB,         // QP prio. of the AO
        usb_QueueSto,        // event queue storage
        Q_DIM(usb_QueueSto), // queue length [events]
        (void *) 0,          // stack storage (not used in QK)
        0U,                  // stack size [bytes] (not used in QK)
        (void *) 0);         // no initialization param

    return QF_run(); // run the QF application
                     /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    // while (1)
    // {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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

    /** Configure the main internal regulator output voltage
     */
    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM            = RCC_PLLM_DIV1;
    RCC_OscInitStruct.PLL.PLLN            = 18;
    RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV5;
    RCC_OscInitStruct.PLL.PLLQ            = RCC_PLLQ_DIV6;
    RCC_OscInitStruct.PLL.PLLR            = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 |
        RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief DAC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_DAC1_Init(void)
{
    /* USER CODE BEGIN DAC1_Init 0 */

    /* USER CODE END DAC1_Init 0 */

    DAC_ChannelConfTypeDef sConfig = {0};

    /* USER CODE BEGIN DAC1_Init 1 */

    /* USER CODE END DAC1_Init 1 */

    /** DAC Initialization
     */
    hdac1.Instance = DAC1;
    if (HAL_DAC_Init(&hdac1) != HAL_OK)
    {
        Error_Handler();
    }

    /** DAC channel OUT1 config
     */
    sConfig.DAC_HighFrequency           = DAC_HIGH_FREQUENCY_INTERFACE_MODE_AUTOMATIC;
    sConfig.DAC_DMADoubleDataMode       = DISABLE;
    sConfig.DAC_SignedFormat            = DISABLE;
    sConfig.DAC_SampleAndHold           = DAC_SAMPLEANDHOLD_DISABLE;
    sConfig.DAC_Trigger                 = DAC_TRIGGER_NONE;
    sConfig.DAC_Trigger2                = DAC_TRIGGER_NONE;
    sConfig.DAC_OutputBuffer            = DAC_OUTPUTBUFFER_ENABLE;
    sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_EXTERNAL;
    sConfig.DAC_UserTrimming            = DAC_TRIMMING_FACTORY;
    if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }

    /** DAC channel OUT2 config
     */
    if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN DAC1_Init 2 */

    /* USER CODE END DAC1_Init 2 */
}

/**
 * @brief DAC3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_DAC3_Init(void)
{
    /* USER CODE BEGIN DAC3_Init 0 */

    /* USER CODE END DAC3_Init 0 */

    DAC_ChannelConfTypeDef sConfig = {0};

    /* USER CODE BEGIN DAC3_Init 1 */

    /* USER CODE END DAC3_Init 1 */

    /** DAC Initialization
     */
    hdac3.Instance = DAC3;
    if (HAL_DAC_Init(&hdac3) != HAL_OK)
    {
        Error_Handler();
    }

    /** DAC channel OUT1 config
     */
    sConfig.DAC_HighFrequency           = DAC_HIGH_FREQUENCY_INTERFACE_MODE_AUTOMATIC;
    sConfig.DAC_DMADoubleDataMode       = DISABLE;
    sConfig.DAC_SignedFormat            = DISABLE;
    sConfig.DAC_SampleAndHold           = DAC_SAMPLEANDHOLD_DISABLE;
    sConfig.DAC_Trigger                 = DAC_TRIGGER_NONE;
    sConfig.DAC_Trigger2                = DAC_TRIGGER_NONE;
    sConfig.DAC_OutputBuffer            = DAC_OUTPUTBUFFER_DISABLE;
    sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_INTERNAL;
    sConfig.DAC_UserTrimming            = DAC_TRIMMING_FACTORY;
    if (HAL_DAC_ConfigChannel(&hdac3, &sConfig, DAC_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN DAC3_Init 2 */

    /* USER CODE END DAC3_Init 2 */
}

/**
 * @brief FDCAN2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_FDCAN2_Init(void)
{
    /* USER CODE BEGIN FDCAN2_Init 0 */

    /* USER CODE END FDCAN2_Init 0 */

    /* USER CODE BEGIN FDCAN2_Init 1 */

    /* USER CODE END FDCAN2_Init 1 */
    hfdcan2.Instance                  = FDCAN2;
    hfdcan2.Init.ClockDivider         = FDCAN_CLOCK_DIV1;
    hfdcan2.Init.FrameFormat          = FDCAN_FRAME_FD_NO_BRS;
    hfdcan2.Init.Mode                 = FDCAN_MODE_NORMAL;
    hfdcan2.Init.AutoRetransmission   = ENABLE;
    hfdcan2.Init.TransmitPause        = DISABLE;
    hfdcan2.Init.ProtocolException    = DISABLE;
    hfdcan2.Init.NominalPrescaler     = 4;
    hfdcan2.Init.NominalSyncJumpWidth = 1;
    hfdcan2.Init.NominalTimeSeg1      = 13;
    hfdcan2.Init.NominalTimeSeg2      = 2;
    hfdcan2.Init.DataPrescaler        = 4;
    hfdcan2.Init.DataSyncJumpWidth    = 1;
    hfdcan2.Init.DataTimeSeg1         = 13;
    hfdcan2.Init.DataTimeSeg2         = 2;
    hfdcan2.Init.StdFiltersNbr        = 1;
    hfdcan2.Init.ExtFiltersNbr        = 0;
    hfdcan2.Init.TxFifoQueueMode      = FDCAN_TX_FIFO_OPERATION;
    if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN FDCAN2_Init 2 */

    /* USER CODE END FDCAN2_Init 2 */
}

/**
 * @brief I2C2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C2_Init(void)
{
    /* USER CODE BEGIN I2C2_Init 0 */

    /* USER CODE END I2C2_Init 0 */

    /* USER CODE BEGIN I2C2_Init 1 */

    /* USER CODE END I2C2_Init 1 */
    hi2c2.Instance              = I2C2;
    hi2c2.Init.Timing           = 0x60715075;
    hi2c2.Init.OwnAddress1      = 0;
    hi2c2.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
    hi2c2.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
    hi2c2.Init.OwnAddress2      = 0;
    hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c2.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
    hi2c2.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c2) != HAL_OK)
    {
        Error_Handler();
    }

    /** Configure Analogue filter
     */
    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
    {
        Error_Handler();
    }

    /** Configure Digital filter
     */
    if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN I2C2_Init 2 */

    /* USER CODE END I2C2_Init 2 */
}

/**
 * @brief OPAMP1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_OPAMP1_Init(void)
{
    /* USER CODE BEGIN OPAMP1_Init 0 */

    /* USER CODE END OPAMP1_Init 0 */

    /* USER CODE BEGIN OPAMP1_Init 1 */

    /* USER CODE END OPAMP1_Init 1 */
    hopamp1.Instance                    = OPAMP1;
    hopamp1.Init.PowerMode              = OPAMP_POWERMODE_NORMALSPEED;
    hopamp1.Init.Mode                   = OPAMP_FOLLOWER_MODE;
    hopamp1.Init.NonInvertingInput      = OPAMP_NONINVERTINGINPUT_DAC;
    hopamp1.Init.InternalOutput         = DISABLE;
    hopamp1.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
    hopamp1.Init.UserTrimming           = OPAMP_TRIMMING_FACTORY;
    if (HAL_OPAMP_Init(&hopamp1) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN OPAMP1_Init 2 */

    /* USER CODE END OPAMP1_Init 2 */
}

/**
 * @brief TIM8 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM8_Init(void)
{
    /* USER CODE BEGIN TIM8_Init 0 */

    /* USER CODE END TIM8_Init 0 */

    TIM_ClockConfigTypeDef sClockSourceConfig           = {0};
    TIM_MasterConfigTypeDef sMasterConfig               = {0};
    TIM_OC_InitTypeDef sConfigOC                        = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

    /* USER CODE BEGIN TIM8_Init 1 */

    /* USER CODE END TIM8_Init 1 */
    htim8.Instance               = TIM8;
    htim8.Init.Prescaler         = 71;
    htim8.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim8.Init.Period            = 65535;
    htim8.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim8.Init.RepetitionCounter = 0;
    htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
    {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger  = TIM_TRGO_RESET;
    sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
    sMasterConfig.MasterSlaveMode      = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sConfigOC.OCMode       = TIM_OCMODE_PWM1;
    sConfigOC.Pulse        = 0;
    sConfigOC.OCPolarity   = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode   = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState  = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
    sBreakDeadTimeConfig.OffStateRunMode  = TIM_OSSR_DISABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBreakDeadTimeConfig.LockLevel        = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime         = 0;
    sBreakDeadTimeConfig.BreakState       = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity    = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.BreakFilter      = 0;
    sBreakDeadTimeConfig.BreakAFMode      = TIM_BREAK_AFMODE_INPUT;
    sBreakDeadTimeConfig.Break2State      = TIM_BREAK2_DISABLE;
    sBreakDeadTimeConfig.Break2Polarity   = TIM_BREAK2POLARITY_HIGH;
    sBreakDeadTimeConfig.Break2Filter     = 0;
    sBreakDeadTimeConfig.Break2AFMode     = TIM_BREAK_AFMODE_INPUT;
    sBreakDeadTimeConfig.AutomaticOutput  = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM8_Init 2 */

    /* USER CODE END TIM8_Init 2 */
    HAL_TIM_MspPostInit(&htim8);
}

/**
 * @brief USB Initialization Function
 * @param None
 * @retval None
 */
static void MX_USB_PCD_Init(void)
{
    /* USER CODE BEGIN USB_Init 0 */

    /* USER CODE END USB_Init 0 */

    /* USER CODE BEGIN USB_Init 1 */

    /* USER CODE END USB_Init 1 */
    hpcd_USB_FS.Instance                     = USB;
    hpcd_USB_FS.Init.dev_endpoints           = 8;
    hpcd_USB_FS.Init.speed                   = PCD_SPEED_FULL;
    hpcd_USB_FS.Init.phy_itface              = PCD_PHY_EMBEDDED;
    hpcd_USB_FS.Init.Sof_enable              = DISABLE;
    hpcd_USB_FS.Init.low_power_enable        = DISABLE;
    hpcd_USB_FS.Init.lpm_enable              = DISABLE;
    hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
    if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN USB_Init 2 */

    /* USER CODE END USB_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    /* USER CODE BEGIN MX_GPIO_Init_1 */
    /* USER CODE END MX_GPIO_Init_1 */

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOB, BACKLIGHT_EN_Pin | FW_LED_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin : BACKLIGHT_DET_Pin */
    GPIO_InitStruct.Pin  = BACKLIGHT_DET_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(BACKLIGHT_DET_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : CAN_FLT_Pin */
    GPIO_InitStruct.Pin  = CAN_FLT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(CAN_FLT_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : BACKLIGHT_EN_Pin FW_LED_Pin */
    GPIO_InitStruct.Pin   = BACKLIGHT_EN_Pin | FW_LED_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* USER CODE BEGIN MX_GPIO_Init_2 */
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
    while (1)
    {
    }
    /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
