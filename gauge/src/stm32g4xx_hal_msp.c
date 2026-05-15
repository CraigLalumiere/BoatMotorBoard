/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file         stm32g4xx_hal_msp.c
 * @brief        This file provides code for the MSP Initialization
 *               and de-Initialization codes.
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
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN Define */

/* USER CODE END Define */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN Macro */

/* USER CODE END Macro */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* External functions --------------------------------------------------------*/
/* USER CODE BEGIN ExternalFunctions */

/* USER CODE END ExternalFunctions */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
/**
 * Initializes the Global MSP.
 */
void HAL_MspInit(void)
{
    /* USER CODE BEGIN MspInit 0 */

    /* USER CODE END MspInit 0 */

    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();

    /* System interrupt init*/

    /** Configure the internal voltage reference buffer voltage scale
     */
    HAL_SYSCFG_VREFBUF_VoltageScalingConfig(SYSCFG_VREFBUF_VOLTAGE_SCALE2);

    /** Configure the internal voltage reference buffer high impedance mode
     */
    HAL_SYSCFG_VREFBUF_HighImpedanceConfig(SYSCFG_VREFBUF_HIGH_IMPEDANCE_DISABLE);

    /** Enable the Internal Voltage Reference buffer
     */
    HAL_SYSCFG_EnableVREFBUF();

    /** Disable the internal Pull-Up in Dead Battery pins of UCPD peripheral
     */
    HAL_PWREx_DisableUCPDDeadBattery();

    /* USER CODE BEGIN MspInit 1 */

    /* USER CODE END MspInit 1 */
}

/**
 * @brief DAC MSP Initialization
 * This function configures the hardware resources used in this example
 * @param hdac: DAC handle pointer
 * @retval None
 */
void HAL_DAC_MspInit(DAC_HandleTypeDef *hdac)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if (hdac->Instance == DAC1)
    {
        /* USER CODE BEGIN DAC1_MspInit 0 */

        /* USER CODE END DAC1_MspInit 0 */
        /* Peripheral clock enable */
        __HAL_RCC_DAC1_CLK_ENABLE();

        __HAL_RCC_GPIOA_CLK_ENABLE();
        /**DAC1 GPIO Configuration
        PA4     ------> DAC1_OUT1
        PA5     ------> DAC1_OUT2
        */
        GPIO_InitStruct.Pin  = GPIO_PIN_4 | GPIO_PIN_5;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /* USER CODE BEGIN DAC1_MspInit 1 */

        /* USER CODE END DAC1_MspInit 1 */
    }
    else if (hdac->Instance == DAC3)
    {
        /* USER CODE BEGIN DAC3_MspInit 0 */

        /* USER CODE END DAC3_MspInit 0 */
        /* Peripheral clock enable */
        __HAL_RCC_DAC3_CLK_ENABLE();
        /* USER CODE BEGIN DAC3_MspInit 1 */

        /* USER CODE END DAC3_MspInit 1 */
    }
}

/**
 * @brief DAC MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param hdac: DAC handle pointer
 * @retval None
 */
void HAL_DAC_MspDeInit(DAC_HandleTypeDef *hdac)
{
    if (hdac->Instance == DAC1)
    {
        /* USER CODE BEGIN DAC1_MspDeInit 0 */

        /* USER CODE END DAC1_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_DAC1_CLK_DISABLE();

        /**DAC1 GPIO Configuration
        PA4     ------> DAC1_OUT1
        PA5     ------> DAC1_OUT2
        */
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_4 | GPIO_PIN_5);

        /* USER CODE BEGIN DAC1_MspDeInit 1 */

        /* USER CODE END DAC1_MspDeInit 1 */
    }
    else if (hdac->Instance == DAC3)
    {
        /* USER CODE BEGIN DAC3_MspDeInit 0 */

        /* USER CODE END DAC3_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_DAC3_CLK_DISABLE();
        /* USER CODE BEGIN DAC3_MspDeInit 1 */

        /* USER CODE END DAC3_MspDeInit 1 */
    }
}

/**
 * @brief FDCAN MSP Initialization
 * This function configures the hardware resources used in this example
 * @param hfdcan: FDCAN handle pointer
 * @retval None
 */
void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef *hfdcan)
{
    GPIO_InitTypeDef GPIO_InitStruct       = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
    if (hfdcan->Instance == FDCAN2)
    {
        /* USER CODE BEGIN FDCAN2_MspInit 0 */

        /* USER CODE END FDCAN2_MspInit 0 */

        /** Initializes the peripherals clocks
         */
        PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
        PeriphClkInit.FdcanClockSelection  = RCC_FDCANCLKSOURCE_PCLK1;
        if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
        {
            Error_Handler();
        }

        /* Peripheral clock enable */
        __HAL_RCC_FDCAN_CLK_ENABLE();

        __HAL_RCC_GPIOB_CLK_ENABLE();
        /**FDCAN2 GPIO Configuration
        PB12     ------> FDCAN2_RX
        PB13     ------> FDCAN2_TX
        */
        GPIO_InitStruct.Pin       = CAN_RX_Pin | CAN_TX_Pin;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN2;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        /* FDCAN2 interrupt Init */
        HAL_NVIC_SetPriority(FDCAN2_IT0_IRQn, 4, 0);
        HAL_NVIC_EnableIRQ(FDCAN2_IT0_IRQn);
        /* USER CODE BEGIN FDCAN2_MspInit 1 */

        /* USER CODE END FDCAN2_MspInit 1 */
    }
}

/**
 * @brief FDCAN MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param hfdcan: FDCAN handle pointer
 * @retval None
 */
void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef *hfdcan)
{
    if (hfdcan->Instance == FDCAN2)
    {
        /* USER CODE BEGIN FDCAN2_MspDeInit 0 */

        /* USER CODE END FDCAN2_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_FDCAN_CLK_DISABLE();

        /**FDCAN2 GPIO Configuration
        PB12     ------> FDCAN2_RX
        PB13     ------> FDCAN2_TX
        */
        HAL_GPIO_DeInit(GPIOB, CAN_RX_Pin | CAN_TX_Pin);

        /* FDCAN2 interrupt DeInit */
        HAL_NVIC_DisableIRQ(FDCAN2_IT0_IRQn);
        /* USER CODE BEGIN FDCAN2_MspDeInit 1 */

        /* USER CODE END FDCAN2_MspDeInit 1 */
    }
}

/**
 * @brief I2C MSP Initialization
 * This function configures the hardware resources used in this example
 * @param hi2c: I2C handle pointer
 * @retval None
 */
void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
    GPIO_InitTypeDef GPIO_InitStruct       = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
    if (hi2c->Instance == I2C2)
    {
        /* USER CODE BEGIN I2C2_MspInit 0 */

        /* USER CODE END I2C2_MspInit 0 */

        /** Initializes the peripherals clocks
         */
        PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C2;
        PeriphClkInit.I2c2ClockSelection   = RCC_I2C2CLKSOURCE_PCLK1;
        if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
        {
            Error_Handler();
        }

        __HAL_RCC_GPIOA_CLK_ENABLE();
        /**I2C2 GPIO Configuration
        PA8     ------> I2C2_SDA
        PA9     ------> I2C2_SCL
        */
        GPIO_InitStruct.Pin       = GPIO_PIN_8 | GPIO_PIN_9;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /* Peripheral clock enable */
        __HAL_RCC_I2C2_CLK_ENABLE();
        /* USER CODE BEGIN I2C2_MspInit 1 */

        /* USER CODE END I2C2_MspInit 1 */
    }
}

/**
 * @brief I2C MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param hi2c: I2C handle pointer
 * @retval None
 */
void HAL_I2C_MspDeInit(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C2)
    {
        /* USER CODE BEGIN I2C2_MspDeInit 0 */

        /* USER CODE END I2C2_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_I2C2_CLK_DISABLE();

        /**I2C2 GPIO Configuration
        PA8     ------> I2C2_SDA
        PA9     ------> I2C2_SCL
        */
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_8);

        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9);

        /* USER CODE BEGIN I2C2_MspDeInit 1 */

        /* USER CODE END I2C2_MspDeInit 1 */
    }
}

/**
 * @brief OPAMP MSP Initialization
 * This function configures the hardware resources used in this example
 * @param hopamp: OPAMP handle pointer
 * @retval None
 */
void HAL_OPAMP_MspInit(OPAMP_HandleTypeDef *hopamp)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if (hopamp->Instance == OPAMP1)
    {
        /* USER CODE BEGIN OPAMP1_MspInit 0 */

        /* USER CODE END OPAMP1_MspInit 0 */

        __HAL_RCC_GPIOA_CLK_ENABLE();
        /**OPAMP1 GPIO Configuration
        PA2     ------> OPAMP1_VOUT
        */
        GPIO_InitStruct.Pin  = GPIO_PIN_2;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /* USER CODE BEGIN OPAMP1_MspInit 1 */

        /* USER CODE END OPAMP1_MspInit 1 */
    }
}

/**
 * @brief OPAMP MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param hopamp: OPAMP handle pointer
 * @retval None
 */
void HAL_OPAMP_MspDeInit(OPAMP_HandleTypeDef *hopamp)
{
    if (hopamp->Instance == OPAMP1)
    {
        /* USER CODE BEGIN OPAMP1_MspDeInit 0 */

        /* USER CODE END OPAMP1_MspDeInit 0 */

        /**OPAMP1 GPIO Configuration
        PA2     ------> OPAMP1_VOUT
        */
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2);

        /* USER CODE BEGIN OPAMP1_MspDeInit 1 */

        /* USER CODE END OPAMP1_MspDeInit 1 */
    }
}

/**
 * @brief TIM_Base MSP Initialization
 * This function configures the hardware resources used in this example
 * @param htim_base: TIM_Base handle pointer
 * @retval None
 */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim_base)
{
    if (htim_base->Instance == TIM8)
    {
        /* USER CODE BEGIN TIM8_MspInit 0 */

        /* USER CODE END TIM8_MspInit 0 */
        /* Peripheral clock enable */
        __HAL_RCC_TIM8_CLK_ENABLE();
        /* USER CODE BEGIN TIM8_MspInit 1 */

        /* USER CODE END TIM8_MspInit 1 */
    }
}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if (htim->Instance == TIM8)
    {
        /* USER CODE BEGIN TIM8_MspPostInit 0 */

        /* USER CODE END TIM8_MspPostInit 0 */

        __HAL_RCC_GPIOA_CLK_ENABLE();
        /**TIM8 GPIO Configuration
        PA15     ------> TIM8_CH1
        */
        GPIO_InitStruct.Pin       = GPIO_PIN_15;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF2_TIM8;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /* USER CODE BEGIN TIM8_MspPostInit 1 */

        /* USER CODE END TIM8_MspPostInit 1 */
    }
}
/**
 * @brief TIM_Base MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param htim_base: TIM_Base handle pointer
 * @retval None
 */
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef *htim_base)
{
    if (htim_base->Instance == TIM8)
    {
        /* USER CODE BEGIN TIM8_MspDeInit 0 */

        /* USER CODE END TIM8_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_TIM8_CLK_DISABLE();
        /* USER CODE BEGIN TIM8_MspDeInit 1 */

        /* USER CODE END TIM8_MspDeInit 1 */
    }
}

/**
 * @brief PCD MSP Initialization
 * This function configures the hardware resources used in this example
 * @param hpcd: PCD handle pointer
 * @retval None
 */
void HAL_PCD_MspInit(PCD_HandleTypeDef *hpcd)
{
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
    if (hpcd->Instance == USB)
    {
        /* USER CODE BEGIN USB_MspInit 0 */

        /* USER CODE END USB_MspInit 0 */

        /** Initializes the peripherals clocks
         */
        PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
        PeriphClkInit.UsbClockSelection    = RCC_USBCLKSOURCE_PLL;
        if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
        {
            Error_Handler();
        }

        /* Peripheral clock enable */
        __HAL_RCC_USB_CLK_ENABLE();
        /* USB interrupt Init */
        HAL_NVIC_SetPriority(USB_HP_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(USB_HP_IRQn);
        HAL_NVIC_SetPriority(USB_LP_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(USB_LP_IRQn);
        /* USER CODE BEGIN USB_MspInit 1 */

        /* USER CODE END USB_MspInit 1 */
    }
}

/**
 * @brief PCD MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param hpcd: PCD handle pointer
 * @retval None
 */
void HAL_PCD_MspDeInit(PCD_HandleTypeDef *hpcd)
{
    if (hpcd->Instance == USB)
    {
        /* USER CODE BEGIN USB_MspDeInit 0 */

        /* USER CODE END USB_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_USB_CLK_DISABLE();

        /* USB interrupt DeInit */
        HAL_NVIC_DisableIRQ(USB_HP_IRQn);
        HAL_NVIC_DisableIRQ(USB_LP_IRQn);
        /* USER CODE BEGIN USB_MspDeInit 1 */

        /* USER CODE END USB_MspDeInit 1 */
    }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
