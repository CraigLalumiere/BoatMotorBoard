/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    stm32g4xx_it.c
 * @brief   Interrupt Service Routines.
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
#include "stm32g4xx_it.h"
#include "main.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "i2c_bus_stm32.h"
#include "qpc.h"
#include "reset.h"
#include "tusb.h"
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern I2C_HandleTypeDef hi2c2;
extern TIM_HandleTypeDef htim15;
extern UART_HandleTypeDef huart2;
extern PCD_HandleTypeDef hpcd_USB_FS;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
 * @brief This function handles Hard fault interrupt.
 */
void HardFault_Handler(void)
{
    /* USER CODE BEGIN HardFault_IRQn 0 */
    // https://interrupt.memfault.com/blog/cortex-m-hardfault-debug#relevant-status-registers
    uint32_t hfsr  = SCB->HFSR;
    uint32_t cfsr  = SCB->CFSR;
    uint32_t abfsr = 0U; // not used
    uint32_t mmfar = SCB->MMFAR;
    Reset_DoResetWithReason(RESET_REASON_HARD_FAULT, hfsr, cfsr, abfsr, mmfar);
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
    // https://interrupt.memfault.com/blog/cortex-m-hardfault-debug#relevant-status-registers
    uint32_t hfsr  = SCB->HFSR;
    uint32_t cfsr  = SCB->CFSR;
    uint32_t abfsr = 0U; // not used
    uint32_t mmfar = SCB->MMFAR;
    Reset_DoResetWithReason(RESET_REASON_MEM_MANAGE_FAULT, hfsr, cfsr, abfsr, mmfar);
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
    // https://interrupt.memfault.com/blog/cortex-m-hardfault-debug
    uint32_t hfsr  = SCB->HFSR;
    uint32_t cfsr  = SCB->CFSR;
    uint32_t abfsr = 0U; // not used
    uint32_t mmfar = SCB->MMFAR;
    Reset_DoResetWithReason(RESET_REASON_BUS_FAULT, hfsr, cfsr, abfsr, mmfar);
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
    // https://interrupt.memfault.com/blog/cortex-m-hardfault-debug
    uint32_t hfsr  = SCB->HFSR;
    uint32_t cfsr  = SCB->CFSR;
    uint32_t abfsr = 0U; // not used
    uint32_t mmfar = SCB->MMFAR;
    Reset_DoResetWithReason(RESET_REASON_USAGE_FAULT, hfsr, cfsr, abfsr, mmfar);
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

/******************************************************************************/
/* STM32G4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g4xx.s).                    */
/******************************************************************************/

/**
 * @brief This function handles USB high priority interrupt remap.
 */
void USB_HP_IRQHandler(void)
{
    /* USER CODE BEGIN USB_HP_IRQn 0 */
    tud_int_handler(0);
    /* USER CODE END USB_HP_IRQn 0 */
    /* USER CODE BEGIN USB_HP_IRQn 1 */

    /* USER CODE END USB_HP_IRQn 1 */
}

/**
 * @brief This function handles USB low priority interrupt remap.
 */
void USB_LP_IRQHandler(void)
{
    /* USER CODE BEGIN USB_LP_IRQn 0 */
    tud_int_handler(0);
    /* USER CODE END USB_LP_IRQn 0 */
    /* USER CODE BEGIN USB_LP_IRQn 1 */

    /* USER CODE END USB_LP_IRQn 1 */
}

/**
 * @brief This function handles TIM1 break interrupt and TIM15 global interrupt.
 */
void TIM1_BRK_TIM15_IRQHandler(void)
{
    /* USER CODE BEGIN TIM1_BRK_TIM15_IRQn 0 */
    QK_ISR_ENTRY();
    /* USER CODE END TIM1_BRK_TIM15_IRQn 0 */
    HAL_TIM_IRQHandler(&htim15);
    /* USER CODE BEGIN TIM1_BRK_TIM15_IRQn 1 */
    QK_ISR_EXIT();
    /* USER CODE END TIM1_BRK_TIM15_IRQn 1 */
}

/**
 * @brief This function handles I2C2 event interrupt / I2C2 wake-up interrupt through EXTI line 24.
 */
void I2C2_EV_IRQHandler(void)
{
    /* USER CODE BEGIN I2C2_EV_IRQn 0 */
    QK_ISR_ENTRY();
    HAL_I2C_EV_IRQHandler(STM32_GetI2CHandle(I2C_BUS_ID_2));
    /* USER CODE END I2C2_EV_IRQn 0 */
    /* USER CODE BEGIN I2C2_EV_IRQn 1 */
    QK_ISR_EXIT();
    /* USER CODE END I2C2_EV_IRQn 1 */
}

/**
 * @brief This function handles I2C2 error interrupt.
 */
void I2C2_ER_IRQHandler(void)
{
    /* USER CODE BEGIN I2C2_ER_IRQn 0 */
    QK_ISR_ENTRY();
    HAL_I2C_ER_IRQHandler(STM32_GetI2CHandle(I2C_BUS_ID_2));
    /* USER CODE END I2C2_ER_IRQn 0 */
    /* USER CODE BEGIN I2C2_ER_IRQn 1 */
    QK_ISR_EXIT();
    /* USER CODE END I2C2_ER_IRQn 1 */
}

/**
 * @brief This function handles USART2 global interrupt / USART2 wake-up interrupt through EXTI
 * line 26.
 */
void USART2_IRQHandler(void)
{
    /* USER CODE BEGIN USART2_IRQn 0 */
    QK_ISR_ENTRY();
    /* USER CODE END USART2_IRQn 0 */
    HAL_UART_IRQHandler(&huart2);
    /* USER CODE BEGIN USART2_IRQn 1 */
    QK_ISR_EXIT();
    /* USER CODE END USART2_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/**
 * @brief This function handles USB wakeup interrupt remap.
 */
void USBWakeUp_IRQHandler(void)
{
    tud_int_handler(0);
}

/* USER CODE END 1 */
