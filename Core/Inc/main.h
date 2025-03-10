/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PRESSURE_EOC_Pin         GPIO_PIN_0
#define PRESSURE_EOC_GPIO_Port   GPIOA
#define PRESSURE_RST_Pin         GPIO_PIN_1
#define PRESSURE_RST_GPIO_Port   GPIOA
#define RED_SENSE_2_Pin          GPIO_PIN_5
#define RED_SENSE_2_GPIO_Port    GPIOA
#define RED_SENSE_1_Pin          GPIO_PIN_6
#define RED_SENSE_1_GPIO_Port    GPIOA
#define ORANGE_SENSE_1_Pin       GPIO_PIN_7
#define ORANGE_SENSE_1_GPIO_Port GPIOA
#define nBUZZER_SENSE_Pin        GPIO_PIN_0
#define nBUZZER_SENSE_GPIO_Port  GPIOB
#define ORANGE_SENSE_2_Pin       GPIO_PIN_1
#define ORANGE_SENSE_2_GPIO_Port GPIOB
#define CAN_FLT_Pin              GPIO_PIN_2
#define CAN_FLT_GPIO_Port        GPIOB
#define NEUTRAL_DETECT_Pin       GPIO_PIN_10
#define NEUTRAL_DETECT_GPIO_Port GPIOB
#define START_DET_Pin            GPIO_PIN_11
#define START_DET_GPIO_Port      GPIOB
#define TACH_Pin                 GPIO_PIN_14
#define TACH_GPIO_Port           GPIOB
#define VBAT_SENSE_Pin           GPIO_PIN_15
#define VBAT_SENSE_GPIO_Port     GPIOB
#define VBUS_SENSE_Pin           GPIO_PIN_10
#define VBUS_SENSE_GPIO_Port     GPIOA
#define DEBUG_GPIO_Pin           GPIO_PIN_4
#define DEBUG_GPIO_GPIO_Port     GPIOB
#define FW_LED_Pin               GPIO_PIN_5
#define FW_LED_GPIO_Port         GPIOB
#define LMT01_Pin                GPIO_PIN_6
#define LMT01_GPIO_Port          GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
