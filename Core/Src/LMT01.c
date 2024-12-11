#include "LMT01.h"
#include "bsp.h"
#include "stm32g4xx.h"
#include "stm32g4xx_hal.h"

/**************************************************************************************************\
* Private memory declarations
\**************************************************************************************************/
static volatile uint16_t lmt01_counter;
static volatile float lmt01_temp = 100;

extern TIM_HandleTypeDef htim6;

/**************************************************************************************************\
* Public functions
\**************************************************************************************************/

/**
 ***************************************************************************************************
 * @brief   LMT01 ISR
 **************************************************************************************************/
void LMT01_ISR(void)
{
    lmt01_counter++;
    BSP_debug_gpio_off();
    
    BSP_LMT01_Timeout_Timer_Enable();
}



/**
 ***************************************************************************************************
 * @brief   LMT01 Timeout Timer Enable/Reset
 **************************************************************************************************/
void BSP_LMT01_Timeout_Timer_Enable() {
    TIM6->CNT = 0;
    __HAL_TIM_CLEAR_FLAG(&htim6, TIM_FLAG_UPDATE);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
    HAL_TIM_Base_Start_IT(&htim6);              // timeout timer
}



/**
 ***************************************************************************************************
 * @brief   LMT01 Timeout 
 **************************************************************************************************/

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM6)
    {
        return;
    }
    BSP_debug_gpio_on();
    HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn);

    lmt01_temp = (lmt01_counter*0.0625)-50;
    lmt01_counter = 0;
}

/**
 ***************************************************************************************************
 *
 * @brief   Read the current encoder data.
 *
 **************************************************************************************************/
int8_t LMT01_Get_Temp(void)
{
    // HAL_NVIC_DisableIRQ(BSP_Get_Motor_Encoder_A_IRQN());
    return lmt01_temp;
    // HAL_NVIC_EnableIRQ(BSP_Get_Motor_Encoder_A_IRQN());
}