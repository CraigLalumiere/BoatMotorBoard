#include "LMT01.h"
#include "bsp.h"
#include "stm32g4xx.h"
#include "stm32g4xx_hal.h"

/**************************************************************************************************\
* Private memory declarations
\**************************************************************************************************/
static volatile uint16_t lmt01_counter;
static volatile float lmt01_temp = 100;

extern TIM_HandleTypeDef htim8; // counter

/**************************************************************************************************\
* Public functions
\**************************************************************************************************/

void LMT01_ctor() {
    // Start the pulse-counter timer
    HAL_TIM_Base_Start_IT(&htim8);
}



/**
 ***************************************************************************************************
 *
 * @brief   Read the current encoder data.
 *
 **************************************************************************************************/
int8_t LMT01_Get_Temp(void)
{
    lmt01_temp = (lmt01_counter*0.0625)-50;
    return lmt01_temp;
}

uint16_t LMT01_Get_Counter(void)
{
    return __HAL_TIM_GET_COUNTER(&htim8);
}