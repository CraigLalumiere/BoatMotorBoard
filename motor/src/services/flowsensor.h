#ifndef FLOWSENSOR_H_
#define FLOWSENSOR_H_

#include "arm_math.h"
#include "stdint.h"
#include "stm32g4xx.h"

#ifdef __cplusplus
extern "C" {
#endif

/**************************************************************************************************\
* Public type definitions
\**************************************************************************************************/

/**************************************************************************************************\
* Public prototypes
\**************************************************************************************************/
void Flow_Sensor_IC_Callback(TIM_HandleTypeDef *htim, uint32_t tim_channel);
void Flow_Sensor_Period_Elapsed_Callback(TIM_HandleTypeDef *htim);
float Flow_Sensor_Read_Hz();

#ifdef __cplusplus
}
#endif
#endif // FLOWSENSOR_H_
