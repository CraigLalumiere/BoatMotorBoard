#ifndef TEST_STM32G4XX_HAL_H_
#define TEST_STM32G4XX_HAL_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    HAL_OK = 0,
    HAL_ERROR = 1,
} HAL_StatusTypeDef;

typedef struct
{
    uint32_t counter;
} TIM_HandleTypeDef;

HAL_StatusTypeDef HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *htim);

#ifdef __cplusplus
}
#endif

#define __HAL_TIM_GET_COUNTER(htim) ((uint16_t) ((htim)->counter))
#define __HAL_TIM_SET_COUNTER(htim, value) ((htim)->counter = (value))

#endif // TEST_STM32G4XX_HAL_H_
