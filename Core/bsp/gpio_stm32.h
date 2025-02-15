#ifndef GPIO_STM32_H_
#define GPIO_STM32_H_

#include "interfaces/gpio.h"
#include "stdint.h"
#include "stm32g4xx.h"

GPIO_TypeDef *STM32_GPIO_GetPort(GPIO_Port_ID_T port);
uint32_t STM32_GPIO_GetPin(uint8_t pin);

#endif // GPIO_STM32_H_
