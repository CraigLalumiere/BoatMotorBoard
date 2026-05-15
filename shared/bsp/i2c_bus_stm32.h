#ifndef I2C_BUS_STM32_H_
#define I2C_BUS_STM32_H_

#include "interfaces/i2c_bus.h"
#include "stm32g4xx.h"

I2C_HandleTypeDef *STM32_GetI2CHandle(I2C_Bus_ID_T bus_id);

#endif // SPI_BUS_STM32_H_
