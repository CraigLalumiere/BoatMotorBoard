#include "i2c_bus_stm32.h"
#include "assert.h"
#include "stddef.h"

static I2C_HandleTypeDef hi2c1;
static I2C_HandleTypeDef hi2c2;
static I2C_HandleTypeDef hi2c3;
static I2C_HandleTypeDef hi2c4;

I2C_HandleTypeDef *STM32_GetI2CHandle(I2C_Bus_ID_T bus_id)
{
    switch (bus_id)
    {
        case I2C_BUS_ID_1:
            return &hi2c1;
        case I2C_BUS_ID_2:
            return &hi2c2;
        case I2C_BUS_ID_3:
            return &hi2c3;
        case I2C_BUS_ID_4:
            return &hi2c4;
        default:
            return NULL;
    }
}
