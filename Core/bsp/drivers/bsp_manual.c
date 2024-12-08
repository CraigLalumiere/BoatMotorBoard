#include "bsp_manual.h"
#include "qsafe.h"
#include "stm32g4xx.h"
#include "stm32g4xx_hal_gpio.h"

Q_DEFINE_THIS_MODULE("bsp_manual")

static GPIO_TypeDef *STM32_GPIO_GetPortHandle(char port_char);
static uint32_t STM32_GPIO_GetPinHandle(uint8_t pin);

void BSP_Manual_Config_and_Set_Digital_Output(char port_char, uint8_t pin, bool is_high)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_TypeDef *gpio_port = STM32_GPIO_GetPortHandle(port_char);
    Q_ASSERT(gpio_port != 0U);

    uint32_t gpio_pin = STM32_GPIO_GetPinHandle(pin);
    Q_ASSERT(gpio_pin != 0U);

    GPIO_InitStruct.Pin   = gpio_pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(gpio_port, &GPIO_InitStruct);

    HAL_GPIO_WritePin(gpio_port, gpio_pin, is_high ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

bool BSP_Manual_Config_and_Read_Digital_Input(char port_char, uint8_t pin)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_TypeDef *gpio_port = STM32_GPIO_GetPortHandle(port_char);
    Q_ASSERT(gpio_port != 0U);

    uint32_t gpio_pin = STM32_GPIO_GetPinHandle(pin);
    Q_ASSERT(gpio_pin != 0U);

    GPIO_InitStruct.Pin   = gpio_pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(gpio_port, &GPIO_InitStruct);

    return HAL_GPIO_ReadPin(gpio_port, gpio_pin) == GPIO_PIN_SET;
}

static GPIO_TypeDef *STM32_GPIO_GetPortHandle(char port_char)
{
    switch (port_char)
    {
        case 'A':
            return GPIOA;
        case 'B':
            return GPIOB;
        case 'C':
            return GPIOC;
        case 'D':
            return GPIOD;
        case 'E':
            return GPIOE;
        case 'F':
            return GPIOF;
        case 'G':
            return GPIOG;
        default:
            return 0U;
    }
}

static uint32_t STM32_GPIO_GetPinHandle(uint8_t pin)
{
    switch (pin)
    {
        case 0:
            return GPIO_PIN_0;
        case 1:
            return GPIO_PIN_1;
        case 2:
            return GPIO_PIN_2;
        case 3:
            return GPIO_PIN_3;
        case 4:
            return GPIO_PIN_4;
        case 5:
            return GPIO_PIN_5;
        case 6:
            return GPIO_PIN_6;
        case 7:
            return GPIO_PIN_7;
        case 8:
            return GPIO_PIN_8;
        case 9:
            return GPIO_PIN_9;
        case 10:
            return GPIO_PIN_10;
        case 11:
            return GPIO_PIN_11;
        case 12:
            return GPIO_PIN_12;
        case 13:
            return GPIO_PIN_13;
        case 14:
            return GPIO_PIN_14;
        case 15:
            return GPIO_PIN_15;
        default:
            return 0U;
    }
}
