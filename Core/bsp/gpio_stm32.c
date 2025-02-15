#include "gpio_stm32.h"
#include "qsafe.h"
#include "stdbool.h"
#include "stm32g4xx.h"

Q_DEFINE_THIS_MODULE("gpio_stm32");

GPIO_TypeDef *STM32_GPIO_GetPort(GPIO_Port_ID_T port)
{
    Q_ASSERT(port < GPIO_PORT_MAX_SUPPORTED);
    switch (port)
    {
        case GPIO_PORT_A_ID:
            return GPIOA;
        case GPIO_PORT_B_ID:
            return GPIOB;
        case GPIO_PORT_C_ID:
            return GPIOC;
        case GPIO_PORT_D_ID:
            return GPIOD;
        case GPIO_PORT_E_ID:
            return GPIOE;
        case GPIO_PORT_F_ID:
            return GPIOF;
        // case GPIO_PORT_G_ID:
        //     return GPIOG;
        // case GPIO_PORT_H_ID:
        //     return GPIOH;
        // case GPIO_PORT_I_ID:
        //     return GPIOI;
        // case GPIO_PORT_J_ID:
        //     return GPIOJ;
        // case GPIO_PORT_K_ID:
        //     return GPIOK;
        default:
            return NULL;
    }
}

uint32_t STM32_GPIO_GetPin(uint8_t pin)
{
    Q_ASSERT(pin < 16);
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
