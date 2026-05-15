#ifndef STM32_DISCOVERY_BSP_MANUAL_H
#define STM32_DISCOVERY_BSP_MANUAL_H

#include "stdbool.h"
#include "stdint.h"

#ifdef __cplusplus
extern "C" {
#endif

void BSP_Manual_Config_and_Set_Digital_Output(char port_char, uint8_t pin, bool is_high);
bool BSP_Manual_Config_and_Read_Digital_Input(char port_char, uint8_t pin);

#ifdef __cplusplus
}
#endif

#endif // STM32_DISCOVERY_BSP_MANUAL_H
