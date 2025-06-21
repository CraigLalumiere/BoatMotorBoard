#ifndef BSP_H_
#define BSP_H_

#include "interfaces/i2c_interface.h"
#include "interfaces/serial_interface.h"
#include "stdint.h"
#include <stdbool.h>

/**************************************************************************************************\
* Public macros
\**************************************************************************************************/

#define BSP_TICKS_PER_SEC         1000U
#define MILLISECONDS_TO_TICKS(ms) ((ms) * ((BSP_TICKS_PER_SEC) / 1000))

/**************************************************************************************************\
* Public type definitions
\**************************************************************************************************/

/**************************************************************************************************\
* Public memory declarations
\**************************************************************************************************/

void BSP_Init(void);
void BSP_terminate(int16_t result);

/**************************************************************************************************\
* Public prototypes
\**************************************************************************************************/

/**
 ***************************************************************************************************
 * @brief   Millisecond Tick
 **************************************************************************************************/
uint32_t BSP_Get_Milliseconds_Tick(void);

/**
 ***************************************************************************************************
 * @brief   Functions for blinky LED
 **************************************************************************************************/
void BSP_LED_On(void);
void BSP_LED_Off(void);
// void BSP_debug_gpio_on(void);
// void BSP_debug_gpio_off(void);
// void BSP_debug_gpio_toggle(void);

/**
 ***************************************************************************************************
 * @brief   Retrieve Serial IO interface for the USB Interface serial comms channels
 **************************************************************************************************/
const Serial_IO_T *BSP_Get_Serial_IO_Interface_USB0();

/**
 ***************************************************************************************************
 * @brief   I2C Functions
 **************************************************************************************************/
I2C_Write BSP_Get_I2C_Write_SSD1306();
I2C_Read BSP_Get_I2C_Read_SSD1306();
I2C_Write BSP_Get_I2C_Write_Pressure();
I2C_Read BSP_Get_I2C_Read_Pressure();

/**
 ***************************************************************************************************
 * @brief   Tachometer Functions
 **************************************************************************************************/

// void BSP_Tach_Capture_Timer_Enable();
int32_t BSP_Get_Flow_Sensor_IRQN(void); // 'flow sensor' since code is copied from Purafy

/**
 ***************************************************************************************************
 * @brief   Pressure Sensor Functions
 **************************************************************************************************/
void BSP_Put_Pressure_Sensor_Into_Reset(bool reset);

/**
 ***************************************************************************************************
 * @brief   GPIO motor ECU Functions
 **************************************************************************************************/
bool BSP_Get_Neutral();
bool BSP_Get_Start();
uint8_t BSP_Get_Red();
uint8_t BSP_Get_Orange();
bool BSP_Get_Buzzer();
float BSP_ADC_Read_VBAT(void);

/**
 ***************************************************************************************************
 * @brief   Perform a reset of the microcontroller
 **************************************************************************************************/
__attribute__((noreturn)) void BSP_SystemReset(void);

/**
 ***************************************************************************************************
 * @brief   Read the microcontroller's RCC CSR Register.
 **************************************************************************************************/
uint32_t BSP_RCC_CSR_Read(void);

/**
 ***************************************************************************************************
 * @brief   Clear reset flags indicated by the RCC CSR Register.
 **************************************************************************************************/
void BSP_RCC_CSR_ClearResetFlags(void);

/**
 ***************************************************************************************************
 * @brief   Read the microcontroller's assigned backup RAM, a single uint32 value, with index.
 *          Backup RAM is some portion of RAM that survives typical processor reset conditions,
 *          for example, a software driven reset of the microcontroller.
 **************************************************************************************************/
bool BSP_Backup_RAM_Read(int index, uint32_t *output);

/**
 ***************************************************************************************************
 * @brief   Write a value to the microcontroller's assigned backup RAM.
 *          See BSP_Backup_RAM_Read(...) for further background.
 **************************************************************************************************/
void BSP_Backup_RAM_Write(int index, uint32_t value);

#endif // BSP_H_
