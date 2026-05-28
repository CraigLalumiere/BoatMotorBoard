#ifndef BSP_H_
#define BSP_H_

#include "interfaces/can_interface.h"
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
void BSP_LED_Toggle(void);
// void BSP_debug_gpio_on(void);
// void BSP_debug_gpio_off(void);
// void BSP_debug_gpio_toggle(void);

/**************************************************************************************************\
* Gauge / Actuator outputs
\**************************************************************************************************/

/**
 * @brief Initialize gauge output peripherals:
 *        - DAC1 CH1 (pressure)
 *        - DAC1 CH2 (temperature)
 *        - DAC3 CH1 + OPAMP1 follower (offset reference, set once)
 *        - TIM8 CH1 PWM (PFM for RPM gauge)
 *
 * @param offset_volts  DAC3/OPAMP1 offset reference voltage (set once, never changed)
 */
void BSP_Gauges_Init(float offset_volts);

/**
 * @brief Set pressure gauge drive voltage (DAC1 OUT1).
 */
void BSP_Gauge_SetPressure_V(float volts);

/**
 * @brief Set temperature gauge drive voltage (DAC1 OUT2).
 */
void BSP_Gauge_SetTemperature_V(float volts);

/**
 * @brief Set op amp + reference voltage (DAC3 OUT1).
 */
void BSP_Gauge_SetOpAmpRef_V(float volts);

/**
 * @brief Set RPM gauge PFM frequency on TIM8_CH1.
 *        freq_hz == 0 stops pulses (output low via 0% duty).
 */
void BSP_RpmGauge_SetPFM_RPM(uint32_t RPM);

/**
 * @brief Read digital input from backlight switch
 */
bool BSP_Get_Backlight(void);

/**
 * @brief Enable/disable backlight on gauge cluster
 */
void BSP_Set_Backlight(bool x);

/**
 ***************************************************************************************************
 * @brief   Retrieve Serial IO interface for the USB Interface serial comms channels
 **************************************************************************************************/
const Serial_IO_T *BSP_Get_Serial_IO_Interface_USB0();
const Serial_IO_T *BSP_Get_Serial_IO_Interface_USB1();
I2C_Write BSP_Get_I2C_Write_FRAM(void);
I2C_MemoryRead BSP_Get_I2C_Memory_Read_FRAM(void);

/**
 ***************************************************************************************************
 * @brief   Init the CAN bus
 **************************************************************************************************/
void BSP_CAN_Bus_Init(void);

/**
 ***************************************************************************************************
 * @brief   Write CAN Message with Standard ID (range of 0 to 0x7FF)
 **************************************************************************************************/
int32_t BSP_CAN_Write_Msg(const CAN_Message_T *msg);

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
