//============================================================================
// Product: Board Support Package example
// Last Updated for Version: 7.3.0
// Date of the Last Update:  2023-08-12
//
//                   Q u a n t u m  L e a P s
//                   ------------------------
//                   Modern Embedded Software
//
// Copyright (C) 2005 Quantum Leaps, LLC. All rights reserved.
//
// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-QL-commercial
//
// This software is dual-licensed under the terms of the open source GNU
// General Public License version 3 (or any later version), or alternatively,
// under the terms of one of the closed source Quantum Leaps commercial
// licenses.
//
// The terms of the open source GNU General Public License version 3
// can be found at: <www.gnu.org/licenses/gpl-3.0>
//
// The terms of the closed source Quantum Leaps commercial licenses
// can be found at: <www.state-machine.com/licensing>
//
// Redistributions in source code must retain this top-level comment block.
// Plagiarizing this software to sidestep the license obligations is illegal.
//
// Contact information:
// <www.state-machine.com/licensing>
// <info@state-machine.com>
//============================================================================
#ifndef BSP_H_
#define BSP_H_

#include "serial_io_interface.h"
#include "i2c_interface.h"
#include "stdint.h"
#include <stdbool.h>

#define BSP_TICKS_PER_SEC    100U

void BSP_Init(void);
void BSP_displayPaused(uint8_t paused);
void BSP_displayPhilStat(uint8_t n, char const *stat);
void BSP_terminate(int16_t result);

void BSP_randomSeed(uint32_t seed); // random seed
uint32_t BSP_random(void);          // pseudo-random generator

/**
 ***************************************************************************************************
 * @brief   Functions for blinky LED
 **************************************************************************************************/
void BSP_ledOn(void);
void BSP_ledOff(void);
void BSP_debug_gpio_on(void);
void BSP_debug_gpio_off(void);

/**
 ***************************************************************************************************
 * @brief   Retrieve Serial IO interface for the USB Interface serial comms channels
 **************************************************************************************************/
const Serial_IO_T *BSP_Get_Serial_IO_Interface_USB0();
const Serial_IO_T *BSP_Get_Serial_IO_Interface_USB1();


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

void BSP_Tach_Capture_Timer_Enable();


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
uint16_t BSP_ADC_Read_VBAT(void);


#endif // BSP_H_
