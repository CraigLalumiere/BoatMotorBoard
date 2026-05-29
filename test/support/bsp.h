#ifndef BSP_H_
#define BSP_H_

#include "interfaces/can_interface.h"
#include "interfaces/serial_interface.h"
#include <stdbool.h>
#include <stdint.h>

#define BSP_TICKS_PER_SEC         1000U
#define MILLISECONDS_TO_TICKS(ms) ((ms) * ((BSP_TICKS_PER_SEC) / 1000))

#ifdef __cplusplus
extern "C" {
#endif

uint32_t BSP_Get_Milliseconds_Tick(void);
void BSP_CAN_Bus_Init(void);
int32_t BSP_CAN_Write_Msg(const CAN_Message_T *msg);

bool BSP_Get_Neutral(void);
bool BSP_Get_Start(void);
bool BSP_Get_Temp_Good(void);
bool BSP_Get_Pres_Good(void);
bool BSP_Get_Buzzer(void);
float BSP_ADC_Read_VBAT(void);

void BSP_Gauge_SetPressure_V(float volts);
void BSP_Gauge_SetTemperature_V(float volts);
void BSP_Gauge_SetOpAmpRef_V(float volts);
void BSP_RpmGauge_SetPFM_RPM(uint32_t rpm);
void BSP_Set_Backlight(bool on);
bool BSP_Get_Backlight(void);

void BSP_Put_Pressure_Sensor_Into_Reset(bool in_reset);

#ifdef __cplusplus
}
#endif

#endif // BSP_H_
