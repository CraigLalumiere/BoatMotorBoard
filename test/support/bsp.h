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

#ifdef __cplusplus
}
#endif

#endif // BSP_H_
