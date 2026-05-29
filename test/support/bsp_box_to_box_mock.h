#ifndef BSP_BOX_TO_BOX_MOCK_H_
#define BSP_BOX_TO_BOX_MOCK_H_

#include "interfaces/can_interface.h"
#include "fault_manager.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void BSP_BoxToBoxMock_Reset(void);
void BSP_BoxToBoxMock_SetCanWriteRetval(int32_t retval);
uint32_t BSP_BoxToBoxMock_GetCanWriteCount(void);
uint32_t BSP_BoxToBoxMock_GetCanBusInitCount(void);
const CAN_Message_T *BSP_BoxToBoxMock_GetLastCanMsg(void);
uint32_t BSP_BoxToBoxMock_GetFaultCount(void);
Fault_ID_T BSP_BoxToBoxMock_GetLastFaultId(void);

#ifdef __cplusplus
}
#endif

#endif // BSP_BOX_TO_BOX_MOCK_H_
