extern "C" {
#include "interfaces/can_interface.h"
#include "fault_manager.h"
}

#include <cstring>

static CAN_Message_T s_last_can_msg;
static uint32_t s_can_write_count;
static int32_t s_can_write_retval;
static uint32_t s_can_bus_init_count;
static uint32_t s_milliseconds_tick;
static Fault_ID_T s_last_fault_id;
static uint32_t s_fault_count;

extern "C" void BSP_BoxToBoxMock_Reset(void)
{
    memset(&s_last_can_msg, 0, sizeof(s_last_can_msg));
    s_can_write_count    = 0;
    s_can_write_retval   = 0;
    s_can_bus_init_count = 0;
    s_milliseconds_tick  = 1234U;
    s_last_fault_id      = FAULT_ID_NONE;
    s_fault_count        = 0;
}

extern "C" void BSP_BoxToBoxMock_SetCanWriteRetval(int32_t retval)
{
    s_can_write_retval = retval;
}

extern "C" uint32_t BSP_BoxToBoxMock_GetCanWriteCount(void)
{
    return s_can_write_count;
}

extern "C" uint32_t BSP_BoxToBoxMock_GetCanBusInitCount(void)
{
    return s_can_bus_init_count;
}

extern "C" const CAN_Message_T *BSP_BoxToBoxMock_GetLastCanMsg(void)
{
    return &s_last_can_msg;
}

extern "C" uint32_t BSP_BoxToBoxMock_GetFaultCount(void)
{
    return s_fault_count;
}

extern "C" Fault_ID_T BSP_BoxToBoxMock_GetLastFaultId(void)
{
    return s_last_fault_id;
}

extern "C" uint32_t BSP_Get_Milliseconds_Tick(void)
{
    return s_milliseconds_tick;
}

extern "C" void BSP_CAN_Bus_Init(void)
{
    s_can_bus_init_count++;
}

extern "C" int32_t BSP_CAN_Write_Msg(const CAN_Message_T *msg)
{
    s_can_write_count++;
    memcpy(&s_last_can_msg, msg, sizeof(s_last_can_msg));
    return s_can_write_retval;
}

extern "C" void Fault_Manager_Generate_Fault(QActive *, Fault_ID_T id, const char *)
{
    s_fault_count++;
    s_last_fault_id = id;
}
