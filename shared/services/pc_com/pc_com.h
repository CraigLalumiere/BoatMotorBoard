#ifndef PC_COM_H_
#define PC_COM_H_

#include "interfaces/serial_interface.h"
#include "qpc.h"
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PC_COM_EVENT_MAX_MSG_LENGTH 64U
#define PC_COM_CLI_DATA_MAX_LENGTH  64U

extern QActive *const AO_PC_COM;

typedef struct
{
    QEvt super;
    uint32_t milliseconds;
    char msg[PC_COM_EVENT_MAX_MSG_LENGTH];
} PCCOMPrintEvent_T;

typedef struct
{
    QEvt super;
    char msg[PC_COM_CLI_DATA_MAX_LENGTH];
    uint8_t msg_size;
} PCCOMCliDataEvent_T;

void PC_COM_ctor(const Serial_IO_T *const serial_io_interface);
int PC_COM_Printf(const char *sFormat, ...);

#ifdef __cplusplus
}
#endif

#endif // PC_COM_H_
