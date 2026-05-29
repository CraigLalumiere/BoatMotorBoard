#ifndef LOG_COM_H_
#define LOG_COM_H_

#include "interfaces/serial_interface.h"
#include "qpc.h"

extern QActive *const AO_LogCom;

void LogCom_ctor(const Serial_IO_T *const serial_io_interface);
int LogCom_Printf(const char *sFormat, ...);

#endif // LOG_COM_H_
