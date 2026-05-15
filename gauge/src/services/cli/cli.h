#ifndef APP_CLI_H
#define APP_CLI_H

#include "interfaces/serial_interface.h"
#include "qpc.h"

void AppCLI_ctor(const Serial_IO_T *const serial_io_interface);

extern QActive *const AO_AppCLI; // opaque pointer

/**
 * AppCLI_PostPrintf:
 * Perform a printf on an allocated event
 * which is posted to the AppCLI AO for actual printing
 * to the CLI.
 *
 * Strings will be cut off if they exceed the length
 * of the event used to post the printf request.
 */
int AppCLI_PostPrintf(const char *sFormat, ...);

#endif // APP_CLI_H