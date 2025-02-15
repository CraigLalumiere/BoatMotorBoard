#ifndef RESET_REASON_PRINT_H_
#define RESET_REASON_PRINT_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
#define _Noreturn [[noreturn]] // for C++ world, during unit testing.
extern "C" {
#endif

/************************************************************************************************
 * @brief   Prints a message about the last reset reason. Useful for CLI and GUI.
 *************************************************************************************************/
void snprintf_reset_reason(char *buffer, uint16_t max_len, char *line_break);

#ifdef __cplusplus
}
#endif

#endif // RESET_REASON_PRINT_H_
