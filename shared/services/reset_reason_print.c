#include "reset_reason_print.h"
#include "reset.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
static void printHfsrCfsrAbfsrMmfar(
    char *buffer,
    uint16_t max_len,
    char *line_break,
    uint32_t arg1,
    uint32_t arg2,
    uint32_t arg3,
    uint32_t arg4);
void snprintf_reset_reason(char *buffer, uint16_t max_len, char *line_break)
{
    uint32_t arg1;
    uint32_t arg2;
    uint32_t arg3;
    uint32_t arg4;
    char argString[9];
    Reset_Reason_T reason = Reset_GetLastReasonDetails(&arg1, &arg2, &arg3, &arg4);
    switch (reason)
    {
        case RESET_REASON_PIN_RESET:
            snprintf(buffer, max_len, "Reset reason: Pin Reset");
            break;
        case RESET_REASON_ASSERT:
            snprintf(buffer, max_len, "Reset reason: Assert!");
            break;
        case RESET_REASON_Q_ASSERT: {
            Reset_ArgsToString(arg1, arg2, argString, sizeof(argString));
            snprintf(
                buffer,
                max_len,
                "Reset reason: Q_ASSERT!%smodule: %s, id / line: %lu(0x %lx)",
                line_break,
                argString,
                arg3,
                arg3);
            break;
        }
        case RESET_REASON_DEFAULT_POWER_ON:
            snprintf(buffer, max_len, "Reset reason: Default power On");
            break;
        case RESET_REASON_ERROR_NOT_INITIALIZED:
            snprintf(buffer, max_len, "Reset reason: Error, not initialized!");
            break;
        case RESET_REASON_HARD_FAULT: {
            int n_written = snprintf(buffer, max_len, "Reset reason: Hard Fault!%s", line_break);
            printHfsrCfsrAbfsrMmfar(
                buffer + n_written, max_len - n_written, line_break, arg1, arg2, arg3, arg4);
            break;
        }
        case RESET_REASON_MEM_MANAGE_FAULT: {
            int n_written = snprintf(
                buffer, max_len, "Reset reason: Mem Manage Fault!%s", line_break);
            printHfsrCfsrAbfsrMmfar(
                buffer + n_written, max_len - n_written, line_break, arg1, arg2, arg3, arg4);
            break;
        }
        case RESET_REASON_BUS_FAULT: {
            int n_written = snprintf(buffer, max_len, "Reset reason: Bus Fault!%s", line_break);
            printHfsrCfsrAbfsrMmfar(
                buffer + n_written, max_len - n_written, line_break, arg1, arg2, arg3, arg4);
            break;
        }
        case RESET_REASON_USAGE_FAULT: {
            int n_written = snprintf(buffer, max_len, "Reset reason: Usage Fault!%s", line_break);
            printHfsrCfsrAbfsrMmfar(
                buffer + n_written, max_len - n_written, line_break, arg1, arg2, arg3, arg4);
            break;
        }
        case RESET_REASON_UNKNOWN:
            snprintf(buffer, max_len, "Reset reason: Unknown!");
            break;
        default:
            snprintf(
                buffer,
                max_len,
                "Reset reason: Undefined default case (code likely needs updates)");
            break;
    }
}
static void printHfsrCfsrAbfsrMmfar(
    char *buffer,
    uint16_t max_len,
    char *line_break,
    uint32_t arg1,
    uint32_t arg2,
    uint32_t arg3,
    uint32_t arg4)
{
    snprintf(
        buffer,
        max_len,
        " HFSR:  0x%08lx%s"
        " CFSR:  0x%08lx%s"
        " SPARE: 0x%08lx%s"
        " MMFAR: 0x%08lx%s",
        arg1,
        line_break,
        arg2,
        line_break,
        arg3,
        line_break,
        arg4,
        line_break);
}