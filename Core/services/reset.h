#ifndef RESET_H
#define RESET_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
#define _Noreturn [[noreturn]] // for C++ world, during unit testing.
extern "C" {
#endif

/************************************************************************************************
 * @brief   Initialize the reset module. Should be called only once and as early as
 *          possible in main().
 *
 * @note    Internally this will copy the assigned reset persistent RAM to an internal cache.
 *************************************************************************************************/
void Reset_Init();

/************************************************************************************************
 * @brief   The enumerated list of possible reasons which may cause a software driven or fault
 *          driven reset of the microcontroller.
 *************************************************************************************************/
typedef enum
{
    RESET_REASON_DEFAULT_POWER_ON = 0,
    RESET_REASON_ERROR_NOT_INITIALIZED,
    RESET_REASON_PIN_RESET,
    RESET_REASON_ASSERT,
    RESET_REASON_Q_ASSERT,
    RESET_REASON_HARD_FAULT,
    RESET_REASON_MEM_MANAGE_FAULT,
    RESET_REASON_BUS_FAULT,
    RESET_REASON_USAGE_FAULT,
    RESET_REASON_UNKNOWN // keep last
} Reset_Reason_T;

/************************************************************************************************
 * @brief   The enumerated list of indexes into contiguous uint32 Backup RAM or registers.
 *          i.e. RAM that remains persistent across microcontroller resets.
 *
 *          Publicly declared for unit testing convenience, although might eventually be needed
 *          to coordinate with a bootloader or other external entity.
 *************************************************************************************************/
typedef enum
{
    RESET_BACKUP_RAM_REASON_INDEX = 0,
    RESET_BACKUP_RAM_ARG_1_INDEX  = 1, // first 4 chars if string provided
    RESET_BACKUP_RAM_ARG_2_INDEX  = 2, // next 4 chars if string provided
    RESET_BACKUP_RAM_ARG_3_INDEX  = 3, // arbitrary based on reason
    RESET_BACKUP_RAM_ARG_4_INDEX  = 4, // arbitrary based on reason
    RESET_BACKUP_RAM_NUM_OF_ELEMENTS
} Reset_Backup_Ram_Index_T;

/************************************************************************************************
 * @brief   Get the last reset reason (i.e. why did the CPU restart)
 *************************************************************************************************/
Reset_Reason_T Reset_GetLastReason();

/************************************************************************************************
 * @brief   Reset_GetLastReasonDetails, get the four ids associated with the last reset.
 *          if not interested in a particular argument, set it to NULL.
 *************************************************************************************************/
Reset_Reason_T Reset_GetLastReasonDetails(
    uint32_t *id1, uint32_t *id2, uint32_t *id3, uint32_t *id4);

/************************************************************************************************
 * @brief   Perform a system reset, recording the reason and four arbitrary values with the
 *          reset. The meaning of the four values recorded with the reason are specific to
 *          each reason.
 *************************************************************************************************/
_Noreturn void Reset_DoResetWithReason(
    Reset_Reason_T reason, uint32_t id1, uint32_t id2, uint32_t id3, uint32_t id4);

/************************************************************************************************
 * @brief   Reset_DoResetWithReason, where id1 is a C string. will take the first eight bytes
 *          of the C string OR will search for the first eight bytes of a filename if a full
 *          directory path is found. The string bytes are stored in id1 and id2
 *************************************************************************************************/
_Noreturn void Reset_DoResetWithReasonWithStr(
    Reset_Reason_T reason, const char *id1_2_string, uint32_t id3, uint32_t id4);

/************************************************************************************************
 * @brief   Reset_ArgsToString is a helper method which reconstructs arg1 and arg2 into
 *          a single string. Provided as a convenience to avoid code duplication.
 *************************************************************************************************/
void Reset_ArgsToString(uint32_t arg1, uint32_t arg2, char *outStr, size_t outStrBufferSize);

/************************************************************************************************
 * @brief   For unit testing, return this module's internal static state to compile time defaults.
 *************************************************************************************************/
void Reset_Destroy();

#ifdef __cplusplus
}
#endif

#endif // RESET_H
