#include "reset.h"
#include "bsp.h"
#include "qsafe.h"
#include <assert.h>
#include <string.h>

Q_DEFINE_THIS_MODULE("reset")

static bool m_InitCalled                                           = false;
static uint32_t m_RccCsr                                           = 0;
static uint32_t m_BackupRamCache[RESET_BACKUP_RAM_NUM_OF_ELEMENTS] = {0};
static_assert(
    RESET_BACKUP_RAM_NUM_OF_ELEMENTS <= 32,
    "We are using STM RTC BKP Registers. Only 32 are available");

#ifndef RCC_CSR_LSION_Pos

// copy in RCC CSR defines to avoid HAL dependencies, but if they are already
// pulled in via some other header, the above guard should prevent duplicates

#define RCC_CSR_RMVF_Pos (24U)
#define RCC_CSR_RMVF_Msk (0x1UL << RCC_CSR_RMVF_Pos) /*!< 0x01000000 */
#define RCC_CSR_RMVF     RCC_CSR_RMVF_Msk

// BORRSTF: BOR reset flag
//          Cleared by software by writing the RMVF bit.
//          This bit is set by hardware when a POR/PDR or BOR reset occurs.
//          POR == Power On Reset
//          BOR == Brownout Reset
//          PDR == Power Down Reset
//  (high by default)
#define RCC_CSR_BORRSTF_Pos (25U)
#define RCC_CSR_BORRSTF_Msk (0x1UL << RCC_CSR_BORRSTF_Pos) /*!< 0x02000000 */
#define RCC_CSR_BORRSTF     RCC_CSR_BORRSTF_Msk

// PIN reset flag (high by default)
#define RCC_CSR_PINRSTF_Pos (26U)
#define RCC_CSR_PINRSTF_Msk (0x1UL << RCC_CSR_PINRSTF_Pos) /*!< 0x04000000 */
#define RCC_CSR_PINRSTF     RCC_CSR_PINRSTF_Msk

// POR/PDR reset flag (high by default)
#define RCC_CSR_PORRSTF_Pos (27U)
#define RCC_CSR_PORRSTF_Msk (0x1UL << RCC_CSR_PORRSTF_Pos) /*!< 0x08000000 */
#define RCC_CSR_PORRSTF     RCC_CSR_PORRSTF_Msk

// software reset flag
#define RCC_CSR_SFTRSTF_Pos (28U)
#define RCC_CSR_SFTRSTF_Msk (0x1UL << RCC_CSR_SFTRSTF_Pos) /*!< 0x10000000 */
#define RCC_CSR_SFTRSTF     RCC_CSR_SFTRSTF_Msk

// independent watchdog reset flag
#define RCC_CSR_IWDGRSTF_Pos (29U)
#define RCC_CSR_IWDGRSTF_Msk (0x1UL << RCC_CSR_IWDGRSTF_Pos) /*!< 0x20000000 */
#define RCC_CSR_IWDGRSTF     RCC_CSR_IWDGRSTF_Msk

// Window watchdog reset flag
#define RCC_CSR_WWDGRSTF_Pos (30U)
#define RCC_CSR_WWDGRSTF_Msk (0x1UL << RCC_CSR_WWDGRSTF_Pos) /*!< 0x40000000 */
#define RCC_CSR_WWDGRSTF     RCC_CSR_WWDGRSTF_Msk

// Low-power reset flag
#define RCC_CSR_LPWRRSTF_Pos (31U)
#define RCC_CSR_LPWRRSTF_Msk (0x1UL << RCC_CSR_LPWRRSTF_Pos) /*!< 0x80000000 */
#define RCC_CSR_LPWRRSTF     RCC_CSR_LPWRRSTF_Msk

#endif

// BORRSTF, PINRSTF, and PORRSTF will be high by default
static const uint32_t DEFAULT_RCC_CSR = RCC_CSR_BORRSTF | RCC_CSR_PINRSTF | RCC_CSR_PORRSTF;

void Reset_Init()
{
    m_InitCalled = true;

    // read RCC_CSR and cache it
    m_RccCsr = BSP_RCC_CSR_Read();
    BSP_RCC_CSR_ClearResetFlags();

    // read each persistent ram element
    bool ok = false;
    for (int i = 0; i < RESET_BACKUP_RAM_NUM_OF_ELEMENTS; ++i)
    {
        ok = BSP_Backup_RAM_Read(i, &m_BackupRamCache[i]);
        Q_ASSERT(ok);
    }

    BSP_Backup_RAM_Write(RESET_BACKUP_RAM_REASON_INDEX, RESET_REASON_UNKNOWN);
}

void Reset_Destroy()
{
    m_InitCalled = false;
    m_RccCsr     = 0;
    memset(m_BackupRamCache, 0, sizeof(m_BackupRamCache));
}

Reset_Reason_T Reset_GetLastReason()
{
    if (!m_InitCalled)
    {
        return RESET_REASON_ERROR_NOT_INITIALIZED;
    }

    if ((m_RccCsr == DEFAULT_RCC_CSR) &&
        (m_BackupRamCache[RESET_BACKUP_RAM_REASON_INDEX] == RESET_REASON_DEFAULT_POWER_ON))
    {
        return RESET_REASON_DEFAULT_POWER_ON;
    }

    // give priority to the software reset bit.
    // the pin reset bit appears to be high at times, even when cleared
    if ((m_RccCsr & RCC_CSR_SFTRSTF) > 0)
    {
        // software reset.
        return m_BackupRamCache[RESET_BACKUP_RAM_REASON_INDEX];
    }

    if ((m_RccCsr & RCC_CSR_PINRSTF) > 0)
    {
        return RESET_REASON_PIN_RESET;
    }

    return RESET_REASON_UNKNOWN;
}

Reset_Reason_T Reset_GetLastReasonDetails(
    uint32_t *id1, uint32_t *id2, uint32_t *id3, uint32_t *id4)
{
    if (!m_InitCalled)
    {
        return RESET_REASON_ERROR_NOT_INITIALIZED;
    }

    Reset_Reason_T reason = Reset_GetLastReason();

    if (id1 != NULL)
    {
        *id1 = m_BackupRamCache[RESET_BACKUP_RAM_ARG_1_INDEX];
    }
    if (id2 != NULL)
    {
        *id2 = m_BackupRamCache[RESET_BACKUP_RAM_ARG_2_INDEX];
    }
    if (id3 != NULL)
    {
        *id3 = m_BackupRamCache[RESET_BACKUP_RAM_ARG_3_INDEX];
    }
    if (id4 != NULL)
    {
        *id4 = m_BackupRamCache[RESET_BACKUP_RAM_ARG_4_INDEX];
    }
    return reason;
}

_Noreturn void Reset_DoResetWithReason(
    Reset_Reason_T reason, uint32_t id1, uint32_t id2, uint32_t id3, uint32_t id4)
{
    BSP_Backup_RAM_Write(RESET_BACKUP_RAM_REASON_INDEX, reason);
    BSP_Backup_RAM_Write(RESET_BACKUP_RAM_ARG_1_INDEX, id1);
    BSP_Backup_RAM_Write(RESET_BACKUP_RAM_ARG_2_INDEX, id2);
    BSP_Backup_RAM_Write(RESET_BACKUP_RAM_ARG_3_INDEX, id3);
    BSP_Backup_RAM_Write(RESET_BACKUP_RAM_ARG_4_INDEX, id4);
    BSP_SystemReset();
}

_Noreturn void Reset_DoResetWithReasonWithStr(
    Reset_Reason_T reason, const char *id1_2_string, uint32_t id3, uint32_t id4)
{
    const char dirChar = '/';

    uint32_t id1 = 0;
    uint32_t id2 = 0;

    const char *usefulString = id1_2_string;
    size_t usefulLength      = 8;
    size_t strLen            = strnlen(id1_2_string, 512);

    if (strchr(id1_2_string, dirChar) == NULL)
    {
        if (strLen < usefulLength)
        {
            usefulLength = strLen;
        }
    }
    else
    {
        // search from the end for the last directory delimiter in the string
        for (size_t i = strLen - 1; i > 0; --i)
        {
            if (id1_2_string[i] == dirChar)
            {
                usefulString = id1_2_string + i + 1;
                break;
            }
        }
    }

    size_t id1Length = usefulLength;
    size_t id2Length = 0;
    if (usefulLength > 4)
    {
        id1Length = 4;
        id2Length = usefulLength - id1Length;
    }

    for (size_t i = 0; i < id1Length; ++i)
    {
        id1 |= (usefulString[i] << (8 * (3 - i)));
    }
    for (size_t i = 0; i < id2Length; ++i)
    {
        id2 |= (usefulString[i + id1Length] << (8 * (3 - i)));
    }

    Reset_DoResetWithReason(reason, id1, id2, id3, id4);
}

void Reset_ArgsToString(uint32_t arg1, uint32_t arg2, char *outStr, size_t outStrBufferSize)
{
    Q_ASSERT(
        outStrBufferSize >= sizeof(arg1) + sizeof(arg2) + 1); // +1 for guaranteed null terminator

    outStr[0] = (char) ((arg1 & 0xFF000000) >> 24);
    outStr[1] = (char) ((arg1 & 0x00FF0000) >> 16);
    outStr[2] = (char) ((arg1 & 0x0000FF00) >> 8);
    outStr[3] = (char) ((arg1 & 0x000000FF) >> 0);

    outStr[4] = (char) ((arg2 & 0xFF000000) >> 24);
    outStr[5] = (char) ((arg2 & 0x00FF0000) >> 16);
    outStr[6] = (char) ((arg2 & 0x0000FF00) >> 8);
    outStr[7] = (char) ((arg2 & 0x000000FF) >> 0);

    outStr[8] = 0;
}
