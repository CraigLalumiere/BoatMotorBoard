#include "bsp.h"
#include "stm32g4xx.h"
#include <assert.h>

// compile time ensure we have the number of registers we think we have
// because I'm just going to use pointer math in the Read/Write methods
static_assert(
    (offsetof(TAMP_TypeDef, BKP31R) - offsetof(TAMP_TypeDef, BKP0R)) == (31 * sizeof(uint32_t)),
    "");
static const int NUM_OF_RTC_BACKUP_REGISTERS = 32;
static __IO uint32_t *const mBackupRegisters = &TAMP->BKP0R;

/**
 ***************************************************************************************************
 * @brief   Read the microcontroller's assigned backup RAM, a single uint32 value, with index.
 *          Backup RAM is some portion of RAM that survives typical processor reset conditions,
 *          for example, a software driven reset of the microcontroller.
 **************************************************************************************************/
bool BSP_Backup_RAM_Read(int index, uint32_t *output)
{
    if (index >= NUM_OF_RTC_BACKUP_REGISTERS)
    {
        return false;
    }

    if (output == NULL)
    {
        return false;
    }

    *output = mBackupRegisters[index];

    return true;
}

void BSP_Backup_RAM_Write(int index, uint32_t value)
{
    assert(index < NUM_OF_RTC_BACKUP_REGISTERS);
    HAL_PWR_EnableBkUpAccess();
    mBackupRegisters[index] = value;
    HAL_PWR_DisableBkUpAccess();
}