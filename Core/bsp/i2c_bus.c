#include "gpio_stm32.h"
#include "i2c_bus_stm32.h"
#include "qsafe.h"
#include <assert.h>
#include <stdbool.h>
#include <stddef.h>

Q_DEFINE_THIS_MODULE("i2c_bus")

static I2C_Bus_T *s_i2c_bus_table[] = {NULL, NULL, NULL, NULL, NULL, NULL};
static_assert(
    sizeof(s_i2c_bus_table) / sizeof(s_i2c_bus_table[0]) == I2C_BUS_MAX_SUPPORTED,
    "s_i2c_bus_table bad length");

void I2C_Bus_Init(I2C_Bus_T *p_I2C_bus, I2C_Bus_ID_T id)
{
    Q_ASSERT(id < I2C_BUS_MAX_SUPPORTED);
    s_i2c_bus_table[id] = p_I2C_bus;
    p_I2C_bus->id       = id;

    p_I2C_bus->active_complete_cb = NULL;
    p_I2C_bus->active_error_cb    = NULL;
    p_I2C_bus->active_cb_data     = NULL;
}

static void Generic_I2C_Complete_CB(I2C_HandleTypeDef *hi2c, bool is_error)
{
    for (unsigned i = 0; i < I2C_BUS_MAX_SUPPORTED; i++)
    {
        if (s_i2c_bus_table[i] != NULL)
        {
            if (hi2c == STM32_GetI2CHandle(s_i2c_bus_table[i]->id))
            {
                if (is_error)
                {
                    if (s_i2c_bus_table[i]->active_error_cb != NULL)
                    {
                        // If there is a valid error callback waiting for this spi bus, call it
                        s_i2c_bus_table[i]->active_error_cb(s_i2c_bus_table[i]->active_cb_data);
                    }
                }
                else
                {
                    if (s_i2c_bus_table[i]->active_complete_cb != NULL)
                    {
                        // If there is a valid complete callback waiting for this spi bus, call it
                        s_i2c_bus_table[i]->active_complete_cb(s_i2c_bus_table[i]->active_cb_data);
                    }
                }

                break;
            }
        }
    }
}

// Externally available callback function, called by the STM32 HAL SI2CPI Interrupt handler
//   once a I2C Transfer is complete
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    Generic_I2C_Complete_CB(hi2c, false);
}

// Externally available callback function, called by the STM32 HAL I2C Interrupt handler
//   once a I2C Write is complete
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    Generic_I2C_Complete_CB(hi2c, false);
}

// Externally available callback function, called by the STM32 HAL I2C Interrupt handler
//   once a I2C Write is complete
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    Generic_I2C_Complete_CB(hi2c, false);
}

// Externally available callback function, called by the STM32 HAL I2C Interrupt handler
//   if there is an error during I2C transmission
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    Generic_I2C_Complete_CB(hi2c, true);
}

// Externally available callback function, called by the STM32 HAL I2C Interrupt handler
//   if the I2C transaction is aborted
void HAL_I2C_AbortCpltCallback(I2C_HandleTypeDef *hi2c)
{
    Generic_I2C_Complete_CB(hi2c, true);
}

I2C_Return_T I2C_Bus_Read(
    I2C_Bus_ID_T bus_id,
    uint8_t address,
    uint8_t *rx_buffer,
    const uint16_t data_len,
    I2C_Complete_Callback complete_cb,
    I2C_Error_Callback error_cb,
    void *cb_data)
{
    Q_ASSERT(bus_id < I2C_BUS_MAX_SUPPORTED);
    Q_ASSERT(s_i2c_bus_table[bus_id] != NULL); // means you didn't call I2C_Bus_Init for this bus
    Q_ASSERT(address != 0);
    Q_ASSERT(rx_buffer != NULL);
    Q_ASSERT(data_len > 0);

    I2C_Bus_T *p_i2c_bus                  = s_i2c_bus_table[bus_id];
    I2C_HandleTypeDef *p_stm32_i2c_handle = STM32_GetI2CHandle(bus_id);

    HAL_I2C_StateTypeDef i2c_bus_state = HAL_I2C_GetState(p_stm32_i2c_handle);
    if (i2c_bus_state != HAL_I2C_STATE_READY)
    {
        if (i2c_bus_state == HAL_I2C_STATE_RESET)
        {
            return I2C_RTN_ERROR;
        }
        else
        {
            return I2C_RTN_BUSY;
        }
    }

    p_i2c_bus->active_complete_cb = complete_cb;
    p_i2c_bus->active_error_cb    = error_cb;
    p_i2c_bus->active_cb_data     = cb_data;

    uint16_t shifted_device_address = ((uint16_t) address) << 1;

    HAL_StatusTypeDef retval = HAL_I2C_Master_Receive_IT(
        p_stm32_i2c_handle, shifted_device_address, rx_buffer, data_len);

    return (retval == HAL_OK) ? I2C_RTN_SUCCESS
                              : ((retval == HAL_BUSY) ? I2C_RTN_BUSY : I2C_RTN_ERROR);
}

I2C_Return_T I2C_Bus_MemoryRead(
    I2C_Bus_ID_T bus_id,
    uint8_t address,
    uint16_t mem_address,
    uint8_t mem_address_size,
    uint8_t *rx_buffer,
    const uint16_t data_len,
    I2C_Complete_Callback complete_cb,
    I2C_Error_Callback error_cb,
    void *cb_data)
{
    Q_ASSERT(bus_id < I2C_BUS_MAX_SUPPORTED);
    Q_ASSERT(s_i2c_bus_table[bus_id] != NULL); // means you didn't call I2C_Bus_Init for this bus
    Q_ASSERT(address != 0);
    Q_ASSERT(mem_address_size == 1 || mem_address_size == 2);
    Q_ASSERT(rx_buffer != NULL);
    Q_ASSERT(data_len > 0);

    I2C_Bus_T *p_i2c_bus                  = s_i2c_bus_table[bus_id];
    I2C_HandleTypeDef *p_stm32_i2c_handle = STM32_GetI2CHandle(bus_id);

    HAL_I2C_StateTypeDef i2c_bus_state = HAL_I2C_GetState(p_stm32_i2c_handle);
    if (i2c_bus_state != HAL_I2C_STATE_READY)
    {
        if (i2c_bus_state == HAL_I2C_STATE_RESET)
        {
            return I2C_RTN_ERROR;
        }
        else
        {
            return I2C_RTN_BUSY;
        }
    }

    p_i2c_bus->active_complete_cb = complete_cb;
    p_i2c_bus->active_error_cb    = error_cb;
    p_i2c_bus->active_cb_data     = cb_data;

    uint16_t shifted_device_address = ((uint16_t) address) << 1;

    HAL_StatusTypeDef retval = HAL_I2C_Mem_Read_IT(
        p_stm32_i2c_handle,
        shifted_device_address,
        mem_address,
        mem_address_size,
        rx_buffer,
        data_len);

    return (retval == HAL_OK) ? I2C_RTN_SUCCESS
                              : ((retval == HAL_BUSY) ? I2C_RTN_BUSY : I2C_RTN_ERROR);
}

I2C_Return_T I2C_Bus_Write(
    I2C_Bus_ID_T bus_id,
    uint8_t address,
    uint8_t *tx_buffer,
    const uint16_t data_len,
    I2C_Complete_Callback complete_cb,
    I2C_Error_Callback error_cb,
    void *cb_data)
{
    Q_ASSERT(bus_id < I2C_BUS_MAX_SUPPORTED);
    Q_ASSERT(s_i2c_bus_table[bus_id] != NULL); // means you didn't call I2C_Bus_Init for this bus
    Q_ASSERT(address != 0);
    Q_ASSERT(tx_buffer != NULL);
    Q_ASSERT(data_len > 0);

    I2C_Bus_T *p_i2c_bus                  = s_i2c_bus_table[bus_id];
    I2C_HandleTypeDef *p_stm32_i2c_handle = STM32_GetI2CHandle(bus_id);

    HAL_I2C_StateTypeDef i2c_bus_state = HAL_I2C_GetState(p_stm32_i2c_handle);
    if (i2c_bus_state != HAL_I2C_STATE_READY)
    {
        if (i2c_bus_state == HAL_I2C_STATE_RESET)
        {
            return I2C_RTN_ERROR;
        }
        else
        {
            return I2C_RTN_BUSY;
        }
    }

    p_i2c_bus->active_complete_cb = complete_cb;
    p_i2c_bus->active_error_cb    = error_cb;
    p_i2c_bus->active_cb_data     = cb_data;

    uint16_t shifted_device_address = ((uint16_t) address) << 1;

    HAL_StatusTypeDef retval = HAL_I2C_Master_Transmit_IT(
        p_stm32_i2c_handle, shifted_device_address, tx_buffer, data_len);

    return (retval == HAL_OK) ? I2C_RTN_SUCCESS
                              : ((retval == HAL_BUSY) ? I2C_RTN_BUSY : I2C_RTN_ERROR);
}
