#ifndef I2C_BUS_H_
#define I2C_BUS_H_

#include "i2c_interface.h"
#include "stdint.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    I2C_BUS_ID_1,
    I2C_BUS_ID_2,
    I2C_BUS_ID_3,
    I2C_BUS_ID_4,
    I2C_BUS_ID_5,
    I2C_BUS_ID_6,
    I2C_BUS_MAX_SUPPORTED,
} I2C_Bus_ID_T;

typedef struct
{
    I2C_Bus_ID_T id;

    // info on operation currently in progress
    I2C_Complete_Callback active_complete_cb;
    I2C_Error_Callback active_error_cb;
    void *active_cb_data;
} I2C_Bus_T;

/**
 ***************************************************************************************************
 *
 * @brief   Initializes the I2C_Bus_T data structure.  Must be called before any I2C operation
 *          functions.
 *
 *          This function does NOT initialize hardware pins or peripherals.  The application is
 *          responsible for configuring hardware prior to calling I2C_Bus_Write, I2C_Bus_Read, or
 *          I2C_Bus_MemoryRead.
 *
 * @param   p_I2C_bus               pointer to I2C_Bus_T data structure
 * @param   id                      id of bus
 *
 **************************************************************************************************/
void I2C_Bus_Init(I2C_Bus_T *p_I2C_bus, I2C_Bus_ID_T id);

/**
 ***************************************************************************************************
 *
 * @brief   Non-blocking I2C write, sending the address then the number of bytes with repeated
 *          start conditions in between each byte.
 *
 * @param   bus_id                  id of the I2C bus
 * @param   address                 7-bit address, no shifting necessary
 * @param   *tx_buffer              data to be transmitted
 * @param   data_len                number of bytes to write
 * @param   complete_cb             callback to call when operation is complete
 * @param   error_cb                callback to call when there is an error with the operation
 * @param   cb_data                 pointer that will be passed as a parameter to the callback
 * @retval  I2C_RTN_SUCCESS         Success
 * @retval  I2C_RTN_BUSY            I2C operation currently in progress
 * @retval  I2C_RTN_INVALID_PARAM   a parameter is not valid
 *
 **************************************************************************************************/
I2C_Return_T I2C_Bus_Write(
    I2C_Bus_ID_T bus_id,
    uint8_t address,
    uint8_t *tx_buffer,
    const uint16_t data_len,
    I2C_Complete_Callback complete_cb,
    I2C_Error_Callback error_cb,
    void *cb_data);

/**
 ***************************************************************************************************
 *
 * @brief   Non-blocking I2C read, sending the address then reading the specified number of bytes
 *          with repeated start conditions in between each byte.
 *
 * @param   bus_id                  id of the I2C bus
 * @param   address                 7-bit address, no shifting necessary
 * @param   *rx_buffer              data buffer for received data
 * @param   data_len                number of bytes to read
 * @param   complete_cb             callback to call when operation is complete
 * @param   error_cb                callback to call when there is an error with the operation
 * @param   cb_data                 pointer that will be passed as a parameter to the callback
 * @retval  I2C_RTN_SUCCESS         Success
 * @retval  I2C_RTN_BUSY            I2C operation currently in progress
 * @retval  I2C_RTN_INVALID_PARAM   a parameter is not valid
 *
 **************************************************************************************************/
I2C_Return_T I2C_Bus_Read(
    I2C_Bus_ID_T bus_id,
    uint8_t address,
    uint8_t *rx_buffer,
    const uint16_t data_len,
    I2C_Complete_Callback complete_cb,
    I2C_Error_Callback error_cb,
    void *cb_data);

/**
 ***************************************************************************************************
 *
 * @brief   Non-blocking I2C read a number of bytes starting from a memory address.
 *          This operation sends the device address, then the memory address,
 *          then reads the number of read bytes.
 *
 *          This handles a common usage pattern without requiring separate write then
 *          read operations.
 *
 * @param   bus_id                  id of the I2C bus
 * @param   address                 7-bit address, no shifting necessary
 * @param   mem_address             memory address to write after the device address
 * @param   mem_address_size        number of bytes to specify memory address (must be 1 or 2)
 * @param   *rx_buffer              data buffer for received data
 * @param   data_len                number of bytes to read
 * @param   complete_cb             callback to call when operation is complete
 * @param   error_cb                callback to call when there is an error with the operation
 * @param   cb_data                 pointer that will be passed as a parameter to the callback
 * @retval  I2C_RTN_SUCCESS         Success
 * @retval  I2C_RTN_BUSY            I2C operation currently in progress
 * @retval  I2C_RTN_INVALID_PARAM   a parameter is not valid
 *
 **************************************************************************************************/
I2C_Return_T I2C_Bus_MemoryRead(
    I2C_Bus_ID_T bus_id,
    uint8_t address,
    uint16_t mem_address,
    uint8_t mem_address_size,
    uint8_t *rx_buffer,
    const uint16_t data_len,
    I2C_Complete_Callback complete_cb,
    I2C_Error_Callback error_cb,
    void *cb_data);

#ifdef __cplusplus
}
#endif

#endif // I2C_BUS_H_
