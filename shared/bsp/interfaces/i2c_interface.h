#ifndef I2C_INTERFACE_H_
#define I2C_INTERFACE_H_

#include "stdint.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    I2C_RTN_SUCCESS,
    I2C_RTN_BUSY,
    I2C_RTN_ERROR,
} I2C_Return_T;

typedef void (*I2C_Complete_Callback)(void *cb_data);
typedef void (*I2C_Error_Callback)(void *cb_data);

/**
 ***************************************************************************************************
 *
 * @brief   Non-blocking I2C write, sending the address then the number of bytes with repeated
 *          start conditions in between each byte.
 *
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
typedef I2C_Return_T (*I2C_Write)(
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
typedef I2C_Return_T (*I2C_Read)(
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
 * @param   address                 7-bit address, no shifting necessary
 * @param   mem_address             memory address to write after the device address
 * @param   mem_address_size        number of bytes to specify memory address (must be 1 or 2)
 * @param   *rx_buffer              data buffer for received data
 * @param   rx_n_bytes              number of bytes to read
 * @param   complete_cb             callback to call when operation is complete
 * @param   error_cb                callback to call when there is an error with the operation
 * @param   cb_data                 pointer that will be passed as a parameter to the callback
 * @retval  I2C_RTN_SUCCESS         Success
 * @retval  I2C_RTN_BUSY            I2C operation currently in progress
 * @retval  I2C_RTN_INVALID_PARAM   a parameter is not valid
 *
 **************************************************************************************************/
typedef I2C_Return_T (*I2C_MemoryRead)(
    uint8_t address,
    uint16_t mem_address,
    uint8_t mem_address_size,
    uint8_t *rx_buffer,
    const uint16_t rx_n_bytes,
    I2C_Complete_Callback complete_cb,
    I2C_Error_Callback error_cb,
    void *cb_data);

#ifdef __cplusplus
}
#endif

#endif // I2C_INTERFACE_H_
