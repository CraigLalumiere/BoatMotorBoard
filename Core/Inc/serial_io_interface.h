#ifndef SERIAL_IO_INTERFACE_H_
#define SERIAL_IO_INTERFACE_H_

#include "stdint.h"

typedef void (*Serial_IO_Data_Ready_Callback)(void *cb_data);

/**
 ***************************************************************************************************
 *
 * @brief   Non-blocking transmit of a block of data
 *
 * @param   *data_ptr               data to be transmitted
 * @param   data_len                length of data to be transmitted
 *
 * @retval  number of bytes transmitted
 *
 **************************************************************************************************/
typedef uint16_t (*Serial_IO_TransmitData)(const uint8_t *data_ptr, const uint16_t data_len);

/**
 ***************************************************************************************************
 *
 * @brief   Non-blocking data receive.
 *
 * @param   *p_data                 buffer to store received data
 * @param    max_data_len           length of buffer -- max number of bytes that can be received
 *
 * @retval  number of bytes copied into data_ptr
 *
 **************************************************************************************************/
typedef uint16_t (*Serial_IO_ReceiveData)(uint8_t *data_ptr, const uint16_t max_data_len);

/**
 ***************************************************************************************************
 *
 * @brief   Register a callback that will be called when new received data is ready.
 *          This callback must be thread-safe as it may be called from an ISR context.
 *
 * @param   *cb         Callback function
 * @param   *cb_data    arbritrary chunk of data to pass to the cb function.
 *                      Useful, for example, if this code is used by object-oriented code,
 *                      and the "this" (for C++) or "me" (for C) pointer can be passed
 *                      to the callback
 * @retval  none
 *
 **************************************************************************************************/
typedef void (*Serial_IO_RegisterDataReadyCB)(Serial_IO_Data_Ready_Callback cb, void *cb_data);

/**
 ***************************************************************************************************
 * @brief   Struct that packages function pointers for all Serial IO interface functions.
 **************************************************************************************************/
typedef struct
{
    Serial_IO_TransmitData tx_func;
    Serial_IO_ReceiveData rx_func;
    Serial_IO_RegisterDataReadyCB register_cb_func;
} Serial_IO_T;

#endif // SERIAL_IO_INTERFACE_H_
