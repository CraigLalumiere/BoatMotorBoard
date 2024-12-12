#ifndef SHARED_I2C_H_
#define SHARED_I2C_H_

#include "i2c_bus.h"
#include "qpc.h"

typedef struct
{
    QActive super;    // inherit QActive
    QTimeEvt timeEvt; // private time event generator
    QEQueue deferred_queue;
    I2C_Bus_ID_T bus_id;
    I2C_Complete_Callback active_complete_cb;
    I2C_Error_Callback active_error_cb;
    void *active_cb_data;
} SharedI2C_T;

void SharedI2C_ctor(
    SharedI2C_T *me, I2C_Bus_ID_T bus_id, QEvt const **deferred_queue_storage, uint16_t queue_len);

I2C_Return_T SharedI2C_Write(
    SharedI2C_T *me,
    uint8_t address,
    uint8_t *tx_buffer,
    const uint16_t data_len,
    I2C_Complete_Callback complete_cb,
    I2C_Error_Callback error_cb,
    void *cb_data);

I2C_Return_T SharedI2C_Read(
    SharedI2C_T *me,
    uint8_t address,
    uint8_t *rx_buffer,
    const uint16_t data_len,
    I2C_Complete_Callback complete_cb,
    I2C_Error_Callback error_cb,
    void *cb_data);

I2C_Return_T SharedI2C_MemoryRead(
    SharedI2C_T *me,
    uint8_t address,
    uint16_t mem_address,
    uint8_t mem_address_size,
    uint8_t *rx_buffer,
    const uint16_t rx_n_bytes,
    I2C_Complete_Callback complete_cb,
    I2C_Error_Callback error_cb,
    void *cb_data);

#endif // SHARED_I2C_H_
