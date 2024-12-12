#ifndef SHARED_I2C_EVENTS_H_
#define SHARED_I2C_EVENTS_H_

#include "i2c_bus.h"
#include "qpc.h"

typedef struct
{
    QEvt super;

    uint8_t address;
    uint8_t *tx_buffer;
    uint16_t data_len;
    I2C_Complete_Callback complete_cb;
    I2C_Error_Callback error_cb;
    void *cb_data;
} SharedI2CWriteEvent_T;

typedef struct
{
    QEvt super;

    uint8_t address;
    uint8_t *rx_buffer;
    uint16_t data_len;
    I2C_Complete_Callback complete_cb;
    I2C_Error_Callback error_cb;
    void *cb_data;
} SharedI2CReadEvent_T;

typedef struct
{
    QEvt super;

    uint8_t address;
    uint16_t mem_address;
    uint8_t mem_address_size;
    uint8_t *rx_buffer;
    uint16_t data_len;
    I2C_Complete_Callback complete_cb;
    I2C_Error_Callback error_cb;
    void *cb_data;
} SharedI2CMemoryReadEvent_T;

#endif // SHARED_I2C_EVENTS_H_
