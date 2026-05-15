#include "shared_i2c.h"
#include "bsp.h"
#include "shared_i2c_events.h"
#include <stddef.h>
#include <stdint.h>

#include "private_signal_ranges.h"

enum SharedI2CSignals
{
    SHARED_I2C_TIMEOUT = PRIVATE_SIGNAL_SHARED_I2C_START,
    SHARED_I2C_WRITE_REQUEST,
    SHARED_I2C_READ_REQUEST,
    SHARED_I2C_MEMORY_READ_REQUEST,
    SHARED_I2C_COMPLETE,
    SHARED_I2C_ERROR,
};

// state handler functions
static QState SharedI2C_initial(SharedI2C_T *const me, void const *const par);
static QState SharedI2C_idle(SharedI2C_T *const me, QEvt const *const e);
static QState SharedI2C_busy(SharedI2C_T *const me, QEvt const *const e);

static void SharedI2C_Complete_CB(void *cb_data);
static void SharedI2C_Error_CB(void *cb_data);

//............................................................................
void SharedI2C_ctor(
    SharedI2C_T *me, I2C_Bus_ID_T bus_id, QEvt const **deferred_queue_storage, uint16_t queue_len)
{
    QActive_ctor(&me->super, Q_STATE_CAST(&SharedI2C_initial));
    QTimeEvt_ctorX(&me->timeEvt, &me->super, SHARED_I2C_TIMEOUT, 0U);
    QEQueue_init(&(me->deferred_queue), deferred_queue_storage, queue_len);

    me->bus_id             = bus_id;
    me->active_complete_cb = NULL;
    me->active_error_cb    = NULL;
    me->active_cb_data     = NULL;
}

// HSM definition ----------------------------------------------------------
QState SharedI2C_initial(SharedI2C_T *const me, void const *const par)
{
    Q_UNUSED_PAR(par);
    return Q_TRAN(&SharedI2C_idle);
}

//............................................................................
QState SharedI2C_idle(SharedI2C_T *const me, QEvt const *const e)
{
    QState status;
    switch (e->sig)
    {
        case Q_ENTRY_SIG: {
            QActive_recall(&me->super, &me->deferred_queue);
            status = Q_HANDLED();
            break;
        }
        case SHARED_I2C_WRITE_REQUEST: {
            SharedI2CWriteEvent_T *write_event = (SharedI2CWriteEvent_T *) e;

            I2C_Return_T retval = I2C_Bus_Write(
                me->bus_id,
                write_event->address,
                write_event->tx_buffer,
                write_event->data_len,
                SharedI2C_Complete_CB,
                SharedI2C_Error_CB,
                (void *) me);

            if (retval != I2C_RTN_SUCCESS)
            {
                if (write_event->error_cb != NULL)
                {
                    // There was some error, so call the error callback of the requester if it
                    // exists
                    write_event->error_cb(write_event->cb_data);
                }
                status = Q_HANDLED();
            }
            else
            {
                me->active_complete_cb = write_event->complete_cb;
                me->active_error_cb    = write_event->error_cb;
                me->active_cb_data     = write_event->cb_data;

                status = Q_TRAN(&SharedI2C_busy);
            }
            break;
        }
        case SHARED_I2C_READ_REQUEST: {
            SharedI2CReadEvent_T *read_event = (SharedI2CReadEvent_T *) e;

            I2C_Return_T retval = I2C_Bus_Read(
                me->bus_id,
                read_event->address,
                read_event->rx_buffer,
                read_event->data_len,
                SharedI2C_Complete_CB,
                SharedI2C_Error_CB,
                (void *) me);
            if (retval != I2C_RTN_SUCCESS)
            {
                if (read_event->error_cb != NULL)
                {
                    // There was some error, so call the error callback of the requester if it
                    // exists
                    read_event->error_cb(read_event->cb_data);
                }
                status = Q_HANDLED();
            }
            else
            {
                me->active_complete_cb = read_event->complete_cb;
                me->active_error_cb    = read_event->error_cb;
                me->active_cb_data     = read_event->cb_data;

                status = Q_TRAN(&SharedI2C_busy);
            }
            break;
        }
        case SHARED_I2C_MEMORY_READ_REQUEST: {
            SharedI2CMemoryReadEvent_T *mem_read_event = (SharedI2CMemoryReadEvent_T *) e;

            I2C_Return_T retval = I2C_Bus_MemoryRead(
                me->bus_id,
                mem_read_event->address,
                mem_read_event->mem_address,
                mem_read_event->mem_address_size,
                mem_read_event->rx_buffer,
                mem_read_event->data_len,
                SharedI2C_Complete_CB,
                SharedI2C_Error_CB,
                (void *) me);
            if (retval != I2C_RTN_SUCCESS)
            {
                if (mem_read_event->error_cb != NULL)
                {
                    // There was some error, so call the error callback of the requester if it
                    // exists
                    mem_read_event->error_cb(mem_read_event->cb_data);
                }
                status = Q_HANDLED();
            }
            else
            {
                me->active_complete_cb = mem_read_event->complete_cb;
                me->active_error_cb    = mem_read_event->error_cb;
                me->active_cb_data     = mem_read_event->cb_data;

                status = Q_TRAN(&SharedI2C_busy);
            }
            break;
        }
        default: {
            status = Q_SUPER(&QHsm_top);
            break;
        }
    }
    return status;
}

QState SharedI2C_busy(SharedI2C_T *const me, QEvt const *const e)
{
    QState status;
    switch (e->sig)
    {
        case Q_ENTRY_SIG: {
            // TODO: kick off timeout timer
            status = Q_HANDLED();
            break;
        }
        case SHARED_I2C_TIMEOUT: {
            // TODO: handle timeout error
            status = Q_HANDLED();
            break;
        }
        case SHARED_I2C_COMPLETE: {
            if (me->active_complete_cb != NULL)
            {
                me->active_complete_cb(me->active_cb_data);
            }
            status = Q_TRAN(&SharedI2C_idle);
            break;
        }
        case SHARED_I2C_ERROR: {
            if (me->active_error_cb != NULL)
            {
                me->active_error_cb(me->active_cb_data);
            }
            status = Q_TRAN(&SharedI2C_idle);
            break;
        }
        case SHARED_I2C_WRITE_REQUEST:
        case SHARED_I2C_READ_REQUEST:
        case SHARED_I2C_MEMORY_READ_REQUEST: {
            // Since we are busy, put this request in the deferred queue.
            //    It will be handled later once back in the idle state
            QActive_defer(&me->super, &me->deferred_queue, e);
            status = Q_HANDLED();
            break;
        }
        default: {
            status = Q_SUPER(&QHsm_top);
            break;
        }
    }
    return status;
}

static void SharedI2C_Complete_CB(void *cb_data)
{
    static QEvt const SharedI2CCompleteEvent = QEVT_INITIALIZER(SHARED_I2C_COMPLETE);

    QActive *me = (QActive *) cb_data;
    QACTIVE_POST(me, &SharedI2CCompleteEvent, &me->super);
}

static void SharedI2C_Error_CB(void *cb_data)
{
    static QEvt const SharedI2CErrorEvent = QEVT_INITIALIZER(SHARED_I2C_ERROR);

    QActive *me = (QActive *) cb_data;
    QACTIVE_POST(me, &SharedI2CErrorEvent, &me->super);
}

I2C_Return_T SharedI2C_Write(
    SharedI2C_T *me,
    uint8_t address,
    uint8_t *tx_buffer,
    const uint16_t data_len,
    I2C_Complete_Callback complete_cb,
    I2C_Error_Callback error_cb,
    void *cb_data)
{
    SharedI2CWriteEvent_T *p_i2c_write_evt = Q_NEW(SharedI2CWriteEvent_T, SHARED_I2C_WRITE_REQUEST);

    p_i2c_write_evt->address     = address;
    p_i2c_write_evt->tx_buffer   = tx_buffer;
    p_i2c_write_evt->data_len    = data_len;
    p_i2c_write_evt->complete_cb = complete_cb;
    p_i2c_write_evt->error_cb    = error_cb;
    p_i2c_write_evt->cb_data     = cb_data;

    QACTIVE_POST(&(me->super), &(p_i2c_write_evt->super), &me->super);

    return I2C_RTN_SUCCESS;
}

I2C_Return_T SharedI2C_Read(
    SharedI2C_T *me,
    uint8_t address,
    uint8_t *rx_buffer,
    const uint16_t data_len,
    I2C_Complete_Callback complete_cb,
    I2C_Error_Callback error_cb,
    void *cb_data)
{
    SharedI2CReadEvent_T *p_i2c_read_evt = Q_NEW(SharedI2CReadEvent_T, SHARED_I2C_READ_REQUEST);

    p_i2c_read_evt->address     = address;
    p_i2c_read_evt->rx_buffer   = rx_buffer;
    p_i2c_read_evt->data_len    = data_len;
    p_i2c_read_evt->complete_cb = complete_cb;
    p_i2c_read_evt->error_cb    = error_cb;
    p_i2c_read_evt->cb_data     = cb_data;

    QACTIVE_POST(&(me->super), &(p_i2c_read_evt->super), &me->super);
    return I2C_RTN_SUCCESS;
}

I2C_Return_T SharedI2C_MemoryRead(
    SharedI2C_T *me,
    uint8_t address,
    uint16_t mem_address,
    uint8_t mem_address_size,
    uint8_t *rx_buffer,
    const uint16_t data_len,
    I2C_Complete_Callback complete_cb,
    I2C_Error_Callback error_cb,
    void *cb_data)
{
    SharedI2CMemoryReadEvent_T *p_i2c_memread_evt = Q_NEW(
        SharedI2CMemoryReadEvent_T, SHARED_I2C_MEMORY_READ_REQUEST);

    p_i2c_memread_evt->address          = address;
    p_i2c_memread_evt->mem_address      = mem_address;
    p_i2c_memread_evt->mem_address_size = mem_address_size;
    p_i2c_memread_evt->rx_buffer        = rx_buffer;
    p_i2c_memread_evt->data_len         = data_len;
    p_i2c_memread_evt->complete_cb      = complete_cb;
    p_i2c_memread_evt->error_cb         = error_cb;
    p_i2c_memread_evt->cb_data          = cb_data;

    QACTIVE_POST(&(me->super), &(p_i2c_memread_evt->super), &me->super);
    return I2C_RTN_SUCCESS;
}
