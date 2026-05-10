#include "fram.h"
#include "fault_manager.h"
#include "posted_signals.h"
#include "private_signal_ranges.h"
#include "pubsub_signals.h"
#include <stdbool.h>
#include <string.h>

#ifdef Q_SPY
Q_DEFINE_THIS_MODULE("fram")
#endif

#define FRAM_BASE_ADDR 0x50U

typedef struct
{
    uint8_t addr;
    FRAM_File_T file;
} __attribute__((packed, aligned(1))) Transfer_Buffer_T;

enum FramSignals
{
    FRAM_I2C_COMPLETE_SIG = PRIVATE_SIGNAL_FRAM_START,
    FRAM_I2C_ERROR_SIG,
};

typedef struct
{
    QActive super;
    I2C_Write i2c_write_fn;
    I2C_MemoryRead i2c_memory_read_fn;
    int8_t latest_valid_page;
    FRAM_File_Footer_T file_footer[2];
    QActive *requester;
    Transfer_Buffer_T transfer_buffer;
    uint8_t transfer_page;
    bool read_in_progress;
} Fram;

static QState Fram_initial(Fram *const me, void const *const par);
static QState Fram_startup(Fram *const me, QEvt const *const e);
static QState Fram_standby(Fram *const me, QEvt const *const e);
static QState Fram_busy(Fram *const me, QEvt const *const e);
static QState Fram_busy_writing(Fram *const me, QEvt const *const e);
static QState Fram_busy_reading(Fram *const me, QEvt const *const e);
static QState Fram_error(Fram *const me, QEvt const *const e);

static bool FRAM_FooterIsValid(const FRAM_File_Footer_T *footer);
static int8_t FRAM_FindLatestValidPage(const FRAM_File_Footer_T footer[2]);
static void FRAM_PublishReady(Fram *const me);
static uint8_t FRAM_GetDeviceAddress(uint8_t page);
static void Fram_I2C_Complete_CB(void *cb_data);
static void Fram_I2C_Error_CB(void *cb_data);

static Fram Fram_inst;
QActive *const AO_Fram = &Fram_inst.super;

void Fram_ctor(I2C_Write i2c_write_fn, I2C_MemoryRead i2c_memory_read_fn)
{
    Fram *const me = &Fram_inst;

    me->i2c_write_fn       = i2c_write_fn;
    me->i2c_memory_read_fn = i2c_memory_read_fn;

    QActive_ctor(&me->super, Q_STATE_CAST(&Fram_initial));
}

static QState Fram_initial(Fram *const me, void const *const par)
{
    Q_UNUSED_PAR(par);

    me->latest_valid_page = -1;
    me->requester         = NULL;
    me->transfer_page     = 0U;
    me->read_in_progress  = false;

    memset(me->file_footer, 0, sizeof(me->file_footer));
    memset(&me->transfer_buffer, 0, sizeof(me->transfer_buffer));

    return Q_TRAN(&Fram_startup);
}

static QState Fram_startup(Fram *const me, QEvt const *const e)
{
    QState status;

    switch (e->sig)
    {
        case Q_ENTRY_SIG: {
            me->transfer_page    = 0U;
            me->read_in_progress = true;

            I2C_Return_T retval = me->i2c_memory_read_fn(
                FRAM_GetDeviceAddress(me->transfer_page),
                0U,
                1U,
                (uint8_t *) &me->transfer_buffer.file,
                sizeof(FRAM_File_T),
                Fram_I2C_Complete_CB,
                Fram_I2C_Error_CB,
                me);

            if (retval != I2C_RTN_SUCCESS)
            {
                static QEvt const event = QEVT_INITIALIZER(FRAM_I2C_ERROR_SIG);
                QACTIVE_POST(&me->super, &event, &me);
            }

            status = Q_HANDLED();
            break;
        }

        case FRAM_I2C_COMPLETE_SIG: {
            me->file_footer[me->transfer_page] = me->transfer_buffer.file.footer;

            if (me->transfer_page == 1U)
            {
                me->latest_valid_page = FRAM_FindLatestValidPage(me->file_footer);
                FRAM_PublishReady(me);
                status = Q_TRAN(&Fram_standby);
            }
            else
            {
                me->transfer_page = 1U;

                I2C_Return_T retval = me->i2c_memory_read_fn(
                    FRAM_GetDeviceAddress(me->transfer_page),
                    0U,
                    1U,
                    (uint8_t *) &me->transfer_buffer.file,
                    sizeof(FRAM_File_T),
                    Fram_I2C_Complete_CB,
                    Fram_I2C_Error_CB,
                    me);

                if (retval != I2C_RTN_SUCCESS)
                {
                    static QEvt const event = QEVT_INITIALIZER(FRAM_I2C_ERROR_SIG);
                    QACTIVE_POST(&me->super, &event, &me);
                }

                status = Q_HANDLED();
            }
            break;
        }

        case FRAM_I2C_ERROR_SIG: {
            me->latest_valid_page = -1;
            Fault_Manager_Generate_Fault(&me->super, FAULT_ID_FRAM_I2C, "");
            FRAM_PublishReady(me);
            status = Q_TRAN(&Fram_standby);
            break;
        }

        default: {
            status = Q_SUPER(&QHsm_top);
            break;
        }
    }

    return status;
}

static QState Fram_standby(Fram *const me, QEvt const *const e)
{
    QState status;

    switch (e->sig)
    {
        case Q_ENTRY_SIG: {
            status = Q_HANDLED();
            break;
        }

        case POSTED_FRAM_WRITE_REQ_SIG: {
            const FramWriteReqEvent_T *evt = Q_EVT_CAST(FramWriteReqEvent_T);

            me->requester = evt->requester;

            if (me->latest_valid_page == 0)
            {
                me->transfer_page = 1U;
                me->transfer_buffer.file.footer.seq = me->file_footer[0].seq + 1U;
            }
            else if (me->latest_valid_page == 1)
            {
                me->transfer_page = 0U;
                me->transfer_buffer.file.footer.seq = me->file_footer[1].seq + 1U;
            }
            else
            {
                me->transfer_page = 0U;
                me->transfer_buffer.file.footer.seq = 0U;
            }

            me->transfer_buffer.file.footer.seq_complement =
                (uint16_t) ~me->transfer_buffer.file.footer.seq;
            me->transfer_buffer.addr            = 0U;
            me->read_in_progress                = false;

            memcpy(me->transfer_buffer.file.data, evt->file.data, sizeof(evt->file.data));

            status = Q_TRAN(&Fram_busy_writing);
            break;
        }

        case POSTED_FRAM_READ_REQ_SIG: {
            const FramReadReqEvent_T *evt = Q_EVT_CAST(FramReadReqEvent_T);

            me->requester = evt->requester;

            if (me->latest_valid_page < 0)
            {
                FramReadRespEvent_T *resp_evt = Q_NEW(FramReadRespEvent_T, POSTED_FRAM_READ_RESP_SIG);
                memset(&resp_evt->file, 0, sizeof(resp_evt->file));
                resp_evt->read_status = FRAM_FILE_READ_FAIL;
                QACTIVE_POST(me->requester, &resp_evt->super, &me->super);
                status = Q_HANDLED();
            }
            else
            {
                me->transfer_page    = (uint8_t) me->latest_valid_page;
                me->read_in_progress = true;
                status               = Q_TRAN(&Fram_busy_reading);
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

static QState Fram_busy(Fram *const me, QEvt const *const e)
{
    QState status;

    switch (e->sig)
    {
        case Q_ENTRY_SIG: {
            status = Q_HANDLED();
            break;
        }

        case FRAM_I2C_ERROR_SIG: {
            Fault_Manager_Generate_Fault(&me->super, FAULT_ID_FRAM_I2C, "");
            status = Q_TRAN(&Fram_error);
            break;
        }

        default: {
            status = Q_SUPER(&QHsm_top);
            break;
        }
    }

    return status;
}

static QState Fram_busy_writing(Fram *const me, QEvt const *const e)
{
    QState status;

    switch (e->sig)
    {
        case Q_ENTRY_SIG: {
            I2C_Return_T retval = me->i2c_write_fn(
                FRAM_GetDeviceAddress(me->transfer_page),
                (uint8_t *) &me->transfer_buffer,
                sizeof(me->transfer_buffer),
                Fram_I2C_Complete_CB,
                Fram_I2C_Error_CB,
                me);

            if (retval != I2C_RTN_SUCCESS)
            {
                static QEvt const event = QEVT_INITIALIZER(FRAM_I2C_ERROR_SIG);
                QACTIVE_POST(&me->super, &event, &me);
            }

            status = Q_HANDLED();
            break;
        }

        case FRAM_I2C_COMPLETE_SIG: {
            me->file_footer[me->transfer_page] = me->transfer_buffer.file.footer;
            me->latest_valid_page              = (int8_t) me->transfer_page;

            if (me->requester != NULL)
            {
                static QEvt const event = QEVT_INITIALIZER(POSTED_FRAM_WRITE_COMPLETE_SIG);
                QACTIVE_POST(me->requester, &event, &me->super);
            }

            status = Q_TRAN(&Fram_standby);
            break;
        }

        default: {
            status = Q_SUPER(&Fram_busy);
            break;
        }
    }

    return status;
}

static QState Fram_busy_reading(Fram *const me, QEvt const *const e)
{
    QState status;

    switch (e->sig)
    {
        case Q_ENTRY_SIG: {
            I2C_Return_T retval = me->i2c_memory_read_fn(
                FRAM_GetDeviceAddress(me->transfer_page),
                0U,
                1U,
                (uint8_t *) &me->transfer_buffer.file,
                sizeof(FRAM_File_T),
                Fram_I2C_Complete_CB,
                Fram_I2C_Error_CB,
                me);

            if (retval != I2C_RTN_SUCCESS)
            {
                static QEvt const event = QEVT_INITIALIZER(FRAM_I2C_ERROR_SIG);
                QACTIVE_POST(&me->super, &event, &me);
            }

            status = Q_HANDLED();
            break;
        }

        case FRAM_I2C_COMPLETE_SIG: {
            FramReadRespEvent_T *resp_evt = Q_NEW(FramReadRespEvent_T, POSTED_FRAM_READ_RESP_SIG);
            resp_evt->read_status         = FRAM_FILE_READ_OK;
            memcpy(&resp_evt->file, &me->transfer_buffer.file, sizeof(resp_evt->file));

            if (me->requester != NULL)
            {
                QACTIVE_POST(me->requester, &resp_evt->super, &me->super);
            }

            status = Q_TRAN(&Fram_standby);
            break;
        }

        default: {
            status = Q_SUPER(&Fram_busy);
            break;
        }
    }

    return status;
}

static QState Fram_error(Fram *const me, QEvt const *const e)
{
    QState status;

    switch (e->sig)
    {
        case Q_ENTRY_SIG: {
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

static bool FRAM_FooterIsValid(const FRAM_File_Footer_T *footer)
{
    return (uint16_t) (footer->seq ^ footer->seq_complement) == 0xFFFFU;
}

static int8_t FRAM_FindLatestValidPage(const FRAM_File_Footer_T footer[2])
{
    const bool page_0_valid = FRAM_FooterIsValid(&footer[0]);
    const bool page_1_valid = FRAM_FooterIsValid(&footer[1]);

    if (page_0_valid && !page_1_valid)
    {
        return 0;
    }
    if (!page_0_valid && page_1_valid)
    {
        return 1;
    }
    if (!page_0_valid && !page_1_valid)
    {
        return -1;
    }

    return (uint16_t) (footer[0].seq - footer[1].seq) < 0x8000U ? 0 : 1;
}

static void FRAM_PublishReady(Fram *const me)
{
    static QEvt const ready_evt = QEVT_INITIALIZER(PUBSUB_FRAM_READY_SIG);
    QACTIVE_PUBLISH(&ready_evt, &me->super);
}

static uint8_t FRAM_GetDeviceAddress(uint8_t page)
{
    return (uint8_t) (FRAM_BASE_ADDR | (page & 0x01U));
}

static void Fram_I2C_Complete_CB(void *cb_data)
{
    static QEvt const event = QEVT_INITIALIZER(FRAM_I2C_COMPLETE_SIG);
    QActive *me = (QActive *) cb_data;
    QACTIVE_POST(me, &event, me);
}

static void Fram_I2C_Error_CB(void *cb_data)
{
    static QEvt const event = QEVT_INITIALIZER(FRAM_I2C_ERROR_SIG);
    QActive *me = (QActive *) cb_data;
    QACTIVE_POST(me, &event, me);
}
