#include "log_com.h"
#include "bsp.h"
#include "posted_signals.h"
#include "private_signal_ranges.h"
#include "pubsub_signals.h"
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>

enum LogComSignals
{
    LOG_COM_STATUS_TIMEOUT_SIG = PRIVATE_SIGNAL_LOG_COM_START,
};

enum
{
    LOG_COM_MOTOR_DATA_EVERY_N_SAMPLES = 10U,
    LOG_COM_STATUS_TIMEOUT_MS          = 1000U,
};

typedef struct
{
    QActive super;
    QTimeEvt status_timeout_evt;
    const Serial_IO_T *serial_io_interface;
    uint8_t motor_data_sample_count;
    bool received_motor_data_since_timeout;
} LogCom;

static QState LogCom_initial(LogCom *const me, void const *const par);
static QState LogCom_active(LogCom *const me, QEvt const *const e);
static void LogCom_WriteEvent(LogCom *const me, const PrintEvent_T *const print_event);
static void LogCom_WriteTimestampedText(LogCom *const me, uint32_t milliseconds, const char *msg);
static void LogCom_WriteLine(LogCom *const me, const char *msg);
static void LogCom_LogMotorData(LogCom *const me, const MotorDataEvent_T *const motor_data_event);

static LogCom LogCom_inst;
QActive *const AO_LogCom = &LogCom_inst.super;

void LogCom_ctor(const Serial_IO_T *const serial_io_interface)
{
    LogCom_inst.serial_io_interface = serial_io_interface;

    LogCom *const me = &LogCom_inst;
    QActive_ctor(&me->super, Q_STATE_CAST(&LogCom_initial));
    QTimeEvt_ctorX(&me->status_timeout_evt, &me->super, LOG_COM_STATUS_TIMEOUT_SIG, 0U);
}

int LogCom_Printf(const char *sFormat, ...)
{
    int r;
    va_list ParamList;
    PrintEvent_T *printEvent = Q_NEW(PrintEvent_T, POSTED_LOG_COM_PRINT_SIG);

    printEvent->milliseconds = BSP_Get_Milliseconds_Tick();

    va_start(ParamList, sFormat);
    r = vsnprintf(printEvent->msg, PRINT_EVENT_MAX_MSG_LENGTH, sFormat, ParamList);
    va_end(ParamList);

    printEvent->msg[PRINT_EVENT_MAX_MSG_LENGTH - 1] = 0;
    QACTIVE_POST(AO_LogCom, &printEvent->super, NULL);

    return r;
}

static QState LogCom_initial(LogCom *const me, void const *const par)
{
    Q_UNUSED_PAR(par);

    QActive_subscribe((QActive *) me, PUBSUB_MOTOR_DATA_SIG);
    QTimeEvt_armX(
        &me->status_timeout_evt,
        MILLISECONDS_TO_TICKS(LOG_COM_STATUS_TIMEOUT_MS),
        MILLISECONDS_TO_TICKS(LOG_COM_STATUS_TIMEOUT_MS));

    return Q_TRAN(&LogCom_active);
}

static QState LogCom_active(LogCom *const me, QEvt const *const e)
{
    QState status;

    switch (e->sig)
    {
        case Q_ENTRY_SIG: {
            LogCom_WriteLine(me, "log interface online");
            status = Q_HANDLED();
            break;
        }

        case POSTED_LOG_COM_PRINT_SIG: {
            const PrintEvent_T *printEvent = Q_EVT_CAST(PrintEvent_T);
            LogCom_WriteEvent(me, printEvent);
            status = Q_HANDLED();
            break;
        }

        case PUBSUB_MOTOR_DATA_SIG: {
            const MotorDataEvent_T *motor_data_event = Q_EVT_CAST(MotorDataEvent_T);

            me->received_motor_data_since_timeout = true;
            me->motor_data_sample_count++;

            if (me->motor_data_sample_count >= LOG_COM_MOTOR_DATA_EVERY_N_SAMPLES)
            {
                me->motor_data_sample_count = 0U;
                LogCom_LogMotorData(me, motor_data_event);
            }

            status = Q_HANDLED();
            break;
        }

        case LOG_COM_STATUS_TIMEOUT_SIG: {
            if (!me->received_motor_data_since_timeout)
            {
                LogCom_WriteLine(me, "warning: no PUBSUB_MOTOR_DATA_SIG received in 1000 ms");
            }

            me->received_motor_data_since_timeout = false;
            status                                = Q_HANDLED();
            break;
        }

        default: {
            status = Q_SUPER(&QHsm_top);
            break;
        }
    }

    return status;
}

static void LogCom_WriteEvent(LogCom *const me, const PrintEvent_T *const print_event)
{
    LogCom_WriteTimestampedText(me, print_event->milliseconds, print_event->msg);
}

static void LogCom_WriteTimestampedText(LogCom *const me, uint32_t milliseconds, const char *msg)
{
    char buffer[144];
    int line_length = snprintf(
        buffer, sizeof(buffer), "[%lu] %s\r\n", (unsigned long) milliseconds, msg);

    if (line_length <= 0)
    {
        return;
    }

    if ((size_t) line_length >= sizeof(buffer))
    {
        line_length = (int) sizeof(buffer) - 1;
    }

    me->serial_io_interface->tx_func((const uint8_t *) buffer, (uint16_t) line_length);
}

static void LogCom_WriteLine(LogCom *const me, const char *msg)
{
    PrintEvent_T print_event = {
        .milliseconds = BSP_Get_Milliseconds_Tick(),
    };

    snprintf(print_event.msg, sizeof(print_event.msg), "%s", msg);
    print_event.msg[PRINT_EVENT_MAX_MSG_LENGTH - 1] = 0;

    LogCom_WriteEvent(me, &print_event);
}

static void LogCom_LogMotorData(LogCom *const me, const MotorDataEvent_T *const motor_data_event)
{
    char motor_msg[120];
    int msg_length = snprintf(
        motor_msg,
        sizeof(motor_msg),
        "RPM:%5.0f VBat:%5.2fV Temp:%4.1fC EngMin:%lu\r\n"
        "Press:%4.1f Neutral:%u Start:%u TG:%u PG:%u Bz:%u",
        motor_data_event->tachometer,
        motor_data_event->vbat,
        motor_data_event->temperature,
        (unsigned long) motor_data_event->engine_minutes,
        motor_data_event->pressure,
        motor_data_event->neutral ? 1U : 0U,
        motor_data_event->start ? 1U : 0U,
        motor_data_event->temp_good ? 1U : 0U,
        motor_data_event->pres_good ? 1U : 0U,
        motor_data_event->buzzer ? 1U : 0U);

    if (msg_length <= 0)
    {
        return;
    }

    motor_msg[sizeof(motor_msg) - 1] = 0;
    LogCom_WriteTimestampedText(me, BSP_Get_Milliseconds_Tick(), motor_msg);
}
