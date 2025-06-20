#include "data_manager.h"
#include "bsp.h"
#include "flowsensor.h"
#include "private_signal_ranges.h"
#include "pubsub_signals.h"
#include "stm32g4xx_hal.h"
#include <stdio.h>
#include <string.h>

#ifdef Q_SPY
Q_DEFINE_THIS_MODULE("Data Manager")
#endif // def Q_SPY

/**************************************************************************************************\
* Private macros
\**************************************************************************************************/

#define TACH_LAMBDA 0.9
#define VBAT_LAMBDA 0.99

/**************************************************************************************************\
* Private type definitions
\**************************************************************************************************/

enum PressureSignals
{
    WAIT_TIMEOUT_SIG = PRIVATE_SIGNAL_DATA_MANAGER_START,
};

typedef struct
{
    QActive super; // inherit QActive
    QTimeEvt timer_evt;

    int16_t pressure;
    int16_t temperature;
    float vbat_volts;
    float tachometer;

    UART_HandleTypeDef *p_huart;
} DataManager;

/**************************************************************************************************\
* Private memory declarations
\**************************************************************************************************/
static DataManager data_manager_inst;
QActive *const AO_Data_Manager = &data_manager_inst.super;

/**************************************************************************************************\
* Private prototypes
\**************************************************************************************************/

// state handler functions
static QState initial(DataManager *const me, void const *const par);
static QState top(DataManager *const me, QEvt const *const e);
static QState running(DataManager *const me, QEvt const *const e);
static QState waitingForUart(DataManager *const me, QEvt const *const e);

/**************************************************************************************************\
* Public functions
\**************************************************************************************************/

/**
 ***************************************************************************************************
 * @brief   Constructor
 **************************************************************************************************/
void Data_Manager_ctor(UART_HandleTypeDef *p_huart)
{
    DataManager *const me = &data_manager_inst;

    me->p_huart = p_huart;

    // BSP_Tach_Capture_Timer_Enable();

    QActive_ctor(&me->super, Q_STATE_CAST(&initial));
    QTimeEvt_ctorX(&me->timer_evt, &me->super, WAIT_TIMEOUT_SIG, 0U);
}

/**************************************************************************************************\
* Private functions
\**************************************************************************************************/

/**
 ***************************************************************************************************
 * @brief   HSM
 **************************************************************************************************/
static QState initial(DataManager *const me, void const *const par)
{
    Q_UNUSED_PAR(par);

    QActive_subscribe((QActive *) me, PUBSUB_PRESSURE_SIG);
    QActive_subscribe((QActive *) me, PUBSUB_TEMPERATURE_SIG);
    QActive_subscribe((QActive *) me, PUBSUB_UART_COMPLETE_SIG);

    // BSP_Tach_Capture_Timer_Enable();

    // Start update loop of 100hz
    QTimeEvt_armX(&me->timer_evt, BSP_TICKS_PER_SEC / 100, BSP_TICKS_PER_SEC / 100);

    return Q_TRAN(&running);
}

// top state that handles receiving data from various sources
static QState top(DataManager *const me, QEvt const *const e)
{
    QState status;

    switch (e->sig)
    {
        case Q_ENTRY_SIG: {
            status = Q_HANDLED();
            break;
        }
        case PUBSUB_PRESSURE_SIG: {
            const Int16Event_T *event = Q_EVT_CAST(Int16Event_T);
            me->pressure              = event->num;
            status                    = Q_HANDLED();
            break;
        }
        case PUBSUB_TEMPERATURE_SIG: {
            const Int16Event_T *event = Q_EVT_CAST(Int16Event_T);
            me->temperature           = event->num;
            status                    = Q_HANDLED();
            break;
        }
        // case PUBSUB_TACH_SIG: {
        //     const FloatEvent_T *event = Q_EVT_CAST(FloatEvent_T);
        //     me->tachometer = TACH_LAMBDA * me->tachometer + (1 - TACH_LAMBDA) * event->num;
        //     BSP_Tach_Capture_Timer_Enable();
        //     status = Q_HANDLED();
        //     break;
        // }
        default: {
            status = Q_SUPER(&QHsm_top);
            break;
        }
    }

    return status;
}

// state that periodically bundles up information and publishes it on QP
// and over the USART2 to the gauge cluster PCB
static QState running(DataManager *const me, QEvt const *const e)
{
    QState status;

    switch (e->sig)
    {
        case Q_ENTRY_SIG: {
            status = Q_HANDLED();
            break;
        }
        case WAIT_TIMEOUT_SIG: {
            bool neutral               = BSP_Get_Neutral();
            bool start                 = BSP_Get_Start();
            Colored_Wire_Stat_T red    = BSP_Get_Red();
            Colored_Wire_Stat_T orange = BSP_Get_Orange();
            bool buzzer                = BSP_Get_Buzzer();
            me->vbat_volts = VBAT_LAMBDA * me->vbat_volts + (1 - VBAT_LAMBDA) * BSP_ADC_Read_VBAT();

            float this_tach = Flow_Sensor_Read_Hz();
            this_tach       = this_tach * 60 / 6.666;
            me->tachometer  = TACH_LAMBDA * me->tachometer + (1 - TACH_LAMBDA) * this_tach;

            MotorDataEvent_T *event = Q_NEW(MotorDataEvent_T, PUBSUB_MOTOR_DATA_SIG);
            event->neutral          = neutral;
            event->start            = start;
            event->red              = red;
            event->orange           = orange;
            event->buzzer           = buzzer;
            event->vbat             = (int16_t) (me->vbat_volts * 100);
            event->temperature      = me->temperature;
            event->pressure         = me->pressure;
            event->tachometer       = (uint16_t) me->tachometer;
            QACTIVE_PUBLISH(&event->super, &me->super);

            static char printBuffer[32];
            memset(printBuffer, 0, sizeof(printBuffer));
            sprintf(
                printBuffer,
                "P%d\r\nT%d\r\nR%d\r\n",
                me->pressure,
                me->temperature,
                (int) me->tachometer);
            HAL_UART_Transmit_IT(me->p_huart, (uint8_t *) printBuffer, sizeof(printBuffer));

            status = Q_TRAN(&waitingForUart);
            break;
        }
        default: {
            status = Q_SUPER(&top);
            break;
        }
    }

    return status;
}

static QState waitingForUart(DataManager *const me, QEvt const *const e)
{
    QState status;

    switch (e->sig)
    {
        case Q_ENTRY_SIG: {
            status = Q_HANDLED();
            break;
        }
        case PUBSUB_UART_COMPLETE_SIG: {
            status = Q_TRAN(&running);
            break;
        }
        default: {
            status = Q_SUPER(&top);
            break;
        }
    }

    return status;
}