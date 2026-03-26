#include "director.h"
#include "bsp.h"
#include "flowsensor.h"
#include "private_signal_ranges.h"
#include "pubsub_signals.h"
#include "stm32g4xx_hal.h"
#include <stdio.h>
#include <string.h>

#ifdef Q_SPY
Q_DEFINE_THIS_MODULE("Director")
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
    WAIT_TIMEOUT_SIG = PRIVATE_SIGNAL_DIRECTOR_START,
};

typedef struct
{
    QActive super; // inherit QActive
    QTimeEvt timer_evt;

    int16_t pressure;
    int16_t temperature;
    float vbat_volts;
    float tachometer;
} Director;

/**************************************************************************************************\
* Private memory declarations
\**************************************************************************************************/
static Director director_inst;
QActive *const AO_Director = &director_inst.super;

/**************************************************************************************************\
* Private prototypes
\**************************************************************************************************/

// state handler functions
static QState initial(Director *const me, void const *const par);
static QState top(Director *const me, QEvt const *const e);
static QState running(Director *const me, QEvt const *const e);

/**************************************************************************************************\
* Public functions
\**************************************************************************************************/

/**
 ***************************************************************************************************
 * @brief   Constructor
 **************************************************************************************************/
void Director_ctor()
{
    Director *const me = &director_inst;

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
static QState initial(Director *const me, void const *const par)
{
    Q_UNUSED_PAR(par);

    QActive_subscribe((QActive *) me, PUBSUB_PRESSURE_SIG);
    QActive_subscribe((QActive *) me, PUBSUB_TEMPERATURE_SIG);

    // BSP_Tach_Capture_Timer_Enable();

    // Start update loop of 100hz
    QTimeEvt_armX(&me->timer_evt, BSP_TICKS_PER_SEC / 100, BSP_TICKS_PER_SEC / 100);

    return Q_TRAN(&running);
}

// top state that handles receiving data from various sources
static QState top(Director *const me, QEvt const *const e)
{
    QState status;

    switch (e->sig)
    {
        case Q_ENTRY_SIG: {
            // startup Box to Box (CAN)
            QEvt *evt = Q_NEW(QEvt, PUBSUB_BOX_TO_BOX_STARTUP_SIG);
            QACTIVE_PUBLISH(evt, &me->super);

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
static QState running(Director *const me, QEvt const *const e)
{
    QState status;

    switch (e->sig)
    {
        case Q_ENTRY_SIG: {
            status = Q_HANDLED();
            break;
        }
        case WAIT_TIMEOUT_SIG: {
            bool neutral   = BSP_Get_Neutral();
            bool start     = BSP_Get_Start();
            bool temp_good = BSP_Get_Temp_Good();
            bool pres_good = BSP_Get_Pres_Good();
            bool buzzer    = BSP_Get_Buzzer();
            me->vbat_volts = VBAT_LAMBDA * me->vbat_volts + (1 - VBAT_LAMBDA) * BSP_ADC_Read_VBAT();

            float this_tach = Flow_Sensor_Read_Hz();
            this_tach       = this_tach * 60 / 6.666; // Hz to RPM + fudge factor for BF20
            me->tachometer  = TACH_LAMBDA * me->tachometer + (1 - TACH_LAMBDA) * this_tach;

            MotorDataEvent_T *event = Q_NEW(MotorDataEvent_T, PUBSUB_MOTOR_DATA_SIG);
            event->neutral          = neutral;
            event->start            = start;
            event->temp_good        = temp_good;
            event->pres_good        = pres_good;
            event->buzzer           = buzzer;
            event->vbat             = (int16_t) (me->vbat_volts * 100);
            event->temperature      = me->temperature;
            event->pressure         = me->pressure;
            event->tachometer       = 3000; //(uint16_t) me->tachometer;
            QACTIVE_PUBLISH(&event->super, &me->super);

            status = Q_HANDLED();
            break;
        }
        default: {
            status = Q_SUPER(&top);
            break;
        }
    }

    return status;
}