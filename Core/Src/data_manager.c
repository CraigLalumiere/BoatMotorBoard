#include "data_manager.h"
#include "bsp.h"
#include "private_signal_ranges.h"
#include "pubsub_signals.h"

#ifdef Q_SPY
Q_DEFINE_THIS_MODULE("Data Manager")
#endif // def Q_SPY

/**************************************************************************************************\
* Private macros
\**************************************************************************************************/

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
static QState running(DataManager *const me, QEvt const *const e);

/**************************************************************************************************\
* Public functions
\**************************************************************************************************/

/**
 ***************************************************************************************************
 * @brief   Constructor
 **************************************************************************************************/
void Data_Manager_ctor()
{
    DataManager *const me = &data_manager_inst;

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

    QActive_subscribe((QActive *)me, PUBSUB_PRESSURE_SIG);
    QActive_subscribe((QActive *)me, PUBSUB_TEMPERATURE_SIG);

    // Start update loop of 100hz
    QTimeEvt_armX(
        &me->timer_evt,
        BSP_TICKS_PER_SEC / 100,
        BSP_TICKS_PER_SEC / 100);

    return Q_TRAN(&running);
}

static QState running(DataManager *const me, QEvt const *const e)
{
    QState status;

    switch (e->sig)
    {
    case Q_ENTRY_SIG:
    {
        status = Q_HANDLED();
        break;
    }
    case WAIT_TIMEOUT_SIG:
    {
        bool neutral = BSP_Get_Neutral();
        bool start = BSP_Get_Start();
        uint8_t red = BSP_Get_Red();
        uint8_t orange = BSP_Get_Orange();
        bool buzzer = BSP_Get_Buzzer();
        uint16_t vbat_voltage = BSP_ADC_Read_VBAT();

        MotorDataEvent_T *event = Q_NEW(
            MotorDataEvent_T, PUBSUB_MOTOR_DATA_SIG);
        event->neutral = neutral;
        event->start = start;
        event->red = red;
        event->orange = orange;
        event->buzzer = buzzer;
        event->vbat = vbat_voltage;
        event->temperature = me->temperature;
        event->pressure = me->pressure;
        QACTIVE_PUBLISH(&event->super, &me->super);

        status = Q_HANDLED();
        break;
    }
    case PUBSUB_PRESSURE_SIG:
    {
        const Int16Event_T *event = Q_EVT_CAST(Int16Event_T);
        me->pressure = event->num;
        status = Q_HANDLED();
        break;
    }
    case PUBSUB_TEMPERATURE_SIG:
    {
        const Int16Event_T *event = Q_EVT_CAST(Int16Event_T);
        me->temperature = event->num;
        status = Q_HANDLED();
        break;
    }
    default:
    {
        status = Q_SUPER(&QHsm_top);
        break;
    }
    }

    return status;
}