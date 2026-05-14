#include "LMT01.h"
#include "bsp.h"
#include "fault_manager.h"
#include "private_signal_ranges.h"
#include "pubsub_signals.h"
#include "stm32g4xx_hal.h"
#include <stdbool.h>

#ifdef Q_SPY
Q_DEFINE_THIS_MODULE("LMT01")
#endif // def Q_SPY

/**************************************************************************************************\
* Private macros
\**************************************************************************************************/

#define LAMBDA 0.95
#define LMT01_POLL_TICKS              (BSP_TICKS_PER_SEC / 100U)
#define LMT01_NO_PULSE_TIMEOUT_MS     1000U
#define LMT01_NO_PULSE_TIMEOUT_COUNTS \
    (MILLISECONDS_TO_TICKS(LMT01_NO_PULSE_TIMEOUT_MS) / LMT01_POLL_TICKS)
#define LMT01_INVALID_TEMPERATURE_C (-999.0f)

/**************************************************************************************************\
* Private type definitions
\**************************************************************************************************/

enum PressureSignals
{
    WAIT_TIMEOUT_SIG = PRIVATE_SIGNAL_LMT01_START,
};

typedef struct
{
    QActive super; // inherit QActive
    QTimeEvt timer_evt;
    uint16_t lmt01_counter;
    float temperature;
    uint16_t no_pulse_counts;
    bool no_pulse_fault_reported;
} LMT01;

/**************************************************************************************************\
* Private memory declarations
\**************************************************************************************************/
static LMT01 lmt01_inst;
QActive *const AO_LMT01 = &lmt01_inst.super;

extern TIM_HandleTypeDef htim8; // counter

/**************************************************************************************************\
* Private prototypes
\**************************************************************************************************/

// state handler functions
static QState initial(LMT01 *const me, void const *const par);
static QState running(LMT01 *const me, QEvt const *const e);

/**************************************************************************************************\
* Public functions
\**************************************************************************************************/

/**
 ***************************************************************************************************
 * @brief   Constructor
 **************************************************************************************************/
void LMT01_ctor()
{
    LMT01 *const me = &lmt01_inst;

    QActive_ctor(&me->super, Q_STATE_CAST(&initial));
    QTimeEvt_ctorX(&me->timer_evt, &me->super, WAIT_TIMEOUT_SIG, 0U);

    me->lmt01_counter = 0U;
    me->temperature = LMT01_INVALID_TEMPERATURE_C;
    me->no_pulse_counts = 0U;
    me->no_pulse_fault_reported = false;

    // Start the pulse-counter timer
    HAL_TIM_Base_Start_IT(&htim8);
}

/**************************************************************************************************\
* Private functions
\**************************************************************************************************/

/**
 ***************************************************************************************************
 * @brief   HSM
 **************************************************************************************************/
static QState initial(LMT01 *const me, void const *const par)
{
    Q_UNUSED_PAR(par);

    // Conversion time of the LMT01 should be under 54ms
    QTimeEvt_armX(&me->timer_evt, LMT01_POLL_TICKS, LMT01_POLL_TICKS);

    return Q_TRAN(&running);
}

static QState running(LMT01 *const me, QEvt const *const e)
{
    QState status;

    switch (e->sig)
    {
        case Q_ENTRY_SIG: {
            status = Q_HANDLED();
            break;
        }
        case WAIT_TIMEOUT_SIG: {
            uint16_t prev_counter = me->lmt01_counter;
            me->lmt01_counter     = __HAL_TIM_GET_COUNTER(&htim8);

            if (me->lmt01_counter == 0U)
            {
                if (me->no_pulse_counts < LMT01_NO_PULSE_TIMEOUT_COUNTS)
                {
                    me->no_pulse_counts++;
                }

                if (
                    me->no_pulse_counts >= LMT01_NO_PULSE_TIMEOUT_COUNTS &&
                    !me->no_pulse_fault_reported)
                {
                    Fault_Manager_Generate_Fault(
                        &me->super, FAULT_ID_LMT01_NO_PULSES, "No LMT01 pulses detected");
                    me->no_pulse_fault_reported = true;
                }
            }
            else
            {
                me->no_pulse_counts = 0U;
                me->no_pulse_fault_reported = false;
            }

            if (prev_counter > 0 && prev_counter == me->lmt01_counter)
            {
                __HAL_TIM_SET_COUNTER(&htim8, 0); // reset the timer

                // deglitching
                if (me->lmt01_counter > 10)
                {
                    float new_temperature = (me->lmt01_counter * 0.0625f) -
                        50; // temperature in degrees C

                    if (me->temperature == LMT01_INVALID_TEMPERATURE_C)
                        me->temperature = new_temperature;
                    else
                        me->temperature = LAMBDA * me->temperature + (1 - LAMBDA) * new_temperature;

                    FloatEvent_T *event = Q_NEW(FloatEvent_T, PUBSUB_TEMPERATURE_SIG);
                    event->num          = me->temperature;
                    QACTIVE_PUBLISH(&event->super, &me->super);
                }
            }
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
