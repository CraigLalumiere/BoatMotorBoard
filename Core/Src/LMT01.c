#include "LMT01.h"
#include "bsp.h"
#include "private_signal_ranges.h"
#include "pubsub_signals.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "stm32g4xx.h"
#include "stm32g4xx_hal.h"

#ifdef Q_SPY
Q_DEFINE_THIS_MODULE("LMT01")
#endif // def Q_SPY

/**************************************************************************************************\
* Private macros
\**************************************************************************************************/

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
    int16_t temperature;
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
    QTimeEvt_armX(
        &me->timer_evt,
        BSP_TICKS_PER_SEC / 100,
        BSP_TICKS_PER_SEC / 100);

    return Q_TRAN(&running);
}

static QState running(LMT01 *const me, QEvt const *const e)
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
        uint16_t prev_counter = me->lmt01_counter;
        me->lmt01_counter = __HAL_TIM_GET_COUNTER(&htim8);

        if (prev_counter > 0 && prev_counter == me->lmt01_counter) {
            __HAL_TIM_SET_COUNTER(&htim8, 0);
            me->temperature = (me->lmt01_counter * 6.25) - 5000; // temperature in hundredths of degrees
            
            Int16Event_T *event = Q_NEW(
                Int16Event_T, PUBSUB_TEMPERATURE_SIG);
            event->num = (int16_t) me->temperature;
            QACTIVE_PUBLISH(&event->super, &me->super);
        }
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