#include "blinky.h"
#include "bsp.h"
#include "private_signal_ranges.h"

/**************************************************************************************************\
* Private macros
\**************************************************************************************************/

/**************************************************************************************************\
* Private type definitions
\**************************************************************************************************/

enum BlinkySignals
{
    BLINKY_TIMEOUT_SIG = PRIVATE_SIGNAL_BLINKY_START,
};

typedef struct
{
    QActive super;    // inherit QActive
    QTimeEvt timeEvt; // private time event generator
} Blinky;

/**************************************************************************************************\
* Private memory declarations
\**************************************************************************************************/

Blinky Blinky_inst; // TODO: should this be static?
QActive *const AO_Blinky = &Blinky_inst.super;

/**************************************************************************************************\
* Private prototypes
\**************************************************************************************************/

static QState Blinky_initial(Blinky *const me, void const *const par);
static QState Blinky_off(Blinky *const me, QEvt const *const e);
static QState Blinky_on(Blinky *const me, QEvt const *const e);

/**************************************************************************************************\
* Public functions
\**************************************************************************************************/
void Blinky_ctor(void)
{
    Blinky *const me = &Blinky_inst;
    QActive_ctor(&me->super, Q_STATE_CAST(&Blinky_initial));
    QTimeEvt_ctorX(&me->timeEvt, &me->super, BLINKY_TIMEOUT_SIG, 0U);
}

// HSM definition ----------------------------------------------------------
QState Blinky_initial(Blinky *const me, void const *const par)
{
    Q_UNUSED_PAR(par);

    // arm the time event to expire in half a second and every half second
    QTimeEvt_armX(&me->timeEvt, BSP_TICKS_PER_SEC / 2U, BSP_TICKS_PER_SEC / 2U);

    return Q_TRAN(&Blinky_off);
}

//............................................................................
QState Blinky_off(Blinky *const me, QEvt const *const e)
{
    QState status;
    switch (e->sig)
    {
        case Q_ENTRY_SIG: {
            BSP_LED_Off();
            status = Q_HANDLED();
            break;
        }
        case BLINKY_TIMEOUT_SIG: {
            status = Q_TRAN(&Blinky_on);
            break;
        }
        default: {
            status = Q_SUPER(&QHsm_top);
            break;
        }
    }
    return status;
}
//............................................................................
QState Blinky_on(Blinky *const me, QEvt const *const e)
{
    QState status;
    switch (e->sig)
    {
        case Q_ENTRY_SIG: {
            BSP_LED_On();
            status = Q_HANDLED();
            break;
        }
        case BLINKY_TIMEOUT_SIG: {
            status = Q_TRAN(&Blinky_off);
            break;
        }
        default: {
            status = Q_SUPER(&QHsm_top);
            break;
        }
    }
    return status;
}

/**************************************************************************************************\
* Private functions
\**************************************************************************************************/