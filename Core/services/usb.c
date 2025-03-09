#include "usb.h"
#include "bsp.h"
#include "private_signal_ranges.h"
#include "tusb.h"
#include <ctype.h>

// Q_DEFINE_THIS_MODULE("usb")

/**************************************************************************************************\
* Private type definitions
\**************************************************************************************************/
enum USBSignals
{
    USB_TASK_TIMEOUT_SIG = PRIVATE_SIGNAL_USB_START,
};

typedef struct
{
    QActive super;         // inherit QActive
    QTimeEvt usb_task_evt; // time event to call tusb task
} USB;

/**************************************************************************************************\
* Private memory declarations
\**************************************************************************************************/
static USB USB_inst;
QActive *const AO_USB = &USB_inst.super;

/**************************************************************************************************\
* Private prototypes
\**************************************************************************************************/
static QState USB_initial(USB *const me, void const *const par);
static QState USB_active(USB *const me, QEvt const *const e);

/**************************************************************************************************\
* Public functions
\**************************************************************************************************/

/**
 ***************************************************************************************************
 * @brief   Constructor
 **************************************************************************************************/
void USB_ctor(void)
{
    USB *const me = &USB_inst;
    QActive_ctor(&me->super, Q_STATE_CAST(&USB_initial));
    QTimeEvt_ctorX(&me->usb_task_evt, &me->super, USB_TASK_TIMEOUT_SIG, 0U);
}

/**************************************************************************************************\
* Private functions
\**************************************************************************************************/
/**
 ***************************************************************************************************
 * @brief   HSM States
 **************************************************************************************************/
QState USB_initial(USB *const me, void const *const par)
{
    Q_UNUSED_PAR(par);

    // Process USB events every 10ms
    QTimeEvt_armX(&me->usb_task_evt, MILLISECONDS_TO_TICKS(10U), MILLISECONDS_TO_TICKS(10U));

    return Q_TRAN(&USB_active);
}

QState USB_active(USB *const me, QEvt const *const e)
{
    QState status;
    switch (e->sig)
    {
        case Q_ENTRY_SIG: {
            status = Q_HANDLED();
            break;
        }
        case USB_TASK_TIMEOUT_SIG: {
            tud_task();
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
