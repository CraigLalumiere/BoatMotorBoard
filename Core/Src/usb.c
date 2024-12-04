#include "usb.h"
#include "bsp.h"
#include "tusb.h"
#include <ctype.h>

#include "private_signal_ranges.h"

enum USBSignals
{
    USB_TASK_TIMEOUT_SIG = PRIVATE_SIGNAL_USB_START,
};

typedef struct
{
    QActive super;        // inherit QActive
    QTimeEvt taskTimeEvt; // time event to call tusb task
} USB;

// state handler functions
static QState USB_initial(USB *const me, void const *const par);
static QState USB_active(USB *const me, QEvt const *const e);

static USB USB_inst;
QActive *const AO_USB = &USB_inst.super;

void USB_ctor(void)
{
    USB *const me = &USB_inst;
    QActive_ctor(&me->super, Q_STATE_CAST(&USB_initial));
    QTimeEvt_ctorX(&me->taskTimeEvt, &me->super, USB_TASK_TIMEOUT_SIG, 0U);
}

////////////////////////
// HSM definition
////////////////////////
QState USB_initial(USB *const me, void const *const par)
{
    Q_UNUSED_PAR(par);

#ifdef Q_SPY
    // Add object and all state functions to the QSPY dictionaries
    QS_OBJ_DICTIONARY(me);
    QS_FUN_DICTIONARY(&USB_active);
#endif

    // taskTimeEvt will fire every 10ms
    QTimeEvt_armX(&me->taskTimeEvt, BSP_TICKS_PER_SEC / 100U, BSP_TICKS_PER_SEC / 100U);

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
