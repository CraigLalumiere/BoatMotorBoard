#include "bsp.h"
#include "pubsub_signals.h"
#include "stm32g4xx_hal.h"

extern bool input_capture_found;

/**
 ***************************************************************************************************
 * @brief   TACH input TIM15 period elapsed callback (engine RPM is very low)
 **************************************************************************************************/

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM15)
    {
        return;
    }

    if (input_capture_found)
        return;

    // engine RPM is very low (below stall speed), or more likely: engine is off

    Int16Event_T *capture_event = Q_NEW(Int16Event_T, PUBSUB_TACH_SIG);
    capture_event->num          = 0;
    QACTIVE_PUBLISH(&capture_event->super, &me->super);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    UNUSED(huart);

    static QEvt const event = QEVT_INITIALIZER(PUBSUB_UART_COMPLETE_SIG);
    QACTIVE_PUBLISH(&event, &me->super);
}