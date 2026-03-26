#include "box_to_box.h"
#include "bsp.h"
#include "flowsensor.h"
#include "posted_signals.h"
#include "pubsub_signals.h"
#include "stm32g4xx_hal.h"

Q_DEFINE_THIS_MODULE("interrupts.c")

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

    Flow_Sensor_Period_Elapsed_Callback(htim);

    // if (input_capture_found)
    //     return;

    // engine RPM is very low (below stall speed), or more likely: engine is off

    // FloatEvent_T *capture_event = Q_NEW(FloatEvent_T, PUBSUB_TACH_SIG);
    // capture_event->num          = 0;
    // QACTIVE_PUBLISH(&capture_event->super, &me->super);
}

/**
 ***************************************************************************************************
 * @brief   TACH input TIM15 capture compare callback (tach input frequency is at least 183 RPM)
 **************************************************************************************************/

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    // static volatile uint32_t captured_val = 0;

    if (htim->Instance != TIM15 || htim->Channel != HAL_TIM_ACTIVE_CHANNEL_1)
    {
        return;
    }

    Flow_Sensor_IC_Callback(htim, TIM_CHANNEL_1);

    // captured_val        = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
    // input_capture_found = true;

    // if (captured_val == 0)
    //     return;

    // // TIM15 is on the APB2 clock bus, which is 144 MHz, and scaled down by (71+1) to 2Mhz
    // // microseconds = periods / 2
    // // Hz = 10^6*2/periods
    // float frequency = 1000.0 * 1000.0 * 2.0 / captured_val; // hz

    // frequency *= 60; // convert to rpm

    // frequency = frequency / 6.666; // fudge factor needed for the BF20D motor

    // FloatEvent_T *capture_event = Q_NEW(FloatEvent_T, PUBSUB_TACH_SIG);
    // capture_event->num          = frequency;
    // QACTIVE_PUBLISH(&capture_event->super, &me->super);

    // HAL_NVIC_DisableIRQ(TIM1_BRK_TIM15_IRQn);
}

/**
 ***************************************************************************************************
 * @brief   CAN Message Received callback
 *
 *          This posts a QP message directly to the BOX_TO_BOX AO
 **************************************************************************************************/
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    HAL_StatusTypeDef retval;
    FDCAN_RxHeaderTypeDef RxHeader;

    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
    {
        CAN_Message_Received_Event_T *evt = Q_NEW(
            CAN_Message_Received_Event_T, POSTED_CAN_MESSAGE_RECEIVED_SIG);

        /* Retrieve Rx messages from RX FIFO0 */
        retval = HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, evt->msg.data);

        Q_ASSERT(retval == HAL_OK);

        evt->msg.id  = RxHeader.Identifier;
        evt->msg.dlc = RxHeader.DataLength;

        QACTIVE_POST(AO_BOX_TO_BOX, &evt->super, 0U);
    }
}