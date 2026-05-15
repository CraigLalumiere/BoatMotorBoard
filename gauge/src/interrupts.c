#include "box_to_box.h"
#include "bsp.h"
#include "posted_signals.h"
#include "pubsub_signals.h"
#include "stm32g4xx_hal.h"

Q_DEFINE_THIS_MODULE("interrupts.c")

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