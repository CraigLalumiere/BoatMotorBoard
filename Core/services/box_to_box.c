#include "box_to_box.h"
#include "bsp.h"
#include "can_messages.h"
#include "fault_manager.h"
#include "flowsensor.h"
// #include "pc_com.h"
// #include "pressuresensor.h"
#include "cli.h"
#include "private_signal_ranges.h"
#include <stdio.h>
#include <string.h>

// Q_DEFINE_THIS_MODULE("box_to_box")

/**************************************************************************************************\
* Private macros
\**************************************************************************************************/

/**************************************************************************************************\
* Private type definitions
\**************************************************************************************************/
typedef union
{
    CAN_Msg_Test1_T test1;
    CAN_Msg_Pressure_Flow_T box2_pressure_flow_msg;
    CAN_Msg_TDS_T water_tds_msg;
    CAN_Msg_Water_Good_T water_good_msg;
} CAN_Msgs_T;

enum BOX_TO_BOX_Signals
{
    TEST_SIG = PRIVATE_SIGNAL_BOX_TO_BOX_START,
    BUS_TIMOUT_SIG
};

typedef struct
{
    QActive super; // inherit QActive

    QTimeEvt testEvt;
    QTimeEvt timeout_evt;
} Box_To_Box;

/**************************************************************************************************\
* Private memory declarations
\**************************************************************************************************/
static Box_To_Box box_to_box_inst;
QActive *const AO_BOX_TO_BOX = &box_to_box_inst.super;

uint32_t fault_bits = 0;

/**************************************************************************************************\
* Private prototypes
\**************************************************************************************************/
static QState initial(Box_To_Box *const me, void const *const par);
static QState uinitialized(Box_To_Box *const me, QEvt const *const e);
static QState active(Box_To_Box *const me, QEvt const *const e);
static QState bus_error(Box_To_Box *const me, QEvt const *const e);

void handle_can_message_received(Box_To_Box *const me, QEvt const *const e);

/**************************************************************************************************\
* Public functions
\**************************************************************************************************/

/**
 ***************************************************************************************************
 * @brief   Constructor
 **************************************************************************************************/
void Box_To_Box_ctor(void)
{
    Box_To_Box *const me = &box_to_box_inst;

    QActive_ctor(&me->super, Q_STATE_CAST(&initial));

    QTimeEvt_ctorX(&me->testEvt, &me->super, TEST_SIG, 0U);
}

/**************************************************************************************************\
* Private functions
\**************************************************************************************************/

/**
 ***************************************************************************************************
 *
 * @brief   HSM
 *
 **************************************************************************************************/
static QState initial(Box_To_Box *const me, void const *const par)
{
    Q_UNUSED_PAR(par);

    QActive_subscribe((QActive *) me, PUBSUB_MOTOR_DATA_SIG);
    QActive_subscribe((QActive *) me, PUBSUB_FAULT_GENERATED_SIG);

    // for testing
    return Q_TRAN(&uinitialized);
}

static QState uinitialized(Box_To_Box *const me, QEvt const *const e)
{
    QState status;

    switch (e->sig)
    {
        case Q_ENTRY_SIG: {
            status = Q_HANDLED();
            break;
        }

        case PUBSUB_BOX_TO_BOX_STARTUP_SIG: {
            status = Q_TRAN(&active);
            break;
        }

        default: {
            status = Q_SUPER(&QHsm_top);
            break;
        }
    }

    return status;
}

static QState active(Box_To_Box *const me, QEvt const *const e)
{
    QState status;
    uint32_t retval;
    CAN_Msgs_T can_msg;

    switch (e->sig)
    {
        case Q_ENTRY_SIG: {
            BSP_CAN_Bus_Init();

            // send test messages
            QTimeEvt_armX(&me->testEvt, MILLISECONDS_TO_TICKS(50), MILLISECONDS_TO_TICKS(50));
            status = Q_HANDLED();
            break;
        }

        case TEST_SIG: {
            can_msg.test1.id         = CAN_MSG_TEST1_ID;
            can_msg.test1.dlc        = CAN_MSG_TEST1_DLC;
            can_msg.test1.tick       = BSP_Get_Milliseconds_Tick();
            can_msg.test1.variable_1 = 1;
            can_msg.test1.variable_2 = 2;

            retval = BSP_CAN_Write_Msg((CAN_Message_T *) &can_msg.test1);

            // TX mailbox full, this means box1 didn't ACK last 3 messages
            if (retval)
            {
                status = Q_TRAN(&bus_error);
            }
            else
            {
                status = Q_HANDLED();
            }

            break;
        }

        case PUBSUB_FAULT_GENERATED_SIG: {
            Fault_ID_T faultId = Q_EVT_CAST(FaultGeneratedEvent_T)->id;

            // bit mask of fault IDs that have occurred
            // subtract 1 because 'fault #0' is 'FAULT_ID_NONE' which isn't used
            fault_bits |= 1 << (faultId - 1);

            status = Q_HANDLED();
            break;
        }

            // case PUBSUB_PRESSURE_FLOW_UPDATE_SIG: {
            //     float32_t pressure_PSI = Q_EVT_CAST(PressureFlowEvent_T)->pressure_PSI;
            //     float32_t flow_Hz      = Q_EVT_CAST(PressureFlowEvent_T)->flow_Hz;

            //     can_msg.box2_pressure_flow_msg.id                = CAN_MSG_PRESSURE_FLOW_ID;
            //     can_msg.box2_pressure_flow_msg.dlc               = CAN_MSG_PRESSURE_FLOW_DLC;
            //     can_msg.box2_pressure_flow_msg.tick              = BSP_Get_Milliseconds_Tick();
            //     can_msg.box2_pressure_flow_msg.box2_flow_Hz      = flow_Hz;
            //     can_msg.box2_pressure_flow_msg.box2_pressure_PSI = pressure_PSI;
            //     can_msg.box2_pressure_flow_msg.fault_status      = fault_bits;

            //     retval = BSP_CAN_Write_Msg((CAN_Message_T *) &can_msg.box2_pressure_flow_msg);

            //     // TX mailbox full, this means box1 didn't ACK last 3 messages
            //     if (retval)
            //     {
            //         status = Q_TRAN(&bus_error);
            //     }
            //     else
            //     {
            //         status = Q_HANDLED();
            //     }

            //     break;
            // }

        case POSTED_CAN_MESSAGE_RECEIVED_SIG: {
            handle_can_message_received(me, e);
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

static QState bus_error(Box_To_Box *const me, QEvt const *const e)
{
    QState status;

    switch (e->sig)
    {
        case Q_ENTRY_SIG: {
            Fault_Manager_Generate_Fault(&me->super, FAULT_ID_CAN_FAILURE, "TX mailbox full");

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

void handle_can_message_received(Box_To_Box *const me, QEvt const *const e)
{
    CAN_Msgs_T *can_msg;

    const CAN_Message_Received_Event_T *msg_received_evt = Q_EVT_CAST(CAN_Message_Received_Event_T);

    can_msg = (CAN_Msgs_T *) &msg_received_evt->msg;

    // switch (msg_received_evt->msg.id)
    // {
    //     case CAN_MSG_WATER_GOOD_ID: {
    //         if (can_msg->water_good_msg.water_good)
    //             BSP_Blue_LED_On();
    //         else
    //             BSP_Blue_LED_Off();
    //         break;
    //     }
    // }
}