extern "C" {
#include "box_to_box.h"
#include "bsp_box_to_box_mock.h"
#include "can_messages.h"
#include "pubsub_signals.h"
}

#include "cms_cpputest_qf_ctrl.hpp"

#include "CppUTest/TestHarness.h"

using namespace cms::test;

static QEvt const *s_queue_storage[10];

TEST_GROUP(BoxToBoxMotorTests) {
    void setup() final
    {
        BSP_BoxToBoxMock_Reset();
        qf_ctrl::Setup(PUBSUB_MAX_SIG, 1000);
        Box_To_Box_ctor();
        QACTIVE_START(
            AO_BOX_TO_BOX,
            qf_ctrl::UNIT_UNDER_TEST_PRIORITY,
            s_queue_storage,
            Q_DIM(s_queue_storage),
            nullptr,
            0,
            nullptr);
        qf_ctrl::ProcessEvents();
        qf_ctrl::PublishAndProcess(PUBSUB_BOX_TO_BOX_STARTUP_SIG);
    }

    void teardown() final
    {
        qf_ctrl::Teardown();
    }
};

TEST(BoxToBoxMotorTests, startup_initializes_can_bus)
{
    CHECK_EQUAL(1U, BSP_BoxToBoxMock_GetCanBusInitCount());
}

TEST(BoxToBoxMotorTests, motor_data_event_is_encoded_as_can_motor_data_message)
{
    MotorDataEvent_T event = {
        .super          = QEVT_INITIALIZER(PUBSUB_MOTOR_DATA_SIG),
        .temperature    = 81.5F,
        .pressure       = 12.25F,
        .tachometer     = 2350.0F,
        .vbat           = 13.2F,
        .engine_minutes = 456U,
        .start          = true,
        .neutral        = false,
        .buzzer         = true,
        .temp_good      = true,
        .pres_good      = false,
    };

    qf_ctrl::PublishAndProcess(&event.super);

    CHECK_EQUAL(1U, BSP_BoxToBoxMock_GetCanWriteCount());

    CAN_Msg_Motor_Data_T const *msg =
        reinterpret_cast<CAN_Msg_Motor_Data_T const *>(BSP_BoxToBoxMock_GetLastCanMsg());
    CHECK_EQUAL(CAN_MSG_MOTOR_DATA_ID, msg->id);
    CHECK_EQUAL(CAN_MSG_MOTOR_DATA_DLC, msg->dlc);
    CHECK_EQUAL(1234U, msg->tick);
    DOUBLES_EQUAL(event.temperature, msg->temperature, 0.001);
    DOUBLES_EQUAL(event.pressure, msg->pressure, 0.001);
    DOUBLES_EQUAL(event.tachometer, msg->tachometer, 0.001);
    DOUBLES_EQUAL(event.vbat, msg->vbat, 0.001);
    CHECK_EQUAL(event.engine_minutes, msg->engine_minutes);
    CHECK_EQUAL(event.start, msg->start);
    CHECK_EQUAL(event.neutral, msg->neutral);
    CHECK_EQUAL(event.buzzer, msg->buzzer);
    CHECK_EQUAL(event.temp_good, msg->temp_good);
    CHECK_EQUAL(event.pres_good, msg->pres_good);
}

TEST(BoxToBoxMotorTests, can_write_failure_generates_can_fault)
{
    BSP_BoxToBoxMock_SetCanWriteRetval(-1);

    MotorDataEvent_T event = {};
    event.super            = QEVT_INITIALIZER(PUBSUB_MOTOR_DATA_SIG);

    qf_ctrl::PublishAndProcess(&event.super);

    CHECK_EQUAL(1U, BSP_BoxToBoxMock_GetFaultCount());
    CHECK_EQUAL(FAULT_ID_CAN_FAILURE, BSP_BoxToBoxMock_GetLastFaultId());
}
