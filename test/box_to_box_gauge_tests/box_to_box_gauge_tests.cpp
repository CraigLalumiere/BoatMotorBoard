extern "C" {
#include "box_to_box.h"
#include "bsp_box_to_box_mock.h"
#include "can_messages.h"
#include "posted_signals.h"
#include "pubsub_signals.h"
}

#include "cmsTestPublishedEventRecorder.hpp"
#include "cms_cpputest_qf_ctrl.hpp"

#include "CppUTest/TestHarness.h"

using namespace cms::test;

static QEvt const *s_queue_storage[10];

TEST_GROUP(BoxToBoxGaugeTests) {
    PublishedEventRecorder *recorder;

    void setup() final
    {
        qf_ctrl::MemPoolConfigs configs = {
            {sizeof(MotorDataEvent_T), 4},
        };

        BSP_BoxToBoxMock_Reset();
        qf_ctrl::Setup(PUBSUB_MAX_SIG, 1000, configs);
        recorder = PublishedEventRecorder::CreatePublishedEventRecorder(
            qf_ctrl::RECORDER_PRIORITY, PUBSUB_MOTOR_DATA_SIG, PUBSUB_MOTOR_DATA_SIG + 1);

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
        delete recorder;
        qf_ctrl::Teardown();
    }

    void postCanMessage(CAN_Message_T const *msg)
    {
        CAN_Message_Received_Event_T event = {
            .super = QEVT_INITIALIZER(POSTED_CAN_MESSAGE_RECEIVED_SIG),
            .msg   = *msg,
        };
        qf_ctrl::PostAndProcess(&event.super, AO_BOX_TO_BOX);
    }
};

TEST(BoxToBoxGaugeTests, startup_initializes_can_bus)
{
    CHECK_EQUAL(1U, BSP_BoxToBoxMock_GetCanBusInitCount());
}

TEST(BoxToBoxGaugeTests, can_motor_data_message_is_republished_as_motor_data_event)
{
    CAN_Msg_Motor_Data_T can_msg = {
        .id             = CAN_MSG_MOTOR_DATA_ID,
        .dlc            = CAN_MSG_MOTOR_DATA_DLC,
        .tick           = 99U,
        .temperature    = 76.5F,
        .pressure       = 8.25F,
        .tachometer     = 1800.0F,
        .vbat           = 12.6F,
        .engine_minutes = 789U,
        .start          = false,
        .neutral        = true,
        .buzzer         = false,
        .temp_good      = true,
        .pres_good      = true,
        .reserved       = {0},
    };

    postCanMessage(reinterpret_cast<CAN_Message_T const *>(&can_msg));

    auto event = recorder->getRecordedEvent();
    CHECK_TRUE(event != nullptr);
    CHECK_EQUAL(PUBSUB_MOTOR_DATA_SIG, event->sig);

    MotorDataEvent_T const *motor_event = reinterpret_cast<MotorDataEvent_T const *>(event.get());
    DOUBLES_EQUAL(can_msg.temperature, motor_event->temperature, 0.001);
    DOUBLES_EQUAL(can_msg.pressure, motor_event->pressure, 0.001);
    DOUBLES_EQUAL(can_msg.tachometer, motor_event->tachometer, 0.001);
    DOUBLES_EQUAL(can_msg.vbat, motor_event->vbat, 0.001);
    CHECK_EQUAL(can_msg.engine_minutes, motor_event->engine_minutes);
    CHECK_EQUAL(can_msg.start, motor_event->start);
    CHECK_EQUAL(can_msg.neutral, motor_event->neutral);
    CHECK_EQUAL(can_msg.buzzer, motor_event->buzzer);
    CHECK_EQUAL(can_msg.temp_good, motor_event->temp_good);
    CHECK_EQUAL(can_msg.pres_good, motor_event->pres_good);
}

TEST(BoxToBoxGaugeTests, unknown_can_id_is_ignored)
{
    CAN_Message_T msg = {};
    msg.id            = 0x7ffU;
    msg.dlc           = CAN_MSG_MOTOR_DATA_DLC;

    postCanMessage(&msg);

    CHECK_FALSE(recorder->isAnyEventRecorded());
}
