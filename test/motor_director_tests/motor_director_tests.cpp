extern "C" {
#include "config.h"
#include "director.h"
#include "private_signal_ranges.h"
#include "pubsub_signals.h"
}

#include "cmsTestPublishedEventRecorder.hpp"
#include "cms_cpputest_qf_ctrl.hpp"

#include "CppUTest/TestHarness.h"

using namespace cms::test;

static QEvt const *s_queue_storage[10];
static bool s_neutral;
static bool s_start;
static bool s_temp_good;
static bool s_pres_good;
static bool s_buzzer;
static float s_vbat;
static float s_flow_hz;
static uint32_t s_engine_minutes;
static uint32_t s_config_write_count;
static uint32_t s_config_save_count;
static ConfigID_T s_last_config_write_id;
static uint32_t s_last_config_write_value;

extern "C" bool BSP_Get_Neutral(void) { return s_neutral; }
extern "C" bool BSP_Get_Start(void) { return s_start; }
extern "C" bool BSP_Get_Temp_Good(void) { return s_temp_good; }
extern "C" bool BSP_Get_Pres_Good(void) { return s_pres_good; }
extern "C" bool BSP_Get_Buzzer(void) { return s_buzzer; }
extern "C" float BSP_ADC_Read_VBAT(void) { return s_vbat; }
extern "C" float Flow_Sensor_Read_Hz(void) { return s_flow_hz; }
extern "C" uint32_t Config_Read_U32(ConfigID_T) { return s_engine_minutes; }
extern "C" void Config_Write_U32(ConfigID_T id, uint32_t value)
{
    s_config_write_count++;
    s_last_config_write_id    = id;
    s_last_config_write_value = value;
    s_engine_minutes          = value;
}
extern "C" void Config_Save(void) { s_config_save_count++; }

static void postDirectorSignal(enum_t sig)
{
    QEvt event = QEVT_INITIALIZER(sig);
    qf_ctrl::PostAndProcess(&event, AO_Director);
}

TEST_GROUP(MotorDirectorTests) {
    PublishedEventRecorder *recorder;

    void setup() final
    {
        qf_ctrl::MemPoolConfigs configs = {
            {sizeof(FloatEvent_T), 4},
            {sizeof(MotorDataEvent_T), 4},
        };

        s_neutral                 = true;
        s_start                   = false;
        s_temp_good               = true;
        s_pres_good               = false;
        s_buzzer                  = true;
        s_vbat                    = 12.0F;
        s_flow_hz                 = 66.66F;
        s_engine_minutes          = 42U;
        s_config_write_count      = 0U;
        s_config_save_count       = 0U;
        s_last_config_write_id    = CFG_ID_INVALID;
        s_last_config_write_value = 0U;

        qf_ctrl::Setup(PUBSUB_MAX_SIG, 1000, configs);
        recorder = PublishedEventRecorder::CreatePublishedEventRecorder(
            qf_ctrl::RECORDER_PRIORITY, PUBSUB_FIRST_SIG, PUBSUB_MAX_SIG);

        Director_ctor();
        QACTIVE_START(
            AO_Director,
            qf_ctrl::UNIT_UNDER_TEST_PRIORITY,
            s_queue_storage,
            Q_DIM(s_queue_storage),
            nullptr,
            0,
            nullptr);
        qf_ctrl::ProcessEvents();
    }

    void teardown() final
    {
        delete recorder;
        qf_ctrl::Teardown();
    }

    cms::QEvtUniquePtr getNextRecordedEventWithSig(PubSubSignals sig)
    {
        for (;;)
        {
            cms::QEvtUniquePtr event = recorder->getRecordedEvent();
            if ((event == nullptr) || (event->sig == sig))
            {
                return event;
            }
        }
    }
};

TEST(MotorDirectorTests, startup_publishes_box_to_box_startup)
{
    auto event = recorder->getRecordedEvent();
    CHECK_TRUE(event != nullptr);
    CHECK_EQUAL(PUBSUB_BOX_TO_BOX_STARTUP_SIG, event->sig);
}

TEST(MotorDirectorTests, periodic_poll_publishes_aggregated_motor_data)
{
    FloatEvent_T pressure_event = {
        .super = QEVT_INITIALIZER(PUBSUB_PRESSURE_SIG),
        .num   = 8.5F,
    };
    FloatEvent_T temperature_event = {
        .super = QEVT_INITIALIZER(PUBSUB_TEMPERATURE_SIG),
        .num   = 72.25F,
    };

    qf_ctrl::PublishAndProcess(&pressure_event.super, recorder);
    qf_ctrl::PublishAndProcess(&temperature_event.super, recorder);
    postDirectorSignal(PRIVATE_SIGNAL_DIRECTOR_START);

    auto event = getNextRecordedEventWithSig(PUBSUB_MOTOR_DATA_SIG);
    CHECK_TRUE(event != nullptr);

    MotorDataEvent_T const *motor_event = reinterpret_cast<MotorDataEvent_T const *>(event.get());
    CHECK_EQUAL(s_neutral, motor_event->neutral);
    CHECK_EQUAL(s_start, motor_event->start);
    CHECK_EQUAL(s_temp_good, motor_event->temp_good);
    CHECK_EQUAL(s_pres_good, motor_event->pres_good);
    CHECK_EQUAL(s_buzzer, motor_event->buzzer);
    DOUBLES_EQUAL(0.12, motor_event->vbat, 0.001);
    DOUBLES_EQUAL(72.25, motor_event->temperature, 0.001);
    DOUBLES_EQUAL(8.5, motor_event->pressure, 0.001);
    DOUBLES_EQUAL(59.994, motor_event->tachometer, 0.01);
    CHECK_EQUAL(42U, motor_event->engine_minutes);
}

TEST(MotorDirectorTests, engine_minute_tick_updates_config_only_when_ready_and_running)
{
    postDirectorSignal(PRIVATE_SIGNAL_DIRECTOR_START);

    postDirectorSignal(PRIVATE_SIGNAL_DIRECTOR_START + 1U);
    CHECK_EQUAL(0U, s_config_write_count);
    CHECK_EQUAL(0U, s_config_save_count);

    QEvt config_ready = QEVT_INITIALIZER(PUBSUB_CONFIG_READY_SIG);
    qf_ctrl::PublishAndProcess(&config_ready, recorder);

    postDirectorSignal(PRIVATE_SIGNAL_DIRECTOR_START + 1U);

    CHECK_EQUAL(1U, s_config_write_count);
    CHECK_EQUAL(CFG_ID_ENGINE_MINUTES, s_last_config_write_id);
    CHECK_EQUAL(43U, s_last_config_write_value);
    CHECK_EQUAL(1U, s_config_save_count);
}
