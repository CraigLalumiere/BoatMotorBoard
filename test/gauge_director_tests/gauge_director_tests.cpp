extern "C" {
#include "director.h"
#include "private_signal_ranges.h"
#include "pubsub_signals.h"
}

#include "cmsTestPublishedEventRecorder.hpp"
#include "cms_cpputest_qf_ctrl.hpp"

#include "CppUTest/TestHarness.h"

using namespace cms::test;

static QEvt const *s_queue_storage[10];
static float s_pressure_volts;
static float s_temperature_volts;
static float s_op_amp_ref_volts;
static uint32_t s_rpm;
static bool s_backlight_output;
static bool s_backlight_input;

extern "C" void BSP_Gauge_SetPressure_V(float volts) { s_pressure_volts = volts; }
extern "C" void BSP_Gauge_SetTemperature_V(float volts) { s_temperature_volts = volts; }
extern "C" void BSP_Gauge_SetOpAmpRef_V(float volts) { s_op_amp_ref_volts = volts; }
extern "C" void BSP_RpmGauge_SetPFM_RPM(uint32_t rpm) { s_rpm = rpm; }
extern "C" void BSP_Set_Backlight(bool on) { s_backlight_output = on; }
extern "C" bool BSP_Get_Backlight(void) { return s_backlight_input; }

static void postPollTimeout(void)
{
    QEvt event = QEVT_INITIALIZER(PRIVATE_SIGNAL_DIRECTOR_START + 1U);
    qf_ctrl::PostAndProcess(&event, AO_DIRECTOR);
}

TEST_GROUP(GaugeDirectorTests) {
    PublishedEventRecorder *recorder;

    void setup() final
    {
        qf_ctrl::MemPoolConfigs configs = {
            {sizeof(QEvt), 4},
            {sizeof(MotorDataEvent_T), 4},
        };

        s_pressure_volts    = 0.0F;
        s_temperature_volts = 0.0F;
        s_op_amp_ref_volts  = 0.0F;
        s_rpm               = 999U;
        s_backlight_output  = true;
        s_backlight_input   = false;

        qf_ctrl::Setup(PUBSUB_MAX_SIG, 1000, configs);
        recorder = PublishedEventRecorder::CreatePublishedEventRecorder(
            qf_ctrl::RECORDER_PRIORITY, PUBSUB_FIRST_SIG, PUBSUB_MAX_SIG);

        Director_ctor();
        QACTIVE_START(
            AO_DIRECTOR,
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
};

TEST(GaugeDirectorTests, startup_sets_default_outputs_and_starts_box_to_box)
{
    DOUBLES_EQUAL(2.8, s_pressure_volts, 0.001);
    DOUBLES_EQUAL(1.3, s_temperature_volts, 0.001);
    DOUBLES_EQUAL(1.78, s_op_amp_ref_volts, 0.001);
    CHECK_EQUAL(0U, s_rpm);
    CHECK_FALSE(s_backlight_output);

    auto event = recorder->getRecordedEvent();
    CHECK_TRUE(event != nullptr);
    CHECK_EQUAL(PUBSUB_BOX_TO_BOX_STARTUP_SIG, event->sig);
}

TEST(GaugeDirectorTests, motor_data_event_updates_gauges_with_mapped_values)
{
    MotorDataEvent_T event = {};
    event.super            = QEVT_INITIALIZER(PUBSUB_MOTOR_DATA_SIG);
    event.temperature      = 44.0F;
    event.pressure         = 10.0F;
    event.tachometer       = 1234.9F;

    qf_ctrl::PublishAndProcess(&event.super);

    DOUBLES_EQUAL(2.7, s_pressure_volts, 0.001);
    DOUBLES_EQUAL(1.9, s_temperature_volts, 0.001);
    CHECK_EQUAL(1234U, s_rpm);
}

TEST(GaugeDirectorTests, motor_data_values_clamp_to_calibration_table_edges)
{
    MotorDataEvent_T event = {};
    event.super            = QEVT_INITIALIZER(PUBSUB_MOTOR_DATA_SIG);
    event.temperature      = 999.0F;
    event.pressure         = -5.0F;
    event.tachometer       = 0.0F;

    qf_ctrl::PublishAndProcess(&event.super);

    DOUBLES_EQUAL(2.8, s_pressure_volts, 0.001);
    DOUBLES_EQUAL(2.7, s_temperature_volts, 0.001);
}

TEST(GaugeDirectorTests, poll_timeout_mirrors_backlight_input_to_output)
{
    s_backlight_input = true;
    postPollTimeout();
    CHECK_TRUE(s_backlight_output);

    s_backlight_input = false;
    postPollTimeout();
    CHECK_FALSE(s_backlight_output);
}
