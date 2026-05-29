extern "C" {
#include "pressure_sensor.h"
#include "private_signal_ranges.h"
#include "pubsub_signals.h"
}

#include "cmsTestPublishedEventRecorder.hpp"
#include "cms_cpputest_qf_ctrl.hpp"

#include "CppUTest/TestHarness.h"

#include <chrono>
#include <cstring>

using namespace cms::test;

static QEvt const *s_queue_storage[10];
static bool s_pressure_sensor_in_reset;
static uint32_t s_reset_write_count;
static uint32_t s_i2c_write_count;
static uint32_t s_i2c_read_count;
static I2C_Return_T s_i2c_write_retval;
static I2C_Return_T s_i2c_read_retval;
static uint8_t s_i2c_read_data[7];

extern "C" void BSP_Put_Pressure_Sensor_Into_Reset(bool in_reset)
{
    s_pressure_sensor_in_reset = in_reset;
    s_reset_write_count++;
}

static I2C_Return_T i2c_write(
    uint8_t,
    uint8_t *tx_buffer,
    const uint16_t data_len,
    I2C_Complete_Callback,
    I2C_Error_Callback,
    void *)
{
    s_i2c_write_count++;
    CHECK_EQUAL(3U, data_len);
    CHECK_EQUAL(0xaaU, tx_buffer[0]);
    CHECK_EQUAL(0x00U, tx_buffer[1]);
    CHECK_EQUAL(0x00U, tx_buffer[2]);
    return s_i2c_write_retval;
}

static I2C_Return_T i2c_read(
    uint8_t,
    uint8_t *rx_buffer,
    const uint16_t data_len,
    I2C_Complete_Callback,
    I2C_Error_Callback,
    void *)
{
    s_i2c_read_count++;
    CHECK_EQUAL(sizeof(s_i2c_read_data), data_len);
    memcpy(rx_buffer, s_i2c_read_data, sizeof(s_i2c_read_data));
    return s_i2c_read_retval;
}

static void advanceThroughStartupToRunning(void)
{
    qf_ctrl::MoveTimeForward(std::chrono::milliseconds(10));
    qf_ctrl::MoveTimeForward(std::chrono::milliseconds(10));
}

static void postPressureSignal(enum_t sig)
{
    QEvt event = QEVT_INITIALIZER(sig);
    qf_ctrl::PostAndProcess(&event, AO_Pressure);
}

TEST_GROUP(PressureSensorTests) {
    PublishedEventRecorder *recorder;

    void setup() final
    {
        qf_ctrl::MemPoolConfigs configs = {
            {sizeof(FloatEvent_T), 4},
            {sizeof(FaultGeneratedEvent_T), 4},
        };

        s_pressure_sensor_in_reset = false;
        s_reset_write_count        = 0U;
        s_i2c_write_count          = 0U;
        s_i2c_read_count           = 0U;
        s_i2c_write_retval         = I2C_RTN_SUCCESS;
        s_i2c_read_retval          = I2C_RTN_SUCCESS;
        memset(s_i2c_read_data, 0, sizeof(s_i2c_read_data));
        s_i2c_read_data[1] = 0x19U;
        s_i2c_read_data[2] = 0x99U;
        s_i2c_read_data[3] = 0x9aU;

        qf_ctrl::Setup(PUBSUB_MAX_SIG, 1000, configs);
        recorder = PublishedEventRecorder::CreatePublishedEventRecorder(
            qf_ctrl::RECORDER_PRIORITY, PUBSUB_FIRST_SIG, PUBSUB_MAX_SIG);

        Pressure_Sensor_ctor(i2c_write, i2c_read);
        QACTIVE_START(
            AO_Pressure,
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

TEST(PressureSensorTests, startup_toggles_sensor_reset_before_running)
{
    CHECK_TRUE(s_pressure_sensor_in_reset);
    CHECK_EQUAL(1U, s_reset_write_count);

    qf_ctrl::MoveTimeForward(std::chrono::milliseconds(10));
    CHECK_FALSE(s_pressure_sensor_in_reset);
    CHECK_EQUAL(2U, s_reset_write_count);
}

TEST(PressureSensorTests, successful_conversion_publishes_pressure)
{
    advanceThroughStartupToRunning();

    qf_ctrl::MoveTimeForward(std::chrono::milliseconds(10));
    CHECK_EQUAL(1U, s_i2c_write_count);

    postPressureSignal(PRIVATE_SIGNAL_PRESSURE_START + 2U);
    qf_ctrl::MoveTimeForward(std::chrono::milliseconds(10));
    CHECK_EQUAL(1U, s_i2c_read_count);

    postPressureSignal(PRIVATE_SIGNAL_PRESSURE_START + 2U);

    auto event = recorder->getRecordedEvent();
    CHECK_TRUE(event != nullptr);
    CHECK_EQUAL(PUBSUB_PRESSURE_SIG, event->sig);
    FloatEvent_T const *pressure_event = reinterpret_cast<FloatEvent_T const *>(event.get());
    DOUBLES_EQUAL(0.0, pressure_event->num, 0.001);
}

TEST(PressureSensorTests, watchdog_without_successful_reading_generates_pressure_i2c_fault)
{
    advanceThroughStartupToRunning();

    qf_ctrl::MoveTimeForward(std::chrono::milliseconds(1000));

    auto event = recorder->getRecordedEvent();
    CHECK_TRUE(event != nullptr);
    CHECK_EQUAL(PUBSUB_FAULT_GENERATED_SIG, event->sig);
    FaultGeneratedEvent_T const *fault_event =
        reinterpret_cast<FaultGeneratedEvent_T const *>(event.get());
    CHECK_EQUAL(FAULT_ID_PRESSURE_SENSOR_I2C, fault_event->id);
    STRCMP_EQUAL("Communication Failure", fault_event->msg);
}

TEST(PressureSensorTests, i2c_write_busy_generates_pressure_i2c_fault)
{
    advanceThroughStartupToRunning();
    s_i2c_write_retval = I2C_RTN_BUSY;

    qf_ctrl::MoveTimeForward(std::chrono::milliseconds(10));

    auto event = recorder->getRecordedEvent();
    CHECK_TRUE(event != nullptr);
    CHECK_EQUAL(PUBSUB_FAULT_GENERATED_SIG, event->sig);
    FaultGeneratedEvent_T const *fault_event =
        reinterpret_cast<FaultGeneratedEvent_T const *>(event.get());
    CHECK_EQUAL(FAULT_ID_PRESSURE_SENSOR_I2C, fault_event->id);
    STRCMP_EQUAL("Busy during command", fault_event->msg);
}
