extern "C" {
#include "LMT01.h"
#include "private_signal_ranges.h"
#include "pubsub_signals.h"
#include "stm32g4xx_hal.h"
}

#include "cmsTestPublishedEventRecorder.hpp"
#include "cms_cpputest_qf_ctrl.hpp"

#include "CppUTest/TestHarness.h"

using namespace cms::test;

TIM_HandleTypeDef htim8;

static QEvt const *s_queue_storage[10];
static uint32_t s_hal_timer_start_count;

extern "C" HAL_StatusTypeDef HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *)
{
    s_hal_timer_start_count++;
    return HAL_OK;
}

static void postPollTimeout(void)
{
    QEvt event = QEVT_INITIALIZER(PRIVATE_SIGNAL_LMT01_START);
    qf_ctrl::PostAndProcess(&event, AO_LMT01);
}

TEST_GROUP(Lmt01Tests) {
    PublishedEventRecorder *recorder;

    void setup() final
    {
        qf_ctrl::MemPoolConfigs configs = {
            {sizeof(FloatEvent_T), 4},
            {sizeof(FaultGeneratedEvent_T), 4},
        };

        htim8.counter           = 0U;
        s_hal_timer_start_count = 0U;

        qf_ctrl::Setup(PUBSUB_MAX_SIG, 1000, configs);
        recorder = PublishedEventRecorder::CreatePublishedEventRecorder(
            qf_ctrl::RECORDER_PRIORITY, PUBSUB_FAULT_GENERATED_SIG, PUBSUB_TEMPERATURE_SIG + 1);

        LMT01_ctor();
        QACTIVE_START(
            AO_LMT01,
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

TEST(Lmt01Tests, startup_starts_timer_counter)
{
    CHECK_EQUAL(1U, s_hal_timer_start_count);
}

TEST(Lmt01Tests, no_pulses_for_timeout_period_generates_one_fault)
{
    for (uint32_t i = 0U; i < 100U; i++)
    {
        postPollTimeout();
    }

    auto event = recorder->getRecordedEvent();
    CHECK_TRUE(event != nullptr);
    CHECK_EQUAL(PUBSUB_FAULT_GENERATED_SIG, event->sig);

    FaultGeneratedEvent_T const *fault_event =
        reinterpret_cast<FaultGeneratedEvent_T const *>(event.get());
    CHECK_EQUAL(FAULT_ID_LMT01_NO_PULSES, fault_event->id);
    STRCMP_EQUAL("No LMT01 pulses detected", fault_event->msg);

    postPollTimeout();
    CHECK_FALSE(recorder->isAnyEventRecorded());
}

TEST(Lmt01Tests, stable_pulse_count_over_deglitch_threshold_publishes_temperature)
{
    htim8.counter = 800U;
    postPollTimeout();
    htim8.counter = 800U;
    postPollTimeout();

    auto event = recorder->getRecordedEvent();
    CHECK_TRUE(event != nullptr);
    CHECK_EQUAL(PUBSUB_TEMPERATURE_SIG, event->sig);

    FloatEvent_T const *temperature_event = reinterpret_cast<FloatEvent_T const *>(event.get());
    DOUBLES_EQUAL(0.0, temperature_event->num, 0.001);
    CHECK_EQUAL(0U, htim8.counter);
}

TEST(Lmt01Tests, pulse_count_at_or_below_deglitch_threshold_does_not_publish_temperature)
{
    htim8.counter = 10U;
    postPollTimeout();
    htim8.counter = 10U;
    postPollTimeout();

    CHECK_FALSE(recorder->isAnyEventRecorded());
}
