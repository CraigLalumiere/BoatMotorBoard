extern "C" {
#include "fault_manager.h"
#include "pubsub_signals.h"
}

#include "cmsTestPublishedEventRecorder.hpp"
#include "cms_cpputest_qf_ctrl.hpp"

#include "CppUTest/TestHarness.h"

#include <cstring>

using namespace cms::test;

TEST_GROUP(FaultManagerTests) {
    PublishedEventRecorder *recorder;

    void setup() final
    {
        qf_ctrl::MemPoolConfigs configs = {
            {sizeof(FaultGeneratedEvent_T), 4},
        };

        qf_ctrl::Setup(PUBSUB_MAX_SIG, 1000, configs);
        recorder = PublishedEventRecorder::CreatePublishedEventRecorder(
            qf_ctrl::RECORDER_PRIORITY, PUBSUB_FAULT_GENERATED_SIG, PUBSUB_FAULT_GENERATED_SIG + 1);
    }

    void teardown() final
    {
        delete recorder;
        qf_ctrl::Teardown();
    }
};

TEST(FaultManagerTests, generate_fault_records_and_publishes_bounded_fault_details)
{
    char long_msg[FAULT_GEN_EVENT_MAX_MSG_LENGTH + 16U];
    memset(long_msg, 'x', sizeof(long_msg));
    long_msg[sizeof(long_msg) - 1U] = '\0';

    Fault_Manager_Generate_Fault(nullptr, FAULT_ID_LMT01_NO_PULSES, long_msg);
    qf_ctrl::ProcessEvents();

    auto event = recorder->getRecordedEvent();
    CHECK_TRUE(event != nullptr);
    CHECK_EQUAL(PUBSUB_FAULT_GENERATED_SIG, event->sig);

    FaultGeneratedEvent_T const *fault_event =
        reinterpret_cast<FaultGeneratedEvent_T const *>(event.get());
    CHECK_EQUAL(FAULT_ID_LMT01_NO_PULSES, fault_event->id);
    CHECK_EQUAL(FAULT_TYPE_DRIVER, fault_event->type);
    CHECK_EQUAL(1006U, fault_event->code);
    CHECK_EQUAL('\0', fault_event->msg[FAULT_GEN_EVENT_MAX_MSG_LENGTH - 1U]);

    Active_Fault_T *active_faults = Fault_Manager_Get_Active_Fault_List();
    CHECK_EQUAL(FAULT_ID_LMT01_NO_PULSES, active_faults[0].id);
    CHECK_EQUAL('\0', active_faults[0].msg[FAULT_GEN_EVENT_MAX_MSG_LENGTH - 1U]);

    STRCMP_EQUAL("LMT01 No Pulse Fault", Fault_Manager_Get_Description(FAULT_ID_LMT01_NO_PULSES));
    CHECK_EQUAL(1006U, Fault_Manager_Get_Code(FAULT_ID_LMT01_NO_PULSES));
}
