#include "fault_manager.h"
#include "assert.h"
#include "pubsub_signals.h"
#include "qsafe.h"
#include "safe_strncpy.h"
#include "string.h"

Q_DEFINE_THIS_MODULE("Fault Manager");

/**************************************************************************************************\
* Private type definitions
\**************************************************************************************************/

typedef struct
{
    Fault_ID_T id;
    uint16_t code;
    char *description;
    Fault_Type_T fault_type;
} Fault_ID_Info_T;

/**************************************************************************************************\
* Private memory declarations
\**************************************************************************************************/

static Active_Fault_T active_faults[FAULT_MANAGER_BUFFER_LENGTH] = {0};
static uint8_t num_active_faults                                 = 0;

static const Fault_ID_Info_T s_fault_info_list[] = {
    {FAULT_ID_NONE, 0, "No Fault", FAULT_TYPE_NONE},
    {FAULT_ID_PRESSURE_SENSOR_GEN, 1000, "Pressure Sensor Generic Fault", FAULT_TYPE_DRIVER},
    {FAULT_ID_PRESSURE_SENSOR_I2C, 1001, "Pressure Sensor I2C Fault", FAULT_TYPE_DRIVER},
    {FAULT_ID_OLED_GEN, 2000, "OLED Generic Fault", FAULT_TYPE_DRIVER},
    {FAULT_ID_OLED_I2C, 2001, "OLED I2C Fault", FAULT_TYPE_DRIVER},
};

static_assert(
    sizeof(s_fault_info_list) / sizeof(s_fault_info_list[0]) == FAULT_ID_NUM_FAULT_IDS,
    "s_fault_info_list bad length");

void Fault_Manager_Generate_Fault(QActive *sender, Fault_ID_T fault_id, const char *msg)
{
    (void) sender; // for non-QSpy builds, 'sender' is unused, so do this to avoid warnings

    Q_ASSERT(fault_id > FAULT_ID_NONE);
    Q_ASSERT(fault_id < FAULT_ID_NUM_FAULT_IDS);

    if (num_active_faults < FAULT_MANAGER_BUFFER_LENGTH)
    { // Don't store faults if the buffer is full
        active_faults[num_active_faults].id = fault_id;
        memcpy(active_faults[num_active_faults].msg, msg, FAULT_GEN_EVENT_MAX_MSG_LENGTH);

        num_active_faults++;
    }
    FaultGeneratedEvent_T *event = Q_NEW(FaultGeneratedEvent_T, PUBSUB_FAULT_GENERATED_SIG);

    event->code = 0;
    event->id   = fault_id;
    event->type = s_fault_info_list[fault_id].fault_type;
    event->code = s_fault_info_list[fault_id].code;

    safe_strncpy(event->msg, msg, sizeof(event->msg));

    QACTIVE_PUBLISH(&event->super, sender);
}

const char *Fault_Manager_Get_Description(Fault_ID_T fault_id)
{
    Q_ASSERT(fault_id < FAULT_ID_NUM_FAULT_IDS);
    return s_fault_info_list[fault_id].description;
}

uint16_t Fault_Manager_Get_Code(Fault_ID_T fault_id)
{
    Q_ASSERT(fault_id < FAULT_ID_NUM_FAULT_IDS);
    return s_fault_info_list[fault_id].code;
}

Active_Fault_T *Fault_Manager_Get_Active_Fault_List()
{
    return active_faults;
}
