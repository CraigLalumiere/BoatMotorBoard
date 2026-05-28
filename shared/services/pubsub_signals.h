#ifndef PUBSUB_SIGNALS_H_
#define PUBSUB_SIGNALS_H_

#include "fault_manager.h"
#include "qpc.h"
#include <stddef.h>

enum PubSubSignals
{
    PUBSUB_FIRST_SIG = Q_USER_SIG,
    PUBSUB_FAULT_GENERATED_SIG,
    PUBSUB_PRESSURE_SIG,
    PUBSUB_TEMPERATURE_SIG,
    PUBSUB_MOTOR_DATA_SIG,
    PUBSUB_FRAM_READY_SIG,
    PUBSUB_CONFIG_READY_SIG,
    PUBSUB_CONFIG_ENTRY_CHANGED_SIG,
    PUBSUB_BOX_TO_BOX_STARTUP_SIG,
    PUBSUB_MAX_SIG
};

typedef struct
{
    QEvt super;
    float num;
} FloatEvent_T;

typedef struct
{
    QEvt super;
    int16_t num;
} Int16Event_T;

typedef struct
{
    QEvt super;
    float temperature;
    float pressure;
    float tachometer;
    float vbat;
    uint32_t engine_minutes;
    bool start;
    bool neutral;
    bool buzzer;
    bool temp_good;
    bool pres_good;
} MotorDataEvent_T;

typedef struct
{
    QEvt super;

    Fault_ID_T id;
    Fault_Type_T type;
    uint16_t code;
    char msg[FAULT_GEN_EVENT_MAX_MSG_LENGTH];
} FaultGeneratedEvent_T;

#endif // PUBSUB_SIGNALS_H_
