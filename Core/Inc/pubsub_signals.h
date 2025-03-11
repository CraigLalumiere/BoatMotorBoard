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
    PUBSUB_TACH_SIG,
    PUBSUB_UART_COMPLETE_SIG,
    PUBSUB_MAX_SIG
};

typedef enum
{
    LOW,
    HIGH,
    HIGH_Z,
    SIG_UNKNOWN
} Colored_Wire_Stat_T;

typedef struct
{
    QEvt super;
    float num; // size of the buffer 'instructions', which should be >= actual size
} FloatEvent_T;

typedef struct
{
    QEvt super;
    int16_t num; // size of the buffer 'instructions', which should be >= actual size
} Int16Event_T;

typedef struct
{
    QEvt super;
    int16_t temperature;
    int16_t pressure;
    int16_t tachometer;
    int16_t vbat;
    bool start;
    bool neutral;
    bool buzzer;
    Colored_Wire_Stat_T red;
    Colored_Wire_Stat_T orange;
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
