#ifndef PUBSUB_SIGNALS_H_
#define PUBSUB_SIGNALS_H_

#include "qpc.h"
//#include "services/fault_manager.h"
#include <stddef.h>

enum PubSubSignals
{
    PUBSUB_FIRST_SIG = Q_USER_SIG,
    PUBSUB_FAULT_GENERATED_SIG,
    PUBSUB_PRESSURE_SIG,
    PUBSUB_MAX_SIG
};


typedef struct
{
    QEvt super;
    float num; // size of the buffer 'instructions', which should be >= actual size
} FloatEvent_T;


#endif // PUBSUB_SIGNALS_H_
