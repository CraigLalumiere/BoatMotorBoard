#ifndef POSTED_SIGNALS_H_
#define POSTED_SIGNALS_H_

#include "pubsub_signals.h"
#include "qp.h"

// These are signals that are directly posted from one Active Object to another
//  and do not traverse the Pub Sub framework
enum PostedSignals
{
    POSTED_FIRST_SIG = PUBSUB_MAX_SIG,
    POSTED_APP_CLI_PRINT_SIG,
    POSTED_MAX_SIG
};

// These are signals that are directly dispatched from an Active Object to a component
enum DispatchedSignals
{
    DISPATCHED_FIRST_SIG = POSTED_MAX_SIG,
    DISPATCHED_MAX_SIG
};


#define PRINT_EVENT_MAX_MSG_LENGTH 64

typedef struct
{
    QEvt super;

    uint32_t milliseconds;
    char msg[PRINT_EVENT_MAX_MSG_LENGTH];
} PrintEvent_T;


#endif // POSTED_SIGNALS_H_
