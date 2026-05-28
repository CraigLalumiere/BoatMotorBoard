#ifndef PC_COM_AO_H_
#define PC_COM_AO_H_

#include "interfaces/serial_interface.h"
#include "qpc.h"
#include "stddef.h"

#ifdef __cplusplus
extern "C" {
#endif

/**************************************************************************************************\
* Public macros
\**************************************************************************************************/

#define PC_COM_EVENT_MAX_MSG_LENGTH 64
#define CLI_DATA_MAX_LENGTH         64

/**************************************************************************************************\
* Public memory declarations
\**************************************************************************************************/
extern QActive *const AO_PC_COM; // opaque pointer

/**************************************************************************************************\
* Public type definitions
\**************************************************************************************************/
typedef struct
{
    QEvt super;

    uint32_t milliseconds;
    char msg[PC_COM_EVENT_MAX_MSG_LENGTH];
} PCCOMPrintEvent_T;

typedef struct
{
    QEvt super;

    char msg[CLI_DATA_MAX_LENGTH];
    uint8_t msg_size;
} PCCOMCliDataEvent_T;

//
// Debug/misc events
//
typedef struct
{
    QEvt super;
    uint32_t desiredFault;
} PCCOMForceFaultEvent_T;

typedef struct
{
    QEvt super;
    uint8_t plot_number;
    char plot_title[64];
    char x_label[64];
    char x_units[8];
    char y_label[64];
    char y_units[8];
    uint16_t max_datapoints;
} ConfigPlotEvent_T;

typedef struct
{
    QEvt super;
    // used when multiple events are needed to transmit all the data for one experiment
    uint16_t message_num;
    // time between samples
    float t_interval;
    float volts[256];
    float amps[256];
    uint16_t data_len;
} Plot_IV_Event_T;

typedef struct
{
    QEvt super;
    // used when multiple events are needed to transmit all the data for one experiment
    uint16_t message_num;
    // time between samples
    float t_interval;
    float volts[256];
    float amps_reverse[128];
    float amps_forward[128];
    uint16_t data_len;
} Plot_DPV_Event_T;

/**************************************************************************************************\
* Public prototypes
\**************************************************************************************************/
void PC_COM_ctor(const Serial_IO_T *const serial_io_interface);
void PC_COM_print(const char *msg);

#ifdef __cplusplus
}
#endif
#endif // PC_COM_AO_H_