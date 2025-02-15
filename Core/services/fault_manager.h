
#ifndef FAULT_MANAGER_H_
#define FAULT_MANAGER_H_

#include "qpc.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**************************************************************************************************\
* Public macros
\**************************************************************************************************/

#define FAULT_MANAGER_BUFFER_LENGTH    10
#define FAULT_GEN_EVENT_MAX_MSG_LENGTH 64

/**************************************************************************************************\
* Public type definitions
\**************************************************************************************************/

typedef enum
{
    FAULT_TYPE_NONE,
    FAULT_TYPE_DRIVER,
} Fault_Type_T;

typedef enum
{
    FAULT_ID_NONE,
    FAULT_ID_PRESSURE_SENSOR_GEN,
    FAULT_ID_PRESSURE_SENSOR_I2C,
    FAULT_ID_OLED_GEN,
    FAULT_ID_OLED_I2C,
    FAULT_ID_NUM_FAULT_IDS,
} Fault_ID_T;

typedef struct
{
    Fault_ID_T id;
    char msg[FAULT_GEN_EVENT_MAX_MSG_LENGTH];
} Active_Fault_T;

/**************************************************************************************************\
* Public prototypes
\**************************************************************************************************/

void Fault_Manager_Generate_Fault(QActive *sender, Fault_ID_T id, const char *msg);

const char *Fault_Manager_Get_Description(Fault_ID_T id);

uint16_t Fault_Manager_Get_Code(Fault_ID_T id);

Active_Fault_T *Fault_Manager_Get_Active_Fault_List();

#ifdef __cplusplus
}
#endif

#endif // FAULT_MANAGER_H_
