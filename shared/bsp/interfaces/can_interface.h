#ifndef CAN_INTERFACE_H_
#define CAN_INTERFACE_H_

#include "stdint.h"

/**************************************************************************************************\
* Public macros
\**************************************************************************************************/
#define CAN_MAX_DATA_LENGTH 64

static const uint8_t CAN_DLC_to_Bytes[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};

/**************************************************************************************************\
* Public type definitions
\**************************************************************************************************/
typedef struct
{
    uint32_t id; // This parameter must be a number between 0 and 0x7FF
    uint32_t dlc;
    uint8_t data[CAN_MAX_DATA_LENGTH];
} __attribute__((packed, aligned(1))) CAN_Message_T;

#endif // CAN_INTERFACE_H_
