#include "stm32g4xx.h"

/**************************************************************************************************\
* Test Message 1
\**************************************************************************************************/
#define CAN_MSG_TEST1_ID  1U
#define CAN_MSG_TEST1_DLC FDCAN_DLC_BYTES_8

typedef struct
{
    uint32_t id;
    uint32_t dlc;
    uint32_t tick;
    uint16_t variable_1;
    uint16_t variable_2;
} __attribute__((packed, aligned(1))) CAN_Msg_Test1_T;

/**************************************************************************************************\
* Motor Data
\**************************************************************************************************/
#define CAN_MSG_MOTOR_DATA_ID  2U
#define CAN_MSG_MOTOR_DATA_DLC FDCAN_DLC_BYTES_16

typedef struct
{
    uint32_t id;  // unique message ID
    uint32_t dlc; // Data Length Code
    uint32_t tick;
    float temperature;
    float pressure;
    float tachometer;
} __attribute__((packed, aligned(1))) CAN_Msg_Motor_Data_T;

/**************************************************************************************************\
* Water salinity + temperature message
\**************************************************************************************************/
#define CAN_MSG_TDS_ID  3U
#define CAN_MSG_TDS_DLC FDCAN_DLC_BYTES_16

typedef struct
{
    uint32_t id;  // unique message ID
    uint32_t dlc; // Data Length Code
    uint32_t tick;
    float32_t water_temperature_C;
    float32_t water_salinity_PPM;
    uint32_t fault_status;
} __attribute__((packed, aligned(1))) CAN_Msg_TDS_T;

/**************************************************************************************************\
* Water Good Message (controls box 2 blue LED)
\**************************************************************************************************/
#define CAN_MSG_WATER_GOOD_ID  4U
#define CAN_MSG_WATER_GOOD_DLC FDCAN_DLC_BYTES_5

typedef struct
{
    uint32_t id;
    uint32_t dlc;
    uint32_t tick;
    bool water_good;
} __attribute__((packed, aligned(1))) CAN_Msg_Water_Good_T;