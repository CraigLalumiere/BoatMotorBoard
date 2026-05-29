#ifndef TEST_CONFIG_H_
#define TEST_CONFIG_H_

#include "qpc.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    CFG_ID_ENGINE_MINUTES,
    CFG_ID_NUM_IDS,
    CFG_ID_INVALID = CFG_ID_NUM_IDS
} ConfigID_T;

typedef struct
{
    QEvt super;
    ConfigID_T id;
} ConfigEntryChangedEvent_T;

typedef enum
{
    CFG_VAL_TYPE_U32,
    CFG_VAL_TYPE_I32,
    CFG_VAL_TYPE_BOOL,
    CFG_VAL_TYPE_F32,
} ConfigValueType_T;

extern QActive *const AO_Config;

uint32_t Config_Get_Num_Elements(void);
uint32_t Config_Get_Version(void);
ConfigValueType_T Config_GetType(ConfigID_T id);
const char *Config_GetName(ConfigID_T id);

uint32_t Config_Read_U32(ConfigID_T id);
uint32_t Config_Read_Default_U32(ConfigID_T id);
void Config_Write_U32(ConfigID_T id, uint32_t value);
void Config_Save(void);

int32_t Config_Read_I32(ConfigID_T id);
int32_t Config_Read_Default_I32(ConfigID_T id);
void Config_Write_I32(ConfigID_T id, int32_t value);

bool Config_Read_Bool(ConfigID_T id);
bool Config_Read_Default_Bool(ConfigID_T id);
void Config_Write_Bool(ConfigID_T id, bool value);

float Config_Read_F32(ConfigID_T id);
float Config_Read_Default_F32(ConfigID_T id);
void Config_Write_F32(ConfigID_T id, float value);

#ifdef __cplusplus
}
#endif

#endif // TEST_CONFIG_H_
