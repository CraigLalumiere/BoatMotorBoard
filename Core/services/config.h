#ifndef CONFIG_H_
#define CONFIG_H_

#include "qpc.h"
#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    CFG_VAL_TYPE_BOOL,
    CFG_VAL_TYPE_U32,
    CFG_VAL_TYPE_I32,
    CFG_VAL_TYPE_F32,
} ConfigValueType_T;

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

extern QActive *const AO_Config;

void Config_ctor(void);

uint32_t Config_Read_U32(ConfigID_T id);
uint32_t Config_Read_Default_U32(ConfigID_T id);
uint32_t Config_Read_Saved_U32(ConfigID_T id);
void Config_Write_U32(ConfigID_T id, uint32_t value);

int32_t Config_Read_I32(ConfigID_T id);
int32_t Config_Read_Default_I32(ConfigID_T id);
int32_t Config_Read_Saved_I32(ConfigID_T id);
void Config_Write_I32(ConfigID_T id, int32_t value);

float Config_Read_F32(ConfigID_T id);
float Config_Read_Default_F32(ConfigID_T id);
float Config_Read_Saved_F32(ConfigID_T id);
void Config_Write_F32(ConfigID_T id, float value);

bool Config_Read_Bool(ConfigID_T id);
bool Config_Read_Default_Bool(ConfigID_T id);
bool Config_Read_Saved_Bool(ConfigID_T id);
void Config_Write_Bool(ConfigID_T id, bool value);

void Config_SetDefault(ConfigID_T id);
void Config_SetDefaultAll(void);
void Config_Save(void);

uint32_t Config_Get_Num_Elements(void);
uint32_t Config_Get_Version(void);

bool Config_IsNVMValid(void);

const char *Config_GetName(ConfigID_T id);
ConfigID_T Config_GetByName(const char *name);
ConfigValueType_T Config_GetType(ConfigID_T id);

#endif // CONFIG_H_
