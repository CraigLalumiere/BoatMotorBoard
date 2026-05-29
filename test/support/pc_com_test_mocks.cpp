extern "C" {
#include "cli_commands.h"
#include "config.h"
#include "qpc.h"
}

extern "C" QActive *const AO_Config = nullptr;

extern "C" void CLI_AddCommands(EmbeddedCli *)
{
}

extern "C" uint32_t Config_Get_Num_Elements(void)
{
    return 0U;
}

extern "C" uint32_t Config_Get_Version(void)
{
    return 0U;
}

extern "C" ConfigValueType_T Config_GetType(ConfigID_T)
{
    return CFG_VAL_TYPE_U32;
}

extern "C" const char *Config_GetName(ConfigID_T)
{
    return "";
}

extern "C" uint32_t Config_Read_U32(ConfigID_T)
{
    return 0U;
}

extern "C" uint32_t Config_Read_Default_U32(ConfigID_T)
{
    return 0U;
}

extern "C" void Config_Write_U32(ConfigID_T, uint32_t)
{
}

extern "C" int32_t Config_Read_I32(ConfigID_T)
{
    return 0;
}

extern "C" int32_t Config_Read_Default_I32(ConfigID_T)
{
    return 0;
}

extern "C" void Config_Write_I32(ConfigID_T, int32_t)
{
}

extern "C" bool Config_Read_Bool(ConfigID_T)
{
    return false;
}

extern "C" bool Config_Read_Default_Bool(ConfigID_T)
{
    return false;
}

extern "C" void Config_Write_Bool(ConfigID_T, bool)
{
}

extern "C" float Config_Read_F32(ConfigID_T)
{
    return 0.0F;
}

extern "C" float Config_Read_Default_F32(ConfigID_T)
{
    return 0.0F;
}

extern "C" void Config_Write_F32(ConfigID_T, float)
{
}
