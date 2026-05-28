#include "config.h"
#include "fram.h"
#include "log_com.h"
#include "posted_signals.h"
#include "pubsub_signals.h"
#include <string.h>

Q_DEFINE_THIS_MODULE("config")

typedef union
{
    uint32_t u32_val;
    int32_t i32_val;
    float f32_val;
    bool bool_val;
} ConfigValue_T;

typedef struct
{
    const ConfigID_T id;
    const ConfigValueType_T val_type;
    ConfigValue_T val;
    const ConfigValue_T default_val;
    const char *name;
} ConfigDBEntry_T;

typedef struct
{
    const ConfigID_T id;
    ConfigValue_T val;
} ConfigNVMFileElement_T;

typedef struct
{
    uint32_t version;
    uint16_t num_elements;
    ConfigNVMFileElement_T values[(CFG_ID_NUM_IDS > 0U) ? CFG_ID_NUM_IDS : 1U];
} ConfigNVMFile_T;

typedef struct
{
    QActive super;
} Config;

static QState Config_initial(Config *const me, void const *const par);
static QState Config_wait_fram(Config *const me, QEvt const *const e);
static QState Config_loading(Config *const me, QEvt const *const e);
static QState Config_idle(Config *const me, QEvt const *const e);
static QState Config_busy_saving(Config *const me, QEvt const *const e);

static void Config_PublishEntryChanged(ConfigID_T id);

static Config Config_inst;
QActive *const AO_Config = &Config_inst.super;

static const uint32_t VERSION        = 0U;
static ConfigDBEntry_T s_config_db[(CFG_ID_NUM_IDS > 0U) ? CFG_ID_NUM_IDS : 1U] = {0};

static_assert(
    (CFG_ID_NUM_IDS == 0U) || (sizeof(s_config_db) / sizeof(s_config_db[0]) == CFG_ID_NUM_IDS),
    "s_config_db bad length");

static ConfigNVMFile_T nvm_file = {0};
static bool nvm_file_is_valid   = false;

void Config_ctor(void)
{
    Config *const me = &Config_inst;
    QActive_ctor(&me->super, Q_STATE_CAST(&Config_initial));
}

uint32_t Config_Read_U32(ConfigID_T id)
{
    Q_ASSERT(id < CFG_ID_NUM_IDS);
    return s_config_db[id].val.u32_val;
}

uint32_t Config_Read_Default_U32(ConfigID_T id)
{
    Q_ASSERT(id < CFG_ID_NUM_IDS);
    return s_config_db[id].default_val.u32_val;
}

uint32_t Config_Read_Saved_U32(ConfigID_T id)
{
    Q_ASSERT(id < CFG_ID_NUM_IDS);
    return nvm_file.values[id].val.u32_val;
}

void Config_Write_U32(ConfigID_T id, uint32_t value)
{
    Q_ASSERT(id < CFG_ID_NUM_IDS);
    s_config_db[id].val.u32_val = value;
    Config_PublishEntryChanged(id);
}

int32_t Config_Read_I32(ConfigID_T id)
{
    Q_ASSERT(id < CFG_ID_NUM_IDS);
    return s_config_db[id].val.i32_val;
}

int32_t Config_Read_Default_I32(ConfigID_T id)
{
    Q_ASSERT(id < CFG_ID_NUM_IDS);
    return s_config_db[id].default_val.i32_val;
}

int32_t Config_Read_Saved_I32(ConfigID_T id)
{
    Q_ASSERT(id < CFG_ID_NUM_IDS);
    return nvm_file.values[id].val.i32_val;
}

void Config_Write_I32(ConfigID_T id, int32_t value)
{
    Q_ASSERT(id < CFG_ID_NUM_IDS);
    s_config_db[id].val.i32_val = value;
    Config_PublishEntryChanged(id);
}

float Config_Read_F32(ConfigID_T id)
{
    Q_ASSERT(id < CFG_ID_NUM_IDS);
    return s_config_db[id].val.f32_val;
}

float Config_Read_Default_F32(ConfigID_T id)
{
    Q_ASSERT(id < CFG_ID_NUM_IDS);
    return s_config_db[id].default_val.f32_val;
}

float Config_Read_Saved_F32(ConfigID_T id)
{
    Q_ASSERT(id < CFG_ID_NUM_IDS);
    return nvm_file.values[id].val.f32_val;
}

void Config_Write_F32(ConfigID_T id, float value)
{
    Q_ASSERT(id < CFG_ID_NUM_IDS);
    s_config_db[id].val.f32_val = value;
    Config_PublishEntryChanged(id);
}

bool Config_Read_Bool(ConfigID_T id)
{
    Q_ASSERT(id < CFG_ID_NUM_IDS);
    return s_config_db[id].val.bool_val;
}

bool Config_Read_Default_Bool(ConfigID_T id)
{
    Q_ASSERT(id < CFG_ID_NUM_IDS);
    return s_config_db[id].default_val.bool_val;
}

bool Config_Read_Saved_Bool(ConfigID_T id)
{
    Q_ASSERT(id < CFG_ID_NUM_IDS);
    return nvm_file.values[id].val.bool_val;
}

void Config_Write_Bool(ConfigID_T id, bool value)
{
    Q_ASSERT(id < CFG_ID_NUM_IDS);
    s_config_db[id].val.bool_val = value;
    Config_PublishEntryChanged(id);
}

void Config_SetDefault(ConfigID_T id)
{
    Q_ASSERT(id < CFG_ID_NUM_IDS);
    s_config_db[id].val = s_config_db[id].default_val;
    Config_PublishEntryChanged(id);
}

void Config_SetDefaultAll(void)
{
    for (unsigned i = 0; i < CFG_ID_NUM_IDS; i++)
    {
        s_config_db[i].val = s_config_db[i].default_val;
        Config_PublishEntryChanged((ConfigID_T) i);
    }
}

void Config_Save(void)
{
    static QEvt const save_evt = QEVT_INITIALIZER(POSTED_CONFIG_SAVE_TO_NVM_REQ_SIG);
    QACTIVE_POST(AO_Config, &save_evt, AO_Config);
}

uint32_t Config_Get_Num_Elements(void)
{
    return (uint32_t) CFG_ID_NUM_IDS;
}

uint32_t Config_Get_Version(void)
{
    return VERSION;
}

bool Config_IsNVMValid(void)
{
    return nvm_file_is_valid;
}

const char *Config_GetName(ConfigID_T id)
{
    Q_ASSERT(id < CFG_ID_NUM_IDS);
    return s_config_db[id].name;
}

ConfigID_T Config_GetByName(const char *name)
{
    for (unsigned i = 0; i < CFG_ID_NUM_IDS; i++)
    {
        if (strcmp(name, s_config_db[i].name) == 0)
        {
            return (ConfigID_T) i;
        }
    }

    return CFG_ID_INVALID;
}

ConfigValueType_T Config_GetType(ConfigID_T id)
{
    Q_ASSERT(id < CFG_ID_NUM_IDS);
    return s_config_db[id].val_type;
}

static QState Config_initial(Config *const me, void const *const par)
{
    Q_UNUSED_PAR(par);

    QActive_subscribe((QActive *) me, PUBSUB_FRAM_READY_SIG);
    nvm_file_is_valid = false;

    return Q_TRAN(&Config_wait_fram);
}

static QState Config_wait_fram(Config *const me, QEvt const *const e)
{
    QState status;

    switch (e->sig)
    {
        case Q_ENTRY_SIG: {
            status = Q_HANDLED();
            break;
        }

        case PUBSUB_FRAM_READY_SIG: {
            status = Q_TRAN(&Config_loading);
            break;
        }

        default: {
            status = Q_SUPER(&QHsm_top);
            break;
        }
    }

    return status;
}

static QState Config_loading(Config *const me, QEvt const *const e)
{
    QState status;

    switch (e->sig)
    {
        case Q_ENTRY_SIG: {
            FramReadReqEvent_T *read_req_evt = Q_NEW(FramReadReqEvent_T, POSTED_FRAM_READ_REQ_SIG);
            read_req_evt->requester          = &me->super;
            QACTIVE_POST(AO_Fram, &read_req_evt->super, &me->super);
            status = Q_HANDLED();
            break;
        }

        case POSTED_FRAM_READ_RESP_SIG: {
            const FramReadRespEvent_T *read_resp_evt = Q_EVT_CAST(FramReadRespEvent_T);

            if (read_resp_evt->read_status == FRAM_FILE_READ_OK)
            {
                memcpy(&nvm_file, read_resp_evt->file.data, sizeof(nvm_file));

                if ((nvm_file.version == VERSION) && (nvm_file.num_elements == CFG_ID_NUM_IDS))
                {
                    for (unsigned i = 0; i < CFG_ID_NUM_IDS; i++)
                    {
                        s_config_db[i].val = nvm_file.values[i].val;
                    }
                    nvm_file_is_valid = true;
                    LogCom_Printf("config loaded from FRAM");
                }
                else
                {
                    LogCom_Printf("config FRAM contents invalid or version-mismatched");
                }
            }
            else
            {
                LogCom_Printf("config FRAM empty, using defaults");
            }

            status = Q_TRAN(&Config_idle);
            break;
        }

        default: {
            status = Q_SUPER(&QHsm_top);
            break;
        }
    }

    return status;
}

static QState Config_idle(Config *const me, QEvt const *const e)
{
    QState status;

    switch (e->sig)
    {
        case Q_ENTRY_SIG: {
            status = Q_HANDLED();
            break;
        }

        case POSTED_CONFIG_SAVE_TO_NVM_REQ_SIG: {
            status = Q_TRAN(&Config_busy_saving);
            break;
        }

        default: {
            status = Q_SUPER(&QHsm_top);
            break;
        }
    }

    return status;
}

static QState Config_busy_saving(Config *const me, QEvt const *const e)
{
    QState status;

    switch (e->sig)
    {
        case Q_ENTRY_SIG: {
            FramWriteReqEvent_T *write_evt = Q_NEW(FramWriteReqEvent_T, POSTED_FRAM_WRITE_REQ_SIG);
            write_evt->requester           = &me->super;

            for (unsigned i = 0; i < CFG_ID_NUM_IDS; i++)
            {
                nvm_file.values[i].val = s_config_db[i].val;
            }

            nvm_file.version      = VERSION;
            nvm_file.num_elements = CFG_ID_NUM_IDS;

            memset(write_evt->file.data, 0, sizeof(write_evt->file.data));
            memcpy(write_evt->file.data, &nvm_file, sizeof(nvm_file));

            QACTIVE_POST(AO_Fram, &write_evt->super, &me->super);
            status = Q_HANDLED();
            break;
        }

        case POSTED_FRAM_WRITE_COMPLETE_SIG: {
            nvm_file_is_valid = true;
            status            = Q_TRAN(&Config_idle);
            break;
        }

        default: {
            status = Q_SUPER(&QHsm_top);
            break;
        }
    }

    return status;
}

static void Config_PublishEntryChanged(ConfigID_T id)
{
    ConfigEntryChangedEvent_T *event = Q_NEW(
        ConfigEntryChangedEvent_T, PUBSUB_CONFIG_ENTRY_CHANGED_SIG);
    event->id = id;
    QACTIVE_PUBLISH(&event->super, AO_Config);
}
