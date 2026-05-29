#include "cli_commands.h"
#include "blinky.h"
#include "bsp.h"
#include "bsp_manual.h"
#include "cli_manual_commands.h"
#include "config.h"
#include "interfaces/gpio.h"
#include "interfaces/i2c_bus.h"
#include "posted_signals.h"
#include "reset.h"
#include "qpc.h"
#include "qsafe.h"
// #include "services/config.h"
// #include "services/director.h"
#include <ctype.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define DIMENSION_OF(a)       ((sizeof(a)) / (sizeof(a[0])))
#define CLI_PRINT_BUFFER_SIZE 128

Q_DEFINE_THIS_MODULE("app_cli_commands")

// Command Functions
static void on_fault(EmbeddedCli *cli, char *args, void *context);
void on_cli_digital_out_set(EmbeddedCli *cli, char *args, void *context);
void on_cli_digital_in_read(EmbeddedCli *cli, char *args, void *context);
// void on_cli_spi(EmbeddedCli *cli, char *args, void *context);
void on_cli_i2c(EmbeddedCli *cli, char *args, void *context);
static void on_cli_config_list(EmbeddedCli *cli, char *args, void *context);
static void on_cli_config_read(EmbeddedCli *cli, char *args, void *context);
static void on_cli_config_set(EmbeddedCli *cli, char *args, void *context);
static void on_cli_config_save(EmbeddedCli *cli, char *args, void *context);
static void on_bootloader(EmbeddedCli *cli, char *args, void *context);
static bool is_numeric(const char *s);
static bool is_positive_numeric(const char *s);
static void lowercase(const char *src, char *dst, unsigned max_len);

static CliCommandBinding cli_cmd_list[] = {

    (CliCommandBinding) {
        "fault",                       // command name (spaces are not allowed)
        "print the active fault info", // Optional help for a command
        false,                         // flag whether to tokenize arguments
        NULL,                          // optional pointer to any application context
        on_fault                       // binding function
    },

    (CliCommandBinding) {
        "digital-out-set", // command name (spaces are not allowed)
        "Config a port and pin as a digital output, and set it to the value.", // Optional help
                                                                               // for a command
        true,                  // flag whether to tokenize arguments
        NULL,                  // optional pointer to any application context
        on_cli_digital_out_set // binding function
    },

    (CliCommandBinding) {
        "digital-in-read", // command name (spaces are not allowed)
        "Config a port and pin as a digital input, and read its value", // Optional help for
                                                                        // a command
        true,                  // flag whether to tokenize arguments
        NULL,                  // optional pointer to any application context
        on_cli_digital_in_read // binding function
    },

    (CliCommandBinding) {
        "i2c",                                 // command name (spaces are not allowed)
        "Perform I2C write or read operation", // Optional help for
                                               // a command
        true,                                  // flag whether to tokenize arguments
        NULL,                                  // optional pointer to any application context
        on_cli_i2c                             // binding function
    },

    (CliCommandBinding) {
        "config-list",
        "List configurable variables",
        false,
        NULL,
        on_cli_config_list,
    },

    (CliCommandBinding) {
        "config-read",
        "Read a config variable by name",
        true,
        NULL,
        on_cli_config_read,
    },

    (CliCommandBinding) {
        "config-set",
        "Set a config variable by name",
        true,
        NULL,
        on_cli_config_set,
    },

    (CliCommandBinding) {
        "config-save",
        "Save current config values to FRAM",
        false,
        NULL,
        on_cli_config_save,
    },

    (CliCommandBinding) {
        "bootloader",
        "Enter STM32 USB DFU bootloader",
        false,
        NULL,
        on_bootloader,
    },
};

void CLI_AddCommands(EmbeddedCli *cli)
{
    for (unsigned i = 0; i < DIMENSION_OF(cli_cmd_list); i++)
    {
        embeddedCliAddBinding(cli, cli_cmd_list[i]);
    }
}

static void on_bootloader(EmbeddedCli *cli, char *args, void *context)
{
    (void) args;
    (void) context;

    embeddedCliPrint(cli, "Bootloader command received, rebooting into USB DFU...");
    Reset_RequestBootloader();
}

static void on_fault(EmbeddedCli *cli, char *args, void *context)
{
    char print_buffer[CLI_PRINT_BUFFER_SIZE] = {0};

    Active_Fault_T *active_faults = Fault_Manager_Get_Active_Fault_List();
    if (active_faults[0].id == FAULT_ID_NONE)
    {
        embeddedCliPrint(cli, "No faults recorded");
        return;
    }

    for (uint8_t i = 0; i < FAULT_MANAGER_BUFFER_LENGTH; i++)
    {
        Active_Fault_T this_fault = active_faults[i];
        if (this_fault.id == FAULT_ID_NONE)
        {
            break;
        }

        snprintf(print_buffer, sizeof(print_buffer), "ID: %d", (int) this_fault.id);
        embeddedCliPrint(cli, print_buffer);
        snprintf(
            print_buffer,
            sizeof(print_buffer),
            "Code: %d",
            (int) Fault_Manager_Get_Code(this_fault.id));
        embeddedCliPrint(cli, print_buffer);
        snprintf(
            print_buffer,
            sizeof(print_buffer),
            "Description: %s",
            Fault_Manager_Get_Description(this_fault.id));
        embeddedCliPrint(cli, print_buffer);
        snprintf(print_buffer, sizeof(print_buffer), "Message: %s\r\n", this_fault.msg);
        embeddedCliPrint(cli, print_buffer);
    }
}

GPIO_Port_ID_T Get_SPI_Bus_Port(char arg_port)
{
    switch (arg_port)
    {
        case '0':
            return GPIO_PORT_NONE_ID; // used if there is no chip select
        case 'A':
            return GPIO_PORT_A_ID;
        case 'B':
            return GPIO_PORT_B_ID;
        case 'C':
            return GPIO_PORT_C_ID;
        case 'F':
            return GPIO_PORT_F_ID;
        default:
            Q_ASSERT(false);
            return GPIO_PORT_NONE_ID;
    }
}

static void on_cli_config_list(EmbeddedCli *cli, char *args, void *context)
{
    (void) args;
    (void) context;

    char print_buffer[CLI_PRINT_BUFFER_SIZE] = {0};
    embeddedCliPrint(cli, "Name                 Current    Default      Saved   Type");
    embeddedCliPrint(cli, "--------------------------------------------------------------");

    for (unsigned i = 0; i < CFG_ID_NUM_IDS; i++)
    {
        ConfigID_T id               = (ConfigID_T) i;
        const char *name            = Config_GetName(id);
        ConfigValueType_T valueType = Config_GetType(id);

        switch (valueType)
        {
            case CFG_VAL_TYPE_U32: {
                if (Config_IsNVMValid())
                {
                    snprintf(
                        print_buffer,
                        sizeof(print_buffer),
                        "%-20s %8lu %10lu %10lu %6s",
                        name,
                        (unsigned long) Config_Read_U32(id),
                        (unsigned long) Config_Read_Default_U32(id),
                        (unsigned long) Config_Read_Saved_U32(id),
                        "u32");
                }
                else
                {
                    snprintf(
                        print_buffer,
                        sizeof(print_buffer),
                        "%-20s %8lu %10lu %10s %6s",
                        name,
                        (unsigned long) Config_Read_U32(id),
                        (unsigned long) Config_Read_Default_U32(id),
                        "--",
                        "u32");
                }
                break;
            }

            case CFG_VAL_TYPE_I32: {
                if (Config_IsNVMValid())
                {
                    snprintf(
                        print_buffer,
                        sizeof(print_buffer),
                        "%-20s %8ld %10ld %10ld %6s",
                        name,
                        (long) Config_Read_I32(id),
                        (long) Config_Read_Default_I32(id),
                        (long) Config_Read_Saved_I32(id),
                        "i32");
                }
                else
                {
                    snprintf(
                        print_buffer,
                        sizeof(print_buffer),
                        "%-20s %8ld %10ld %10s %6s",
                        name,
                        (long) Config_Read_I32(id),
                        (long) Config_Read_Default_I32(id),
                        "--",
                        "i32");
                }
                break;
            }

            case CFG_VAL_TYPE_F32: {
                if (Config_IsNVMValid())
                {
                    snprintf(
                        print_buffer,
                        sizeof(print_buffer),
                        "%-20s %8.2f %10.2f %10.2f %6s",
                        name,
                        Config_Read_F32(id),
                        Config_Read_Default_F32(id),
                        Config_Read_Saved_F32(id),
                        "f32");
                }
                else
                {
                    snprintf(
                        print_buffer,
                        sizeof(print_buffer),
                        "%-20s %8.2f %10.2f %10s %6s",
                        name,
                        Config_Read_F32(id),
                        Config_Read_Default_F32(id),
                        "--",
                        "f32");
                }
                break;
            }

            case CFG_VAL_TYPE_BOOL: {
                snprintf(
                    print_buffer,
                    sizeof(print_buffer),
                    "%-20s %8s %10s %10s %6s",
                    name,
                    Config_Read_Bool(id) ? "true" : "false",
                    Config_Read_Default_Bool(id) ? "true" : "false",
                    Config_IsNVMValid() ? (Config_Read_Saved_Bool(id) ? "true" : "false") : "--",
                    "bool");
                break;
            }

            default: {
                snprintf(print_buffer, sizeof(print_buffer), "%s", "Unsupported config type");
                break;
            }
        }

        embeddedCliPrint(cli, print_buffer);
    }
}

#define HELP_CONFIG_READ \
    "\r\n\
Usage: config-read NAME\r\n\
\r\n\
NAME  config variable name -- run config-list to see options\r\n\
"
static void on_cli_config_read(EmbeddedCli *cli, char *args, void *context)
{
    (void) context;

    char print_buffer[CLI_PRINT_BUFFER_SIZE] = {0};

    if (embeddedCliGetTokenCount(args) != 1)
    {
        embeddedCliPrint(cli, HELP_CONFIG_READ);
        return;
    }

    const char *arg_name = embeddedCliGetToken(args, 1);
    ConfigID_T id        = Config_GetByName(arg_name);
    if (id == CFG_ID_INVALID)
    {
        embeddedCliPrint(cli, "Invalid variable name");
        return;
    }

    switch (Config_GetType(id))
    {
        case CFG_VAL_TYPE_U32:
            snprintf(print_buffer, sizeof(print_buffer), "%lu", (unsigned long) Config_Read_U32(id));
            break;
        case CFG_VAL_TYPE_I32:
            snprintf(print_buffer, sizeof(print_buffer), "%ld", (long) Config_Read_I32(id));
            break;
        case CFG_VAL_TYPE_F32:
            snprintf(print_buffer, sizeof(print_buffer), "%.3f", Config_Read_F32(id));
            break;
        case CFG_VAL_TYPE_BOOL:
            snprintf(
                print_buffer,
                sizeof(print_buffer),
                "%s",
                Config_Read_Bool(id) ? "true" : "false");
            break;
        default:
            snprintf(print_buffer, sizeof(print_buffer), "%s", "Unsupported config type");
            break;
    }

    embeddedCliPrint(cli, print_buffer);
}

#define HELP_CONFIG_SET \
    "\r\n\
Usage: config-set NAME VALUE\r\n\
\r\n\
NAME   config variable name\r\n\
VALUE  new value in the variable's type\r\n\
\r\n\
Run config-list to see valid names and types.\r\n\
"
static void on_cli_config_set(EmbeddedCli *cli, char *args, void *context)
{
    (void) context;

    char *arg_end;
    char print_buffer[CLI_PRINT_BUFFER_SIZE] = {0};

    if (embeddedCliGetTokenCount(args) != 2)
    {
        embeddedCliPrint(cli, HELP_CONFIG_SET);
        return;
    }

    const char *arg_name  = embeddedCliGetToken(args, 1);
    const char *arg_value = embeddedCliGetToken(args, 2);
    ConfigID_T id         = Config_GetByName(arg_name);

    if (id == CFG_ID_INVALID)
    {
        embeddedCliPrint(cli, "Invalid variable name");
        return;
    }

    switch (Config_GetType(id))
    {
        case CFG_VAL_TYPE_U32: {
            if (!is_positive_numeric(arg_value))
            {
                embeddedCliPrint(cli, "Invalid value, expected a positive integer");
                return;
            }

            uint32_t old_val = Config_Read_U32(id);
            uint32_t new_val = (uint32_t) strtoul(arg_value, &arg_end, 10);
            (void) arg_end;
            Config_Write_U32(id, new_val);

            snprintf(
                print_buffer,
                sizeof(print_buffer),
                "Old: %lu  New: %lu  Run config-save to persist",
                (unsigned long) old_val,
                (unsigned long) Config_Read_U32(id));
            break;
        }

        case CFG_VAL_TYPE_I32: {
            if (!is_numeric(arg_value))
            {
                embeddedCliPrint(cli, "Invalid value, expected an integer");
                return;
            }

            int32_t old_val = Config_Read_I32(id);
            int32_t new_val = (int32_t) strtol(arg_value, &arg_end, 10);
            (void) arg_end;
            Config_Write_I32(id, new_val);

            snprintf(
                print_buffer,
                sizeof(print_buffer),
                "Old: %ld  New: %ld  Run config-save to persist",
                (long) old_val,
                (long) Config_Read_I32(id));
            break;
        }

        case CFG_VAL_TYPE_F32: {
            if (!is_numeric(arg_value))
            {
                embeddedCliPrint(cli, "Invalid value, expected a number");
                return;
            }

            float old_val = Config_Read_F32(id);
            float new_val = strtof(arg_value, &arg_end);
            (void) arg_end;
            Config_Write_F32(id, new_val);

            snprintf(
                print_buffer,
                sizeof(print_buffer),
                "Old: %.3f  New: %.3f  Run config-save to persist",
                old_val,
                Config_Read_F32(id));
            break;
        }

        case CFG_VAL_TYPE_BOOL: {
            char lower_arg_value[8] = {0};
            bool old_val            = Config_Read_Bool(id);
            bool new_val;

            lowercase(arg_value, lower_arg_value, sizeof(lower_arg_value));

            if ((strcmp(lower_arg_value, "true") == 0) || (strcmp(lower_arg_value, "1") == 0))
            {
                new_val = true;
            }
            else if (
                (strcmp(lower_arg_value, "false") == 0) || (strcmp(lower_arg_value, "0") == 0))
            {
                new_val = false;
            }
            else
            {
                embeddedCliPrint(cli, "Invalid boolean value, use true/false/1/0");
                return;
            }

            Config_Write_Bool(id, new_val);
            snprintf(
                print_buffer,
                sizeof(print_buffer),
                "Old: %s  New: %s  Run config-save to persist",
                old_val ? "true" : "false",
                Config_Read_Bool(id) ? "true" : "false");
            break;
        }

        default:
            embeddedCliPrint(cli, "Unsupported config type");
            return;
    }

    embeddedCliPrint(cli, print_buffer);
}

static void on_cli_config_save(EmbeddedCli *cli, char *args, void *context)
{
    (void) cli;
    (void) args;
    (void) context;

    Config_Save();
    embeddedCliPrint(cli, "Config save requested");
}

static bool is_numeric(const char *s)
{
    if ((s == NULL) || (*s == '\0'))
    {
        return false;
    }

    if ((*s == '-') || (*s == '+'))
    {
        s++;
    }

    bool saw_digit = false;
    bool saw_dot   = false;

    while (*s != '\0')
    {
        if (isdigit((unsigned char) *s))
        {
            saw_digit = true;
        }
        else if ((*s == '.') && !saw_dot)
        {
            saw_dot = true;
        }
        else
        {
            return false;
        }

        s++;
    }

    return saw_digit;
}

static bool is_positive_numeric(const char *s)
{
    if ((s == NULL) || (*s == '\0'))
    {
        return false;
    }

    while (*s != '\0')
    {
        if (!isdigit((unsigned char) *s))
        {
            return false;
        }
        s++;
    }

    return true;
}

static void lowercase(const char *src, char *dst, unsigned max_len)
{
    unsigned i;

    if (max_len == 0U)
    {
        return;
    }

    for (i = 0U; (i + 1U) < max_len; i++)
    {
        if (src[i] == '\0')
        {
            break;
        }
        dst[i] = (char) tolower((unsigned char) src[i]);
    }

    dst[i] = '\0';
}
