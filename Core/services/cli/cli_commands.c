#include "cli_commands.h"
#include "cli_manual_commands.h"
#include "bsp.h"
#include "posted_signals.h"
#include "qpc.h"
#include "qsafe.h"
#include "cli.h"
#include "blinky.h"
#include "interfaces/gpio.h"
#include "interfaces/i2c_bus.h"
#include "bsp_manual.h"
// #include "services/config.h"
// #include "services/director.h"
#include <ctype.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define DIMENSION_OF(a) ((sizeof(a)) / (sizeof(a[0])))
#define CLI_PRINT_BUFFER_SIZE 128

Q_DEFINE_THIS_MODULE("app_cli_commands")

// Command Functions
static void on_cli_toggle_led(EmbeddedCli *cli, char *args, void *context);
static void on_fault(EmbeddedCli *cli, char *args, void *context);
void on_cli_digital_out_set(EmbeddedCli *cli, char *args, void *context);
void on_cli_digital_in_read(EmbeddedCli *cli, char *args, void *context);
// void on_cli_spi(EmbeddedCli *cli, char *args, void *context);
void on_cli_i2c(EmbeddedCli *cli, char *args, void *context);

static CliCommandBinding cli_cmd_list[] = {
    (CliCommandBinding){
        "toggle-led",                     // command name (spaces are not allowed)
        "Activates the green blinky LED", // Optional help for a command (NULL for no help)
        false,                            // flag whether to tokenize arguments
        NULL,                             // optional pointer to any application context
        on_cli_toggle_led                 // binding function
    },

    (CliCommandBinding){
        "fault",                       // command name (spaces are not allowed)
        "print the active fault info", // Optional help for a command
        false,                         // flag whether to tokenize arguments
        NULL,                          // optional pointer to any application context
        on_fault                       // binding function
    },

    (CliCommandBinding){
        "digital-out-set",                                                     // command name (spaces are not allowed)
        "Config a port and pin as a digital output, and set it to the value.", // Optional help
                                                                               // for a command
        true,                                                                  // flag whether to tokenize arguments
        NULL,                                                                  // optional pointer to any application context
        on_cli_digital_out_set                                                 // binding function
    },

    (CliCommandBinding){
        "digital-in-read",                                              // command name (spaces are not allowed)
        "Config a port and pin as a digital input, and read its value", // Optional help for
                                                                        // a command
        true,                                                           // flag whether to tokenize arguments
        NULL,                                                           // optional pointer to any application context
        on_cli_digital_in_read                                          // binding function
    },

    (CliCommandBinding){
        "i2c",                                 // command name (spaces are not allowed)
        "Perform I2C write or read operation", // Optional help for
                                               // a command
        true,                                  // flag whether to tokenize arguments
        NULL,                                  // optional pointer to any application context
        on_cli_i2c                             // binding function
    },
};

void AppCLI_AddCommandsToCLI(EmbeddedCli *cli)
{
    for (unsigned i = 0; i < DIMENSION_OF(cli_cmd_list); i++)
    {
        embeddedCliAddBinding(cli, cli_cmd_list[i]);
    }
}

static void on_cli_toggle_led(EmbeddedCli *cli, char *args, void *context)
{
    // statically allocated and const event to post to the Blinky active object
    static QEvt const ToggleLEDEvent = QEVT_INITIALIZER(POSTED_BLINKY_TOGGLE_USER_LED);

    // send (post) the event to the Blinky active object
    QACTIVE_POST(AO_Blinky, &ToggleLEDEvent, AO_AppCLI);
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

        snprintf(print_buffer, sizeof(print_buffer), "ID: %d", (int)this_fault.id);
        embeddedCliPrint(cli, print_buffer);
        snprintf(
            print_buffer,
            sizeof(print_buffer),
            "Code: %d",
            (int)Fault_Manager_Get_Code(this_fault.id));
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