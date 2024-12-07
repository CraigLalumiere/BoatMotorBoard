#include "app_cli_commands.h"
#include "bsp.h"
#include "posted_signals.h"
#include "qpc.h"
#include "qsafe.h"
#include "app_cli.h"
#include "blinky.h"
#include "gpio.h"
#include "i2c_bus.h"
#include "bsp_manual.h"
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
static void on_cli_toggle_led(EmbeddedCli *cli, char *args, void *context);
void on_cli_digital_out_set(EmbeddedCli *cli, char *args, void *context);
void on_cli_digital_in_read(EmbeddedCli *cli, char *args, void *context);
// void on_cli_spi(EmbeddedCli *cli, char *args, void *context);
// void on_cli_i2c(EmbeddedCli *cli, char *args, void *context);


// static SPI_Bus_ID_T Get_SPI_Bus_ID(unsigned long arg_bus_id);
static GPIO_Port_ID_T Get_SPI_Bus_Port(char arg_port);
// static I2C_Bus_ID_T Get_I2C_Bus_ID(unsigned long arg_bus_id);

static CliCommandBinding cli_cmd_list[] = {
    (CliCommandBinding) {
        "toggle-led",               // command name (spaces are not allowed)
        "Activates the green blinky LED", // Optional help for a command (NULL for no help)
        false,                      // flag whether to tokenize arguments
        NULL,                       // optional pointer to any application context
        on_cli_toggle_led           // binding function
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

    // (CliCommandBinding) {
    //     "i2c",                                 // command name (spaces are not allowed)
    //     "Perform I2C write or read operation", // Optional help for
    //                                            // a command
    //     true,                                  // flag whether to tokenize arguments
    //     NULL,                                  // optional pointer to any application context
    //     on_cli_i2c                             // binding function
    // },
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

#define HELP_FULL_DIGITAL_OUT_SET \
    "\r\n\
 Usage: digital-out-set PORT PIN VALUE\r\n\
 \r\n\
 PORT   Character A through K\r\n\
 PIN    Number 0 through 15\r\n\
 Value  0 (for low) or 1 (for high)\r\n"

void on_cli_digital_out_set(EmbeddedCli *cli, char *args, void *context)
{
    char arg_port;
    unsigned long arg_pin;
    unsigned long arg_value;
    char *arg_end;
    bool is_invalid_arg = false;

    if (embeddedCliGetTokenCount(args) != 3)
    {
        embeddedCliPrint(cli, HELP_FULL_DIGITAL_OUT_SET);
        return;
    }

    const char *arg1 = embeddedCliGetToken(args, 1);
    const char *arg2 = embeddedCliGetToken(args, 2);
    const char *arg3 = embeddedCliGetToken(args, 3);

    arg_port  = *arg1;
    arg_pin   = strtoul(arg2, &arg_end, 10);
    arg_value = strtoul(arg3, &arg_end, 10);

    if (strlen(arg1) > 1 || arg_port < 'A' || arg_port > 'K')
    {
        embeddedCliPrint(cli, " PORT must be a character between A and K\r\n");
        is_invalid_arg = true;
    }

    if (arg_pin > 15)
    {
        embeddedCliPrint(cli, " PIN must be a number between 0 and 15\r\n");
        is_invalid_arg = true;
    }

    if (arg_value > 1)
    {
        embeddedCliPrint(cli, " VALUE must be 0 (for low) or 1 (for high)\r\n");
        is_invalid_arg = true;
    }

    if (is_invalid_arg)
    {
        embeddedCliPrint(cli, HELP_FULL_DIGITAL_OUT_SET);
        return;
    }

    BSP_Manual_Config_and_Set_Digital_Output(arg_port, (uint8_t) arg_pin, (arg_value > 0));
}

#define HELP_FULL_DIGITAL_IN_READ \
    "\r\n\
 Usage: digital-in-read PORT PIN\r\n\
 \r\n\
 PORT   Character A through K\r\n\
 PIN    Number 0 through 15\r\n"

void on_cli_digital_in_read(EmbeddedCli *cli, char *args, void *context)
{
    char arg_port;
    unsigned long arg_pin;
    char *arg_end;
    bool is_invalid_arg = false;

    if (embeddedCliGetTokenCount(args) != 2)
    {
        embeddedCliPrint(cli, HELP_FULL_DIGITAL_IN_READ);
        return;
    }

    const char *arg1 = embeddedCliGetToken(args, 1);
    const char *arg2 = embeddedCliGetToken(args, 2);

    arg_port = *arg1;
    arg_pin  = strtoul(arg2, &arg_end, 10);

    if (strlen(arg1) > 1 || arg_port < 'A' || arg_port > 'K')
    {
        embeddedCliPrint(cli, " PORT must be a character between A and K\r\n");
        is_invalid_arg = true;
    }

    if (arg_pin > 15)
    {
        embeddedCliPrint(cli, " PIN must be a number between 0 and 15\r\n");
        is_invalid_arg = true;
    }

    if (is_invalid_arg)
    {
        embeddedCliPrint(cli, HELP_FULL_DIGITAL_IN_READ);
        return;
    }

    bool is_set = BSP_Manual_Config_and_Read_Digital_Input(arg_port, (uint8_t) arg_pin);
    embeddedCliPrint(cli, is_set ? "\r\n1\r\n" : "\r\n0\r\n");
}
