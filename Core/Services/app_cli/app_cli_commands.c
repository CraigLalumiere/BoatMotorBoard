#include "app_cli_commands.h"
#include "bsp.h"
#include "posted_signals.h"
#include "qpc.h"
#include "qsafe.h"
#include "app_cli.h"
#include "blinky.h"
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

static CliCommandBinding cli_cmd_list[] = {
    (CliCommandBinding) {
        "toggle-led",               // command name (spaces are not allowed)
        "Toggles the red user LED", // Optional help for a command (NULL for no help)
        false,                      // flag whether to tokenize arguments
        NULL,                       // optional pointer to any application context
        on_cli_toggle_led           // binding function
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
