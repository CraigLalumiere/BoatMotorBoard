#include "bsp.h"
// #include "services/pc_com/pc_com.h"
// #include "services/reset.h"

#define EMBEDDED_CLI_IMPL
#include "embedded_cli.h"

#include "app_cli.h"
#include "app_cli_commands.h"
#include "private_signal_ranges.h"
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>

#define CLI_BUFFER_SIZE 2048

Q_DEFINE_THIS_MODULE("app_cli")

enum AppCLI_Signals
{
    APP_CLI_DATA_AVAILABLE_SIG = PRIVATE_SIGNAL_APP_CLI_START,
    APP_CLI_PROCESS_TIMEOUT_SIG
};

typedef struct
{
    QActive super; // inherit QActive (inheritance in C)
    QTimeEvt processTimeEvt;
    const Serial_IO_T *serial_io_interface;
    EmbeddedCli *cli;
    CLI_UINT cliBuffer[BYTES_TO_CLI_UINTS(CLI_BUFFER_SIZE)];
} AppCLI;

// state handler functions
static QState AppCLI_initial(AppCLI *const me, void const *const par);
static QState AppCLI_active(AppCLI *const me, QEvt const *const e);

static AppCLI AppCLI_inst;
QActive *const AO_AppCLI = &AppCLI_inst.super;

// Helper functions
static void DataReadyCB(void *cb_data);
// static void cli_printf(const char *format, ...);
static void CLI_WriteChar(EmbeddedCli *embeddedCli, char c);

void AppCLI_ctor(const Serial_IO_T *const serial_io_interface)
{
    AppCLI_inst.serial_io_interface = serial_io_interface;

    // CLI config
    EmbeddedCliConfig *config = embeddedCliDefaultConfig();

    // use static allocated buffer for CLI storage
    config->cliBuffer       = AppCLI_inst.cliBuffer;
    config->cliBufferSize   = CLI_BUFFER_SIZE;
    config->maxBindingCount = 60;

    // create an instance of CLI
    AppCLI_inst.cli = embeddedCliNew(config);

    // store char function
    AppCLI_inst.cli->writeChar = CLI_WriteChar;

    // add commands from command table
    AppCLI_AddCommandsToCLI(AppCLI_inst.cli);

    // call QP object constructors
    AppCLI *const me = &AppCLI_inst;
    QActive_ctor(&me->super, Q_STATE_CAST(&AppCLI_initial));
    QTimeEvt_ctorX(&me->processTimeEvt, &me->super, APP_CLI_PROCESS_TIMEOUT_SIG, 0U);
}

/////////////////////////
// HSM Definition
/////////////////////////
static QState AppCLI_initial(AppCLI *const me, void const *const par)
{
    Q_UNUSED_PAR(par);

#ifdef Q_SPY
    // Add object and all state functions to the QSPY dictionaries
    QS_OBJ_DICTIONARY(me);
    QS_FUN_DICTIONARY(&AppCLI_active);
#endif

    // QActive_subscribe((QActive *) me, PUBSUB_DMF_DETECT_SUCCESS_SIG);

    // Process CLI  every 20ms
    QTimeEvt_armX(
        &me->processTimeEvt, 20U * BSP_TICKS_PER_SEC / 1000U, 20U * BSP_TICKS_PER_SEC / 1000U);

    // Register my callback with the SerialIO interface to be called when new data is available
    AppCLI_inst.serial_io_interface->register_cb_func(DataReadyCB, &AppCLI_inst);

    return Q_TRAN(&AppCLI_active);
}

static QState AppCLI_active(AppCLI *const me, QEvt const *const e)
{
    QState status;
    switch (e->sig)
    {
        case Q_ENTRY_SIG: {
            status = Q_HANDLED();
            break;
        }
        case APP_CLI_DATA_AVAILABLE_SIG: {
            uint8_t rx_byte;
            while (AppCLI_inst.serial_io_interface->rx_func(&rx_byte, 1) == 1)
            {
                embeddedCliReceiveChar(me->cli, (char) rx_byte);
            }
            status = Q_HANDLED();
            break;
        }
        case APP_CLI_PROCESS_TIMEOUT_SIG: {
            embeddedCliProcess(me->cli);
            status = Q_HANDLED();
            break;
        }

        case POSTED_APP_CLI_PRINT_SIG: {
            const PrintEvent_T *printEvent = Q_EVT_CAST(PrintEvent_T);
            embeddedCliPrint(me->cli, printEvent->msg);
            embeddedCliPrint(me->cli, "\r\n");
            status = Q_HANDLED();
            break;
        }

        default: {
            status = Q_SUPER(&QHsm_top);
            break;
        }
    }
    return status;
}


////////////////////////////////
// Helper Function Definitions
////////////////////////////////

static void DataReadyCB(void *cb_data)
{
    // statically allocated and const event to post to our CLI active object
    static QEvt const DataReadyEvent = QEVT_INITIALIZER(APP_CLI_DATA_AVAILABLE_SIG);

    // callback data was our 'me' pointer
    QActive *me = (QActive *) cb_data;

    // send (post) the event to our CLI active object
    QACTIVE_POST(me, &DataReadyEvent, NULL);
}

static void CLI_WriteChar(EmbeddedCli *embeddedCli, char c)
{
    AppCLI_inst.serial_io_interface->tx_func((uint8_t *) &c, 1);
}


// Function to encapsulate the 'embeddedCliPrint()' call with print formatting arguments (act like
// printf(), but keeps cursor at correct location). The 'embeddedCliPrint()' function does already
// add a linebreak ('\r\n') to the end of the print statement, so no need to add it yourself.
// void cli_printf(const char *format, ...)
// {
//     // Create a buffer to store the formatted string
//     char buffer[CLI_PRINT_BUFFER_SIZE];

//     // Format the string using snprintf
//     va_list args;
//     va_start(args, format);
//     int length = vsnprintf(buffer, sizeof(buffer), format, args);
//     va_end(args);

//     // Check if string fitted in buffer else print error to stderr
//     if (length < 0)
//     {
//         fprintf(stderr, "Error formatting the string\r\n");
//         return;
//     }

//     // Call embeddedCliPrint with the formatted string
//     embeddedCliPrint(AppCLI_inst.cli, buffer);
// }

int AppCLI_PostPrintf(const char *sFormat, ...)
{
    int r;
    va_list ParamList;

    PrintEvent_T *printEvent = Q_NEW(PrintEvent_T, POSTED_APP_CLI_PRINT_SIG);

    va_start(ParamList, sFormat);
    r = vsnprintf(printEvent->msg,PRINT_EVENT_MAX_MSG_LENGTH, sFormat, ParamList);
    va_end(ParamList);

    printEvent->msg[PRINT_EVENT_MAX_MSG_LENGTH - 1] = 0; // guarantee null terminator
    QACTIVE_POST(AO_AppCLI, &printEvent->super, NULL);

    return r;
}
