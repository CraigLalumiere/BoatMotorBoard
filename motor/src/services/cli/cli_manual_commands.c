#include "cli_manual_commands.h"
#include "bsp_manual.h"
#include "interfaces/i2c_bus.h"
#include "qsafe.h"
#include "stdlib.h"
#include <stdio.h>
#include <string.h>

Q_DEFINE_THIS_MODULE("cli_manual_cmds");

// static SPI_Bus_ID_T Get_SPI_Bus_ID(unsigned long arg_bus_id);
// static GPIO_Port_ID_T Get_SPI_Bus_Port(char arg_port);
static I2C_Bus_ID_T Get_I2C_Bus_ID(unsigned long arg_bus_id);

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

#define MANUAL_I2C_XFER_BUFFER_LEN 20
static uint8_t s_i2c_write_buffer[MANUAL_I2C_XFER_BUFFER_LEN];
static uint8_t s_i2c_read_buffer[MANUAL_I2C_XFER_BUFFER_LEN];
static uint16_t s_i2c_data_len;

static void I2C_Read_Complete_CB(void *cb_data)
{
    EmbeddedCli *cli = (EmbeddedCli *) cb_data;

    embeddedCliPrint(cli, "RX bytes:");
    for (uint16_t i = 0; i < s_i2c_data_len; i++)
    {
        char byte_str[4];
        uint8_t nibble0 = s_i2c_read_buffer[i] & 0xF;
        uint8_t nibble1 = (s_i2c_read_buffer[i] & 0xF0) >> 4;

        // print most significant nibble first
        byte_str[0] = (char) nibble1 + ((nibble1 > 9) ? ('A' - 10) : '0');
        byte_str[1] = (char) nibble0 + ((nibble0 > 9) ? ('A' - 10) : '0');
        byte_str[2] = ' ';
        byte_str[3] = '\0';
        embeddedCliPrint(cli, byte_str);
    }
}

static void I2C_Operation_Error_CB(void *cb_data)
{
    EmbeddedCli *cli = (EmbeddedCli *) cb_data;
    embeddedCliPrint(cli, " A I2C Bus error occurred\r\n");
}

#define HELP_FULL_I2C \
    "\r\n\
 Usage: i2c [write|read] BUS_ID ADDRESS [BYTES]\r\n\
 \r\n\
 write    Write bytes, no rx\r\n\
 read     Print rx bytes\r\n\
 BUS_ID   Number 1-6\r\n\
 ADDRESS  Hex Number 01-7F\r\n\
 BYTES    Optional list of 1-20 bytes to transmit, in hex notation (e.g. A5 EE 4C)\r\n\
"

#define MANUAL_I2C_MIN_NUM_ARGS_BEFORE_TX 3
void on_cli_i2c(EmbeddedCli *cli, char *args, void *context)
{
    unsigned long arg_bus_id;
    const char *arg_i2c_type;
    unsigned long arg_address;
    char *arg_end;
    bool is_invalid_arg = false;
    bool is_i2c_read    = false;

    if (embeddedCliGetTokenCount(args) < MANUAL_I2C_MIN_NUM_ARGS_BEFORE_TX + 1)
    {
        embeddedCliPrint(cli, HELP_FULL_I2C);
        return;
    }
    else if (
        embeddedCliGetTokenCount(args) >
        MANUAL_I2C_MIN_NUM_ARGS_BEFORE_TX + MANUAL_I2C_XFER_BUFFER_LEN)
    {
        embeddedCliPrint(cli, " Number of TX bytes limited to 20\r\n");
        return;
    }

    const char *arg1 = embeddedCliGetToken(args, 1);
    const char *arg2 = embeddedCliGetToken(args, 2);
    const char *arg3 = embeddedCliGetToken(args, 3);

    arg_i2c_type = arg1;
    arg_bus_id   = strtoul(arg2, &arg_end, 10);
    arg_address  = strtoul(arg3, &arg_end, 16);

    if (strcmp(arg_i2c_type, "write") == 0)
    {
        is_i2c_read = false;
    }
    else if (strcmp(arg_i2c_type, "read") == 0)
    {
        is_i2c_read = true;
    }
    else
    {
        embeddedCliPrint(cli, " 1st argument must either be 'write' or 'read'\r\n");
        is_invalid_arg = true;
    }

    if (arg_bus_id < 1 || arg_bus_id > 6)
    {
        embeddedCliPrint(cli, " BUS_ID must be a number between 1 and 6\r\n");
        is_invalid_arg = true;
    }

    if (arg_address < 1 || arg_address > 127)
    {
        embeddedCliPrint(cli, " ADDRESS must be a hex number between 01 and 7F\r\n");
        is_invalid_arg = true;
    }

    s_i2c_data_len = embeddedCliGetTokenCount(args) - MANUAL_I2C_MIN_NUM_ARGS_BEFORE_TX;
    for (uint16_t i = 0; i < s_i2c_data_len; i++)
    {
        const char *arg     = embeddedCliGetToken(args, i + MANUAL_I2C_MIN_NUM_ARGS_BEFORE_TX + 1);
        unsigned long value = strtoul(arg, &arg_end, 16);
        if (value > 0xFF)
        {
            embeddedCliPrint(cli, " Invalid TX Byte\r\n");
            is_invalid_arg = true;
            break;
        }
        s_i2c_write_buffer[i] = value;
    }

    if (is_invalid_arg)
    {
        embeddedCliPrint(cli, HELP_FULL_I2C);
        return;
    }

    I2C_Bus_ID_T bus_id = Get_I2C_Bus_ID(arg_bus_id);
    uint8_t address     = (uint8_t) arg_address;

    I2C_Return_T retval;
    if (is_i2c_read)
    {
        retval = I2C_Bus_Read(
            bus_id,
            address,
            s_i2c_read_buffer,
            s_i2c_data_len,
            I2C_Read_Complete_CB,
            I2C_Operation_Error_CB,
            cli);
    }
    else
    {
        retval = I2C_Bus_Write(
            bus_id, address, s_i2c_write_buffer, s_i2c_data_len, NULL, I2C_Operation_Error_CB, cli);
    }

    if (retval == I2C_RTN_ERROR)
    {
        embeddedCliPrint(cli, "I2C bus error\r\n");
    }
    else if (retval == I2C_RTN_BUSY)
    {
        embeddedCliPrint(cli, "I2C bus busy\r\n");
    }
}

static I2C_Bus_ID_T Get_I2C_Bus_ID(unsigned long arg_bus_id)
{
    Q_ASSERT(arg_bus_id > 0 && arg_bus_id < 5);

    switch (arg_bus_id)
    {
        case 1:
            return I2C_BUS_ID_1;
        case 2:
            return I2C_BUS_ID_2;
        case 3:
            return I2C_BUS_ID_3;
        case 4:
            return I2C_BUS_ID_4;
        case 5:
            return I2C_BUS_ID_5;
        case 6:
            return I2C_BUS_ID_6;
        default:
            Q_ASSERT(false);
            return I2C_BUS_MAX_SUPPORTED;
    }
}