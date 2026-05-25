#include "pc_com.h"
#include "bsp.h"
#include "cli_commands.h"
#include "crc16.h"
#include "hdlc.h"
#include "private_signal_ranges.h"
#include "reset.h"
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#define EMBEDDED_CLI_IMPL
#include "embedded_cli.h"

enum PC_COM_Signals
{
    PC_COM_SERIAL_DATA_AVAILABLE_SIG = PRIVATE_SIGNAL_PC_COM_START,
    PC_COM_CLI_PROCESS_TICK_SIG
};

enum PC_COM_MessageType
{
    PC_COM_MSG_LOG_PRINT = 1U,
    PC_COM_MSG_CLI_DATA  = 2U,
};

enum
{
    PC_COM_CLI_BUFFER_SIZE     = 2048U,
    PC_COM_RX_PACKET_MAX_BYTES = 96U,
    PC_COM_TX_PACKET_MAX_BYTES = 160U,
    PC_COM_PROCESS_TICK_MS     = 25U,
};

typedef struct
{
    uint16_t crc;
    uint8_t type;
    uint8_t message[PC_COM_RX_PACKET_MAX_BYTES];
} __attribute__((packed, aligned(1))) PC_COM_RX_Packet_T;

typedef struct
{
    uint16_t crc;
    uint8_t type;
    uint8_t message[PC_COM_TX_PACKET_MAX_BYTES];
} __attribute__((packed, aligned(1))) PC_COM_TX_Packet_T;

typedef struct
{
    QActive super;
    QTimeEvt cli_process_tick_evt;
    const Serial_IO_T *serial_io_interface;
    EmbeddedCli *embedded_cli;
    CLI_UINT cli_buffer[BYTES_TO_CLI_UINTS(PC_COM_CLI_BUFFER_SIZE)];
    HDLC_Unpacker_T hdlc_unpacker;
    PC_COM_RX_Packet_T rx_packet;
    PC_COM_TX_Packet_T tx_packet;
} PC_COM;

static PC_COM pc_com_inst;
QActive *const AO_PC_COM = &pc_com_inst.super;

static PCCOMCliDataEvent_T *s_cli_data_event;

static QState PC_COM_initial(PC_COM *const me, void const *const par);
static QState PC_COM_active(PC_COM *const me, QEvt const *const e);
static void PC_COM_SerialDataReady(void *cb_data);
static void PC_COM_CLIWriteChar(EmbeddedCli *embeddedCli, char c);
static void PC_COM_FlushCliDataEvent(void);
static void PC_COM_HandleReceivedPacket(PC_COM *const me);
static void PC_COM_HandleCliDataMessage(PC_COM *const me, const uint8_t *data, size_t data_len);
static void PC_COM_SendCliData(PC_COM *const me, const PCCOMCliDataEvent_T *cli_data_evt);
static void PC_COM_SendLogPrint(PC_COM *const me, const PCCOMPrintEvent_T *print_evt);
static void PC_COM_SendPacket(PC_COM *const me, size_t message_len);
static bool PC_COM_EncodeCliData(
    const uint8_t *data, size_t data_len, uint8_t *out, size_t out_len, size_t *written);
static bool PC_COM_EncodeLogPrint(
    uint32_t milliseconds, const char *msg, uint8_t *out, size_t out_len, size_t *written);
static bool PC_COM_DecodeCliData(
    const uint8_t *data, size_t data_len, const uint8_t **out, size_t *out_len);
static bool PC_COM_ReadVarint(const uint8_t *data, size_t data_len, size_t *offset, uint32_t *value);
static bool PC_COM_WriteVarint(uint32_t value, uint8_t *out, size_t out_len, size_t *offset);
static bool PC_COM_WriteBytesField(
    uint8_t tag, const uint8_t *data, size_t data_len, uint8_t *out, size_t out_len, size_t *offset);

void PC_COM_ctor(const Serial_IO_T *const serial_io_interface)
{
    PC_COM *const me        = &pc_com_inst;
    me->serial_io_interface = serial_io_interface;

    EmbeddedCliConfig *config = embeddedCliDefaultConfig();
    config->cliBuffer         = me->cli_buffer;
    config->cliBufferSize     = PC_COM_CLI_BUFFER_SIZE;
    config->maxBindingCount   = 60U;

    me->embedded_cli            = embeddedCliNew(config);
    me->embedded_cli->writeChar = PC_COM_CLIWriteChar;

#ifdef BOARD_MOTOR
    AppCLI_AddCommandsToCLI(me->embedded_cli);
#else
    CLI_AddCommands(me->embedded_cli);
#endif

    hdlc_unpacker_init(&me->hdlc_unpacker, (uint8_t *) &me->rx_packet, sizeof(me->rx_packet));

    s_cli_data_event           = Q_NEW(PCCOMCliDataEvent_T, POSTED_PC_COM_CLI_DATA_SIG);
    s_cli_data_event->msg_size = 0U;
    memset(s_cli_data_event->msg, 0, sizeof(s_cli_data_event->msg));

    QActive_ctor(&me->super, Q_STATE_CAST(&PC_COM_initial));
    QTimeEvt_ctorX(&me->cli_process_tick_evt, &me->super, PC_COM_CLI_PROCESS_TICK_SIG, 0U);
}

int PC_COM_Printf(const char *sFormat, ...)
{
    int r;
    va_list ParamList;
    PCCOMPrintEvent_T *printEvent = Q_NEW(PCCOMPrintEvent_T, POSTED_PC_COM_PRINT_SIG);

    printEvent->milliseconds = BSP_Get_Milliseconds_Tick();

    va_start(ParamList, sFormat);
    r = vsnprintf(printEvent->msg, sizeof(printEvent->msg), sFormat, ParamList);
    va_end(ParamList);

    printEvent->msg[sizeof(printEvent->msg) - 1U] = 0;
    QACTIVE_POST(AO_PC_COM, &printEvent->super, NULL);

    return r;
}

static QState PC_COM_initial(PC_COM *const me, void const *const par)
{
    Q_UNUSED_PAR(par);

    QTimeEvt_armX(
        &me->cli_process_tick_evt,
        MILLISECONDS_TO_TICKS(PC_COM_PROCESS_TICK_MS),
        MILLISECONDS_TO_TICKS(PC_COM_PROCESS_TICK_MS));

    me->serial_io_interface->register_cb_func(PC_COM_SerialDataReady, me);
    return Q_TRAN(&PC_COM_active);
}

static QState PC_COM_active(PC_COM *const me, QEvt const *const e)
{
    QState status;

    switch (e->sig)
    {
        case PC_COM_SERIAL_DATA_AVAILABLE_SIG: {
            uint8_t rx_byte;
            while (me->serial_io_interface->rx_func(&rx_byte, 1U) == 1U)
            {
                if (hdlc_unpacker_add_byte(&me->hdlc_unpacker, rx_byte) == FRAME_UNPACK_COMPLETE)
                {
                    PC_COM_HandleReceivedPacket(me);
                }
            }
            status = Q_HANDLED();
            break;
        }

        case PC_COM_CLI_PROCESS_TICK_SIG: {
            PC_COM_FlushCliDataEvent();
            embeddedCliProcess(me->embedded_cli);
            status = Q_HANDLED();
            break;
        }

        case POSTED_PC_COM_CLI_DATA_SIG: {
            PC_COM_SendCliData(me, Q_EVT_CAST(PCCOMCliDataEvent_T));
            status = Q_HANDLED();
            break;
        }

        case POSTED_PC_COM_PRINT_SIG: {
            PC_COM_SendLogPrint(me, Q_EVT_CAST(PCCOMPrintEvent_T));
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

static void PC_COM_SerialDataReady(void *cb_data)
{
    static QEvt const event = QEVT_INITIALIZER(PC_COM_SERIAL_DATA_AVAILABLE_SIG);
    QActive *me             = (QActive *) cb_data;
    QACTIVE_POST(me, &event, me);
}

static void PC_COM_CLIWriteChar(EmbeddedCli *embeddedCli, char c)
{
    (void) embeddedCli;

    s_cli_data_event->msg[s_cli_data_event->msg_size] = c;
    s_cli_data_event->msg_size++;

    if (s_cli_data_event->msg_size == PC_COM_CLI_DATA_MAX_LENGTH)
    {
        PC_COM_FlushCliDataEvent();
    }
}

static void PC_COM_FlushCliDataEvent(void)
{
    if (s_cli_data_event->msg_size == 0U)
    {
        return;
    }

    QACTIVE_POST(AO_PC_COM, &s_cli_data_event->super, AO_PC_COM);

    s_cli_data_event           = Q_NEW(PCCOMCliDataEvent_T, POSTED_PC_COM_CLI_DATA_SIG);
    s_cli_data_event->msg_size = 0U;
    memset(s_cli_data_event->msg, 0, sizeof(s_cli_data_event->msg));
}

static void PC_COM_HandleReceivedPacket(PC_COM *const me)
{
    if (me->hdlc_unpacker.packet_length < (sizeof(me->rx_packet.crc) + sizeof(me->rx_packet.type)))
    {
        return;
    }

    const uint8_t *packet_bytes = (const uint8_t *) &me->rx_packet;
    const size_t checked_len    = me->hdlc_unpacker.packet_length - sizeof(me->rx_packet.crc);
    uint16_t crc_calc           = crc_calculate(&packet_bytes[sizeof(me->rx_packet.crc)], checked_len);

    if (me->rx_packet.crc != crc_calc)
    {
        return;
    }

    const uint8_t *message = &packet_bytes[sizeof(me->rx_packet.crc) + sizeof(me->rx_packet.type)];
    const size_t message_len =
        me->hdlc_unpacker.packet_length - sizeof(me->rx_packet.crc) - sizeof(me->rx_packet.type);

    switch (me->rx_packet.type)
    {
        case PC_COM_MSG_CLI_DATA:
            PC_COM_HandleCliDataMessage(me, message, message_len);
            break;

        default:
            break;
    }
}

static void PC_COM_HandleCliDataMessage(PC_COM *const me, const uint8_t *data, size_t data_len)
{
    const uint8_t *cli_data = NULL;
    size_t cli_data_len     = 0U;

    if (!PC_COM_DecodeCliData(data, data_len, &cli_data, &cli_data_len))
    {
        return;
    }

    for (size_t i = 0U; i < cli_data_len; i++)
    {
        embeddedCliReceiveChar(me->embedded_cli, (char) cli_data[i]);
    }
}

static void PC_COM_SendCliData(PC_COM *const me, const PCCOMCliDataEvent_T *cli_data_evt)
{
    size_t message_len = 0U;
    if (!PC_COM_EncodeCliData(
            (const uint8_t *) cli_data_evt->msg,
            cli_data_evt->msg_size,
            me->tx_packet.message,
            sizeof(me->tx_packet.message),
            &message_len))
    {
        return;
    }

    me->tx_packet.type = PC_COM_MSG_CLI_DATA;
    PC_COM_SendPacket(me, message_len);
}

static void PC_COM_SendLogPrint(PC_COM *const me, const PCCOMPrintEvent_T *print_evt)
{
    size_t message_len = 0U;
    if (!PC_COM_EncodeLogPrint(
            print_evt->milliseconds,
            print_evt->msg,
            me->tx_packet.message,
            sizeof(me->tx_packet.message),
            &message_len))
    {
        return;
    }

    me->tx_packet.type = PC_COM_MSG_LOG_PRINT;
    PC_COM_SendPacket(me, message_len);
}

static void PC_COM_SendPacket(PC_COM *const me, size_t message_len)
{
    uint8_t *packet_bytes = (uint8_t *) &me->tx_packet;
    size_t packet_len     = sizeof(me->tx_packet.crc) + sizeof(me->tx_packet.type) + message_len;

    me->tx_packet.crc =
        crc_calculate(&packet_bytes[sizeof(me->tx_packet.crc)], packet_len - sizeof(me->tx_packet.crc));

    hdlc_transmit_packet(me->serial_io_interface->tx_func, packet_bytes, packet_len);
}

static bool PC_COM_EncodeCliData(
    const uint8_t *data, size_t data_len, uint8_t *out, size_t out_len, size_t *written)
{
    size_t offset = 0U;
    if (!PC_COM_WriteBytesField(0x0AU, data, data_len, out, out_len, &offset))
    {
        return false;
    }

    *written = offset;
    return true;
}

static bool PC_COM_EncodeLogPrint(
    uint32_t milliseconds, const char *msg, uint8_t *out, size_t out_len, size_t *written)
{
    size_t offset = 0U;

    if (offset >= out_len)
    {
        return false;
    }
    out[offset++] = 0x08U;
    if (!PC_COM_WriteVarint(milliseconds, out, out_len, &offset))
    {
        return false;
    }

    if (!PC_COM_WriteBytesField(
            0x12U,
            (const uint8_t *) msg,
            strnlen(msg, PC_COM_EVENT_MAX_MSG_LENGTH),
            out,
            out_len,
            &offset))
    {
        return false;
    }

    *written = offset;
    return true;
}

static bool PC_COM_DecodeCliData(
    const uint8_t *data, size_t data_len, const uint8_t **out, size_t *out_len)
{
    size_t offset = 0U;

    while (offset < data_len)
    {
        uint32_t tag = 0U;
        if (!PC_COM_ReadVarint(data, data_len, &offset, &tag))
        {
            return false;
        }

        uint32_t wire_type = tag & 0x07U;
        uint32_t field_num = tag >> 3U;

        if (wire_type != 2U)
        {
            return false;
        }

        uint32_t length = 0U;
        if (!PC_COM_ReadVarint(data, data_len, &offset, &length))
        {
            return false;
        }
        if ((size_t) length > (data_len - offset))
        {
            return false;
        }

        if (field_num == 1U)
        {
            *out     = &data[offset];
            *out_len = length;
            return true;
        }

        offset += length;
    }

    return false;
}

static bool PC_COM_ReadVarint(const uint8_t *data, size_t data_len, size_t *offset, uint32_t *value)
{
    uint32_t result = 0U;
    uint8_t shift   = 0U;

    while (*offset < data_len && shift < 32U)
    {
        uint8_t byte = data[*offset];
        (*offset)++;
        result |= ((uint32_t) (byte & 0x7FU)) << shift;

        if ((byte & 0x80U) == 0U)
        {
            *value = result;
            return true;
        }

        shift += 7U;
    }

    return false;
}

static bool PC_COM_WriteVarint(uint32_t value, uint8_t *out, size_t out_len, size_t *offset)
{
    do
    {
        if (*offset >= out_len)
        {
            return false;
        }

        uint8_t byte = value & 0x7FU;
        value >>= 7U;
        if (value != 0U)
        {
            byte |= 0x80U;
        }
        out[*offset] = byte;
        (*offset)++;
    } while (value != 0U);

    return true;
}

static bool PC_COM_WriteBytesField(
    uint8_t tag, const uint8_t *data, size_t data_len, uint8_t *out, size_t out_len, size_t *offset)
{
    if (*offset >= out_len)
    {
        return false;
    }

    out[*offset] = tag;
    (*offset)++;

    if (!PC_COM_WriteVarint((uint32_t) data_len, out, out_len, offset))
    {
        return false;
    }

    if (data_len > (out_len - *offset))
    {
        return false;
    }

    memcpy(&out[*offset], data, data_len);
    *offset += data_len;

    return true;
}
