#include "pc_com.h"
#include "bsp.h"
#include "c/CLIData.pb.h"
#include "c/LogPrint.pb.h"
#include "c/MessageType.pb.h"
#include "cli_commands.h"
#include "crc16.h"
#include "hdlc.h"
#include "pb_decode.h"
#include "pb_encode.h"
#include "private_signal_ranges.h"
#include "pubsub_signals.h"
#include "reset.h"
#include "safe_strncpy.h"
#include "stdio.h"
#include <string.h>

#include "c/ConfigDB.pb.h"
#include "config.h"

#define EMBEDDED_CLI_IMPL
#include "embedded_cli.h"

Q_DEFINE_THIS_MODULE("pc_com")

/**************************************************************************************************\
* Private macros
\**************************************************************************************************/
#define CLI_BUFFER_SIZE 1024

/**************************************************************************************************\
* Private type definitions
\**************************************************************************************************/
enum PC_COM_Signals
{
    SERIAL_DATA_AVAILABLE_SIG = PRIVATE_SIGNAL_PC_COM_START,
    CLI_PROCESS_TICK_SIG,
};

typedef uint16_t Packet_CRC_T;
typedef uint8_t Packet_Type_T;

// encoded PB messages are variable length, so this union ensures allocation to get the largest
// encoded message
typedef union
{
    uint8_t LogPrint_max[LOGPRINT_PB_H_MAX_SIZE];
    uint8_t CLIData_max[CLIDATA_PB_H_MAX_SIZE];
    uint8_t ConfigDBInfoResp_max[ConfigDBInfoResp_size];
    uint8_t ConfigEntryDataResp_max[ConfigEntryDataResp_size];
} TX_Message_Buffer_T;

typedef union
{
    uint8_t LogPrint_max[LOGPRINT_PB_H_MAX_SIZE];
    uint8_t CLIData_max[CLIDATA_PB_H_MAX_SIZE];
    uint8_t ConfigDBGetEntryReq_max[ConfigDBGetEntryReq_size];
    uint8_t ConfigDBSetEntryReq_max[ConfigDBSetEntryReq_size];
    uint8_t ConfigDBSetEntryToDefaultReq_max[ConfigDBSetEntryToDefaultReq_size];
} RX_Message_Buffer_T;

typedef union
{
    CLIData CLI_data;
    ConfigDBGetEntryReq config_db_get_entry_req;
    ConfigDBSetEntryReq config_db_set_entry_req;
} RX_Message_Decoded_T;

typedef struct
{
    Packet_CRC_T crc;
    Packet_Type_T type;
    TX_Message_Buffer_T message;
} __attribute__((packed, aligned(1))) PC_COM_TX_Packet_T;

typedef struct
{
    Packet_CRC_T crc;
    Packet_Type_T type;
    RX_Message_Buffer_T message;
} __attribute__((packed, aligned(1))) PC_COM_RX_Packet_T;

typedef struct
{
    QActive super; // inherit QActive
    const Serial_IO_T *serial_io_interface;

    PC_COM_TX_Packet_T tx_packet;
    PC_COM_RX_Packet_T rx_packet;

    HDLC_Unpacker_T hdlc_unpacker;
    RX_Message_Decoded_T rx_message_decoded;

    EmbeddedCli *embedded_cli;
    CLI_UINT cliBuffer[BYTES_TO_CLI_UINTS(CLI_BUFFER_SIZE)];

    QTimeEvt testEvt;
    QTimeEvt cli_process_tick_evt;
} PC_COM;

/**************************************************************************************************\
* Private memory declarations
\**************************************************************************************************/
static PC_COM pc_com_inst;
QActive *const AO_PC_COM = &pc_com_inst.super;

PCCOMCliDataEvent_T *cli_data_event;

/**************************************************************************************************\
* Private prototypes
\**************************************************************************************************/

static QState initial(PC_COM *const me, void const *const par);
static QState active(PC_COM *const me, QEvt const *const e);

static void calculate_crc_and_send_packet(PC_COM *const me, size_t message_len);
static void Serial_Data_Ready(void *cb_data);
static void parse_and_handle_pc_packet(PC_COM *const me);
static void handle_cli_char_received(PC_COM *const me);
static void handle_config_db_info_req(PC_COM *const me);
static void handle_config_get_entry_req(PC_COM *const me);
static void handle_config_set_entry_req(PC_COM *const me);
static void handle_config_db_save_to_nvm_req(PC_COM *const me);

static void send_db_entry_data_resp_msg(PC_COM *const me, uint32_t id);

static void cli_write_char(EmbeddedCli *embeddedCli, char c);

/**************************************************************************************************\
* Public functions
\**************************************************************************************************/

/**
 ***************************************************************************************************
 * @brief   Constructor
 **************************************************************************************************/
void PC_COM_ctor(const Serial_IO_T *const serial_io_interface)
{
    PC_COM *const me = &pc_com_inst;

    pc_com_inst.serial_io_interface = serial_io_interface;

    // CLI config
    EmbeddedCliConfig *config = embeddedCliDefaultConfig();

    // use static allocated buffer for CLI storage
    config->cliBuffer       = me->cliBuffer;
    config->cliBufferSize   = CLI_BUFFER_SIZE;
    config->maxBindingCount = 30;

    // create an instance of CLI
    me->embedded_cli = embeddedCliNew(config);

    // store char function
    me->embedded_cli->writeChar = cli_write_char;

    // add commands from command table
    CLI_AddCommands(me->embedded_cli);

    // initialize the HDLC frame unpacker by pointing it to the receive buffer
    hdlc_unpacker_init(
        &pc_com_inst.hdlc_unpacker,
        (uint8_t *) (&pc_com_inst.rx_packet),
        sizeof(PC_COM_RX_Packet_T));

    // init cli data buffer
    cli_data_event           = Q_NEW(PCCOMCliDataEvent_T, POSTED_PC_COM_CLI_DATA_SIG);
    cli_data_event->msg_size = 0;
    memset(cli_data_event->msg, 0, CLI_DATA_MAX_LENGTH);

    QActive_ctor(&me->super, Q_STATE_CAST(&initial));

    QTimeEvt_ctorX(&me->cli_process_tick_evt, &me->super, CLI_PROCESS_TICK_SIG, 0U);
}

/**
 ***************************************************************************************************
 *
 * @brief   Publish message to PC COM with string message
 *
 **************************************************************************************************/
void PC_COM_print(const char *msg)
{
    PCCOMPrintEvent_T *event = Q_NEW(PCCOMPrintEvent_T, POSTED_PC_COM_PRINT_SIG);

    event->milliseconds = BSP_Get_Milliseconds_Tick();
    safe_strncpy(event->msg, msg, sizeof(event->msg));

    QACTIVE_POST(AO_PC_COM, (QEvt *) (event), AO_PC_COM);
}

/**************************************************************************************************\
* Private functions
\**************************************************************************************************/

/**
 ***************************************************************************************************
 *
 * @brief   HSM
 *
 **************************************************************************************************/
static QState initial(PC_COM *const me, void const *const par)
{
    Q_UNUSED_PAR(par);

    QActive_subscribe((QActive *) me, PUBSUB_CONFIG_ENTRY_CHANGED_SIG);

    // Process CLI  every 25ms
    QTimeEvt_armX(
        &me->cli_process_tick_evt, MILLISECONDS_TO_TICKS(25U), MILLISECONDS_TO_TICKS(25U));

    // test
    // QTimeEvt_armX(&me->testEvt, MILLISECONDS_TO_TICKS(1000), MILLISECONDS_TO_TICKS(1000));

    // Register callback with the SerialIO interface to be called when new data is available
    me->serial_io_interface->register_cb_func(Serial_Data_Ready, me);
    return Q_TRAN(&active);
}

static QState active(PC_COM *const me, QEvt const *const e)
{
    QState status;

    switch (e->sig)
    {
        case Q_ENTRY_SIG: {
            status = Q_HANDLED();
            break;
        }

        case SERIAL_DATA_AVAILABLE_SIG: {
            uint8_t rx_byte;
            HDLC_Unpack_State_T unpack_state;

            while (me->serial_io_interface->rx_func(&rx_byte, 1) == 1)
            {
                unpack_state = hdlc_unpacker_add_byte(&me->hdlc_unpacker, rx_byte);

                if (unpack_state == FRAME_UNPACK_COMPLETE)
                {
                    parse_and_handle_pc_packet(me);
                }
            }

            status = Q_HANDLED();
            break;
        }

        case CLI_PROCESS_TICK_SIG: {
            // check to see if there is CLI data in buffer to be sent
            if (cli_data_event->msg_size > 0)
            {
                QACTIVE_POST(AO_PC_COM, (QEvt *) (cli_data_event), AO_PC_COM);

                // init a new cli data event
                cli_data_event           = Q_NEW(PCCOMCliDataEvent_T, POSTED_PC_COM_CLI_DATA_SIG);
                cli_data_event->msg_size = 0;
                memset(cli_data_event->msg, 0, CLI_DATA_MAX_LENGTH);
            }

            embeddedCliProcess(me->embedded_cli);
            status = Q_HANDLED();
            break;
        }

        case POSTED_PC_COM_CLI_DATA_SIG: {
            // set message type
            me->tx_packet.type = MessageType_CLI_DATA;

            // create pb message
            CLIData message = CLIData_init_zero;

            // populate message and encode it
            pb_ostream_t stream = pb_ostream_from_buffer(
                ((uint8_t *) &me->tx_packet.message), sizeof(TX_Message_Buffer_T));

            const PCCOMCliDataEvent_T *cli_data_evt = Q_EVT_CAST(PCCOMCliDataEvent_T);

            memcpy(message.msg.bytes, cli_data_evt->msg, cli_data_evt->msg_size);
            message.msg.size = cli_data_evt->msg_size;

            bool ok = pb_encode(&stream, CLIData_fields, &message);
            Q_ASSERT(ok);

            // calculate CRC and transmit
            calculate_crc_and_send_packet(me, stream.bytes_written);
            status = Q_HANDLED();
            break;
        }

        case POSTED_PC_COM_PRINT_SIG: {
            // set message type
            me->tx_packet.type = MessageType_LOG_PRINT;

            // create pb message
            LogPrint message = LogPrint_init_zero;

            // populate message and encode it
            pb_ostream_t stream = pb_ostream_from_buffer(
                ((uint8_t *) &me->tx_packet.message), sizeof(TX_Message_Buffer_T));

            message.milliseconds_tick = Q_EVT_CAST(PCCOMPrintEvent_T)->milliseconds;
            safe_strncpy(message.msg, Q_EVT_CAST(PCCOMPrintEvent_T)->msg, sizeof(message.msg));

            bool ok = pb_encode(&stream, LogPrint_fields, &message);
            Q_ASSERT(ok);

            // calculate CRC and transmit
            calculate_crc_and_send_packet(me, stream.bytes_written);
            status = Q_HANDLED();
            break;
        }

        case PUBSUB_CONFIG_ENTRY_CHANGED_SIG: {
            const ConfigEntryChangedEvent_T *evt = Q_EVT_CAST(ConfigEntryChangedEvent_T);
            send_db_entry_data_resp_msg(me, evt->id);
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

static void calculate_crc_and_send_packet(PC_COM *const me, size_t message_len)
{
    // calculate CRC, including packet type and data (ignore the CRC itself)
    uint16_t crc_calc = crc_calculate(
        &((uint8_t *) &me->tx_packet)[sizeof(Packet_CRC_T)], sizeof(Packet_Type_T) + message_len);

    // add CRC to the packet
    me->tx_packet.crc = crc_calc;

    // transmit packet
    hdlc_transmit_packet(
        me->serial_io_interface->tx_func,
        ((uint8_t *) &me->tx_packet),
        sizeof(Packet_CRC_T) + sizeof(Packet_Type_T) + message_len);
}

/**
 ***************************************************************************************************
 *
 * @brief   Serial data ready callback, called by external context.
 *
 **************************************************************************************************/
static void Serial_Data_Ready(void *cb_data)
{
    static QEvt const event = QEVT_INITIALIZER(SERIAL_DATA_AVAILABLE_SIG);

    QActive *me = (QActive *) cb_data;
    QACTIVE_POST(me, &event, me);
}

/**
 ***************************************************************************************************
 *
 * @brief   Adapter for CLI write char
 *
 **************************************************************************************************/
static void cli_write_char(EmbeddedCli *embeddedCli, char c)
{
    cli_data_event->msg[cli_data_event->msg_size] = c;
    cli_data_event->msg_size++;

    if (cli_data_event->msg_size == CLI_DATA_MAX_LENGTH)
    {
        QACTIVE_POST(AO_PC_COM, (QEvt *) (cli_data_event), AO_PC_COM);

        // init a new cli data event
        cli_data_event           = Q_NEW(PCCOMCliDataEvent_T, POSTED_PC_COM_CLI_DATA_SIG);
        cli_data_event->msg_size = 0;
        memset(cli_data_event->msg, 0, CLI_DATA_MAX_LENGTH);
    }
}

/**
 ***************************************************************************************************
 *
 * @brief   Parse packet recieved from PC and handle it
 *
 **************************************************************************************************/
static void parse_and_handle_pc_packet(PC_COM *const me)
{
    uint16_t crc_calc;

    // calculate the CRC of the received packet, excluding the CRC itself (the initial bytes)
    crc_calc = crc_calculate(
        &((uint8_t *) &me->rx_packet)[sizeof(Packet_CRC_T)],
        me->hdlc_unpacker.packet_length - sizeof(Packet_CRC_T));

    // does crc in the packet match the calculated crc?
    if (me->rx_packet.crc == crc_calc)
    {
        switch (me->rx_packet.type)
        {
            // CLI character(s) received
            case MessageType_CLI_DATA:
                handle_cli_char_received(me);
                break;

            // config DB request - database info
            case MessageType_CONFIG_DB_REQ_DATABASE_INFO_REQ:
                handle_config_db_info_req(me);
                break;

            // config DB request - get entry
            case MessageType_CONFIG_DB_GET_ENTRY_REQ:
                handle_config_get_entry_req(me);
                break;

            // config DB request - set entry
            case MessageType_CONFIG_DB_SET_ENTRY_REQ:
                handle_config_set_entry_req(me);
                break;

            // config DB request - commit to NVM
            case MessageType_CONFIG_DB_SAVE_TO_NVM_REQ:
                handle_config_db_save_to_nvm_req(me);
                break;

            // command not found, let it go
            default:
                break;
        }
    }
}

static void handle_cli_char_received(PC_COM *const me)
{
    pb_istream_t stream = pb_istream_from_buffer(
        ((uint8_t *) &me->rx_packet.message), sizeof(RX_Message_Buffer_T));

    pb_decode(&stream, CLIData_fields, &me->rx_message_decoded);

    for (size_t i = 0; i < me->rx_message_decoded.CLI_data.msg.size; i++)
    {
        embeddedCliReceiveChar(me->embedded_cli, me->rx_message_decoded.CLI_data.msg.bytes[i]);
    }
}

static void handle_config_db_info_req(PC_COM *const me)
{
    // set message type
    me->tx_packet.type = MessageType_CONFIG_DB_INFO_RESP;

    // create pb message
    ConfigDBInfoResp message = ConfigDBInfoResp_init_zero;

    // populate message and encode it
    pb_ostream_t stream = pb_ostream_from_buffer(
        ((uint8_t *) &me->tx_packet.message), sizeof(TX_Message_Buffer_T));

    message.num_elements = Config_Get_Num_Elements();
    message.version      = Config_Get_Version();
    bool ok              = pb_encode(&stream, ConfigDBInfoResp_fields, &message);
    Q_ASSERT(ok);

    // calculate CRC and transmit
    calculate_crc_and_send_packet(me, stream.bytes_written);
}

static void handle_config_db_save_to_nvm_req(PC_COM *const me)
{
    static QEvt const event = QEVT_INITIALIZER(POSTED_CONFIG_SAVE_TO_NVM_REQ_SIG);
    QACTIVE_POST(AO_Config, &event, &me);
}

static void handle_config_get_entry_req(PC_COM *const me)
{
    pb_istream_t istream = pb_istream_from_buffer(
        ((uint8_t *) &me->rx_packet.message), sizeof(RX_Message_Buffer_T));

    pb_decode(&istream, ConfigDBGetEntryReq_fields, &me->rx_message_decoded);
    uint32_t entry_id = me->rx_message_decoded.config_db_get_entry_req.entry_id;

    if (entry_id < Config_Get_Num_Elements())
    {
        send_db_entry_data_resp_msg(me, entry_id);
    }
}

/**
 ***************************************************************************************************
 *
 * @brief   Sets config dB entry, then reads it and sends it back to pc
 *
 **************************************************************************************************/
// static void handle_config_set_entry_req(PC_COM *const me)
// {
//     pb_istream_t istream = pb_istream_from_buffer(
//         ((uint8_t *) &me->rx_packet.message), sizeof(RX_Message_Buffer_T));

//     pb_decode(&istream, ConfigDBSetEntryReq_fields, &me->rx_message_decoded);

//     // set message type
//     me->tx_packet.type = MessageType_CONFIG_DB_ENTRY_DATA_RESP;

//     // create pb message
//     ConfigEntryDataResp message = ConfigEntryDataResp_init_zero;

//     // populate message and encode it
//     pb_ostream_t ostream = pb_ostream_from_buffer(
//         ((uint8_t *) &me->tx_packet.message), sizeof(TX_Message_Buffer_T));

//     message.entry_id = me->rx_message_decoded.config_db_set_entry_req.entry_id;

//     ConfigValueType_T val_type = Config_GetType(message.entry_id);

//     switch (val_type)
//     {
//         case CFG_VAL_TYPE_U32:
//             // ensure value type received by message matches expected config value type
//             // if not, don't write it
//             if (me->rx_message_decoded.config_db_set_entry_req.value.which_value ==
//                 ConfigValue_value_uint32_tag)
//             {
//                 Config_Write_U32(
//                     message.entry_id,
//                     me->rx_message_decoded.config_db_set_entry_req.value.value.value_uint32);
//             }

//             message.value.which_value        = ConfigValue_value_uint32_tag;
//             message.value.value.value_uint32 = Config_Read_U32(message.entry_id);

//             message.default_value.which_value        = ConfigValue_value_uint32_tag;
//             message.default_value.value.value_uint32 = Config_Read_Default_U32(message.entry_id);
//             break;

//         case CFG_VAL_TYPE_I32:
//             // ensure value type received by message matches expected config value type
//             // if not, don't write it
//             if (me->rx_message_decoded.config_db_set_entry_req.value.which_value ==
//                 ConfigValue_value_int32_tag)
//             {
//                 Config_Write_I32(
//                     message.entry_id,
//                     me->rx_message_decoded.config_db_set_entry_req.value.value.value_int32);
//             }

//             message.value.which_value       = ConfigValue_value_int32_tag;
//             message.value.value.value_int32 = Config_Read_I32(message.entry_id);

//             message.default_value.which_value       = ConfigValue_value_int32_tag;
//             message.default_value.value.value_int32 = Config_Read_Default_I32(message.entry_id);
//             break;

//         case CFG_VAL_TYPE_BOOL:
//             // ensure value type received by message matches expected config value type
//             // if not, don't write it
//             if (me->rx_message_decoded.config_db_set_entry_req.value.which_value ==
//                 ConfigValue_value_bool_tag)
//             {
//                 Config_Write_Bool(
//                     message.entry_id,
//                     me->rx_message_decoded.config_db_set_entry_req.value.value.value_bool);
//             }

//             message.value.which_value      = ConfigValue_value_bool_tag;
//             message.value.value.value_bool = Config_Read_Bool(message.entry_id);

//             message.default_value.which_value      = ConfigValue_value_bool_tag;
//             message.default_value.value.value_bool = Config_Read_Default_Bool(message.entry_id);
//             break;

//         case CFG_VAL_TYPE_F32:
//             // if not, don't write it
//             if (me->rx_message_decoded.config_db_set_entry_req.value.which_value ==
//                 ConfigValue_value_float32_tag)
//             {
//                 Config_Write_F32(
//                     message.entry_id,
//                     me->rx_message_decoded.config_db_set_entry_req.value.value.value_float32);
//             }

//             message.value.which_value         = ConfigValue_value_float32_tag;
//             message.value.value.value_float32 = Config_Read_F32(message.entry_id);

//             message.default_value.which_value         = ConfigValue_value_float32_tag;
//             message.default_value.value.value_float32 =
//             Config_Read_Default_F32(message.entry_id); break;

//         default:
//             Q_ASSERT(false);
//     }

//     safe_strncpy(message.name, Config_GetName(message.entry_id), sizeof(message.name));

//     bool ok = pb_encode(&ostream, ConfigEntryDataResp_fields, &message);
//     Q_ASSERT(ok);

//     // calculate CRC and transmit
//     calculate_crc_and_send_packet(me, ostream.bytes_written);
// }

static void handle_config_set_entry_req(PC_COM *const me)
{
    pb_istream_t istream = pb_istream_from_buffer(
        ((uint8_t *) &me->rx_packet.message), sizeof(RX_Message_Buffer_T));

    pb_decode(&istream, ConfigDBSetEntryReq_fields, &me->rx_message_decoded);

    uint32_t entry_id = me->rx_message_decoded.config_db_set_entry_req.entry_id;
    if (entry_id >= Config_Get_Num_Elements())
    {
        return;
    }

    ConfigValueType_T val_type = Config_GetType(entry_id);

    switch (val_type)
    {
        case CFG_VAL_TYPE_U32:
            // ensure value type received by message matches expected config value type
            // if not, don't write it
            if (me->rx_message_decoded.config_db_set_entry_req.value.which_value ==
                ConfigValue_value_uint32_tag)
            {
                Config_Write_U32(
                    entry_id,
                    me->rx_message_decoded.config_db_set_entry_req.value.value.value_uint32);
            }
            break;

        case CFG_VAL_TYPE_I32:
            // ensure value type received by message matches expected config value type
            // if not, don't write it
            if (me->rx_message_decoded.config_db_set_entry_req.value.which_value ==
                ConfigValue_value_int32_tag)
            {
                Config_Write_I32(
                    entry_id,
                    me->rx_message_decoded.config_db_set_entry_req.value.value.value_int32);
            }
            break;

        case CFG_VAL_TYPE_BOOL:
            // ensure value type received by message matches expected config value type
            // if not, don't write it
            if (me->rx_message_decoded.config_db_set_entry_req.value.which_value ==
                ConfigValue_value_bool_tag)
            {
                Config_Write_Bool(
                    entry_id,
                    me->rx_message_decoded.config_db_set_entry_req.value.value.value_bool);
            }
            break;

        case CFG_VAL_TYPE_F32:
            // if not, don't write it
            if (me->rx_message_decoded.config_db_set_entry_req.value.which_value ==
                ConfigValue_value_float32_tag)
            {
                Config_Write_F32(
                    entry_id,
                    me->rx_message_decoded.config_db_set_entry_req.value.value.value_float32);
            }
            break;

        default:
            Q_ASSERT(false);
    }

    send_db_entry_data_resp_msg(me, entry_id);
}

static void send_db_entry_data_resp_msg(PC_COM *const me, uint32_t id)
{
    // set message type
    me->tx_packet.type = MessageType_CONFIG_DB_ENTRY_DATA_RESP;

    // create pb message
    ConfigEntryDataResp message = ConfigEntryDataResp_init_zero;

    // populate message and encode it
    pb_ostream_t ostream = pb_ostream_from_buffer(
        ((uint8_t *) &me->tx_packet.message), sizeof(TX_Message_Buffer_T));

    message.entry_id = id;

    ConfigValueType_T val_type = Config_GetType(message.entry_id);

    switch (val_type)
    {
        case CFG_VAL_TYPE_U32:
            message.value.which_value        = ConfigValue_value_uint32_tag;
            message.value.value.value_uint32 = Config_Read_U32(message.entry_id);

            message.default_value.which_value        = ConfigValue_value_uint32_tag;
            message.default_value.value.value_uint32 = Config_Read_Default_U32(message.entry_id);
            break;

        case CFG_VAL_TYPE_I32:
            message.value.which_value       = ConfigValue_value_int32_tag;
            message.value.value.value_int32 = Config_Read_I32(message.entry_id);

            message.default_value.which_value       = ConfigValue_value_int32_tag;
            message.default_value.value.value_int32 = Config_Read_Default_I32(message.entry_id);
            break;

        case CFG_VAL_TYPE_BOOL:
            message.value.which_value      = ConfigValue_value_bool_tag;
            message.value.value.value_bool = Config_Read_Bool(message.entry_id);

            message.default_value.which_value      = ConfigValue_value_bool_tag;
            message.default_value.value.value_bool = Config_Read_Default_Bool(message.entry_id);
            break;

        case CFG_VAL_TYPE_F32:
            message.value.which_value         = ConfigValue_value_float32_tag;
            message.value.value.value_float32 = Config_Read_F32(message.entry_id);

            message.default_value.which_value         = ConfigValue_value_float32_tag;
            message.default_value.value.value_float32 = Config_Read_Default_F32(message.entry_id);
            break;

        default:
            Q_ASSERT(false);
    }

    safe_strncpy(message.name, Config_GetName(message.entry_id), sizeof(message.name));

    bool ok = pb_encode(&ostream, ConfigEntryDataResp_fields, &message);
    Q_ASSERT(ok);

    // calculate CRC and transmit
    calculate_crc_and_send_packet(me, ostream.bytes_written);
}
