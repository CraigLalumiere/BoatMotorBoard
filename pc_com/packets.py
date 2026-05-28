import struct
from crc import calculate_crc
from messages.CLIData_pb2 import CLIData
from messages.LogPrint_pb2 import LogPrint
from messages.ConfigDB_pb2 import ConfigDBSetEntryReq, ConfigDBGetEntryReq, ConfigDBSetEntryToDefaultReq, ConfigEntryDataResp, ConfigDBInfoResp
from messages.MessageType_pb2 import MessageType

message_from_id = {MessageType.LOG_PRINT: LogPrint,
                   MessageType.CLI_DATA: CLIData,
                   MessageType.CONFIG_DB_INFO_RESP: ConfigDBInfoResp,
                   MessageType.CONFIG_DB_ENTRY_DATA_RESP: ConfigEntryDataResp,
                   }


def get_message_from_packet(packet):
    """
    Unpacks framed packet, confirms CRC is good, than returns decoded protobuf object
    """
    message = None

    # extract crc from packet
    # little endian, first 2 bytes
    packet_crc = struct.unpack('<H', packet[0:2])[0]

    # calculate packet crc
    crc_calc = calculate_crc(packet[2:])

    # check if crc provided by packet matches the calculated crc
    if packet_crc == crc_calc:
        # packet id follows CRC, the 3rd byte
        packet_id = struct.unpack('<B', packet[2:3])[0]

        try:
            message = message_from_id[packet_id]()
            message.ParseFromString(packet[3:])
        except KeyError:
            return None
    else:
        print('CRC fail!')

    return message


def build_packet_cli_data(data: bytes):
    packet_id = struct.pack('<B', MessageType.CLI_DATA)

    message_pb = CLIData()
    message_pb.msg = data
    message_bytes = message_pb.SerializeToString()

    packet_id_and_data = packet_id + message_bytes
    packet_crc = struct.pack('<H', calculate_crc(packet_id_and_data))
    packet = packet_crc + packet_id_and_data

    return packet


def build_packet_config_db_info_req():
    packet_id = struct.pack('<B', MessageType.CONFIG_DB_REQ_DATABASE_INFO_REQ)

    packet_crc = struct.pack('<H', calculate_crc(packet_id))
    packet = packet_crc + packet_id

    return packet

def build_packet_config_db_save_no_nvm_req():
    packet_id = struct.pack('<B', MessageType.CONFIG_DB_SAVE_TO_NVM_REQ)

    packet_crc = struct.pack('<H', calculate_crc(packet_id))
    packet = packet_crc + packet_id

    return packet



def build_packet_config_db_get_entry_req(entry_id):
    packet_id = struct.pack('<B', MessageType.CONFIG_DB_GET_ENTRY_REQ)

    message_pb = ConfigDBGetEntryReq()
    message_pb.entry_id = entry_id
    message_bytes = message_pb.SerializeToString()

    packet_id_and_data = packet_id + message_bytes
    packet_crc = struct.pack('<H', calculate_crc(packet_id_and_data))
    packet = packet_crc + packet_id_and_data

    return packet

def build_packet_config_db_set_entry_req(msg_config_db_get_entry_req):
    packet_id = struct.pack('<B', MessageType.CONFIG_DB_SET_ENTRY_REQ)

    message_bytes = msg_config_db_get_entry_req.SerializeToString()

    packet_id_and_data = packet_id + message_bytes
    packet_crc = struct.pack('<H', calculate_crc(packet_id_and_data))
    packet = packet_crc + packet_id_and_data

    return packet




