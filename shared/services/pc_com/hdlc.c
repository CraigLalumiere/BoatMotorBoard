#include "hdlc.h"

/**************************************************************************************************\
* Private macros
\**************************************************************************************************/
static uint8_t FLAG   = 0x7EU;
static uint8_t DLE    = 0x7DU;
static uint8_t XORCHR = 0x20U;

// convenience macro to add bytes to unpacker, since this code is repeated a few times
// NOTE: if packer buffer length exceeded, frame is cancelled (no error is thrown)
#define ADD_BYTE(packet_byte)                                     \
    {                                                             \
        me->packet_buffer_ptr[me->packet_length++] = packet_byte; \
        if (me->packet_length == me->packet_buffer_length)        \
        {                                                         \
            me->packet_length = 0;                                \
            me->unpack_state  = FRAME_UNPACK_WAIT_SYNC;           \
        }                                                         \
    }

/**************************************************************************************************\
* Public functions
\**************************************************************************************************/

/**
 ***************************************************************************************************
 *
 * @brief Creates a new frame unpacker object
 *
 * @param me                     Pointer to the HDLC unpacker context
 * @param packet_buffer_ptr      Pointer to buffer where packet will be stored
 * @param packet_buffer_length   Length of buffer where packet will be stored
 *
 **************************************************************************************************/
void hdlc_unpacker_init(
    HDLC_Unpacker_T *me, uint8_t *packet_buffer_ptr, size_t packet_buffer_length)
{
    me->unpack_state         = FRAME_UNPACK_WAIT_SYNC;
    me->packet_length        = 0;
    me->packet_buffer_length = packet_buffer_length;
    me->packet_buffer_ptr    = (uint8_t *) packet_buffer_ptr;
}

/**
 ***************************************************************************************************
 *
 * @brief   Adds new byte to HLDC unpacker, which unpacks serialized byte stream
 *
 * @details The frame unpacker receives data from the serialized byte stream a
 *          byte at a time.
 *
 *          Once unpack_state == FRAME_UNPACK_MSG_COMPLETE, packet_buffer_ptr will
 *          point to a complete unpacked packet.
 *
 * @param   me                          Pointer to the HDLC unpacker context
 * @param   new_byte                    New byte from serialized stream to unpack
 *
 * @retval FRAME_UNPACK_WAIT_SYNC       Not in sync with data stream
 * @retval FRAME_UNPACK_AFTER_FLAG      Waiting for byte after FLAG
 * @retval FRAME_UNPACK_AFTER_DLE       Waiting for byte after DLE
 * @retval FRAME_UNPACK_IN_MSG          Waiting for next byte
 * @retval FRAME_UNPACK_COMPLETE        Packed is complete
 *
 **************************************************************************************************/
HDLC_Unpack_State_T hdlc_unpacker_add_byte(HDLC_Unpacker_T *me, uint8_t new_byte)
{
    switch (me->unpack_state)
    {
        // Stream is out of sync. Waiting for the first flag in stream...
        case FRAME_UNPACK_WAIT_SYNC:
            // First flag has been received
            if (new_byte == FLAG)
            {
                me->unpack_state = FRAME_UNPACK_AFTER_FLAG;
            }
            break;

        case FRAME_UNPACK_COMPLETE:
            // new byte arrived when frame completed
            // create a new packet then handle FRAME_UNPACK_AFTER_FLAG
            me->packet_length = 0;
            me->unpack_state  = FRAME_UNPACK_AFTER_FLAG;
            __attribute__((fallthrough));

        case FRAME_UNPACK_AFTER_FLAG:
            //  If a second flag immediately follows the first, re-start message
            if (new_byte == FLAG)
            {
                // nothing required to do, stay in this state
            }
            // if byte is DLE, an escaped message byte follows
            else if (new_byte == DLE)
            {
                me->unpack_state = FRAME_UNPACK_AFTER_DLE;
            }
            // the first byte has been received
            else
            {
                me->unpack_state = FRAME_UNPACK_IN_MSG;
                // add byte to the packet
                ADD_BYTE(new_byte);
            }
            break;

        case FRAME_UNPACK_AFTER_DLE:
            me->unpack_state = FRAME_UNPACK_IN_MSG;

            // add x-or byte to the packet
            ADD_BYTE(new_byte ^ XORCHR);
            break;

        case FRAME_UNPACK_IN_MSG:
            // once in message and next flag occurs, message is complete
            if (new_byte == FLAG)
            {
                me->unpack_state = FRAME_UNPACK_COMPLETE;
            }
            // if byte is DLE, an escaped message byte follows
            else if (new_byte == DLE)
            {
                me->unpack_state = FRAME_UNPACK_AFTER_DLE;
            }
            else
            {
                // at byte to the packet
                ADD_BYTE(new_byte);
            }
            break;
    }

    return me->unpack_state;
}

/**
 ***************************************************************************************************
 *
 * @brief   Frames and transmits packet over serial interface
 *
 *          The packet could be framed into a local buffer, than transmitted with single serial
 *          transmit call. But given that this function is run to complection, it dumps packets
 *          one at a time into the serial buffer. If the buffer is full, the transfer will fail, so
 *          the application must manage the volume of TX data.
 *
 * @param   me           Pointer to the HDLC transmitter context
 * @param   data         Pointer to the packet data to be transmitted
 * @param   data_length  Length of data to be transmitted
 *
 * @retval  0 Packet framed and transmitted OK
 * @retval -1 Serial TX error occured. Packet failed to fully transmit (or got corrupted)
 *
 **************************************************************************************************/
int8_t hdlc_transmit_packet(Serial_IO_TransmitData tx_func, uint8_t *data, size_t data_length)
{
    int8_t error     = 0;
    int8_t byte_sent = 1;
    uint8_t byte_xor;
    size_t i;

    // start frame with a FLAG
    byte_sent &= tx_func(&FLAG, 1);

    // send data
    for (i = 0; i < data_length; i++)
    {
        // if FLAG or DLE byte encountered, replace with DLE and XOR'd flag
        if (data[i] == FLAG || data[i] == DLE)
        {
            byte_xor = data[i] ^ XORCHR;
            byte_sent &= tx_func(&DLE, 1);
            byte_sent &= tx_func(&byte_xor, 1);
        }
        // else, send byte as-is
        else
        {
            byte_sent &= tx_func(&data[i], 1);
        }
    }

    // end frame with a FLAG
    byte_sent &= tx_func(&FLAG, 1);

    if (byte_sent == 0)
    {
        error = -1;
    }

    return error;
}