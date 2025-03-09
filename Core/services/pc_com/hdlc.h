#ifndef HDLC_H_
#define HDLC_H_

#include "interfaces/serial_interface.h"
#include "stddef.h"
#include "stdint.h"

#ifdef __cplusplus
extern "C" {
#endif

/**************************************************************************************************\
* Public type definitions
\**************************************************************************************************/
typedef enum
{
    FRAME_UNPACK_WAIT_SYNC,  // Not in sync with data stream
    FRAME_UNPACK_AFTER_FLAG, // Waiting for byte after FLAG
    FRAME_UNPACK_AFTER_DLE,  // Waiting for byte after DLE
    FRAME_UNPACK_IN_MSG,     // Waiting for next byte
    FRAME_UNPACK_COMPLETE    // Packet unpack complete
} HDLC_Unpack_State_T;

typedef struct
{
    HDLC_Unpack_State_T unpack_state;
    size_t packet_length;
    size_t packet_buffer_length;
    uint8_t *packet_buffer_ptr;
} HDLC_Unpacker_T;

/**************************************************************************************************\
* Public prototypes
\**************************************************************************************************/
void hdlc_unpacker_init(
    HDLC_Unpacker_T *me, uint8_t *packet_buffer_ptr, size_t packet_buffer_length);
HDLC_Unpack_State_T hdlc_unpacker_add_byte(HDLC_Unpacker_T *me, uint8_t new_byte);
int8_t hdlc_transmit_packet(Serial_IO_TransmitData tx_func, uint8_t *data, size_t data_length);

#ifdef __cplusplus
}
#endif
#endif // HDLC_H_