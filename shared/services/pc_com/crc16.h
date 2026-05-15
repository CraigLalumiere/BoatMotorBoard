#ifndef CRC16_H_
#define CRC16_H_

#include "stdint.h"

#ifdef __cplusplus
extern "C" {
#endif

/**************************************************************************************************\
* Public prototypes
\**************************************************************************************************/
uint16_t crc_calculate(const uint8_t *data, uint16_t data_len);

#ifdef __cplusplus
}
#endif
#endif // CRC16_H_
