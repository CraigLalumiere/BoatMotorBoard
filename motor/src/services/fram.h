#ifndef FRAM_H_
#define FRAM_H_

#include "interfaces/i2c_interface.h"
#include "qpc.h"

#define FRAM_PAGE_SIZE 256U

typedef struct
{
    uint16_t seq;
    uint16_t seq_complement;
} __attribute__((packed, aligned(1))) FRAM_File_Footer_T;

#define FRAM_FILE_DATA_MAX_LEN (FRAM_PAGE_SIZE - sizeof(FRAM_File_Footer_T))

typedef struct
{
    uint8_t data[FRAM_FILE_DATA_MAX_LEN];
    FRAM_File_Footer_T footer;
} __attribute__((packed, aligned(1))) FRAM_File_T;

typedef enum
{
    FRAM_FILE_READ_OK,
    FRAM_FILE_READ_FAIL
} Fram_Read_Status_T;

typedef struct
{
    QEvt super;
    QActive *requester;
} FramReadReqEvent_T;

typedef struct
{
    QEvt super;
    Fram_Read_Status_T read_status;
    FRAM_File_T file;
} FramReadRespEvent_T;

typedef struct
{
    QEvt super;
    QActive *requester;
    FRAM_File_T file;
} FramWriteReqEvent_T;

extern QActive *const AO_Fram;

void Fram_ctor(I2C_Write i2c_write_fn, I2C_MemoryRead i2c_memory_read_fn);

#endif // FRAM_H_
