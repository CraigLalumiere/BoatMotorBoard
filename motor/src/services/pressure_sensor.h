#ifndef PRESSURE_SENSOR_AO_H
#define PRESSURE_SENSOR_AO_H

#include "qpc.h"
#include "interfaces/i2c_interface.h"

#ifdef __cplusplus
extern "C"
{
#endif

#include <stddef.h>

    /**************************************************************************************************\
    * Public memory declarations
    \**************************************************************************************************/
    extern QActive *const AO_Pressure; // opaque pointer

    /**************************************************************************************************\
    * Public prototypes
    \**************************************************************************************************/
    void Pressure_Sensor_ctor(I2C_Write i2c_write_fn, I2C_Read i2c_read_fn);

#ifdef __cplusplus
}
#endif
#endif // PRESSURE_SENSOR_AO_H