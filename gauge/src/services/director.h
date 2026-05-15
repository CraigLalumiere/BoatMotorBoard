#ifndef DIRECTOR_H
#define DIRECTOR_H

#include "qpc.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Public AO pointer (same pattern as your other AOs)
extern QActive *const AO_DIRECTOR;

// Hypothetical event published by your CAN AO.
// Adjust units to match your CAN messages once defined.
typedef struct
{
    QEvt super;
    float rpm;          // engine RPM
    float temperature_c; // degrees C
    float pressure_psi;  // PSI
} EngineDataEvt;

// Constructor
void Director_ctor(void);

#ifdef __cplusplus
}
#endif

#endif // DIRECTOR_H