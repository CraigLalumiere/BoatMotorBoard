#ifndef DIRECTOR_AO_H
#define DIRECTOR_AO_H

#include "qpc.h"

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>

/**************************************************************************************************\
* Public memory declarations
\**************************************************************************************************/
extern QActive *const AO_Director; // opaque pointer

/**************************************************************************************************\
* Public prototypes
\**************************************************************************************************/
void Director_ctor();

#ifdef __cplusplus
}
#endif
#endif // DIRECTOR_AO_H
