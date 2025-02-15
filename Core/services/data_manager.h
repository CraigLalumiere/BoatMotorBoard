#ifndef DATA_MANAGER_AO_H
#define DATA_MANAGER_AO_H

#include "qpc.h"

#ifdef __cplusplus
extern "C" {
#endif


#include <stddef.h>



/**************************************************************************************************\
* Public memory declarations
\**************************************************************************************************/
extern QActive *const AO_Data_Manager; // opaque pointer

/**************************************************************************************************\
* Public prototypes
\**************************************************************************************************/
void Data_Manager_ctor();

#ifdef __cplusplus
}
#endif
#endif // DATA_MANAGER_AO_H

