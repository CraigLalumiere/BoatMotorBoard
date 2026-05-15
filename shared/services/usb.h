#ifndef USB_AO_H_
#define USB_AO_H_

#include "qpc.h"

void USB_ctor(void);
extern QActive *const AO_USB; // opaque pointer

#endif // USB_AO_H_
