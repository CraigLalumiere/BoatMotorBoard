#ifndef USB_BOARD_API_H
#define USB_BOARD_API_H

#include "stm32g4xx.h"
#include "tusb.h"

size_t board_get_unique_id(uint8_t id[], size_t max_len);
size_t board_usb_get_serial(uint16_t desc_str1[], size_t max_chars);

#endif // USB_BOARD_API_H
