#ifndef SSD1306_AO_H
#define SSD1306_AO_H

#include "qpc.h"

#ifdef __cplusplus
extern "C"
{
#endif

#include <stddef.h>
#include "interfaces/i2c_interface.h"

#define SSD1306_INCLUDE_FONT_7x10

    /** Font */
    typedef struct
    {
        const uint8_t width;             /**< Font width in pixels */
        const uint8_t height;            /**< Font height in pixels */
        const uint16_t *const data;      /**< Pointer to font data array */
        const uint8_t *const char_width; /**< Proportional character width in pixels (NULL for monospaced) */
    } SSD1306_Font_t;

    /**************************************************************************************************\
    * Public memory declarations
    \**************************************************************************************************/
    extern QActive *const AO_SSD1306; // opaque pointer

    /**************************************************************************************************\
    * Public prototypes
    \**************************************************************************************************/
    void SSD1306_ctor(I2C_Write i2c_write_fn, I2C_Read i2c_read_fn);

#ifdef __cplusplus
}
#endif
#endif // SSD1306_AO_H