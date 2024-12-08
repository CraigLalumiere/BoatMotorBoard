#include "SSD1306.h"
#include "bsp.h"
#include "private_signal_ranges.h"
#include "pubsub_signals.h"
#include <stdio.h>
#include <string.h>

Q_DEFINE_THIS_MODULE("SSD1306")

/**************************************************************************************************\
* Private macros
\**************************************************************************************************/

// Success / Error
// ------------------------------------------------------------------------------------
#define SSD1306_SUCCESS           0
#define SSD1306_ERROR             1

// Address definition
// ------------------------------------------------------------------------------------
#define SSD1306_ADDR              0x3C

// Command definition
// ------------------------------------------------------------------------------------
#define SSD1306_COMMAND           0x80  // Continuation bit=1, D/C=0; 1000 0000
#define SSD1306_COMMAND_STREAM    0x00  // Continuation bit=0, D/C=0; 0000 0000
#define SSD1306_DATA              0xC0  // Continuation bit=1, D/C=1; 1100 0000
#define SSD1306_DATA_STREAM       0x40  // Continuation bit=0, D/C=1; 0100 0000

#define SSD1306_SET_MUX_RATIO     0xA8  // Set MUX ratio to N+1 MUX, N=A[5:0] : from 16MUX to 64MUX
#define SSD1306_DISPLAY_OFFSET    0xD3  // Set Display Offset
#define SSD1306_DISPLAY_ON        0xAF  // Display ON in normal mode
#define SSD1306_DISPLAY_OFF       0xAE  // Display OFF (sleep mode)
#define SSD1306_DIS_ENT_DISP_ON   0xA4  // Entire Display ON, Output ignores RAM content
#define SSD1306_DIS_IGNORE_RAM    0xA5  // Resume to RAM content display, Output follows RAM content
#define SSD1306_DIS_NORMAL        0xA6  // Normal display, 0 in RAM: OFF in display panel, 1 in RAM: ON in display panel
#define SSD1306_DIS_INVERSE       0xA7  // Inverse display, 0 in RAM: ON in display panel, 1 in RAM: OFF in display panel
#define SSD1306_DEACT_SCROLL      0x2E  // Stop scrolling that is configured by command 26h/27h/29h/2Ah
#define SSD1306_ACTIVE_SCROLL     0x2F  // Start scrolling that is configured by the scrolling setup commands:26h/27h/29h/2Ah
#define SSD1306_SET_START_LINE    0x40  // Set Display Start Line
#define SSD1306_MEMORY_ADDR_MODE  0x20  // Set Memory, Addressing Mode
#define SSD1306_SET_COLUMN_ADDR   0x21  // Set Column Address
#define SSD1306_SET_PAGE_ADDR     0x22  // Set Page Address 
#define SSD1306_SEG_REMAP         0xA0  // Set Segment Re-map, X[0]=0b column address 0 is mapped to SEG0
#define SSD1306_SEG_REMAP_OP      0xA1  // Set Segment Re-map, X[0]=1b: column address 127 is mapped to SEG0
#define SSD1306_COM_SCAN_DIR      0xC0  // Set COM Output, X[3]=0b: normal mode (RESET) Scan from COM0 to COM[N –1], e N is the Multiplex ratio
#define SSD1306_COM_SCAN_DIR_OP   0xC8  // Set COM Output, X[3]=1b: remapped mode. Scan from COM[N-1] to COM0, e N is the Multiplex ratio
#define SSD1306_COM_PIN_CONF      0xDA  // Set COM Pins Hardware Configuration, 
                                    // A[4]=0b, Sequential COM pin configuration, A[4]=1b(RESET), Alternative COM pin configuration
                                    // A[5]=0b(RESET), Disable COM Left/Right remap, A[5]=1b, Enable COM Left/Right remap                                        
#define SSD1306_SET_CONTRAST      0x81  // Set Contrast Control, Double byte command to select 1 to 256 contrast steps, increases as the value increases
#define SSD1306_SET_OSC_FREQ      0xD5  // Set Display Clock Divide Ratio/Oscillator Frequency
                                    // A[3:0] : Define the divide ratio (D) of the  display clocks (DCLK): Divide ratio= A[3:0] + 1, RESET is 0000b (divide ratio = 1) 
                                    // A[7:4] : Set the Oscillator Frequency, FOSC. Oscillator Frequency increases with the value of A[7:4] and vice versa. RESET is 1000b
#define SSD1306_SET_CHAR_REG      0x8D  // Charge Pump Setting, A[2] = 0b, Disable charge pump(RESET), A[2] = 1b, Enable charge pump during display on 
                                    // The Charge Pump must be enabled by the following command:
                                    // 8Dh ; Charge Pump Setting
                                    // 14h ; Enable Charge Pump
                                    // AFh; Display ON
#define SSD1306_SET_PRECHARGE     0xD9  // Set Pre-charge Period
#define SSD1306_VCOM_DESELECT     0xDB  // Set VCOMH Deselect Leve
#define SSD1306_NOP               0xE3  // No operation
#define SSD1306_RESET             0xE4  // Maybe SW RESET, @source https://github.com/SmingHub/Sming/issues/501

// Clear Color
// ------------------------------------------------------------------------------------
#define CLEAR_COLOR               0x00

// Init Status
// ------------------------------------------------------------------------------------
#define INIT_STATUS               0xFF

// AREA definition
// ------------------------------------------------------------------------------------
#define START_PAGE_ADDR           0
#define END_PAGE_ADDR             7     // 7 for 128x64, 3 for 128x32 version
#define START_COLUMN_ADDR         0
#define END_COLUMN_ADDR           127
#define RAM_X_END                 END_COLUMN_ADDR + 1
#define RAM_Y_END                 END_PAGE_ADDR + 1

#define CACHE_SIZE_MEM            (1 + END_PAGE_ADDR) * (1 + END_COLUMN_ADDR)

#define MAX_X                     END_COLUMN_ADDR
#define MAX_Y                     (END_PAGE_ADDR + 1) * 8

// Init Procedure
// ------------------------------------------------------------------------------------
#define NUM_BYTES_INIT 27


#define N_BYTES_I2C_DATA CACHE_SIZE_MEM


/**************************************************************************************************\
* Private type definitions
\**************************************************************************************************/

enum SSD1306Signals
{
    WAIT_TIMEOUT_SIG = PRIVATE_SIGNAL_SSD1306_START,
    I2C_COMPLETE_SIG,
    I2C_ERROR_SIG,
};

typedef enum
{
    HV_high,
    HV_low,
} HV_polarity_T;

typedef struct
{
    QActive super;  // inherit QActive
    QTimeEvt wait_evt; // timer to wait for voltage to settle
    I2C_Write i2c_write;
    I2C_Read i2c_read;
    uint8_t i2c_data[N_BYTES_I2C_DATA];

    uint8_t init_command_num; // Keeps track of which command we're on

    uint8_t _counter;

} SSD1306;

/**************************************************************************************************\
* Private memory declarations
\**************************************************************************************************/
static SSD1306 ssd1306_inst;
QActive *const AO_SSD1306 = &ssd1306_inst.super;

const uint8_t INIT_SEQUENCE[] = {
    SSD1306_COMMAND_STREAM,
    SSD1306_DISPLAY_OFF,
    SSD1306_SET_MUX_RATIO, 0x3F,                                // 0xA8 - 0x3F for 128 x 64 version (64MUX)
                                                                //      - 0x1F for 128 x 32 version (32MUX)
    SSD1306_MEMORY_ADDR_MODE, 0x00,                             // 0x20 = Set Memory Addressing Mode
                                                                // 0x00 - Horizontal Addressing Mode
                                                                // 0x01 - Vertical Addressing Mode
                                                                // 0x02 - Page Addressing Mode (RESET)
    SSD1306_SET_START_LINE,
    SSD1306_DISPLAY_OFFSET, 0x00,
    SSD1306_SEG_REMAP_OP,                                       // 0xA0 / remap 0xA1
    SSD1306_COM_SCAN_DIR_OP,                                    // 0xC0 / remap 0xC8
    SSD1306_COM_PIN_CONF, 0x12,                                 // 0xDA, 0x12 - Disable COM Left/Right remap, Alternative COM pin configuration
                                                                //       0x12 - for 128 x 64 version
                                                                //       0x02 - for 128 x 32 version
    SSD1306_SET_CONTRAST, 0x7F,                                 // 0x81, 0x7F - reset value (max 0xFF)
    SSD1306_DIS_ENT_DISP_ON,
    SSD1306_DIS_NORMAL,
    SSD1306_SET_OSC_FREQ, 0x80,                                 // 0xD5, 0x80 => D=1; DCLK = Fosc / D <=> DCLK = Fosc
    SSD1306_SET_PRECHARGE, 0xc2,                                // 0xD9, higher value less blinking
                                                                // 0xC2, 1st phase = 2 DCLK,  2nd phase = 13 DCLK
    SSD1306_VCOM_DESELECT, 0x20,                                // Set V COMH Deselect, reset value 0x22 = 0,77xUcc
    SSD1306_SET_CHAR_REG, 0x14,                                 // 0x8D, Enable charge pump during display on
    SSD1306_DEACT_SCROLL, 
    SSD1306_DISPLAY_ON,                                         // 0xAF = Set Display ON
};

static char cacheMemLcd[CACHE_SIZE_MEM];

/**************************************************************************************************\
* Private prototypes
\**************************************************************************************************/

// state handler functions
static QState initial(SSD1306 *const me, void const *const par);
static QState startup(SSD1306 *const me, QEvt const *const e);
static QState startup_error(SSD1306 *const me, QEvt const *const e);

static QState standby(SSD1306 *const me, QEvt const *const e);
static QState update_screen(SSD1306 *const me, QEvt const *const e);



static void I2C_Complete_CB(void *cb_data);
static void I2C_Error_CB(void *cb_data);

/**************************************************************************************************\
* Public functions
\**************************************************************************************************/

/**
 ***************************************************************************************************
 * @brief   Constructor
 **************************************************************************************************/
void SSD1306_ctor(I2C_Write i2c_write_fn, I2C_Read i2c_read_fn)
{
    SSD1306 *const me = &ssd1306_inst;

    me->i2c_write = i2c_write_fn;
    me->i2c_read = i2c_read_fn;

    memset (cacheMemLcd, 0x00, CACHE_SIZE_MEM);

    QActive_ctor(&me->super, Q_STATE_CAST(&initial));
    QTimeEvt_ctorX(&me->wait_evt, &me->super, WAIT_TIMEOUT_SIG, 0U);
}

/**************************************************************************************************\
* Private functions
\**************************************************************************************************/

/**
 ***************************************************************************************************
 * @brief   HSM
 **************************************************************************************************/
static QState initial(SSD1306 *const me, void const *const par)
{
    Q_UNUSED_PAR(par);
    // QActive_subscribe((QActive *) me, PUBSUB_TEST_CARTRIDGE_BEGIN);

    return Q_TRAN(&startup);
}

static QState startup(SSD1306 *const me, QEvt const *const e)
{
    QState status;

    switch (e->sig)
    {
        case Q_ENTRY_SIG: {

            memcpy(me->i2c_data, INIT_SEQUENCE, NUM_BYTES_INIT);

            I2C_Return_T retval = me->i2c_write(
                SSD1306_ADDR,
                me->i2c_data,
                NUM_BYTES_INIT,
                I2C_Complete_CB,
                I2C_Error_CB,
                &ssd1306_inst);

            if (retval != I2C_RTN_SUCCESS)
            {
                static QEvt const event = QEVT_INITIALIZER(I2C_ERROR_SIG);
                QACTIVE_POST(me, &event, me);
            }

            status = Q_HANDLED();
            break;
        }
        case I2C_COMPLETE_SIG: {
            status = Q_TRAN(&update_screen);
            break;
        }
        case I2C_ERROR_SIG: {
            status = Q_TRAN(&startup_error);
            break;
        }
        default: {
            status = Q_SUPER(&QHsm_top);
            break;
        }
    }

    return status;
}


static QState startup_error(SSD1306 *const me, QEvt const *const e)
{
    QState status;

    switch (e->sig)
    {
        case Q_ENTRY_SIG: {
            status = Q_HANDLED();
            break;
        }

        default: {
            status = Q_SUPER(&startup);
            break;
        }
    }

    return status;
}





static QState standby(SSD1306 *const me, QEvt const *const e)
{
    QState status;

    switch (e->sig)
    {
        case Q_ENTRY_SIG: {
            status = Q_HANDLED();
            break;
        }
        default: {
            status = Q_SUPER(&QHsm_top);
            break;
        }
    }

    return status;
}


static QState update_screen(SSD1306 *const me, QEvt const *const e)
{
    QState status;

    switch (e->sig)
    {
        case Q_ENTRY_SIG: {

            memcpy(me->i2c_data, *cacheMemLcd, CACHE_SIZE_MEM);

            I2C_Return_T retval = me->i2c_write(
                SSD1306_ADDR,
                me->i2c_data,
                CACHE_SIZE_MEM,
                I2C_Complete_CB,
                I2C_Error_CB,
                &ssd1306_inst);

            if (retval != I2C_RTN_SUCCESS)
            {
                static QEvt const event = QEVT_INITIALIZER(I2C_ERROR_SIG);
                QACTIVE_POST(me, &event, me);
            }

            status = Q_HANDLED();
            break;
        }
        case I2C_COMPLETE_SIG: {
            status = Q_TRAN(&standby);
            break;
        }
        case I2C_ERROR_SIG: {
            status = Q_TRAN(&startup_error);
            break;
        }
        default: {
            status = Q_SUPER(&QHsm_top);
            break;
        }
    }

    return status;
}




/**
 ***************************************************************************************************
 *
 * @brief   I2C callback, called by external context.
 *
 **************************************************************************************************/
static void I2C_Complete_CB(void *cb_data)
{
    static QEvt const event = QEVT_INITIALIZER(I2C_COMPLETE_SIG);

    QActive *me = (QActive *) cb_data;
    QACTIVE_POST(me, &event, &me);
}

/**
 ***************************************************************************************************
 *
 * @brief   I2C callback, called by external context.
 *
 **************************************************************************************************/
static void I2C_Error_CB(void *cb_data)
{
    static QEvt const event = QEVT_INITIALIZER(I2C_ERROR_SIG);

    QActive *me = (QActive *) cb_data;
    QACTIVE_POST(me, &event, me);
}
