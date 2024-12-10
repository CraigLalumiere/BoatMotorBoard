#include "SSD1306.h"
#include "bsp.h"
#include "private_signal_ranges.h"
#include "pubsub_signals.h"
#include <stdio.h>
#include <string.h>
#include "font.h"

Q_DEFINE_THIS_MODULE("SSD1306")

/**************************************************************************************************\
* Private macros
\**************************************************************************************************/

// Success / Error
// ------------------------------------------------------------------------------------
#define SSD1306_SUCCESS 0
#define SSD1306_ERROR 1

// Address definition
// ------------------------------------------------------------------------------------
#define SSD1306_ADDR 0x3C

// Command definition
// ------------------------------------------------------------------------------------
#define SSD1306_COMMAND 0x80        // Continuation bit=1, D/C=0; 1000 0000
#define SSD1306_COMMAND_STREAM 0x00 // Continuation bit=0, D/C=0; 0000 0000
#define SSD1306_DATA 0xC0           // Continuation bit=1, D/C=1; 1100 0000
#define SSD1306_DATA_STREAM 0x40    // Continuation bit=0, D/C=1; 0100 0000

#define SSD1306_SET_MUX_RATIO 0xA8    // Set MUX ratio to N+1 MUX, N=A[5:0] : from 16MUX to 64MUX
#define SSD1306_DISPLAY_OFFSET 0xD3   // Set Display Offset
#define SSD1306_DISPLAY_ON 0xAF       // Display ON in normal mode
#define SSD1306_DISPLAY_OFF 0xAE      // Display OFF (sleep mode)
#define SSD1306_DIS_ENT_DISP_ON 0xA4  // Entire Display ON, Output ignores RAM content
#define SSD1306_DIS_IGNORE_RAM 0xA5   // Resume to RAM content display, Output follows RAM content
#define SSD1306_DIS_NORMAL 0xA6       // Normal display, 0 in RAM: OFF in display panel, 1 in RAM: ON in display panel
#define SSD1306_DIS_INVERSE 0xA7      // Inverse display, 0 in RAM: ON in display panel, 1 in RAM: OFF in display panel
#define SSD1306_DEACT_SCROLL 0x2E     // Stop scrolling that is configured by command 26h/27h/29h/2Ah
#define SSD1306_ACTIVE_SCROLL 0x2F    // Start scrolling that is configured by the scrolling setup commands:26h/27h/29h/2Ah
#define SSD1306_SET_START_LINE 0x40   // Set Display Start Line
#define SSD1306_MEMORY_ADDR_MODE 0x20 // Set Memory, Addressing Mode
#define SSD1306_SET_COLUMN_ADDR 0x21  // Set Column Address
#define SSD1306_SET_PAGE_ADDR 0x22    // Set Page Address
#define SSD1306_SEG_REMAP 0xA0        // Set Segment Re-map, X[0]=0b column address 0 is mapped to SEG0
#define SSD1306_SEG_REMAP_OP 0xA1     // Set Segment Re-map, X[0]=1b: column address 127 is mapped to SEG0
#define SSD1306_COM_SCAN_DIR 0xC0     // Set COM Output, X[3]=0b: normal mode (RESET) Scan from COM0 to COM[N –1], e N is the Multiplex ratio
#define SSD1306_COM_SCAN_DIR_OP 0xC8  // Set COM Output, X[3]=1b: remapped mode. Scan from COM[N-1] to COM0, e N is the Multiplex ratio
#define SSD1306_COM_PIN_CONF 0xDA     // Set COM Pins Hardware Configuration,
                                      // A[4]=0b, Sequential COM pin configuration, A[4]=1b(RESET), Alternative COM pin configuration
                                      // A[5]=0b(RESET), Disable COM Left/Right remap, A[5]=1b, Enable COM Left/Right remap
#define SSD1306_SET_CONTRAST 0x81     // Set Contrast Control, Double byte command to select 1 to 256 contrast steps, increases as the value increases
#define SSD1306_SET_OSC_FREQ 0xD5     // Set Display Clock Divide Ratio/Oscillator Frequency
                                      // A[3:0] : Define the divide ratio (D) of the  display clocks (DCLK): Divide ratio= A[3:0] + 1, RESET is 0000b (divide ratio = 1)
                                      // A[7:4] : Set the Oscillator Frequency, FOSC. Oscillator Frequency increases with the value of A[7:4] and vice versa. RESET is 1000b
#define SSD1306_SET_CHAR_REG 0x8D     // Charge Pump Setting, A[2] = 0b, Disable charge pump(RESET), A[2] = 1b, Enable charge pump during display on
                                      // The Charge Pump must be enabled by the following command:
                                      // 8Dh ; Charge Pump Setting
                                      // 14h ; Enable Charge Pump
                                      // AFh; Display ON
#define SSD1306_SET_PRECHARGE 0xD9    // Set Pre-charge Period
#define SSD1306_VCOM_DESELECT 0xDB    // Set VCOMH Deselect Leve
#define SSD1306_NOP 0xE3              // No operation
#define SSD1306_RESET 0xE4            // Maybe SW RESET, @source https://github.com/SmingHub/Sming/issues/501

// Clear Color
// ------------------------------------------------------------------------------------
#define CLEAR_COLOR 0x00

// Init Status
// ------------------------------------------------------------------------------------
#define INIT_STATUS 0xFF

// AREA definition
// ------------------------------------------------------------------------------------
#define START_PAGE_ADDR 0
#define END_PAGE_ADDR 7 // 7 for 128x64, 3 for 128x32 version
#define START_COLUMN_ADDR 0
#define END_COLUMN_ADDR 127
#define RAM_X_END END_COLUMN_ADDR + 1
#define RAM_Y_END END_PAGE_ADDR + 1

#define WIDTH 128
#define HEIGHT 64
#define CACHE_SIZE_MEM (WIDTH * ((HEIGHT + 7) / 8))
//(1 + END_PAGE_ADDR) * (1 + END_COLUMN_ADDR)

#define MAX_X END_COLUMN_ADDR
#define MAX_Y (END_PAGE_ADDR + 1) * 8

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
    QActive super;     // inherit QActive
    QTimeEvt wait_evt; // timer to wait for voltage to settle
    I2C_Write i2c_write;
    I2C_Read i2c_read;
    uint8_t i2c_data[N_BYTES_I2C_DATA];

    uint8_t _counter;

    uint16_t xfer_counter;
    char *xfer_ptr;

    char cacheMemLcd[CACHE_SIZE_MEM + 1];

    // submachine state memory
    QStateHandler substate_next_state;
    QStateHandler substate_super_state;
    uint8_t command;
    uint8_t num_args;
    uint8_t *args;

    uint8_t init_command_num;
    uint8_t *init_command_ptr;

} SSD1306;

/**************************************************************************************************\
* Private memory declarations
\**************************************************************************************************/
static SSD1306 ssd1306_inst;
QActive *const AO_SSD1306 = &ssd1306_inst.super;

// Matiasus sequence https://github.com/Matiasus/SSD1306/tree/master
// const uint8_t INIT_SEQUENCE[] = {
//     SSD1306_COMMAND_STREAM,
//     SSD1306_DISPLAY_OFF,
//     SSD1306_SET_MUX_RATIO, 0x3F,    // 0xA8 - 0x3F for 128 x 64 version (64MUX)
//                                     //      - 0x1F for 128 x 32 version (32MUX)
//     SSD1306_MEMORY_ADDR_MODE, 0x00, // 0x20 = Set Memory Addressing Mode
//                                     // 0x00 - Horizontal Addressing Mode
//                                     // 0x01 - Vertical Addressing Mode
//                                     // 0x02 - Page Addressing Mode (RESET)
//     SSD1306_SET_START_LINE,
//     SSD1306_DISPLAY_OFFSET, 0x00,
//     SSD1306_SEG_REMAP_OP,       // 0xA0 / remap 0xA1
//     SSD1306_COM_SCAN_DIR_OP,    // 0xC0 / remap 0xC8
//     SSD1306_COM_PIN_CONF, 0x12, // 0xDA, 0x12 - Disable COM Left/Right remap, Alternative COM pin configuration
//                                 //       0x12 - for 128 x 64 version
//                                 //       0x02 - for 128 x 32 version
//     SSD1306_SET_CONTRAST, 0x7F, // 0x81, 0x7F - reset value (max 0xFF)
//     SSD1306_DIS_ENT_DISP_ON,
//     SSD1306_DIS_NORMAL,
//     SSD1306_SET_OSC_FREQ, 0x80,  // 0xD5, 0x80 => D=1; DCLK = Fosc / D <=> DCLK = Fosc
//     SSD1306_SET_PRECHARGE, 0xc2, // 0xD9, higher value less blinking
//                                  // 0xC2, 1st phase = 2 DCLK,  2nd phase = 13 DCLK
//     SSD1306_VCOM_DESELECT, 0x20, // Set V COMH Deselect, reset value 0x22 = 0,77xUcc
//     SSD1306_SET_CHAR_REG, 0x14,  // 0x8D, Enable charge pump during display on
//     SSD1306_DEACT_SCROLL,
//     SSD1306_DISPLAY_ON, // 0xAF = Set Display ON
// };

// Adafruit sequence
const uint8_t INIT_SEQUENCE[] = {
    SSD1306_COMMAND_STREAM,
    SSD1306_DISPLAY_OFF,
    SSD1306_SET_OSC_FREQ,
    0x80,
    SSD1306_SET_MUX_RATIO,
    0x3F,
    SSD1306_DISPLAY_OFFSET,
    0x00,
    SSD1306_SET_START_LINE,
    SSD1306_SET_CHAR_REG,
    0x14,
    SSD1306_MEMORY_ADDR_MODE,
    0,
    SSD1306_SEG_REMAP_OP,
    SSD1306_COM_SCAN_DIR_OP,
    SSD1306_COM_PIN_CONF,
    0x12,
    SSD1306_SET_CONTRAST,
    0xCF,
    SSD1306_SET_PRECHARGE,
    0xF1,
    SSD1306_VCOM_DESELECT,
    0x40,
    SSD1306_DIS_ENT_DISP_ON,
    SSD1306_DIS_NORMAL,
    SSD1306_DEACT_SCROLL,
    SSD1306_DISPLAY_ON,
};

#define NUM_INIT_COMMANDS 17
const uint8_t INIT_SSD1306_ADAFRUIT[] = {
    SSD1306_DISPLAY_OFF, 0,            // 0xAE / Set Display OFF
    SSD1306_SET_OSC_FREQ, 1, 0x80,     // 0xD5 / 0x80 => D=1; DCLK = Fosc / D <=> DCLK = Fosc
    SSD1306_SET_MUX_RATIO, 1, 0x3F,    // 0xA8 / 0x3F (64MUX) for 128 x 64 version
                                       //      / 0x1F (32MUX) for 128 x 32 version
    SSD1306_DISPLAY_OFFSET, 1, 0x00,   // 0xD3
    SSD1306_SET_START_LINE, 0,         // 0x40
    SSD1306_SET_CHAR_REG, 1, 0x14,     // 0x8D / Enable charge pump during display on
    SSD1306_MEMORY_ADDR_MODE, 1, 0x00, // 0x20 / Set Memory Addressing Mode
                                       // 0x00 / Horizontal Addressing Mode
                                       // 0x01 / Vertical Addressing Mode
                                       // 0x02 /  Page Addressing Mode (RESET)
    SSD1306_SEG_REMAP_OP, 0,           // 0xA0 / remap 0xA1
    SSD1306_COM_SCAN_DIR_OP, 0,        // 0xC8
    SSD1306_COM_PIN_CONF, 1, 0x12,     // 0xDA / 0x12 - Disable COM Left/Right remap, Alternative COM pin configuration
                                       //        0x12 - for 128 x 64 version
                                       //        0x02 - for 128 x 32 version
    SSD1306_SET_CONTRAST, 1, 0xCF,     // 0x81 / 0x8F - reset value (max 0xFF)
    SSD1306_SET_PRECHARGE, 1, 0xF1,    // 0xD9 / higher value less blinking
                                       //        0xC2, 1st phase = 2 DCLK,  2nd phase = 13 DCLK
    SSD1306_VCOM_DESELECT, 1, 0x40,    // 0xDB / Set V COMH Deselect, reset value 0x22 = 0,77xUcc
    SSD1306_DIS_ENT_DISP_ON, 0,        // 0xA4
    SSD1306_DIS_NORMAL, 0,             // 0xA6
    SSD1306_DEACT_SCROLL, 0,           // 0x2E
    SSD1306_DISPLAY_ON, 0              // 0xAF / Set Display ON
};
/**************************************************************************************************\
* Private prototypes
\**************************************************************************************************/

// state handler functions
static QState initial(SSD1306 *const me, void const *const par);
static QState startup(SSD1306 *const me, QEvt const *const e);
static QState startup_error(SSD1306 *const me, QEvt const *const e);

static QState standby(SSD1306 *const me, QEvt const *const e);
static QState update_screen_1(SSD1306 *const me, QEvt const *const e);
static QState update_screen_2(SSD1306 *const me, QEvt const *const e);

static QState substate_send_command(SSD1306 *const me, QEvt const *const e);

static QState send_command_substate_machine(
    QStateHandler super_state,
    QStateHandler next_state,
    uint8_t command,
    uint8_t numArgs,
    uint8_t *args);

static void I2C_Complete_CB(void *cb_data);
static void I2C_Error_CB(void *cb_data);

void SSD1306_ClearScreen(void);
//   uint8_t SSD1306_NormalScreen (uint8_t);
//   uint8_t SSD1306_InverseScreen (uint8_t);
uint8_t SSD1306_UpdatePosition(void);
void SSD1306_SetPosition(uint8_t, uint8_t);
uint8_t SSD1306_DrawChar(char);
void SSD1306_DrawString(char *);
uint8_t SSD1306_DrawPixel(uint8_t, uint8_t);
uint8_t SSD1306_DrawLine(uint8_t, uint8_t, uint8_t, uint8_t);
void drawFastHLineInternal(uint8_t, uint8_t, uint8_t, uint8_t);
void AdafruitDrawPixel(uint16_t x, uint16_t y, uint16_t color);

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

    SSD1306_ClearScreen();

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
    case Q_INIT_SIG:
    {
        QStateHandler nextState = (QStateHandler) &startup;
        // If we've transmitted all of the startup commands
        if (me->init_command_num == NUM_INIT_COMMANDS - 1)
        {
            nextState = (QStateHandler) (&update_screen_1);
        }
        uint8_t command = me->init_command_ptr[0];
        uint8_t num_args = me->init_command_ptr[1];
        uint8_t *args = me->init_command_ptr + 2;
        // Move into substate machine
        status = send_command_substate_machine(
            (QStateHandler)&startup,
            nextState,
            command,
            num_args,
            args);
        me->init_command_ptr += 2 + num_args;
        me->init_command_num++;
        break;
    }
    case Q_ENTRY_SIG:
    {
        me->init_command_ptr = INIT_SSD1306_ADAFRUIT;
        me->init_command_num = 0;

        status = Q_HANDLED();
        break;
    }
    case I2C_ERROR_SIG:
    {
        status = Q_TRAN(&startup_error);
        break;
    }
    default:
    {
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
    case Q_ENTRY_SIG:
    {
        status = Q_HANDLED();
        break;
    }

    default:
    {
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
    case Q_ENTRY_SIG:
    {
        status = Q_HANDLED();
        break;
    }
    default:
    {
        status = Q_SUPER(&QHsm_top);
        break;
    }
    }

    return status;
}

static QState update_screen_1(SSD1306 *const me, QEvt const *const e)
{
    QState status;

    switch (e->sig)
    {
    case Q_ENTRY_SIG:
    {
        me->xfer_counter = 0;
        me->xfer_ptr = me->cacheMemLcd;

        SSD1306_ClearScreen();                // clear screen
        SSD1306_DrawLine(0, MAX_X, 4, 4);     // draw line
        SSD1306_DrawLine(0, MAX_X, 0, MAX_Y); // draw line
        // SSD1306_DrawLine(0, MAX_X, 4, 4); // draw line
        // AdafruitDrawPixel(10, 10, 1);
        // AdafruitDrawPixel(15, 15, 1);
        SSD1306_SetPosition(7, 10); // set position
        // SSD1306_DrawString("SSD1306 OLED DRIVER"); // draw string
        SSD1306_DrawString("XXXXXXXXXXX"); // draw string

        memset(me->i2c_data, 0, sizeof(me->i2c_data));
        me->i2c_data[0] = 0x00; // Begin command
        me->i2c_data[1] = SSD1306_SET_PAGE_ADDR;
        me->i2c_data[2] = 0x00; // Start address
        me->i2c_data[3] = 0xFF; // End  addresss
        me->i2c_data[4] = SSD1306_SET_COLUMN_ADDR;
        me->i2c_data[5] = 0x00; // Start address
        me->i2c_data[6] = 0x7F; // End  addresss

        // me->i2c_data[0] = SSD1306_DATA_STREAM;
        // memcpy(me->i2c_data + 1, me->cacheMemLcd, CACHE_SIZE_MEM);

        I2C_Return_T retval = me->i2c_write(
            SSD1306_ADDR,
            me->i2c_data,
            7,
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
    case I2C_COMPLETE_SIG:
    {
        status = Q_TRAN(&update_screen_2);
        break;
    }
    case I2C_ERROR_SIG:
    {
        status = Q_TRAN(&startup_error);
        break;
    }
    default:
    {
        status = Q_SUPER(&QHsm_top);
        break;
    }
    }

    return status;
}

static QState update_screen_2(SSD1306 *const me, QEvt const *const e)
{
    QState status;

    switch (e->sig)
    {
    case Q_ENTRY_SIG:
    {

        me->i2c_data[0] = SSD1306_DATA_STREAM;
        memcpy(me->i2c_data + 1, me->xfer_ptr, 256);
        // memset(me->i2c_data, 0x00, sizeof(me->i2c_data));
        me->xfer_ptr += 256;

        I2C_Return_T retval = me->i2c_write(
            SSD1306_ADDR,
            me->i2c_data,
            257,
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
    case I2C_COMPLETE_SIG:
    {
        status = Q_TRAN(&update_screen_2);
        break;
    }
    case I2C_ERROR_SIG:
    {
        status = Q_TRAN(&startup_error);
        break;
    }
    default:
    {
        status = Q_SUPER(&QHsm_top);
        break;
    }
    }

    return status;
}

/**
 ***************************************************************************************************
 * @brief   Helper function to initiate substate machine
 **************************************************************************************************/
static QState send_command_substate_machine(
    QStateHandler super_state,
    QStateHandler next_state,
    uint8_t command,
    uint8_t numArgs,
    uint8_t *args)
{
    SSD1306 *me = &ssd1306_inst;

    me->substate_super_state = super_state;
    me->substate_next_state = next_state;
    me->num_args = numArgs;
    me->command = command;
    me->args = args;

    return Q_TRAN(&substate_send_command);
}

/**
 ***************************************************************************************************
 * @brief   Third state of 'write' substate: Get the response to the 'read register' command, wait
 *for callback
 **************************************************************************************************/

static QState substate_send_command(SSD1306 *const me, QEvt const *const e)
{
    QState status;

    switch (e->sig)
    {
    case Q_ENTRY_SIG:
    {
        me->i2c_data[0] = SSD1306_COMMAND_STREAM;
        me->i2c_data[1] = me->command;
        memcpy(me->i2c_data + 2, me->args, me->num_args);

        I2C_Return_T retval = me->i2c_write(
            SSD1306_ADDR,
            me->i2c_data,
            2 + me->num_args,
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
    case I2C_COMPLETE_SIG:
    {
        status = Q_TRAN(me->substate_next_state);
        break;
    }

    default:
    {
        status = Q_SUPER(me->substate_super_state);
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

    QActive *me = (QActive *)cb_data;
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

    QActive *me = (QActive *)cb_data;
    QACTIVE_POST(me, &event, me);
}

/**
 * @brief   SSD1306 Clear screen
 *
 * @param   void
 *
 * @return  void
 */
void SSD1306_ClearScreen(void)
{
    SSD1306 *me = &ssd1306_inst;
    memset(me->cacheMemLcd, 0x00, CACHE_SIZE_MEM); // null cache memory lcd
}

/**
 * @brief   SSD1306 Set position
 *
 * @param   uint8_t column -> 0 ... 127
 * @param   uint8_t page -> 0 ... 7 or 3
 *
 * @return  void
 */
void SSD1306_SetPosition(uint8_t x, uint8_t y)
{
    SSD1306 *me = &ssd1306_inst;
    me->_counter = x + (y << 7); // update counter
}

/**
 * @brief   SSD1306 Update text poisition - this ensure that character will not be divided at the end of row,
 *          the whole character will be depicted on the new row
 *
 * @param   void
 *
 * @return  uint8_t
 */
uint8_t SSD1306_UpdatePosition(void)
{
    SSD1306 *me = &ssd1306_inst;
    uint8_t y = me->_counter >> 7;             // y / 8
    uint8_t x = me->_counter - (y << 7);       // y % 8
    uint8_t x_new = x + CHARS_COLS_LENGTH + 1; // x + character length + 1

    if (x_new > END_COLUMN_ADDR)
    { // check position
        if (y > END_PAGE_ADDR)
        {                         // if more than allowable number of pages
            return SSD1306_ERROR; // return out of range
        }
        else if (y < (END_PAGE_ADDR - 1))
        {                                // if x reach the end but page in range
            me->_counter = ((++y) << 7); // update
        }
    }

    return SSD1306_SUCCESS;
}

/**
 * @brief   SSD1306 Draw character
 *
 * @param   char character
 *
 * @return  uint8_t
 */
uint8_t SSD1306_DrawChar(char character)
{
    SSD1306 *me = &ssd1306_inst;
    uint8_t i = 0;

    if (SSD1306_UpdatePosition() == SSD1306_ERROR)
    {
        return SSD1306_ERROR;
    }
    while (i < CHARS_COLS_LENGTH)
    {
        me->cacheMemLcd[me->_counter++] = &FONTS[character - 32][i++];
    }
    me->_counter++;

    return SSD1306_SUCCESS;
}

/**
 * @brief   SSD1306 Draw String
 *
 * @param   char * string
 *
 * @return  void
 */
void SSD1306_DrawString(char *str)
{
    int i = 0;
    while (str[i] != '\0')
    {
        SSD1306_DrawChar(str[i++]);
    }
}

/**
 * @brief   Draw pixel
 *
 * @param   uint8_t x -> 0 ... MAX_X
 * @param   uint8_t y -> 0 ... MAX_Y
 *
 * @return  uint8_t
 */
uint8_t SSD1306_DrawPixel(uint8_t x, uint8_t y)
{
    SSD1306 *me = &ssd1306_inst;
    uint8_t page = 0;
    uint8_t pixel = 0;

    if ((x > MAX_X) || (y > MAX_Y))
    {                         // if out of range
        return SSD1306_ERROR; // out of range
    }
    page = y >> 3;                            // find page (y / 8)
    pixel = 1 << (y - (page << 3));           // which pixel (y % 8)
    me->_counter = x + (page << 7);           // update counter
    me->cacheMemLcd[me->_counter++] |= pixel; // save pixel

    return SSD1306_SUCCESS;
}

void AdafruitDrawPixel(uint16_t x, uint16_t y, uint16_t color)
{
    SSD1306 *const me = &ssd1306_inst;
    switch (color)
    {
    case 1:
        me->cacheMemLcd[x + (y / 8) * WIDTH] |= (1 << (y & 7));
        break;
    case 0:
        me->cacheMemLcd[x + (y / 8) * WIDTH] &= ~(1 << (y & 7));
        break;
    case 2:
        me->cacheMemLcd[x + (y / 8) * WIDTH] ^= (1 << (y & 7));
        break;
    }
}

/**
 * @brief   Draw line by Bresenham algoritm
 *
 * @param   uint8_t x start position / 0 <= cols <= MAX_X-1
 * @param   uint8_t x end position   / 0 <= cols <= MAX_X-1
 * @param   uint8_t y start position / 0 <= rows <= MAX_Y-1
 * @param   uint8_t y end position   / 0 <= rows <= MAX_Y-1
 *
 * @return  uint8_t
 */
uint8_t SSD1306_DrawLine(uint8_t x1, uint8_t x2, uint8_t y1, uint8_t y2)
{
    int16_t D;                        // determinant
    int16_t delta_x, delta_y;         // deltas
    int16_t trace_x = 1, trace_y = 1; // steps

    delta_x = x2 - x1; // delta x
    delta_y = y2 - y1; // delta y

    if (delta_x < 0)
    {                       // check if x2 > x1
        delta_x = -delta_x; // negate delta x
        trace_x = -trace_x; // negate step x
    }

    if (delta_y < 0)
    {                       // check if y2 > y1
        delta_y = -delta_y; // negate detla y
        trace_y = -trace_y; // negate step y
    }

    // Bresenham condition for m < 1 (dy < dx)
    // -------------------------------------------------------------------------------------
    if (delta_y < delta_x)
    {
        D = (delta_y << 1) - delta_x; // calculate determinant
        SSD1306_DrawPixel(x1, y1);    // draw first pixel
        while (x1 != x2)
        {                  // check if x1 equal x2
            x1 += trace_x; // update x1
            if (D >= 0)
            {                     // check if determinant is positive
                y1 += trace_y;    // update y1
                D -= 2 * delta_x; // update determinant
            }
            D += 2 * delta_y;          // update deteminant
            SSD1306_DrawPixel(x1, y1); // draw next pixel
        }
        // for m > 1 (dy > dx)
        // -------------------------------------------------------------------------------------
    }
    else
    {
        D = delta_y - (delta_x << 1); // calculate determinant
        SSD1306_DrawPixel(x1, y1);    // draw first pixel
        while (y1 != y2)
        {                  // check if y2 equal y1
            y1 += trace_y; // update y1
            if (D <= 0)
            {                     // check if determinant is positive
                x1 += trace_x;    // update y1
                D += 2 * delta_y; // update determinant
            }
            D -= 2 * delta_x;          // update deteminant
            SSD1306_DrawPixel(x1, y1); // draw next pixel
        }
    }

    return SSD1306_SUCCESS;
}

void drawFastHLineInternal(uint8_t x, uint8_t y, uint8_t w,
                           uint8_t color)
{

    SSD1306 *const me = &ssd1306_inst;
    if ((y >= 0) && (y < HEIGHT))
    { // Y coord in bounds?
        if (x < 0)
        { // Clip left
            w += x;
            x = 0;
        }
        if ((x + w) > WIDTH)
        { // Clip right
            w = (WIDTH - x);
        }
        if (w > 0)
        { // Proceed only if width is positive
            uint8_t *pBuf = me->cacheMemLcd[(y / 8) * WIDTH + x], mask = 1 << (y & 7);
            switch (color)
            {
            case 1:
                while (w--)
                {
                    *pBuf++ |= mask;
                };
                break;
            case 0:
                mask = ~mask;
                while (w--)
                {
                    *pBuf++ &= mask;
                };
                break;
            case 2:
                while (w--)
                {
                    *pBuf++ ^= mask;
                };
                break;
            }
        }
    }
}