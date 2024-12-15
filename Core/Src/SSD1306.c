// Based on this library:
// https://github.com/afiskon/stm32-ssd1306/tree/master

// Datasheet for OLED control IC:
// https://cdn-shop.adafruit.com/datasheets/SSD1306.pdf

#include "SSD1306.h"
#include "bsp.h"
#include "private_signal_ranges.h"
#include "pubsub_signals.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "ssd1306_fonts.h"
#include "LMT01.h"

#ifdef Q_SPY
Q_DEFINE_THIS_MODULE("SSD1306")
#endif // def Q_SPY

/**************************************************************************************************\
* Private macros
\**************************************************************************************************/

#define OLED_FPS 30
#define COLOR_CHANGE_SECONDS 5

#define FAULT_STR_LEN 512

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

// Screen dimensions
// ------------------------------------------------------------------------------------
#define SSD1306_WIDTH 128
#define SSD1306_HEIGHT 64
#define SSD1306_BUFFER_SIZE SSD1306_WIDTH *SSD1306_HEIGHT / 8

// Init Procedure
// ------------------------------------------------------------------------------------

#define N_BYTES_I2C_DATA 1 + SSD1306_WIDTH

/**************************************************************************************************\
* Private type definitions
\**************************************************************************************************/

enum SSD1306Signals
{
    WAIT_TIMEOUT_SIG = PRIVATE_SIGNAL_SSD1306_START,
    COLOR_CHANGE_SIG,
    I2C_COMPLETE_SIG,
    I2C_ERROR_SIG,
};

typedef enum
{
    SSD1306_OK = 0x00,
    SSD1306_ERR = 0x01 // Generic error.
} SSD1306_Error_t;

typedef struct
{
    uint8_t x;
    uint8_t y;
} SSD1306_VERTEX;

// Enumeration for screen colors
typedef enum
{
    Black = 0x00, // Black color, no pixel
    White = 0x01  // Pixel is set. Color depends on OLED
} SSD1306_COLOR;

typedef struct
{
    QActive super;              // inherit QActive
    QTimeEvt screen_update_evt; // timer to set screen refresh rate
    QTimeEvt color_update_evt;  // timer to change screen from light to dark mode (for burn in prevention)
    I2C_Write i2c_write;
    I2C_Read i2c_read;
    uint8_t i2c_data[N_BYTES_I2C_DATA];
    uint16_t counter;

    SSD1306_COLOR text_color;

    int16_t temperature;
    int16_t pressure;
    int16_t tachometer;
    int16_t vbat;
    bool start;
    bool neutral;
    bool buzzer;
    Colored_Wire_Stat_T red;
    Colored_Wire_Stat_T orange;

    char SSD1306_Buffer[SSD1306_BUFFER_SIZE];

    // submachine state memory
    QStateHandler substate_next_state;
    QStateHandler substate_super_state;
    uint8_t command;
    uint8_t num_args;
    uint8_t *args;

    // Variables used by startup routine
    uint8_t init_command_num;
    uint8_t *init_command_ptr;

    // Variables used by 'update screen' routine
    uint8_t pageNumber;

    // Variables used by the graphics algorithms
    uint16_t CurrentX;
    uint16_t CurrentY;

    // Variables used by the fault printing
    uint16_t fault_str_scroll_offset;

} SSD1306;

/**************************************************************************************************\
* Private memory declarations
\**************************************************************************************************/
static SSD1306 ssd1306_inst;
QActive *const AO_SSD1306 = &ssd1306_inst.super;

#define NUM_INIT_COMMANDS 18
const uint8_t INIT_SSD1306_ST[] = {
    SSD1306_DISPLAY_OFF, 0,            // 0xAE / Set Display OFF
    SSD1306_MEMORY_ADDR_MODE, 1, 0x00, // 0x20 / Set Memory Addressing Mode
    SSD1306_COM_SCAN_DIR_OP, 0,        // 0xC8
    0x00, 0,                           //---set low column address
    0x10, 0,                           //---set high column address
    0x10, 0,                           //--set start line address - CHECK
    SSD1306_SET_CONTRAST, 1, 0x8F,     // 0x81 / 0x8F - reset value (max 0xFF)
    SSD1306_SEG_REMAP_OP, 0,           // 0xA0 / remap 0xA1
    SSD1306_DIS_NORMAL, 0,             // 0xA6
    SSD1306_SET_MUX_RATIO, 1, 0x3F,    // 0xA8 / 0x3F (64MUX) for 128 x 64 version
                                       //      / 0x1F (32MUX) for 128 x 32 version
    SSD1306_DIS_ENT_DISP_ON, 0,        // 0xA4
    SSD1306_DISPLAY_OFFSET, 1, 0x00,   // 0xD3
    SSD1306_SET_OSC_FREQ, 1, 0xF0,     // 0xD5 / 0x80 => D=1; DCLK = Fosc / D <=> DCLK = Fosc
    SSD1306_SET_PRECHARGE, 1, 0x10,    // 0xD9 / higher value less blinking
                                       //        0xC2, 1st phase = 2 DCLK,  2nd phase = 13 DCLK
    SSD1306_COM_PIN_CONF, 1, 0x12,     // 0xDA / 0x12 - Disable COM Left/Right remap, Alternative COM pin configuration
                                       //        0x12 - for 128 x 64 version
                                       //        0x02 - for 128 x 32 version
    SSD1306_VCOM_DESELECT, 1, 0x20,    // 0xDB / Set V COMH Deselect, reset value 0x22 = 0,77xUcc
    SSD1306_SET_CHAR_REG, 1, 0x14,     // 0x8D / Enable charge pump during display on
    SSD1306_DISPLAY_ON, 0,             // 0xAF / Set Display ON
};
/**************************************************************************************************\
* Private prototypes
\**************************************************************************************************/

// state handler functions
static QState initial(SSD1306 *const me, void const *const par);
static QState startup(SSD1306 *const me, QEvt const *const e);
static QState startup_error(SSD1306 *const me, QEvt const *const e);
static QState error(SSD1306 *const me, QEvt const *const e);

static QState top(SSD1306 *const me, QEvt const *const e);
static QState waiting(SSD1306 *const me, QEvt const *const e);
static QState update_screen(SSD1306 *const me, QEvt const *const e);
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

static void drawFaultTextToScreen();

// Procedure definitions
void ssd1306_Fill(SSD1306_COLOR color);
void ssd1306_DrawPixel(uint8_t x, uint8_t y, SSD1306_COLOR color);
char ssd1306_WriteChar(char ch, SSD1306_Font_t Font, SSD1306_COLOR color);
char ssd1306_WriteString(char *str, SSD1306_Font_t Font, SSD1306_COLOR color);
void ssd1306_SetCursor(uint8_t x, uint8_t y);
void ssd1306_Line(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, SSD1306_COLOR color);
void ssd1306_DrawArc(uint8_t x, uint8_t y, uint8_t radius, uint16_t start_angle, uint16_t sweep, SSD1306_COLOR color);
void ssd1306_DrawArcWithRadiusLine(uint8_t x, uint8_t y, uint8_t radius, uint16_t start_angle, uint16_t sweep, SSD1306_COLOR color);
void ssd1306_DrawCircle(uint8_t par_x, uint8_t par_y, uint8_t par_r, SSD1306_COLOR color);
void ssd1306_FillCircle(uint8_t par_x, uint8_t par_y, uint8_t par_r, SSD1306_COLOR par_color);
void ssd1306_Polyline(const SSD1306_VERTEX *par_vertex, uint16_t par_size, SSD1306_COLOR color);
void ssd1306_DrawRectangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, SSD1306_COLOR color);
void ssd1306_FillRectangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, SSD1306_COLOR color);

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

    me->fault_str_scroll_offset = 0;

    me->text_color = White;

    ssd1306_Fill(Black);

    QActive_ctor(&me->super, Q_STATE_CAST(&initial));
    QTimeEvt_ctorX(&me->screen_update_evt, &me->super, WAIT_TIMEOUT_SIG, 0U);
    QTimeEvt_ctorX(&me->color_update_evt, &me->super, COLOR_CHANGE_SIG, 0U);
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
    QActive_subscribe((QActive *)me, PUBSUB_FAULT_GENERATED_SIG);
    QActive_subscribe((QActive *)me, PUBSUB_MOTOR_DATA_SIG);

    return Q_TRAN(&startup);
}

static QState startup(SSD1306 *const me, QEvt const *const e)
{
    QState status;

    switch (e->sig)
    {
    case Q_INIT_SIG:
    {
        QStateHandler nextState = (QStateHandler)&startup;
        // If we're on the last command, then move to 'running' afterwards
        if (me->init_command_num == NUM_INIT_COMMANDS - 1)
        {
            nextState = (QStateHandler)(&waiting);
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

        QTimeEvt_armX(
            &me->screen_update_evt,
            BSP_TICKS_PER_SEC / OLED_FPS,
            BSP_TICKS_PER_SEC / OLED_FPS);

        me->init_command_ptr = INIT_SSD1306_ST;
        me->init_command_num = 0;

        status = Q_HANDLED();
        break;
    }
    case I2C_ERROR_SIG:
    {
        Fault_Manager_Generate_Fault(&me->super, FAULT_ID_OLED_I2C, "Startup Error");
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

static QState error(SSD1306 *const me, QEvt const *const e)
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
        status = Q_SUPER(&top);
        break;
    }
    }

    return status;
}

static QState top(SSD1306 *const me, QEvt const *const e)
{
    QState status;

    switch (e->sig)
    {
    case Q_ENTRY_SIG:
    {

        QTimeEvt_armX(
            &me->color_update_evt,
            BSP_TICKS_PER_SEC * COLOR_CHANGE_SECONDS,
            BSP_TICKS_PER_SEC * COLOR_CHANGE_SECONDS);
        status = Q_HANDLED();
        break;
    }
    case PUBSUB_MOTOR_DATA_SIG:
    {
        const MotorDataEvent_T *event = Q_EVT_CAST(MotorDataEvent_T);
        me->temperature = event->temperature;
        me->pressure = event->pressure;
        me->tachometer = event->tachometer;
        me->vbat = event->vbat;
        me->buzzer = event->buzzer;
        me->start = event->start;
        me->neutral = event->neutral;
        me->red = event->red;
        me->orange = event->orange;
        status = Q_HANDLED();
        break;
    }
    case COLOR_CHANGE_SIG:
    {
        me->text_color = (SSD1306_COLOR)!me->text_color;
        status = Q_HANDLED();
        break;
    }
    case I2C_ERROR_SIG:
    {
        Fault_Manager_Generate_Fault(&me->super, FAULT_ID_OLED_I2C, "Unknown Error");
        status = Q_TRAN(&error);
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

static QState waiting(SSD1306 *const me, QEvt const *const e)
{
    QState status;

    switch (e->sig)
    {
    case Q_ENTRY_SIG:
    {
        status = Q_HANDLED();
        break;
    }
    case WAIT_TIMEOUT_SIG:
    {
        status = Q_TRAN(&update_screen);
        break;
    }
    default:
    {
        status = Q_SUPER(&top);
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
    case Q_INIT_SIG:
    {
        status = Q_TRAN(&update_screen_1);
        break;
    }
    case Q_ENTRY_SIG:
    {

        SSD1306_COLOR background_color = (SSD1306_COLOR)!me->text_color;
        ssd1306_Fill(background_color);
        char print_buffer[32] = {0};

        snprintf(
            print_buffer,
            sizeof(print_buffer),
            "%d.%.2dV",
            me->vbat / 100, me->vbat % 100);

        ssd1306_SetCursor(0, 0);
        ssd1306_WriteString(print_buffer, Font_7x10, me->text_color);

        if (me->start)
        {
            ssd1306_SetCursor(50, 0);
            ssd1306_WriteString("S", Font_7x10, me->text_color);
        }

        if (me->neutral)
        {
            ssd1306_SetCursor(75, 0);
            ssd1306_WriteString("N", Font_7x10, me->text_color);
        }

        if (me->buzzer)
        {
            ssd1306_SetCursor(25, 36);
            ssd1306_WriteString("--ALARM--", Font_7x10, background_color);
        }

        snprintf(
            print_buffer,
            sizeof(print_buffer),
            "%d.%.2dC   %d.%.2d PSI",
            me->temperature / 100, me->temperature % 100, me->pressure / 100, me->pressure % 100);
        ssd1306_SetCursor(0, 12);
        ssd1306_WriteString(print_buffer, Font_7x10, me->text_color);

        snprintf(
            print_buffer,
            sizeof(print_buffer),
            "%d.%.2dRPM",
            me->tachometer / 100, me->tachometer % 100);
        ssd1306_SetCursor(0, 24);
        ssd1306_WriteString(print_buffer, Font_7x10, me->text_color);

        switch (me->red)
        {
        case LOW:
        {
            break;
        }
        case HIGH:
        {
            ssd1306_SetCursor(80, 24);
            ssd1306_WriteString("R", Font_7x10, me->text_color);
            break;
        }
        case HIGH_Z:
        {
            ssd1306_SetCursor(80, 24);
            ssd1306_WriteString("R", Font_7x10, background_color);
            break;
        }
        case SIG_UNKNOWN:
        {
            ssd1306_SetCursor(80, 24);
            ssd1306_WriteString("R?", Font_7x10, background_color);
            break;
        }
        }

        switch (me->orange)
        {
        case LOW:
        {
            break;
        }
        case HIGH:
        {
            ssd1306_SetCursor(100, 24);
            ssd1306_WriteString("O", Font_7x10, me->text_color);
            break;
        }
        case HIGH_Z:
        {
            ssd1306_SetCursor(100, 24);
            ssd1306_WriteString("O", Font_7x10, background_color);
            break;
        }
        case SIG_UNKNOWN:
        {
            ssd1306_SetCursor(100, 24);
            ssd1306_WriteString("O?", Font_7x10, background_color);
            break;
        }
        }

        me->counter++;
        if (me->counter <= 3)
        {
            ssd1306_SetCursor(120, 0);
            ssd1306_WriteString("*", Font_7x10, me->text_color);
        }
        if (me->counter == 6)
            me->counter = 0;

        drawFaultTextToScreen();

        me->pageNumber = 0;
        status = Q_HANDLED();
        break;
    }
    default:
    {
        status = Q_SUPER(&top);
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
    case Q_INIT_SIG:
    {
        static const uint8_t args[] = {0x00, 0x10};
        // Set desired RAM page address
        status = send_command_substate_machine(
            (QStateHandler)&update_screen_1,
            (QStateHandler)&update_screen_2,
            0xB0 + me->pageNumber,
            2,
            args);
        break;
    }
    case I2C_ERROR_SIG:
    {
        Fault_Manager_Generate_Fault(&me->super, FAULT_ID_OLED_I2C, "Error while updating RAM address");
        status = Q_TRAN(&error);
        break;
    }
    default:
    {
        status = Q_SUPER(&update_screen);
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
        memcpy(me->i2c_data + 1, &me->SSD1306_Buffer[SSD1306_WIDTH * me->pageNumber], SSD1306_WIDTH);

        I2C_Return_T retval = me->i2c_write(
            SSD1306_ADDR,
            me->i2c_data,
            1 + SSD1306_WIDTH,
            I2C_Complete_CB,
            I2C_Error_CB,
            &ssd1306_inst);

        if (retval != I2C_RTN_SUCCESS)
        {
            static QEvt const event = QEVT_INITIALIZER(I2C_ERROR_SIG);
            QACTIVE_POST((QActive *)me, &event, me);
        }

        status = Q_HANDLED();
        break;
    }
    case I2C_COMPLETE_SIG:
    {
        me->pageNumber++;
        if (me->pageNumber == SSD1306_HEIGHT / 8)
            status = Q_TRAN(&waiting);
        else
            status = Q_TRAN(&update_screen_1); // Send the next page of RAM
        break;
    }
    case I2C_ERROR_SIG:
    {
        Fault_Manager_Generate_Fault(&me->super, FAULT_ID_OLED_I2C, "Error while sending RAM data");
        status = Q_TRAN(&error);
        break;
    }
    default:
    {
        status = Q_SUPER(&update_screen);
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
            QACTIVE_POST((QActive *)me, &event, me);
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
 ***************************************************************************************************
 *
 * @brief   Print the faults (if any) to screen and scroll horizontally
 *
 **************************************************************************************************/

static void drawFaultTextToScreen()
{
    SSD1306 *me = &ssd1306_inst;
    char print_buffer[FAULT_STR_LEN] = {0};

    char *str_cur = print_buffer;
    const char *str_end = print_buffer + sizeof(print_buffer);

    // pad with spaces
    str_cur += snprintf(str_cur, str_end - str_cur, "                    errors, ");

    Active_Fault_T *active_faults = Fault_Manager_Get_Active_Fault_List();
    if (active_faults[0].id == FAULT_ID_NONE)
    {
        return;
    }

    uint8_t i;
    for (i = 0; i < FAULT_MANAGER_BUFFER_LENGTH; i++)
    {
        Active_Fault_T this_fault = active_faults[i];
        if (this_fault.id == FAULT_ID_NONE)
        {
            break;
        }

        str_cur += snprintf(str_cur, str_end - str_cur, "#%d: ", (i + 1));

        str_cur += snprintf(str_cur, str_end - str_cur,
                            "%s",
                            Fault_Manager_Get_Description(this_fault.id));

        str_cur += snprintf(str_cur, str_end - str_cur, "/%s, ", this_fault.msg);
    }

    // Add to the beginning of the string how many errors we found
    if (i < 10)
        snprintf(print_buffer + 18, 2, "%d", i);
    else if (i < 100)
        snprintf(print_buffer + 17, 3, "%d", i);
    else
        snprintf(print_buffer + 16, 4, "%d", i);
    print_buffer[19] = ' '; // replace the null terminator with a space

    ssd1306_SetCursor(0, 54);
    ssd1306_WriteString(print_buffer + me->fault_str_scroll_offset, Font_7x10, White);
    me->fault_str_scroll_offset++;

    if (me->fault_str_scroll_offset >= strlen(print_buffer))
        me->fault_str_scroll_offset = 0;
}

/* Fill the whole screen with the given color */
void ssd1306_Fill(SSD1306_COLOR color)
{
    SSD1306 *me = &ssd1306_inst;
    memset(me->SSD1306_Buffer, (color == Black) ? 0x00 : 0xFF, sizeof(me->SSD1306_Buffer));
}

/*
 * Draw one pixel in the screenbuffer
 * X => X Coordinate
 * Y => Y Coordinate
 * color => Pixel color
 */
void ssd1306_DrawPixel(uint8_t x, uint8_t y, SSD1306_COLOR color)
{
    SSD1306 *me = &ssd1306_inst;
    if (x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT)
    {
        // Don't write outside the buffer
        return;
    }

    // Draw in the right color
    if (color == White)
    {
        me->SSD1306_Buffer[x + (y / 8) * SSD1306_WIDTH] |= 1 << (y % 8);
    }
    else
    {
        me->SSD1306_Buffer[x + (y / 8) * SSD1306_WIDTH] &= ~(1 << (y % 8));
    }
}

/*
 * Draw 1 char to the screen buffer
 * ch       => char om weg te schrijven
 * Font     => Font waarmee we gaan schrijven
 * color    => Black or White
 */
char ssd1306_WriteChar(char ch, SSD1306_Font_t Font, SSD1306_COLOR color)
{
    SSD1306 *me = &ssd1306_inst;
    uint32_t i, b, j;

    // Check if character is valid
    if (ch < 32 || ch > 126)
        return 0;

    // Check remaining space on current line
    if (SSD1306_WIDTH < (me->CurrentX + Font.width) ||
        SSD1306_HEIGHT < (me->CurrentY + Font.height))
    {
        // Not enough space on current line
        return 0;
    }

    // Use the font to write
    for (i = 0; i < Font.height; i++)
    {
        b = Font.data[(ch - 32) * Font.height + i];
        for (j = 0; j < Font.width; j++)
        {
            if ((b << j) & 0x8000)
            {
                ssd1306_DrawPixel(me->CurrentX + j, (me->CurrentY + i), (SSD1306_COLOR)color);
            }
            else
            {
                ssd1306_DrawPixel(me->CurrentX + j, (me->CurrentY + i), (SSD1306_COLOR)!color);
            }
        }
    }

    // The current space is now taken
    me->CurrentX += Font.char_width ? Font.char_width[ch - 32] : Font.width;

    // Return written char for validation
    return ch;
}

/* Write full string to screenbuffer */
char ssd1306_WriteString(char *str, SSD1306_Font_t Font, SSD1306_COLOR color)
{
    while (*str)
    {
        if (ssd1306_WriteChar(*str, Font, color) != *str)
        {
            // Char could not be written
            return *str;
        }
        str++;
    }

    // Everything ok
    return *str;
}

/* Position the cursor */
void ssd1306_SetCursor(uint8_t x, uint8_t y)
{
    SSD1306 *me = &ssd1306_inst;
    me->CurrentX = x;
    me->CurrentY = y;
}

/* Draw line by Bresenhem's algorithm */
void ssd1306_Line(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, SSD1306_COLOR color)
{
    int32_t deltaX = abs(x2 - x1);
    int32_t deltaY = abs(y2 - y1);
    int32_t signX = ((x1 < x2) ? 1 : -1);
    int32_t signY = ((y1 < y2) ? 1 : -1);
    int32_t error = deltaX - deltaY;
    int32_t error2;

    ssd1306_DrawPixel(x2, y2, color);

    while ((x1 != x2) || (y1 != y2))
    {
        ssd1306_DrawPixel(x1, y1, color);
        error2 = error * 2;
        if (error2 > -deltaY)
        {
            error -= deltaY;
            x1 += signX;
        }

        if (error2 < deltaX)
        {
            error += deltaX;
            y1 += signY;
        }
    }
    return;
}

/* Draw polyline */
void ssd1306_Polyline(const SSD1306_VERTEX *par_vertex, uint16_t par_size, SSD1306_COLOR color)
{
    uint16_t i;
    if (par_vertex == NULL)
    {
        return;
    }

    for (i = 1; i < par_size; i++)
    {
        ssd1306_Line(par_vertex[i - 1].x, par_vertex[i - 1].y, par_vertex[i].x, par_vertex[i].y, color);
    }

    return;
}

/* Convert Degrees to Radians */
static float ssd1306_DegToRad(float par_deg)
{
    return par_deg * (3.14f / 180.0f);
}

/* Normalize degree to [0;360] */
static uint16_t ssd1306_NormalizeTo0_360(uint16_t par_deg)
{
    uint16_t loc_angle;
    if (par_deg <= 360)
    {
        loc_angle = par_deg;
    }
    else
    {
        loc_angle = par_deg % 360;
        loc_angle = (loc_angle ? loc_angle : 360);
    }
    return loc_angle;
}

/*
 * DrawArc. Draw angle is beginning from 4 quart of trigonometric circle (3pi/2)
 * start_angle in degree
 * sweep in degree
 */
void ssd1306_DrawArc(uint8_t x, uint8_t y, uint8_t radius, uint16_t start_angle, uint16_t sweep, SSD1306_COLOR color)
{
    static const uint8_t CIRCLE_APPROXIMATION_SEGMENTS = 36;
    float approx_degree;
    uint32_t approx_segments;
    uint8_t xp1, xp2;
    uint8_t yp1, yp2;
    uint32_t count;
    uint32_t loc_sweep;
    float rad;

    loc_sweep = ssd1306_NormalizeTo0_360(sweep);

    count = (ssd1306_NormalizeTo0_360(start_angle) * CIRCLE_APPROXIMATION_SEGMENTS) / 360;
    approx_segments = (loc_sweep * CIRCLE_APPROXIMATION_SEGMENTS) / 360;
    approx_degree = loc_sweep / (float)approx_segments;
    while (count < approx_segments)
    {
        rad = ssd1306_DegToRad(count * approx_degree);
        xp1 = x + (int8_t)(sinf(rad) * radius);
        yp1 = y + (int8_t)(cosf(rad) * radius);
        count++;
        if (count != approx_segments)
        {
            rad = ssd1306_DegToRad(count * approx_degree);
        }
        else
        {
            rad = ssd1306_DegToRad(loc_sweep);
        }
        xp2 = x + (int8_t)(sinf(rad) * radius);
        yp2 = y + (int8_t)(cosf(rad) * radius);
        ssd1306_Line(xp1, yp1, xp2, yp2, color);
    }

    return;
}

/*
 * Draw arc with radius line
 * Angle is beginning from 4 quart of trigonometric circle (3pi/2)
 * start_angle: start angle in degree
 * sweep: finish angle in degree
 */
void ssd1306_DrawArcWithRadiusLine(uint8_t x, uint8_t y, uint8_t radius, uint16_t start_angle, uint16_t sweep, SSD1306_COLOR color)
{
    const uint32_t CIRCLE_APPROXIMATION_SEGMENTS = 36;
    float approx_degree;
    uint32_t approx_segments;
    uint8_t xp1;
    uint8_t xp2 = 0;
    uint8_t yp1;
    uint8_t yp2 = 0;
    uint32_t count;
    uint32_t loc_sweep;
    float rad;

    loc_sweep = ssd1306_NormalizeTo0_360(sweep);

    count = (ssd1306_NormalizeTo0_360(start_angle) * CIRCLE_APPROXIMATION_SEGMENTS) / 360;
    approx_segments = (loc_sweep * CIRCLE_APPROXIMATION_SEGMENTS) / 360;
    approx_degree = loc_sweep / (float)approx_segments;

    rad = ssd1306_DegToRad(count * approx_degree);
    uint8_t first_point_x = x + (int8_t)(sinf(rad) * radius);
    uint8_t first_point_y = y + (int8_t)(cosf(rad) * radius);
    while (count < approx_segments)
    {
        rad = ssd1306_DegToRad(count * approx_degree);
        xp1 = x + (int8_t)(sinf(rad) * radius);
        yp1 = y + (int8_t)(cosf(rad) * radius);
        count++;
        if (count != approx_segments)
        {
            rad = ssd1306_DegToRad(count * approx_degree);
        }
        else
        {
            rad = ssd1306_DegToRad(loc_sweep);
        }
        xp2 = x + (int8_t)(sinf(rad) * radius);
        yp2 = y + (int8_t)(cosf(rad) * radius);
        ssd1306_Line(xp1, yp1, xp2, yp2, color);
    }

    // Radius line
    ssd1306_Line(x, y, first_point_x, first_point_y, color);
    ssd1306_Line(x, y, xp2, yp2, color);
    return;
}

/* Draw circle by Bresenhem's algorithm */
void ssd1306_DrawCircle(uint8_t par_x, uint8_t par_y, uint8_t par_r, SSD1306_COLOR par_color)
{
    int32_t x = -par_r;
    int32_t y = 0;
    int32_t err = 2 - 2 * par_r;
    int32_t e2;

    if (par_x >= SSD1306_WIDTH || par_y >= SSD1306_HEIGHT)
    {
        return;
    }

    do
    {
        ssd1306_DrawPixel(par_x - x, par_y + y, par_color);
        ssd1306_DrawPixel(par_x + x, par_y + y, par_color);
        ssd1306_DrawPixel(par_x + x, par_y - y, par_color);
        ssd1306_DrawPixel(par_x - x, par_y - y, par_color);
        e2 = err;

        if (e2 <= y)
        {
            y++;
            err = err + (y * 2 + 1);
            if (-x == y && e2 <= x)
            {
                e2 = 0;
            }
        }

        if (e2 > x)
        {
            x++;
            err = err + (x * 2 + 1);
        }
    } while (x <= 0);

    return;
}

/* Draw filled circle. Pixel positions calculated using Bresenham's algorithm */
void ssd1306_FillCircle(uint8_t par_x, uint8_t par_y, uint8_t par_r, SSD1306_COLOR par_color)
{
    int32_t x = -par_r;
    int32_t y = 0;
    int32_t err = 2 - 2 * par_r;
    int32_t e2;

    if (par_x >= SSD1306_WIDTH || par_y >= SSD1306_HEIGHT)
    {
        return;
    }

    do
    {
        for (uint8_t _y = (par_y + y); _y >= (par_y - y); _y--)
        {
            for (uint8_t _x = (par_x - x); _x >= (par_x + x); _x--)
            {
                ssd1306_DrawPixel(_x, _y, par_color);
            }
        }

        e2 = err;
        if (e2 <= y)
        {
            y++;
            err = err + (y * 2 + 1);
            if (-x == y && e2 <= x)
            {
                e2 = 0;
            }
        }

        if (e2 > x)
        {
            x++;
            err = err + (x * 2 + 1);
        }
    } while (x <= 0);

    return;
}

/* Draw a rectangle */
void ssd1306_DrawRectangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, SSD1306_COLOR color)
{
    ssd1306_Line(x1, y1, x2, y1, color);
    ssd1306_Line(x2, y1, x2, y2, color);
    ssd1306_Line(x2, y2, x1, y2, color);
    ssd1306_Line(x1, y2, x1, y1, color);

    return;
}

/* Draw a filled rectangle */
void ssd1306_FillRectangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, SSD1306_COLOR color)
{
    uint8_t x_start = ((x1 <= x2) ? x1 : x2);
    uint8_t x_end = ((x1 <= x2) ? x2 : x1);
    uint8_t y_start = ((y1 <= y2) ? y1 : y2);
    uint8_t y_end = ((y1 <= y2) ? y2 : y1);

    for (uint8_t y = y_start; (y <= y_end) && (y < SSD1306_HEIGHT); y++)
    {
        for (uint8_t x = x_start; (x <= x_end) && (x < SSD1306_WIDTH); x++)
        {
            ssd1306_DrawPixel(x, y, color);
        }
    }
    return;
}

SSD1306_Error_t ssd1306_InvertRectangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2)
{
    SSD1306 *const me = &ssd1306_inst;
    if ((x2 >= SSD1306_WIDTH) || (y2 >= SSD1306_HEIGHT))
    {
        return SSD1306_ERR;
    }
    if ((x1 > x2) || (y1 > y2))
    {
        return SSD1306_ERR;
    }
    uint32_t i;
    if ((y1 / 8) != (y2 / 8))
    {
        /* if rectangle doesn't lie on one 8px row */
        for (uint32_t x = x1; x <= x2; x++)
        {
            i = x + (y1 / 8) * SSD1306_WIDTH;
            me->SSD1306_Buffer[i] ^= 0xFF << (y1 % 8);
            i += SSD1306_WIDTH;
            for (; i < x + (y2 / 8) * SSD1306_WIDTH; i += SSD1306_WIDTH)
            {
                me->SSD1306_Buffer[i] ^= 0xFF;
            }
            me->SSD1306_Buffer[i] ^= 0xFF >> (7 - (y2 % 8));
        }
    }
    else
    {
        /* if rectangle lies on one 8px row */
        const uint8_t mask = (0xFF << (y1 % 8)) & (0xFF >> (7 - (y2 % 8)));
        for (i = x1 + (y1 / 8) * SSD1306_WIDTH;
             i <= (uint32_t)x2 + (y2 / 8) * SSD1306_WIDTH; i++)
        {
            me->SSD1306_Buffer[i] ^= mask;
        }
    }
    return SSD1306_OK;
}

/* Draw a bitmap */
void ssd1306_DrawBitmap(uint8_t x, uint8_t y, const unsigned char *bitmap, uint8_t w, uint8_t h, SSD1306_COLOR color)
{
    int16_t byteWidth = (w + 7) / 8; // Bitmap scanline pad = whole byte
    uint8_t byte = 0;

    if (x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT)
    {
        return;
    }

    for (uint8_t j = 0; j < h; j++, y++)
    {
        for (uint8_t i = 0; i < w; i++)
        {
            if (i & 7)
            {
                byte <<= 1;
            }
            else
            {
                byte = (*(const unsigned char *)(&bitmap[j * byteWidth + i / 8]));
            }

            if (byte & 0x80)
            {
                ssd1306_DrawPixel(x + i, y, color);
            }
        }
    }
    return;
}