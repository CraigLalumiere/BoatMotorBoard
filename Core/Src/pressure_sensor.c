// For pressure sensor MPRLS0025PA00001A

// Datasheet for sensor:
// https://prod-edam.honeywell.com/content/dam/honeywell-edam/sps/siot/en-us/products/sensors/pressure-sensors/board-mount-pressure-sensors/micropressure-mpr-series/documents/sps-siot-mpr-series-datasheet-32332628-ciid-172626.pdf?download=false

#include "pressure_sensor.h"
#include "bsp.h"
#include "private_signal_ranges.h"
#include "pubsub_signals.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef Q_SPY
Q_DEFINE_THIS_MODULE("Pressure Sensor")
#endif // def Q_SPY

/**************************************************************************************************\
* Private macros
\**************************************************************************************************/

// Address definition
// ------------------------------------------------------------------------------------
#define Sensor_ADDR 0x18

#define OUTPUTMAX 15099494 // output at maximum pressure [counts]
#define OUTPUTMIN 1677722  // output at minimum pressure [counts]
#define PMAX 30            // maximum value of pressure range [bar, psi, kPa, etc.]
#define PMIN 0             // minimum value of pressure range [bar, psi, kPa, etc.]

#define N_BYTES_I2C_DATA 7 // one status byte + 3 bytes pressure + 3 bytes temperature

/**************************************************************************************************\
* Private type definitions
\**************************************************************************************************/

enum PressureSignals
{
    WAIT_TIMEOUT_SIG = PRIVATE_SIGNAL_SSD1306_START,
    I2C_COMPLETE_SIG,
    I2C_ERROR_SIG,
};

typedef struct
{
    QActive super;      // inherit QActive
    QTimeEvt timer_evt; // timer to wait for voltage to settle
    I2C_Write i2c_write;
    I2C_Read i2c_read;
    uint8_t i2c_data[N_BYTES_I2C_DATA];

    // submachine state memory
    QStateHandler substate_next_state;
    QStateHandler substate_super_state;
    uint8_t command;
    uint8_t num_args;
    uint8_t *args;

} PRESSURE;

/**************************************************************************************************\
* Private memory declarations
\**************************************************************************************************/
static PRESSURE pressure_inst;
QActive *const AO_Pressure = &pressure_inst.super;

/**************************************************************************************************\
* Private prototypes
\**************************************************************************************************/

// state handler functions
static QState initial(PRESSURE *const me, void const *const par);
static QState startup(PRESSURE *const me, QEvt const *const e);
static QState startup_into_reset(PRESSURE *const me, QEvt const *const e);
static QState startup_out_of_reset(PRESSURE *const me, QEvt const *const e);
static QState startup_error(PRESSURE *const me, QEvt const *const e);
static QState error(PRESSURE *const me, QEvt const *const e);

static QState top(PRESSURE *const me, QEvt const *const e);
static QState running(PRESSURE *const me, QEvt const *const e);
static QState beginConversion(PRESSURE *const me, QEvt const *const e);
static QState readData(PRESSURE *const me, QEvt const *const e);


static void I2C_Complete_CB(void *cb_data);
static void I2C_Error_CB(void *cb_data);

/**************************************************************************************************\
* Public functions
\**************************************************************************************************/

/**
 ***************************************************************************************************
 * @brief   Constructor
 **************************************************************************************************/
void Pressure_Sensor_ctor(I2C_Write i2c_write_fn, I2C_Read i2c_read_fn)
{
    PRESSURE *const me = &pressure_inst;

    me->i2c_write = i2c_write_fn;
    me->i2c_read = i2c_read_fn;

    QActive_ctor(&me->super, Q_STATE_CAST(&initial));
    QTimeEvt_ctorX(&me->timer_evt, &me->super, WAIT_TIMEOUT_SIG, 0U);
}

/**************************************************************************************************\
* Private functions
\**************************************************************************************************/

/**
 ***************************************************************************************************
 * @brief   HSM
 **************************************************************************************************/
static QState initial(PRESSURE *const me, void const *const par)
{
    Q_UNUSED_PAR(par);
    // QActive_subscribe((QActive *) me, PUBSUB_TEST_CARTRIDGE_BEGIN);

    return Q_TRAN(&startup);
}

static QState startup(PRESSURE *const me, QEvt const *const e)
{
    QState status;

    switch (e->sig)
    {
    case Q_INIT_SIG:
    {
        status = Q_TRAN(&startup_into_reset);
        break;
    }
    case Q_ENTRY_SIG:
    {
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

static QState startup_into_reset(PRESSURE *const me, QEvt const *const e)
{
    QState status;

    switch (e->sig)
    {
    case Q_ENTRY_SIG:
    {
        BSP_Put_Pressure_Sensor_Into_Reset(true);
        // wait 10ms
        QTimeEvt_armX(
            &me->timer_evt,
            BSP_TICKS_PER_SEC / 100,
            0);
        status = Q_HANDLED();
        break;
    }
    case WAIT_TIMEOUT_SIG:
    {
        status = Q_TRAN(&startup_out_of_reset);
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

static QState startup_out_of_reset(PRESSURE *const me, QEvt const *const e)
{
    QState status;

    switch (e->sig)
    {
    case Q_ENTRY_SIG:
    {
        BSP_Put_Pressure_Sensor_Into_Reset(false);
        // wait 10ms
        QTimeEvt_armX(
            &me->timer_evt,
            BSP_TICKS_PER_SEC / 100,
            0);
        status = Q_HANDLED();
        break;
    }
    case WAIT_TIMEOUT_SIG:
    {
        status = Q_TRAN(&running);
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

static QState startup_error(PRESSURE *const me, QEvt const *const e)
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

static QState error(PRESSURE *const me, QEvt const *const e)
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

static QState top(PRESSURE *const me, QEvt const *const e)
{
    QState status;

    switch (e->sig)
    {
    case Q_ENTRY_SIG:
    {
        status = Q_HANDLED();
        break;
    }
    case I2C_ERROR_SIG:
    {
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

static QState running(PRESSURE *const me, QEvt const *const e)
{
    QState status;

    switch (e->sig)
    {
    case Q_ENTRY_SIG:
    {
        // Begin 10ms timer (to set how frequent we sample the sensor)
        QTimeEvt_armX(
            &me->timer_evt,
            BSP_TICKS_PER_SEC / 100,
            0);
        status = Q_HANDLED();
        break;
    }
    case WAIT_TIMEOUT_SIG:
    {
        status = Q_TRAN(&beginConversion);
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

static QState beginConversion(PRESSURE *const me, QEvt const *const e)
{
    QState status;

    switch (e->sig)
    {
    case Q_ENTRY_SIG:
    {
        // Send 'output measurement command' of 0xAA, 0x00, 0x00
        memset(me->i2c_data, 0, sizeof(me->i2c_data));
        me->i2c_data[0] = 0xAA;

        I2C_Return_T retval = me->i2c_write(
            Sensor_ADDR,
            me->i2c_data,
            3,
            I2C_Complete_CB,
            I2C_Error_CB,
            me);

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
        status = Q_TRAN(&readData);
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

static QState readData(PRESSURE *const me, QEvt const *const e)
{
    QState status;

    switch (e->sig)
    {
    case Q_ENTRY_SIG:
    {
        // Begin 10ms timer (conversion should be complete in under 5ms)
        QTimeEvt_armX(
            &me->timer_evt,
            BSP_TICKS_PER_SEC / 100,
            0);
        status = Q_HANDLED();
        break;
    }
    case WAIT_TIMEOUT_SIG:
    {
        // Read 7 bytes of data
        memset(me->i2c_data, 0, sizeof(me->i2c_data));

        I2C_Return_T retval = me->i2c_read(
            Sensor_ADDR,
            me->i2c_data,
            N_BYTES_I2C_DATA,
            I2C_Complete_CB,
            I2C_Error_CB,
            me);

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
        uint8_t sensor_status = me->i2c_data[0];
        bool sensor_busy = sensor_status && 0x20;
        (void) sensor_busy;
    uint32_t press_counts = me->i2c_data[3] + (me->i2c_data[2] << 8) + (me->i2c_data[1] << 16);
        // calculation of pressure value according to equation 2 of datasheet
        float pressure = ((press_counts - OUTPUTMIN) * (PMAX - PMIN)) / (OUTPUTMAX - OUTPUTMIN) + PMIN;

        uint32_t temp_counts = me->i2c_data[6] + (me->i2c_data[5] << 8) + (me->i2c_data[4] << 16);
        int8_t temperature = (temp_counts * 200 / 16777215) - 50;
        (void) temperature;

        FloatEvent_T *event = Q_NEW(
            FloatEvent_T, PUBSUB_PRESSURE_SIG);
        event->num = pressure;
        QACTIVE_PUBLISH(&event->super, &me->super);

        status = Q_TRAN(&running);
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
