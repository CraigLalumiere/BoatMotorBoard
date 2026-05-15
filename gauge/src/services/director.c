#include "director.h"
#include "bsp.h"
#include "private_signal_ranges.h"

#include <stddef.h>

Q_DEFINE_THIS_MODULE("Director");

typedef struct
{
    float input_value;
    float output_volts;
} GaugeMapPoint_T;

// Gauge calibration tables.
// Inputs are engineering units; outputs are DAC volts after analog front-end tuning.
static const GaugeMapPoint_T s_temp_gauge_map[] = {
    {25.0f, 1.30f},
    {26.0f, 1.40f},
    {29.0f, 1.50f},
    {33.0f, 1.60f},
    {36.0f, 1.70f},
    {40.0f, 1.80f},
    {44.0f, 1.90f},
    {48.0f, 2.00f},
    {53.0f, 2.10f},
    {58.0f, 2.20f},
    {65.0f, 2.30f},
    {73.0f, 2.40f},
    {83.0f, 2.50f},
    {98.0f, 2.60f},
    {120.0f, 2.70f},
};

static const GaugeMapPoint_T s_pressure_gauge_map[] = {
    {0.0f, 2.80f},
    {10.0f, 2.70f},
    {29.0f, 2.60f},
    {49.0f, 2.50f},
    {72.0f, 2.40f},
    {97.0f, 2.30f},
    {123.0f, 2.20f},
    {145.0f, 2.10f},
};

/**************************************************************************************************\
* Private type definitions
\**************************************************************************************************/
enum DIRECTOR_Signals
{
    DIRECTOR_DUMMY_SIG = PRIVATE_SIGNAL_DIRECTOR_START,
    POLL_TIMEOUT_SIG,
};

typedef struct
{
    QActive super;
    QTimeEvt timeEvt;
} Director;

/**************************************************************************************************\
* Private memory declarations
\**************************************************************************************************/
static Director director_inst;
QActive *const AO_DIRECTOR = &director_inst.super;

/**************************************************************************************************\
* Private prototypes
\**************************************************************************************************/
static QState initial(Director *const me, void const *const par);
static QState top(Director *const me, QEvt const *const e);
static float lookup_gauge_voltage(
    const GaugeMapPoint_T *map, size_t map_len, float input_value);
static float map_temperature_to_voltage(float temperature_c);
static float map_pressure_to_voltage(float pressure_psi);

/**************************************************************************************************\
* Public functions
\**************************************************************************************************/
void Director_ctor(void)
{
    Director *const me = &director_inst;
    QActive_ctor(&me->super, Q_STATE_CAST(&initial));

    QTimeEvt_ctorX(&me->timeEvt, &me->super, POLL_TIMEOUT_SIG, 0U);
}

/**************************************************************************************************\
* HSM
\**************************************************************************************************/
static QState initial(Director *const me, void const *const par)
{
    Q_UNUSED_PAR(par);

    QActive_subscribe((QActive *) me, PUBSUB_MOTOR_DATA_SIG);

    QTimeEvt_armX(&me->timeEvt, BSP_TICKS_PER_SEC / 10U, BSP_TICKS_PER_SEC / 10U);

    return Q_TRAN(&top);
}

static QState top(Director *const me, QEvt const *const e)
{
    QState status;

    switch (e->sig)
    {
        case Q_ENTRY_SIG: {
            BSP_Gauge_SetPressure_V(map_pressure_to_voltage(0.0f));
            BSP_Gauge_SetTemperature_V(map_temperature_to_voltage(25.0f));
            BSP_Gauge_SetOpAmpRef_V(1.78f);

            BSP_RpmGauge_SetPFM_RPM(0U);

            BSP_Set_Backlight(false);

            QEvt *evt = Q_NEW(QEvt, PUBSUB_BOX_TO_BOX_STARTUP_SIG);
            QACTIVE_PUBLISH(evt, &me->super);

            status = Q_HANDLED();
            break;
        }

        case PUBSUB_MOTOR_DATA_SIG: {
            const MotorDataEvent_T *evt = Q_EVT_CAST(MotorDataEvent_T);

            BSP_Gauge_SetPressure_V(map_pressure_to_voltage(evt->pressure));
            BSP_Gauge_SetTemperature_V(map_temperature_to_voltage(evt->temperature));
            BSP_RpmGauge_SetPFM_RPM((uint32_t) evt->tachometer);

            status = Q_HANDLED();
            break;
        }

        case POLL_TIMEOUT_SIG: {
            BSP_Set_Backlight(BSP_Get_Backlight());

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

static float lookup_gauge_voltage(
    const GaugeMapPoint_T *map, size_t map_len, float input_value)
{
    Q_ASSERT(map != NULL);
    Q_ASSERT(map_len > 0U);

    if (input_value <= map[0].input_value)
    {
        return map[0].output_volts;
    }

    for (size_t i = 1U; i < map_len; ++i)
    {
        if (input_value <= map[i].input_value)
        {
            const float x0   = map[i - 1U].input_value;
            const float x1   = map[i].input_value;
            const float y0   = map[i - 1U].output_volts;
            const float y1   = map[i].output_volts;
            const float span = x1 - x0;

            if (span <= 0.0f)
            {
                return y1;
            }

            const float position = (input_value - x0) / span;
            return y0 + (position * (y1 - y0));
        }
    }

    return map[map_len - 1U].output_volts;
}

static float map_temperature_to_voltage(float temperature_c)
{
    return lookup_gauge_voltage(s_temp_gauge_map, Q_DIM(s_temp_gauge_map), temperature_c);
}

static float map_pressure_to_voltage(float pressure_psi)
{
    return lookup_gauge_voltage(s_pressure_gauge_map, Q_DIM(s_pressure_gauge_map), pressure_psi);
}
