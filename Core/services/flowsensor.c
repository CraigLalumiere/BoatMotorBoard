#include "flowsensor.h"
#include "bsp.h"
#include <math.h>
#include <stdbool.h>

/**************************************************************************************************\
* Private macros
\**************************************************************************************************/

#define FILTER_TAU 0.1 // time constant of first order filter [seconds]

/**************************************************************************************************\
* Private type definitions
\**************************************************************************************************/

typedef enum
{
    NO_EDGES,
    FIRST_EDGE,
    SECOND_EDGE
} Edges_Detected_T;

/**************************************************************************************************\
* Private memory declarations
\**************************************************************************************************/
static volatile float32_t flow_rate_Hz;

static volatile uint16_t pulse_time;

static bool edgeFound         = false;
static Edges_Detected_T edges = NO_EDGES;

/**************************************************************************************************\
* Private prototypes
\**************************************************************************************************/

/**************************************************************************************************\
* Public functions
\**************************************************************************************************/

/**
 ***************************************************************************************************
 *
 * @brief   Handle input capture event
 *
 **************************************************************************************************/
void Flow_Sensor_IC_Callback(TIM_HandleTypeDef *htim, uint32_t TIM_CHANNEL)
{
    // the first edge (the one that 'resets' the timer), will also
    // fire this interrupt, and the captured value will be zero
    uint16_t this_pulse = (uint16_t) HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL);
    if (this_pulse > 0)
    {
        pulse_time = this_pulse;
        edgeFound  = true;
        if (edges == NO_EDGES)
            edges = FIRST_EDGE;
        else if (edges == FIRST_EDGE)
        {
            edges = SECOND_EDGE;
        }
    }
}

/**
 ***************************************************************************************************
 *
 * @brief   Handle timer overflow (udpate event)
 *
 **************************************************************************************************/
void Flow_Sensor_Period_Elapsed_Callback(TIM_HandleTypeDef *htim)
{
    float32_t deltaT;

    if (edgeFound)
    {
        if (edges == SECOND_EDGE)
        {
            // TIM15 is on the APB2 clock bus, which is 144 MHz, and scaled down by (71+1) to 2Mhz
            // microseconds = periods / 2
            deltaT       = pulse_time / 2000000.0; // Seconds
            flow_rate_Hz = 1.0 / deltaT;           /// Hz
        }
    }
    else
    {
        deltaT       = 65536 / 2000000.0; // Seconds
        flow_rate_Hz = 0;
        edges        = NO_EDGES;
    }

    edgeFound = false;
    __HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_UPDATE);
    __HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_CC1);
}

/**
 ***************************************************************************************************
 *
 * @brief   Read the current encoder data.
 *
 **************************************************************************************************/
float Flow_Sensor_Read_Hz()
{
    HAL_NVIC_DisableIRQ(BSP_Get_Flow_Sensor_IRQN());
    float my_flow_rate = flow_rate_Hz;
    HAL_NVIC_EnableIRQ(BSP_Get_Flow_Sensor_IRQN());
    return my_flow_rate;
}

/**************************************************************************************************\
* Private functions
\**************************************************************************************************/