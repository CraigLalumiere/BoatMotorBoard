#include "bsp.h" // Board Support Package
#include "halt_if_debugging.h"
#include "i2c_bus_stm32.h"
#include "main.h"
#include "pubsub_signals.h"
#include "qpc.h" // QP/C real-time embedded framework
#include "reset.h"
#include "shared_i2c.h"
#include "stm32g4xx_hal.h"
#include "tusb.h"
#include <stdio.h>

Q_DEFINE_THIS_MODULE("bsp.c")

/**************************************************************************************************\
* Private macros
\**************************************************************************************************/

#define USB_INTERFACE                       0
#define SHARED_I2C_BUS_2_DEFERRED_QUEUE_LEN 3
#define AVREF                               2.9

/**************************************************************************************************\
* Private type definitions
\**************************************************************************************************/

/**************************************************************************************************\
* Private prototypes
\**************************************************************************************************/

static uint16_t USB0_TransmitData(const uint8_t *data_ptr, const uint16_t data_len);
static uint16_t USB0_ReceiveData(uint8_t *data_ptr, const uint16_t max_data_len);
static void USB0_RegisterDataReadyCB(Serial_IO_Data_Ready_Callback cb, void *cb_data);

static I2C_Return_T BSP_I2C_Write_SSD1306(
    uint8_t address,
    uint8_t *tx_buffer,
    const uint16_t data_len,
    I2C_Complete_Callback complete_cb,
    I2C_Error_Callback error_cb,
    void *cb_data);

static I2C_Return_T BSP_I2C_Read_SSD1306(
    uint8_t address,
    uint8_t *tx_buffer,
    const uint16_t data_len,
    I2C_Complete_Callback complete_cb,
    I2C_Error_Callback error_cb,
    void *cb_data);

static I2C_Return_T BSP_I2C_Write_Pressure(
    uint8_t address,
    uint8_t *tx_buffer,
    const uint16_t data_len,
    I2C_Complete_Callback complete_cb,
    I2C_Error_Callback error_cb,
    void *cb_data);

static I2C_Return_T BSP_I2C_Read_Pressure(
    uint8_t address,
    uint8_t *tx_buffer,
    const uint16_t data_len,
    I2C_Complete_Callback complete_cb,
    I2C_Error_Callback error_cb,
    void *cb_data);

/**************************************************************************************************\
* Private memory declarations
\**************************************************************************************************/

static I2C_Bus_T s_i2c_bus2;

static SharedI2C_T SharedI2C_Bus2;
const QActive *AO_SharedI2C2 = &(SharedI2C_Bus2.super); // externally available
QEvt const *i2c_bus_2_deferred_queue_storage[SHARED_I2C_BUS_2_DEFERRED_QUEUE_LEN];

extern ADC_HandleTypeDef hadc2;   // defined in main.c by cubeMX
extern TIM_HandleTypeDef htim15;  // defined in main.c by cubeMX
extern UART_HandleTypeDef huart2; // defined in main.c by cubeMX

bool input_capture_found;

static Serial_IO_Data_Ready_Callback s_usb0_data_ready_cb = 0;
static void *s_usb0_data_ready_cb_data                    = 0;

const Serial_IO_T s_bsp_serial_io_usb0 = {
    .tx_func          = USB0_TransmitData,
    .rx_func          = USB0_ReceiveData,
    .register_cb_func = USB0_RegisterDataReadyCB,
};

/**************************************************************************************************\
* Public functions
\**************************************************************************************************/

I2C_Write BSP_Get_I2C_Write_SSD1306()
{
    return BSP_I2C_Write_SSD1306;
}

I2C_Read BSP_Get_I2C_Read_SSD1306()
{
    return BSP_I2C_Read_SSD1306;
}

I2C_Write BSP_Get_I2C_Write_Pressure()
{
    return BSP_I2C_Write_Pressure;
}

I2C_Read BSP_Get_I2C_Read_Pressure()
{
    return BSP_I2C_Read_Pressure;
}

/**
 ***************************************************************************************************
 * @brief   Put the Honeywell pressure sensor into or out of reset
 **************************************************************************************************/
void BSP_Put_Pressure_Sensor_Into_Reset(bool reset)
{
    // Active low signal
    HAL_GPIO_WritePin(PRESSURE_RST_GPIO_Port, PRESSURE_RST_Pin, !reset);
}

bool BSP_Get_Neutral()
{
    // If in neutral, the neutral wire is 6Ω path to GND, so the GPIO is low
    return HAL_GPIO_ReadPin(NEUTRAL_DETECT_GPIO_Port, NEUTRAL_DETECT_Pin) == GPIO_PIN_RESET;
}

bool BSP_Get_Start()
{
    // If starting, the neutral wire +12, so the GPIO is high
    return HAL_GPIO_ReadPin(START_DET_GPIO_Port, START_DET_Pin) == GPIO_PIN_SET;
}
uint8_t BSP_Get_Red()
{
    // Test for tri-state
    bool pin1    = HAL_GPIO_ReadPin(RED_SENSE_1_GPIO_Port, RED_SENSE_1_Pin) == GPIO_PIN_SET;
    bool pin2    = HAL_GPIO_ReadPin(RED_SENSE_2_GPIO_Port, RED_SENSE_2_Pin) == GPIO_PIN_SET;
    uint8_t data = (pin2 << 1) | pin1;
    switch (data)
    {
        case 0x00: {
            return LOW; // the wire is low
        }
        case 0x03: {
            return HIGH; // the wire is high
        }
        case 0x02: {
            return HIGH_Z; // the wire is high-Z
        }
        default: {
            return SIG_UNKNOWN; // shouldn't happen
        }
    }
}

uint8_t BSP_Get_Orange()
{
    // Test for tri-state
    bool pin1    = HAL_GPIO_ReadPin(ORANGE_SENSE_1_GPIO_Port, ORANGE_SENSE_1_Pin) == GPIO_PIN_SET;
    bool pin2    = HAL_GPIO_ReadPin(ORANGE_SENSE_2_GPIO_Port, ORANGE_SENSE_2_Pin) == GPIO_PIN_SET;
    uint8_t data = (pin2 << 1) | pin1;
    switch (data)
    {
        case 0x00: {
            return LOW; // the wire is low
        }
        case 0x03: {
            return HIGH; // the wire is high
        }
        case 0x02: {
            return HIGH_Z; // the wire is high-Z
        }
        default: {
            return SIG_UNKNOWN; // shouldn't happen
        }
    }
}

bool BSP_Get_Buzzer()
{
    // Buzzer is active low
    return HAL_GPIO_ReadPin(nBUZZER_SENSE_GPIO_Port, nBUZZER_SENSE_Pin) == GPIO_PIN_RESET;
}

float BSP_ADC_Read_VBAT(void)
{
    uint32_t raw_adc = HAL_ADC_GetValue(&hadc2);
    HAL_ADC_Start(&hadc2); // a bit hacky, but start the next conversion
    // Multiplier is 4.6 nominally, but the actual value seems to be 4.3
    float hv_sense = raw_adc * AVREF / 4096. * 4.6; // return volts
    // calibration scale & offset
    hv_sense = 1.5633 * hv_sense + 0.81196;
    return hv_sense;
}

/**
 ***************************************************************************************************
 * @brief   GPIO motor ECU Functions
 **************************************************************************************************/

// void BSP_Tach_Capture_Timer_Enable()
// {
//     HAL_NVIC_EnableIRQ(TIM1_BRK_TIM15_IRQn);
//     input_capture_found = false;
// }

int32_t BSP_Get_Flow_Sensor_IRQN(void)
{
    return TIM1_BRK_TIM15_IRQn;
}

/**
 ***************************************************************************************************
 * @brief   QP Assert handler
 **************************************************************************************************/
Q_NORETURN Q_onError(char const *const module, int_t const loc)
{
    // NOTE: this implementation of the error handler is intended only
    // for debugging and MUST be changed for deployment of the application
    // (assuming that you ship your production code with assertions enabled).
    Q_UNUSED_PAR(module);
    Q_UNUSED_PAR(loc);
    QS_ASSERTION(module, loc, 10000U);

#ifndef NDEBUG
    BSP_LED_On();
#endif

    const uint32_t NOT_USED = 0;
    Reset_DoResetWithReasonWithStr(RESET_REASON_Q_ASSERT, module, loc, NOT_USED);
}

/**
 ***************************************************************************************************
 * @brief   Perform a reset of the microcontroller
 **************************************************************************************************/
__attribute__((noreturn)) void BSP_SystemReset(void)
{
    HALT_IF_DEBUGGING();
    NVIC_SystemReset();
}

/**
 ***************************************************************************************************
 * @brief   Read the microcontroller's RCC CSR Register.
 **************************************************************************************************/
uint32_t BSP_RCC_CSR_Read(void)
{
    return RCC->CSR;
}

/**
 ***************************************************************************************************
 * @brief   Clear reset flags indicated by the RCC CSR Register.
 **************************************************************************************************/
void BSP_RCC_CSR_ClearResetFlags(void)
{
    // From Ref Manual:
    //   RMVF: Remove reset flag -- This bit is set by software to clear the reset flags.
    RCC->CSR |= RCC_CSR_RMVF;
}

//............................................................................
void assert_failed(char const *const module, int_t const id); // prototype
void assert_failed(char const *const module, int_t const id)
{
    Q_onError(module, id);
}

//............................................................................
void SysTick_Handler(void); // prototype
void SysTick_Handler(void)
{
    QK_ISR_ENTRY();
    HAL_IncTick();
    QTIMEEVT_TICK(0U); // process time events for primary clock rate
    QK_ISR_EXIT();
}

//============================================================================
// BSP functions...

/**
 ***************************************************************************************************
 *
 * @brief   Millisecond Tick
 *
 **************************************************************************************************/
uint32_t BSP_Get_Milliseconds_Tick(void)
{
    return HAL_GetTick();
}

//............................................................................
void BSP_Init_I2C(void)
{
    HAL_StatusTypeDef retval;

    /////////////////////////
    // I2C Bus 2
    /////////////////////////

    // I2C Bus 2 Peripheral
    I2C_HandleTypeDef *p_hi2c2 = STM32_GetI2CHandle(I2C_BUS_ID_2);

    p_hi2c2->Instance              = I2C2;
    p_hi2c2->Init.Timing           = 0x50916E9F; // 0x00503D58;
    p_hi2c2->Init.OwnAddress1      = 0;
    p_hi2c2->Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
    p_hi2c2->Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
    p_hi2c2->Init.OwnAddress2      = 0;
    p_hi2c2->Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    p_hi2c2->Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
    p_hi2c2->Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;

    retval = HAL_I2C_Init(p_hi2c2);
    Q_ASSERT(retval == HAL_OK);

    retval = HAL_I2CEx_ConfigAnalogFilter(p_hi2c2, I2C_ANALOGFILTER_ENABLE);
    Q_ASSERT(retval == HAL_OK);

    retval = HAL_I2CEx_ConfigDigitalFilter(p_hi2c2, 0);
    Q_ASSERT(retval == HAL_OK);

    I2C_Bus_Init(&s_i2c_bus2, I2C_BUS_ID_2);
}

//............................................................................
void BSP_Init(void)
{
    // initialize TinyUSB device stack on configured roothub port
    tud_init(BOARD_TUD_RHPORT);

    /**********************************************************************************P****************\
    * Init TIM15 for tach input capture
    \**************************************************************************************************/

    // TIM15 is prescaled to 144Mhz/(71+1)=2Mhz, or 0.5 microsecond per tick
    // __HAL_TIM_CLEAR_FLAG(&htim15, TIM_FLAG_UPDATE);
    HAL_TIM_IC_Start_IT(&htim15, TIM_CHANNEL_1); // input capture timer
    // the above function only enables the input capture interrupt flag
    // enable also the update event (oveflow)
    __HAL_TIM_ENABLE_IT(&htim15, TIM_IT_UPDATE);
    // __HAL_TIM_ENABLE_IT(&htim15, TIM_IT_UPDATE);

    /**************************************************************************************************\
    * Init UART2
    \**************************************************************************************************/
    /* Flush the data registers from unexpected data */
    __HAL_UART_FLUSH_DRREGISTER(&huart2);
    // if (HAL_UART_Receive_IT(&huart2, (uint8_t *)&rx_byte, 1) != HAL_OK)
    // {
    //   Error_Handler();
    // }

    // Initialize I2C buses
    BSP_Init_I2C();

    SharedI2C_ctor(
        &SharedI2C_Bus2,
        I2C_BUS_ID_2,
        i2c_bus_2_deferred_queue_storage,
        SHARED_I2C_BUS_2_DEFERRED_QUEUE_LEN);
}

//............................................................................
void BSP_LED_On()
{
    HAL_GPIO_WritePin(FW_LED_GPIO_Port, FW_LED_Pin, 1);
}
//............................................................................
void BSP_LED_Off()
{
    HAL_GPIO_WritePin(FW_LED_GPIO_Port, FW_LED_Pin, 0);
}
//............................................................................
void BSP_terminate(int16_t result)
{
    Q_UNUSED_PAR(result);
}

//============================================================================
// QF callbacks...
void QF_onStartup(void)
{
    // set up the SysTick timer to fire at BSP_TICKS_PER_SEC rate
    SysTick_Config(SystemCoreClock / BSP_TICKS_PER_SEC);

    // assign all priority bits for preemption-prio. and none to sub-prio.
    NVIC_SetPriorityGrouping(0U);

    // set priorities of ALL ISRs used in the system, see NOTE1
    // NVIC_SetPriority(EXTI0_IRQn, QF_AWARE_ISR_CMSIS_PRI + 0U);
    // NVIC_SetPriority(TIM4_IRQn, QF_AWARE_ISR_CMSIS_PRI + 0U);
    NVIC_SetPriority(TIM1_BRK_TIM15_IRQn, QF_AWARE_ISR_CMSIS_PRI + 0U); // tach input capture
    NVIC_SetPriority(I2C2_EV_IRQn, QF_AWARE_ISR_CMSIS_PRI + 1U);        // I2C for pressure and OLED
    NVIC_SetPriority(I2C2_ER_IRQn, QF_AWARE_ISR_CMSIS_PRI + 1U);        // I2C for pressure and OLED
    NVIC_SetPriority(USART2_IRQn, QF_AWARE_ISR_CMSIS_PRI + 2U);
    NVIC_SetPriority(SysTick_IRQn, QF_AWARE_ISR_CMSIS_PRI + 12U);
    // ...

    // enable IRQs...
    // NVIC_EnableIRQ(EXTI0_IRQn);
    // HAL_NVIC_EnableIRQ(TIM1_BRK_TIM15_IRQn); // enaled by AO

#ifdef Q_SPY
    NVIC_EnableIRQ(USART2_IRQn); // UART2 interrupt used for QS-RX
#endif
}
//............................................................................
void QF_onCleanup(void)
{
}
//............................................................................
void QK_onIdle(void)
{ // called with interrupts DISABLED, see NOTE01
}

/*****************************************************************************
 * NOTE1:
 * The QF_AWARE_ISR_CMSIS_PRI constant from the QF port specifies the highest
 * ISR priority that is disabled by the QF framework. The value is suitable
 * for the NVIC_SetPriority() CMSIS function.
 *
 * Only ISRs prioritized at or below the QF_AWARE_ISR_CMSIS_PRI level (i.e.,
 * with the numerical values of priorities equal or higher than
 * QF_AWARE_ISR_CMSIS_PRI) are allowed to call the QK_ISR_ENTRY/QK_ISR_ENTRY
 * macros or any other QF services. These ISRs are "QF-aware".
 *
 * Conversely, any ISRs prioritized above the QF_AWARE_ISR_CMSIS_PRI priority
 * level (i.e., with the numerical values of priorities less than
 * QF_AWARE_ISR_CMSIS_PRI) are never disabled and are not aware of the kernel.
 * Such "QF-unaware" ISRs cannot call any QF services. In particular they
 * can NOT call the macros QK_ISR_ENTRY/QK_ISR_ENTRY. The only mechanism
 * by which a "QF-unaware" ISR can communicate with the QF framework is by
 * triggering a "QF-aware" ISR, which can post/publish events.
 *
 */

/**
 ***************************************************************************************************
 *
 * @brief   USB0 Interface Getter
 *
 **************************************************************************************************/
const Serial_IO_T *BSP_Get_Serial_IO_Interface_USB0()
{
    return &s_bsp_serial_io_usb0;
}

/**************************************************************************************************\
* Private functions
\**************************************************************************************************/

static I2C_Return_T BSP_I2C_Write_SSD1306(
    uint8_t address,
    uint8_t *tx_buffer,
    const uint16_t data_len,
    I2C_Complete_Callback complete_cb,
    I2C_Error_Callback error_cb,
    void *cb_data)
{
    // return I2C_Bus_Write(
    //     I2C_BUS_ID_2, address, tx_buffer, data_len, complete_cb, error_cb, cb_data);
    return SharedI2C_Write(
        &SharedI2C_Bus2, address, tx_buffer, data_len, complete_cb, error_cb, cb_data);
}

static I2C_Return_T BSP_I2C_Read_SSD1306(
    uint8_t address,
    uint8_t *rx_buffer,
    const uint16_t data_len,
    I2C_Complete_Callback complete_cb,
    I2C_Error_Callback error_cb,
    void *cb_data)
{
    // return I2C_Bus_Read(I2C_BUS_ID_2, address, rx_buffer, data_len, complete_cb, error_cb,
    // cb_data);
    return SharedI2C_Read(
        &SharedI2C_Bus2, address, rx_buffer, data_len, complete_cb, error_cb, cb_data);
}

static I2C_Return_T BSP_I2C_Write_Pressure(
    uint8_t address,
    uint8_t *tx_buffer,
    const uint16_t data_len,
    I2C_Complete_Callback complete_cb,
    I2C_Error_Callback error_cb,
    void *cb_data)
{
    // return I2C_Bus_Write(
    //     I2C_BUS_ID_2, address, tx_buffer, data_len, complete_cb, error_cb, cb_data);
    return SharedI2C_Write(
        &SharedI2C_Bus2, address, tx_buffer, data_len, complete_cb, error_cb, cb_data);
}

static I2C_Return_T BSP_I2C_Read_Pressure(
    uint8_t address,
    uint8_t *rx_buffer,
    const uint16_t data_len,
    I2C_Complete_Callback complete_cb,
    I2C_Error_Callback error_cb,
    void *cb_data)
{
    // return I2C_Bus_Read(
    //     I2C_BUS_ID_2, address, rx_buffer, data_len, complete_cb, error_cb, cb_data);
    return SharedI2C_Read(
        &SharedI2C_Bus2, address, rx_buffer, data_len, complete_cb, error_cb, cb_data);
}

/**
 ***************************************************************************************************
 *  @brief   Functions for USB
 **************************************************************************************************/
static uint16_t USB0_TransmitData(const uint8_t *data_ptr, const uint16_t data_len)
{
    uint16_t n_written = tud_cdc_n_write(USB_INTERFACE, data_ptr, data_len);
    tud_cdc_n_write_flush(USB_INTERFACE);

    return n_written;
}

static uint16_t USB0_ReceiveData(uint8_t *data_ptr, const uint16_t max_data_len)
{
    uint32_t count = 0;
    if (tud_cdc_n_available(USB_INTERFACE))
    {
        count = tud_cdc_n_read(USB_INTERFACE, data_ptr, max_data_len);
    }

    return count;
}

static void USB0_RegisterDataReadyCB(Serial_IO_Data_Ready_Callback cb, void *cb_data)
{
    s_usb0_data_ready_cb      = cb;
    s_usb0_data_ready_cb_data = cb_data;
}

void tud_cdc_rx_cb(uint8_t itf)
{
    if (itf == USB_INTERFACE && s_usb0_data_ready_cb != 0)
    {
        s_usb0_data_ready_cb(s_usb0_data_ready_cb_data);
    }
    else
    {
        // do nothing
    }
}