//============================================================================
// Product: Blinky example, NUCLEO-C031C6 board, QV kernel
// Last updated for version 8.0.0
// Last updated on  2024-09-18
//
//                   Q u a n t u m  L e a P s
//                   ------------------------
//                   Modern Embedded Software
//
// Copyright (C) 2005 Quantum Leaps, LLC. <state-machine.com>
//
// This program is open source software: you can redistribute it and/or
// modify it under the terms of the GNU General Public License as published
// by the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// Alternatively, this program may be distributed and modified under the
// terms of Quantum Leaps commercial licenses, which expressly supersede
// the GNU General Public License and are specifically designed for
// licensees interested in retaining the proprietary status of their code.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program. If not, see <www.gnu.org/licenses/>.
//
// Contact information:
// <www.state-machine.com/licensing>
// <info@state-machine.com>
//============================================================================
#include "qpc.h" // QP/C real-time embedded framework
#include "bsp.h" // Board Support Package
#include "pubsub_signals.h"
#include "shared_i2c.h"
#include "tusb.h"
#include "i2c_bus.h"
#include "LMT01.h"
#include "i2c_bus_stm32.h"
#include "main.h"

#include "stm32g4xx_hal.h"
#include <stdio.h>

Q_DEFINE_THIS_MODULE("bsp.c")

#ifdef Q_SPY
#error The Simple Blinky Application does not support Spy build configuration
#endif
#define USB_INTERFACE_CLI 0
#define USB_INTERFACE_PC_COM 1
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)

// Static Data
#define SHARED_I2C_BUS_2_DEFERRED_QUEUE_LEN 3
static I2C_Bus_T s_i2c_bus2;

static SharedI2C_T SharedI2C_Bus2;
const QActive *AO_SharedI2C2 = &(SharedI2C_Bus2.super); // externally available
QEvt const *i2c_bus_2_deferred_queue_storage[SHARED_I2C_BUS_2_DEFERRED_QUEUE_LEN];

extern ADC_HandleTypeDef hadc2;  // defined in main.c by cubeMX
extern TIM_HandleTypeDef htim15; // defined in main.c by cubeMX

// Static Function Declarations

static uint16_t USB0_TransmitData(const uint8_t *data_ptr, const uint16_t data_len);
static uint16_t USB0_ReceiveData(uint8_t *data_ptr, const uint16_t max_data_len);
static void USB0_RegisterDataReadyCB(Serial_IO_Data_Ready_Callback cb, void *cb_data);

static uint16_t USB1_TransmitData(const uint8_t *data_ptr, const uint16_t data_len);
static uint16_t USB1_ReceiveData(uint8_t *data_ptr, const uint16_t max_data_len);
static void USB1_RegisterDataReadyCB(Serial_IO_Data_Ready_Callback cb, void *cb_data);

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

static Serial_IO_Data_Ready_Callback s_usb0_data_ready_cb = 0;
static void *s_usb0_data_ready_cb_data = 0;

static Serial_IO_Data_Ready_Callback s_usb1_data_ready_cb = 0;
static void *s_usb1_data_ready_cb_data = 0;

const Serial_IO_T s_bsp_serial_io_usb0 = {
    .tx_func = USB0_TransmitData,
    .rx_func = USB0_ReceiveData,
    .register_cb_func = USB0_RegisterDataReadyCB,
};

static const Serial_IO_T s_bsp_serial_io_usb1 = {
    .tx_func = USB1_TransmitData,
    .rx_func = USB1_ReceiveData,
    .register_cb_func = USB1_RegisterDataReadyCB,
};

/**
 ***************************************************************************************************
 * @brief   Functions for USB
 **************************************************************************************************/

static uint16_t USB0_TransmitData(const uint8_t *data_ptr, const uint16_t data_len)
{
    uint16_t n_written = tud_cdc_n_write(USB_INTERFACE_CLI, data_ptr, data_len);
    tud_cdc_n_write_flush(USB_INTERFACE_CLI);

    return n_written;
}

static uint16_t USB0_ReceiveData(uint8_t *data_ptr, const uint16_t max_data_len)
{
    uint32_t count = 0;
    if (tud_cdc_n_available(USB_INTERFACE_CLI))
    {
        count = tud_cdc_n_read(USB_INTERFACE_CLI, data_ptr, max_data_len);
    }

    return count;
}

static void USB0_RegisterDataReadyCB(Serial_IO_Data_Ready_Callback cb, void *cb_data)
{
    s_usb0_data_ready_cb = cb;
    s_usb0_data_ready_cb_data = cb_data;
}

static uint16_t USB1_TransmitData(const uint8_t *data_ptr, const uint16_t data_len)
{
    uint16_t n_written = tud_cdc_n_write(USB_INTERFACE_PC_COM, data_ptr, data_len);
    tud_cdc_n_write_flush(USB_INTERFACE_PC_COM);

    return n_written;
}

static uint16_t USB1_ReceiveData(uint8_t *data_ptr, const uint16_t max_data_len)
{
    uint32_t count = 0;
    if (tud_cdc_n_available(USB_INTERFACE_PC_COM))
    {
        count = tud_cdc_n_read(USB_INTERFACE_PC_COM, data_ptr, max_data_len);
    }

    return count;
}

static void USB1_RegisterDataReadyCB(Serial_IO_Data_Ready_Callback cb, void *cb_data)
{
    s_usb1_data_ready_cb = cb;
    s_usb1_data_ready_cb_data = cb_data;
}

void tud_cdc_rx_cb(uint8_t itf)
{
    if (itf == USB_INTERFACE_CLI && s_usb0_data_ready_cb != 0)
    {
        s_usb0_data_ready_cb(s_usb0_data_ready_cb_data);
    }
    else if (itf == USB_INTERFACE_PC_COM && s_usb1_data_ready_cb != 0)
    {
        s_usb1_data_ready_cb(s_usb1_data_ready_cb_data);
    }
    else
    {
        // do nothing
    }
}

//============================================================================
// Error handler and ISRs...

Q_NORETURN Q_onError(char const *const module, int_t const id)
{
    // NOTE: this implementation of the error handler is intended only
    // for debugging and MUST be changed for deployment of the application
    // (assuming that you ship your production code with assertions enabled).
    Q_UNUSED_PAR(module);
    Q_UNUSED_PAR(id);
    QS_ASSERTION(module, id, 10000U);

#ifndef NDEBUG
    // for debugging, hang on in an endless loop...
    for (;;)
    {
    }
#endif

    NVIC_SystemReset();
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
    QTIMEEVT_TICK_X(0U, &l_SysTick_Handler); // time events at rate 0
    QV_ARM_ERRATUM_838869();
}

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
    return I2C_Bus_Read(
        I2C_BUS_ID_2, address, rx_buffer, data_len, complete_cb, error_cb, cb_data);
    // return SharedI2C_Read(
    //     &SharedI2C_Bus2, address, rx_buffer, data_len, complete_cb, error_cb, cb_data);
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

//============================================================================
// BSP functions...

//............................................................................
void BSP_Init_I2C(void)
{
    HAL_StatusTypeDef retval;

    /////////////////////////
    // I2C Bus 2
    /////////////////////////

    // I2C Bus 2 Peripheral
    I2C_HandleTypeDef *p_hi2c2 = STM32_GetI2CHandle(I2C_BUS_ID_2);

    p_hi2c2->Instance = I2C2;
    p_hi2c2->Init.Timing = 0x00503D58;
    p_hi2c2->Init.OwnAddress1 = 0;
    p_hi2c2->Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    p_hi2c2->Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    p_hi2c2->Init.OwnAddress2 = 0;
    p_hi2c2->Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    p_hi2c2->Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    p_hi2c2->Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

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

    // Initialize I2C buses
    BSP_Init_I2C();

    SharedI2C_ctor(
        &SharedI2C_Bus2,
        I2C_BUS_ID_2,
        i2c_bus_2_deferred_queue_storage,
        SHARED_I2C_BUS_2_DEFERRED_QUEUE_LEN);

    // initialize TinyUSB device stack on configured roothub port
    tud_init(BOARD_TUD_RHPORT);
}
//............................................................................
void BSP_ledOn()
{
    printf("LED on\n\r");
    HAL_GPIO_WritePin(FW_LED_GPIO_Port, FW_LED_Pin, 1);
}
//............................................................................
void BSP_ledOff()
{
    printf("LED off\n\r");
    HAL_GPIO_WritePin(FW_LED_GPIO_Port, FW_LED_Pin, 0);
}
//............................................................................
void BSP_debug_gpio_on()
{
    HAL_GPIO_WritePin(DEBUG_GPIO_GPIO_Port, DEBUG_GPIO_Pin, 1);
}
//............................................................................
void BSP_debug_gpio_off()
{
    HAL_GPIO_WritePin(DEBUG_GPIO_GPIO_Port, DEBUG_GPIO_Pin, 0);
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
    NVIC_SetPriority(USART2_IRQn, 0); // kernel UNAWARE interrupt
    NVIC_SetPriority(EXTI0_IRQn, QF_AWARE_ISR_CMSIS_PRI + 0U);
    NVIC_SetPriority(TIM4_IRQn, QF_AWARE_ISR_CMSIS_PRI + 0U);
    NVIC_SetPriority(I2C2_EV_IRQn, QF_AWARE_ISR_CMSIS_PRI + 1U);
    NVIC_SetPriority(I2C2_ER_IRQn, QF_AWARE_ISR_CMSIS_PRI + 1U);
    NVIC_SetPriority(SysTick_IRQn, QF_AWARE_ISR_CMSIS_PRI + 2U);
    // ...

    // enable IRQs...
    NVIC_EnableIRQ(EXTI0_IRQn);

#ifdef Q_SPY
    NVIC_EnableIRQ(USART2_IRQn); // UART2 interrupt used for QS-RX
#endif
}
//............................................................................
void QF_onCleanup(void)
{
}
//............................................................................
void QV_onIdle(void)
{ // called with interrupts DISABLED, see NOTE01

    // toggle an LED on and then off (not enough LEDs, see NOTE02)
    // GPIOA->BSRR = (1U << LD4_PIN);         // turn LED[n] on
    // GPIOA->BSRR = (1U << (LD4_PIN + 16U)); // turn LED[n] off

#ifdef Q_SPY
#elif defined NDEBUG
  // Put the CPU and peripherals to the low-power mode.
    // you might need to customize the clock management for your application,
    // see the datasheet for your particular Cortex-M MCU.
    //
    // !!!CAUTION!!!
    // QV_CPU_SLEEP() contains the WFI instruction, which stops the CPU
    // clock, which unfortunately disables the JTAG port, so the ST-Link
    // debugger can no longer connect to the board. For that reason, the call
    // to QV_CPU_SLEEP() has to be used with CAUTION.
    //
    // NOTE: If you find your board "frozen" like this, strap BOOT0 to VDD and
    // reset the board, then connect with ST-Link Utilities and erase the part.
    // The trick with BOOT(0) is it gets the part to run the System Loader
    // instead of your broken code. When done disconnect BOOT0, and start over.
    //
    // QV_CPU_SLEEP();  // atomically go to sleep and enable interrupts
    QF_INT_ENABLE(); // for now, just enable interrupts
#else
    QF_INT_ENABLE(); // just enable interrupts
#endif
}

//============================================================================
// NOTE1:
// The QF_AWARE_ISR_CMSIS_PRI constant from the QF port specifies the highest
// ISR priority that is disabled by the QF framework. The value is suitable
// for the NVIC_SetPriority() CMSIS function.
//
// Only ISRs prioritized at or below the QF_AWARE_ISR_CMSIS_PRI level (i.e.,
// with the numerical values of priorities equal or higher than
// QF_AWARE_ISR_CMSIS_PRI) are allowed to call the QV_ISR_ENTRY/
// QV_ISR_ENTRY macros or any other QF/QV services. These ISRs are
// "QF-aware".
//
// Conversely, any ISRs prioritized above the QF_AWARE_ISR_CMSIS_PRI priority
// level (i.e., with the numerical values of priorities less than
// QF_AWARE_ISR_CMSIS_PRI) are never disabled and are not aware of the kernel.
// Such "QF-unaware" ISRs cannot call ANY QF/QV services. In particular they
// can NOT call the macros QV_ISR_ENTRY/QV_ISR_ENTRY. The only mechanism
// by which a "QF-unaware" ISR can communicate with the QF framework is by
// triggering a "QF-aware" ISR, which can post/publish events.
//
// NOTE2:
// The User LED is used to visualize the idle loop activity. The brightness
// of the LED is proportional to the frequency of the idle loop.
// Please note that the LED is toggled with interrupts locked, so no interrupt
// execution time contributes to the brightness of the User LED.
//

const Serial_IO_T *BSP_Get_Serial_IO_Interface_USB0()
{
    return &s_bsp_serial_io_usb0;
}

const Serial_IO_T *BSP_Get_Serial_IO_Interface_USB1()
{
    return &s_bsp_serial_io_usb1;
}

PUTCHAR_PROTOTYPE
{
    /* Place your implementation of fputc here */
    /* e.g. write a character to the USART1 and Loop until the end of transmission */
    //  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);

    tud_cdc_n_write(USB_INTERFACE_PC_COM, (uint8_t *)&ch, 1);
    tud_cdc_n_write_flush(USB_INTERFACE_PC_COM);

    return ch;
}

/**
 ***************************************************************************************************
 * @brief   EXTI callback, called by HAL IRQ handler
 **************************************************************************************************/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    (void)GPIO_Pin;
    /* Prevent unused argument(s) compilation warning */
    // switch (GPIO_Pin)
    // {
    // case LMT01_Pin:
    // {
    //     break;
    // }
    // }
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

/**
 ***************************************************************************************************
 * @brief   GPIO motor ECU Functions
 **************************************************************************************************/

void BSP_Tach_Capture_Timer_Enable()
{
    // If the input capture occurs (rising edge and falling edge on DROP_SENSE input),
    //   then HAL_TIM_IC_CaptureCallback will be called (twice, once for each edge)
    // If the timeout counter counts down to zero, then HAL_TIM_PeriodElapsedCallback will
    //   be called, indicating that the timeout period has occurred.
    //
    // TIM15 is prescaled to 16Mhz/(7+1)=2Mhz, or 0.5 microsecond per tick

    __HAL_TIM_CLEAR_FLAG(&htim15, TIM_FLAG_UPDATE);

    HAL_NVIC_EnableIRQ(TIM1_BRK_TIM15_IRQn);

    HAL_TIM_IC_Start_IT(&htim15, TIM_CHANNEL_1); // input capture timer
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    static volatile uint32_t captured_val_old = 0;
    static volatile uint32_t captured_val_new = 0;

    if (htim->Instance != TIM15 || htim->Channel != HAL_TIM_ACTIVE_CHANNEL_1)
    {
        return;
    }

    captured_val_old = captured_val_new;
    captured_val_new = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

    // unsigned integer overflow will make this correct even
    //  if counter has wrapped between capture 1 and 2
    uint32_t width_clks = captured_val_new - captured_val_old;

    Int16Event_T *capture_event = Q_NEW(
        Int16Event_T, PUBSUB_TACH_SIG);

    // TIM15 is on the APB2 clock bus, which is 16 MHz, and scaled down by (7+1) to 2Mhz
    // microseconds = clocks / 2
    // Hz = 10^6*2/clocks
    uint32_t frequency = 1000 * 1000 * 2 / width_clks * 100 * 60; // hundredths of RPM
    capture_event->num = (uint16_t)frequency;

    QACTIVE_PUBLISH(&capture_event->super, &me->super);
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
    bool pin1 = HAL_GPIO_ReadPin(RED_SENSE_1_GPIO_Port, RED_SENSE_1_Pin) == GPIO_PIN_SET;
    bool pin2 = HAL_GPIO_ReadPin(RED_SENSE_2_GPIO_Port, RED_SENSE_2_Pin) == GPIO_PIN_SET;
    uint8_t data = (pin2 << 1) | pin1;
    switch (data)
    {
    case 0x00:
    {
        return 0; // the wire is low
    }
    case 0x03:
    {
        return 1; // the wire is high
    }
    case 0x02:
    {
        return 3; // the wire is high-Z
    }
    default:
    {
        return 4; // shouldn't happen
    }
    }
}

uint8_t BSP_Get_Orange()
{
    // Test for tri-state
    bool pin1 = HAL_GPIO_ReadPin(ORANGE_SENSE_1_GPIO_Port, ORANGE_SENSE_1_Pin) == GPIO_PIN_SET;
    bool pin2 = HAL_GPIO_ReadPin(ORANGE_SENSE_2_GPIO_Port, ORANGE_SENSE_2_Pin) == GPIO_PIN_SET;
    uint8_t data = (pin2 << 1) | pin1;
    switch (data)
    {
    case 0x00:
    {
        return 0; // the wire is low
    }
    case 0x03:
    {
        return 1; // the wire is high
    }
    case 0x02:
    {
        return 3; // the wire is high-Z
    }
    default:
    {
        return 4; // shouldn't happen
    }
    }
}

bool BSP_Get_Buzzer()
{
    // Buzzer is active low
    return HAL_GPIO_ReadPin(nBUZZER_SENSE_GPIO_Port, nBUZZER_SENSE_Pin) == GPIO_PIN_RESET;
}

uint16_t BSP_ADC_Read_VBAT(void)
{
    uint32_t raw_adc = HAL_ADC_GetValue(&hadc2);
    HAL_ADC_Start(&hadc2); // a bit hacky, but start the next conversion
    // Multiplier is 4.6 nominally, but the actual value seems to be 4.3
    float hv_sense = raw_adc / 4096. * 4.3 * 100; // return hundredths of volts
    return (uint16_t)hv_sense;
}