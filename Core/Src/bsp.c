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
#include "qpc.h"                 // QP/C real-time embedded framework
#include "blinky.h"              // Blinky Application interface
#include "usb.h"
#include "bsp.h"                 // Board Support Package
#include "pubsub_signals.h"
#include "app_cli.h"
#include "tusb.h"

#include "stm32g4xx_hal.h"
// add other drivers if necessary...
#include <stdio.h>

//Q_DEFINE_THIS_FILE  // define the name of this file for assertions


#ifdef Q_SPY
#error The Simple Blinky Application does not support Spy build configuration
#endif
#define USB_INTERFACE_CLI    0
#define USB_INTERFACE_PC_COM 1
// #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)

// Static Function Declarations

static uint16_t USB0_TransmitData(const uint8_t *data_ptr, const uint16_t data_len);
static uint16_t USB0_ReceiveData(uint8_t *data_ptr, const uint16_t max_data_len);
static void USB0_RegisterDataReadyCB(Serial_IO_Data_Ready_Callback cb, void *cb_data);

static uint16_t USB1_TransmitData(const uint8_t *data_ptr, const uint16_t data_len);
static uint16_t USB1_ReceiveData(uint8_t *data_ptr, const uint16_t max_data_len);
static void USB1_RegisterDataReadyCB(Serial_IO_Data_Ready_Callback cb, void *cb_data);

// QP Priorities for Active Objects must be unique
// Lower number is lower priority
// 0 is reserved, lowest available is 1
typedef enum
{
    AO_RESERVED = 0U,
    AO_PRIO_BLINKY,
    AO_PRIO_APP_CLI,
    AO_PRIO_USB,
} AO_Priority_T;

static Serial_IO_Data_Ready_Callback s_usb0_data_ready_cb = 0;
static void *s_usb0_data_ready_cb_data                    = 0;

static Serial_IO_Data_Ready_Callback s_usb1_data_ready_cb = 0;
static void *s_usb1_data_ready_cb_data                    = 0;

const Serial_IO_T s_bsp_serial_io_usb0 = {
    .tx_func          = USB0_TransmitData,
    .rx_func          = USB0_ReceiveData,
    .register_cb_func = USB0_RegisterDataReadyCB,
};

static const Serial_IO_T s_bsp_serial_io_usb1 = {
    .tx_func          = USB1_TransmitData,
    .rx_func          = USB1_ReceiveData,
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
    s_usb0_data_ready_cb      = cb;
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
    s_usb1_data_ready_cb      = cb;
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

Q_NORETURN Q_onError(char const * const module, int_t const id) {
    // NOTE: this implementation of the error handler is intended only
    // for debugging and MUST be changed for deployment of the application
    // (assuming that you ship your production code with assertions enabled).
    Q_UNUSED_PAR(module);
    Q_UNUSED_PAR(id);
    QS_ASSERTION(module, id, 10000U);

#ifndef NDEBUG
    // for debugging, hang on in an endless loop...
    for (;;) {
    }
#endif

    NVIC_SystemReset();
}
//............................................................................
void assert_failed(char const * const module, int_t const id); // prototype
void assert_failed(char const * const module, int_t const id) {
    Q_onError(module, id);
}

//............................................................................
void SysTick_Handler(void); // prototype
void SysTick_Handler(void) {
    QTIMEEVT_TICK_X(0U, &l_SysTick_Handler); // time events at rate 0
    QV_ARM_ERRATUM_838869();
}

//============================================================================
// BSP functions...

//............................................................................
void BSP_start(void) {
    // initialize event pools
    static QF_MPOOL_EL(QEvt) smlPoolSto[10];
    QF_poolInit(smlPoolSto, sizeof(smlPoolSto), sizeof(smlPoolSto[0]));

    // initialize publish-subscribe
    static QSubscrList subscrSto[PUBSUB_MAX_SIG];
    QActive_psInit(subscrSto, Q_DIM(subscrSto));

    // instantiate and start AOs/threads...

    static QEvt const *blinkyQueueSto[10];
    Blinky_ctor();
    QACTIVE_START(
		AO_Blinky,
		AO_PRIO_BLINKY,              // QP prio. of the AO
        blinkyQueueSto,              // event queue storage
        Q_DIM(blinkyQueueSto),       // queue length [events]
        (void *)0, 0U,               // no stack storage
        (void *)0);                  // no initialization param



    // static QEvt const *app_cli_QueueSto[10];
    // AppCLI_ctor(BSP_Get_Serial_IO_Interface_USB0());
    // QACTIVE_START(
    //     AO_AppCLI,
    //     AO_PRIO_APP_CLI,         // QP prio. of the AO
    //     app_cli_QueueSto,        // event queue storage
    //     Q_DIM(app_cli_QueueSto), // queue length [events]
    //     (void *) 0,              // stack storage (not used in QK)
    //     0U,                      // stack size [bytes] (not used in QK)
    //     (void *) 0);             // no initialization param

    static QEvt const *usb_QueueSto[10];
    USB_ctor();
    QACTIVE_START(
        AO_USB,
        AO_PRIO_USB,         // QP prio. of the AO
        usb_QueueSto,        // event queue storage
        Q_DIM(usb_QueueSto), // queue length [events]
        (void *) 0,          // stack storage (not used in QK)
        0U,                  // stack size [bytes] (not used in QK)
        (void *) 0);         // no initialization param
}
//............................................................................
void BSP_ledOn() {
  printf("LED on\n\r");
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 1);
}
//............................................................................
void BSP_ledOff() {
  printf("LED off\n\r");
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0);
}
//............................................................................
void BSP_terminate(int16_t result) {
    Q_UNUSED_PAR(result);
}

//============================================================================
// QF callbacks...
void QF_onStartup(void) {
    // set up the SysTick timer to fire at BSP_TICKS_PER_SEC rate
    SysTick_Config(SystemCoreClock / BSP_TICKS_PER_SEC);

    // assign all priority bits for preemption-prio. and none to sub-prio.
    NVIC_SetPriorityGrouping(0U);

    // set priorities of ALL ISRs used in the system, see NOTE1
    NVIC_SetPriority(USART2_IRQn,    0); // kernel UNAWARE interrupt
    NVIC_SetPriority(EXTI0_IRQn,   QF_AWARE_ISR_CMSIS_PRI + 0U);
    NVIC_SetPriority(SysTick_IRQn,   QF_AWARE_ISR_CMSIS_PRI + 1U);
    // ...

    // enable IRQs...
    NVIC_EnableIRQ(EXTI0_IRQn);

#ifdef Q_SPY
    NVIC_EnableIRQ(USART2_IRQn); // UART2 interrupt used for QS-RX
#endif
}
//............................................................................
void QF_onCleanup(void) {
}
//............................................................................
void QV_onIdle(void) { // called with interrupts DISABLED, see NOTE01

    // toggle an LED on and then off (not enough LEDs, see NOTE02)
    //GPIOA->BSRR = (1U << LD4_PIN);         // turn LED[n] on
    //GPIOA->BSRR = (1U << (LD4_PIN + 16U)); // turn LED[n] off

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
    //QV_CPU_SLEEP();  // atomically go to sleep and enable interrupts
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


// PUTCHAR_PROTOTYPE
// {
//   /* Place your implementation of fputc here */
//   /* e.g. write a character to the USART1 and Loop until the end of transmission */
// //  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);

//     tud_cdc_n_write(USB_INTERFACE_CLI, (uint8_t *)&ch, 1);
//     tud_cdc_n_write_flush(USB_INTERFACE_CLI);

//   return ch;
// }