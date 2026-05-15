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
#define AVREF                               2.895

/**************************************************************************************************\
* Private type definitions
\**************************************************************************************************/

/**************************************************************************************************\
* Private prototypes
\**************************************************************************************************/

static uint16_t USB0_TransmitData(const uint8_t *data_ptr, const uint16_t data_len);
static uint16_t USB0_ReceiveData(uint8_t *data_ptr, const uint16_t max_data_len);
static void USB0_RegisterDataReadyCB(Serial_IO_Data_Ready_Callback cb, void *cb_data);

static uint16_t volts_to_code12(float volts);

/**************************************************************************************************\
* Private memory declarations
\**************************************************************************************************/

// CubeMX-generated handles (defined in main.c)
extern DAC_HandleTypeDef hdac1;
extern DAC_HandleTypeDef hdac3;
extern OPAMP_HandleTypeDef hopamp1;
extern TIM_HandleTypeDef htim8;
extern FDCAN_HandleTypeDef hfdcan2; // defined in main.c by cubeMX

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

/**************************************************************************************************\
* Gauge / Actuator outputs
\**************************************************************************************************/

void BSP_Gauge_SetPressure_V(float volts)
{
    uint16_t code12 = volts_to_code12(volts);

    HAL_StatusTypeDef retval = HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, code12);
    Q_ASSERT(retval == HAL_OK);
}

void BSP_Gauge_SetTemperature_V(float volts)
{
    uint16_t code12 = volts_to_code12(volts);

    HAL_StatusTypeDef retval = HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, code12);
    Q_ASSERT(retval == HAL_OK);
}

void BSP_Gauge_SetOpAmpRef_V(float volts)
{
    uint16_t code12 = volts_to_code12(volts);

    HAL_StatusTypeDef retval = HAL_DAC_SetValue(&hdac3, DAC_CHANNEL_1, DAC_ALIGN_12B_R, code12);
    Q_ASSERT(retval == HAL_OK);
}

void BSP_RpmGauge_SetPFM_RPM(uint32_t target_RPM)
{
    uint32_t target_Hz = target_RPM / 60 * 10;
    if (target_Hz == 0U)
    {
        // keep PWM running but force 0% duty => steady low
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0U);
        return;
    }

    // 144MHz base click with 72 PSC
    uint32_t tim_clk = 2000000U;

    uint32_t arr = tim_clk / target_Hz;
    if (arr == 0U)
    {
        arr = 1U;
    }
    arr -= 1U;

    __HAL_TIM_SET_AUTORELOAD(&htim8, arr);

    // 50% duty
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, (arr + 1U) / 2U);

    // __HAL_TIM_GENERATE_EVENT(&htim8, TIM_EVENTSOURCE_UPDATE);
}

/**
 * @brief Read digital input from backlight switch
 */
bool BSP_Get_Backlight(void)
{
    return HAL_GPIO_ReadPin(BACKLIGHT_DET_GPIO_Port, BACKLIGHT_DET_Pin) == GPIO_PIN_SET;
}

/**
 * @brief Enable/disable backlight on gauge cluster
 */
void BSP_Set_Backlight(bool x)
{
    HAL_GPIO_WritePin(BACKLIGHT_EN_GPIO_Port, BACKLIGHT_EN_Pin, x);
}

/**
 ***************************************************************************************************
 *
 * @brief   Write CAN Message with Standard ID (range of 0 to 0x7FF)
 *
 * @retval  0 if message is sucsssfully queued into TX mailbox
 * @retval  1 if TX mailbox is full and message cannot be send (likely BUS error or
 *disconnected)
 *
 **************************************************************************************************/
int32_t BSP_CAN_Write_Msg(const CAN_Message_T *msg)
{
    HAL_StatusTypeDef retval;
    FDCAN_TxHeaderTypeDef TxHeader;

    /* Prepare Tx Header */
    TxHeader.Identifier          = msg->id;
    TxHeader.IdType              = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType         = FDCAN_DATA_FRAME;
    TxHeader.DataLength          = (uint32_t) msg->dlc;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch       = FDCAN_BRS_OFF;
    TxHeader.FDFormat            = FDCAN_FD_CAN;
    TxHeader.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker       = 0;

    retval = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader, msg->data);

    return (int32_t) retval;
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

/**
 ***************************************************************************************************
 *
 * @brief   Configure FDCAN
 *
 **************************************************************************************************/
void BSP_CAN_Bus_Init(void)
{
    FDCAN_FilterTypeDef sFilterConfig;
    HAL_StatusTypeDef retval;

    // Configure Rx filter
    sFilterConfig.IdType       = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex  = 0;
    sFilterConfig.FilterType   = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1    = 0x00000001U; // id = don't care
    sFilterConfig.FilterID2    = 0x00000000U; // mask = allow all (don't compare any ID)

    retval = HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig);
    Q_ASSERT(retval == HAL_OK);

    // Configure global filter:
    // Filter all remote frames with STD and EXT ID
    // Reject non matching frames with STD ID and EXT ID
    retval = HAL_FDCAN_ConfigGlobalFilter(
        &hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
    Q_ASSERT(retval == HAL_OK);

    // Start the FDCAN module
    retval = HAL_FDCAN_Start(&hfdcan2);
    Q_ASSERT(retval == HAL_OK);

    retval = HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    Q_ASSERT(retval == HAL_OK);
}

//............................................................................
void BSP_Init(void)
{
    HAL_StatusTypeDef retval;

    // initialize TinyUSB device stack on configured roothub port
    tud_init(BOARD_TUD_RHPORT);

    // --- Start DAC1 external outputs ---
    retval = HAL_DAC_Start(&hdac1, DAC_CHANNEL_1); // DAC1_OUT1
    Q_ASSERT(retval == HAL_OK);

    retval = HAL_DAC_Start(&hdac1, DAC_CHANNEL_2); // DAC1_OUT2
    Q_ASSERT(retval == HAL_OK);

    // --- Start DAC3 driving OPAMP1 follower input ---
    retval = HAL_DAC_Start(&hdac3, DAC_CHANNEL_1); // DAC3_OUT1 -> OPAMP1 VINP
    Q_ASSERT(retval == HAL_OK);

    retval = HAL_OPAMP_Start(&hopamp1); // buffer DAC3 via OPAMP1 follower
    Q_ASSERT(retval == HAL_OK);

    // -- Start TIM8 for TACH PFM output --
    retval = HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    Q_ASSERT(retval == HAL_OK);
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
void BSP_LED_Toggle()
{
    HAL_GPIO_TogglePin(FW_LED_GPIO_Port, FW_LED_Pin);
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

/**************************************************************************************************\
* BSP Helper Functions
\**************************************************************************************************/

static uint16_t volts_to_code12(float volts)
{
    if (volts <= 0.0f)
    {
        return 0U;
    }
    if (volts >= AVREF)
    {
        return 0x0FFFU;
    }
    float ratio   = volts / AVREF;
    uint32_t code = (uint32_t) (ratio * 4095.0f + 0.5f);
    if (code > 0x0FFFU)
    {
        code = 0x0FFFU;
    }
    return (uint16_t) code;
}