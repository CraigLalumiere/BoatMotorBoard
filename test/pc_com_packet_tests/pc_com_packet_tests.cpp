extern "C" {
#include "c/MessageType.pb.h"
#include "c/MotorData.pb.h"
#include "pc_com.h"
#include "pc_com/crc16.h"
#include "pc_com/hdlc.h"
#include "pb_decode.h"
#include "pubsub_signals.h"
}

#include "cms_cpputest_qf_ctrl.hpp"

#include "CppUTest/TestHarness.h"

#include <cstring>

using namespace cms::test;

static QEvt const *s_queue_storage[10];
static uint8_t s_tx_bytes[256];
static size_t s_tx_len;
static Serial_IO_Data_Ready_Callback s_data_ready_cb;
static void *s_data_ready_cb_data;

extern "C" uint32_t BSP_Get_Milliseconds_Tick(void)
{
    return 4321U;
}

static uint16_t serial_tx(const uint8_t *data_ptr, const uint16_t data_len)
{
    if ((s_tx_len + data_len) > sizeof(s_tx_bytes))
    {
        return 0U;
    }

    memcpy(&s_tx_bytes[s_tx_len], data_ptr, data_len);
    s_tx_len += data_len;
    return data_len;
}

static uint16_t serial_rx(uint8_t *, const uint16_t)
{
    return 0U;
}

static void serial_register_cb(Serial_IO_Data_Ready_Callback cb, void *cb_data)
{
    s_data_ready_cb      = cb;
    s_data_ready_cb_data = cb_data;
}

static const Serial_IO_T s_serial = {
    .tx_func          = serial_tx,
    .rx_func          = serial_rx,
    .register_cb_func = serial_register_cb,
};

static size_t unpack_last_frame(uint8_t *packet, size_t packet_len)
{
    HDLC_Unpacker_T unpacker;
    hdlc_unpacker_init(&unpacker, packet, packet_len);

    HDLC_Unpack_State_T state = FRAME_UNPACK_WAIT_SYNC;
    for (size_t i = 0; i < s_tx_len; i++)
    {
        state = hdlc_unpacker_add_byte(&unpacker, s_tx_bytes[i]);
    }

    CHECK_EQUAL(FRAME_UNPACK_COMPLETE, state);
    return unpacker.packet_length;
}

TEST_GROUP(PcComPacketTests) {
    void setup() final
    {
        qf_ctrl::MemPoolConfigs configs = {
            {sizeof(MotorDataEvent_T), 4},
            {sizeof(PCCOMCliDataEvent_T), 4},
        };

        s_tx_len           = 0;
        s_data_ready_cb    = nullptr;
        s_data_ready_cb_data = nullptr;

        qf_ctrl::Setup(
            PUBSUB_MAX_SIG,
            1000,
            configs,
            qf_ctrl::MemPoolTeardownOption::IGNORE);
        PC_COM_ctor(&s_serial);
        QACTIVE_START(
            AO_PC_COM,
            qf_ctrl::UNIT_UNDER_TEST_PRIORITY,
            s_queue_storage,
            Q_DIM(s_queue_storage),
            nullptr,
            0,
            nullptr);
        qf_ctrl::ProcessEvents();
    }

    void teardown() final
    {
        qf_ctrl::Teardown();
    }
};

TEST(PcComPacketTests, startup_registers_serial_data_ready_callback)
{
    CHECK_TRUE(s_data_ready_cb != nullptr);
    CHECK_TRUE(s_data_ready_cb_data != nullptr);
}

TEST(PcComPacketTests, motor_data_event_transmits_motor_data_protobuf_packet)
{
    MotorDataEvent_T event = {
        .super          = QEVT_INITIALIZER(PUBSUB_MOTOR_DATA_SIG),
        .temperature    = 70.5F,
        .pressure       = 9.75F,
        .tachometer     = 1450.0F,
        .vbat           = 12.4F,
        .engine_minutes = 321U,
        .start          = true,
        .neutral        = false,
        .buzzer         = true,
        .temp_good      = false,
        .pres_good      = true,
    };

    qf_ctrl::PublishAndProcess(&event.super);

    uint8_t packet[128];
    size_t packet_len = unpack_last_frame(packet, sizeof(packet));

    CHECK_TRUE(packet_len > 3U);

    uint16_t packet_crc = (uint16_t) packet[0] | ((uint16_t) packet[1] << 8U);
    CHECK_EQUAL(packet_crc, crc_calculate(&packet[2], (uint16_t) (packet_len - 2U)));
    CHECK_EQUAL(MessageType_MOTOR_DATA, packet[2]);

    MotorData decoded = MotorData_init_zero;
    pb_istream_t stream = pb_istream_from_buffer(&packet[3], packet_len - 3U);
    CHECK_TRUE(pb_decode(&stream, MotorData_fields, &decoded));

    CHECK_EQUAL(4321U, decoded.milliseconds_tick);
    DOUBLES_EQUAL(event.temperature, decoded.temperature, 0.001);
    DOUBLES_EQUAL(event.pressure, decoded.pressure, 0.001);
    DOUBLES_EQUAL(event.tachometer, decoded.tachometer, 0.001);
    DOUBLES_EQUAL(event.vbat, decoded.vbat, 0.001);
    CHECK_EQUAL(event.engine_minutes, decoded.engine_minutes);
    CHECK_EQUAL(event.start, decoded.start);
    CHECK_EQUAL(event.neutral, decoded.neutral);
    CHECK_EQUAL(event.buzzer, decoded.buzzer);
    CHECK_EQUAL(event.temp_good, decoded.temp_good);
    CHECK_EQUAL(event.pres_good, decoded.pres_good);
}
