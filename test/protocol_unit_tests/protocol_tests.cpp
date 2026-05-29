extern "C" {
#ifndef restrict
#define restrict __restrict
#endif
#include "pc_com/crc16.h"
#include "pc_com/hdlc.h"
#include "safe_strncpy.h"
}

#include <cstdint>
#include <cstring>

#include "CppUTest/TestHarness.h"

static uint8_t s_tx_bytes[64];
static size_t s_tx_len;
static bool s_tx_should_fail;

static uint16_t capture_tx_bytes(const uint8_t *data_ptr, const uint16_t data_len)
{
    if (s_tx_should_fail)
    {
        return 0;
    }

    if ((s_tx_len + data_len) > sizeof(s_tx_bytes))
    {
        return 0;
    }

    for (uint16_t i = 0; i < data_len; i++)
    {
        s_tx_bytes[s_tx_len++] = data_ptr[i];
    }

    return data_len;
}

TEST_GROUP(Crc16Tests) {
};

TEST(Crc16Tests, empty_buffer_returns_initial_crc)
{
    CHECK_EQUAL(0xffffU, crc_calculate(reinterpret_cast<const uint8_t *>(""), 0));
}

TEST(Crc16Tests, standard_modbus_check_vector_matches)
{
    const uint8_t data[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    CHECK_EQUAL(0x4b37U, crc_calculate(data, sizeof(data)));
}

TEST(Crc16Tests, changed_byte_changes_crc)
{
    const uint8_t data_a[] = {0x02, 0x10, 0x01, 0x00};
    const uint8_t data_b[] = {0x02, 0x10, 0x01, 0x01};

    CHECK_TRUE(crc_calculate(data_a, sizeof(data_a)) != crc_calculate(data_b, sizeof(data_b)));
}

TEST_GROUP(HdlcTransmitTests) {
    void setup() final
    {
        s_tx_len         = 0;
        s_tx_should_fail = false;
    }
};

TEST(HdlcTransmitTests, wraps_packet_in_flags)
{
    uint8_t packet[] = {0x01, 0x02, 0x03};

    CHECK_EQUAL(0, hdlc_transmit_packet(capture_tx_bytes, packet, sizeof(packet)));

    const uint8_t expected[] = {0x7e, 0x01, 0x02, 0x03, 0x7e};
    CHECK_EQUAL(sizeof(expected), s_tx_len);
    MEMCMP_EQUAL(expected, s_tx_bytes, sizeof(expected));
}

TEST(HdlcTransmitTests, escapes_flag_and_dle_bytes)
{
    uint8_t packet[] = {0x01, 0x7e, 0x02, 0x7d};

    CHECK_EQUAL(0, hdlc_transmit_packet(capture_tx_bytes, packet, sizeof(packet)));

    const uint8_t expected[] = {0x7e, 0x01, 0x7d, 0x5e, 0x02, 0x7d, 0x5d, 0x7e};
    CHECK_EQUAL(sizeof(expected), s_tx_len);
    MEMCMP_EQUAL(expected, s_tx_bytes, sizeof(expected));
}

TEST(HdlcTransmitTests, reports_error_if_tx_callback_fails)
{
    uint8_t packet[] = {0x01};
    s_tx_should_fail = true;

    CHECK_EQUAL(-1, hdlc_transmit_packet(capture_tx_bytes, packet, sizeof(packet)));
}

TEST_GROUP(HdlcUnpackTests) {
    HDLC_Unpacker_T unpacker;
    uint8_t buffer[8];

    void setup() final
    {
        s_tx_len = 0;
        s_tx_should_fail = false;
        memset(buffer, 0, sizeof(buffer));
        hdlc_unpacker_init(&unpacker, buffer, sizeof(buffer));
    }
};

TEST(HdlcUnpackTests, ignores_bytes_until_first_flag)
{
    CHECK_EQUAL(FRAME_UNPACK_WAIT_SYNC, hdlc_unpacker_add_byte(&unpacker, 0x01));
    CHECK_EQUAL(0U, unpacker.packet_length);

    CHECK_EQUAL(FRAME_UNPACK_AFTER_FLAG, hdlc_unpacker_add_byte(&unpacker, 0x7e));
}

TEST(HdlcUnpackTests, unpacks_simple_frame)
{
    const uint8_t frame[] = {0x7e, 0x01, 0x02, 0x03, 0x7e};
    HDLC_Unpack_State_T state = FRAME_UNPACK_WAIT_SYNC;

    for (uint8_t byte : frame)
    {
        state = hdlc_unpacker_add_byte(&unpacker, byte);
    }

    CHECK_EQUAL(FRAME_UNPACK_COMPLETE, state);
    CHECK_EQUAL(3U, unpacker.packet_length);
    const uint8_t expected[] = {0x01, 0x02, 0x03};
    MEMCMP_EQUAL(expected, buffer, sizeof(expected));
}

TEST(HdlcUnpackTests, unescapes_flag_and_dle_bytes)
{
    const uint8_t frame[] = {0x7e, 0x7d, 0x5e, 0x7d, 0x5d, 0x7e};
    HDLC_Unpack_State_T state = FRAME_UNPACK_WAIT_SYNC;

    for (uint8_t byte : frame)
    {
        state = hdlc_unpacker_add_byte(&unpacker, byte);
    }

    CHECK_EQUAL(FRAME_UNPACK_COMPLETE, state);
    CHECK_EQUAL(2U, unpacker.packet_length);
    const uint8_t expected[] = {0x7e, 0x7d};
    MEMCMP_EQUAL(expected, buffer, sizeof(expected));
}

TEST(HdlcUnpackTests, round_trips_transmitted_frame)
{
    uint8_t packet[] = {0x10, 0x7e, 0x20, 0x7d, 0x30};
    CHECK_EQUAL(0, hdlc_transmit_packet(capture_tx_bytes, packet, sizeof(packet)));

    HDLC_Unpack_State_T state = FRAME_UNPACK_WAIT_SYNC;
    for (size_t i = 0; i < s_tx_len; i++)
    {
        state = hdlc_unpacker_add_byte(&unpacker, s_tx_bytes[i]);
    }

    CHECK_EQUAL(FRAME_UNPACK_COMPLETE, state);
    CHECK_EQUAL(sizeof(packet), unpacker.packet_length);
    MEMCMP_EQUAL(packet, buffer, sizeof(packet));
}

TEST(HdlcUnpackTests, resets_when_packet_buffer_fills)
{
    uint8_t small_buffer[3];
    hdlc_unpacker_init(&unpacker, small_buffer, sizeof(small_buffer));

    hdlc_unpacker_add_byte(&unpacker, 0x7e);
    hdlc_unpacker_add_byte(&unpacker, 0x01);
    hdlc_unpacker_add_byte(&unpacker, 0x02);
    CHECK_EQUAL(FRAME_UNPACK_WAIT_SYNC, hdlc_unpacker_add_byte(&unpacker, 0x03));
    CHECK_EQUAL(0U, unpacker.packet_length);
}

TEST_GROUP(SafeStrncpyTests) {
};

TEST(SafeStrncpyTests, copies_short_string_and_null_terminates)
{
    char dest[8];
    memset(dest, 'x', sizeof(dest));

    char *result = safe_strncpy(dest, "abc", sizeof(dest));

    POINTERS_EQUAL(dest, result);
    STRCMP_EQUAL("abc", dest);
}

TEST(SafeStrncpyTests, truncates_and_null_terminates)
{
    char dest[5];
    memset(dest, 'x', sizeof(dest));

    safe_strncpy(dest, "abcdef", sizeof(dest));

    STRCMP_EQUAL("abcd", dest);
    CHECK_EQUAL('\0', dest[sizeof(dest) - 1]);
}

TEST(SafeStrncpyTests, exact_fit_is_truncated_to_leave_terminator)
{
    char dest[4];
    memset(dest, 'x', sizeof(dest));

    safe_strncpy(dest, "abcd", sizeof(dest));

    STRCMP_EQUAL("abc", dest);
}
