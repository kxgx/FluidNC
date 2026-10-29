#include <gtest/gtest.h>

#include "Motors/Pd42s1Protocol.h"

#include <array>
#include <cstdint>

using MotorDrivers::Pd42s1Protocol;

namespace {
    // Fills in the checksum and tail of a hand-built frame so that tests can
    // state the interesting bytes only.
    void finish_response(uint8_t* frame, size_t len) {
        frame[len - 2] = Pd42s1Protocol::checksum(frame, len - 2);
        frame[len - 1] = Pd42s1Protocol::FRAME_TAIL;
    }
}

TEST(Pd42s1Protocol, ChecksumMatchesVendorExample) {
    // The control protocol document's example: C5 01 80 00 -> 0x46.
    const uint8_t data[] = { 0xC5, 0x01, 0x80, 0x00 };
    EXPECT_EQ(Pd42s1Protocol::checksum(data, sizeof(data)), 0x46);
}

TEST(Pd42s1Protocol, BuildsReadCommand) {
    std::array<uint8_t, 8> frame {};
    size_t                 len = Pd42s1Protocol::build_read_command(1, Pd42s1Protocol::FUNC_READ_POSITION, frame.data(), frame.size());

    ASSERT_EQ(len, 5u);
    EXPECT_EQ(frame[0], 0xC5);
    EXPECT_EQ(frame[1], 1);
    EXPECT_EQ(frame[2], Pd42s1Protocol::FUNC_READ_POSITION);
    EXPECT_EQ(frame[3], Pd42s1Protocol::checksum(frame.data(), 3));
    EXPECT_EQ(frame[4], 0x5C);
}

TEST(Pd42s1Protocol, BuildsPositionCommand) {
    std::array<uint8_t, Pd42s1Protocol::MAX_FRAME_LEN> frame {};
    size_t len = Pd42s1Protocol::build_position_command(2, 200, 600, 0x0001C200, frame.data(), frame.size());

    ASSERT_EQ(len, 13u);
    EXPECT_EQ(frame[0], 0xC5);
    EXPECT_EQ(frame[1], 2);
    EXPECT_EQ(frame[2], Pd42s1Protocol::FUNC_POSITION_MODE);
    EXPECT_EQ(frame[3], 0);     // positive target -> forward
    EXPECT_EQ(frame[4], 200);   // accel
    EXPECT_EQ(frame[5], 0x02);  // speed 600 = 0x0258, big endian
    EXPECT_EQ(frame[6], 0x58);
    EXPECT_EQ(frame[7], 0x00);  // position 0x0001C200, big endian
    EXPECT_EQ(frame[8], 0x01);
    EXPECT_EQ(frame[9], 0xC2);
    EXPECT_EQ(frame[10], 0x00);
    EXPECT_EQ(frame[11], Pd42s1Protocol::checksum(frame.data(), 11));
    EXPECT_EQ(frame[12], 0x5C);
}

TEST(Pd42s1Protocol, NegativePositionSetsReverseDirectionByte) {
    std::array<uint8_t, Pd42s1Protocol::MAX_FRAME_LEN> frame {};
    // -12345 = 0xFFFFCFC7 as a two's-complement target.
    size_t len = Pd42s1Protocol::build_position_command(1, 200, 600, uint32_t(int32_t(-12345)), frame.data(), frame.size());

    ASSERT_EQ(len, 13u);
    EXPECT_EQ(frame[3], 1);  // reverse
    EXPECT_EQ(frame[7], 0xFF);
    EXPECT_EQ(frame[8], 0xFF);
    EXPECT_EQ(frame[9], 0xCF);
    EXPECT_EQ(frame[10], 0xC7);
}

TEST(Pd42s1Protocol, BuildsEnableCommand) {
    std::array<uint8_t, Pd42s1Protocol::MAX_FRAME_LEN> frame {};
    size_t                                             len = Pd42s1Protocol::build_enable_command(3, true, frame.data(), frame.size());

    ASSERT_EQ(len, 6u);
    EXPECT_EQ(frame[1], 3);
    EXPECT_EQ(frame[2], Pd42s1Protocol::FUNC_SET_MOTOR_ENABLE);
    EXPECT_EQ(frame[3], 0);  // 0 enables the motor

    len = Pd42s1Protocol::build_enable_command(3, false, frame.data(), frame.size());
    ASSERT_EQ(len, 6u);
    EXPECT_EQ(frame[3], 1);  // 1 disables the motor
}

TEST(Pd42s1Protocol, RejectsFrameThatDoesNotFit) {
    uint8_t frame[4] = {};
    EXPECT_EQ(Pd42s1Protocol::build_read_command(1, Pd42s1Protocol::FUNC_READ_POSITION, frame, sizeof(frame)), 0u);
}

TEST(Pd42s1Protocol, DeterminesResponseLength) {
    const uint8_t shortFrame[] = { 0xC5, 0x01, Pd42s1Protocol::FUNC_READ_POSITION };
    EXPECT_EQ(Pd42s1Protocol::expected_response_length(shortFrame, sizeof(shortFrame)), 0u);

    const uint8_t badHeader[] = { 0x00, 0x01, Pd42s1Protocol::FUNC_READ_POSITION, Pd42s1Protocol::STATUS_OK };
    EXPECT_EQ(Pd42s1Protocol::expected_response_length(badHeader, sizeof(badHeader)), SIZE_MAX);

    const uint8_t position[] = { 0xC5, 0x01, Pd42s1Protocol::FUNC_READ_POSITION, Pd42s1Protocol::STATUS_OK };
    EXPECT_EQ(Pd42s1Protocol::expected_response_length(position, sizeof(position)), 10u);

    const uint8_t version[] = { 0xC5, 0x01, Pd42s1Protocol::FUNC_READ_VERSION, Pd42s1Protocol::STATUS_OK };
    EXPECT_EQ(Pd42s1Protocol::expected_response_length(version, sizeof(version)), 8u);

    const uint8_t error[] = { 0xC5, 0x01, Pd42s1Protocol::FUNC_POSITION_MODE, 0xE4 };
    EXPECT_EQ(Pd42s1Protocol::expected_response_length(error, sizeof(error)), 6u);
}

TEST(Pd42s1Protocol, ParsesVersionResponse) {
    uint8_t frame[8] = { 0xC5, 0x01, Pd42s1Protocol::FUNC_READ_VERSION, Pd42s1Protocol::STATUS_OK, 0x0A, 0x0B };
    finish_response(frame, sizeof(frame));

    EXPECT_TRUE(Pd42s1Protocol::valid_response(frame, sizeof(frame)));

    uint8_t firmware = 0;
    uint8_t hardware = 0;
    ASSERT_TRUE(Pd42s1Protocol::parse_version(frame, sizeof(frame), firmware, hardware));
    EXPECT_EQ(firmware, 0x0A);
    EXPECT_EQ(hardware, 0x0B);
}

TEST(Pd42s1Protocol, ParsesPositionResponse) {
    uint8_t frame[10] = { 0xC5, 0x01, Pd42s1Protocol::FUNC_READ_POSITION, Pd42s1Protocol::STATUS_OK, 0x00, 0x00, 0x01, 0x00 };
    finish_response(frame, sizeof(frame));

    int32_t position = 0;
    ASSERT_TRUE(Pd42s1Protocol::parse_position(frame, sizeof(frame), position));
    EXPECT_EQ(position, 256);
}

TEST(Pd42s1Protocol, ParsesNegativePositionResponse) {
    // Two's complement 0xFFFFFF00 = -256.
    uint8_t frame[10] = { 0xC5, 0x01, Pd42s1Protocol::FUNC_READ_POSITION, Pd42s1Protocol::STATUS_OK, 0xFF, 0xFF, 0xFF, 0x00 };
    finish_response(frame, sizeof(frame));

    int32_t position = 0;
    ASSERT_TRUE(Pd42s1Protocol::parse_position(frame, sizeof(frame), position));
    EXPECT_EQ(position, -256);
}

TEST(Pd42s1Protocol, ParsesEnableAck) {
    uint8_t frame[7] = { 0xC5, 0x03, Pd42s1Protocol::FUNC_SET_MOTOR_ENABLE, Pd42s1Protocol::STATUS_OK, 0x00 };
    finish_response(frame, sizeof(frame));

    EXPECT_TRUE(Pd42s1Protocol::parse_ack(frame, sizeof(frame)));
}

TEST(Pd42s1Protocol, RejectsBadChecksum) {
    uint8_t frame[7] = { 0xC5, 0x03, Pd42s1Protocol::FUNC_SET_MOTOR_ENABLE, Pd42s1Protocol::STATUS_OK, 0x00 };
    finish_response(frame, sizeof(frame));
    frame[sizeof(frame) - 2] ^= 0xFF;

    EXPECT_FALSE(Pd42s1Protocol::valid_response(frame, sizeof(frame)));
    EXPECT_FALSE(Pd42s1Protocol::parse_ack(frame, sizeof(frame)));
}

TEST(Pd42s1Protocol, RejectsErrorStatus) {
    uint8_t frame[6] = { 0xC5, 0x01, Pd42s1Protocol::FUNC_POSITION_MODE, 0xE4 };
    finish_response(frame, sizeof(frame));

    EXPECT_TRUE(Pd42s1Protocol::valid_response(frame, sizeof(frame)));
    EXPECT_FALSE(Pd42s1Protocol::parse_ack(frame, sizeof(frame)));
}

TEST(Pd42s1Protocol, BuildsClearStateCommand) {
    // Manual example 4.4.12: C5 01 FB C1 5C.
    std::array<uint8_t, 8> frame {};
    size_t len = Pd42s1Protocol::build_clear_state_command(1, frame.data(), frame.size());

    ASSERT_EQ(len, 5u);
    EXPECT_EQ(frame[0], 0xC5);
    EXPECT_EQ(frame[1], 1);
    EXPECT_EQ(frame[2], Pd42s1Protocol::FUNC_CLEAR_STATE);
    EXPECT_EQ(frame[3], 0xC1);
    EXPECT_EQ(frame[4], 0x5C);
}

TEST(Pd42s1Protocol, BuildsClearStallCommand) {
    // Manual example 4.4.10: C5 01 F9 BF 5C.
    std::array<uint8_t, 8> frame {};
    size_t len = Pd42s1Protocol::build_clear_stall_command(1, frame.data(), frame.size());

    ASSERT_EQ(len, 5u);
    EXPECT_EQ(frame[2], Pd42s1Protocol::FUNC_CLEAR_STALL);
    EXPECT_EQ(frame[3], 0xBF);
    EXPECT_EQ(frame[4], 0x5C);
}

TEST(Pd42s1Protocol, ParsesClearStateAck) {
    // Manual example 4.4.12 response: C5 01 FB 01 C2 5C.
    uint8_t frame[6] = { 0xC5, 0x01, Pd42s1Protocol::FUNC_CLEAR_STATE, Pd42s1Protocol::STATUS_OK };
    finish_response(frame, sizeof(frame));

    EXPECT_EQ(Pd42s1Protocol::expected_response_length(frame, sizeof(frame)), 6u);
    EXPECT_TRUE(Pd42s1Protocol::valid_response(frame, sizeof(frame)));
    EXPECT_TRUE(Pd42s1Protocol::parse_ack(frame, sizeof(frame)));
}

TEST(Pd42s1Protocol, ParsesMotorStatusByte) {
    // Manual example 4.3.13: C5 01 2C 01 01 F4 5C (status byte = 1, done).
    uint8_t frame[7] = { 0xC5, 0x01, Pd42s1Protocol::FUNC_READ_STATUS, Pd42s1Protocol::STATUS_OK, 0x01 };
    finish_response(frame, sizeof(frame));

    uint8_t value = 0xFF;
    EXPECT_EQ(Pd42s1Protocol::expected_response_length(frame, sizeof(frame)), 7u);
    EXPECT_TRUE(Pd42s1Protocol::parse_byte_value(frame, sizeof(frame), Pd42s1Protocol::FUNC_READ_STATUS, value));
    EXPECT_EQ(value, 1);
}

TEST(Pd42s1Protocol, ParsesStallAndEnableBytes) {
    uint8_t stall[7] = { 0xC5, 0x01, Pd42s1Protocol::FUNC_READ_STALL, Pd42s1Protocol::STATUS_OK, 0x01 };
    finish_response(stall, sizeof(stall));
    uint8_t value = 0xFF;
    EXPECT_TRUE(Pd42s1Protocol::parse_byte_value(stall, sizeof(stall), Pd42s1Protocol::FUNC_READ_STALL, value));
    EXPECT_EQ(value, 1);

    uint8_t enable[7] = { 0xC5, 0x01, Pd42s1Protocol::FUNC_READ_ENABLE, Pd42s1Protocol::STATUS_OK, 0x00 };
    finish_response(enable, sizeof(enable));
    EXPECT_TRUE(Pd42s1Protocol::parse_byte_value(enable, sizeof(enable), Pd42s1Protocol::FUNC_READ_ENABLE, value));
    EXPECT_EQ(value, 0);
}

TEST(Pd42s1Protocol, ParsesSpeedAndPositionError) {
    // Manual 4.3.10: C5 01 29 01 00 1F 0F 5C -> 0x001F = 31 RPM.
    uint8_t speed[8] = { 0xC5, 0x01, Pd42s1Protocol::FUNC_READ_SPEED, Pd42s1Protocol::STATUS_OK, 0x00, 0x1F };
    finish_response(speed, sizeof(speed));
    int16_t rpm = 0;
    EXPECT_EQ(Pd42s1Protocol::expected_response_length(speed, sizeof(speed)), 8u);
    EXPECT_TRUE(Pd42s1Protocol::parse_int16_value(speed, sizeof(speed), Pd42s1Protocol::FUNC_READ_SPEED, rpm));
    EXPECT_EQ(rpm, 31);

    // Manual 4.3.12: C5 01 2B 01 00 00 00 01 F3 5C -> position error 1.
    uint8_t error[10] = { 0xC5, 0x01, Pd42s1Protocol::FUNC_READ_POS_ERROR, Pd42s1Protocol::STATUS_OK, 0x00, 0x00, 0x00, 0x01 };
    finish_response(error, sizeof(error));
    int32_t err = 0;
    EXPECT_EQ(Pd42s1Protocol::expected_response_length(error, sizeof(error)), 10u);
    EXPECT_TRUE(Pd42s1Protocol::parse_int32_value(error, sizeof(error), Pd42s1Protocol::FUNC_READ_POS_ERROR, err));
    EXPECT_EQ(err, 1);
}

TEST(Pd42s1Protocol, BuildsCanBaudCommand) {
    // Manual 4.3.28: C5 01 68 01 F4 23 5C sets 500 kbps.
    std::array<uint8_t, Pd42s1Protocol::MAX_FRAME_LEN> frame {};
    size_t len = Pd42s1Protocol::build_set_can_baud_command(1, 500, frame.data(), frame.size());
    ASSERT_EQ(len, 7u);
    EXPECT_EQ(frame[0], 0xC5);
    EXPECT_EQ(frame[1], 1);
    EXPECT_EQ(frame[2], Pd42s1Protocol::FUNC_SET_CAN_BAUD);
    EXPECT_EQ(frame[3], 0x01);
    EXPECT_EQ(frame[4], 0xF4);
    EXPECT_EQ(frame[5], 0x23);
    EXPECT_EQ(frame[6], 0x5C);
}

TEST(Pd42s1Protocol, BuildsSaveParamsCommand) {
    // Manual example: C5 01 04 CA 5C.
    std::array<uint8_t, Pd42s1Protocol::MAX_FRAME_LEN> frame {};
    size_t len = Pd42s1Protocol::build_save_params_command(1, frame.data(), frame.size());
    ASSERT_EQ(len, 5u);
    EXPECT_EQ(frame[2], Pd42s1Protocol::FUNC_SAVE_PARAMS);
    EXPECT_EQ(frame[3], 0xCA);
    EXPECT_EQ(frame[4], 0x5C);
}

TEST(Pd42s1Protocol, DeterminesLongParameterResponseLengths) {
    uint8_t system[4] = { 0xC5, 0x01, 0x31, Pd42s1Protocol::STATUS_OK };
    EXPECT_EQ(Pd42s1Protocol::expected_response_length(system, sizeof(system)), 46u);

    uint8_t driver[4] = { 0xC5, 0x01, 0x32, Pd42s1Protocol::STATUS_OK };
    EXPECT_EQ(Pd42s1Protocol::expected_response_length(driver, sizeof(driver)), 50u);
}
