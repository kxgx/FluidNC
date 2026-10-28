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
    EXPECT_EQ(frame[3], 0);     // absolute target decides the direction
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
