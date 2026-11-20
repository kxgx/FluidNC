#include <gtest/gtest.h>

#include "Can/Protocols/Private/Zdt/ZdtProtocol.h"

#include <cstdint>

using MotorDrivers::ZdtChecksum;
using MotorDrivers::ZdtFirmware;
using MotorDrivers::ZdtProtocol;

TEST(ZdtProtocol, BuildsReadPosition) {
    uint8_t frame[8] = {};
    size_t  len      = ZdtProtocol::build_read_position(3, ZdtChecksum::Fixed6B, frame, sizeof(frame));
    ASSERT_EQ(len, 3u);
    EXPECT_EQ(frame[0], 3);
    EXPECT_EQ(frame[1], 0x36);
    EXPECT_EQ(frame[2], 0x6B);
}

TEST(ZdtProtocol, BuildsEnable) {
    uint8_t frame[8] = {};
    size_t  len      = ZdtProtocol::build_enable(1, true, ZdtChecksum::Fixed6B, frame, sizeof(frame));
    ASSERT_EQ(len, 6u);
    EXPECT_EQ(frame[0], 1);
    EXPECT_EQ(frame[1], 0xF3);
    EXPECT_EQ(frame[2], 0xAB);
    EXPECT_EQ(frame[3], 1);
    EXPECT_EQ(frame[4], 0);
    EXPECT_EQ(frame[5], 0x6B);
}

TEST(ZdtProtocol, BuildsEmmRelativePosition) {
    uint8_t frame[ZdtProtocol::MAX_MESSAGE] = {};
    size_t  len = ZdtProtocol::build_relative_position(
        ZdtFirmware::Emm, 2, 3200, 600, 50, 3200, ZdtChecksum::Fixed6B, frame, sizeof(frame));
    ASSERT_EQ(len, 13u);
    EXPECT_EQ(frame[0], 2);
    EXPECT_EQ(frame[1], 0xFD);
    EXPECT_EQ(frame[2], 0);  // positive
    EXPECT_EQ(frame[3], 0x02);
    EXPECT_EQ(frame[4], 0x58);  // 600 RPM
    EXPECT_EQ(frame[5], 50);
    EXPECT_EQ(frame[6], 0x00);
    EXPECT_EQ(frame[7], 0x00);
    EXPECT_EQ(frame[8], 0x0C);
    EXPECT_EQ(frame[9], 0x80);  // 3200 pulses
    EXPECT_EQ(frame[10], 0);    // relative to previous target
    EXPECT_EQ(frame[11], 0);
    EXPECT_EQ(frame[12], 0x6B);
}

TEST(ZdtProtocol, NegativeEmmPositionUsesDirectionBit) {
    uint8_t frame[ZdtProtocol::MAX_MESSAGE] = {};
    size_t  len = ZdtProtocol::build_relative_position(
        ZdtFirmware::Emm, 1, -3200, 600, 0, 3200, ZdtChecksum::Fixed6B, frame, sizeof(frame));
    ASSERT_EQ(len, 13u);
    EXPECT_EQ(frame[2], 1);
    EXPECT_EQ(frame[6], 0x00);
    EXPECT_EQ(frame[7], 0x00);
    EXPECT_EQ(frame[8], 0x0C);
    EXPECT_EQ(frame[9], 0x80);
}

TEST(ZdtProtocol, XPositionUsesTenthsOfDegree) {
    uint8_t frame[ZdtProtocol::MAX_MESSAGE] = {};
    // 3200 pulses is one revolution = 3600 units of 0.1 degree.
    size_t len = ZdtProtocol::build_relative_position(
        ZdtFirmware::X, 1, 3200, 600, 50, 3200, ZdtChecksum::Fixed6B, frame, sizeof(frame));
    ASSERT_EQ(len, 16u);
    EXPECT_EQ(frame[1], 0xFD);
    EXPECT_EQ(frame[2], 0);
    EXPECT_EQ(frame[7], 0x17);
    EXPECT_EQ(frame[8], 0x70);  // 6000 = 0.1 RPM units
    EXPECT_EQ(frame[9], 0x00);
    EXPECT_EQ(frame[10], 0x00);
    EXPECT_EQ(frame[11], 0x0E);
    EXPECT_EQ(frame[12], 0x10);  // 3600 = 360.0 degrees
    EXPECT_EQ(frame[15], 0x6B);
}

TEST(ZdtProtocol, ChecksumModes) {
    const uint8_t data[] = { 0x01, 0x06, 0x45 };
    EXPECT_EQ(ZdtProtocol::checksum(data, sizeof(data), ZdtChecksum::Fixed6B), 0x6B);
    EXPECT_EQ(ZdtProtocol::checksum(data, sizeof(data), ZdtChecksum::Xor), uint8_t(0x01 ^ 0x06 ^ 0x45));
}

TEST(ZdtProtocol, ParsesAck) {
    const uint8_t ack[] = { 0xFD, 0x02, 0x6B };
    EXPECT_TRUE(ZdtProtocol::parse_ack(ack, sizeof(ack), 0xFD));
    EXPECT_FALSE(ZdtProtocol::parse_ack(ack, sizeof(ack), 0xF3));
    const uint8_t error[] = { 0xFD, 0xE2, 0x6B };
    EXPECT_FALSE(ZdtProtocol::parse_ack(error, sizeof(error), 0xFD));
}

TEST(ZdtProtocol, ParsesEmmPosition) {
    // 0x36, positive, 180 degrees = 32768 of 65536.
    const uint8_t frame[] = { 0x36, 0x00, 0x00, 0x00, 0x80, 0x00, 0x6B };
    int32_t       counts  = 0;
    ASSERT_TRUE(ZdtProtocol::parse_position(ZdtFirmware::Emm, frame, sizeof(frame), 3200, counts));
    EXPECT_EQ(counts, 1600);
}

TEST(ZdtProtocol, ParsesXPosition) {
    // 0x36, positive, 180.0 degrees = 1800 units of 0.1 degree.
    const uint8_t frame[] = { 0x36, 0x00, 0x00, 0x00, 0x07, 0x08, 0x6B };
    int32_t       counts  = 0;
    ASSERT_TRUE(ZdtProtocol::parse_position(ZdtFirmware::X, frame, sizeof(frame), 3200, counts));
    EXPECT_EQ(counts, 1600);
}
