#include <gtest/gtest.h>

#include "Can/Protocols/Private/Zdt/ZdtProtocol.h"

#include <cstdint>
#include <cstring>

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


TEST(ZdtProtocol, SplitsFramesLikeVendorCanExample) {
    // Manual example: 01 FD 01 0F A0 00 00 01 FA 00 00 00 6B
    const uint8_t logical[] = { 0x01, 0xFD, 0x01, 0x0F, 0xA0, 0x00, 0x00, 0x01, 0xFA, 0x00, 0x00, 0x00, 0x6B };
    ZdtProtocol::Frame frames[4] = {};
    const size_t count = ZdtProtocol::build_frames(1, logical + 1, sizeof(logical) - 1, frames, 4);
    ASSERT_EQ(count, 2u);
    EXPECT_EQ(frames[0].id, 0x0100u);
    EXPECT_EQ(frames[0].length, 8u);
    const uint8_t first[] = { 0xFD, 0x01, 0x0F, 0xA0, 0x00, 0x00, 0x01, 0xFA };
    EXPECT_EQ(std::memcmp(frames[0].data, first, sizeof(first)), 0);
    EXPECT_EQ(frames[1].id, 0x0101u);
    EXPECT_EQ(frames[1].length, 5u);
    const uint8_t second[] = { 0xFD, 0x00, 0x00, 0x00, 0x6B };
    EXPECT_EQ(std::memcmp(frames[1].data, second, sizeof(second)), 0);

    EXPECT_EQ(ZdtProtocol::build_frames(1, logical + 1, sizeof(logical) - 1, frames, 1), 0u);
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
    EXPECT_EQ(frame[3], 0x00);
    EXPECT_EQ(frame[4], 0x32);  // X acceleration is native RPM/s, not *256
    EXPECT_EQ(frame[5], 0x00);
    EXPECT_EQ(frame[6], 0x32);
    EXPECT_EQ(frame[7], 0x17);
    EXPECT_EQ(frame[8], 0x70);  // 6000 = 0.1 RPM units
    EXPECT_EQ(frame[9], 0x00);
    EXPECT_EQ(frame[10], 0x00);
    EXPECT_EQ(frame[11], 0x0E);
    EXPECT_EQ(frame[12], 0x10);  // 3600 = 360.0 degrees
    EXPECT_EQ(frame[15], 0x6B);
}

TEST(ZdtProtocol, MapsExecutedStepDeltaToFollowingSpeed) {
    // 16 counts in 20 ms at 3200 counts/rev = 15 RPM.
    EXPECT_EQ(ZdtProtocol::speed_rpm_for_counts(16, 3200, 20, 20), 15u);
    EXPECT_EQ(ZdtProtocol::speed_rpm_for_counts(-16, 3200, 20, 20), 15u);
    // One count still maps to the Emm minimum of 1 RPM.
    EXPECT_EQ(ZdtProtocol::speed_rpm_for_counts(1, 3200, 20, 20), 1u);
    // Never exceed the configured following cap.
    EXPECT_EQ(ZdtProtocol::speed_rpm_for_counts(16, 3200, 10, 20), 20u);
    // Invalid/empty inputs have no speed field to send.
    EXPECT_EQ(ZdtProtocol::speed_rpm_for_counts(0, 3200, 20, 20), 0u);
    EXPECT_EQ(ZdtProtocol::speed_rpm_for_counts(16, 0, 20, 20), 0u);
    EXPECT_EQ(ZdtProtocol::speed_rpm_for_counts(16, 3200, 0, 20), 0u);
}

TEST(ZdtProtocol, SpeedMapAppliesCatchUpMargin) {
    EXPECT_EQ(ZdtProtocol::speed_rpm_for_counts(16, 3200, 20, 20, 25), 19u);
    EXPECT_EQ(ZdtProtocol::speed_rpm_for_counts(16, 3200, 20, 20, 100), 20u);
    EXPECT_EQ(ZdtProtocol::speed_rpm_for_counts(16, 3200, 20, 20, 0), 15u);
}

TEST(ZdtProtocol, RejectsUnsupportedPositionMode) {
    uint8_t frame[ZdtProtocol::MAX_MESSAGE] = {};
    EXPECT_EQ(ZdtProtocol::build_relative_position(
                  ZdtFirmware::Emm, 1, 3200, 600, 50, 3200, ZdtChecksum::Fixed6B, frame, sizeof(frame), nullptr, 1),
              0u);
}

TEST(ZdtProtocol, BuildsReadVersion) {
    uint8_t frame[8] = {};
    size_t  len      = ZdtProtocol::build_read_version(6, ZdtChecksum::Fixed6B, frame, sizeof(frame));
    ASSERT_EQ(len, 3u);
    EXPECT_EQ(frame[0], 6);
    EXPECT_EQ(frame[1], 0x1F);
    EXPECT_EQ(frame[2], 0x6B);
}

TEST(ZdtProtocol, ParsesVersion) {
    // ID6 observed: FW 0x00CF (2.0.7), hardware 0x2114 (series 2, type 28, V2.0).
    const uint8_t frame[] = { 0x1F, 0x00, 0xCF, 0x21, 0x14, 0x6B };
    ZdtProtocol::VersionInfo info;
    ASSERT_TRUE(ZdtProtocol::parse_version(frame, sizeof(frame), info));
    EXPECT_EQ(info.firmware, 0x00CF);
    EXPECT_EQ(info.hardwareSeries, 2);
    EXPECT_EQ(info.hardwareType, 1);
    EXPECT_EQ(info.hardwareVersion, 20);

    ASSERT_FALSE(ZdtProtocol::parse_version(frame, 5, info));
    const uint8_t wrong[] = { 0x36, 0x00, 0xCF, 0x21, 0x14, 0x6B };
    ASSERT_FALSE(ZdtProtocol::parse_version(wrong, sizeof(wrong), info));
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
    const uint8_t reached[] = { 0xFD, 0x9F, 0x6B };
    EXPECT_TRUE(ZdtProtocol::parse_ack(reached, sizeof(reached), 0xFD));
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
