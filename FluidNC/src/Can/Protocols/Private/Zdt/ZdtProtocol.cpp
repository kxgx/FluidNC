// Copyright (c) 2026 - FluidNC contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "Config.h"

#if MAX_N_CAN

#    include "ZdtProtocol.h"

#    include <algorithm>
#    include <cmath>
#    include <cstring>

namespace MotorDrivers {
    namespace {
        const uint8_t kCrc8Table[256] = {
            0x00, 0x5E, 0xBC, 0xE2, 0x61, 0x3F, 0xDD, 0x83, 0xC2, 0x9C, 0x7E, 0x20, 0xA3, 0xFD, 0x1F, 0x41, 0x9D, 0xC3, 0x21,
            0x7F, 0xFC, 0xA2, 0x40, 0x1E, 0x5F, 0x01, 0xE3, 0xBD, 0x3E, 0x60, 0x82, 0xDC, 0x23, 0x7D, 0x9F, 0xC1, 0x42, 0x1C,
            0xFE, 0xA0, 0xE1, 0xBF, 0x5D, 0x03, 0x80, 0xDE, 0x3C, 0x62, 0xBE, 0xE0, 0x02, 0x5C, 0xDF, 0x81, 0x63, 0x3D, 0x7C,
            0x22, 0xC0, 0x9E, 0x1D, 0x43, 0xA1, 0xFF, 0x46, 0x18, 0xFA, 0xA4, 0x27, 0x79, 0x9B, 0xC5, 0x84, 0xDA, 0x38, 0x66,
            0xE5, 0xBB, 0x59, 0x07, 0xDB, 0x85, 0x67, 0x39, 0xBA, 0xE4, 0x06, 0x58, 0x19, 0x47, 0xA5, 0xFB, 0x78, 0x26, 0xC4,
            0x9A, 0x65, 0x3B, 0xD9, 0x87, 0x04, 0x5A, 0xB8, 0xE6, 0xA7, 0xF9, 0x1B, 0x45, 0xC6, 0x98, 0x7A, 0x24, 0xF8, 0xA6,
            0x44, 0x1A, 0x99, 0xC7, 0x25, 0x7B, 0x3A, 0x64, 0x86, 0xD8, 0x5B, 0x05, 0xE7, 0xB9, 0x8C, 0xD2, 0x30, 0x6E, 0xED,
            0xB3, 0x51, 0x0F, 0x4E, 0x10, 0xF2, 0xAC, 0x2F, 0x71, 0x93, 0xCD, 0x11, 0x4F, 0xAD, 0xF3, 0x70, 0x2E, 0xCC, 0x92,
            0xD3, 0x8D, 0x6F, 0x31, 0xB2, 0xEC, 0x0E, 0x50, 0xAF, 0xF1, 0x13, 0x4D, 0xCE, 0x90, 0x72, 0x2C, 0x6D, 0x33, 0xD1,
            0x8F, 0x0C, 0x52, 0xB0, 0xEE, 0x32, 0x6C, 0x8E, 0xD0, 0x53, 0x0D, 0xEF, 0xB1, 0xF0, 0xAE, 0x4C, 0x12, 0x91, 0xCF,
            0x2D, 0x73, 0xCA, 0x94, 0x76, 0x28, 0xAB, 0xF5, 0x17, 0x49, 0x08, 0x56, 0xB4, 0xEA, 0x69, 0x37, 0xD5, 0x8B, 0x57,
            0x09, 0xEB, 0xB5, 0x36, 0x68, 0x8A, 0xD4, 0x95, 0xCB, 0x29, 0x77, 0xF4, 0xAA, 0x48, 0x16, 0xE9, 0xB7, 0x55, 0x0B,
            0x88, 0xD6, 0x34, 0x6A, 0x2B, 0x75, 0x97, 0xC9, 0x4A, 0x14, 0xF6, 0xA8, 0x74, 0x2A, 0xC8, 0x96, 0x15, 0x4B, 0xA9,
            0xF7, 0xB6, 0xE8, 0x0A, 0x54, 0xD7, 0x89, 0x6B, 0x35,
        };

        void put16(uint8_t* p, uint16_t v) {
            p[0] = uint8_t(v >> 8);
            p[1] = uint8_t(v);
        }

        void put32(uint8_t* p, uint32_t v) {
            p[0] = uint8_t(v >> 24);
            p[1] = uint8_t(v >> 16);
            p[2] = uint8_t(v >> 8);
            p[3] = uint8_t(v);
        }

        uint32_t get32(const uint8_t* p) {
            return (uint32_t(p[0]) << 24) | (uint32_t(p[1]) << 16) | (uint32_t(p[2]) << 8) | uint32_t(p[3]);
        }

        size_t finish(uint8_t* out, size_t len, ZdtChecksum mode) {
            out[len] = ZdtProtocol::checksum(out, len, mode);
            return len + 1;
        }
    }

    uint8_t ZdtProtocol::checksum(const uint8_t* data, size_t len, ZdtChecksum mode) {
        switch (mode) {
            case ZdtChecksum::Fixed6B:
                return 0x6B;
            case ZdtChecksum::Xor: {
                uint8_t value = 0;
                for (size_t i = 0; i < len; ++i) {
                    value ^= data[i];
                }
                return value;
            }
            case ZdtChecksum::Crc8: {
                uint8_t value = data[0];
                for (size_t i = 1; i < len; ++i) {
                    value = kCrc8Table[value ^ data[i]];
                }
                return value;
            }
        }
        return 0x6B;
    }

    size_t ZdtProtocol::build_read_position(uint8_t address, ZdtChecksum checksumMode, uint8_t* out, size_t capacity) {
        if (capacity < 3) {
            return 0;
        }
        out[0] = address;
        out[1] = 0x36;
        return finish(out, 2, checksumMode);
    }

    size_t ZdtProtocol::build_enable(uint8_t address, bool enable, ZdtChecksum checksumMode, uint8_t* out, size_t capacity) {
        if (capacity < 6) {
            return 0;
        }
        out[0] = address;
        out[1] = 0xF3;
        out[2] = 0xAB;
        out[3] = enable ? 1 : 0;
        out[4] = 0;
        return finish(out, 5, checksumMode);
    }

    size_t ZdtProtocol::build_stop(uint8_t address, ZdtChecksum checksumMode, uint8_t* out, size_t capacity) {
        if (capacity < 5) {
            return 0;
        }
        out[0] = address;
        out[1] = 0xFE;
        out[2] = 0x98;
        out[3] = 0;
        return finish(out, 4, checksumMode);
    }

    size_t ZdtProtocol::build_zero(uint8_t address, ZdtChecksum checksumMode, uint8_t* out, size_t capacity) {
        if (capacity < 4) {
            return 0;
        }
        out[0] = address;
        out[1] = 0x0A;
        out[2] = 0x6D;
        return finish(out, 3, checksumMode);
    }

    size_t ZdtProtocol::build_sync(uint8_t address, ZdtChecksum checksumMode, uint8_t* out, size_t capacity) {
        if (capacity < 4) {
            return 0;
        }
        out[0] = address;
        out[1] = 0xFF;
        out[2] = 0x66;
        return finish(out, 3, checksumMode);
    }

    size_t ZdtProtocol::build_relative_position(ZdtFirmware firmware,
                                                uint8_t      address,
                                                int32_t      delta_counts,
                                                uint16_t     speed_rpm,
                                                uint16_t     accel,
                                                uint32_t     counts_per_rev,
                                                ZdtChecksum  checksumMode,
                                                uint8_t*     out,
                                                size_t       capacity,
                                                double*      angle_residual,
                                                uint8_t      position_mode) {
        if (counts_per_rev == 0 || delta_counts == 0 || position_mode != 0) {
            return 0;
        }
        const bool     negative = delta_counts < 0;
        const uint32_t magnitude = negative ? uint32_t(-int64_t(delta_counts)) : uint32_t(delta_counts);

        if (firmware == ZdtFirmware::Emm) {
            if (capacity < 13) {
                return 0;
            }
            const uint16_t velocity = std::min<uint16_t>(speed_rpm, 3000);
            out[0] = address;
            out[1] = 0xFD;
            out[2] = negative ? 1 : 0;
            put16(out + 3, velocity);
            out[5] = uint8_t(std::min<uint32_t>(accel, 255u));  // Emm ramp setting
            put32(out + 6, magnitude);
            out[10] = position_mode;  // relative to the previous target position by default
            out[11] = 0;              // do not defer with the multi-machine sync flag
            return finish(out, 12, checksumMode);
        }

        // X firmware position is in 0.1 degree units.  Carry the sub-unit
        // remainder across streamed commands; otherwise a 5 ms stream of
        // 0.45-unit deltas would round up on every packet and accumulate a
        // large angle error.
        if (capacity < 16) {
            return 0;
        }
        double signedAngle = (negative ? -1.0 : 1.0) * (double(magnitude) * 3600.0 / double(counts_per_rev));
        if (angle_residual) {
            signedAngle += *angle_residual;
        }
        const int64_t angleRounded = llround(signedAngle);
        if (angle_residual) {
            *angle_residual = signedAngle - double(angleRounded);
        }
        const bool     xNegative  = angleRounded < 0;
        const uint64_t xMagnitude = uint64_t(xNegative ? -angleRounded : angleRounded);
        if (xMagnitude > 0xFFFFFFFFull) {
            return 0;
        }
        const uint16_t velocity01 = uint16_t(std::min<uint32_t>(uint32_t(speed_rpm) * 10u, 30000u));
        out[0]  = address;
        out[1]  = 0xFD;
        out[2]  = xNegative ? 1 : 0;
        put16(out + 3, accel);  // X firmware: RPM/s, sent as an absolute value
        put16(out + 5, accel);  // deceleration in RPM/s
        put16(out + 7, velocity01);
        put32(out + 9, uint32_t(xMagnitude));
        out[13] = position_mode;  // relative to the previous target position by default
        out[14] = 0;              // do not defer with the multi-machine sync flag
        return finish(out, 15, checksumMode);
    }

    uint16_t ZdtProtocol::speed_rpm_for_counts(
        int32_t delta_counts, uint32_t counts_per_rev, uint32_t elapsed_ms, uint16_t max_rpm, uint8_t margin_percent) {
        if (delta_counts == 0 || counts_per_rev == 0 || elapsed_ms == 0 || max_rpm == 0) {
            return 0;
        }
        const uint64_t magnitude = delta_counts < 0 ? uint64_t(-int64_t(delta_counts)) : uint64_t(delta_counts);
        const uint64_t denominator = uint64_t(counts_per_rev) * uint64_t(elapsed_ms);
        if (denominator == 0) {
            return 0;
        }
        // Ceil so a streamed delta is never slower than the step stream that
        // produced it.  This is a following limit, not a target speed.  The
        // optional margin restores some catch-up headroom instead of sending
        // exactly the average stream rate.
        const uint64_t numerator = magnitude * 60000ull;
        uint64_t       rpm       = (numerator + denominator - 1) / denominator;
        if (margin_percent) {
            rpm = (rpm * (100ull + margin_percent) + 99ull) / 100ull;
        }
        if (rpm > max_rpm) {
            rpm = max_rpm;
        }
        if (rpm == 0) {
            rpm = 1;  // Emm speed field minimum
        }
        return uint16_t(rpm);
    }

    bool ZdtProtocol::parse_ack(const uint8_t* data, size_t len, uint8_t function) {
        if (!data || len < 3 || data[0] != function) {
            return false;
        }
        // 0x02 = command accepted; 0x9F = motion/zero completed.  Some
        // Response modes (Reached/Both) return 9F for position commands.
        return data[1] == 0x02 || data[1] == 0x9F;
    }

    bool ZdtProtocol::parse_position(ZdtFirmware firmware,
                                     const uint8_t* data,
                                     size_t         len,
                                     uint32_t       counts_per_rev,
                                     int32_t&       counts) {
        if (!data || len < 6 || data[0] != 0x36 || counts_per_rev == 0) {
            return false;
        }
        const bool     negative = data[1] != 0;
        const uint32_t raw      = get32(data + 2);
        int64_t        value    = 0;
        if (firmware == ZdtFirmware::Emm) {
            value = int64_t((uint64_t(raw) * counts_per_rev + 32768u) / 65536u);
        } else {
            value = int64_t((uint64_t(raw) * counts_per_rev + 1800u) / 3600u);
        }
        if (negative) {
            value = -value;
        }
        counts = int32_t(std::max<int64_t>(INT32_MIN, std::min<int64_t>(INT32_MAX, value)));
        return true;
    }
}

#endif
