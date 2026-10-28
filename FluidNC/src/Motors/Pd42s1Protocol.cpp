// Copyright (c) 2026 - FluidNC contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "Pd42s1Protocol.h"

namespace MotorDrivers {
    uint8_t Pd42s1Protocol::checksum(const uint8_t* data, size_t len) {
        uint8_t sum = 0;
        for (size_t i = 0; i < len; ++i) {
            sum += data[i];
        }
        return sum;
    }

    size_t Pd42s1Protocol::build_frame(uint8_t address, uint8_t function, const uint8_t* data, size_t data_len, uint8_t* out, size_t out_size) {
        size_t total = data_len + 5;  // head, address, function, checksum, tail
        if (out_size < total) {
            return 0;
        }

        out[0] = FRAME_HEAD;
        out[1] = address;
        out[2] = function;
        if (data_len) {
            for (size_t i = 0; i < data_len; ++i) {
                out[3 + i] = data[i];
            }
        }
        out[3 + data_len] = checksum(out, 3 + data_len);
        out[4 + data_len] = FRAME_TAIL;
        return total;
    }

    size_t Pd42s1Protocol::build_position_command(
        uint8_t address, uint8_t accel, uint16_t speed_rpm, uint32_t position, uint8_t* out, size_t out_size) {
        uint8_t data[8];
        // Byte 1 is the rotation direction in the PD42S1 protocol
        // (0 = forward, 1 = reverse).  It is currently fixed to 0: the
        // driver's behaviour for negative absolute targets is not documented,
        // so direction handling is pending a hardware check (see the CAN code
        // review).
        data[0] = 0;
        data[1] = accel;
        data[2] = uint8_t(speed_rpm >> 8);
        data[3] = uint8_t(speed_rpm & 0xFF);
        data[4] = uint8_t(position >> 24);
        data[5] = uint8_t(position >> 16);
        data[6] = uint8_t(position >> 8);
        data[7] = uint8_t(position & 0xFF);
        return build_frame(address, FUNC_POSITION_MODE, data, sizeof(data), out, out_size);
    }

    size_t Pd42s1Protocol::build_enable_command(uint8_t address, bool enable, uint8_t* out, size_t out_size) {
        // 0 enables the motor, 1 disables it.
        uint8_t data[1] = { uint8_t(enable ? 0 : 1) };
        return build_frame(address, FUNC_SET_MOTOR_ENABLE, data, sizeof(data), out, out_size);
    }

    size_t Pd42s1Protocol::build_read_command(uint8_t address, uint8_t function, uint8_t* out, size_t out_size) {
        return build_frame(address, function, nullptr, 0, out, out_size);
    }

    size_t Pd42s1Protocol::expected_response_length(const uint8_t* frame, size_t len) {
        if (len < 1) {
            return 0;
        }
        if (frame[0] != FRAME_HEAD) {
            return SIZE_MAX;
        }
        if (len < 4) {
            return 0;
        }

        uint8_t function = frame[2];
        uint8_t status   = frame[3];
        if (status != STATUS_OK) {
            // Every failed command answers with just the status byte.
            return 6;
        }

        switch (function) {
            case FUNC_READ_VERSION:
                return 8;  // status + firmware version + hardware version
            case FUNC_READ_POSITION:
                return 10;  // status + 4-byte position
            case FUNC_READ_STATUS:
                return 7;  // status + 1-byte motor status
            case FUNC_SET_MOTOR_ENABLE:
                return 7;  // status + 1-byte enable state
            case FUNC_SET_ECHO:
                return 7;  // status + 1-byte echo state
            case FUNC_POSITION_MODE:
                return 6;  // status only
            default:
                // The length of an unmodelled response is unknown; the caller
                // must fall back to a timeout.
                return 0;
        }
    }

    bool Pd42s1Protocol::valid_response(const uint8_t* frame, size_t len) {
        if (len < 6 || len > MAX_FRAME_LEN) {
            return false;
        }
        if (frame[0] != FRAME_HEAD || frame[len - 1] != FRAME_TAIL) {
            return false;
        }
        return checksum(frame, len - 2) == frame[len - 2];
    }

    bool Pd42s1Protocol::parse_version(const uint8_t* frame, size_t len, uint8_t& firmware_version, uint8_t& hardware_version) {
        if (len != 8 || frame[2] != FUNC_READ_VERSION || !valid_response(frame, len)) {
            return false;
        }
        firmware_version = frame[4];
        hardware_version = frame[5];
        return true;
    }

    bool Pd42s1Protocol::parse_position(const uint8_t* frame, size_t len, int32_t& position) {
        if (len != 10 || frame[2] != FUNC_READ_POSITION || !valid_response(frame, len)) {
            return false;
        }
        position = int32_t((uint32_t(frame[4]) << 24) | (uint32_t(frame[5]) << 16) | (uint32_t(frame[6]) << 8) | uint32_t(frame[7]));
        return true;
    }

    bool Pd42s1Protocol::parse_motor_status(const uint8_t* frame, size_t len, uint8_t& status) {
        if (len != 7 || frame[2] != FUNC_READ_STATUS || !valid_response(frame, len)) {
            return false;
        }
        status = frame[4];
        return true;
    }

    bool Pd42s1Protocol::parse_ack(const uint8_t* frame, size_t len) {
        return valid_response(frame, len) && frame[3] == STATUS_OK;
    }
}
