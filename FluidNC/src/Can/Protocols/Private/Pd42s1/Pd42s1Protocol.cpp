// Copyright (c) 2026 - FluidNC contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "Pd42s1Protocol.h"

namespace MotorDrivers {
    const char* Pd42s1Protocol::Status::error_code_name(uint8_t code) {
        switch (code) {
            case ERROR_FRAME_TOO_SHORT:
                return "frame too short";
            case ERROR_BAD_HEADER:
                return "bad frame header";
            case ERROR_BAD_TAIL:
                return "bad frame tail";
            case ERROR_CHECKSUM:
                return "checksum error";
            case ERROR_UNSUPPORTED:
                return "unsupported function";
            case ERROR_INVALID_DATA:
                return "invalid data";
            default:
                return "unknown error";
        }
    }

    bool Pd42s1Protocol::Status::is_error_code(uint8_t code) {
        return code >= ERROR_FRAME_TOO_SHORT && code <= ERROR_INVALID_DATA;
    }

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
        // The protocol keeps direction in byte 1 and an unsigned position
        // magnitude in bytes 5..8.  It is not a two's-complement signed
        // coordinate: the vendor examples only send positive magnitudes.
        // Encode negative targets as reverse + magnitude.
        const int32_t  signedPosition = int32_t(position);
        const bool     reverse        = signedPosition < 0;
        const uint32_t magnitude      = reverse ? uint32_t(-int64_t(signedPosition)) : uint32_t(signedPosition);
        uint8_t data[8];
        data[0] = reverse ? 1 : 0;
        data[1] = accel;
        data[2] = uint8_t(speed_rpm >> 8);
        data[3] = uint8_t(speed_rpm & 0xFF);
        data[4] = uint8_t(magnitude >> 24);
        data[5] = uint8_t(magnitude >> 16);
        data[6] = uint8_t(magnitude >> 8);
        data[7] = uint8_t(magnitude & 0xFF);
        return build_frame(address, FUNC_POSITION_MODE, data, sizeof(data), out, out_size);
    }

    size_t Pd42s1Protocol::build_enable_command(uint8_t address, bool enable, uint8_t* out, size_t out_size) {
        // 0 enables the motor, 1 disables it.
        uint8_t data[1] = { uint8_t(enable ? 0 : 1) };
        return build_frame(address, FUNC_SET_MOTOR_ENABLE, data, sizeof(data), out, out_size);
    }

    size_t Pd42s1Protocol::build_clear_state_command(uint8_t address, uint8_t* out, size_t out_size) {
        return build_frame(address, FUNC_CLEAR_STATE, nullptr, 0, out, out_size);
    }

    size_t Pd42s1Protocol::build_clear_stall_command(uint8_t address, uint8_t* out, size_t out_size) {
        return build_frame(address, FUNC_CLEAR_STALL, nullptr, 0, out, out_size);
    }

    size_t Pd42s1Protocol::build_zero_position_command(uint8_t address, uint8_t* out, size_t out_size) {
        return build_frame(address, FUNC_ZERO_POSITION, nullptr, 0, out, out_size);
    }

    size_t Pd42s1Protocol::build_stop_now_command(uint8_t address, uint8_t* out, size_t out_size) {
        return build_frame(address, FUNC_STOP_NOW, nullptr, 0, out, out_size);
    }

    size_t Pd42s1Protocol::build_set_position_pid_command(
        uint8_t address, uint32_t p, uint32_t i, uint32_t d, uint8_t* out, size_t out_size) {
        uint8_t data[12] = {
            uint8_t(p >> 24), uint8_t(p >> 16), uint8_t(p >> 8), uint8_t(p),
            uint8_t(i >> 24), uint8_t(i >> 16), uint8_t(i >> 8), uint8_t(i),
            uint8_t(d >> 24), uint8_t(d >> 16), uint8_t(d >> 8), uint8_t(d),
        };
        return build_frame(address, FUNC_SET_POSITION_PID, data, sizeof(data), out, out_size);
    }

    size_t Pd42s1Protocol::build_set_max_torque_command(uint8_t address, uint16_t milliamps, uint8_t* out, size_t out_size) {
        uint8_t data[2] = { uint8_t(milliamps >> 8), uint8_t(milliamps & 0xFF) };
        return build_frame(address, FUNC_SET_MAX_TORQUE, data, sizeof(data), out, out_size);
    }

    size_t Pd42s1Protocol::build_set_group_address_command(uint8_t address, uint8_t group, uint8_t* out, size_t out_size) {
        uint8_t data[1] = { group };
        return build_frame(address, FUNC_SET_GROUP_ADDRESS, data, sizeof(data), out, out_size);
    }

    size_t Pd42s1Protocol::build_set_can_baud_command(uint8_t address, uint16_t kbps, uint8_t* out, size_t out_size) {
        uint8_t data[2] = { uint8_t(kbps >> 8), uint8_t(kbps & 0xFF) };
        return build_frame(address, FUNC_SET_CAN_BAUD, data, sizeof(data), out, out_size);
    }

    size_t Pd42s1Protocol::build_save_params_command(uint8_t address, uint8_t* out, size_t out_size) {
        return build_frame(address, FUNC_SAVE_PARAMS, nullptr, 0, out, out_size);
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
        if (status != Pd42s1Protocol::Status::OK) {
            // Every failed command answers with just the status byte.
            return RESP_LEN_STATUS_ONLY;
        }

        switch (function) {
            case FUNC_READ_VERSION:
                return RESP_LEN_VERSION;
            case FUNC_READ_POSITION:
            case FUNC_READ_PULSES:
            case FUNC_READ_POS_ERROR:
                return RESP_LEN_INT32_VALUE;
            case FUNC_READ_SPEED:
                return RESP_LEN_INT16_VALUE;
            case FUNC_READ_STATUS:
                return RESP_LEN_BYTE_VALUE;
            case FUNC_READ_STALL:
            case FUNC_READ_ENABLE:
            case FUNC_READ_IN_POSITION:
                return RESP_LEN_BYTE_VALUE;
            case FUNC_SET_MOTOR_ENABLE:
                return RESP_LEN_BYTE_VALUE;
            case FUNC_SET_ECHO:
                return RESP_LEN_BYTE_VALUE;
            case FUNC_SET_GROUP_ADDRESS:
                return RESP_LEN_BYTE_VALUE;
            case FUNC_CLEAR_STALL:
            case FUNC_CLEAR_STATE:
            case FUNC_STOP_NOW:
            case FUNC_ZERO_POSITION:
            case FUNC_SAVE_PARAMS:
                return RESP_LEN_STATUS_ONLY;
            case FUNC_SET_CAN_BAUD:
                return RESP_LEN_CAN_BAUD;
            case FUNC_READ_SYSTEM_PARAMS:
                return RESP_LEN_SYSTEM_PARAMS;
            case FUNC_READ_DRIVER_PARAMS:
                return RESP_LEN_DRIVER_PARAMS;
            case FUNC_SET_POSITION_PID:
                return RESP_LEN_PID;
            case FUNC_SET_MAX_TORQUE:
                return RESP_LEN_TORQUE;
            case FUNC_POSITION_MODE:
                return RESP_LEN_STATUS_ONLY;
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
        if (len != RESP_LEN_VERSION || frame[2] != FUNC_READ_VERSION || !valid_response(frame, len)) {
            return false;
        }
        firmware_version = frame[4];
        hardware_version = frame[5];
        return true;
    }

    bool Pd42s1Protocol::parse_position(const uint8_t* frame, size_t len, int32_t& position) {
        return parse_int32_value(frame, len, FUNC_READ_POSITION, position);
    }

    bool Pd42s1Protocol::parse_int32_value(const uint8_t* frame, size_t len, uint8_t function, int32_t& value) {
        if (len != RESP_LEN_INT32_VALUE || frame[2] != function || !valid_response(frame, len)) {
            return false;
        }
        value = int32_t((uint32_t(frame[4]) << 24) | (uint32_t(frame[5]) << 16) | (uint32_t(frame[6]) << 8) | uint32_t(frame[7]));
        return true;
    }

    bool Pd42s1Protocol::parse_int16_value(const uint8_t* frame, size_t len, uint8_t function, int16_t& value) {
        if (len != RESP_LEN_INT16_VALUE || frame[2] != function || !valid_response(frame, len)) {
            return false;
        }
        value = int16_t((uint16_t(frame[4]) << 8) | uint16_t(frame[5]));
        return true;
    }

    bool Pd42s1Protocol::parse_motor_status(const uint8_t* frame, size_t len, uint8_t& status) {
        return parse_byte_value(frame, len, FUNC_READ_STATUS, status);
    }

    bool Pd42s1Protocol::parse_byte_value(const uint8_t* frame, size_t len, uint8_t function, uint8_t& value) {
        if (len != RESP_LEN_BYTE_VALUE || frame[2] != function || !valid_response(frame, len)) {
            return false;
        }
        value = frame[4];
        return true;
    }

    bool Pd42s1Protocol::parse_ack(const uint8_t* frame, size_t len) {
        return valid_response(frame, len) && frame[3] == Pd42s1Protocol::Status::OK;
    }
}
