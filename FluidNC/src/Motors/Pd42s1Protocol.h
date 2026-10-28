// Copyright (c) 2026 - FluidNC contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

/*
    Framing and command helpers for the ALIENTEK (正点原子) PD42S1 integrated
    closed-loop stepper driver, in the "CAN" dialect of its serial protocol.

    The CAN transport carries the same 0xC5 ... 0x5C byte stream that the UART
    transport uses: the host splits one logical frame into as many 8-byte CAN
    data frames as necessary, and the response comes back the same way.  A
    frame is

        0xC5 | address | function | data[N] | checksum | 0x5C

    where checksum is the low 8 bits of the sum of all preceding bytes.  Values
    wider than one byte are big-endian.  The response has the same shape with
    the command status byte (0x01 = success) immediately after the function
    code.

    Because the response length is fully determined by the function code and
    the status byte, this class can tell the caller how many bytes to expect,
    so the receiver does not have to rely on inter-frame timing to find the
    end of a response.
*/

#pragma once

#include <cstddef>
#include <cstdint>

namespace MotorDrivers {
    class Pd42s1Protocol {
    public:
        static constexpr uint8_t FRAME_HEAD = 0xC5;
        static constexpr uint8_t FRAME_TAIL = 0x5C;

        // Command status byte in a response frame.
        static constexpr uint8_t STATUS_OK = 0x01;

        // Function codes used by this driver.  The full list is in the
        // vendor's control protocol document; these are the ones the CAN
        // motor driver needs.
        static constexpr uint8_t FUNC_READ_VERSION     = 0x20;
        static constexpr uint8_t FUNC_READ_POSITION    = 0x2A;
        static constexpr uint8_t FUNC_READ_STATUS      = 0x2C;
        static constexpr uint8_t FUNC_RESTART          = 0x02;  // reset/reboot the driver
        static constexpr uint8_t FUNC_SET_ECHO         = 0x6F;
        static constexpr uint8_t FUNC_CLEAR_STALL      = 0xF9;  // clear stall protection
        static constexpr uint8_t FUNC_SET_MOTOR_ENABLE = 0xFA;
        static constexpr uint8_t FUNC_CLEAR_STATE      = 0xFB;  // clear stall/brake/disabled state
        static constexpr uint8_t FUNC_STOP_NOW         = 0xFC;  // brake immediately
        static constexpr uint8_t FUNC_POSITION_MODE    = 0xF2;

        static constexpr size_t MAX_FRAME_LEN = 16;

        // Low 8 bits of the sum of data[0..len-1].
        static uint8_t checksum(const uint8_t* data, size_t len);

        // Builds one complete frame.  Returns the number of bytes written, or
        // 0 if out is too small.
        static size_t build_frame(uint8_t address, uint8_t function, const uint8_t* data, size_t data_len, uint8_t* out, size_t out_size);

        // Builds an absolute-position command (0xF2).  accel is in r/s^2
        // (0..200), speed_rpm is in RPM (0..6000), position is in driver
        // position units (51200 units per revolution).
        static size_t build_position_command(
            uint8_t address, uint8_t accel, uint16_t speed_rpm, uint32_t position, uint8_t* out, size_t out_size);

        // Builds the motor enable/disable command (0xFA).
        static size_t build_enable_command(uint8_t address, bool enable, uint8_t* out, size_t out_size);

        // Builds the "clear abnormal state" command (0xFB): clears stall,
        // brake and disabled state.  Returns 6-byte acknowledge.
        static size_t build_clear_state_command(uint8_t address, uint8_t* out, size_t out_size);

        // Builds the stall-protection release command (0xF9), for the case
        // where only the stall latch needs clearing.
        static size_t build_clear_stall_command(uint8_t address, uint8_t* out, size_t out_size);

        // Builds the immediate-brake command (0xFC).  The manual warns that
        // the motor can get hot if the state is not cleared afterwards.
        static size_t build_stop_now_command(uint8_t address, uint8_t* out, size_t out_size);

        // Builds a command with no data payload, e.g. a read command.
        static size_t build_read_command(uint8_t address, uint8_t function, uint8_t* out, size_t out_size);

        // Returns the total length of the response frame that starts at
        // frame[0].  Returns 0 when more bytes are needed to decide, and
        // SIZE_MAX when the bytes cannot be the start of a valid response.
        static size_t expected_response_length(const uint8_t* frame, size_t len);

        // True when a complete response frame with a valid header, checksum
        // and tail is present in frame[0..len-1].
        static bool valid_response(const uint8_t* frame, size_t len);

        // Response parsers.  They return false when the frame is not a
        // successful response of the requested type.
        static bool parse_version(const uint8_t* frame, size_t len, uint8_t& firmware_version, uint8_t& hardware_version);
        static bool parse_position(const uint8_t* frame, size_t len, int32_t& position);
        static bool parse_motor_status(const uint8_t* frame, size_t len, uint8_t& status);
        static bool parse_ack(const uint8_t* frame, size_t len);
    };
}
