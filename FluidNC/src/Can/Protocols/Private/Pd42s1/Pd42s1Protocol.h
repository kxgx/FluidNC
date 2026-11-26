// Copyright (c) 2026 - FluidNC contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

/*
    ============================================================================
    METADATA
    ============================================================================
    Vendor:                     ALIENTEK (正点原子)
    Driver family:              PD42S1 integrated closed-loop stepper driver
    Protocol name:              PD42S1 CAN command dialect of the vendor serial
                                control protocol
    Protocol revision:          PD42S1 步进电机闭环驱动器用户手册 V1.2
    Supported firmware model:   PD42S1 factory firmware (version/hardware bytes
                                are queried with function 0x20)
    Hardware variants:          PD42S1 CAN bus version; one fixed command CAN ID
    Transport:                  CAN 2.0 extended frames.  One logical frame is
                                split into as many 8-byte CAN data frames as
                                needed; the response returns the same way.
    Source documents:           PD42S1_步进电机闭环驱动器用户手册_V1.2.pdf
                                (workspace reference directory: 参考/PD42S1资料/, not part of the FluidNC repo)
    Verification status:        CAN framing, command echo, baud migration and
                                position stream exercised on hardware; the
                                vendor manual is the authority for limits.
    Notes:                      The logical frame is the vendor byte stream:
                                  C5 | address | function | data[N] | checksum | 5C
                                Values wider than one byte are big-endian.
                                Checksum = low 8 bits of the sum of all bytes
                                before it.  The command CAN ID is fixed 0x1000;
                                the response carries the command status byte
                                immediately after the function code.
    ============================================================================

    Complete protocol surface used by FluidNC:

      1. Framing and identifiers
      2. Addressing
      3. Function codes and response lengths
      4. Parameter limits and defaults
      5. Status/error values
      6. Timing and buffer sizes
      7. Builders
      8. Parsers

    This header follows FluidNC/src/Can/Protocols/PROTOCOL_TEMPLATE.h.
*/

#pragma once

#include <cstddef>
#include <cstdint>

namespace MotorDrivers {
    class Pd42s1Protocol {
    public:
        // ====================================================================
        // 1. Framing and identifiers
        // ====================================================================
        static constexpr uint8_t FRAME_HEAD = 0xC5;
        static constexpr uint8_t FRAME_TAIL = 0x5C;

        // Fixed 29-bit CAN identifiers used by the vendor examples:
        //   host -> drive: 0x00001000 (command)
        //   drive -> host: 0x00000001 (response)
        // The driver currently correlates responses by the address byte in the
        // payload; this constant documents the wire fact for future filtering.
        static constexpr uint32_t CAN_COMMAND_ID  = 0x1000;
        static constexpr uint32_t CAN_RESPONSE_ID = 0x00000001;

        // The vendor CAN dialect has one fixed command ID (0x1000) and one
        // fixed response ID (0x1).  These are protocol constants, not
        // configurable bus parameters.
        static constexpr size_t MAX_FRAME_LEN = 64;

        // ====================================================================
        // 2. Function codes
        // ====================================================================
        // Grouped by function, not by numeric order.
        static constexpr uint8_t FUNC_READ_VERSION      = 0x20;  // fw/hw version
        static constexpr uint8_t FUNC_READ_POSITION     = 0x2A;
        static constexpr uint8_t FUNC_READ_STATUS       = 0x2C;  // run/state bits
        static constexpr uint8_t FUNC_READ_STALL        = 0x2D;
        static constexpr uint8_t FUNC_READ_ENABLE       = 0x2F;
        static constexpr uint8_t FUNC_READ_IN_POSITION  = 0x30;
        static constexpr uint8_t FUNC_READ_PULSES       = 0x28;
        static constexpr uint8_t FUNC_READ_SPEED        = 0x29;
        static constexpr uint8_t FUNC_READ_POS_ERROR    = 0x2B;
        static constexpr uint8_t FUNC_READ_SYSTEM_PARAMS = 0x31;
        static constexpr uint8_t FUNC_READ_DRIVER_PARAMS = 0x32;
        static constexpr uint8_t FUNC_RESTART           = 0x02;  // reset/reboot
        static constexpr uint8_t FUNC_SAVE_PARAMS       = 0x04;  // NVRAM save
        static constexpr uint8_t FUNC_SET_POSITION_PID  = 0x63;
        static constexpr uint8_t FUNC_SET_MAX_TORQUE    = 0x64;  // mA
        static constexpr uint8_t FUNC_SET_GROUP_ADDRESS = 0x61;
        static constexpr uint8_t FUNC_SET_CAN_BAUD      = 0x68;  // kbps
        static constexpr uint8_t FUNC_SET_ECHO          = 0x6F;
        static constexpr uint8_t FUNC_CLEAR_STALL       = 0xF9;
        static constexpr uint8_t FUNC_SET_MOTOR_ENABLE  = 0xFA;
        static constexpr uint8_t FUNC_CLEAR_STATE       = 0xFB;
        static constexpr uint8_t FUNC_STOP_NOW          = 0xFC;
        static constexpr uint8_t FUNC_ZERO_POSITION     = 0xF8;
        static constexpr uint8_t FUNC_POSITION_MODE     = 0xF2;

        // ====================================================================
        // 3. Response lengths (bytes, including C5/5C framing)
        // ====================================================================
        // A failed command answers with the status byte only.
        static constexpr size_t RESP_LEN_STATUS_ONLY   = 6;
        static constexpr size_t RESP_LEN_BYTE_VALUE    = 7;
        static constexpr size_t RESP_LEN_VERSION       = 8;
        static constexpr size_t RESP_LEN_INT16_VALUE   = 8;
        static constexpr size_t RESP_LEN_INT32_VALUE   = 10;
        static constexpr size_t RESP_LEN_PID           = 18;
        static constexpr size_t RESP_LEN_TORQUE        = 8;
        static constexpr size_t RESP_LEN_CAN_BAUD      = 8;
        static constexpr size_t RESP_LEN_SYSTEM_PARAMS = 46;
        static constexpr size_t RESP_LEN_DRIVER_PARAMS = 50;

        // ====================================================================
        // 4. Parameter limits and defaults
        // ====================================================================
        struct Limits {
            // Address is a single byte in the vendor protocol.  FluidNC itself
            // limits configured CAN clients to CanBus::MAX_CLIENTS.
            static constexpr int32_t ADDRESS_MIN = 1;
            static constexpr int32_t ADDRESS_MAX = 255;
            static constexpr int32_t GROUP_MIN    = 0;
            static constexpr int32_t GROUP_MAX    = 255;

            static constexpr int32_t SPEED_RPM_MIN = 1;
            static constexpr int32_t SPEED_RPM_MAX = 6000;

            // Acceleration is in revolutions/second^2 in the position command.
            static constexpr int32_t ACCEL_MIN = 0;
            static constexpr int32_t ACCEL_MAX = 200;

            // Position command units.  This is fixed by the PD42S1 firmware.
            static constexpr uint32_t POSITION_UNITS_PER_REV = 51200;

            static constexpr int32_t MAX_TORQUE_MA_MIN = 0;
            static constexpr int32_t MAX_TORQUE_MA_MAX = 3000;

            static constexpr int32_t PID_MIN = 0;
            static constexpr int32_t PID_MAX = 1000000;

            static constexpr uint16_t CAN_BAUD_KBPS_MIN = 125;
            static constexpr uint16_t CAN_BAUD_KBPS_MAX = 1000;

            // Bit rates supported by the PD42S1 CAN command 0x68.  The driver
            // probes/migrates only these values; keep the list here so a new
            // rate cannot be added in one place and forgotten in another.
            static constexpr uint16_t SUPPORTED_BAUD_KBPS[] = { 125, 250, 500, 1000 };

            static constexpr int32_t UPDATE_MS_MIN    = 2;
            static constexpr int32_t UPDATE_MS_MAX    = 1000;
            static constexpr int32_t STATUS_POLL_MS_MAX = 60000;
        };

        struct Defaults {
            static constexpr int32_t SPEED_RPM = 600;
            static constexpr int32_t ACCEL     = 200;
            static constexpr int32_t UPDATE_MS = 20;
            static constexpr int32_t STATUS_POLL_MS = 1000;
        };

        // ====================================================================
        // 5. Status / error values
        // ====================================================================
        struct Status {
            static constexpr uint8_t OK = 0x01;
            static constexpr uint8_t ERROR_FRAME_TOO_SHORT = 0xE1;
            static constexpr uint8_t ERROR_BAD_HEADER      = 0xE2;
            static constexpr uint8_t ERROR_BAD_TAIL        = 0xE3;
            static constexpr uint8_t ERROR_CHECKSUM        = 0xE4;
            static constexpr uint8_t ERROR_UNSUPPORTED     = 0xE5;
            static constexpr uint8_t ERROR_INVALID_DATA    = 0xE6;

            // Human-readable name for a vendor status byte.  Returns a stable
            // static string; unknown statuses return "unknown error".
            static const char* error_code_name(uint8_t code);
            static bool        is_error_code(uint8_t code);
        };

        // ====================================================================
        // 6. Timing
        // ====================================================================
        struct Timing {
            static constexpr uint32_t TRANSACTION_TIMEOUT_MS = 300;
            static constexpr uint32_t RETRY_MS               = 500;
            static constexpr uint32_t SAVE_SETTLE_MS         = 300;
        };

        // ====================================================================
        // 7. Builders
        // ====================================================================
        // All builders produce a complete logical frame and return its length,
        // or 0 when out_size is too small.
        static uint8_t checksum(const uint8_t* data, size_t len);
        static size_t  build_frame(uint8_t address, uint8_t function, const uint8_t* data, size_t data_len, uint8_t* out, size_t out_size);

        // Absolute-position command (0xF2).  accel: r/s^2 (0..200);
        // speed_rpm: 0..6000; position: signed driver units (51200/rev).
        static size_t build_position_command(
            uint8_t address, uint8_t accel, uint16_t speed_rpm, uint32_t position, uint8_t* out, size_t out_size);

        static size_t build_enable_command(uint8_t address, bool enable, uint8_t* out, size_t out_size);
        static size_t build_clear_state_command(uint8_t address, uint8_t* out, size_t out_size);
        static size_t build_clear_stall_command(uint8_t address, uint8_t* out, size_t out_size);
        static size_t build_zero_position_command(uint8_t address, uint8_t* out, size_t out_size);
        static size_t build_stop_now_command(uint8_t address, uint8_t* out, size_t out_size);
        static size_t build_set_position_pid_command(
            uint8_t address, uint32_t p, uint32_t i, uint32_t d, uint8_t* out, size_t out_size);
        static size_t build_set_max_torque_command(uint8_t address, uint16_t milliamps, uint8_t* out, size_t out_size);
        static size_t build_set_group_address_command(uint8_t address, uint8_t group, uint8_t* out, size_t out_size);
        static size_t build_set_can_baud_command(uint8_t address, uint16_t kbps, uint8_t* out, size_t out_size);
        static size_t build_save_params_command(uint8_t address, uint8_t* out, size_t out_size);
        static size_t build_read_command(uint8_t address, uint8_t function, uint8_t* out, size_t out_size);

        // ====================================================================
        // 8. Parsers / validation
        // ====================================================================
        // Returns total response length, 0 when more bytes are needed, or
        // SIZE_MAX when the bytes cannot begin a valid response.
        static size_t expected_response_length(const uint8_t* frame, size_t len);
        static bool   valid_response(const uint8_t* frame, size_t len);

        static bool parse_version(const uint8_t* frame, size_t len, uint8_t& firmware_version, uint8_t& hardware_version);
        static bool parse_position(const uint8_t* frame, size_t len, int32_t& position);
        static bool parse_motor_status(const uint8_t* frame, size_t len, uint8_t& status);
        static bool parse_byte_value(const uint8_t* frame, size_t len, uint8_t function, uint8_t& value);
        static bool parse_int32_value(const uint8_t* frame, size_t len, uint8_t function, int32_t& value);
        static bool parse_int16_value(const uint8_t* frame, size_t len, uint8_t function, int16_t& value);
        static bool parse_ack(const uint8_t* frame, size_t len);

        // ====================================================================
        // 9. Safety notes
        // ====================================================================
        // 0xF2/0xFA/0xFB/0xFC/0xF8 cause motion or state changes.
        // 0xFC (stop now) may leave the drive's internal state such that it
        // continues to hold current/torque; clear state explicitly.
        // 0xF9 clears a stall; 0xFB clears the controller state.  They are not
        // interchangeable.
        // 0x61 (set group address) makes the drive respond only to its group
        // address and stop answering its individual address.  Writing it may
        // require a power cycle and re-addressing to recover.
        // 0x04/0x68/0x63/0x64 can write NVRAM or change bus addressing;
        // they must not be used by read-only diagnostics.
    };
}
