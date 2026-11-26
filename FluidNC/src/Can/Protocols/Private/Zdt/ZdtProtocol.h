// Copyright (c) 2026 - FluidNC contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

/*
    ============================================================================
    METADATA
    ============================================================================
    Vendor:                     Zhang Datou (张大头) / ZDT
    Driver family:              XS-series closed-loop stepper (X28S/X35S/X42S/X57S)
    Protocol name:              ZDT free protocol over CAN (private/vendor)
    Protocol revision:          ZDT X42S user manual V1.1.0 (2026-11-10)
    Supported firmware model:   Emm (default, pulse/RPM) and X (angle/0.1 RPM)
    Hardware variants:          X series CAN driver boards; this firmware uses 16
                                configurable CAN clients
    Transport:                  CAN 2.0 extended frames, up to 8 data bytes;
                                baud is configured by can.baud (default 500 kbit)
    Source documents:           ZDT_X42S第二代闭环步进电机用户手册V1.1.0_261110
                                (workspace reference directory: 参考/ZDT资料/zdt_docs/, not part of the FluidNC repo)
    Verification status:        Framing/builders verified against the manual;
                                Emm read-only bus behaviour exercised on hardware;
                                X firmware and motion paths unverified on hardware
    Notes:                      This is a vendor protocol.  It is deliberately
                                isolated in this header; the motor/driver classes
                                must not duplicate its numeric constants.
    ============================================================================

    Complete protocol surface used by FluidNC:

      1. Framing and identifiers (including protocol addressing)
      2. Function codes (identity, motion, enable/stop, config, diagnostics)
      3. Parameter limits and defaults
      4. Status/error values
      5. Position encoding
      6. Timing and buffer sizes
      7. Builders
      8. Parsers
      9. Safety notes

    The CAN layout is:

        extended ID = (address << 8) | packet
        data frame  = function code, payload..., checksum

    Commands longer than one CAN frame are split: every packet repeats the
    function code and carries up to 7 payload bytes; packet numbers start at 0.
    The checksum is the last payload byte (factory default: fixed 0x6B).

    This header follows FluidNC/src/Can/Protocols/PROTOCOL_TEMPLATE.h.
*/

#pragma once

#include "Config.h"

#if MAX_N_CAN

#    include <cstddef>
#    include <cstdint>

namespace MotorDrivers {
    // Two firmware families share the ZDT framing but have different payload
    // semantics.  Keep this enum protocol-local; a motor chooses one family.
    enum class ZdtFirmware { Emm = 0, X = 1 };

    enum class ZdtChecksum { Fixed6B = 0, Xor = 1, Crc8 = 2 };

    // Position command mode (the 0xFD "运动模式" field in the ZDT manual).
    // Only RelativePrevious is currently supported by this driver; the other
    // values are reserved names so a misconfiguration can fail closed instead
    // of silently changing the position contract.
    enum class ZdtPositionMode : uint8_t { RelativePrevious = 0, Absolute = 1, RelativeCurrent = 2 };

    class ZdtProtocol {
    public:
        // ====================================================================
        // 1. Framing and identifiers
        // ====================================================================
        // Protocol address is one byte in the serial protocol.  The FluidNC CAN
        // transport can host at most CanBus::MAX_CLIENTS (16) configured nodes;
        // that machine limit is intentionally not repeated here.
        static constexpr uint8_t ADDRESS_MIN = 1;
        static constexpr uint8_t ADDRESS_MAX = 255;

        // Maximum logical request/response sizes for buffer sizing.  A CAN
        // packet carries at most 8 bytes; these limits cover split commands.
        static constexpr size_t MAX_MESSAGE  = 32;
        static constexpr size_t MAX_RESPONSE = 16;
        static constexpr size_t FRAME_DATA_MAX = 8;  // CAN 2.0 data bytes
        static constexpr size_t FRAME_PAYLOAD  = 7;  // function byte + 7 payload

        // One physical CAN frame produced from a logical ZDT command.  The
        // protocol layer owns packet numbering and repeated function codes; the
        // transport only sends raw frames.
        struct Frame {
            uint32_t id     = 0;
            uint8_t  length = 0;
            uint8_t  data[FRAME_DATA_MAX] = {};
        };

        // ====================================================================
        // 2. Function codes
        // ====================================================================
        struct Function {
            // Identity / configuration reads.
            static constexpr uint8_t READ_VERSION  = 0x1F;
            static constexpr uint8_t READ_CONFIG   = 0x42;
            static constexpr uint8_t READ_STATUS   = 0x43;
            static constexpr uint8_t READ_POSITION = 0x36;

            // Motion.
            static constexpr uint8_t POSITION       = 0xFD;  // streaming position command
            static constexpr uint8_t POSITION_FAST  = 0xFC;  // X firmware fast position
            static constexpr uint8_t ENABLE         = 0xF3;
            static constexpr uint8_t STOP           = 0xFE;
            static constexpr uint8_t ZERO_POSITION  = 0x0A;
            static constexpr uint8_t SYNC           = 0xFF;

            // Configuration writes.
            static constexpr uint8_t WRITE_CONFIG    = 0x48;
            static constexpr uint8_t WRITE_MICROSTEP = 0x84;
            static constexpr uint8_t WRITE_ADDRESS   = 0xAE;
        };

        // Sub-function bytes used by the commands above.
        struct Sub {
            static constexpr uint8_t ENABLE_AB       = 0xAB;
            static constexpr uint8_t STOP_98         = 0x98;
            static constexpr uint8_t ZERO_6D         = 0x6D;
            static constexpr uint8_t SYNC_66         = 0x66;
            static constexpr uint8_t READ_CONFIG_6C  = 0x6C;
            static constexpr uint8_t WRITE_CONFIG_D1 = 0xD1;
            static constexpr uint8_t MICROSTEP_8A    = 0x8A;
            static constexpr uint8_t ADDRESS_4B      = 0x4B;
        };

        // ====================================================================
        // 3. Parameter limits and defaults
        // ====================================================================
        struct Limits {
            // Motion fields in the 0xFD position command.
            static constexpr int32_t SPEED_RPM_MIN = 1;
            static constexpr int32_t SPEED_RPM_MAX = 3000;  // vendor free-protocol limit
            static constexpr int32_t ACCEL_EMM_MIN = 0;
            static constexpr int32_t ACCEL_EMM_MAX = 255;
            static constexpr int32_t ACCEL_X_MIN   = 1;
            static constexpr int32_t ACCEL_X_MAX   = 65535;
            static constexpr int32_t X_SPEED_01RPM_MAX = 30000;  // 0.1 RPM units
            static constexpr int32_t UPDATE_MS_MIN = 2;
            static constexpr int32_t UPDATE_MS_MAX = 1000;
            static constexpr int32_t SPEED_MARGIN_MIN = 0;
            static constexpr int32_t SPEED_MARGIN_MAX = 100;

            // Step resolution / driver configuration.
            static constexpr int32_t COUNTS_PER_REV_MIN  = 1;
            static constexpr int32_t COUNTS_PER_REV_MAX  = 1000000;
            static constexpr int32_t COUNTS_PER_STEP_MIN = 1;
            static constexpr int32_t MICROSTEPS_MIN      = 1;
            static constexpr int32_t MICROSTEPS_MAX      = 255;  // 0 is encoded as 256
            static constexpr int32_t MICROSTEPS_256      = 0;
        };

        struct Defaults {
            // Factory 1.8 degree motor with 16 microsteps: 200 * 16 = 3200.
            static constexpr int32_t COUNTS_PER_REV  = 3200;
            static constexpr int32_t SPEED_RPM       = 600;
            static constexpr int32_t ACCEL_EMM       = 80;
            static constexpr int32_t UPDATE_MS       = 20;
            static constexpr int32_t SPEED_MARGIN_PCT = 25;
        };

        // ====================================================================
        // 4. Status / response values
        // ====================================================================
        struct Status {
            static constexpr uint8_t ACK_OK          = 0x02;
            static constexpr uint8_t ACK_REACHED     = 0x9F;
            static constexpr uint8_t ACK_HOMING_FAIL = 0x9E;
            static constexpr uint8_t ACK_ERROR       = 0xE2;
            static constexpr uint8_t ACK_BAD_FORMAT  = 0xEE;
            static constexpr uint8_t CHECKSUM_FIXED  = 0x6B;
        };

        // ====================================================================
        // 5. Position encoding
        // ====================================================================
        // The 0x36/0xFD position fields are fixed-point.  Emm uses pulses with
        // 65536 units per revolution; X uses 0.1 degree with 3600 units/rev.
        struct Encoding {
            static constexpr uint32_t EMM_POSITION_UNITS_PER_REV = 65536;
            static constexpr uint32_t X_POSITION_UNITS_PER_REV   = 3600;
            static constexpr uint32_t EMM_ROUND                  = 32768;
            static constexpr uint32_t X_ROUND                    = 1800;
        };

        // ====================================================================
        // 6. Timing and buffer sizes
        // ====================================================================
        struct Timing {
            static constexpr uint32_t TRANSACTION_TIMEOUT_MS = 300;
            static constexpr uint32_t DEFAULT_UPDATE_MS      = 20;
        };

        // ====================================================================
        // 7. Builders
        // ====================================================================
        // All builders write a complete logical command including the checksum
        // and return its length, or 0 when the output buffer is too small.

        static size_t build_read_position(uint8_t address, ZdtChecksum checksum, uint8_t* out, size_t capacity);

        // Split one logical command (message[0] is the function code, followed
        // by payload and checksum) into physical frames.  Returns the number
        // of frames written, or 0 when the input/capacity is invalid.
        static size_t build_frames(uint8_t address,
                                   const uint8_t* message,
                                   size_t length,
                                   Frame* out,
                                   size_t capacity);
        static size_t build_read_version(uint8_t address, ZdtChecksum checksum, uint8_t* out, size_t capacity);
        static size_t build_enable(uint8_t address, bool enable, ZdtChecksum checksum, uint8_t* out, size_t capacity);
        static size_t build_stop(uint8_t address, ZdtChecksum checksum, uint8_t* out, size_t capacity);
        static size_t build_zero(uint8_t address, ZdtChecksum checksum, uint8_t* out, size_t capacity);
        static size_t build_sync(uint8_t address, ZdtChecksum checksum, uint8_t* out, size_t capacity);

        // Emit a relative move from the previously commanded target.  The
        // caller keeps its own target accumulator; this function converts a
        // signed pulse delta to the selected firmware's position units.
        // accel follows the selected firmware's native unit: Emm uses the
        // 0..255 ramp setting; X uses RPM/s directly (0..65535).
        static size_t build_relative_position(ZdtFirmware  firmware,
                                              uint8_t       address,
                                              int32_t       delta_counts,
                                              uint16_t      speed_rpm,
                                              uint16_t      accel,
                                              uint32_t      counts_per_rev,
                                              ZdtChecksum   checksum,
                                              uint8_t*      out,
                                              size_t        capacity,
                                              double*       angle_residual = nullptr,
                                              uint8_t       position_mode = 0);

        // Map an actually executed step delta to the ZDT speed field for
        // position-stream speed following.  elapsed_ms is the time since the
        // previous successfully transmitted position command; the result is
        // clamped to 1..max_rpm (Emm's speed field has 1 RPM resolution).
        static uint16_t speed_rpm_for_counts(int32_t  delta_counts,
                                             uint32_t counts_per_rev,
                                             uint32_t elapsed_ms,
                                             uint16_t max_rpm,
                                             uint8_t  margin_percent = 0);

        // ====================================================================
        // 8. Parsers
        // ====================================================================
        static bool parse_ack(const uint8_t* data, size_t len, uint8_t function);

        struct VersionInfo {
            uint16_t firmware        = 0;
            uint8_t  hardwareSeries  = 0;
            uint8_t  hardwareType    = 0;
            uint8_t  hardwareVersion = 0;
        };

        // Decode the 0x1F response (CAN data begins with the function code).
        // firmware=0x00CF is V2.0.7, hardware type 1/3/... is 28/42/...
        static bool parse_version(const uint8_t* data, size_t len, VersionInfo& out);

        static bool parse_position(ZdtFirmware    firmware,
                                   const uint8_t* data,
                                   size_t         len,
                                   uint32_t       counts_per_rev,
                                   int32_t&       counts);

        static uint8_t checksum(const uint8_t* data, size_t len, ZdtChecksum mode);

        // ====================================================================
        // 9. Safety notes
        // ====================================================================
        // 0xF3/0xFE/0x0A/0xFD can enable, stop, zero or move the motor.
        // 0x48/0x84/0xAE can alter NVRAM (address, microsteps, protocol
        // settings) and may make a drive unresponsive until power-cycled.
        // Read-only diagnostics must use 0x1F/0x36/0x42 only.
    };
}

#endif
