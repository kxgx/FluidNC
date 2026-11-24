// Copyright (c) 2026 - FluidNC contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include "Config.h"

#if MAX_N_CAN

#    include <cstddef>
#    include <cstdint>

namespace MotorDrivers {
    // Zhang Datou (ZDT) XS-series closed-loop stepper, CAN transport.
    //
    // The CAN layer is the vendor's free protocol:
    //   extended ID = (address << 8) | packet
    //   data        = function code, payload..., checksum
    // Long messages are split into CAN frames; every frame repeats the function
    // code and carries up to 7 payload bytes, with packet 0, 1, ... in the ID.
    //
    // Two firmware families share this framing but use different function
    // payloads:
    //   Emm (default/legacy) - position in pulses, speed in RPM
    //   X   (newer)          - position in 0.1 degree, speed in 0.1 RPM
    // The protocol selection is per motor, because both families can coexist
    // on one bus during migration.
    enum class ZdtFirmware { Emm = 0, X = 1 };

    enum class ZdtChecksum { Fixed6B = 0, Xor = 1, Crc8 = 2 };

    // Position command mode (the 0xFD "运动模式" field in the ZDT manual).
    // Only RelativePrevious is currently supported by the driver; the other
    // values are reserved names so a misconfiguration can fail closed instead
    // of silently changing the position contract.
    enum class ZdtPositionMode : uint8_t { RelativePrevious = 0, Absolute = 1, RelativeCurrent = 2 };

    class ZdtProtocol {
    public:
        static constexpr size_t MAX_MESSAGE = 32;

        static size_t build_read_position(uint8_t address, ZdtChecksum checksum, uint8_t* out, size_t capacity);
        static size_t build_enable(uint8_t address, bool enable, ZdtChecksum checksum, uint8_t* out, size_t capacity);
        static size_t build_stop(uint8_t address, ZdtChecksum checksum, uint8_t* out, size_t capacity);
        static size_t build_zero(uint8_t address, ZdtChecksum checksum, uint8_t* out, size_t capacity);
        static size_t build_sync(uint8_t address, ZdtChecksum checksum, uint8_t* out, size_t capacity);

        // Emit a relative move from the previously commanded target.  The
        // caller keeps its own target accumulator; this function converts a
        // signed pulse delta to the selected firmware's position units.
        // accel follows the selected firmware's native unit: Emm uses the
        // 0..255 ramp setting, X uses RPM/s directly (0..65535).
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

        static bool parse_ack(const uint8_t* data, size_t len, uint8_t function);
        static bool parse_position(ZdtFirmware    firmware,
                                   const uint8_t* data,
                                   size_t         len,
                                   uint32_t       counts_per_rev,
                                   int32_t&       counts);

        static uint8_t checksum(const uint8_t* data, size_t len, ZdtChecksum mode);
    };
}

#endif
