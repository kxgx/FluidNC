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
    // Long messages are split into 8-byte frames, packet 0, 1, ...
    //
    // Two firmware families share this framing but use different function
    // payloads:
    //   Emm (default/legacy) - position in pulses, speed in RPM
    //   X   (newer)          - position in 0.1 degree, speed in 0.1 RPM
    // The protocol selection is per motor, because both families can coexist
    // on one bus during migration.
    enum class ZdtFirmware { Emm = 0, X = 1 };

    enum class ZdtChecksum { Fixed6B = 0, Xor = 1, Crc8 = 2 };

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
        static size_t build_relative_position(ZdtFirmware  firmware,
                                              uint8_t       address,
                                              int32_t       delta_counts,
                                              uint16_t      speed_rpm,
                                              uint8_t       accel,
                                              uint32_t      counts_per_rev,
                                              ZdtChecksum   checksum,
                                              uint8_t*      out,
                                              size_t        capacity);

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
