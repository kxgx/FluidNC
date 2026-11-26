// Copyright (c) 2026 - FluidNC contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

/*
    ============================================================================
    FLUIDNC CAN DRIVER PROTOCOL HEADER TEMPLATE
    ============================================================================

    Copy this file to:
        FluidNC/src/Can/Protocols/<Standard|Private>/<Vendor>/<Driver>Protocol.h

    This file is documentation, not a compiled interface.  Keep it in the tree
    as the review checklist for every CAN motor-driver protocol header.

    Rules
    -----
    1. A protocol header is the single source of truth for everything that is
       vendor/protocol specific:
         - framing, identifiers and byte order
         - command/function codes
         - response lengths and status/error codes
         - parameter limits and defaults
         - timing constants and retry limits
         - state/status bit definitions
    2. Driver classes may choose transport, locking and policy, but must not
       repeat raw protocol numbers or limits.  They include this header and use
       its named constants.
    3. Protocol code must remain transport-independent: no TWAI, UART, GPIO,
       RTOS or FluidNC machine state in a protocol header.
    4. Every public builder/parser documents:
         - input range
         - output length / buffer requirement
         - valid response shape
         - failure conditions
    5. Limits belong in a `Limits` namespace/struct.  Defaults belong in
       `Defaults`.  Do not scatter numeric bounds through the driver.
    6. The implementation (.cpp) must contain no unexplained magic numbers.
    7. Keep this metadata block at the top.  Update it when the vendor document
       revision or supported firmware model changes.

    ============================================================================
    METADATA
    ============================================================================

    Vendor:                     <vendor name>
    Driver family:              <driver family / model>
    Protocol name:              <vendor protocol or standard name>
    Protocol revision:          <document revision/date>
    Supported firmware model:   <firmware family/model; state if unknown>
    Hardware variants:          <board/motor variants covered by this header>
    Transport:                  <CAN standard/extended; frame size; baud>
    Source documents:           <manual title + revision + repository path>
    Verification status:        <verified / partially verified / unverified>
    Notes:                      <known deviations, unsupported features>

    ============================================================================
    1. FRAMING AND IDENTIFIERS
    ============================================================================

    // Describe the exact frame layout here, including:
    //   - CAN identifier type and formula
    //   - DLC / payload layout
    //   - byte order and sign convention
    //   - checksum algorithm
    //   - multi-frame packet numbering
    //   - broadcast/group address rules
    //
    // namespace <vendor>::<driver> {
    //     namespace Frame {
    //         constexpr ...;
    //     }
    // }

    ============================================================================
    2. ADDRESSING AND IDENTIFIERS
    ============================================================================

    // namespace <vendor>::<driver>::Limits {
    //     constexpr uint8_t ADDRESS_MIN = ...;
    //     constexpr uint8_t ADDRESS_MAX = ...;
    //     constexpr uint8_t BROADCAST_ADDRESS = ...;
    // }

    ============================================================================
    3. COMMAND / FUNCTION CODES
    ============================================================================

    // Group commands by function, not by numeric order, for example:
    //   - identity/version
    //   - motion
    //   - enable/disable/stop
    //   - position/zero
    //   - configuration
    //   - diagnostics
    //
    // Each entry: code, request length, response length, direction, notes.

    // namespace <vendor>::<driver>::Function {
    //     constexpr uint8_t READ_VERSION = ...;
    // }

    ============================================================================
    4. PARAMETER LIMITS AND DEFAULTS
    ============================================================================

    // Every configurable protocol field must have a named range and default.
    //
    // namespace <vendor>::<driver>::Limits {
    //     constexpr uint32_t SPEED_RPM_MIN = ...;
    //     constexpr uint32_t SPEED_RPM_MAX = ...;
    //     constexpr uint32_t ACCEL_MIN = ...;
    //     constexpr uint32_t ACCEL_MAX = ...;
    // }
    // namespace <vendor>::<driver>::Defaults {
    //     constexpr uint32_t SPEED_RPM = ...;
    // }

    ============================================================================
    5. STATUS, ERRORS AND STATE MACHINE
    ============================================================================

    // Status bytes, fault bits, state machines and error-code names belong here.
    // Keep the numeric protocol value next to its human-readable meaning.

    ============================================================================
    6. TIMING, RETRIES AND BUFFER SIZES
    ============================================================================

    // Protocol-wide constants only:
    //   - transaction timeout
    //   - retry count and retry delay
    //   - maximum frame/response length
    //   - recommended update period
    // Policy-specific values (for example machine motion limits) stay in the
    // driver or Axis config, not in this header.

    // namespace <vendor>::<driver>::Timing { ... }
    // namespace <vendor>::<driver>::Buffer { ... }

    ============================================================================
    7. BUILDERS
    ============================================================================

    // Every builder documents:
    //   - parameter ranges
    //   - exact output length or 0 on invalid input
    //   - whether the caller must hold a bus lock
    //
    // size_t build_xxx(uint8_t address, ..., uint8_t* out, size_t capacity);

    ============================================================================
    8. PARSERS
    ============================================================================

    // Every parser documents:
    //   - exact expected length
    //   - accepted function code and status
    //   - checksum/tail validation
    //   - false on abort/malformed/timeout
    //
    // bool parse_xxx(const uint8_t* data, size_t len, ...);

    ============================================================================
    9. SAFETY NOTES
    ============================================================================

    // Document protocol operations that can cause motion, disable a motor,
    // change NVRAM/address/baud, clear faults, or make the drive unresponsive.
    // These must not be invoked by read-only diagnostics.

    ============================================================================
*/

#pragma once

// Intentionally no declarations.  See the protocol headers for real examples:
//   Private/Zdt/ZdtProtocol.h
//   Private/Pd42s1/Pd42s1Protocol.h
//   Standard/Canopen/CanopenProtocol.h
