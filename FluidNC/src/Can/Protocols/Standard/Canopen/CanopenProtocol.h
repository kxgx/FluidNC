// Copyright (c) 2026 - FluidNC contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

/*
    ============================================================================
    METADATA
    ============================================================================
    Vendor:                     CAN in Automation (CiA), vendor independent
    Driver family:              CANopen motor driver (CiA301 communication +
                                CiA402 drive profile)
    Protocol name:              CANopen / CiA301 / CiA402
    Protocol revision:          CiA301 and CiA402 standard defaults
    Supported firmware model:   Any drive implementing CiA301/CiA402; a vendor
                                that deviates overrides NodeConfig indices/COB-IDs
    Hardware variants:          Standard 11-bit CANopen nodes, node ids 1..127
    Transport:                  CAN 2.0 standard frames, 8-byte payload
    Source documents:           CiA301 / CiA402 specifications; vendor manuals
                                are authoritative for any non-standard object
    Verification status:        Protocol layer follows the standard; no vendor
                                object dictionary has been verified on hardware.
    Notes:                      This is a standard protocol, not a vendor
                                private protocol.  Keep standard identifiers and
                                CiA402 bits here, not in the motor/driver class.
    ============================================================================

    Standard CANopen (CiA301) transport plus the CiA402 drive-profile subset
    used for CAN motor drivers.

    This is deliberately a standard, vendor-independent protocol: the default
    object dictionary entries and COB-IDs are the CiA301/CiA402 ones, so a drive
    from any vendor that really implements CANopen works without a vendor class.
    A vendor whose object dictionary deviates can override the node id, the SDO
    COB-IDs and the CiA402 object indices through Canopen::NodeConfig (the
    canopen motor driver exposes those as configuration items).

    Implemented here:

      - NMT start/stop/pre-operational/reset-node commands (CiA301 COB-ID 0x000)
      - SDO expedited download (client -> server) and upload (server -> client),
        including the abort transfer (0x80) and a readable abort-code name
      - Heartbeat (CiA301 boot-up / stopped / operational / pre-operational
        state byte from 0x700 + node id)
      - PDO1/PDO2 basic encode/decode.  The defaults assume the conventional
        mapping PDO1 = controlword/statusword and PDO2 = target/actual position.
        PDOs are NOT used for the default control path because the required PDO
        mapping (0x1400/0x1600/0x1800/0x1A00) is vendor/drive specific; the
        blocking SDO path does not depend on it.
      - CiA402 drive state machine helpers: controlword commands, statusword
        state decoding, and the shutdown -> switch on -> enable operation
        sequence, quick stop and fault reset.

    IMPORTANT: no CANopen object dictionary has been verified against the
    Zhang Datou (张大头) X42S V2.0 / X28S V2.0 manuals.  Those manuals do not
    contain a CANopen tutorial or object dictionary, so this implementation
    assumes the standard CiA402 defaults and must be checked on real hardware
    (see CanopenMotor.md).
*/

#pragma once

#include "Config.h"

#if MAX_N_CAN

#    include <cstddef>
#    include <cstdint>

namespace Machine {
    class CanBus;

    namespace Canopen {
        // ------------------------------------------------------------------
        // CiA301 default identifiers.  Node ids are 1..127; node id 0 means
        // broadcast in NMT.  CANopen uses 11-bit standard frames throughout.
        // ------------------------------------------------------------------
        constexpr uint32_t NMT_COBID       = 0x000;
        constexpr uint32_t SYNC_COBID      = 0x080;
        constexpr uint32_t EMCY_COBID_BASE = 0x080;  // 0x080 + node id
        constexpr uint32_t TIME_COBID      = 0x100;
        constexpr uint32_t TPDO1_COBID_BASE = 0x180;
        constexpr uint32_t RPDO1_COBID_BASE = 0x200;
        constexpr uint32_t TPDO2_COBID_BASE = 0x280;
        constexpr uint32_t RPDO2_COBID_BASE = 0x300;
        constexpr uint32_t SDO_RESPONSE_COBID_BASE = 0x580;  // server -> client
        constexpr uint32_t SDO_REQUEST_COBID_BASE  = 0x600;  // client -> server
        constexpr uint32_t HEARTBEAT_COBID_BASE    = 0x700;  // 0x700 + node id

        constexpr uint8_t MIN_NODE_ID   = 1;
        constexpr uint8_t MAX_NODE_ID   = 127;
        constexpr uint8_t BROADCAST_ID  = 0;
        constexpr size_t  MAX_FRAME_DATA = 8;

        // Protocol-level limits and default timings.  These are standard
        // CANopen properties; node/vendor overrides belong in NodeConfig.
        namespace Limits {
            constexpr size_t   MAX_EXPEDITED_SDO_DATA = 4;
            constexpr size_t   MAX_PDO_DATA           = 8;
            constexpr size_t   MAX_REGISTERED_NODES   = 16;  // machine registry cap; CanopenClient asserts bus equality
            constexpr uint32_t DEFAULT_SDO_TIMEOUT_MS = 150;
            constexpr uint32_t DEFAULT_HEARTBEAT_TIMEOUT_MS = 1000;
            constexpr uint32_t NMT_TIMEOUT_MS         = 50;
        }

        // ------------------------------------------------------------------
        // NMT (network management).
        // ------------------------------------------------------------------
        enum class NmtCommand : uint8_t {
            Start              = 0x01,
            Stop               = 0x02,
            PreOperational     = 0x80,
            ResetNode          = 0x81,
            ResetCommunication = 0x82,
        };

        enum class NmtState : uint8_t {
            BootUp         = 0x00,
            Stopped        = 0x04,
            Operational    = 0x05,
            PreOperational = 0x7F,
            Unknown        = 0xFF,
        };

        const char* nmt_command_name(NmtCommand command);
        const char* nmt_state_name(NmtState state);

        // ------------------------------------------------------------------
        // Standard object dictionary entries.  Only the CiA301 device info and
        // the CiA402 drive profile objects used by the motor/driver path are
        // listed; a vendor-specific dictionary is supported by overriding these
        // indices in NodeConfig.
        // ------------------------------------------------------------------
        namespace Od {
            constexpr uint16_t DEVICE_TYPE              = 0x1000;
            constexpr uint16_t ERROR_REGISTER           = 0x1001;
            constexpr uint16_t MANUFACTURER_STATUS      = 0x1002;
            constexpr uint16_t STORE_PARAMETERS         = 0x1010;
            constexpr uint16_t RESTORE_DEFAULT          = 0x1011;
            constexpr uint16_t PRODUCER_HEARTBEAT_TIME  = 0x1017;
            constexpr uint16_t CONTROLWORD              = 0x6040;
            constexpr uint16_t STATUSWORD               = 0x6041;
            constexpr uint16_t MODES_OF_OPERATION       = 0x6060;
            constexpr uint16_t MODES_OF_OPERATION_DISPLAY = 0x6061;
            constexpr uint16_t POSITION_ACTUAL          = 0x6064;
            constexpr uint16_t TARGET_POSITION          = 0x607A;
            constexpr uint16_t PROFILE_VELOCITY         = 0x6081;
            constexpr uint16_t TARGET_VELOCITY          = 0x60FF;
            constexpr uint16_t SUPPORTED_DRIVE_MODES    = 0x6502;
        }

        // CiA402 modes of operation (0x6060 / 0x6061 values).
        namespace Mode {
            constexpr int8_t PROFILE_POSITION        = 1;
            constexpr int8_t VELOCITY                = 2;
            constexpr int8_t PROFILE_VELOCITY        = 3;
            constexpr int8_t HOMING                  = 6;
            constexpr int8_t CYCLIC_SYNC_POSITION    = 8;
            constexpr int8_t CYCLIC_SYNC_VELOCITY    = 9;
            constexpr int8_t CYCLIC_SYNC_TORQUE      = 10;
        }

        // Configuration of one remote node.  All COB-ID overrides use 0 for
        // "use the standard default COB-ID for this node id".
        struct NodeConfig {
            uint8_t node_id = 1;

            uint32_t sdo_request_cobid  = 0;  // 0 => 0x600 + node_id
            uint32_t sdo_response_cobid = 0;  // 0 => 0x580 + node_id
            uint32_t rpdo1_cobid        = 0;  // 0 => 0x200 + node_id
            uint32_t tpdo1_cobid        = 0;  // 0 => 0x180 + node_id
            uint32_t rpdo2_cobid        = 0;  // 0 => 0x300 + node_id
            uint32_t tpdo2_cobid        = 0;  // 0 => 0x280 + node_id
            uint32_t heartbeat_cobid    = 0;  // 0 => 0x700 + node_id

            // CiA402 object indices.  Override these for a vendor that does not
            // use the standard object dictionary.
            uint16_t controlword_index      = Od::CONTROLWORD;
            uint16_t statusword_index       = Od::STATUSWORD;
            uint16_t mode_index             = Od::MODES_OF_OPERATION;
            uint16_t mode_display_index     = Od::MODES_OF_OPERATION_DISPLAY;
            uint16_t target_position_index  = Od::TARGET_POSITION;
            uint16_t actual_position_index  = Od::POSITION_ACTUAL;
            uint16_t profile_velocity_index = Od::PROFILE_VELOCITY;
            uint16_t target_velocity_index  = Od::TARGET_VELOCITY;
            uint16_t supported_modes_index  = Od::SUPPORTED_DRIVE_MODES;
            uint16_t device_type_index      = Od::DEVICE_TYPE;
            uint16_t error_register_index   = Od::ERROR_REGISTER;

            uint32_t sdo_request() const { return sdo_request_cobid ? sdo_request_cobid : SDO_REQUEST_COBID_BASE + node_id; }
            uint32_t sdo_response() const { return sdo_response_cobid ? sdo_response_cobid : SDO_RESPONSE_COBID_BASE + node_id; }
            uint32_t rpdo1() const { return rpdo1_cobid ? rpdo1_cobid : RPDO1_COBID_BASE + node_id; }
            uint32_t tpdo1() const { return tpdo1_cobid ? tpdo1_cobid : TPDO1_COBID_BASE + node_id; }
            uint32_t rpdo2() const { return rpdo2_cobid ? rpdo2_cobid : RPDO2_COBID_BASE + node_id; }
            uint32_t tpdo2() const { return tpdo2_cobid ? tpdo2_cobid : TPDO2_COBID_BASE + node_id; }
            uint32_t heartbeat() const { return heartbeat_cobid ? heartbeat_cobid : HEARTBEAT_COBID_BASE + node_id; }
        };

        // ------------------------------------------------------------------
        // Pure frame codecs.  They never touch the bus.
        // ------------------------------------------------------------------

        // NMT command frame: 2 bytes {command, node id}.  node_id 0 broadcasts.
        size_t build_nmt(NmtCommand command, uint8_t node_id, uint8_t* out, size_t max_len);

        // SDO expedited download for 1..4 data bytes.  Returns 8 (a complete
        // expedited frame) or 0 when the arguments are invalid.
        size_t build_sdo_download(uint16_t index, uint8_t subindex, const uint8_t* data, size_t length, uint8_t* out, size_t max_len);

        // SDO initiate-upload request for an object.
        size_t build_sdo_upload(uint16_t index, uint8_t subindex, uint8_t* out, size_t max_len);

        // SDO abort transfer.
        size_t build_sdo_abort(uint16_t index, uint8_t subindex, uint32_t abort_code, uint8_t* out, size_t max_len);

        // True when the frame is an SDO abort for any object; abort_code is the
        // 32-bit little-endian code from bytes 4..7.
        bool parse_sdo_abort(const uint8_t* data, size_t length, uint32_t& abort_code);

        // Decodes an expedited SDO upload response.  Returns false for an abort,
        // a segmented-transfer initiate response (this implementation only
        // supports expedited transfers), or a malformed frame.
        bool parse_sdo_upload(const uint8_t* data, size_t length, uint8_t* value, size_t& value_length);

        // Human-readable name for a CiA301 SDO abort code.
        const char* sdo_abort_name(uint32_t abort_code);

        // Heartbeat / boot-up frame decoder (1 state byte).
        bool parse_heartbeat(const uint8_t* data, size_t length, NmtState& state);

        // PDO1/PDO2 basic codec for the conventional CiA402 default mapping:
        // PDO1 = controlword (Rx, 2 bytes) / statusword (Tx, 2 bytes),
        // PDO2 = target position (Rx, 4 bytes) / actual position (Tx, 4 bytes).
        // All values are little-endian, as CANopen requires.
        size_t build_pdo1(uint16_t controlword, uint8_t* out, size_t max_len);
        bool   parse_pdo1(const uint8_t* data, size_t length, uint16_t& statusword);
        size_t build_pdo2(int32_t target_position, uint8_t* out, size_t max_len);
        bool   parse_pdo2(const uint8_t* data, size_t length, int32_t& actual_position);

        // ------------------------------------------------------------------
        // CiA402 drive profile (standard, vendor independent).
        // ------------------------------------------------------------------
        namespace CiA402 {
            // Controlword commands (CiA402 controlword bits 0..3, bit 7).
            constexpr uint16_t CW_DISABLE_VOLTAGE   = 0x0000;
            constexpr uint16_t CW_QUICK_STOP        = 0x0002;
            constexpr uint16_t CW_SHUTDOWN          = 0x0006;
            constexpr uint16_t CW_SWITCH_ON         = 0x0007;
            constexpr uint16_t CW_ENABLE_OPERATION  = 0x000F;
            constexpr uint16_t CW_FAULT_RESET       = 0x0080;

            // Controlword bits used by the profile position mode.
            constexpr uint16_t CW_NEW_SETPOINT      = 0x0010;
            constexpr uint16_t CW_CHANGE_IMMEDIATELY = 0x0020;

            // Statusword bits.
            constexpr uint16_t SW_FAULT             = 0x0008;
            constexpr uint16_t SW_TARGET_REACHED    = 0x0400;
            constexpr uint16_t SW_SETPOINT_ACK      = 0x1000;
            constexpr uint16_t SW_FOLLOWING_ERROR   = 0x2000;

            enum class State {
                NotReadyToSwitchOn,
                SwitchOnDisabled,
                ReadyToSwitchOn,
                SwitchedOn,
                OperationEnabled,
                QuickStopActive,
                FaultReactionActive,
                Fault,
                Unknown,
            };

            const char* state_name(State state);
            State       decode_state(uint16_t statusword);
            bool        is_fault(uint16_t statusword);
            bool        is_operation_enabled(uint16_t statusword);
            bool        is_target_reached(uint16_t statusword);
            bool        is_setpoint_acknowledged(uint16_t statusword);
        }

        // ------------------------------------------------------------------
    }
}

#endif  // MAX_N_CAN
