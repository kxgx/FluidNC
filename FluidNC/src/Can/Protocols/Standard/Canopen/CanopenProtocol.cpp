// Copyright (c) 2026 - FluidNC contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "Config.h"

#if MAX_N_CAN

#    include "CanopenProtocol.h"




namespace Machine {
    namespace Canopen {
        // ------------------------------------------------------------------
        // Names.
        // ------------------------------------------------------------------
        const char* nmt_command_name(NmtCommand command) {
            switch (command) {
                case NmtCommand::Start:
                    return "start";
                case NmtCommand::Stop:
                    return "stop";
                case NmtCommand::PreOperational:
                    return "pre-operational";
                case NmtCommand::ResetNode:
                    return "reset-node";
                case NmtCommand::ResetCommunication:
                    return "reset-communication";
                default:
                    return "unknown";
            }
        }

        const char* nmt_state_name(NmtState state) {
            switch (state) {
                case NmtState::BootUp:
                    return "boot-up";
                case NmtState::Stopped:
                    return "stopped";
                case NmtState::Operational:
                    return "operational";
                case NmtState::PreOperational:
                    return "pre-operational";
                default:
                    return "unknown";
            }
        }

        const char* sdo_abort_name(uint32_t abort_code) {
            switch (abort_code) {
                case 0x05030000:
                    return "toggle bit not alternated";
                case 0x05040000:
                    return "SDO protocol timed out";
                case 0x05040001:
                    return "command specifier not valid";
                case 0x05040005:
                    return "out of memory";
                case 0x06010000:
                    return "unsupported access";
                case 0x06010001:
                    return "attempt to read a write-only object";
                case 0x06010002:
                    return "attempt to write a read-only object";
                case 0x06020000:
                    return "object does not exist";
                case 0x06040041:
                    return "object cannot be mapped to a PDO";
                case 0x06040042:
                    return "PDO length exceeded";
                case 0x06060000:
                    return "hardware access failure";
                case 0x06070010:
                    return "data type does not match";
                case 0x06090011:
                    return "sub-index does not exist";
                case 0x06090030:
                    return "value range exceeded";
                case 0x08000000:
                    return "general error";
                default:
                    return "unknown abort code";
            }
        }

        namespace CiA402 {
            const char* state_name(State state) {
                switch (state) {
                    case State::NotReadyToSwitchOn:
                        return "not-ready-to-switch-on";
                    case State::SwitchOnDisabled:
                        return "switch-on-disabled";
                    case State::ReadyToSwitchOn:
                        return "ready-to-switch-on";
                    case State::SwitchedOn:
                        return "switched-on";
                    case State::OperationEnabled:
                        return "operation-enabled";
                    case State::QuickStopActive:
                        return "quick-stop-active";
                    case State::FaultReactionActive:
                        return "fault-reaction-active";
                    case State::Fault:
                        return "FAULT";
                    default:
                        return "unknown";
                }
            }

            State decode_state(uint16_t statusword) {
                // CiA402 statusword state bits (mask 0x006F for the states).
                if ((statusword & 0x004F) == 0x0000) {
                    return State::NotReadyToSwitchOn;
                }
                if ((statusword & 0x004F) == 0x0040) {
                    return State::SwitchOnDisabled;
                }
                if ((statusword & 0x006F) == 0x0021) {
                    return State::ReadyToSwitchOn;
                }
                if ((statusword & 0x006F) == 0x0023) {
                    return State::SwitchedOn;
                }
                if ((statusword & 0x006F) == 0x0027) {
                    return State::OperationEnabled;
                }
                if ((statusword & 0x006F) == 0x0007) {
                    return State::QuickStopActive;
                }
                if ((statusword & 0x004F) == 0x000F) {
                    return State::FaultReactionActive;
                }
                if ((statusword & 0x004F) == 0x0008) {
                    return State::Fault;
                }
                return State::Unknown;
            }

            bool is_fault(uint16_t statusword) {
                return (statusword & SW_FAULT) != 0;
            }

            bool is_operation_enabled(uint16_t statusword) {
                return decode_state(statusword) == State::OperationEnabled;
            }

            bool is_target_reached(uint16_t statusword) {
                return (statusword & SW_TARGET_REACHED) != 0;
            }

            bool is_setpoint_acknowledged(uint16_t statusword) {
                return (statusword & SW_SETPOINT_ACK) != 0;
            }
        }

        // ------------------------------------------------------------------
        // Frame codecs.
        // ------------------------------------------------------------------
        size_t build_nmt(NmtCommand command, uint8_t node_id, uint8_t* out, size_t max_len) {
            if (!out || max_len < 2 || node_id > MAX_NODE_ID) {
                return 0;
            }
            out[0] = static_cast<uint8_t>(command);
            out[1] = node_id;
            return 2;
        }

        size_t build_sdo_download(uint16_t index, uint8_t subindex, const uint8_t* data, size_t length, uint8_t* out, size_t max_len) {
            if (!out || max_len < MAX_FRAME_DATA || !data || length == 0 || length > Limits::MAX_EXPEDITED_SDO_DATA) {
                return 0;
            }
            // Command specifier: 0x23 (4 bytes), 0x27 (3), 0x2B (2), 0x2F (1);
            // expedited (bit 1) and size indicated (bit 0) are both set.
            uint8_t command = 0;
            switch (length) {
                case 1:
                    command = 0x2F;
                    break;
                case 2:
                    command = 0x2B;
                    break;
                case 3:
                    command = 0x27;
                    break;
                default:
                    command = 0x23;
                    break;
            }
            out[0] = command;
            out[1] = uint8_t(index & 0xFF);
            out[2] = uint8_t(index >> 8);
            out[3] = subindex;
            for (size_t i = 0; i < 4; ++i) {
                out[4 + i] = i < length ? data[i] : 0;
            }
            return MAX_FRAME_DATA;
        }

        size_t build_sdo_upload(uint16_t index, uint8_t subindex, uint8_t* out, size_t max_len) {
            if (!out || max_len < MAX_FRAME_DATA) {
                return 0;
            }
            out[0] = 0x40;  // initiate expedited upload request
            out[1] = uint8_t(index & 0xFF);
            out[2] = uint8_t(index >> 8);
            out[3] = subindex;
            for (size_t i = 4; i < MAX_FRAME_DATA; ++i) {
                out[i] = 0;
            }
            return MAX_FRAME_DATA;
        }

        size_t build_sdo_abort(uint16_t index, uint8_t subindex, uint32_t abort_code, uint8_t* out, size_t max_len) {
            if (!out || max_len < MAX_FRAME_DATA) {
                return 0;
            }
            out[0] = 0x80;
            out[1] = uint8_t(index & 0xFF);
            out[2] = uint8_t(index >> 8);
            out[3] = subindex;
            out[4] = uint8_t(abort_code & 0xFF);
            out[5] = uint8_t((abort_code >> 8) & 0xFF);
            out[6] = uint8_t((abort_code >> 16) & 0xFF);
            out[7] = uint8_t((abort_code >> 24) & 0xFF);
            return MAX_FRAME_DATA;
        }

        bool parse_sdo_abort(const uint8_t* data, size_t length, uint32_t& abort_code) {
            abort_code = 0;
            if (!data || length < MAX_FRAME_DATA || data[0] != 0x80) {
                return false;
            }
            abort_code = uint32_t(data[4]) | (uint32_t(data[5]) << 8) | (uint32_t(data[6]) << 16) | (uint32_t(data[7]) << 24);
            return true;
        }

        bool parse_sdo_upload(const uint8_t* data, size_t length, uint8_t* value, size_t& value_length) {
            value_length = 0;
            if (!data || length < 4) {
                return false;
            }
            // Response command specifier: 0x43/0x47/0x4B/0x4F for 4/3/2/1
            // byte expedited uploads.  Bit 1 is the expedited flag; bits 2..3
            // are the number of unused bytes.
            if ((data[0] & 0xE0) != 0x40 || (data[0] & 0x02) == 0) {
                return false;
            }
            const size_t unused = (data[0] >> 2) & 0x03;
            const size_t count  = 4 - unused;
            if (length < 4 + count) {
                return false;
            }
            if (value) {
                memcpy(value, data + 4, count);
            }
            value_length = count;
            return true;
        }

        bool parse_heartbeat(const uint8_t* data, size_t length, NmtState& state) {
            if (!data || length < 1) {
                return false;
            }
            switch (data[0]) {
                case 0x00:
                    state = NmtState::BootUp;
                    return true;
                case 0x04:
                    state = NmtState::Stopped;
                    return true;
                case 0x05:
                    state = NmtState::Operational;
                    return true;
                case 0x7F:
                    state = NmtState::PreOperational;
                    return true;
                default:
                    state = NmtState::Unknown;
                    return true;
            }
        }

        size_t build_pdo1(uint16_t controlword, uint8_t* out, size_t max_len) {
            if (!out || max_len < 2) {
                return 0;
            }
            out[0] = uint8_t(controlword & 0xFF);
            out[1] = uint8_t(controlword >> 8);
            return 2;
        }

        bool parse_pdo1(const uint8_t* data, size_t length, uint16_t& statusword) {
            statusword = 0;
            if (!data || length < 2) {
                return false;
            }
            statusword = uint16_t(data[0]) | (uint16_t(data[1]) << 8);
            return true;
        }

        size_t build_pdo2(int32_t target_position, uint8_t* out, size_t max_len) {
            if (!out || max_len < 4) {
                return 0;
            }
            const uint32_t value = uint32_t(target_position);
            out[0]               = uint8_t(value & 0xFF);
            out[1]               = uint8_t((value >> 8) & 0xFF);
            out[2]               = uint8_t((value >> 16) & 0xFF);
            out[3]               = uint8_t((value >> 24) & 0xFF);
            return 4;
        }

        bool parse_pdo2(const uint8_t* data, size_t length, int32_t& actual_position) {
            actual_position = 0;
            if (!data || length < 4) {
                return false;
            }
            const uint32_t value = uint32_t(data[0]) | (uint32_t(data[1]) << 8) | (uint32_t(data[2]) << 16) | (uint32_t(data[3]) << 24);
            actual_position      = int32_t(value);
            return true;
        }

        // ------------------------------------------------------------------
        // Blocking client.
        // ------------------------------------------------------------------

    }
}

#endif  // MAX_N_CAN
