// Copyright (c) 2026 - FluidNC contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "Config.h"

#if MAX_N_CAN

#    include "CanopenProtocol.h"

#    include "Can/CanBus.h"
#    include "Logging.h"

#    include <Arduino.h>

#    include <cstdio>
#    include <cstring>

namespace Machine {
    namespace {
        // Formats an object index for a log line into caller-owned storage so
        // two tasks logging concurrently cannot corrupt each other's text.
        void index_text(uint16_t index, char (&out)[8]) {
            snprintf(out, sizeof(out), "0x%04X", unsigned(index));
        }

        using Canopen::NodeConfig;
        static_assert(Canopen::MAX_REGISTERED_NODES == CanBus::MAX_CLIENTS,
                      "the CANopen node registry must match the shared CAN client limit");

        NodeConfig registrySlots[Canopen::MAX_REGISTERED_NODES];
        bool       registryUsed[Canopen::MAX_REGISTERED_NODES] = {};
    }

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
        // Node registry.
        // ------------------------------------------------------------------
        void NodeRegistry::add(const NodeConfig& config) {
            size_t free_slot = MAX_REGISTERED_NODES;
            for (size_t i = 0; i < MAX_REGISTERED_NODES; ++i) {
                if (registryUsed[i] && registrySlots[i].node_id == config.node_id) {
                    registrySlots[i] = config;
                    return;
                }
                if (!registryUsed[i] && free_slot == MAX_REGISTERED_NODES) {
                    free_slot = i;
                }
            }
            if (free_slot == MAX_REGISTERED_NODES) {
                can_log_error("CANopen: node registry is full; node " << int(config.node_id) << " cannot be registered");
                return;
            }
            registrySlots[free_slot] = config;
            registryUsed[free_slot]  = true;
        }

        void NodeRegistry::remove(uint8_t node_id) {
            for (size_t i = 0; i < MAX_REGISTERED_NODES; ++i) {
                if (registryUsed[i] && registrySlots[i].node_id == node_id) {
                    registryUsed[i] = false;
                    return;
                }
            }
        }

        bool NodeRegistry::get(uint8_t node_id, NodeConfig& out) {
            for (size_t i = 0; i < MAX_REGISTERED_NODES; ++i) {
                if (registryUsed[i] && registrySlots[i].node_id == node_id) {
                    out = registrySlots[i];
                    return true;
                }
            }
            return false;
        }

        void NodeRegistry::clear() {
            for (size_t i = 0; i < MAX_REGISTERED_NODES; ++i) {
                registryUsed[i] = false;
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
            if (!out || max_len < MAX_FRAME_DATA || !data || length == 0 || length > 4) {
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
        Client::Client(CanBus* bus, const NodeConfig& node) : _bus(bus), _node(node) {}

        Client::Client(CanBus* bus, uint8_t node_id) : _bus(bus) {
            _node.node_id = node_id;
        }

        bool Client::transact(const uint8_t* request, size_t length, uint8_t* response, size_t& response_length, uint32_t timeout_ms) {
            response_length = 0;
            if (!_bus || !_bus->ready() || !request || !response || length == 0 || length > MAX_FRAME_DATA) {
                return false;
            }

            CanBus::Lock lock(*_bus);
            // A timed-out transaction leaves frames behind; start clean so a
            // stale response cannot be taken for this request's answer.
            _bus->flush_receive();
            if (!_bus->send_frame(_node.sdo_request(), false, request, length, timeout_ms)) {
                _bus->flush_receive();
                return false;
            }

            const uint32_t deadline = millis() + timeout_ms;
            for (;;) {
                const int32_t remaining = int32_t(deadline - millis());
                if (remaining <= 0) {
                    break;
                }
                uint32_t id       = 0;
                bool     extended = false;
                uint8_t  frame[MAX_FRAME_DATA] = {};
                size_t   received              = _bus->receive_frame(id, extended, frame, sizeof(frame), uint32_t(remaining));
                if (received == 0) {
                    break;
                }
                // The bus is shared: heartbeat frames of this node, replies of
                // another node and stale frames from an earlier transaction
                // must all be skipped while waiting for this node's SDO reply.
                if (extended || id != _node.sdo_response()) {
                    continue;
                }
                if (received > sizeof(frame)) {
                    received = sizeof(frame);
                }
                memcpy(response, frame, received);
                response_length = received;
                return true;
            }
            _bus->flush_receive();
            return false;
        }

        bool Client::send_cobid_frame(uint32_t cobid, const uint8_t* data, size_t length, uint32_t timeout_ms) {
            if (!_bus || !_bus->ready() || !data || length == 0 || length > MAX_FRAME_DATA) {
                return false;
            }
            CanBus::Lock lock(*_bus);
            return _bus->send_frame(cobid, false, data, length, timeout_ms);
        }

        bool Client::wait_cobid_frame(uint32_t cobid, uint8_t* data, size_t& length, uint32_t timeout_ms) {
            length = 0;
            if (!_bus || !_bus->ready() || !data) {
                return false;
            }
            CanBus::Lock    lock(*_bus);
            const uint32_t deadline = millis() + timeout_ms;
            for (;;) {
                const int32_t remaining = int32_t(deadline - millis());
                if (remaining <= 0) {
                    break;
                }
                uint32_t id       = 0;
                bool     extended = false;
                uint8_t  frame[MAX_FRAME_DATA] = {};
                size_t   received              = _bus->receive_frame(id, extended, frame, sizeof(frame), uint32_t(remaining));
                if (received == 0) {
                    break;
                }
                if (extended || id != cobid) {
                    continue;
                }
                memcpy(data, frame, received);
                length = received;
                return true;
            }
            return false;
        }

        bool Client::nmt(NmtCommand command, uint32_t timeout_ms) {
            uint8_t frame[2] = {};
            size_t  length   = build_nmt(command, _node.node_id, frame, sizeof(frame));
            if (!length) {
                can_log_error("CANopen: invalid NMT command for node " << int(_node.node_id));
                return false;
            }
            return send_cobid_frame(NMT_COBID, frame, length, timeout_ms);
        }

        bool Client::sdo_download(uint16_t index, uint8_t subindex, const uint8_t* data, size_t length, uint32_t timeout_ms) {
            uint8_t request[MAX_FRAME_DATA]  = {};
            uint8_t response[MAX_FRAME_DATA] = {};
            char    object[8];
            index_text(index, object);

            const size_t request_length = build_sdo_download(index, subindex, data, length, request, sizeof(request));
            if (!request_length) {
                can_log_error("CANopen: invalid SDO download for object " << object);
                return false;
            }
            size_t response_length = 0;
            if (!transact(request, request_length, response, response_length, timeout_ms)) {
                can_log_warn("CANopen: node " << int(_node.node_id) << " SDO download " << object << " timed out");
                return false;
            }
            if (response_length < 4) {
                can_log_warn("CANopen: node " << int(_node.node_id) << " SDO download " << object << " short response");
                return false;
            }
            if (response[0] == 0x80) {
                uint32_t abort_code = 0;
                parse_sdo_abort(response, response_length, abort_code);
                can_log_warn("CANopen: node " << int(_node.node_id) << " SDO download " << object << " aborted: " << sdo_abort_name(abort_code));
                return false;
            }
            if (response[0] != 0x60) {
                can_log_warn("CANopen: node " << int(_node.node_id) << " SDO download " << object << " unexpected response");
                return false;
            }
            return true;
        }

        bool Client::sdo_download_u8(uint16_t index, uint8_t subindex, uint8_t value, uint32_t timeout_ms) {
            return sdo_download(index, subindex, &value, 1, timeout_ms);
        }

        bool Client::sdo_download_u16(uint16_t index, uint8_t subindex, uint16_t value, uint32_t timeout_ms) {
            const uint8_t data[2] = { uint8_t(value & 0xFF), uint8_t(value >> 8) };
            return sdo_download(index, subindex, data, sizeof(data), timeout_ms);
        }

        bool Client::sdo_download_i32(uint16_t index, uint8_t subindex, int32_t value, uint32_t timeout_ms) {
            const uint32_t raw    = uint32_t(value);
            const uint8_t  data[4] = { uint8_t(raw & 0xFF), uint8_t((raw >> 8) & 0xFF), uint8_t((raw >> 16) & 0xFF), uint8_t((raw >> 24) & 0xFF) };
            return sdo_download(index, subindex, data, sizeof(data), timeout_ms);
        }

        bool Client::sdo_upload(uint16_t index, uint8_t subindex, uint8_t* data, size_t& length, uint32_t timeout_ms) {
            length = 0;
            uint8_t request[MAX_FRAME_DATA]  = {};
            uint8_t response[MAX_FRAME_DATA] = {};
            char    object[8];
            index_text(index, object);

            const size_t request_length = build_sdo_upload(index, subindex, request, sizeof(request));
            if (!request_length) {
                can_log_error("CANopen: invalid SDO upload for object " << object);
                return false;
            }
            size_t response_length = 0;
            if (!transact(request, request_length, response, response_length, timeout_ms)) {
                can_log_verbose("CANopen: node " << int(_node.node_id) << " SDO upload " << object << " timed out");
                return false;
            }
            if (response_length < 4) {
                can_log_warn("CANopen: node " << int(_node.node_id) << " SDO upload " << object << " short response");
                return false;
            }
            if (response[0] == 0x80) {
                uint32_t abort_code = 0;
                parse_sdo_abort(response, response_length, abort_code);
                can_log_warn("CANopen: node " << int(_node.node_id) << " SDO upload " << object << " aborted: " << sdo_abort_name(abort_code));
                return false;
            }
            size_t value_length = 0;
            if (!parse_sdo_upload(response, response_length, data, value_length)) {
                if ((response[0] & 0xE0) == 0x40 && (response[0] & 0x02) == 0) {
                    // Initiate segmented upload (0x41).  This implementation
                    // only supports expedited transfers, so abort cleanly
                    // instead of leaving the server waiting for a segment.
                    can_log_warn("CANopen: node " << int(_node.node_id) << " object " << object
                                                  << " needs a segmented SDO upload; only expedited (<=4 byte) objects are supported");
                    uint8_t abort_frame[MAX_FRAME_DATA] = {};
                    uint8_t abort_reply[MAX_FRAME_DATA] = {};
                    size_t  abort_length                = build_sdo_abort(index, subindex, 0x05040001, abort_frame, sizeof(abort_frame));
                    size_t  abort_reply_length          = 0;
                    if (abort_length) {
                        transact(abort_frame, abort_length, abort_reply, abort_reply_length, 100);
                    }
                } else {
                    can_log_warn("CANopen: node " << int(_node.node_id) << " object " << object << " malformed SDO upload response");
                }
                return false;
            }
            length = value_length;
            return true;
        }

        bool Client::sdo_upload_u8(uint16_t index, uint8_t subindex, uint8_t& value, uint32_t timeout_ms) {
            uint8_t buffer[4] = {};
            size_t  length    = 0;
            if (!sdo_upload(index, subindex, buffer, length, timeout_ms)) {
                return false;
            }
            if (length != 1) {
                can_log_warn("CANopen: node " << int(_node.node_id) << " object " << index << " is not 1 byte");
                return false;
            }
            value = buffer[0];
            return true;
        }

        bool Client::sdo_upload_u16(uint16_t index, uint8_t subindex, uint16_t& value, uint32_t timeout_ms) {
            uint8_t buffer[4] = {};
            size_t  length    = 0;
            if (!sdo_upload(index, subindex, buffer, length, timeout_ms)) {
                return false;
            }
            if (length != 2) {
                can_log_warn("CANopen: node " << int(_node.node_id) << " object " << index << " is not 2 bytes");
                return false;
            }
            value = uint16_t(buffer[0]) | (uint16_t(buffer[1]) << 8);
            return true;
        }

        bool Client::sdo_upload_u32(uint16_t index, uint8_t subindex, uint32_t& value, uint32_t timeout_ms) {
            uint8_t buffer[4] = {};
            size_t  length    = 0;
            if (!sdo_upload(index, subindex, buffer, length, timeout_ms)) {
                return false;
            }
            if (length != 4) {
                can_log_warn("CANopen: node " << int(_node.node_id) << " object " << index << " is not 4 bytes");
                return false;
            }
            value = uint32_t(buffer[0]) | (uint32_t(buffer[1]) << 8) | (uint32_t(buffer[2]) << 16) | (uint32_t(buffer[3]) << 24);
            return true;
        }

        bool Client::sdo_upload_i8(uint16_t index, uint8_t subindex, int8_t& value, uint32_t timeout_ms) {
            uint8_t raw = 0;
            if (!sdo_upload_u8(index, subindex, raw, timeout_ms)) {
                return false;
            }
            value = int8_t(raw);
            return true;
        }

        bool Client::sdo_upload_i32(uint16_t index, uint8_t subindex, int32_t& value, uint32_t timeout_ms) {
            uint32_t raw = 0;
            if (!sdo_upload_u32(index, subindex, raw, timeout_ms)) {
                return false;
            }
            value = int32_t(raw);
            return true;
        }

        bool Client::sdo_abort(uint16_t index, uint8_t subindex, uint32_t abort_code, uint32_t timeout_ms) {
            uint8_t request[MAX_FRAME_DATA]  = {};
            uint8_t response[MAX_FRAME_DATA] = {};
            const size_t request_length      = build_sdo_abort(index, subindex, abort_code, request, sizeof(request));
            if (!request_length) {
                return false;
            }
            size_t response_length = 0;
            return transact(request, request_length, response, response_length, timeout_ms);
        }

        bool Client::wait_heartbeat(NmtState& state, uint32_t timeout_ms) {
            state                     = NmtState::Unknown;
            uint8_t frame[MAX_FRAME_DATA] = {};
            size_t  length                = 0;
            if (!wait_cobid_frame(_node.heartbeat(), frame, length, timeout_ms)) {
                return false;
            }
            return parse_heartbeat(frame, length, state);
        }

        bool Client::send_pdo1(const uint8_t* data, size_t length, uint32_t timeout_ms) {
            return send_cobid_frame(_node.rpdo1(), data, length, timeout_ms);
        }

        bool Client::send_pdo2(const uint8_t* data, size_t length, uint32_t timeout_ms) {
            return send_cobid_frame(_node.rpdo2(), data, length, timeout_ms);
        }

        bool Client::wait_pdo1(uint16_t& statusword, uint32_t timeout_ms) {
            uint8_t frame[MAX_FRAME_DATA] = {};
            size_t  length                = 0;
            if (!wait_cobid_frame(_node.tpdo1(), frame, length, timeout_ms)) {
                return false;
            }
            return parse_pdo1(frame, length, statusword);
        }

        bool Client::wait_pdo2(int32_t& actual_position, uint32_t timeout_ms) {
            uint8_t frame[MAX_FRAME_DATA] = {};
            size_t  length                = 0;
            if (!wait_cobid_frame(_node.tpdo2(), frame, length, timeout_ms)) {
                return false;
            }
            return parse_pdo2(frame, length, actual_position);
        }

        bool Client::read_statusword(uint16_t& statusword, uint32_t timeout_ms) {
            return sdo_upload_u16(_node.statusword_index, 0, statusword, timeout_ms);
        }

        bool Client::write_controlword(uint16_t controlword, uint32_t timeout_ms) {
            return sdo_download_u16(_node.controlword_index, 0, controlword, timeout_ms);
        }

        bool Client::read_actual_position(int32_t& position, uint32_t timeout_ms) {
            return sdo_upload_i32(_node.actual_position_index, 0, position, timeout_ms);
        }

        bool Client::write_target_position(int32_t position, uint32_t timeout_ms) {
            return sdo_download_i32(_node.target_position_index, 0, position, timeout_ms);
        }

        bool Client::read_mode_display(int8_t& mode, uint32_t timeout_ms) {
            return sdo_upload_i8(_node.mode_display_index, 0, mode, timeout_ms);
        }

        bool Client::write_profile_velocity(uint32_t velocity, uint32_t timeout_ms) {
            return sdo_download_i32(_node.profile_velocity_index, 0, int32_t(velocity), timeout_ms);
        }

        bool Client::read_device_type(uint32_t& device_type, uint32_t timeout_ms) {
            return sdo_upload_u32(_node.device_type_index, 0, device_type, timeout_ms);
        }

        bool Client::read_error_register(uint8_t& error_register, uint32_t timeout_ms) {
            return sdo_upload_u8(_node.error_register_index, 0, error_register, timeout_ms);
        }

        bool Client::read_supported_drive_modes(uint32_t& modes, uint32_t timeout_ms) {
            return sdo_upload_u32(_node.supported_modes_index, 0, modes, timeout_ms);
        }

        bool Client::enable_operation(uint32_t timeout_ms) {
            const uint32_t deadline        = millis() + timeout_ms;
            bool           reset_attempted = false;

            while (int32_t(deadline - millis()) > 0) {
                uint16_t statusword = 0;
                if (!read_statusword(statusword, 150)) {
                    vTaskDelay(pdMS_TO_TICKS(10));
                    continue;
                }
                if (CiA402::is_fault(statusword)) {
                    if (reset_attempted) {
                        can_log_warn("CANopen: node " << int(_node.node_id) << " stayed in FAULT after a reset attempt");
                        return false;
                    }
                    reset_attempted = true;
                    if (!fault_reset(500)) {
                        can_log_warn("CANopen: node " << int(_node.node_id) << " fault reset failed");
                        return false;
                    }
                    continue;
                }

                switch (CiA402::decode_state(statusword)) {
                    case CiA402::State::OperationEnabled:
                        return true;
                    case CiA402::State::SwitchOnDisabled:
                        write_controlword(CiA402::CW_SHUTDOWN, 150);
                        break;
                    case CiA402::State::ReadyToSwitchOn:
                        write_controlword(CiA402::CW_SWITCH_ON, 150);
                        break;
                    case CiA402::State::SwitchedOn:
                        write_controlword(CiA402::CW_ENABLE_OPERATION, 150);
                        break;
                    case CiA402::State::QuickStopActive:
                        // Leaving quick stop needs a disable-voltage command
                        // first; the next pass then runs shutdown/switch on.
                        write_controlword(CiA402::CW_DISABLE_VOLTAGE, 150);
                        break;
                    case CiA402::State::FaultReactionActive:
                    case CiA402::State::NotReadyToSwitchOn:
                    case CiA402::State::Fault:
                    case CiA402::State::Unknown:
                        break;  // wait for the drive
                }
                vTaskDelay(pdMS_TO_TICKS(10));
            }

            can_log_warn("CANopen: node " << int(_node.node_id) << " did not reach operation enabled within " << timeout_ms << " ms");
            return false;
        }

        bool Client::disable_operation(uint32_t timeout_ms) {
            // Shutdown removes the torque/tracks the safe disabled state; this
            // is the normal MD/ME disable path, not an emergency stop.
            if (!write_controlword(CiA402::CW_SHUTDOWN, 150)) {
                return false;
            }
            const uint32_t deadline = millis() + timeout_ms;
            while (int32_t(deadline - millis()) > 0) {
                uint16_t statusword = 0;
                if (!read_statusword(statusword, 150)) {
                    vTaskDelay(pdMS_TO_TICKS(10));
                    continue;
                }
                const CiA402::State state = CiA402::decode_state(statusword);
                if (state == CiA402::State::SwitchOnDisabled || state == CiA402::State::ReadyToSwitchOn) {
                    return true;
                }
                if (state == CiA402::State::Fault) {
                    return false;
                }
                vTaskDelay(pdMS_TO_TICKS(10));
            }
            return false;
        }

        bool Client::quick_stop(uint32_t timeout_ms) {
            if (!write_controlword(CiA402::CW_QUICK_STOP, timeout_ms)) {
                return false;
            }
            // The controlword write is what trips the drive; the readback is
            // diagnostic only and must not turn a safely commanded stop into a
            // reported failure.
            uint16_t statusword = 0;
            if (read_statusword(statusword, 150)) {
                can_log_verbose("CANopen: node " << int(_node.node_id) << " quick stop -> " << CiA402::state_name(CiA402::decode_state(statusword)));
            }
            return true;
        }

        bool Client::fault_reset(uint32_t timeout_ms) {
            // CiA402: fault reset happens on a rising edge of controlword bit 7.
            if (!write_controlword(CiA402::CW_DISABLE_VOLTAGE, 150)) {
                return false;
            }
            if (!write_controlword(CiA402::CW_FAULT_RESET, 150)) {
                return false;
            }
            const uint32_t deadline = millis() + timeout_ms;
            while (int32_t(deadline - millis()) > 0) {
                vTaskDelay(pdMS_TO_TICKS(10));
                uint16_t statusword = 0;
                if (!read_statusword(statusword, 150)) {
                    continue;
                }
                if (!CiA402::is_fault(statusword)) {
                    // Leave the drive in switch-on-disabled; the caller runs
                    // the normal shutdown/switch-on/enable sequence next.
                    write_controlword(CiA402::CW_SHUTDOWN, 150);
                    return true;
                }
            }
            return false;
        }
    }
}

#endif  // MAX_N_CAN
