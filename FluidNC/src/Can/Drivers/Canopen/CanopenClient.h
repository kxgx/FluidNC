// Copyright (c) 2026 - FluidNC contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

/*
    Transport adapter for the standard CANopen (CiA301/CiA402) protocol.

    This file deliberately lives in the driver layer: it owns CanBus locking,
    frame send/receive and Arduino/RTOS timing.  The protocol header
    (CanopenProtocol.h) remains a transport-independent codec/constant table.
*/

#pragma once

#include "Config.h"

#if MAX_N_CAN

#    include "Can/CanBus.h"
#    include "Can/Protocols/Standard/Canopen/CanopenProtocol.h"

namespace Machine {
    namespace Canopen {
        // Registry of the effective NodeConfig of every configured CANopen
        // motor.  CanopenMotor registers its configuration in validate(), before
        // CanBus::init() creates the selected CanDriver; the driver uses it so
        // $CANEnable/$CANMove/$CANStatus talk to the same node ids and object
        // dictionary that the motion path uses.  Empty entries mean "use the
        // standard defaults for that node id".
        // ------------------------------------------------------------------
        constexpr size_t MAX_REGISTERED_NODES = Limits::MAX_REGISTERED_NODES;

        class NodeRegistry {
        public:
            // Adds or replaces the entry for config.node_id.
            static void add(const NodeConfig& config);
            static void remove(uint8_t node_id);
            // Returns false when no motor registered that node id.
            static bool get(uint8_t node_id, NodeConfig& out);
            static void clear();
        };

        // ------------------------------------------------------------------
        // Blocking CANopen client for one node on one bus.
        //
        // Every public operation is one complete transaction: it takes the
        // CanBus::Lock, flushes stale frames, sends the request and waits for a
        // response with this node's expected COB-ID (frames belonging to other
        // nodes on the shared bus are ignored).  Because the lock is taken
        // inside, callers must NOT already hold it.
        // ------------------------------------------------------------------
        class Client {
        public:
            Client(CanBus* bus, const NodeConfig& node);
            Client(CanBus* bus, uint8_t node_id);

            CanBus*           bus() const { return _bus; }
            const NodeConfig& node() const { return _node; }

            // Network management.  A broadcast (node id 0) command frame has no
            // response; the return value is only the transmit result.
            bool nmt(NmtCommand command, uint32_t timeout_ms = 50);

            // Generic expedited SDO transfers.  `length` is 1..4 data bytes.
            bool sdo_download(uint16_t index, uint8_t subindex, const uint8_t* data, size_t length, uint32_t timeout_ms = 150);
            bool sdo_download_u8(uint16_t index, uint8_t subindex, uint8_t value, uint32_t timeout_ms = 150);
            bool sdo_download_u16(uint16_t index, uint8_t subindex, uint16_t value, uint32_t timeout_ms = 150);
            bool sdo_download_i32(uint16_t index, uint8_t subindex, int32_t value, uint32_t timeout_ms = 150);
            bool sdo_upload(uint16_t index, uint8_t subindex, uint8_t* data, size_t& length, uint32_t timeout_ms = 150);
            bool sdo_upload_u8(uint16_t index, uint8_t subindex, uint8_t& value, uint32_t timeout_ms = 150);
            bool sdo_upload_u16(uint16_t index, uint8_t subindex, uint16_t& value, uint32_t timeout_ms = 150);
            bool sdo_upload_u32(uint16_t index, uint8_t subindex, uint32_t& value, uint32_t timeout_ms = 150);
            bool sdo_upload_i8(uint16_t index, uint8_t subindex, int8_t& value, uint32_t timeout_ms = 150);
            bool sdo_upload_i32(uint16_t index, uint8_t subindex, int32_t& value, uint32_t timeout_ms = 150);
            bool sdo_abort(uint16_t index, uint8_t subindex, uint32_t abort_code, uint32_t timeout_ms = 150);

            // Heartbeat listen.  Returns true when a heartbeat or boot-up frame
            // from this node arrived within the timeout.
            bool wait_heartbeat(NmtState& state, uint32_t timeout_ms);

            // PDO1/PDO2 basic send/receive helpers.  The drive's PDO mapping
            // must already be configured for these frames to mean anything.
            bool send_pdo1(const uint8_t* data, size_t length, uint32_t timeout_ms = 50);
            bool send_pdo2(const uint8_t* data, size_t length, uint32_t timeout_ms = 50);
            bool wait_pdo1(uint16_t& statusword, uint32_t timeout_ms);
            bool wait_pdo2(int32_t& actual_position, uint32_t timeout_ms);

            // CiA402 convenience wrappers using the node's configured indices.
            bool read_statusword(uint16_t& statusword, uint32_t timeout_ms = 150);
            bool write_controlword(uint16_t controlword, uint32_t timeout_ms = 150);
            bool read_actual_position(int32_t& position, uint32_t timeout_ms = 150);
            bool write_target_position(int32_t position, uint32_t timeout_ms = 150);
            bool read_mode_display(int8_t& mode, uint32_t timeout_ms = 150);
            bool write_profile_velocity(uint32_t velocity, uint32_t timeout_ms = 150);
            bool read_device_type(uint32_t& device_type, uint32_t timeout_ms = 150);
            bool read_error_register(uint8_t& error_register, uint32_t timeout_ms = 150);
            bool read_supported_drive_modes(uint32_t& modes, uint32_t timeout_ms = 150);

            // CiA402 state-machine transitions.
            bool enable_operation(uint32_t timeout_ms = 1000);
            bool disable_operation(uint32_t timeout_ms = 500);
            bool quick_stop(uint32_t timeout_ms = 500);
            bool fault_reset(uint32_t timeout_ms = 500);

        private:
            bool transact(const uint8_t* request, size_t length, uint8_t* response, size_t& response_length, uint32_t timeout_ms);
            bool send_cobid_frame(uint32_t cobid, const uint8_t* data, size_t length, uint32_t timeout_ms);
            bool wait_cobid_frame(uint32_t cobid, uint8_t* data, size_t& length, uint32_t timeout_ms);

            CanBus*    _bus;
            NodeConfig _node;
        };
    }
}

#endif  // MAX_N_CAN
