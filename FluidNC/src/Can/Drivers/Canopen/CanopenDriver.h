// Copyright (c) 2026 - FluidNC contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

/*
    CanDriver implementation for the standard CANopen (CiA301) / CiA402 drive
    profile.

    This driver is reusable across vendors: it uses the standard COB-IDs and the
    standard CiA402 object indices from CanopenProtocol, with per-node overrides
    taken from the NodeConfig that each configured CanopenMotor registers before
    the bus starts.  It deliberately has NO vendor-specific fallback: if a drive
    does not answer standard CANopen SDOs, it is reported as absent instead of
    being poked with a guess at a private protocol.

    The command surface covers the minimal set needed for this machine:
    scanning (SDO read of 0x1000, with a heartbeat fallback), status/fault/
    position reports, CiA402 enable/disable (shutdown -> switch on -> enable
    operation), quick stop for $CANStop/M157, and a single absolute position
    move.  Position zeroing, group addressing and raw vendor reads have no
    standard CANopen equivalent, so they are reported as unsupported rather than
    faked.

    IMPORTANT: this has not been verified against any real CANopen hardware.
    The Zhang Datou X42S V2.0 / X28S V2.0 manuals do not document a CANopen
    object dictionary, so the CiA402 defaults are assumed (see
    Motors/CanopenMotor.md).
*/

#pragma once

#include "Config.h"

#if MAX_N_CAN

#    include "Can/CanBus.h"
#    include "Can/CanDriver.h"
#    include "Can/Protocols/Standard/Canopen/CanopenProtocol.h"

#    include <cstddef>
#    include <cstdint>

namespace Machine {
    class CanopenDriver final : public CanDriver {
    public:
        explicit CanopenDriver(CanBus* bus) : _bus(bus) {}

        const char*       driver_name() const override { return "canopen"; }
        CanProtocolClass  protocol_class() const override { return CanProtocolClass::Standard; }

        // Sends NMT Start to every configured node id so an operational-capable
        // drive leaves pre-operational mode.  Does not fail the bus when a node
        // does not answer: each motor synchronises itself and control stays
        // disabled until it does, while diagnostics remain available.
        bool init_driver() override;

        int  scan() override;
        int  scan_all_bauds() override;
        void report_status() override;
        void clear_driver_faults() override;
        void restart_drivers() override;
        void enable_all_driver_echo() override;
        bool set_driver_enable(uint8_t address, bool en) override;
        bool send_single_position(uint8_t address, int32_t delta, uint16_t speed_rpm, uint8_t accel) override;
        bool set_group_address(uint8_t address, uint8_t group) override;
        size_t client_addresses(uint8_t* out, size_t max) const override;
        bool zero_driver_position(uint8_t address) override;
        bool wait_for_driver_sync(const uint8_t* addresses, size_t count, uint32_t timeout_ms) override;
        void report_driver_faults() override;
        void report_driver_positions() override;
        bool stop_all_drivers() override;
        void read_raw(uint8_t address, uint8_t function) override;
        bool run_auto_config() override;
        void report_driver_configs() override;
        bool suspend() override;
        bool resume() override;
        bool control_ready() const override;

    private:
        // Configured node ids, or the probe range 1..MAX_CLIENTS when no CANopen
        // motor was configured.
        size_t target_addresses(uint8_t* out, size_t max) const;

        // Effective node configuration: the CanopenMotor's registered config, or
        // the standard CiA402 defaults for a bare node id.
        bool node_for_address(uint8_t address, Canopen::NodeConfig& node) const;

        // One node probe: SDO read of 0x1000, falling back to a heartbeat listen.
        int probe_node(uint8_t node_id);

        CanBus* _bus;
    };
}

#endif  // MAX_N_CAN
