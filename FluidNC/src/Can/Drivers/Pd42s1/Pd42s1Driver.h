// Copyright (c) 2026 - FluidNC contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

/*
    METADATA
    Vendor:                   ALIENTEK (正点原子)
    Driver:                   PD42S1 CAN command facade
    Protocol:                 PD42S1 CAN command dialect (manual V1.2)
    Firmware model:           PD42S1 factory firmware
    Protocol header:          Can/Protocols/Private/Pd42s1/Pd42s1Protocol.h
    Verification status:      hardware exercised; protocol constants owned by
                              the protocol header above.

    CanDriver implementation for the ALIENTEK (正点原子) PD42S1 integrated
    closed-loop stepper driver, in the "CAN" dialect of its serial protocol.

    This class owns everything that is specific to the PD42S1 protocol:
    command echo handling, bit-rate auto-configuration, addressing/group
    addressing, scanning, diagnostics and the $CAN... maintenance commands.  It
    works exclusively through the generic CanBus transport (send/receive/Lock);
    it does not know how the bus worker or the TWAI controller are implemented.

    The streaming motion path is separate: MotorDrivers::Pd42s1Motor keeps its
    own transaction code and shares the framing helpers in Pd42s1Protocol, so a
    motor keeps streaming even when a diagnostic command is not running.

    The fixed 29-bit extended command identifier is Pd42s1Protocol::
    CAN_COMMAND_ID (0x1000); it used to be the can.command_id setting.
*/

#pragma once

#include "Config.h"

#if MAX_N_CAN

#    include "Can/CanBus.h"
#    include "Can/CanDriver.h"
#    include "Can/Protocols/Private/Pd42s1/Pd42s1Protocol.h"

#    include <cstddef>
#    include <cstdint>

namespace Machine {
    class Pd42s1Driver final : public CanDriver {
    public:
        explicit Pd42s1Driver(CanBus* bus) : _bus(bus) {}

        // Fixed 29-bit command identifier used from controller to PD42S1
        // (0x1000).  Responses use the vendor's fixed 0x1 identifier; this
        // driver currently correlates by the address byte in the payload.
        // The value is owned by Pd42s1Protocol::CAN_COMMAND_ID.
        static constexpr uint32_t CAN_COMMAND_ID = MotorDrivers::Pd42s1Protocol::CAN_COMMAND_ID;

        const char* driver_name() const override { return "pd42s1"; }
        CanProtocolClass protocol_class() const override { return CanProtocolClass::Private; }

        // Runs the PD42S1 auto-configuration (echo probe, bit-rate discovery,
        // parameter save/readback).  Called by CanBus::init() after the worker
        // is running; the transport is suspended while this probes.
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
        bool driver_ok(const char* operation);

        // One complete request/response transaction.  The caller must hold the
        // CanBus::Lock (except during init_driver()/scan, where the worker is
        // suspended).
        bool transact(
            const uint8_t* request, size_t requestLen, uint8_t* response, size_t responseCapacity, size_t& responseLen, uint32_t timeoutMs);

        bool probe_address_quiet(uint8_t address, uint8_t& firmware, uint8_t& hardware, uint32_t timeout_ms, int attempts);
        bool read_driver_config(uint8_t address, uint16_t& can_baud_kbps, uint8_t& echo, uint8_t& mode);
        bool read_driver_position(uint8_t address, int32_t& position);

        int scan_address(uint8_t address, uint32_t timeout_ms);
        int scan_addresses();

        // Drives the transceiver D line and reads its R line back to detect a
        // wired CAN transceiver.  Suspends and restores the transport around
        // the test.
        bool test_transceiver();

        bool auto_configure_drivers();
        bool switch_driver_baud_retry(uint8_t address, uint32_t oldBaud, uint32_t newBaud);

        // Echo handling.  The *_locked variants require the caller to hold the
        // CanBus::Lock.
        bool send_echo_state(bool enable);
        bool send_echo_state_forced(bool enable);
        bool enable_driver_echo_locked(uint8_t address);
        void set_echo_to_address_locked(uint8_t address, bool enable);
        void enable_all_driver_echo_locked();
        void disable_all_driver_echo_locked();

        // Configured slave addresses, or the full probe range 1..MAX_CLIENTS
        // when no address was registered from the configuration.
        size_t target_addresses(uint8_t* out, size_t max) const;

        CanBus* _bus;

        // Last echo state we configured on the drivers; avoids re-sending the
        // broadcast (and flooding the log) when several motors re-sync.
        bool _echo_state = false;
    };
}

#endif  // MAX_N_CAN

