// Copyright (c) 2026 - FluidNC contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include "Config.h"

#if MAX_N_CAN

#    include "Can/CanDriver.h"
#    include "Can/CanBus.h"

namespace Machine {
    // Command/diagnostic facade for the Zhang Datou XS CAN protocol.  The
    // motion path lives in MotorDrivers::ZdtMotor; this class only exposes the
    // generic $CAN.../M15x command surface.
    class ZdtCanDriver : public CanDriver {
    public:
        explicit ZdtCanDriver(CanBus* bus) : _bus(bus) {}

        const char* driver_name() const override { return "zdt"; }
        CanProtocolClass protocol_class() const override { return CanProtocolClass::Private; }
        bool        init_driver() override;
        int         scan() override;
        int         scan_all_bauds() override;
        void        report_status() override;
        void        clear_driver_faults() override;
        void        restart_drivers() override;
        void        enable_all_driver_echo() override;
        bool        set_driver_enable(uint8_t address, bool en) override;
        bool        send_single_position(uint8_t address, int32_t delta, uint16_t speed_rpm, uint8_t accel) override;
        bool        set_group_address(uint8_t address, uint8_t group) override;
        size_t      client_addresses(uint8_t* out, size_t max) const override;
        bool        zero_driver_position(uint8_t address) override;
        bool        wait_for_driver_sync(const uint8_t* addresses, size_t count, uint32_t timeout_ms) override;
        void        report_driver_faults() override;
        void        report_driver_positions() override;
        bool        stop_all_drivers() override;
        void        read_raw(uint8_t address, uint8_t function) override;
        bool        run_auto_config() override;
        void        report_driver_configs() override;
        bool        suspend() override;
        bool        resume() override;
        bool        control_ready() const override;

    private:
        bool transact(uint8_t address, const uint8_t* message, size_t length, uint8_t expectedFunction, uint8_t* response, size_t& responseLen);
        bool read_counts(uint8_t address, int32_t& counts);

        CanBus* _bus;
    };
}

#endif
