// Copyright (c) 2026 - FluidNC contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include "Config.h"

#if MAX_N_CAN

#    include <cstddef>
#    include <cstdint>

namespace Machine {
    // High-level operations used by the $CAN... and M15x command layer.
    //
    // This is intentionally independent of the PD42S1 wire format.  A new CAN
    // motor vendor implements this interface (or a transport-level protocol
    // adapter plus this command interface) without touching GCode.cpp or
    // ProcessSettings.cpp.  The existing CanBus is the PD42S1 implementation
    // and also owns the shared TWAI transport for now.
    class CanDriver {
    public:
        virtual ~CanDriver() = default;

        virtual const char* driver_name() const = 0;

        // False for a driver that does not need the shared TWAI transport,
        // e.g. a command-layer stub used to validate can.driver selection.
        virtual bool uses_transport() const { return true; }

        virtual int scan()                                        = 0;
        virtual int scan_all_bauds()                              = 0;
        virtual void report_status()                              = 0;
        virtual void clear_driver_faults()                        = 0;
        virtual void restart_drivers()                            = 0;
        virtual void enable_all_driver_echo()                     = 0;
        virtual bool set_driver_enable(uint8_t address, bool en)  = 0;
        virtual bool send_single_position(uint8_t address, int32_t delta, uint16_t speed_rpm, uint8_t accel) = 0;
        virtual bool set_group_address(uint8_t address, uint8_t group) = 0;
        virtual size_t client_addresses(uint8_t* out, size_t max) const = 0;
        virtual bool zero_driver_position(uint8_t address)              = 0;
        virtual bool wait_for_driver_sync(const uint8_t* addresses, size_t count, uint32_t timeout_ms) = 0;
        virtual void report_driver_faults()                                = 0;
        virtual void report_driver_positions()                             = 0;
        virtual bool stop_all_drivers()                                    = 0;
        virtual void read_raw(uint8_t address, uint8_t function)           = 0;
        virtual bool run_auto_config()                                     = 0;
        virtual void report_driver_configs()                               = 0;
        virtual bool suspend()                                             = 0;
        virtual bool resume()                                              = 0;
        virtual bool control_ready() const                                 = 0;
    };
}

#endif
