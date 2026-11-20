// Copyright (c) 2026 - FluidNC contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "Config.h"

#if MAX_N_CAN

#    include "CanDriverRegistry.h"
#    include "Logging.h"

namespace Machine {
    namespace {
        // Command-layer stub used to verify that can.driver actually selects a
        // registered implementation.  It deliberately owns no hardware and
        // never touches the motor path.
        class NullCanDriver final : public CanDriver {
        public:
            const char* driver_name() const override { return "null_can"; }
            CanProtocolClass protocol_class() const override { return CanProtocolClass::Private; }
            bool        uses_transport() const override { return false; }

            int scan() override {
                log_info("CAN scan: driver=null_can (stub): no devices");
                return 0;
            }
            int scan_all_bauds() override {
                log_info("CAN scan: driver=null_can (stub): no devices (all bauds)");
                return 0;
            }
            void report_status() override { log_info("CAN status: driver=null_can transport=none nodes=0"); }
            void clear_driver_faults() override {}
            void restart_drivers() override {}
            void enable_all_driver_echo() override {}
            bool set_driver_enable(uint8_t, bool) override { return false; }
            bool send_single_position(uint8_t, int32_t, uint16_t, uint8_t) override { return false; }
            bool set_group_address(uint8_t, uint8_t) override { return false; }
            size_t client_addresses(uint8_t*, size_t) const override { return 0; }
            bool zero_driver_position(uint8_t) override { return false; }
            bool wait_for_driver_sync(const uint8_t*, size_t count, uint32_t) override { return count == 0; }
            void report_driver_faults() override { log_info("CAN faults: driver=null_can: none"); }
            void report_driver_positions() override { log_info("CAN positions: driver=null_can: none"); }
            bool stop_all_drivers() override { return true; }
            void read_raw(uint8_t, uint8_t) override { log_info("CAN read: driver=null_can (stub)"); }
            bool run_auto_config() override { return true; }
            void report_driver_configs() override { log_info("CAN driver info: driver=null_can (stub)"); }
            bool suspend() override { return true; }
            bool resume() override { return true; }
            bool control_ready() const override { return true; }
        };

        CanDriverRegistration _nullCanRegistration("null_can", [](CanBus*) -> CanDriver* { return new NullCanDriver(); });
    }
}

#endif
