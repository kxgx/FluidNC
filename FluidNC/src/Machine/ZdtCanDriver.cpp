// Copyright (c) 2026 - FluidNC contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "Config.h"

#if MAX_N_CAN

#    include "ZdtCanDriver.h"

#    include "CanDriverRegistry.h"
#    include "Motors/ZdtProtocol.h"
#    include "Report.h"

using namespace MotorDrivers;

namespace Machine {
    namespace {
        constexpr uint32_t TRANSACTION_TIMEOUT_MS = 300;

        CanDriver* createZdt(CanBus* bus) {
            return bus ? new ZdtCanDriver(bus) : nullptr;
        }

        CanDriverRegistration _zdtRegistration("zdt", createZdt);
    }

    bool ZdtCanDriver::init_driver() {
        if (!_bus || !_bus->ready()) {
            log_error("ZDT CAN: transport is not ready");
            return false;
        }
        return true;
    }

    bool ZdtCanDriver::transact(
        uint8_t address, const uint8_t* message, size_t length, uint8_t expectedFunction, uint8_t* response, size_t& responseLen) {
        responseLen = 0;
        if (!_bus || !_bus->ready() || !message || length < 2) {
            return false;
        }
        CanBus::Lock lock(*_bus);
        _bus->flush_receive();
        if (!_bus->send_chunked(uint32_t(address) << 8, message + 1, length - 1, TRANSACTION_TIMEOUT_MS)) {
            return false;
        }
        responseLen = _bus->receive_chunk(response, 16, TRANSACTION_TIMEOUT_MS);
        return ZdtProtocol::parse_ack(response, responseLen, expectedFunction);
    }

    bool ZdtCanDriver::read_counts(uint8_t address, int32_t& counts) {
        uint8_t request[8];
        size_t  length = ZdtProtocol::build_read_position(address, ZdtChecksum::Fixed6B, request, sizeof(request));
        if (!length) {
            return false;
        }

        uint8_t response[16];
        size_t  responseLen = 0;
        {
            CanBus::Lock lock(*_bus);
            _bus->flush_receive();
            if (!_bus->send_chunked(uint32_t(address) << 8, request + 1, length - 1, TRANSACTION_TIMEOUT_MS)) {
                return false;
            }
            responseLen = _bus->receive_chunk(response, sizeof(response), TRANSACTION_TIMEOUT_MS);
        }
        return ZdtProtocol::parse_position(ZdtFirmware::Emm, response, responseLen, 3200, counts);
    }

    int ZdtCanDriver::scan() {
        int found = 0;
        for (uint8_t address = 1; address <= CanBus::MAX_CLIENTS; ++address) {
            int32_t counts = 0;
            if (read_counts(address, counts)) {
                log_info("ZDT CAN: address " << int(address) << " position " << counts);
                ++found;
            }
        }
        log_info("ZDT CAN: scan found " << found << " driver(s)");
        return found;
    }

    int ZdtCanDriver::scan_all_bauds() {
        // Changing the bus bit rate while motors are configured is a separate
        // diagnostic operation; the ZDT driver uses the configured bus rate.
        return scan();
    }

    void ZdtCanDriver::report_status() {
        if (!_bus) {
            return;
        }
        uint8_t addresses[CanBus::MAX_CLIENTS];
        size_t  count = _bus->client_addresses(addresses, CanBus::MAX_CLIENTS);
        log_info("ZDT CAN: ready:" << (_bus->ready() ? "yes" : "no") << " control:" << (_bus->control_ready() ? "yes" : "no")
                                   << " clients:" << count);
    }

    void ZdtCanDriver::clear_driver_faults() {
        log_info("ZDT CAN: clear_faults is not implemented; power-cycle the driver if it is latched");
    }

    void ZdtCanDriver::restart_drivers() {
        log_info("ZDT CAN: restart is not implemented for safety");
    }

    void ZdtCanDriver::enable_all_driver_echo() {
        // The ZDT free protocol always answers the commands used here.
    }

    bool ZdtCanDriver::set_driver_enable(uint8_t address, bool en) {
        uint8_t request[8];
        size_t  length = ZdtProtocol::build_enable(address, en, ZdtChecksum::Fixed6B, request, sizeof(request));
        if (!length) {
            return false;
        }
        uint8_t response[16];
        size_t  responseLen = 0;
        return transact(address, request, length, 0xF3, response, responseLen);
    }

    bool ZdtCanDriver::send_single_position(uint8_t address, int32_t delta, uint16_t speed_rpm, uint8_t accel) {
        uint8_t request[ZdtProtocol::MAX_MESSAGE];
        size_t  length = ZdtProtocol::build_relative_position(
            ZdtFirmware::Emm, address, delta, speed_rpm, accel, 3200, ZdtChecksum::Fixed6B, request, sizeof(request));
        if (!length) {
            return false;
        }
        uint8_t response[16];
        size_t  responseLen = 0;
        bool    ok          = transact(address, request, length, 0xFD, response, responseLen);
        if (ok && _bus) {
            _bus->invalidate_client_position(address);
        }
        return ok;
    }

    bool ZdtCanDriver::set_group_address(uint8_t address, uint8_t group) {
        log_info("ZDT CAN: group addressing is not supported");
        return false;
    }

    size_t ZdtCanDriver::client_addresses(uint8_t* out, size_t max) const {
        return _bus ? _bus->client_addresses(out, max) : 0;
    }

    bool ZdtCanDriver::zero_driver_position(uint8_t address) {
        uint8_t request[8];
        size_t  length = ZdtProtocol::build_zero(address, ZdtChecksum::Fixed6B, request, sizeof(request));
        if (!length) {
            return false;
        }
        uint8_t response[16];
        size_t  responseLen = 0;
        bool    ok          = transact(address, request, length, 0x0A, response, responseLen);
        if (ok && _bus) {
            _bus->invalidate_client_position(address);
        }
        return ok;
    }

    bool ZdtCanDriver::wait_for_driver_sync(const uint8_t* addresses, size_t count, uint32_t timeout_ms) {
        // ZDT streaming is relative and does not keep a cached absolute
        // position base, so there is nothing to wait for here.
        (void)addresses;
        (void)count;
        (void)timeout_ms;
        return true;
    }

    void ZdtCanDriver::report_driver_faults() {
        log_info("ZDT CAN: fault reporting is not implemented");
    }

    void ZdtCanDriver::report_driver_positions() {
        for (uint8_t address = 1; address <= CanBus::MAX_CLIENTS; ++address) {
            int32_t counts = 0;
            if (read_counts(address, counts)) {
                log_info("ZDT CAN: address " << int(address) << " position " << counts);
            }
        }
    }

    bool ZdtCanDriver::stop_all_drivers() {
        if (!_bus) {
            return false;
        }
        uint8_t addresses[CanBus::MAX_CLIENTS];
        size_t  count = _bus->client_addresses(addresses, CanBus::MAX_CLIENTS);
        bool    ok    = true;
        for (size_t i = 0; i < count; ++i) {
            uint8_t request[8];
            size_t  length = ZdtProtocol::build_stop(addresses[i], ZdtChecksum::Fixed6B, request, sizeof(request));
            if (!length) {
                ok = false;
                continue;
            }
            uint8_t response[16];
            size_t  responseLen = 0;
            if (!transact(addresses[i], request, length, 0xFE, response, responseLen)) {
                can_log_error("ZDT CAN: stop failed for address " << int(addresses[i]));
                ok = false;
            }
        }
        return ok;
    }

    void ZdtCanDriver::read_raw(uint8_t address, uint8_t function) {
        uint8_t message[4] = { address, function, 0x6B, 0 };
        (void)message;
        log_info("ZDT CAN: raw read is not implemented");
    }

    bool ZdtCanDriver::run_auto_config() {
        return true;
    }

    void ZdtCanDriver::report_driver_configs() {
        log_info("ZDT CAN: configuration report is not implemented");
    }

    bool ZdtCanDriver::suspend() {
        return _bus ? _bus->suspend() : false;
    }

    bool ZdtCanDriver::resume() {
        return _bus ? _bus->resume() : false;
    }

    bool ZdtCanDriver::control_ready() const {
        return _bus && _bus->control_ready();
    }
}

#endif
