// Copyright (c) 2026 - FluidNC contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "Config.h"

#if MAX_N_CAN

#    include "Pd42s1Driver.h"

#    include "Can/CanDriverRegistry.h"
#    include "Can/Protocols/Private/Pd42s1/Pd42s1Protocol.h"
#    include "Logging.h"

#    include <Arduino.h>
#    include <esp_timer.h>

#    include <cstdio>
#    include <cstring>
#    include <limits>

using MotorDrivers::Pd42s1Protocol;

namespace Machine {
    namespace {
        // Diagnostic reports must tolerate a slow/late driver response.  The
        // worker still uses short timeouts, but a foreground report should not
        // silently drop a whole address because one reply took >50 ms.
        constexpr uint32_t DIAGNOSTIC_TIMEOUT_MS = 300;
        constexpr int      DIAGNOSTIC_ATTEMPTS   = 3;

        uint32_t now_ms() {
            return uint32_t(esp_timer_get_time() / 1000);
        }

        const char* motor_status_name(uint8_t status) {
            switch (status) {
                case 0:
                    return "stopped";
                case 1:
                    return "done";
                case 2:
                    return "running";
                case 3:
                    return "OVERLOAD";
                case 4:
                    return "STALL";
                case 5:
                    return "UNDERVOLTAGE";
                default:
                    return "unknown";
            }
        }

        CanDriver* createPd42s1(CanBus* bus) {
            return bus ? new Pd42s1Driver(bus) : nullptr;
        }

        CanDriverRegistration _pd42s1Registration("pd42s1", createPd42s1, true);
    }

    bool Pd42s1Driver::driver_ok(const char* operation) {
        if (!CanBus::driver_configured()) {
            can_log_info("CAN " << operation << ": no CAN motor configured; CAN disabled");
            return false;
        }
        return true;
    }

    bool Pd42s1Driver::init_driver() {
        if (!_bus) {
            return false;
        }
        // The PD42S1 manual requires a minimum gap between two complete
        // instructions; the transport implements the pacing.
        _bus->set_min_message_gap_ms(2);

        // The bus was already installed and the worker started by
        // CanBus::init().  Auto-config probes several bit rates, so it needs
        // exclusive control of the transport.
        const uint32_t configuredBaud = _bus->baud();
        if (!_bus->suspend()) {
            can_log_error("CAN: cannot suspend the bus for auto-config");
            return false;
        }
        if (!auto_configure_drivers()) {
            can_log_error("CAN: auto-config failed; CAN motor control stays disabled");
            // Leave the transport installed at the configured rate so serial
            // diagnostics still work without a worker.
            _bus->install_transport(configuredBaud);
            return false;
        }
        if (!_bus->resume()) {
            can_log_error("CAN: cannot restart the bus after auto-config");
            return false;
        }
        can_log_info("CAN: driver 'pd42s1' initialised at " << _bus->baud() << " baud");
        return true;
    }

    bool Pd42s1Driver::transact(
        const uint8_t* request, size_t requestLen, uint8_t* response, size_t responseCapacity, size_t& responseLen, uint32_t timeoutMs) {
        responseLen = 0;
        if (!_bus || !_bus->ready() || requestLen == 0 || !response || responseCapacity == 0) {
            return false;
        }

        _bus->flush_receive();
        if (!_bus->send_message(CAN_COMMAND_ID, request, requestLen, timeoutMs)) {
            _bus->flush_receive();
            _bus->flush_transmit();
            return false;
        }

        const uint32_t deadline = now_ms() + timeoutMs;
        size_t         total    = 0;
        while (total < responseCapacity) {
            int32_t remaining = int32_t(deadline - now_ms());
            if (remaining <= 0) {
                break;
            }
            size_t received = _bus->receive_chunk(response + total, responseCapacity - total, uint32_t(remaining));
            if (received == 0) {
                break;
            }
            total += received;

            size_t expected = Pd42s1Protocol::expected_response_length(response, total);
            if (expected == SIZE_MAX) {
                // Not a valid frame start.  Drop it and keep looking for the
                // response belonging to this request instead of flushing the
                // queue and potentially discarding a good late response.
                total = 0;
                continue;
            }
            if (expected != 0 && total >= expected) {
                if (total == expected && Pd42s1Protocol::valid_response(response, total) && response[1] == request[1] &&
                    response[2] == request[2]) {
                    if (total == 6 && Pd42s1Protocol::Status::is_error_code(response[3])) {
                        // Addressed command error (not a stale frame).
                        _bus->flush_receive();
                        _bus->flush_transmit();
                        return false;
                    }
                    responseLen = total;
                    return true;
                }
                // A complete but mismatched/stale frame: discard it and keep
                // waiting until the deadline for the real response.
                total = 0;
            }
        }

        _bus->flush_receive();
        _bus->flush_transmit();
        return false;
    }

    bool Pd42s1Driver::probe_address_quiet(uint8_t address, uint8_t& firmware, uint8_t& hardware, uint32_t timeout_ms, int attempts) {
        uint8_t request[Pd42s1Protocol::MAX_FRAME_LEN];
        uint8_t response[Pd42s1Protocol::MAX_FRAME_LEN];
        size_t  len = Pd42s1Protocol::build_read_command(address, Pd42s1Protocol::FUNC_READ_VERSION, request, sizeof(request));
        for (int attempt = 0; attempt < attempts; ++attempt) {
            size_t responseLen = 0;
            if (transact(request, len, response, sizeof(response), responseLen, timeout_ms) &&
                Pd42s1Protocol::parse_version(response, responseLen, firmware, hardware)) {
                return true;
            }
            vTaskDelay(pdMS_TO_TICKS(5));
        }
        return false;
    }

    void Pd42s1Driver::set_echo_to_address_locked(uint8_t address, bool enable) {
        uint8_t request[Pd42s1Protocol::MAX_FRAME_LEN];
        uint8_t echo = enable ? 0 : 1;
        size_t  len  = Pd42s1Protocol::build_frame(address, Pd42s1Protocol::FUNC_SET_ECHO, &echo, 1, request, sizeof(request));
        if (!len) {
            return;
        }
        _bus->send_message(CAN_COMMAND_ID, request, len, 50);
        vTaskDelay(pdMS_TO_TICKS(2));
        _bus->flush_receive();
    }

    size_t Pd42s1Driver::target_addresses(uint8_t* out, size_t max) const {
        size_t count = _bus ? CanBus::configured_addresses(out, max) : 0;
        if (!count) {
            // Discovery fallback when no CAN motor was configured.
            for (size_t i = 0; i < max; ++i) {
                out[count++] = uint8_t(i + 1);
            }
        }
        return count;
    }

    void Pd42s1Driver::disable_all_driver_echo_locked() {
        // Broadcast first, then each address; the addressed calls also drain
        // the acknowledgment from drivers that were still echoing.
        send_echo_state_forced(false);
        vTaskDelay(pdMS_TO_TICKS(5));
        uint8_t addresses[CanBus::MAX_CLIENTS];
        size_t  count = target_addresses(addresses, CanBus::MAX_CLIENTS);
        for (size_t i = 0; i < count; ++i) {
            set_echo_to_address_locked(addresses[i], false);
        }
        _bus->flush_receive();
        _echo_state = false;
        can_log_info("CAN: command echo disabled on the drivers after auto-config");
    }

    // Switch one driver to a new bit rate and prove it answers there.  A
    // single fire-and-forget 0x68 frame is not sufficient: if that frame is
    // lost the driver stays at the old rate while the controller moves on,
    // which leaves a mixed-rate bus that is hard to recover.  Retry by going
    // back to the known-old rate, re-sending the switch, and probing again.
    bool Pd42s1Driver::switch_driver_baud_retry(uint8_t address, uint32_t oldBaud, uint32_t newBaud) {
        if (oldBaud == newBaud) {
            uint8_t firmware = 0;
            uint8_t hardware = 0;
            return probe_address_quiet(address, firmware, hardware, 150, 3);
        }
        uint8_t request[Pd42s1Protocol::MAX_FRAME_LEN];
        size_t  len = Pd42s1Protocol::build_set_can_baud_command(address, uint16_t(newBaud / 1000), request, sizeof(request));
        if (!len) {
            return false;
        }
        for (int attempt = 0; attempt < 3; ++attempt) {
            _bus->uninstall_transport();
            if (!_bus->install_transport(oldBaud)) {
                return false;
            }
            _bus->send_message(CAN_COMMAND_ID, request, len, 100);
            vTaskDelay(pdMS_TO_TICKS(40));

            _bus->uninstall_transport();
            if (!_bus->install_transport(newBaud)) {
                return false;
            }
            uint8_t firmware = 0;
            uint8_t hardware = 0;
            if (probe_address_quiet(address, firmware, hardware, 150, 3)) {
                log_info("CAN: address " << int(address) << " verified at " << newBaud << " baud after retry " << attempt + 1);
                return true;
            }
            log_info("CAN: address " << int(address) << " did not answer at " << newBaud << " baud, retry " << attempt + 1);
            vTaskDelay(pdMS_TO_TICKS(20));
        }
        return false;
    }

    bool Pd42s1Driver::auto_configure_drivers() {
        const uint32_t target = _bus->baud();

        // This routine must never broadcast a baud-rate or save command.  A
        // broadcast has no acknowledgement, so one driver that misses the
        // frame can be left at the wrong rate with its echo disabled.  Probe
        // at every supported rate, then write each driver at its own address
        // and verify every write.
        log_info("CAN auto-config: target " << target << " baud; probing all rates");
        can_log_info("CAN: auto-config: target " << target << " baud; probing all rates, addressed writes only");

        int expected = CanBus::configured_driver_count();
        if (expected <= 0) {
            expected = 1;  // discovery fallback: at least one driver expected
        }

        struct Found {
            uint8_t  address;
            uint32_t rate;
            uint8_t  firmware;
            uint8_t  hardware;
        };
        Found   found[CanBus::MAX_CLIENTS];
        int     foundCount = 0;
        uint8_t addresses[CanBus::MAX_CLIENTS];
        size_t  addressCount = target_addresses(addresses, CanBus::MAX_CLIENTS);

        // Phase 1: discover drivers at all rates and move each one to the
        // target rate using an addressed 0x68 command.  Echo is enabled at
        // each configured address first so a driver that previously had echo
        // disabled can answer version probes.
        for (uint16_t kbps : Pd42s1Protocol::Limits::SUPPORTED_BAUD_KBPS) {
            const uint32_t rate = uint32_t(kbps) * 1000u;
            if (!_bus->install_transport(rate)) {
                continue;
            }

            for (size_t i = 0; i < addressCount; ++i) {
                enable_driver_echo_locked(addresses[i]);
            }
            vTaskDelay(pdMS_TO_TICKS(5));

            int foundAtRate = 0;
            for (size_t i = 0; i < addressCount; ++i) {
                uint8_t address  = addresses[i];
                uint8_t firmware = 0;
                uint8_t hardware = 0;
                if (probe_address_quiet(address, firmware, hardware, 80, 3)) {
                    ++foundAtRate;
                    if (foundCount < int(sizeof(found) / sizeof(found[0]))) {
                        found[foundCount++] = { address, rate, firmware, hardware };
                    }
                    can_log_info("CAN: found address " << int(address) << " at " << rate << " baud (fw V" << int(firmware / 10) << "."
                                                       << int(firmware % 10) << " hw V" << int(hardware / 10) << "."
                                                       << int(hardware % 10) << ")");
                }
            }

            if (rate != target && foundAtRate) {
                for (int i = 0; i < foundCount; ++i) {
                    if (found[i].rate != rate) {
                        continue;
                    }
                    uint8_t request[Pd42s1Protocol::MAX_FRAME_LEN];
                    size_t  len =
                        Pd42s1Protocol::build_set_can_baud_command(found[i].address, uint16_t(target / 1000), request, sizeof(request));
                    if (len) {
                        can_log_info("CAN: address " << int(found[i].address) << " moving " << rate << " -> " << target << " baud");
                        // A driver changes its bit rate as soon as it accepts
                        // this frame.  Sending it twice would put the second
                        // frame on the old rate after the driver has already
                        // switched, so only send once.
                        _bus->send_message(CAN_COMMAND_ID, request, len, 100);
                        vTaskDelay(pdMS_TO_TICKS(20));
                    }
                }
            }

            _bus->uninstall_transport();
        }

        // Phase 2: install the target rate, save each driver individually and
        // read the saved configuration back to prove the change survived.
        if (!_bus->install_transport(target)) {
            can_log_error("CAN: auto-config: cannot install target rate");
            return false;
        }

        for (size_t i = 0; i < addressCount; ++i) {
            enable_driver_echo_locked(addresses[i]);
        }
        _echo_state = true;
        vTaskDelay(pdMS_TO_TICKS(10));

        int  confirmed = 0;
        int  savedOk   = 0;
        bool verified[CanBus::MAX_CLIENTS + 1] = { false };
        // Verify exactly the IDs that phase 1 discovered, not an arbitrary
        // address range.  A recorded ID may have appeared at more than one
        // probe rate, so process each address once.
        for (int foundIndex = 0; foundIndex < foundCount; ++foundIndex) {
            const uint8_t address = found[foundIndex].address;
            bool          already = false;
            for (int j = 0; j < foundIndex; ++j) {
                if (found[j].address == address) {
                    already = true;
                    break;
                }
            }
            if (already) {
                continue;
            }

            uint8_t firmware = 0;
            uint8_t hardware = 0;
            if (!probe_address_quiet(address, firmware, hardware, 100, 3)) {
                can_log_warn("CAN: recorded address " << int(address) << " did not answer at the target rate");
                continue;
            }
            verified[address] = true;
            ++confirmed;

            uint8_t  currentEcho = 0;
            uint8_t  currentMode = 0;
            uint16_t currentBaud = 0;
            bool     needSave    = true;
            if (read_driver_config(address, currentBaud, currentEcho, currentMode) && currentBaud == uint16_t(target / 1000)) {
                // Already saved correctly: avoid unnecessary NVRAM writes.
                needSave = false;
            }

            if (needSave) {
                uint8_t request[Pd42s1Protocol::MAX_FRAME_LEN];
                uint8_t response[64];
                size_t  len   = Pd42s1Protocol::build_save_params_command(address, request, sizeof(request));
                bool    saved = false;
                for (int attempt = 0; attempt < 2 && !saved; ++attempt) {
                    size_t responseLen = 0;
                    if (len && transact(request, len, response, sizeof(response), responseLen, 100) &&
                        Pd42s1Protocol::parse_ack(response, responseLen)) {
                        saved = true;
                    }
                }
                if (!saved) {
                    can_log_warn("CAN: address " << int(address) << " did not acknowledge parameter save");
                }
            }

            uint16_t savedBaud = 0;
            if (read_driver_config(address, savedBaud, currentEcho, currentMode) && savedBaud == uint16_t(target / 1000)) {
                ++savedOk;
                can_log_info("CAN: address " << int(address) << " saved CAN baud=" << savedBaud << " kbps");
            } else {
                can_log_warn("CAN: address " << int(address) << " saved CAN baud readback=" << savedBaud << " kbps (expected "
                                             << uint16_t(target / 1000) << ")");
            }
        }

        // Phase 3: any driver that was discovered but did not answer at the
        // target rate gets a bounded retry from its recorded old rate.
        for (int i = 0; i < foundCount; ++i) {
            const uint8_t address = found[i].address;
            if (address < 1 || address > CanBus::MAX_CLIENTS || verified[address]) {
                continue;
            }
            if (found[i].rate == target) {
                continue;
            }
            log_info("CAN auto-config: retrying address " << int(address) << " from " << found[i].rate << " to " << target);
            if (!switch_driver_baud_retry(address, found[i].rate, target)) {
                log_warn("CAN auto-config: address " << int(address) << " did not switch to " << target << " baud");
                continue;
            }
            verified[address] = true;
            ++confirmed;

            uint8_t request[Pd42s1Protocol::MAX_FRAME_LEN];
            uint8_t response[64];
            size_t  len   = Pd42s1Protocol::build_save_params_command(address, request, sizeof(request));
            bool    saved = false;
            for (int attempt = 0; attempt < 2 && !saved; ++attempt) {
                size_t responseLen = 0;
                if (len && transact(request, len, response, sizeof(response), responseLen, 100) &&
                    Pd42s1Protocol::parse_ack(response, responseLen)) {
                    saved = true;
                }
            }
            uint8_t  echo      = 0;
            uint8_t  mode      = 0;
            uint16_t savedBaud = 0;
            if (read_driver_config(address, savedBaud, echo, mode) && savedBaud == uint16_t(target / 1000)) {
                ++savedOk;
                log_info("CAN auto-config: address " << int(address) << " saved CAN baud=" << savedBaud << " kbps");
            } else {
                log_warn("CAN auto-config: address " << int(address) << " saved baud readback mismatch");
            }
        }

        log_info("CAN auto-config: configured=" << CanBus::configured_driver_count() << " discovered=" << foundCount << " confirmed=" << confirmed
                                                << " saved=" << savedOk << " at " << target << " baud");
        can_log_info("CAN: auto-config: configured=" << CanBus::configured_driver_count() << " discovered=" << foundCount
                                                     << " confirmed=" << confirmed << " saved=" << savedOk << " at " << target << " baud");

        for (int i = 0; i < foundCount; ++i) {
            uint8_t firmware = 0;
            uint8_t hardware = 0;
            if (found[i].rate != target && !probe_address_quiet(found[i].address, firmware, hardware, 100, 2)) {
                can_log_warn("CAN: address " << int(found[i].address) << " was found at " << found[i].rate << " baud but did not answer at "
                                             << target << " baud");
            }
        }

        if (expected > 0 && (confirmed < expected || savedOk < confirmed)) {
            log_warn("CAN auto-config incomplete: expected " << expected << " confirmed " << confirmed << " saved " << savedOk
                                                             << "; CAN control worker will stay stopped for safety");
            can_log_warn("CAN: " << (expected - confirmed) << " configured driver(s) did not answer at the configured baud; command echo is left ENABLED");
            // Do not disable echo when the system is incomplete: the drivers
            // must remain reachable for recovery without a power cycle.
            return false;
        }

        // All configured drivers answered and saved the new rate.  Disable
        // echo by address rather than broadcast so a missed frame cannot hide
        // a driver at the wrong setting.
        for (size_t i = 0; i < addressCount; ++i) {
            set_echo_to_address_locked(addresses[i], false);
        }
        _echo_state = false;
        can_log_info("CAN: command echo disabled on the drivers after auto-config");
        return true;
    }

    bool Pd42s1Driver::run_auto_config() {
        if (!driver_ok("auto-config")) {
            return false;
        }
        if (!_bus || !_bus->ready()) {
            can_log_info("CAN auto-config: bus not ready");
            return false;
        }
        const uint32_t configuredBaud = _bus->baud();
        const bool     stopWasLatched = _bus->stop_latched();
        can_log_info("CAN: starting manual auto-config (target " << configuredBaud << " baud)");
        if (!_bus->suspend()) {
            return false;
        }
        bool ok = auto_configure_drivers();
        if (!ok) {
            log_error("CAN auto-config failed or incomplete; CAN motor worker is not started");
            can_log_error("CAN: auto-config failed");
            _bus->set_control_ready(false);
            _bus->install_transport(configuredBaud);  // serial diagnostics may still use the installed bus
            // The drivers may still be enabled from before the failed
            // reconfiguration.  The worker is stopped, so the normal
            // apply_disable() path cannot reach them; perform a direct,
            // readback-verified stop now.
            _bus->invalidate_all_client_positions();
            if (!stop_all_drivers()) {
                can_log_error("CAN: failed auto-config left at least one driver enabled");
            }
            // stop_all_drivers() latches the stop to protect the restart path.
            // A failed auto-config never reaches that restart path; leaving the
            // latch set would make a later successful retry report "OK" while
            // the worker silently refuses all motion.  Preserve a latch that
            // the operator had already set with $CANStop.
            if (!stopWasLatched) {
                _bus->clear_stop_latch();
            }
            return false;
        }
        // Do not clear a latched $CANStop here; only $CANReset may release it.
        // Runtime reconfiguration closes command echo and may have reset the
        // driver parameter state.  Force every client through a fresh sync so
        // it cannot keep using an old position base.  This must happen before
        // the worker is started again.
        _bus->invalidate_all_client_positions();
        // Allow the recovery worker to start, then resume the transport.
        // start_worker() intentionally refuses to start when control_ready is
        // false; setting it first is what makes a successful manual
        // $CANAutoConfig able to recover from a previous failure.
        _bus->set_control_ready(true);
        if (!_bus->resume()) {
            _bus->set_control_ready(false);
            return false;
        }
        log_info("CAN auto-config finished OK");
        can_log_info("CAN: auto-config finished");
        return true;
    }

    int Pd42s1Driver::scan_address(uint8_t address, uint32_t timeout_ms) {
        uint8_t request[Pd42s1Protocol::MAX_FRAME_LEN];
        uint8_t response[Pd42s1Protocol::MAX_FRAME_LEN];
        size_t  requestLen = Pd42s1Protocol::build_read_command(address, Pd42s1Protocol::FUNC_READ_VERSION, request, sizeof(request));

        // A marginal bus can lose a response occasionally, so give every
        // address a few tries before declaring it absent.
        constexpr int MAX_ATTEMPTS = 3;
        for (int attempt = 1; attempt <= MAX_ATTEMPTS; ++attempt) {
            size_t  responseLen     = 0;
            uint8_t firmwareVersion = 0;
            uint8_t hardwareVersion = 0;
            if (transact(request, requestLen, response, sizeof(response), responseLen, timeout_ms) &&
                Pd42s1Protocol::parse_version(response, responseLen, firmwareVersion, hardwareVersion)) {
                log_info("  CAN device at address " << int(address) << " firmware V" << int(firmwareVersion / 10) << "."
                                                    << int(firmwareVersion % 10) << " hardware V" << int(hardwareVersion / 10) << "."
                                                    << int(hardwareVersion % 10) << (attempt > 1 ? " (after retry)" : ""));
                return 1;
            }
            vTaskDelay(pdMS_TO_TICKS(5));
        }
        return 0;
    }

    int Pd42s1Driver::scan_addresses() {
        int found = 0;
        for (uint8_t address = 1; address <= CanBus::MAX_CLIENTS; ++address) {
            found += scan_address(address, 100);
        }
        if (!found) {
            can_log_info("  (no devices)");
        }
        return found;
    }

    bool Pd42s1Driver::test_transceiver() {
        // The transceiver has no status output that the controller could read,
        // but it does loop the bus state back on its R pin: driving D dominant
        // (TX low) must make R dominant (RX low), and driving D recessive (TX
        // high) must make R recessive (RX high).  This test temporarily gives
        // the TX/RX pins back to the GPIO controller, so the TWAI driver and
        // worker are suspended for the duration.
        if (!_bus || !_bus->ready()) {
            log_info("CAN transceiver test: bus not ready");
            return false;
        }

        auto tx_pin = _bus->tx_pin().getNative(Pin::Capabilities::Native | Pin::Capabilities::Output);
        auto rx_pin = _bus->rx_pin().getNative(Pin::Capabilities::Native | Pin::Capabilities::Input);

        if (!_bus->suspend()) {
            return false;
        }

        pinMode(rx_pin, INPUT_PULLUP);
        pinMode(tx_pin, OUTPUT);

        digitalWrite(tx_pin, 1);  // recessive
        delay(2);
        int rx_recessive = digitalRead(rx_pin);

        digitalWrite(tx_pin, 0);  // dominant
        delay(2);
        int rx_dominant = digitalRead(rx_pin);

        digitalWrite(tx_pin, 1);  // back to recessive
        delay(2);

        log_info("CAN transceiver test: D=recessive -> R=" << rx_recessive << ", D=dominant -> R=" << rx_dominant);
        bool present = (rx_recessive == 1 && rx_dominant == 0);
        if (present) {
            log_info("CAN transceiver: detected (D/R loopback OK)");
        } else {
            log_error("CAN transceiver: NOT detected or not responding");
        }

        pinMode(tx_pin, INPUT_PULLUP);
        pinMode(rx_pin, INPUT_PULLUP);
        delay(10);
        _bus->resume();
        return present;
    }

    int Pd42s1Driver::scan() {
        if (!driver_ok("scan")) {
            return 0;
        }
        if (!_bus || !_bus->ready()) {
            log_info("CAN scan: bus not ready");
            return 0;
        }

        // Check the physical layer first: this also restarts the bus and the
        // worker when it is done.
        test_transceiver();
        if (!_bus->ready()) {
            log_info("CAN scan: bus not ready");
            return 0;
        }

        int found = 0;
        {
            CanBus::Lock lock(*_bus);

            CanBus::BusStatus status;
            if (_bus->bus_status(status)) {
                log_info("CAN scan: state=" << status.state << " tx_err=" << status.tx_error << " rx_err=" << status.rx_error
                                            << " bus_errors=" << status.bus_errors);
            }

            // Discard any frames that are still queued from earlier transactions.
            _bus->flush_receive();

            uint32_t tecBefore = _bus->tx_error_counter();
            found              = scan_addresses();
            uint32_t tecAfter  = _bus->tx_error_counter();
            can_log_info("  TWAI tx_err " << tecBefore << " -> " << tecAfter
                                          << (tecAfter == 0 ? " (frames were ACKed by a node)"
                                                            : " (no ACK - check driver power, wiring and bit rate)"));
        }

        return found;
    }

    int Pd42s1Driver::scan_all_bauds() {
        if (!driver_ok("scan")) {
            return 0;
        }
        if (!_bus || !_bus->ready()) {
            log_info("CAN scan: bus not ready");
            return 0;
        }

        // Transceiver detection works with GPIOs and is independent of the bit
        // rate, so run it once at the configured rate before changing anything.
        test_transceiver();
        if (!_bus->ready()) {
            log_info("CAN scan: bus not ready");
            return 0;
        }

        const uint32_t configured = _bus->baud();
        int            total      = 0;

        // Stop the worker and release the bus; no client transaction may run
        // while the driver is reinstalled at the different bit rates.
        if (!_bus->suspend()) {
            return 0;
        }

        {
            CanBus::Lock lock(*_bus);
            for (uint16_t kbps : Pd42s1Protocol::Limits::SUPPORTED_BAUD_KBPS) {
                const uint32_t baud = uint32_t(kbps) * 1000u;
                if (!_bus->install_transport(baud)) {
                    log_error("CAN scan: cannot install driver at " << baud << " baud");
                    continue;
                }
                log_info("CAN scan at " << baud << " baud:");
                _bus->flush_receive();
                uint32_t tecBefore = _bus->tx_error_counter();
                // The multi-baud recovery scan explicitly enables command
                // echo on every address before reading; the plain $CANScan
                // stays faithful to the vendor example (no 0x6F commands).
                log_info("CAN scan: enabling echo on configured addresses");
                enable_all_driver_echo_locked();
                vTaskDelay(pdMS_TO_TICKS(5));
                total += scan_addresses();
                uint32_t tecAfter = _bus->tx_error_counter();
                can_log_info("  TWAI tx_err " << tecBefore << " -> " << tecAfter
                                              << (tecAfter == 0 ? " (frames were ACKed by a node)"
                                                                : " (no ACK - check driver power, wiring and bit rate)"));
                _bus->uninstall_transport();
            }
        }

        // The probe loop above leaves the transport uninstalled with the last
        // probed bit rate selected, so restore the configured rate before the
        // worker is restarted.
        if (!_bus->install_transport(configured) || !_bus->resume()) {
            can_log_error("CAN: cannot restore the bus after the baud scan");
            return total;
        }
        {
            CanBus::Lock lock(*_bus);
            // Normal motor control requires command echo to be enabled.  The
            // old code restored the cached pre-scan state, but the motors open
            // echo by themselves during sync; that cache is stale and could
            // silence every driver after a scan.  Always restore the normal
            // control state and force a fresh client sync.
            enable_all_driver_echo_locked();
            _echo_state = true;
        }
        _bus->invalidate_all_client_positions();
        log_info("CAN scan: restored " << _bus->baud() << " baud and command echo enabled");
        return total;
    }

    void Pd42s1Driver::report_status() {
        if (!driver_ok("status")) {
            return;
        }
        if (!_bus || !_bus->ready()) {
            log_info("CAN status: bus not ready");
            return;
        }
        CanBus::Lock lock(*_bus);

        CanBus::BusStatus status;
        if (!_bus->bus_status(status)) {
            log_error("CAN status: cannot read TWAI status");
            return;
        }
        log_info("CAN status: state=" << status.state << " tx_err=" << status.tx_error << " rx_err=" << status.rx_error
                                      << " bus_errors=" << status.bus_errors << " tx_queue=" << status.msgs_to_tx
                                      << " rx_queue=" << status.msgs_to_rx << " bus_off_count=" << status.bus_off_count);
        log_info("CAN config: tx=" << _bus->tx_pin().name() << " rx=" << _bus->rx_pin().name() << " baud=" << _bus->baud()
                                   << " driver=pd42s1 command_id=" << CAN_COMMAND_ID
                                   << " echoes=" << (_echo_state ? "on" : "off"));
        if (_bus->inhibited()) {
            can_log_error("CAN control inhibited: " << (_bus->inhibit_reason() ? _bus->inhibit_reason() : "unknown reason"));
        }
    }

    void Pd42s1Driver::clear_driver_faults() {
        if (!driver_ok("clear")) {
            return;
        }
        if (!_bus || !_bus->ready()) {
            log_info("CAN clear: bus not ready");
            return;
        }
        CanBus::Lock lock(*_bus);

        uint8_t request[Pd42s1Protocol::MAX_FRAME_LEN];
        uint8_t response[Pd42s1Protocol::MAX_FRAME_LEN];

        // 0xF9 releases the stall protection.  Unlike 0xFB it does not clear
        // the enable/brake state; in testing on firmware V1.6 a broadcast
        // 0xFB made all four drivers stop answering until they were power
        // cycled, so this command deliberately avoids 0xFB and broadcast.
        uint8_t addresses[CanBus::MAX_CLIENTS];
        size_t  count = target_addresses(addresses, CanBus::MAX_CLIENTS);
        log_info("CAN: releasing stall protection (0xF9) on configured addresses");
        int acked = 0;
        for (size_t i = 0; i < count; ++i) {
            uint8_t address = addresses[i];
            size_t  len     = Pd42s1Protocol::build_clear_stall_command(address, request, sizeof(request));
            for (int attempt = 0; attempt < 2; ++attempt) {
                size_t responseLen = 0;
                if (transact(request, len, response, sizeof(response), responseLen, 50) &&
                    Pd42s1Protocol::parse_ack(response, responseLen)) {
                    acked++;
                    break;
                }
            }
        }
        log_info("CAN: 0xF9 stall-release acknowledged by " << acked << " address(es)");
    }

    void Pd42s1Driver::report_driver_faults() {
        if (!driver_ok("faults")) {
            return;
        }
        if (!_bus || !_bus->ready()) {
            log_info("CAN faults: bus not ready");
            return;
        }
        _bus->begin_diagnostic();
        CanBus::Lock lock(*_bus);

        auto read_byte = [this](uint8_t address, uint8_t function, uint8_t& value) {
            uint8_t request[Pd42s1Protocol::MAX_FRAME_LEN];
            uint8_t response[Pd42s1Protocol::MAX_FRAME_LEN];
            size_t  len = Pd42s1Protocol::build_read_command(address, function, request, sizeof(request));
            for (int attempt = 0; attempt < DIAGNOSTIC_ATTEMPTS; ++attempt) {
                size_t responseLen = 0;
                if (transact(request, len, response, sizeof(response), responseLen, DIAGNOSTIC_TIMEOUT_MS) &&
                    Pd42s1Protocol::parse_byte_value(response, responseLen, function, value)) {
                    return true;
                }
            }
            return false;
        };

        int found    = 0;
        int abnormal = 0;
        for (uint8_t address = 1; address <= CanBus::MAX_CLIENTS; ++address) {
            if (!_bus->has_client(address)) {
                continue;
            }
            uint8_t request[Pd42s1Protocol::MAX_FRAME_LEN];
            uint8_t response[Pd42s1Protocol::MAX_FRAME_LEN];
            size_t  len = Pd42s1Protocol::build_read_command(address, Pd42s1Protocol::FUNC_READ_VERSION, request, sizeof(request));

            uint8_t firmware = 0;
            uint8_t hardware = 0;
            bool    have     = false;
            for (int attempt = 0; attempt < DIAGNOSTIC_ATTEMPTS && !have; ++attempt) {
                size_t responseLen = 0;
                have = transact(request, len, response, sizeof(response), responseLen, DIAGNOSTIC_TIMEOUT_MS) &&
                       Pd42s1Protocol::parse_version(response, responseLen, firmware, hardware);
            }
            if (!have) {
                log_warn("  address " << int(address) << ": no response (version read)");
                continue;
            }
            found++;

            uint8_t status = 0xFF;
            uint8_t stall  = 0xFF;
            uint8_t enable = 0xFF;
            uint8_t inpos  = 0xFF;
            read_byte(address, Pd42s1Protocol::FUNC_READ_STATUS, status);
            read_byte(address, Pd42s1Protocol::FUNC_READ_STALL, stall);
            read_byte(address, Pd42s1Protocol::FUNC_READ_ENABLE, enable);
            read_byte(address, Pd42s1Protocol::FUNC_READ_IN_POSITION, inpos);

            log_info("  address " << int(address) << ": fw V" << int(firmware / 10) << "." << int(firmware % 10) << " hw V"
                                  << int(hardware / 10) << "." << int(hardware % 10) << " status=" << motor_status_name(status)
                                  << " stall=" << (stall == 0xFF ? "?" : (stall ? "YES" : "no"))
                                  << " enabled=" << (enable == 0xFF ? "?" : (enable ? "no" : "yes"))
                                  << " in-position=" << (inpos == 0xFF ? "?" : (inpos ? "yes" : "no")));
            if ((status != 0xFF && status >= 3) || stall == 1) {
                abnormal++;
            }
        }

        _bus->end_diagnostic();
        if (!found) {
            log_info("CAN faults: (no devices)");
        } else if (abnormal) {
            log_warn("CAN faults: " << abnormal << " of " << found << " driver(s) abnormal");
        } else {
            log_info("CAN faults: no abnormal drivers");
        }
    }

    bool Pd42s1Driver::read_driver_config(uint8_t address, uint16_t& can_baud_kbps, uint8_t& echo, uint8_t& mode) {
        uint8_t request[Pd42s1Protocol::MAX_FRAME_LEN];
        uint8_t response[Pd42s1Protocol::MAX_FRAME_LEN];
        size_t  len = Pd42s1Protocol::build_read_command(address, Pd42s1Protocol::FUNC_READ_DRIVER_PARAMS, request, sizeof(request));
        for (int attempt = 0; attempt < 3; ++attempt) {
            size_t responseLen = 0;
            if (transact(request, len, response, sizeof(response), responseLen, Pd42s1Protocol::Timing::TRANSACTION_TIMEOUT_MS) && responseLen == Pd42s1Protocol::RESP_LEN_DRIVER_PARAMS && response[2] == Pd42s1Protocol::FUNC_READ_DRIVER_PARAMS &&
                response[3] == Pd42s1Protocol::Status::OK) {
                mode          = response[4];
                echo          = response[5];
                can_baud_kbps = uint16_t((uint16_t(response[10]) << 8) | response[11]);
                return true;
            }
            vTaskDelay(pdMS_TO_TICKS(5));
        }
        return false;
    }

    void Pd42s1Driver::report_driver_configs() {
        if (!driver_ok("config")) {
            return;
        }
        if (!_bus || !_bus->ready()) {
            log_info("CAN config: bus not ready");
            return;
        }
        _bus->begin_diagnostic();
        CanBus::Lock lock(*_bus);
        int found = 0;
        for (uint8_t address = 1; address <= CanBus::MAX_CLIENTS; ++address) {
            if (!_bus->has_client(address)) {
                continue;
            }
            uint16_t baud = 0;
            uint8_t  echo = 0;
            uint8_t  mode = 0;
            if (read_driver_config(address, baud, echo, mode)) {
                found++;
                log_info("  address " << int(address) << ": mode=" << int(mode) << " echo=" << int(echo) << " saved-CAN-baud=" << baud
                                      << " kbps");
            } else {
                log_warn("  address " << int(address) << ": no response (config read)");
            }
        }
        _bus->end_diagnostic();
        if (!found) {
            log_info("CAN config: (no devices)");
        }
    }

    bool Pd42s1Driver::read_driver_position(uint8_t address, int32_t& position) {
        if (!_bus || !_bus->ready()) {
            return false;
        }
        uint8_t request[Pd42s1Protocol::MAX_FRAME_LEN];
        uint8_t response[16];
        size_t  responseLen = 0;
        size_t  len         = Pd42s1Protocol::build_read_command(address, Pd42s1Protocol::FUNC_READ_POSITION, request, sizeof(request));
        if (!len) {
            return false;
        }
        CanBus::Lock lock(*_bus);
        return transact(request, len, response, sizeof(response), responseLen, 150) &&
               Pd42s1Protocol::parse_position(response, responseLen, position);
    }

    void Pd42s1Driver::report_driver_positions() {
        if (!driver_ok("positions")) {
            return;
        }
        if (!_bus || !_bus->ready()) {
            log_info("CAN positions: bus not ready");
            return;
        }
        _bus->begin_diagnostic();
        CanBus::Lock lock(*_bus);

        int found = 0;
        for (uint8_t address = 1; address <= CanBus::MAX_CLIENTS; ++address) {
            if (!_bus->has_client(address)) {
                continue;
            }
            uint8_t request[Pd42s1Protocol::MAX_FRAME_LEN];
            uint8_t response[Pd42s1Protocol::MAX_FRAME_LEN];
            size_t  len = Pd42s1Protocol::build_read_command(address, Pd42s1Protocol::FUNC_READ_VERSION, request, sizeof(request));

            uint8_t firmware = 0;
            uint8_t hardware = 0;
            bool    have     = false;
            for (int attempt = 0; attempt < DIAGNOSTIC_ATTEMPTS && !have; ++attempt) {
                size_t responseLen = 0;
                have = transact(request, len, response, sizeof(response), responseLen, DIAGNOSTIC_TIMEOUT_MS) &&
                       Pd42s1Protocol::parse_version(response, responseLen, firmware, hardware);
            }
            if (!have) {
                log_warn("  address " << int(address) << ": no response (version read)");
                continue;
            }
            found++;

            int32_t position = 0;
            int32_t error    = 0;
            int16_t speed    = 0;
            uint8_t inpos    = 0xFF;
            bool    hasPos   = false;
            bool    hasError = false;
            bool    hasSpeed = false;

            for (int attempt = 0; attempt < DIAGNOSTIC_ATTEMPTS && !hasPos; ++attempt) {
                size_t responseLen = 0;
                len = Pd42s1Protocol::build_read_command(address, Pd42s1Protocol::FUNC_READ_POSITION, request, sizeof(request));
                hasPos = transact(request, len, response, sizeof(response), responseLen, DIAGNOSTIC_TIMEOUT_MS) &&
                         Pd42s1Protocol::parse_int32_value(response, responseLen, Pd42s1Protocol::FUNC_READ_POSITION, position);
            }
            for (int attempt = 0; attempt < DIAGNOSTIC_ATTEMPTS && !hasError; ++attempt) {
                size_t responseLen = 0;
                len = Pd42s1Protocol::build_read_command(address, Pd42s1Protocol::FUNC_READ_POS_ERROR, request, sizeof(request));
                hasError = transact(request, len, response, sizeof(response), responseLen, DIAGNOSTIC_TIMEOUT_MS) &&
                           Pd42s1Protocol::parse_int32_value(response, responseLen, Pd42s1Protocol::FUNC_READ_POS_ERROR, error);
            }
            for (int attempt = 0; attempt < DIAGNOSTIC_ATTEMPTS && !hasSpeed; ++attempt) {
                size_t responseLen = 0;
                len = Pd42s1Protocol::build_read_command(address, Pd42s1Protocol::FUNC_READ_SPEED, request, sizeof(request));
                hasSpeed = transact(request, len, response, sizeof(response), responseLen, DIAGNOSTIC_TIMEOUT_MS) &&
                           Pd42s1Protocol::parse_int16_value(response, responseLen, Pd42s1Protocol::FUNC_READ_SPEED, speed);
            }
            for (int attempt = 0; attempt < DIAGNOSTIC_ATTEMPTS && inpos == 0xFF; ++attempt) {
                size_t responseLen = 0;
                len = Pd42s1Protocol::build_read_command(address, Pd42s1Protocol::FUNC_READ_IN_POSITION, request, sizeof(request));
                if (transact(request, len, response, sizeof(response), responseLen, DIAGNOSTIC_TIMEOUT_MS)) {
                    Pd42s1Protocol::parse_byte_value(response, responseLen, Pd42s1Protocol::FUNC_READ_IN_POSITION, inpos);
                }
            }

            char posText[16];
            char errText[16];
            char speedText[16];
            snprintf(posText, sizeof(posText), hasPos ? "%ld" : "?", long(position));
            snprintf(errText, sizeof(errText), hasError ? "%ld" : "?", long(error));
            snprintf(speedText, sizeof(speedText), hasSpeed ? "%d" : "?", int(speed));

            log_info("  address " << int(address) << ": fw V" << int(firmware / 10) << "." << int(firmware % 10) << " pos=" << posText
                                  << " error=" << errText << " speed=" << speedText << "RPM in-position="
                                  << (inpos == 0xFF ? "?" : (inpos ? "yes" : "no")));
        }
        _bus->end_diagnostic();
        if (!found) {
            log_info("CAN positions: (no devices)");
        }
    }

    bool Pd42s1Driver::set_driver_enable(uint8_t address, bool enable) {
        if (!_bus || !_bus->ready() || !_bus->control_ready() || _bus->inhibited() || (_bus->stop_latched() && enable)) {
            return false;
        }
        uint8_t request[Pd42s1Protocol::MAX_FRAME_LEN];
        uint8_t response[16];
        size_t  responseLen = 0;
        size_t  len         = Pd42s1Protocol::build_enable_command(address, enable, request, sizeof(request));
        if (!len) {
            return false;
        }
        CanBus::Lock lock(*_bus);
        bool ok = transact(request, len, response, sizeof(response), responseLen, 150) &&
                  Pd42s1Protocol::parse_ack(response, responseLen);
        can_log_info("CAN: " << (enable ? "enable" : "disable") << " address " << int(address) << (ok ? " ok" : " failed"));
        return ok;
    }

    bool Pd42s1Driver::set_group_address(uint8_t address, uint8_t group) {
        if (!_bus || !_bus->ready() || !_bus->control_ready() || _bus->stop_latched() || _bus->inhibited() || group == 0 ||
            address == group) {
            return false;
        }
        uint8_t request[Pd42s1Protocol::MAX_FRAME_LEN];
        uint8_t response[16];
        size_t  responseLen = 0;

        {
            CanBus::Lock lock(*_bus);
            size_t len = Pd42s1Protocol::build_set_group_address_command(address, group, request, sizeof(request));
            if (!len || !_bus->send_message(CAN_COMMAND_ID, request, len, 150)) {
                can_log_error("CAN: setting group address on address " << int(address) << " failed");
                return false;
            }
            vTaskDelay(pdMS_TO_TICKS(50));

            // Some drivers do not echo the 0x61 set command.  Success is
            // proven by the old slave address no longer replying.
            uint8_t firmware = 0;
            uint8_t hardware = 0;
            if (probe_address_quiet(address, firmware, hardware, 80, 2)) {
                can_log_error("CAN: address " << int(address) << " still answers at the old address after group switch");
                return false;
            }

            // The driver now listens only on the group address, so save the
            // parameter through the group.  The normal slave member of the
            // group answers and the group-only member executes silently.
            len = Pd42s1Protocol::build_save_params_command(group, request, sizeof(request));
            if (!len || !transact(request, len, response, sizeof(response), responseLen, 200) ||
                !Pd42s1Protocol::parse_ack(response, responseLen)) {
                can_log_error("CAN: group address " << int(group) << " set, but save failed");
                return false;
            }
        }

        can_log_info("CAN: address " << int(address) << " is now group member " << int(group));
        return true;
    }

    bool Pd42s1Driver::send_single_position(uint8_t address, int32_t delta, uint16_t speed_rpm, uint8_t accel) {
        if (!_bus || !_bus->ready() || !_bus->control_ready() || _bus->stop_latched() || _bus->inhibited()) {
            return false;
        }
        int32_t current = 0;
        if (!read_driver_position(address, current)) {
            can_log_error("CAN move: cannot read position at address " << int(address));
            return false;
        }
        const int64_t target64 = int64_t(current) + int64_t(delta);
        if (target64 < std::numeric_limits<int32_t>::min() || target64 > std::numeric_limits<int32_t>::max()) {
            can_log_error("CAN move: target overflow at address " << int(address));
            return false;
        }
        const int32_t target = int32_t(target64);
        uint8_t request[Pd42s1Protocol::MAX_FRAME_LEN];
        uint8_t response[16];
        size_t  responseLen = 0;
        size_t  len = Pd42s1Protocol::build_position_command(address, accel, speed_rpm, uint32_t(target), request, sizeof(request));
        if (!len) {
            return false;
        }
        CanBus::Lock lock(*_bus);
        bool ok = transact(request, len, response, sizeof(response), responseLen, Pd42s1Protocol::Timing::TRANSACTION_TIMEOUT_MS) &&
                  Pd42s1Protocol::parse_ack(response, responseLen);
        can_log_info("CAN move: address " << int(address) << " from " << current << " to " << target << " (delta " << delta << ") "
                                          << (ok ? "ok" : "failed"));
        if (ok) {
            _bus->invalidate_client_position(address);
        }
        return ok;
    }

    bool Pd42s1Driver::zero_driver_position(uint8_t address) {
        if (!_bus || !_bus->ready()) {
            return false;
        }
        uint8_t request[Pd42s1Protocol::MAX_FRAME_LEN];
        size_t  len = Pd42s1Protocol::build_zero_position_command(address, request, sizeof(request));
        if (!len) {
            return false;
        }

        bool ok = false;
        {
            CanBus::Lock lock(*_bus);
            for (int attempt = 0; attempt < 3 && !ok; ++attempt) {
                uint8_t response[16];
                size_t  responseLen = 0;
                if (transact(request, len, response, sizeof(response), responseLen, 150) &&
                    Pd42s1Protocol::parse_ack(response, responseLen)) {
                    ok = true;
                } else {
                    vTaskDelay(pdMS_TO_TICKS(20));
                }
            }
            if (ok) {
                // Make the client unsynced before releasing the bus lock, so
                // the worker cannot send an old absolute target between the
                // zeroing transaction and the cache invalidation.
                _bus->invalidate_client_position(address);
            }
        }

        can_log_info("CAN: zero position at address " << int(address) << (ok ? " ok" : " failed"));
        return ok;
    }

    bool Pd42s1Driver::wait_for_driver_sync(const uint8_t* addresses, size_t count, uint32_t timeout_ms) {
        if (!count) {
            return true;
        }
        if (!_bus) {
            return false;
        }
        const uint32_t deadline = now_ms() + timeout_ms;
        while (int32_t(deadline - now_ms()) > 0) {
            bool all = true;
            for (size_t a = 0; a < count && all; ++a) {
                all = _bus->client_synced(addresses[a]);
            }
            if (all) {
                return true;
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        return false;
    }

    bool Pd42s1Driver::stop_all_drivers() {
        if (!driver_ok("stop")) {
            return false;
        }
        if (!_bus || !_bus->ready()) {
            log_info("CAN stop: bus not ready");
            return false;
        }
        _bus->latch_stop();
        // Let a worker pass that already passed the latch check finish its
        // current poll.  Only then take the bus lock, otherwise that pass can
        // re-enable a motor after the addressed disable below.
        _bus->wait_for_worker_idle();
        CanBus::Lock lock(*_bus);

        // Safe stop = disable all motor outputs.  The 0xFC immediate-brake
        // command is deliberately NOT used: on firmware V1.6 the brake state
        // can only be cleared with 0xFB, and a broadcast 0xFB leaves all
        // drivers silent until they are power cycled (bench-tested).
        // Addressed disable is used because a broadcast is not acknowledged
        // by every driver variant; then latch the bus so sync/$ME cannot
        // silently re-enable motion.
        uint8_t addresses[CanBus::MAX_CLIENTS];
        size_t  count = target_addresses(addresses, CanBus::MAX_CLIENTS);
        int     disabled = 0;
        for (size_t i = 0; i < count; ++i) {
            uint8_t request[Pd42s1Protocol::MAX_FRAME_LEN];
            uint8_t response[16];
            bool    off = false;
            for (int attempt = 0; attempt < 5 && !off; ++attempt) {
                size_t len         = Pd42s1Protocol::build_enable_command(addresses[i], false, request, sizeof(request));
                size_t responseLen = 0;
                if (!transact(request, len, response, sizeof(response), responseLen, Pd42s1Protocol::Timing::TRANSACTION_TIMEOUT_MS) ||
                    !Pd42s1Protocol::parse_ack(response, responseLen)) {
                    vTaskDelay(pdMS_TO_TICKS(10));
                    continue;
                }
                // Verify the actual enable state, not only the ACK.
                len = Pd42s1Protocol::build_read_command(addresses[i], Pd42s1Protocol::FUNC_READ_ENABLE, request, sizeof(request));
                uint8_t state = 0xFF;
                responseLen   = 0;
                if (transact(request, len, response, sizeof(response), responseLen, Pd42s1Protocol::Timing::TRANSACTION_TIMEOUT_MS) &&
                    Pd42s1Protocol::parse_byte_value(response, responseLen, Pd42s1Protocol::FUNC_READ_ENABLE, state) && state == 1) {
                    off = true;
                } else {
                    vTaskDelay(pdMS_TO_TICKS(10));
                }
            }
            if (off) {
                ++disabled;
            }
        }
        can_log_warn("CAN: safe stop: addressed motor disable verified for " << disabled << "/" << count
                                                                           << " driver(s); bus control latched");
        _bus->flush_receive();
        return disabled == int(count);
    }

    void Pd42s1Driver::read_raw(uint8_t address, uint8_t function) {
        if (!driver_ok("read")) {
            return;
        }
        if (!_bus || !_bus->ready()) {
            log_info("CAN read: bus not ready");
            return;
        }
        struct DiagnosticGuard {
            CanBus* bus;
            ~DiagnosticGuard() { bus->end_diagnostic(); }
        } guard{ _bus };
        _bus->begin_diagnostic();
        CanBus::Lock lock(*_bus);

        uint8_t request[Pd42s1Protocol::MAX_FRAME_LEN];
        uint8_t response[128];
        size_t  len = Pd42s1Protocol::build_read_command(address, function, request, sizeof(request));

        char funcText[8];
        snprintf(funcText, sizeof(funcText), "0x%02X", int(function));

        // Deliberately not using transact(): this diagnostic should show the
        // raw bytes even when the response length is not modelled yet.
        size_t total = 0;
        for (int attempt = 0; attempt < 3; ++attempt) {
            total = 0;
            _bus->flush_receive();
            if (!_bus->send_message(CAN_COMMAND_ID, request, len, 150)) {
                log_warn("CAN read: address " << int(address) << " function " << funcText << ": transmit failed");
                return;
            }

            uint32_t deadline = now_ms() + 300;
            while (total < sizeof(response)) {
                int32_t left = int32_t(deadline - now_ms());
                if (left <= 0) {
                    break;
                }
                // Give the first frame the whole remaining timeout.  The old
                // code capped every receive at 50 ms and broke on the first
                // timeout, so a driver reply that arrived after 50 ms was
                // reported as "no response" even though the transaction
                // timeout was 300 ms.  After the first frame, only wait for
                // the next frame of a multi-frame answer.
                uint32_t wait     = total ? (left > 2 ? 2 : uint32_t(left)) : uint32_t(left);
                size_t   received = _bus->receive_chunk(response + total, sizeof(response) - total, wait);
                if (received == 0) {
                    if (total == 0) {
                        continue;  // keep waiting for the first frame until deadline
                    }
                    break;  // end of a multi-frame answer
                }
                total += received;
                // Stop as soon as the protocol-reported length is complete;
                // only unmodelled function codes need to wait for silence.
                size_t expected = Pd42s1Protocol::expected_response_length(response, total);
                if (expected != 0 && total >= expected) {
                    break;
                }
            }
            if (total) {
                break;
            }
            if (attempt < 2) {
                vTaskDelay(pdMS_TO_TICKS(2));
            }
        }
        _bus->flush_receive();

        if (!total) {
            log_warn("CAN read: address " << int(address) << " function " << funcText << ": no response");
            return;
        }
        char  text[3 * 128 + 1];
        char* out = text;
        for (size_t i = 0; i < total; ++i) {
            out += snprintf(out, size_t(text + sizeof(text) - out), "%02X ", response[i]);
        }
        log_info("CAN read: address " << int(address) << " function " << funcText << " response(" << int(total) << "): " << text);
    }

    void Pd42s1Driver::restart_drivers() {
        if (!driver_ok("reset")) {
            return;
        }
        if (!_bus || !_bus->ready()) {
            log_info("CAN reset: bus not ready");
            return;
        }
        CanBus::Lock lock(*_bus);

        uint8_t request[Pd42s1Protocol::MAX_FRAME_LEN];
        size_t  len = Pd42s1Protocol::build_frame(0, Pd42s1Protocol::FUNC_RESTART, nullptr, 0, request, sizeof(request));
        if (!len) {
            return;
        }
        can_log_info("CAN: broadcasting driver reset (0x02)");
        if (!_bus->send_message(CAN_COMMAND_ID, request, len, 100)) {
            can_log_error("CAN: driver reset transmit failed; keeping control latched");
            _bus->flush_receive();
            return;
        }
        _bus->flush_receive();

        // The drivers may have lost their position/state.  Invalidate every
        // client while the bus lock is still held, then let the normal sync
        // path rebuild the base before any target can be sent.
        _bus->invalidate_all_client_positions();

        // Give the driver firmware time to boot and re-initialise its CAN
        // controller before trying to talk to it again.
        vTaskDelay(pdMS_TO_TICKS(2500));
        _bus->flush_receive();

        send_echo_state_forced(true);
        vTaskDelay(pdMS_TO_TICKS(50));
        uint8_t addresses[CanBus::MAX_CLIENTS];
        size_t  count = target_addresses(addresses, CanBus::MAX_CLIENTS);
        for (size_t i = 0; i < count; ++i) {
            enable_driver_echo_locked(addresses[i]);
        }
        _bus->flush_receive();
        _bus->clear_stop_latch();
        can_log_info("CAN: driver reset done, command echo re-enabled; run $CANScan");
    }

    bool Pd42s1Driver::send_echo_state(bool enable) {
        if (_echo_state == enable) {
            return true;  // already configured, no need to retransmit or log
        }
        // Broadcast (address 0) "set command echo" (0x6F).  Broadcast
        // commands are executed but not answered, so this works even when the
        // drivers are currently set to not reply.  A present receiver ACKs
        // the frame at the CAN level regardless of its echo setting.
        uint8_t request[Pd42s1Protocol::MAX_FRAME_LEN];
        uint8_t echo = enable ? 0 : 1;
        size_t  len  = Pd42s1Protocol::build_frame(0, Pd42s1Protocol::FUNC_SET_ECHO, &echo, 1, request, sizeof(request));
        bool    ok   = false;
        if (len) {
            can_log_info("CAN: " << (enable ? "enabling" : "disabling") << " command echo (broadcast 0x6F=" << int(echo) << ")");
            ok = _bus->send_message(CAN_COMMAND_ID, request, len, 50);
            if (ok) {
                _echo_state = enable;
            }
            vTaskDelay(pdMS_TO_TICKS(2));
        }
        _bus->flush_receive();
        return ok;
    }

    bool Pd42s1Driver::send_echo_state_forced(bool enable) {
        // The scan must transmit the echo setting every time, even when the
        // cached state says it is already correct: a driver may have missed an
        // earlier broadcast (for example because another driver was not
        // powered at that moment).
        _echo_state = !enable;
        return send_echo_state(enable);
    }

    bool Pd42s1Driver::enable_driver_echo_locked(uint8_t address) {
        uint8_t request[Pd42s1Protocol::MAX_FRAME_LEN];
        uint8_t echo = 0;  // 0 = acknowledge commands
        size_t  len  = Pd42s1Protocol::build_frame(address, Pd42s1Protocol::FUNC_SET_ECHO, &echo, 1, request, sizeof(request));
        if (!len) {
            return false;
        }
        can_log_verbose("CAN: enabling command echo at address " << int(address));
        bool ok = _bus->send_message(CAN_COMMAND_ID, request, len, 50);
        vTaskDelay(pdMS_TO_TICKS(2));  // let the driver apply the setting
        _bus->flush_receive();         // discard its ack if echo was already on
        return ok;
    }

    void Pd42s1Driver::enable_all_driver_echo_locked() {
        // Broadcast first: every powered driver executes it without replying,
        // even if it is currently set to "no reply".  Then send the same
        // setting to each address individually, which also covers drivers
        // that filter broadcast frames.
        send_echo_state_forced(true);
        vTaskDelay(pdMS_TO_TICKS(5));
        uint8_t addresses[CanBus::MAX_CLIENTS];
        size_t  count = target_addresses(addresses, CanBus::MAX_CLIENTS);
        for (size_t i = 0; i < count; ++i) {
            enable_driver_echo_locked(addresses[i]);
        }
        _echo_state = true;
    }

    void Pd42s1Driver::enable_all_driver_echo() {
        if (!driver_ok("echo")) {
            return;
        }
        if (!_bus || !_bus->ready()) {
            can_log_info("CAN echo: bus not ready");
            return;
        }
        CanBus::Lock lock(*_bus);
        enable_all_driver_echo_locked();
        can_log_info("CAN echo: sent 0x6F=0 to configured addresses");
    }

    size_t Pd42s1Driver::client_addresses(uint8_t* out, size_t max) const {
        return _bus ? _bus->client_addresses(out, max) : 0;
    }

    bool Pd42s1Driver::suspend() {
        return _bus ? _bus->suspend() : false;
    }

    bool Pd42s1Driver::resume() {
        return _bus ? _bus->resume() : false;
    }

    bool Pd42s1Driver::control_ready() const {
        return _bus && _bus->control_ready();
    }
}

#endif  // MAX_N_CAN

