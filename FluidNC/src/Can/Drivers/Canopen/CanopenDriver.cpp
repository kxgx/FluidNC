// Copyright (c) 2026 - FluidNC contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "Config.h"

#if MAX_N_CAN

#    include "CanopenDriver.h"

#    include "Can/CanDriverRegistry.h"
#    include "Can/Protocols/Standard/Canopen/CanopenProtocol.h"
#    include "Logging.h"

#    include <Arduino.h>
#    include <esp_timer.h>

#    include <cstdio>
#    include <cstring>
#    include <limits>

namespace Machine {
    using namespace Canopen;

    namespace {
        // Foreground ($CAN...) reports may run while the worker task polls the
        // same nodes.  Every SDO transaction is atomic (it takes the bus lock),
        // so a longer per-operation timeout is safe and avoids dropping a node
        // from a report because one reply was slow.
        constexpr uint32_t DIAGNOSTIC_TIMEOUT_MS = 300;

        uint32_t now_ms() {
            return uint32_t(esp_timer_get_time() / 1000);
        }

        CanDriver* createCanopen(CanBus* bus) {
            return bus ? new CanopenDriver(bus) : nullptr;
        }

        // Not a default: pd42s1 remains the default for existing machines.
        CanDriverRegistration _canopenRegistration("canopen", createCanopen);

        struct DiagnosticGuard {
            CanBus* bus;
            ~DiagnosticGuard() { bus->end_diagnostic(); }
        };
    }

    size_t CanopenDriver::target_addresses(uint8_t* out, size_t max) const {
        size_t count = CanBus::configured_addresses(out, max);
        if (!count) {
            // Discovery fallback only; CanBus::init() normally refuses to start
            // the bus without a configured CAN motor driver.
            for (size_t i = 0; i < max; ++i) {
                out[count++] = uint8_t(i + 1);
            }
        }
        return count;
    }

    bool CanopenDriver::node_for_address(uint8_t address, Canopen::NodeConfig& node) const {
        if (Canopen::NodeRegistry::get(address, node)) {
            return true;
        }
        node          = Canopen::NodeConfig();
        node.node_id  = address;
        return false;
    }

    bool CanopenDriver::init_driver() {
        if (!_bus || !_bus->ready()) {
            can_log_error("CANopen: TWAI transport is not ready");
            return false;
        }

        uint8_t addresses[CanBus::MAX_CLIENTS];
        size_t  count = target_addresses(addresses, CanBus::MAX_CLIENTS);

        size_t started = 0;
        for (size_t i = 0; i < count; ++i) {
            Canopen::NodeConfig node;
            node_for_address(addresses[i], node);
            Canopen::Client client(_bus, node);
            if (client.nmt(Canopen::NmtCommand::Start, 50)) {
                ++started;
                can_log_info("CANopen: NMT start sent to node " << int(addresses[i]));
            } else {
                can_log_warn("CANopen: NMT start to node " << int(addresses[i]) << " could not be transmitted");
            }
            vTaskDelay(pdMS_TO_TICKS(2));
        }

        // A node that is not ready yet must not fail bus initialisation: its
        // motor keeps control disabled until the node answers standard SDOs.
        can_log_info("CANopen: driver initialised at " << _bus->baud() << " baud (" << started << "/" << count
                                                       << " configured node(s) acknowledged NMT start); CiA402 defaults assumed");
        return true;
    }

    int CanopenDriver::probe_node(uint8_t node_id) {
        Canopen::NodeConfig node;
        node_for_address(node_id, node);
        Canopen::Client client(_bus, node);

        uint32_t device_type = 0;
        if (client.read_device_type(device_type, 120)) {
            char text[16];
            snprintf(text, sizeof(text), "0x%08X", unsigned(device_type));
            log_info("  CANopen node " << int(node_id) << ": device type " << text);
            return 1;
        }

        // Some drives answer heartbeats but keep 0x1000 unavailable (or send a
        // boot-up frame only).  A heartbeat still proves a live CANopen node.
        Canopen::NmtState state = Canopen::NmtState::Unknown;
        if (client.wait_heartbeat(state, 250)) {
            log_info("  CANopen node " << int(node_id) << ": heartbeat state " << Canopen::nmt_state_name(state)
                                       << " (SDO read of 0x1000 did not answer)");
            return 1;
        }
        return 0;
    }

    int CanopenDriver::scan() {
        if (!_bus || !_bus->ready()) {
            log_info("CANopen scan: bus not ready");
            return 0;
        }

        CanBus::BusStatus status;
        if (_bus->bus_status(status)) {
            log_info("CANopen scan: state=" << status.state << " tx_err=" << status.tx_error << " rx_err=" << status.rx_error
                                            << " bus_errors=" << status.bus_errors);
        }

        uint8_t addresses[CanBus::MAX_CLIENTS];
        size_t  count = target_addresses(addresses, CanBus::MAX_CLIENTS);
        if (!CanBus::configured_driver_count()) {
            log_info("CANopen scan: no CANopen motor configured; probing standard node ids 1.." << int(CanBus::MAX_CLIENTS));
        }

        int found = 0;
        for (size_t i = 0; i < count; ++i) {
            found += probe_node(addresses[i]);
        }
        if (!found) {
            log_info("CANopen scan: no nodes answered (check can.driver: canopen, node ids, CAN bit rate and wiring)");
        } else {
            log_info("CANopen scan: found " << found << " of " << int(count) << " probed node(s)");
        }
        return found;
    }

    int CanopenDriver::scan_all_bauds() {
        // CANopen has no in-band bit-rate negotiation: every node on the bus
        // must already use can.baud.  Probing other rates would only garble the
        // bus, so this reports the limitation and scans at the configured rate.
        log_info("CANopen scan: CANopen nodes do not negotiate the bit rate in-band; scanning at the configured " << (_bus ? _bus->baud() : 0)
                                                                                                                   << " baud");
        return scan();
    }

    void CanopenDriver::report_status() {
        if (!_bus || !_bus->ready()) {
            log_info("CANopen status: bus not ready");
            return;
        }

        CanBus::BusStatus status;
        if (!_bus->bus_status(status)) {
            log_error("CANopen status: cannot read TWAI status");
            return;
        }
        log_info("CANopen status: state=" << status.state << " tx_err=" << status.tx_error << " rx_err=" << status.rx_error
                                          << " bus_errors=" << status.bus_errors << " tx_queue=" << status.msgs_to_tx
                                          << " rx_queue=" << status.msgs_to_rx << " bus_off_count=" << status.bus_off_count);
        log_info("CANopen config: tx=" << _bus->tx_pin().name() << " rx=" << _bus->rx_pin().name() << " baud=" << _bus->baud()
                                       << " driver=canopen (standard CiA301/CiA402, no private fallback)");
        if (_bus->inhibited()) {
            log_error("CANopen: control inhibited: " << (_bus->inhibit_reason() ? _bus->inhibit_reason() : "unknown reason"));
        }
        if (_bus->stop_latched()) {
            log_info("CANopen: control is latched stopped; $CANReset releases it");
        }

        uint8_t addresses[CanBus::MAX_CLIENTS];
        size_t  count = target_addresses(addresses, CanBus::MAX_CLIENTS);
        log_info("CANopen: " << int(count) << " configured node id(s)");

        _bus->begin_diagnostic();
        DiagnosticGuard guard { _bus };
        for (size_t i = 0; i < count; ++i) {
            Canopen::NodeConfig node;
            node_for_address(addresses[i], node);
            Canopen::Client client(_bus, node);
            uint16_t        statusword = 0;
            if (client.read_statusword(statusword, DIAGNOSTIC_TIMEOUT_MS)) {
                char text[8];
                snprintf(text, sizeof(text), "0x%04X", unsigned(statusword));
                log_info("  node " << int(addresses[i]) << ": statusword=" << text << " state="
                                   << CiA402::state_name(CiA402::decode_state(statusword)));
            } else {
                char text[8];
                snprintf(text, sizeof(text), "0x%04X", unsigned(node.statusword_index));
                log_warn("  node " << int(addresses[i]) << ": no SDO response (" << text << ")");
            }
        }
    }

    void CanopenDriver::clear_driver_faults() {
        if (!_bus || !_bus->ready()) {
            log_info("CANopen clear: bus not ready");
            return;
        }

        uint8_t addresses[CanBus::MAX_CLIENTS];
        size_t  count = target_addresses(addresses, CanBus::MAX_CLIENTS);

        _bus->begin_diagnostic();
        DiagnosticGuard guard { _bus };
        for (size_t i = 0; i < count; ++i) {
            Canopen::NodeConfig node;
            node_for_address(addresses[i], node);
            Canopen::Client client(_bus, node);
            uint16_t        statusword = 0;
            if (!client.read_statusword(statusword, DIAGNOSTIC_TIMEOUT_MS)) {
                log_warn("  node " << int(addresses[i]) << ": no response; cannot inspect or clear faults");
                continue;
            }
            if (!CiA402::is_fault(statusword)) {
                log_info("  node " << int(addresses[i]) << ": no fault (" << CiA402::state_name(CiA402::decode_state(statusword)) << ")");
                continue;
            }
            log_warn("  node " << int(addresses[i]) << ": FAULT; writing CiA402 fault reset");
            bool cleared = client.fault_reset(500);
            if (cleared && client.read_statusword(statusword, DIAGNOSTIC_TIMEOUT_MS)) {
                log_info("  node " << int(addresses[i]) << ": fault reset -> " << CiA402::state_name(CiA402::decode_state(statusword)));
            } else if (!cleared) {
                log_error("  node " << int(addresses[i]) << ": fault reset failed; the drive fault may be latched");
            }
        }
    }

    void CanopenDriver::restart_drivers() {
        if (!_bus || !_bus->ready()) {
            log_info("CANopen reset: bus not ready");
            return;
        }

        uint8_t addresses[CanBus::MAX_CLIENTS];
        size_t  count = target_addresses(addresses, CanBus::MAX_CLIENTS);

        int reset = 0;
        for (size_t i = 0; i < count; ++i) {
            Canopen::NodeConfig node;
            node_for_address(addresses[i], node);
            Canopen::Client client(_bus, node);
            if (client.nmt(Canopen::NmtCommand::ResetNode, 100)) {
                ++reset;
            } else {
                can_log_warn("CANopen: NMT reset could not be transmitted to node " << int(addresses[i]));
            }
            vTaskDelay(pdMS_TO_TICKS(5));
        }

        // A CiA301 reset node re-boots the node into pre-operational mode.
        vTaskDelay(pdMS_TO_TICKS(500));
        for (size_t i = 0; i < count; ++i) {
            Canopen::NodeConfig node;
            node_for_address(addresses[i], node);
            Canopen::Client client(_bus, node);
            client.nmt(Canopen::NmtCommand::Start, 100);
            vTaskDelay(pdMS_TO_TICKS(5));
        }
        _bus->flush_receive();

        // The drives may have lost their position/state, and the reset also
        // releases the safe-stop latch as the explicit recovery command.  Every
        // client resynchronises before it may send another absolute target.
        _bus->invalidate_all_client_positions();
        _bus->clear_stop_latch();
        can_log_warn("CANopen: NMT reset sent to " << reset << "/" << int(count) << " node(s); run $CANScan before moving");
    }

    void CanopenDriver::enable_all_driver_echo() {
        log_info("CANopen: command echo is not applicable; a CANopen server always answers SDO requests");
    }

    bool CanopenDriver::set_driver_enable(uint8_t address, bool enable) {
        if (!_bus || !_bus->ready() || !_bus->control_ready() || _bus->inhibited() || (_bus->stop_latched() && enable)) {
            return false;
        }

        Canopen::NodeConfig node;
        node_for_address(address, node);
        Canopen::Client client(_bus, node);

        bool ok = false;
        if (enable) {
            ok = client.enable_operation(1000);
        } else {
            ok = client.disable_operation(500);
        }
        can_log_info("CANopen: " << (enable ? "enable" : "disable") << " node " << int(address) << (ok ? " ok" : " failed"));
        return ok;
    }

    bool CanopenDriver::send_single_position(uint8_t address, int32_t delta, uint16_t speed_rpm, uint8_t accel) {
        if (!_bus || !_bus->ready() || !_bus->control_ready() || _bus->stop_latched() || _bus->inhibited()) {
            return false;
        }
        // The SDO target-position path does not use a per-move speed/accel
        // parameter: CiA402 profile position mode uses the drive's profile
        // velocity/acceleration objects.  They are only informational here.
        (void)speed_rpm;
        (void)accel;

        Canopen::NodeConfig node;
        node_for_address(address, node);
        Canopen::Client client(_bus, node);

        int32_t current = 0;
        if (!client.read_actual_position(current, 250)) {
            can_log_error("CANopen move: cannot read actual position of node " << int(address));
            return false;
        }
        const int64_t target64 = int64_t(current) + int64_t(delta);
        if (target64 < std::numeric_limits<int32_t>::min() || target64 > std::numeric_limits<int32_t>::max()) {
            can_log_error("CANopen move: target overflow on node " << int(address));
            return false;
        }
        const int32_t target = int32_t(target64);
        const bool    ok     = client.write_target_position(target, 300);
        can_log_info("CANopen move: node " << int(address) << " from " << current << " to " << target << " (delta " << delta << ") "
                                           << (ok ? "ok" : "failed"));
        if (ok) {
            // The cached base position of the owning motor is now stale.
            _bus->invalidate_client_position(address);
        }
        return ok;
    }

    bool CanopenDriver::set_group_address(uint8_t address, uint8_t group) {
        log_info("CANopen: group addressing is not part of CiA301; use NMT broadcast or separate node ids");
        (void)address;
        (void)group;
        return false;
    }

    size_t CanopenDriver::client_addresses(uint8_t* out, size_t max) const {
        return _bus ? _bus->client_addresses(out, max) : 0;
    }

    bool CanopenDriver::zero_driver_position(uint8_t address) {
        // CiA402 has no standard "set actual position to zero" object.  Homing
        // (0x6060 = 6) or a vendor homing-offset object would be needed, and
        // that is exactly the kind of unverified vendor dictionary this standard
        // driver refuses to guess at.
        log_info("CANopen: zeroing the actual position of node " << int(address)
                                                                 << " is not supported: CiA402 has no standard zero-position object");
        return false;
    }

    bool CanopenDriver::wait_for_driver_sync(const uint8_t* addresses, size_t count, uint32_t timeout_ms) {
        if (!count || !_bus) {
            return true;
        }
        const uint32_t deadline = now_ms() + timeout_ms;
        while (int32_t(deadline - now_ms()) > 0) {
            bool all = true;
            for (size_t i = 0; i < count && all; ++i) {
                all = _bus->client_synced(addresses[i]);
            }
            if (all) {
                return true;
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        return false;
    }

    void CanopenDriver::report_driver_faults() {
        if (!_bus || !_bus->ready()) {
            log_info("CANopen faults: bus not ready");
            return;
        }

        uint8_t addresses[CanBus::MAX_CLIENTS];
        size_t  count = target_addresses(addresses, CanBus::MAX_CLIENTS);

        _bus->begin_diagnostic();
        DiagnosticGuard guard { _bus };
        int             abnormal = 0;
        int             found    = 0;
        for (size_t i = 0; i < count; ++i) {
            Canopen::NodeConfig node;
            node_for_address(addresses[i], node);
            Canopen::Client client(_bus, node);
            uint16_t        statusword = 0;
            if (!client.read_statusword(statusword, DIAGNOSTIC_TIMEOUT_MS)) {
                log_warn("  node " << int(addresses[i]) << ": no SDO response");
                continue;
            }
            ++found;
            uint8_t error_register = 0;
            const bool has_error_register = client.read_error_register(error_register, DIAGNOSTIC_TIMEOUT_MS);
            const bool fault              = CiA402::is_fault(statusword);
            if (fault) {
                ++abnormal;
            }
            char text[8];
            snprintf(text, sizeof(text), "0x%04X", unsigned(statusword));
            char error_text[8];
            if (has_error_register) {
                snprintf(error_text, sizeof(error_text), "0x%02X", unsigned(error_register));
            } else {
                snprintf(error_text, sizeof(error_text), "?");
            }
            log_info("  node " << int(addresses[i]) << ": statusword=" << text << " state="
                               << CiA402::state_name(CiA402::decode_state(statusword))
                               << " fault=" << (fault ? "YES" : "no") << " error_register=" << error_text);
        }
        if (!found) {
            log_info("CANopen faults: no nodes answered");
        } else if (abnormal) {
            log_warn("CANopen faults: " << abnormal << " of " << found << " node(s) in FAULT");
        } else {
            log_info("CANopen faults: no node in FAULT");
        }
    }

    void CanopenDriver::report_driver_positions() {
        if (!_bus || !_bus->ready()) {
            log_info("CANopen positions: bus not ready");
            return;
        }

        uint8_t addresses[CanBus::MAX_CLIENTS];
        size_t  count = target_addresses(addresses, CanBus::MAX_CLIENTS);

        _bus->begin_diagnostic();
        DiagnosticGuard guard { _bus };
        int             found = 0;
        for (size_t i = 0; i < count; ++i) {
            Canopen::NodeConfig node;
            node_for_address(addresses[i], node);
            Canopen::Client client(_bus, node);

            int32_t target = 0;
            int32_t actual = 0;
            uint16_t statusword = 0;
            int8_t   mode       = 0;
            const bool has_actual = client.read_actual_position(actual, DIAGNOSTIC_TIMEOUT_MS);
            const bool has_status = client.read_statusword(statusword, DIAGNOSTIC_TIMEOUT_MS);
            const bool has_mode   = client.read_mode_display(mode, DIAGNOSTIC_TIMEOUT_MS);
            // The target position object normally reads back the last target in
            // CiA402, which makes the report show the commanded/actual pair.
            const bool has_target = client.sdo_upload_i32(node.target_position_index, 0, target, DIAGNOSTIC_TIMEOUT_MS);
            if (!has_actual && !has_status) {
                log_warn("  node " << int(addresses[i]) << ": no SDO response");
                continue;
            }
            ++found;
            char actual_text[16];
            char target_text[16];
            char mode_text[8];
            if (has_actual) {
                snprintf(actual_text, sizeof(actual_text), "%ld", long(actual));
            } else {
                snprintf(actual_text, sizeof(actual_text), "?");
            }
            if (has_target) {
                snprintf(target_text, sizeof(target_text), "%ld", long(target));
            } else {
                snprintf(target_text, sizeof(target_text), "?");
            }
            if (has_mode) {
                snprintf(mode_text, sizeof(mode_text), "%d", int(mode));
            } else {
                snprintf(mode_text, sizeof(mode_text), "?");
            }
            log_info("  node " << int(addresses[i]) << ": actual=" << actual_text << " target=" << target_text << " mode=" << mode_text
                               << " state=" << (has_status ? CiA402::state_name(CiA402::decode_state(statusword)) : "?")
                               << " target_reached=" << (has_status && CiA402::is_target_reached(statusword) ? "yes" : "no"));
        }
        if (!found) {
            log_info("CANopen positions: no nodes answered");
        }
    }

    bool CanopenDriver::stop_all_drivers() {
        if (!_bus || !_bus->ready()) {
            log_info("CANopen stop: bus not ready");
            return false;
        }

        // Latch first, then let a worker pass that already passed the latch
        // check finish, so an in-flight poll cannot re-enable the drive after
        // the quick stop below.
        _bus->latch_stop();
        _bus->wait_for_worker_idle();

        uint8_t addresses[CanBus::MAX_CLIENTS];
        size_t  count = target_addresses(addresses, CanBus::MAX_CLIENTS);

        int acked = 0;
        for (size_t i = 0; i < count; ++i) {
            Canopen::NodeConfig node;
            node_for_address(addresses[i], node);
            Canopen::Client client(_bus, node);
            // CiA402 quick stop (controlword 0x0002) is the standard controlled
            // stop.  A drive that does not answer cannot be stopped by this
            // firmware, so it is reported and the bus stays latched.
            if (client.quick_stop(300)) {
                ++acked;
            } else {
                can_log_error("CANopen: quick stop was not acknowledged by node " << int(addresses[i]));
            }
        }
        _bus->flush_receive();
        can_log_warn("CANopen: safe stop: CiA402 quick stop acknowledged by " << acked << "/" << int(count)
                                                                              << " node(s); control latched");
        return count == 0 || acked == int(count);
    }

    void CanopenDriver::read_raw(uint8_t address, uint8_t function) {
        // CANopen addresses objects by 16-bit index/sub-index, not by a byte
        // "function code"; interpreting the PD42S1-style function argument as an
        // object index would read an arbitrary object.  Report instead.
        log_info("CANopen: raw vendor reads are not supported (node " << int(address) << ", function " << int(function)
                                                                     << "); use $CANStatus/$CANFaults/$CANPositions");
    }

    bool CanopenDriver::run_auto_config() {
        log_info("CANopen: no auto-configuration required; node ids come from the motor configuration and the bit rate from "
                 "can.baud");
        return true;
    }

    void CanopenDriver::report_driver_configs() {
        if (!_bus || !_bus->ready()) {
            log_info("CANopen config: bus not ready");
            return;
        }

        uint8_t addresses[CanBus::MAX_CLIENTS];
        size_t  count = target_addresses(addresses, CanBus::MAX_CLIENTS);

        _bus->begin_diagnostic();
        DiagnosticGuard guard { _bus };
        for (size_t i = 0; i < count; ++i) {
            Canopen::NodeConfig node;
            const bool registered = node_for_address(addresses[i], node);
            char       sdo[16];
            char       statusword[8];
            char       target[8];
            char       actual[8];
            snprintf(sdo, sizeof(sdo), "0x%03X/0x%03X", unsigned(node.sdo_request()), unsigned(node.sdo_response()));
            snprintf(statusword, sizeof(statusword), "0x%04X", unsigned(node.statusword_index));
            snprintf(target, sizeof(target), "0x%04X", unsigned(node.target_position_index));
            snprintf(actual, sizeof(actual), "0x%04X", unsigned(node.actual_position_index));
            log_info("  node " << int(addresses[i]) << ": " << (registered ? "configured by a motor" : "default CiA402 dictionary")
                               << " sdo=" << sdo << " statusword=" << statusword << " target_position=" << target
                               << " actual_position=" << actual);

            Canopen::Client client(_bus, node);
            uint32_t        device_type = 0;
            if (client.read_device_type(device_type, DIAGNOSTIC_TIMEOUT_MS)) {
                char text[16];
                snprintf(text, sizeof(text), "0x%08X", unsigned(device_type));
                log_info("    device type " << text);
            }
            uint32_t modes = 0;
            if (client.read_supported_drive_modes(modes, DIAGNOSTIC_TIMEOUT_MS)) {
                char text[16];
                snprintf(text, sizeof(text), "0x%08X", unsigned(modes));
                log_info("    supported drive modes " << text);
            }
            Canopen::NmtState state = Canopen::NmtState::Unknown;
            if (client.wait_heartbeat(state, 100)) {
                log_info("    heartbeat state " << Canopen::nmt_state_name(state));
            }
        }
    }

    bool CanopenDriver::suspend() {
        return _bus ? _bus->suspend() : false;
    }

    bool CanopenDriver::resume() {
        return _bus ? _bus->resume() : false;
    }

    bool CanopenDriver::control_ready() const {
        return _bus && _bus->control_ready();
    }
}

#endif  // MAX_N_CAN
