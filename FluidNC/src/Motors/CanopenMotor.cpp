// Copyright (c) 2026 - FluidNC contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "Config.h"

#if MAX_N_CAN

#    include "CanopenMotor.h"

#    include "Machine/Axes.h"
#    include "Machine/MachineConfig.h"
#    include "Logging.h"
#    include "Protocol.h"
#    include "Stepping.h"

#    include <Arduino.h>
#    include <esp_timer.h>

#    include <cmath>
#    include <cstdio>
#    include <cstring>
#    include <limits>

using namespace Machine;
using namespace Machine::Canopen;

namespace MotorDrivers {
    namespace {
        constexpr uint32_t SDO_TIMEOUT_MS      = 150;
        constexpr uint32_t ENABLE_TIMEOUT_MS   = 1000;
        constexpr uint32_t SYNC_RETRY_MS       = 500;
        constexpr uint32_t ENABLE_RETRY_MS     = 1000;
        constexpr uint32_t STATUS_SETTLE_MS    = 500;
        constexpr uint32_t WARN_INTERVAL_MS    = 2000;
        constexpr int      MAX_FAILURES        = 3;
        // Same conservative limit as the other CAN motor drivers: a target
        // outside this range is more likely a coordinate bug than real travel.
        constexpr int32_t  TARGET_SAFE_LIMIT   = 1000000000;

        void IRAM_ATTR canopen_step_thunk(MotorDriver* driver) {
            static_cast<CanopenMotor*>(driver)->step_isr();
        }

        void IRAM_ATTR canopen_dir_thunk(MotorDriver* driver, bool dir) {
            static_cast<CanopenMotor*>(driver)->set_direction_isr(dir);
        }

        void hex16_text(uint16_t value, char (&out)[8]) {
            snprintf(out, sizeof(out), "0x%04X", unsigned(value));
        }
    }

    CanopenMotor::CanopenMotor(const char* name) : MotorDriver(name) {
        Machine::CanBus::mark_driver_configured();
    }

    CanopenMotor::~CanopenMotor() {
        if (_registered) {
            Machine::CanBus::unregister_configured_address(_node.node_id);
            Machine::Canopen::NodeRegistry::remove(_node.node_id);
        }
        Machine::CanBus::unmark_driver_configured();
    }

    void CanopenMotor::validate() {
        Assert(config != nullptr && config->_can != nullptr, "canopen motor driver requires a can: section");
        Assert(config->_can->configured(), "canopen motor driver requires can: tx_pin and rx_pin");
        Assert(std::strcmp(config->_can->driver_name(), "canopen") == 0, "canopen motor driver requires can.driver: canopen");

        int32_t nodeId = _address;
        if (_nodeId != 0) {
            Assert(_nodeId == _address, "canopen node_id and address must be equal when both are set");
            nodeId = _nodeId;
        }
        Assert(nodeId >= 1 && nodeId <= int(Machine::CanBus::MAX_CLIENTS),
               "canopen address/node_id must be 1..16 (the shared CAN client limit)");
        Assert(_countsPerStep > 0.0f, "canopen counts_per_step must be positive");
        Assert(_countsPerRev > 0, "canopen counts_per_rev must be positive");
        Assert(_updateMs >= 2 && _updateMs <= 1000, "canopen update_ms must be 2..1000");
        Assert(_statusPollMs <= 60000, "canopen status_poll_ms must be 0..60000");
        Assert(_speedRpm >= 0 && _speedRpm <= 6000, "canopen speed_rpm must be 0..6000");
        Assert(_controlwordIndex >= 0 && _controlwordIndex <= 0xFFFF, "canopen controlword_index must be a 16-bit object index");
        Assert(_statuswordIndex >= 0 && _statuswordIndex <= 0xFFFF, "canopen statusword_index must be a 16-bit object index");
        Assert(_targetPositionIndex >= 0 && _targetPositionIndex <= 0xFFFF, "canopen target_position_index must be a 16-bit object index");
        Assert(_actualPositionIndex >= 0 && _actualPositionIndex <= 0xFFFF, "canopen actual_position_index must be a 16-bit object index");
        Assert(_sdoRequestCobid >= 0 && _sdoRequestCobid <= 0x7FF, "canopen sdo_request_cobid must be an 11-bit id or 0");
        Assert(_sdoResponseCobid >= 0 && _sdoResponseCobid <= 0x7FF, "canopen sdo_response_cobid must be an 11-bit id or 0");

        _node                             = Machine::Canopen::NodeConfig();
        _node.node_id                     = uint8_t(nodeId);
        _node.sdo_request_cobid           = uint32_t(_sdoRequestCobid);
        _node.sdo_response_cobid          = uint32_t(_sdoResponseCobid);
        _node.controlword_index           = uint16_t(_controlwordIndex);
        _node.statusword_index            = uint16_t(_statuswordIndex);
        _node.mode_index                  = uint16_t(_modeIndex);
        _node.mode_display_index          = uint16_t(_modeIndex + 1);  // CiA402 defines 0x6061 next to 0x6060
        _node.target_position_index       = uint16_t(_targetPositionIndex);
        _node.actual_position_index       = uint16_t(_actualPositionIndex);

        // A closed-loop drive settles exactly on its target; allow 5% of a
        // revolution (at least 100 units) for encoder quantization/servo
        // following before treating a readback as a lost-step event.
        const int32_t percentLimit = _countsPerRev / 20;
        _followingErrorLimit       = percentLimit > 100 ? percentLimit : 100;

        Machine::CanBus::register_configured_address(_node.node_id);
        Machine::Canopen::NodeRegistry::add(_node);
        _registered = true;
    }

    void CanopenMotor::init() {
        config_message();

        _bus = config ? config->_can : nullptr;
        Assert(_bus != nullptr, "canopen motor driver requires a can: section");
        Assert(_bus->ready(), "CAN bus is not ready");
        Assert(_bus->control_ready(), "CANopen driver did not initialise; motion disabled");

        _bus->add_client(this);
        Stepping::assignMotorDriver(axis_index(), dual_axis_index(), this);
        _lastStreamedSteps = _stepCount;
        _nextSyncMs        = 0;
    }

    void CanopenMotor::config_message() {
        char request[8];
        char response[8];
        snprintf(request, sizeof(request), "0x%03X", unsigned(_node.sdo_request()));
        snprintf(response, sizeof(response), "0x%03X", unsigned(_node.sdo_response()));
        can_log_info("    " << name() << " CANopen node:" << int(_node.node_id) << " sdo:" << request << "/" << response
                            << " target:0x" << _targetPositionIndex << " actual:0x" << _actualPositionIndex
                            << " counts_per_rev:" << _countsPerRev << " counts_per_step:" << _countsPerStep);
    }

    void IRAM_ATTR CanopenMotor::step_isr() {
        // Direction bit 1 is negative, matching Stepping::step()'s axis_steps
        // convention.  Keep this motor-local counter in the same sign
        // convention as the shared axis counter.
        _stepCount += _direction ? -1 : 1;
    }

    void IRAM_ATTR CanopenMotor::set_direction_isr(bool dir) {
        _direction = dir;
    }

    IsrStepFn CanopenMotor::isr_step_fn() {
        return canopen_step_thunk;
    }

    IsrDirFn CanopenMotor::isr_dir_fn() {
        return canopen_dir_thunk;
    }

    void CanopenMotor::set_disable(bool disable) {
        // Called from contexts that must not run CAN I/O; the CAN task performs
        // the CiA402 state transition.
        _disableRequested = disable;
    }

    void CanopenMotor::stop_motion() {
        // Feed hold / safety stop: quick stop the drive and keep it disabled
        // until resume_motion() explicitly re-arms the axis.
        _holdLatched   = true;
        _stopRequested = true;
    }

    void CanopenMotor::resume_motion() {
        _holdLatched = false;
        _targetDirty = true;
    }

    void CanopenMotor::on_stepping_reset() {
        // The step coordinate changed underneath the cached base; adopt the
        // drive position again before sending another absolute target.
        _synced    = false;
        _online    = false;
        _targetDirty = true;
        _nextSyncMs  = 0;
    }

    void CanopenMotor::on_position_zeroed() {
        _synced    = false;
        _online    = false;
        _targetDirty = true;
        _nextSyncMs  = 0;
    }

    void CanopenMotor::on_diagnostic_complete() {
        // Do not immediately consume an overdue background poll right after a
        // long $CAN... report.
        _nextStatusMs = millis() + _statusPollMs;
    }

    int32_t CanopenMotor::target_position() const {
        // Do not assume _baseStepCount is zero: a resync during motion updates
        // the base position but the hardware step counter keeps counting.
        // Use this motor's own counter, not the shared axis counter:
        // Stepping::step() advances the axis counter even when this motor is
        // blocked/limited, so a dual-motor or limit-blocked axis would
        // otherwise receive a target that moves the stationary motor.
        const int64_t steps  = int64_t(_stepCount) - int64_t(_baseStepCount);
        double        scaled = double(steps) * double(_countsPerStep);
        if (_invertDirection) {
            scaled = -scaled;
        }
        const int64_t target = int64_t(_basePosition) + int64_t(llround(scaled));
        if (target > TARGET_SAFE_LIMIT) {
            return TARGET_SAFE_LIMIT;
        }
        if (target < -TARGET_SAFE_LIMIT) {
            return -TARGET_SAFE_LIMIT;
        }
        return int32_t(target);
    }

    bool CanopenMotor::write_profile_velocity(Canopen::Client& client) {
        if (_speedRpm <= 0 || _countsPerRev <= 0) {
            return true;
        }
        const int64_t unitsPerSecond = (int64_t(_countsPerRev) * int64_t(_speedRpm)) / 60;
        if (!client.write_profile_velocity(uint32_t(unitsPerSecond), SDO_TIMEOUT_MS)) {
            if (!_velocityWriteWarned) {
                _velocityWriteWarned = true;
                warn_throttled(millis(), "profile velocity (0x6081) write failed; speed_rpm will be ignored");
            }
            return false;
        }
        return true;
    }

    void CanopenMotor::resync_base(int32_t actual, const char* reason) {
        _basePosition      = actual;
        _baseStepCount     = _stepCount;
        _driverPosition    = actual;
        _lastTarget        = actual;
        _lastStreamedSteps = _baseStepCount;
        _targetDirty       = true;
        _idleSinceMs       = 0;
        can_log_verbose("CANopen: " << name() << " base " << reason << " at " << actual);
    }

    bool CanopenMotor::sync() {
        if (!_bus || !_bus->ready()) {
            return false;
        }
        if (_bus->stop_latched()) {
            _enabled = false;
            return false;
        }

        Canopen::Client client(_bus, _node);
        uint16_t        statusword = 0;
        int32_t         actual     = 0;

        // The first transaction after boot can catch a node that is still in
        // boot-up; one lost frame must not disable the axis.
        bool ok = client.read_statusword(statusword, SDO_TIMEOUT_MS) && client.read_actual_position(actual, SDO_TIMEOUT_MS);
        if (!ok) {
            vTaskDelay(pdMS_TO_TICKS(20));
            ok = client.read_statusword(statusword, SDO_TIMEOUT_MS) && client.read_actual_position(actual, SDO_TIMEOUT_MS);
        }
        if (!ok) {
            warn_throttled(millis(), "no SDO response from node; check power, node_id and can.baud");
            return false;
        }

        if (CiA402::is_fault(statusword)) {
            char text[8];
            hex16_text(statusword, text);
            can_log_warn("CANopen: " << name() << " node " << int(_node.node_id) << " statusword " << text
                                     << " is FAULT; writing CiA402 fault reset");
            if (!client.fault_reset(500) || !client.read_statusword(statusword, SDO_TIMEOUT_MS)) {
                warn_throttled(millis(), "drive is in FAULT and did not accept a fault reset");
                return false;
            }
        }

        // A resync never assumes the drive is still enabled: the enable
        // sequence runs again from the reported state machine state.  This also
        // recovers a drive left in quick-stop after $CANStop.
        _enabled = false;

        // Read the mode display once per change for diagnostics only.  The
        // firmware does not write the mode: the drive must already be set to a
        // position mode that accepts target-position updates.
        int8_t mode = 0;
        if (client.read_mode_display(mode, SDO_TIMEOUT_MS) && (!_modeReported || mode != _mode)) {
            _modeReported = true;
            _mode         = mode;
            can_log_info("CANopen: " << name() << " node " << int(_node.node_id) << " mode of operation " << int(mode)
                                     << " (CiA402 profile position is " << int(Canopen::Mode::PROFILE_POSITION) << ")");
        }

        write_profile_velocity(client);

        // Adopt the drive's actual position as the reference for the current
        // step counter.  A resync deliberately adopts ACTUAL, not the previous
        // commanded target: if steps were lost, re-sending the stale far target
        // would make the axis jump.
        resync_base(actual, "synchronised");
        _synced            = true;
        _online            = true;
        _consecutiveFailures = 0;
        _faultLatched      = false;

        char text[8];
        hex16_text(statusword, text);
        can_log_info("CANopen: " << name() << " node " << int(_node.node_id) << " online, statusword " << text << " ("
                                 << CiA402::state_name(CiA402::decode_state(statusword)) << "), position " << actual);
        return true;
    }

    bool CanopenMotor::apply_enable(bool enable) {
        Canopen::Client client(_bus, _node);

        if (!enable) {
            if (!client.disable_operation(500)) {
                warn_throttled(millis(), "disable (CiA402 shutdown) failed");
                return false;
            }
            _enabled     = false;
            _targetDirty = true;
            can_log_info("CANopen: " << name() << " node " << int(_node.node_id) << " disabled (CiA402 shutdown)");
            return true;
        }

        // In CSP/PP the drive may start following the last value in 0x607A as
        // soon as Operation Enabled is reached.  Read the actual position and
        // pre-load it as the target before enabling, then use that same actual
        // position as the new step-coordinate base.  Without the pre-load a
        // stale target from before a feed hold or soft reset can cause a large
        // uncontrolled move.
        int32_t actual = 0;
        if (!client.read_actual_position(actual, SDO_TIMEOUT_MS)) {
            warn_throttled(millis(), "cannot read actual position before enable");
            return false;
        }
        if (!client.write_target_position(actual, SDO_TIMEOUT_MS)) {
            warn_throttled(millis(), "cannot pre-load target position before enable");
            return false;
        }
        if (!client.enable_operation(ENABLE_TIMEOUT_MS)) {
            warn_throttled(millis(), "CiA402 enable sequence failed (shutdown/switch on/enable operation)");
            return false;
        }
        _enabled = true;
        resync_base(actual, "enabled");
        _targetDirty = true;
        can_log_info("CANopen: " << name() << " node " << int(_node.node_id) << " enabled (CiA402 operation enabled)");
        return true;
    }

    bool CanopenMotor::send_target(int32_t target) {
        Canopen::Client client(_bus, _node);
        if (!client.write_target_position(target, SDO_TIMEOUT_MS)) {
            return false;
        }
        if (_setpointToggle) {
            // CiA402 profile position latches 0x607A on a rising edge of
            // controlword bit 4.  Provide an explicit low -> high edge so a
            // drive that needs it sees a new set-point for every target.
            if (!client.write_controlword(CiA402::CW_ENABLE_OPERATION, SDO_TIMEOUT_MS) ||
                !client.write_controlword(CiA402::CW_ENABLE_OPERATION | CiA402::CW_NEW_SETPOINT, SDO_TIMEOUT_MS)) {
                return false;
            }
        }
        return true;
    }

    void CanopenMotor::poll_status(uint32_t now_ms) {
        Canopen::Client client(_bus, _node);

        uint16_t statusword = 0;
        if (!client.read_statusword(statusword, SDO_TIMEOUT_MS)) {
            note_failure("statusword readback failed");
            return;
        }
        if (CiA402::is_fault(statusword)) {
            char text[8];
            hex16_text(statusword, text);
            can_log_error("CANopen: " << name() << " node " << int(_node.node_id) << " reported FAULT (statusword " << text << ")");
            report_driver_fault("drive reported FAULT");
            _synced     = false;
            _nextSyncMs = now_ms + SYNC_RETRY_MS;
            return;
        }
        if (!_disableRequested && !CiA402::is_operation_enabled(statusword)) {
            // The drive dropped out of operation enabled (STO, quick stop,
            // fault reaction, ...).  Do not keep streaming targets into a
            // disabled drive; resynchronise and let the enable sequence run
            // again.
            can_log_warn("CANopen: " << name() << " node " << int(_node.node_id) << " left operation enabled ("
                                     << CiA402::state_name(CiA402::decode_state(statusword)) << "); resynchronising");
            _enabled   = false;
            _synced    = false;
            _nextSyncMs = now_ms + SYNC_RETRY_MS;
            return;
        }

        int32_t actual = 0;
        if (!client.read_actual_position(actual, SDO_TIMEOUT_MS)) {
            note_failure("position readback failed");
            return;
        }
        _driverPosition = actual;
        _online         = true;
        clear_failure();

        // Lost-step detection while the step stream is idle: the closed-loop
        // drive must settle on the last commanded target.  A persistent
        // mismatch means steps or target updates were lost, so re-anchor to the
        // actual position instead of commanding a stale target.
        const int32_t steps = _stepCount;
        if (steps == _lastStreamedSteps) {
            if (!_idleSinceMs) {
                _idleSinceMs = now_ms;
            } else if (now_ms - _idleSinceMs >= STATUS_SETTLE_MS) {
                const int64_t error = int64_t(actual) - int64_t(_lastTarget);
                if (error > _followingErrorLimit || error < -_followingErrorLimit) {
                    can_log_warn("CANopen: " << name() << " node " << int(_node.node_id) << " position mismatch: target "
                                             << _lastTarget << " actual " << actual << "; resynchronising");
                    resync_base(actual, "position mismatch");
                }
            }
        } else {
            _idleSinceMs = 0;
        }
    }

    void CanopenMotor::can_poll(uint32_t now_ms) {
        if (!_bus || !_bus->ready() || _bus->stop_latched() || _bus->inhibited()) {
            return;
        }

        if (!_synced) {
            if (int32_t(now_ms - _nextSyncMs) < 0) {
                return;
            }
            _nextSyncMs = now_ms + SYNC_RETRY_MS;
            if (!sync()) {
                _nextSyncMs = now_ms + SYNC_RETRY_MS * 2;
                return;
            }
        }

        if (_holdLatched) {
            if (_stopRequested) {
                _stopRequested = false;
                Canopen::Client client(_bus, _node);
                if (client.quick_stop(300)) {
                    _enabled     = false;
                    _targetDirty = true;
                    can_log_warn("CANopen: " << name() << " node " << int(_node.node_id) << " feed hold: CiA402 quick stop sent");
                } else {
                    warn_throttled(now_ms, "feed hold quick stop failed");
                }
            }
            return;
        }

        // Enable/disable requests are applied at most once per retry window so a
        // drive that refuses to enable cannot monopolise the bus.
        if (_disableRequested != !_enabled) {
            if (int32_t(now_ms - _nextEnableMs) < 0) {
                return;
            }
            if (!apply_enable(!_disableRequested)) {
                _nextEnableMs = now_ms + ENABLE_RETRY_MS;
                return;
            }
        }

        if (_stopRequested) {
            _stopRequested = false;
            Canopen::Client client(_bus, _node);
            if (client.quick_stop(300)) {
                _enabled     = false;
                _targetDirty = true;
                can_log_warn("CANopen: " << name() << " node " << int(_node.node_id) << " feed hold: CiA402 quick stop sent");
            } else {
                warn_throttled(now_ms, "feed hold quick stop failed");
            }
            return;
        }

        if (_disableRequested || !_enabled) {
            return;
        }

        if (int32_t(now_ms - _nextUpdateMs) >= 0) {
            _nextUpdateMs = now_ms + _updateMs;

            const int32_t steps  = _stepCount;
            const int32_t target = target_position();
            if (_targetDirty || target != _lastTarget) {
                if (send_target(target)) {
                    _lastTarget        = target;
                    _targetDirty       = false;
                    _lastStreamedSteps = steps;
                    _online            = true;
                    clear_failure();
                    can_log_verbose("CANopen: " << name() << " target " << target << " at steps " << steps);
                } else {
                    note_failure("target position SDO write failed");
                }
            }
        }

        if (_statusPollMs && !_bus->diagnostic_active() && int32_t(now_ms - _nextStatusMs) >= 0) {
            _nextStatusMs = now_ms + _statusPollMs;
            poll_status(now_ms);
        }
    }

    void CanopenMotor::warn_throttled(uint32_t now_ms, const char* message) {
        if (_lastWarnMs != 0 && now_ms - _lastWarnMs < WARN_INTERVAL_MS) {
            return;
        }
        _lastWarnMs = now_ms;
        can_log_warn("CANopen: " << name() << " node " << int(_node.node_id) << ": " << message);
    }

    void CanopenMotor::note_failure(const char* reason) {
        warn_throttled(millis(), reason);
        if (_consecutiveFailures < 255) {
            ++_consecutiveFailures;
        }
        if (_consecutiveFailures >= MAX_FAILURES) {
            _consecutiveFailures = 0;
            report_driver_fault(reason);
            _synced     = false;
            _nextSyncMs = millis() + SYNC_RETRY_MS;
        }
    }

    void CanopenMotor::clear_failure() {
        _consecutiveFailures = 0;
    }

    void CanopenMotor::report_driver_fault(const char* reason) {
        if (_faultLatched) {
            return;
        }
        _faultLatched = true;
        _enabled      = false;
        can_log_error("CANopen: " << name() << " node " << int(_node.node_id) << " lost (" << reason << "); stopping all motors");
        // Remove torque from every driver and stop the machine: an axis whose
        // closed-loop drive no longer follows the step stream must not keep
        // receiving motion commands.
        Axes::set_disable(true, true);
        send_alarm(ExecAlarm::HardStop);
    }

    namespace {
        MotorFactory::InstanceBuilder<CanopenMotor> registration("canopen");
    }
}

#endif  // MAX_N_CAN
