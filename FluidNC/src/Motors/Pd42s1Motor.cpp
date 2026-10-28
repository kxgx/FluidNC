// Copyright (c) 2026 - FluidNC contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "Config.h"

#if MAX_N_CAN

#    include "Pd42s1Motor.h"

#    include "Machine/MachineConfig.h"
#    include "Motors/Pd42s1Protocol.h"
#    include "Stepping.h"

#    include <esp_timer.h>

#    include <cmath>
#    include <cstdio>
#    include <cstring>

using namespace Machine;

namespace MotorDrivers {
    namespace {
        constexpr uint32_t TRANSACTION_TIMEOUT_MS = 50;
        constexpr uint32_t RETRY_MS               = 500;
        constexpr uint32_t WARN_INTERVAL_MS       = 5000;
        constexpr uint32_t POSITION_UNITS_PER_REV = 51200;

        uint32_t now_ms() {
            return uint32_t(esp_timer_get_time() / 1000);
        }

        const char* error_code_name(uint8_t code) {
            switch (code) {
                case 0xE1:
                    return "frame too short";
                case 0xE2:
                    return "bad frame header";
                case 0xE3:
                    return "bad frame tail";
                case 0xE4:
                    return "checksum error";
                case 0xE5:
                    return "unsupported function";
                case 0xE6:
                    return "invalid data";
                default:
                    return "unknown error";
            }
        }

        void IRAM_ATTR pd42s1_step_thunk(MotorDriver* driver) {
            static_cast<Pd42s1Motor*>(driver)->step_isr();
        }

        void IRAM_ATTR pd42s1_dir_thunk(MotorDriver* driver, bool dir) {
            static_cast<Pd42s1Motor*>(driver)->set_direction_isr(dir);
        }
    }

    void Pd42s1Motor::init() {
        config_message();

        _bus = config ? config->_can : nullptr;
        Assert(_bus != nullptr, "pd42s1 motor driver requires a can: section");
        Assert(_bus->ready(), "CAN bus is not ready");

        _bus->add_client(this);
        Stepping::assignMotorDriver(axis_index(), dual_axis_index(), this);
        _nextSyncMs = 0;
    }

    void Pd42s1Motor::validate() {
        Assert(config != nullptr && config->_can != nullptr, "pd42s1 motor driver requires a can: section");
        Assert(config->_can->configured(), "pd42s1 motor driver requires can: tx_pin and rx_pin to be configured");
        Assert(_address >= 1 && _address <= 255, "pd42s1 address must be 1..255");
        Assert(_countsPerStep > 0.0f, "pd42s1 counts_per_step must be positive");
        Assert(_speedRpm >= 1 && _speedRpm <= 6000, "pd42s1 speed_rpm must be 1..6000");
        Assert(_accel >= 0 && _accel <= 200, "pd42s1 accel must be 0..200");
        Assert(_updateMs >= 2 && _updateMs <= 1000, "pd42s1 update_ms must be 2..1000");
    }

    void Pd42s1Motor::config_message() {
        log_info("    " << name() << " CAN address:" << _address << " counts_per_step:" << _countsPerStep << " speed:" << _speedRpm
                        << "rpm accel:" << _accel << " update:" << _updateMs << "ms");
    }

    void Pd42s1Motor::debug_message() {
        log_info("    " << name() << " address:" << _address << " synced:" << (_synced ? "yes" : "no")
                        << " online:" << (_online ? "yes" : "no") << " disabled:" << (_disabled ? "yes" : "no") << " steps:" << _stepCount
                        << " driver position:" << _driverPosition);
    }

    void Pd42s1Motor::set_disable(bool disable) {
        // Called from IRAM_ATTR context (MotorDriver::set_disable is invoked
        // with interrupts disabled in some paths), so this only records the
        // request; the CAN task performs the actual bus transaction.
        _disableRequested = disable;
        _disablePending   = true;
    }

    void IRAM_ATTR Pd42s1Motor::set_direction_isr(bool dir) {
        _direction = dir;
    }

    void IRAM_ATTR Pd42s1Motor::step_isr() {
        _stepCount += _direction ? 1 : -1;
    }

    IsrStepFn Pd42s1Motor::isr_step_fn() {
        return pd42s1_step_thunk;
    }

    IsrDirFn Pd42s1Motor::isr_dir_fn() {
        return pd42s1_dir_thunk;
    }

    uint32_t Pd42s1Motor::target_position() const {
        double scaled = double(_stepCount) * double(_countsPerStep);
        if (_invertDirection) {
            scaled = -scaled;
        }
        int64_t delta = int64_t(llround(scaled));
        // The driver position wraps at 2^32; truncating the sum to uint32_t
        // implements that wrap without signed-overflow undefined behavior.
        return uint32_t(uint64_t(int64_t(_basePosition) + delta));
    }

    bool Pd42s1Motor::transact(
        const uint8_t* request, size_t requestLen, uint8_t* response, size_t responseCapacity, size_t& responseLen, uint32_t timeoutMs) {
        responseLen    = 0;
        _lastErrorCode = 0;
        if (!_bus || !_bus->ready() || requestLen == 0) {
            return false;
        }

        CanBus::Lock lock(*_bus);

        // Start from a clean receive queue: a leftover byte from an earlier
        // timed-out transaction would otherwise look like the start of this
        // response, and a stale frame could even carry the expected length.
        _bus->flush_receive();
        if (!_bus->send_message(request, requestLen, timeoutMs)) {
            _bus->flush_receive();
            _bus->flush_transmit();
            return false;
        }

        const uint32_t deadline = now_ms() + timeoutMs;
        size_t         total    = 0;
        bool           complete = false;
        while (true) {
            uint32_t now = now_ms();
            if (int32_t(deadline - now) <= 0) {
                break;
            }
            if (total >= responseCapacity) {
                break;
            }

            size_t received = _bus->receive_chunk(response + total, responseCapacity - total, deadline - now);
            if (received == 0) {
                break;
            }
            total += received;

            size_t expected = Pd42s1Protocol::expected_response_length(response, total);
            if (expected == SIZE_MAX) {
                break;
            }
            if (expected != 0 && total >= expected) {
                complete = (total == expected);
                break;
            }
        }

        bool ok = false;
        if (complete && Pd42s1Protocol::valid_response(response, total) && response[1] == request[1] && response[2] == request[2]) {
            if (total == 6 && response[3] >= 0xE1 && response[3] <= 0xE6) {
                // The driver answered but rejected the command.  Report the
                // manual's error code (0xE1..0xE6) instead of letting it hide
                // behind the generic "not acknowledged" warnings.
                char message[64];
                snprintf(message, sizeof(message), "command error 0x%02X (%s)", response[3], error_code_name(response[3]));
                _lastErrorCode = response[3];
                warn_throttled(now_ms(), message);
            } else {
                ok = true;
            }
        }

        if (!ok) {
            _bus->flush_receive();
            _bus->flush_transmit();
            return false;
        }
        responseLen = total;
        return true;
    }

    bool Pd42s1Motor::transact_retry(
        const uint8_t* request, size_t requestLen, uint8_t* response, size_t responseCapacity, size_t& responseLen, uint32_t timeoutMs, uint8_t attempts) {
        for (uint8_t attempt = 0; attempt < attempts; ++attempt) {
            if (transact(request, requestLen, response, responseCapacity, responseLen, timeoutMs)) {
                return true;
            }
            if (attempt + 1 < attempts) {
                vTaskDelay(pdMS_TO_TICKS(4));
            }
        }
        return false;
    }

    bool Pd42s1Motor::clear_fault() {
        uint8_t request[Pd42s1Protocol::MAX_FRAME_LEN];
        uint8_t response[Pd42s1Protocol::MAX_FRAME_LEN];
        size_t  responseLen = 0;
        uint8_t address     = uint8_t(_address);

        // 0xF9 releases the stall protection; 0xFB is deliberately avoided
        // because a broadcast 0xFB made the V1.6 drivers stop answering in
        // bench testing.
        size_t len = Pd42s1Protocol::build_clear_stall_command(address, request, sizeof(request));
        if (!transact_retry(request, len, response, sizeof(response), responseLen, TRANSACTION_TIMEOUT_MS, 2) ||
            !Pd42s1Protocol::parse_ack(response, responseLen)) {
            warn_throttled(now_ms(), "clearing stall protection failed");
            return false;
        }

        // Clearing the state also clears the enable state; enable the motor
        // again so the next sync can immediately stream positions.
        len = Pd42s1Protocol::build_enable_command(address, true, request, sizeof(request));
        if (!transact_retry(request, len, response, sizeof(response), responseLen, TRANSACTION_TIMEOUT_MS, 2) ||
            !Pd42s1Protocol::parse_ack(response, responseLen)) {
            warn_throttled(now_ms(), "re-enable after clearing abnormal state failed");
            return false;
        }

        _lastErrorCode = 0;
        _readFailures  = 0;
        _writeFailures = 0;
        _synced        = false;  // force a fresh version/position sync
        _targetDirty   = true;
        _disabled      = false;
        _online        = true;
        log_info("CAN: " << name() << " stall protection released (0xF9), motor re-enabled");
        return true;
    }

    bool Pd42s1Motor::sync() {
        uint8_t request[Pd42s1Protocol::MAX_FRAME_LEN];
        uint8_t response[Pd42s1Protocol::MAX_FRAME_LEN];
        size_t  responseLen = 0;
        uint8_t address     = uint8_t(_address);

        size_t  len             = Pd42s1Protocol::build_read_command(address, Pd42s1Protocol::FUNC_READ_VERSION, request, sizeof(request));
        uint8_t firmwareVersion = 0;
        uint8_t hardwareVersion = 0;
        bool    ok              = transact_retry(request, len, response, sizeof(response), responseLen, TRANSACTION_TIMEOUT_MS, 2) &&
                     Pd42s1Protocol::parse_version(response, responseLen, firmwareVersion, hardwareVersion);
        if (!ok) {
            // A driver that was left at 0x6F = 1 never answers.  Re-enable
            // echo on this address once, wait for it to take effect, and try
            // again.  This is the only 0x6F traffic in the normal motor path;
            // the vendor example simply assumes the factory default.
            _bus->enable_driver_echo(address);
            vTaskDelay(pdMS_TO_TICKS(20));
            ok = transact_retry(request, len, response, sizeof(response), responseLen, TRANSACTION_TIMEOUT_MS, 2) &&
                 Pd42s1Protocol::parse_version(response, responseLen, firmwareVersion, hardwareVersion);
        }
        if (!ok) {
            _readFailures++;
            return false;
        }

        len              = Pd42s1Protocol::build_read_command(address, Pd42s1Protocol::FUNC_READ_POSITION, request, sizeof(request));
        int32_t position = 0;
        if (!transact_retry(request, len, response, sizeof(response), responseLen, TRANSACTION_TIMEOUT_MS, 2) ||
            !Pd42s1Protocol::parse_position(response, responseLen, position)) {
            _readFailures++;
            return false;
        }

        len = Pd42s1Protocol::build_enable_command(address, true, request, sizeof(request));
        if (!transact_retry(request, len, response, sizeof(response), responseLen, TRANSACTION_TIMEOUT_MS, 2) ||
            !Pd42s1Protocol::parse_ack(response, responseLen)) {
            _writeFailures++;
            return false;
        }

        _basePosition     = uint32_t(position);
        _lastTarget       = _basePosition;
        _driverPosition   = position;
        _targetDirty      = true;
        _disabled         = false;
        _disablePending   = false;
        _disableRequested = false;
        _synced           = true;
        _online           = true;
        _readFailures     = 0;
        _writeFailures    = 0;

        log_info("CAN: " << name() << " address:" << _address << " online, firmware V" << int(firmwareVersion / 10) << "."
                         << int(firmwareVersion % 10) << " hardware V" << int(hardwareVersion / 10) << "." << int(hardwareVersion % 10)
                         << " position:" << position);
        return true;
    }

    void Pd42s1Motor::apply_disable(uint32_t nowMs) {
        if (int32_t(nowMs - _nextDisableMs) < 0) {
            return;
        }
        _nextDisableMs = nowMs + RETRY_MS;

        uint8_t request[Pd42s1Protocol::MAX_FRAME_LEN];
        uint8_t response[Pd42s1Protocol::MAX_FRAME_LEN];
        size_t  responseLen = 0;
        bool    disable     = _disableRequested;

        size_t len = Pd42s1Protocol::build_enable_command(uint8_t(_address), !disable, request, sizeof(request));
        if (transact_retry(request, len, response, sizeof(response), responseLen, TRANSACTION_TIMEOUT_MS, 2) &&
            Pd42s1Protocol::parse_ack(response, responseLen)) {
            _disabled       = disable;
            _disablePending = false;
            _online         = true;
            _writeFailures  = 0;
            if (!disable) {
                // Re-send the target after enabling so the driver is not left
                // holding a stale target from before it was disabled.
                _targetDirty = true;
            }
            log_info("CAN: " << name() << " motor " << (disable ? "disabled" : "enabled"));
        } else {
            _online = false;
            _writeFailures++;
            if (_writeFailures >= 3) {
                _writeFailures = 0;
                _synced        = false;  // resync re-enables command echo and retries
            }
            warn_throttled(nowMs, "motor enable/disable command failed");
        }
    }

    bool Pd42s1Motor::send_target(uint32_t target, uint32_t timeoutMs) {
        uint8_t request[Pd42s1Protocol::MAX_FRAME_LEN];
        uint8_t response[Pd42s1Protocol::MAX_FRAME_LEN];
        size_t  responseLen = 0;

        size_t len = Pd42s1Protocol::build_position_command(
            uint8_t(_address), uint8_t(_accel), uint16_t(_speedRpm), target, request, sizeof(request));
        if (!transact_retry(request, len, response, sizeof(response), responseLen, timeoutMs, 2) ||
            !Pd42s1Protocol::parse_ack(response, responseLen)) {
            _online = false;
            _writeFailures++;
            // Retry even if the target has not changed: the driver may have
            // missed this command, and while the machine is idle there will be
            // no new target to trigger another update.
            _targetDirty = true;
            warn_throttled(now_ms(), "position command was not acknowledged");
            if (_lastErrorCode) {
                // The driver answered with a fault code (0xE1..0xE6).  Clear
                // the abnormal state and re-enable it; the next can_poll()
                // pass re-synchronises and re-sends the target.
                warn_throttled(now_ms(), "driver reported an error; clearing abnormal state");
                clear_fault();
                return false;
            }
            if (_writeFailures >= 3) {
                _writeFailures = 0;
                _synced        = false;  // resync (and re-enable echo) after repeated failures
            }
            return false;
        }

        _lastTarget   = target;
        _targetDirty  = false;
        _online       = true;
        _writeFailures = 0;
        log_verbose("CAN: " << name() << " target:" << int32_t(target) << " steps:" << _stepCount);
        return true;
    }

    void Pd42s1Motor::poll_position() {
        uint8_t request[Pd42s1Protocol::MAX_FRAME_LEN];
        uint8_t response[Pd42s1Protocol::MAX_FRAME_LEN];
        size_t  responseLen = 0;

        size_t  len = Pd42s1Protocol::build_read_command(uint8_t(_address), Pd42s1Protocol::FUNC_READ_POSITION, request, sizeof(request));
        int32_t position = 0;
        if (transact_retry(request, len, response, sizeof(response), responseLen, TRANSACTION_TIMEOUT_MS, 2) &&
            Pd42s1Protocol::parse_position(response, responseLen, position)) {
            _driverPosition = position;
            _online         = true;
            _readFailures   = 0;
            log_verbose("CAN: " << name() << " driver position:" << position << " target:" << int32_t(_lastTarget));
        } else {
            _online = false;
            _readFailures++;
            warn_throttled(now_ms(), "position readback failed");
            if (_readFailures >= 3) {
                _readFailures = 0;
                _synced       = false;  // resync (and re-enable echo) after repeated failures
            }
        }
    }

    void Pd42s1Motor::warn_throttled(uint32_t nowMs, const char* message) {
        if (_lastWarnMs != 0 && nowMs - _lastWarnMs < WARN_INTERVAL_MS) {
            return;
        }
        _lastWarnMs = nowMs;
        log_warn("CAN: " << name() << " address:" << _address << ": " << message);
    }

    void Pd42s1Motor::can_poll(uint32_t nowMs) {
        if (!_bus || !_bus->ready()) {
            return;
        }

        if (!_synced) {
            if (int32_t(nowMs - _nextSyncMs) < 0) {
                return;
            }
            _nextSyncMs = nowMs + RETRY_MS;
            if (sync()) {
                _nextUpdateMs = nowMs;
                _nextStatusMs = nowMs + _statusPollMs;
            } else {
                _online = false;
                warn_throttled(nowMs, "driver did not answer CAN init sequence");
            }
            return;
        }

        if (_disablePending) {
            apply_disable(nowMs);
        }

        if (!_disabled) {
            if (int32_t(nowMs - _nextUpdateMs) >= 0) {
                _nextUpdateMs   = nowMs + _updateMs;
                uint32_t target = target_position();
                if (_targetDirty || target != _lastTarget) {
                    send_target(target, TRANSACTION_TIMEOUT_MS);
                }
            }
        }

        if (_statusPollMs && int32_t(nowMs - _nextStatusMs) >= 0) {
            _nextStatusMs = nowMs + _statusPollMs;
            poll_position();
        }
    }

    // Configuration registration
    namespace {
        MotorFactory::InstanceBuilder<Pd42s1Motor> registration("pd42s1");
    }
}

#endif  // MAX_N_CAN
