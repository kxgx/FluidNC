// Copyright (c) 2026 - FluidNC contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "Config.h"

#if MAX_N_CAN

#    include "ZdtMotor.h"

#    include "Machine/Axes.h"
#    include "Machine/MachineConfig.h"
#    include "Protocol.h"
#    include "Stepping.h"
#    include "string_util.h"

#    include <cmath>
#    include <cstring>

using namespace Machine;

namespace MotorDrivers {
    namespace {
        constexpr uint32_t TRANSACTION_TIMEOUT_MS = ZdtProtocol::Timing::TRANSACTION_TIMEOUT_MS;

        void IRAM_ATTR zdt_step_thunk(MotorDriver* driver) {
            static_cast<ZdtMotor*>(driver)->step_isr();
        }

        void IRAM_ATTR zdt_dir_thunk(MotorDriver* driver, bool dir) {
            static_cast<ZdtMotor*>(driver)->set_direction_isr(dir);
        }
    }

    ZdtMotor::ZdtMotor(const char* name) : MotorDriver(name) {
        Machine::CanBus::mark_driver_configured();
    }

    ZdtMotor::~ZdtMotor() {
        Machine::CanBus::unregister_configured_address(uint8_t(_address));
        Machine::CanBus::unmark_driver_configured();
    }

    void ZdtMotor::validate() {
        Assert(config != nullptr && config->_can != nullptr, "zdt motor driver requires a can: section");
        Assert(config->_can->configured(), "zdt motor driver requires can: tx_pin and rx_pin");
        Assert(_address >= ZdtProtocol::ADDRESS_MIN && _address <= Machine::CanBus::MAX_CLIENTS, "zdt address must be 1..MAX_CLIENTS");

        auto axisConfig = Axes::_axis[axis_index()];
        if (_countsPerRev == 0) {
            // New human-readable configuration: derive the driver's step count
            // from the mechanical declaration instead of asking the user to
            // compute it.  Derivation is only defined for one driver unit per
            // FluidNC step; a different counts_per_step would silently scale
            // the position stream.
            Assert(axisConfig->_fullStepsPerRev > 0 && axisConfig->_microsteps > 0,
                   "zdt counts_per_rev omitted but axis full_steps_per_rev/microsteps are not both set");
            Assert(_countsPerStep == 1.0f,
                   "zdt counts_per_rev is derived from microsteps only when counts_per_step is 1.0");
            _countsPerRev = int32_t(axisConfig->_fullStepsPerRev) * int32_t(axisConfig->_microsteps);
            log_info("ZDT CAN: address " << int(_address) << " derived counts_per_rev " << _countsPerRev << " from "
                                         << axisConfig->_fullStepsPerRev << " full steps x " << axisConfig->_microsteps
                                         << " microsteps");
        } else if (axisConfig->_fullStepsPerRev > 0 && axisConfig->_microsteps > 0) {
            const int32_t derived = int32_t(axisConfig->_fullStepsPerRev) * int32_t(axisConfig->_microsteps);
            Assert(_countsPerRev == derived,
                   "zdt counts_per_rev conflicts with axis full_steps_per_rev*microsteps; remove one or make them equal");
            Assert(_countsPerStep == 1.0f,
                   "zdt counts_per_step must be 1.0 when axis mechanics supply counts_per_rev");
        }
        Assert(_countsPerRev >= ZdtProtocol::Limits::COUNTS_PER_REV_MIN && _countsPerRev <= ZdtProtocol::Limits::COUNTS_PER_REV_MAX, "zdt counts_per_rev is outside the protocol range");
        Assert(_countsPerStep > 0.0f, "zdt counts_per_step must be positive");
        Assert(_speedRpm >= ZdtProtocol::Limits::SPEED_RPM_MIN && _speedRpm <= ZdtProtocol::Limits::SPEED_RPM_MAX, "zdt speed_rpm is outside the X42S free-protocol range");
        Assert(_speedMarginPct >= ZdtProtocol::Limits::SPEED_MARGIN_MIN && _speedMarginPct <= ZdtProtocol::Limits::SPEED_MARGIN_MAX, "zdt speed_margin_pct is outside the protocol range");
        Assert(_accel >= 0 && _accel <= 65535, "zdt accel must be 0..65535");
        Assert(_updateMs >= ZdtProtocol::Limits::UPDATE_MS_MIN && _updateMs <= ZdtProtocol::Limits::UPDATE_MS_MAX, "zdt update_ms is outside the protocol range");

        Assert(string_util::equal_ignore_case(_modeName, "position_stream"),
               "zdt mode must be position_stream; other motion modes are not implemented");
        if (string_util::equal_ignore_case(_positionModeName, "relative_previous")) {
            _positionMode = ZdtPositionMode::RelativePrevious;
        } else {
            Assert(false, "zdt position_mode must be relative_previous; absolute/current modes are not implemented");
        }
        if (string_util::equal_ignore_case(_speedSourceName, "fixed")) {
            _plannerSpeed = false;
        } else if (string_util::equal_ignore_case(_speedSourceName, "planner")) {
            _plannerSpeed = true;
        } else {
            Assert(false, "zdt speed_source must be fixed or planner");
        }

        if (string_util::equal_ignore_case(_firmwareName, "X")) {
            _firmware = ZdtFirmware::X;
        } else if (string_util::equal_ignore_case(_firmwareName, "Emm")) {
            _firmware = ZdtFirmware::Emm;
        } else {
            Assert(false, "zdt firmware must be Emm or X");
        }
        if (_firmware == ZdtFirmware::Emm) {
            Assert(_accel >= ZdtProtocol::Limits::ACCEL_EMM_MIN && _accel <= ZdtProtocol::Limits::ACCEL_EMM_MAX, "zdt Emm accel is the native 0..255 ramp setting");
        } else {
            Assert(_accel >= 1, "zdt X accel is RPM/s and must be at least 1");
        }
        // The streamed target must be able to cover the axis's configured
        // maximum feed.  This applies to fixed speed too: fixed does not mean
        // "any speed is safe" - a too-small RPM leaves an invisible following
        // error because ZDT streaming is open-loop with respect to position.
        const double requiredRpm = double(axisConfig->_maxRate) * double(axisConfig->_stepsPerMm) * double(_countsPerStep) /
                                   double(_countsPerRev);
        if (_plannerSpeed) {
            Assert(_firmware == ZdtFirmware::Emm, "zdt speed_source=planner currently requires firmware Emm");
            // The margin is applied before the speed_rpm clamp, so speed_rpm
            // must cover the margin too; otherwise the configured catch-up
            // headroom would silently disappear at maximum feed.
            const double requiredWithMargin = requiredRpm * (100.0 + double(_speedMarginPct)) / 100.0;
            Assert(requiredWithMargin <= double(ZdtProtocol::Limits::SPEED_RPM_MAX),
                   "zdt axis maximum feed plus speed_margin_pct needs >3000 RPM; lower max speed/accel or margin");
            // Tiny epsilon avoids a float-derived 100.0000001 becoming 101.
            const int required = int(std::ceil(requiredWithMargin - 1e-9));
            Assert(_speedRpm >= required,
                   "zdt speed_rpm is below the axis maximum feed plus speed_margin_pct for speed_source=planner");
        } else {
            Assert(requiredRpm <= double(ZdtProtocol::Limits::SPEED_RPM_MAX),
                   "zdt axis maximum feed needs >3000 RPM; lower max speed or use a larger distance_per_rev");
            const int required = int(std::ceil(requiredRpm - 1e-9));
            Assert(_speedRpm >= required,
                   "zdt speed_rpm is below the axis maximum feed (fixed speed stream would lag); raise speed_rpm");
        }

        if (string_util::equal_ignore_case(_checksumName, "XOR")) {
            _checksum = ZdtChecksum::Xor;
        } else if (string_util::equal_ignore_case(_checksumName, "CRC8")) {
            _checksum = ZdtChecksum::Crc8;
        } else if (string_util::equal_ignore_case(_checksumName, "6B")) {
            _checksum = ZdtChecksum::Fixed6B;
        } else {
            Assert(false, "zdt checksum must be 6B, XOR or CRC8");
        }

        Machine::CanBus::register_configured_address(uint8_t(_address));
    }

    void ZdtMotor::init() {
        config_message();
        _bus = config ? config->_can : nullptr;
        Assert(_bus != nullptr, "zdt motor driver requires a can: section");
        Assert(_bus->ready(), "CAN bus is not ready");
        Assert(_bus->control_ready(), "CAN control is not ready");
        _bus->add_client(this);
        Stepping::assignMotorDriver(axis_index(), dual_axis_index(), this);
        _lastSentSteps = _stepCount;
        _nextPollMs    = 0;
        _lastSendMs    = 0;
    }

    void ZdtMotor::config_message() {
        log_info("    " << name() << " CAN address:" << _address << " firmware:" << _firmwareName << " checksum:" << _checksumName
                        << " counts_per_rev:" << _countsPerRev << " counts_per_step:" << _countsPerStep
                        << " mode:" << _modeName << " speed_source:" << _speedSourceName);
    }

    void ZdtMotor::step_isr() {
        _stepCount += _direction ? 1 : -1;
    }

    void ZdtMotor::set_direction_isr(bool dir) {
        _direction = dir;
    }

    IsrStepFn ZdtMotor::isr_step_fn() {
        return zdt_step_thunk;
    }

    IsrDirFn ZdtMotor::isr_dir_fn() {
        return zdt_dir_thunk;
    }

    void ZdtMotor::set_disable(bool disable) {
        _disableRequested = disable;
    }

    void ZdtMotor::stop_motion() {
        _stopRequested = true;
    }

    void ZdtMotor::resume_motion() {}

    void ZdtMotor::on_stepping_reset() {
        // Relative streaming needs no absolute position base, but the next
        // delta must be measured from the current step counter, not from a
        // stale target.
        _lastSentSteps = _stepCount;
        _residual      = 0.0;
        _lastSendMs    = 0;
    }

    void ZdtMotor::on_position_zeroed() {
        _lastSentSteps  = _stepCount;
        _residual       = 0.0;
        _xAngleResidual = 0.0;
        _lastSendMs     = 0;
        _synced         = true;
    }

    bool ZdtMotor::transact(
        const uint8_t* request, size_t length, uint8_t function, uint8_t* response, size_t& response_len) {
        response_len      = 0;
        _lastSendOk       = false;
        _lastAckError     = false;
        if (!_bus || !_bus->ready() || length < 2) {
            return false;
        }
        CanBus::Lock lock(*_bus);
        _bus->flush_receive();
        ZdtProtocol::Frame frames[(ZdtProtocol::MAX_MESSAGE / ZdtProtocol::FRAME_PAYLOAD) + 1];
        const size_t count =
            ZdtProtocol::build_frames(uint8_t(_address), request + 1, length - 1, frames, sizeof(frames) / sizeof(frames[0]));
        if (!count) {
            return false;
        }
        for (size_t i = 0; i < count; ++i) {
            if (!_bus->send_frame(frames[i].id, true, frames[i].data, frames[i].length, TRANSACTION_TIMEOUT_MS)) {
                return false;
            }
        }
        // The relative move has already left the controller.  Record that fact
        // separately from the acknowledgement: a missing ACK must not cause the
        // same relative delta to be sent a second time.
        _lastSendOk = true;

        uint32_t responseId = 0;
        bool     extended   = false;
        response_len        = _bus->receive_frame(responseId, extended, response, ZdtProtocol::MAX_RESPONSE, TRANSACTION_TIMEOUT_MS);
        if (!response_len || !extended || uint8_t(responseId >> 8) != uint8_t(_address)) {
            return false;
        }
        if (response[0] == function && response_len >= 3) {
            const uint8_t status = response[1];
            if (status == ZdtProtocol::Status::ACK_ERROR || status == ZdtProtocol::Status::ACK_BAD_FORMAT || status == ZdtProtocol::Status::ACK_HOMING_FAIL) {
                _lastAckError = true;
                return false;
            }
        }
        return ZdtProtocol::parse_ack(response, response_len, function);
    }

    bool ZdtMotor::send_enable(bool enable) {
        _lastSendOk = false;
        uint8_t request[ZdtProtocol::MAX_MESSAGE];
        size_t  length = ZdtProtocol::build_enable(uint8_t(_address), enable, _checksum, request, sizeof(request));
        if (!length) {
            return false;
        }
        uint8_t response[ZdtProtocol::MAX_RESPONSE];
        size_t  response_len = 0;
        if (!transact(request, length, ZdtProtocol::Function::ENABLE, response, response_len)) {
            return false;
        }
        _enabled = enable;
        _synced  = true;
        // A new enable starts a new speed-following interval; otherwise a
        // stale timestamp from before a disable could produce a 1 RPM first
        // frame in planner mode.
        _lastSendMs = 0;
        return true;
    }

    bool ZdtMotor::send_stop() {
        _lastSendOk = false;
        uint8_t request[ZdtProtocol::MAX_MESSAGE];
        size_t  length = ZdtProtocol::build_stop(uint8_t(_address), _checksum, request, sizeof(request));
        if (!length) {
            return false;
        }
        uint8_t response[ZdtProtocol::MAX_RESPONSE];
        size_t  response_len = 0;
        return transact(request, length, ZdtProtocol::Function::STOP, response, response_len);
    }

    void ZdtMotor::report_motion_fault(const char* reason) {
        _synced           = false;
        _stopRequested    = true;
        _disableRequested = true;
        log_error("ZDT " << name() << ": " << reason << "; stopping all motors");
        Axes::set_disable(true, true);
        send_alarm(ExecAlarm::HardStop);
    }

    bool ZdtMotor::send_move(int32_t delta_counts, uint16_t speed_rpm) {
        _lastSendOk = false;
        uint8_t request[ZdtProtocol::MAX_MESSAGE];
        size_t  length = ZdtProtocol::build_relative_position(_firmware,
                                                             uint8_t(_address),
                                                             delta_counts,
                                                             speed_rpm,
                                                             uint16_t(_accel),
                                                             uint32_t(_countsPerRev),
                                                             _checksum,
                                                             request,
                                                             sizeof(request),
                                                             _firmware == ZdtFirmware::X ? &_xAngleResidual : nullptr,
                                                             uint8_t(_positionMode));
        if (!length) {
            return false;
        }
        uint8_t response[ZdtProtocol::MAX_RESPONSE];
        size_t  response_len = 0;
        return transact(request, length, ZdtProtocol::Function::POSITION, response, response_len);
    }

    void ZdtMotor::can_poll(uint32_t now_ms) {
        if (!_bus || !_bus->ready() || _bus->stop_latched() || _bus->inhibited()) {
            return;
        }

        if (_stopRequested) {
            bool sent = send_stop();
            // FE is idempotent; a lost acknowledgement must not cause a retry
            // storm, but a failed transmission must be retried.
            if (sent || _lastSendOk) {
                _stopRequested = false;
                _stopFailures  = 0;
                _lastSentSteps = _stepCount;
                _residual      = 0.0;
                _xAngleResidual = 0.0;
                _lastSendMs    = 0;
            } else if (++_stopFailures >= 3) {
                _stopRequested = false;
                _stopFailures  = 0;
                report_motion_fault("emergency stop could not be delivered");
            }
            return;
        }

        if (_disableRequested != !_enabled) {
            if (!send_enable(!_disableRequested)) {
                return;
            }
        }
        if (_disableRequested) {
            return;
        }

        if (int32_t(now_ms - _nextPollMs) < 0) {
            return;
        }
        _nextPollMs = now_ms + _updateMs;

        const int32_t targetSteps = _stepCount;
        const int32_t stepDelta   = targetSteps - _lastSentSteps;
        if (stepDelta == 0) {
            // No displacement to carry a speed field.  Restart the elapsed
            // interval so the first frame after an idle gap is not interpreted
            // as an extremely slow move.
            _lastSendMs = 0;
            return;
        }

        double scaled = double(stepDelta) * double(_countsPerStep) + _residual;
        if (_invertDirection) {
            scaled = -scaled;
        }
        int32_t counts = int32_t(llround(scaled));
        _residual      = scaled - double(counts);
        if (counts == 0) {
            _lastSentSteps = targetSteps;
            _lastSendMs    = 0;
            return;
        }

        // fixed: preserve the historical bytes exactly (always _speedRpm).
        // planner: derive the following speed from the step stream that was
        // actually executed; keep _speedRpm as the upper clamp.
        uint16_t commandSpeed = uint16_t(_speedRpm);
        if (_plannerSpeed) {
            uint32_t elapsed = _updateMs;
            if (_lastSendMs != 0) {
                uint32_t measured = now_ms - _lastSendMs;
                // A short/zero interval or a stale timestamp (worker stall,
                // disable/enable) must not collapse the following speed to
                // the 1 RPM floor; use the nominal update interval instead.
                if (measured != 0 && measured <= _updateMs * 10) {
                    elapsed = measured;
                }
            }
            commandSpeed = ZdtProtocol::speed_rpm_for_counts(
                counts, uint32_t(_countsPerRev), elapsed, uint16_t(_speedRpm), uint8_t(_speedMarginPct));
        }

        bool acknowledged = send_move(counts, commandSpeed);
        if (!_lastSendOk) {
            if (++_txFailures >= 3) {
                _txFailures = 0;
                report_motion_fault("relative move could not be transmitted");
            }
            return;
        }
        _txFailures = 0;
        if (_lastAckError) {
            // The driver explicitly rejected the command.  Do not keep
            // accumulating a target that it is not executing.
            report_motion_fault("driver rejected a relative move");
            return;
        }
        if (_lastSendOk) {
            // The command was transmitted.  Advance the relative-base even if
            // the response was lost or the driver's Response mode is None;
            // replaying a relative move after a lost ACK would move twice.
            _lastSentSteps = targetSteps;
            _lastSendMs    = now_ms;
            if (acknowledged) {
                _ackFailures = 0;
            } else {
                if (now_ms - _lastAckWarnMs > 5000) {
                    _lastAckWarnMs = now_ms;
                    log_warn("ZDT " << name() << ": relative move sent without acknowledged reply");
                }
                if (++_ackFailures >= 3) {
                    _ackFailures = 0;
                    report_motion_fault("repeated missing replies");
                }
            }
        }
    }

    namespace {
        MotorFactory::InstanceBuilder<ZdtMotor> registration("zdt");
    }
}

#endif
