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
        constexpr uint32_t TRANSACTION_TIMEOUT_MS = 300;

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
        Assert(_address >= 1 && _address <= Machine::CanBus::MAX_CLIENTS, "zdt address must be 1..MAX_CLIENTS");
        Assert(_countsPerRev > 0, "zdt counts_per_rev must be positive");
        Assert(_countsPerStep > 0.0f, "zdt counts_per_step must be positive");
        Assert(_speedRpm >= 1 && _speedRpm <= 6000, "zdt speed_rpm must be 1..6000");
        Assert(_accel >= 0 && _accel <= 200, "zdt accel must be 0..200");
        Assert(_updateMs >= 2 && _updateMs <= 1000, "zdt update_ms must be 2..1000");

        if (string_util::equal_ignore_case(_firmwareName, "X")) {
            _firmware = ZdtFirmware::X;
        } else if (string_util::equal_ignore_case(_firmwareName, "Emm")) {
            _firmware = ZdtFirmware::Emm;
        } else {
            Assert(false, "zdt firmware must be Emm or X");
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
    }

    void ZdtMotor::config_message() {
        log_info("    " << name() << " CAN address:" << _address << " firmware:" << _firmwareName << " checksum:" << _checksumName
                        << " counts_per_rev:" << _countsPerRev << " counts_per_step:" << _countsPerStep);
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
    }

    void ZdtMotor::on_position_zeroed() {
        _lastSentSteps = _stepCount;
        _residual      = 0.0;
        _synced        = true;
    }

    bool ZdtMotor::transact(
        const uint8_t* request, size_t length, uint8_t function, uint8_t* response, size_t& response_len) {
        response_len = 0;
        if (!_bus || !_bus->ready() || length < 2) {
            return false;
        }
        CanBus::Lock lock(*_bus);
        _bus->flush_receive();
        if (!_bus->send_chunked(uint32_t(_address) << 8, request + 1, length - 1, TRANSACTION_TIMEOUT_MS)) {
            return false;
        }
        response_len = _bus->receive_chunk(response, 16, TRANSACTION_TIMEOUT_MS);
        return ZdtProtocol::parse_ack(response, response_len, function);
    }

    bool ZdtMotor::send_enable(bool enable) {
        uint8_t request[ZdtProtocol::MAX_MESSAGE];
        size_t  length = ZdtProtocol::build_enable(uint8_t(_address), enable, _checksum, request, sizeof(request));
        if (!length) {
            return false;
        }
        uint8_t response[16];
        size_t  response_len = 0;
        if (!transact(request, length, 0xF3, response, response_len)) {
            return false;
        }
        _enabled = enable;
        _synced  = true;
        return true;
    }

    bool ZdtMotor::send_stop() {
        uint8_t request[ZdtProtocol::MAX_MESSAGE];
        size_t  length = ZdtProtocol::build_stop(uint8_t(_address), _checksum, request, sizeof(request));
        if (!length) {
            return false;
        }
        uint8_t response[16];
        size_t  response_len = 0;
        return transact(request, length, 0xFE, response, response_len);
    }

    bool ZdtMotor::send_move(int32_t delta_counts) {
        uint8_t request[ZdtProtocol::MAX_MESSAGE];
        size_t  length = ZdtProtocol::build_relative_position(_firmware,
                                                             uint8_t(_address),
                                                             delta_counts,
                                                             uint16_t(_speedRpm),
                                                             uint8_t(_accel),
                                                             uint32_t(_countsPerRev),
                                                             _checksum,
                                                             request,
                                                             sizeof(request));
        if (!length) {
            return false;
        }
        uint8_t response[16];
        size_t  response_len = 0;
        return transact(request, length, 0xFD, response, response_len);
    }

    void ZdtMotor::can_poll(uint32_t now_ms) {
        if (!_bus || !_bus->ready() || _bus->stop_latched() || _bus->inhibited()) {
            return;
        }

        if (_disableRequested != !_enabled) {
            if (!send_enable(!_disableRequested)) {
                return;
            }
        }
        if (_stopRequested) {
            _stopRequested = false;
            send_stop();
            _lastSentSteps = _stepCount;
            _residual      = 0.0;
            return;
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
            return;
        }

        if (send_move(counts)) {
            _lastSentSteps = targetSteps;
        }
    }

    namespace {
        MotorFactory::InstanceBuilder<ZdtMotor> registration("zdt");
    }
}

#endif
