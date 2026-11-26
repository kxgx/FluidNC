// Copyright (c) 2026 - FluidNC contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include "Config.h"

#if MAX_N_CAN

#    include "Can/CanBus.h"
#    include "Motors/MotorDriver.h"
#    include "Can/Protocols/Private/Zdt/ZdtProtocol.h"

#    include <string>

namespace MotorDrivers {
    // Zhang Datou XS-series closed-loop stepper on the shared CAN bus.
    //
    // Motion is streamed as relative position deltas, because the vendor's
    // CAN position mode is relative to the previously commanded target and
    // the segment counter is the authoritative FluidNC coordinate.  This
    // avoids any absolute-position base mapping that a soft reset could
    // invalidate.
    class ZdtMotor : public MotorDriver, public Machine::CanBusClient {
    public:
        ZdtMotor(const char* name);
        ~ZdtMotor() override;

        void init() override;
        void validate() override;
        void config_message() override;

        void group(Configuration::HandlerBase& handler) override {
            // @config address
            // @default 1
            // @tuning per-machine
            // CAN slave address of this Zhang Datou driver.  Must be unique on
            // the bus and must match the address configured in the driver.
            handler.item("address", _address, 1, 16);

            // @config firmware
            // @default "Emm"
            // @tuning per-machine
            // ZDT firmware family: Emm (pulse-based, default) or X (angle-based).
            handler.item("firmware", _firmwareName, 1, 3);

            // @config checksum
            // @default "6B"
            // @tuning per-machine
            // ZDT checksum mode: 6B (fixed), XOR or CRC8.  It must match the
            // Checksum setting in the driver.
            handler.item("checksum", _checksumName, 2, 3);

            // @config counts_per_rev
            // @default 0
            // @tuning per-machine
            // Driver position units per motor revolution.  Omit (0) to derive it
            // from the axis human mechanics (full_steps_per_rev x microsteps).
            handler.item("counts_per_rev", _countsPerRev, 0, 1000000);

            // @config counts_per_step
            // @default 1.0
            // @tuning per-machine
            // Driver position units per FluidNC step.
            handler.item("counts_per_step", _countsPerStep, 0.000001f, 1000000.0f);

            // @config invert_direction
            // @default false
            // @tuning typical
            // Invert the direction of the streamed position deltas.
            handler.item("invert_direction", _invertDirection);

            // @config speed_rpm
            // @default 600
            // @ignore_drift protocol header owns the symbolic default/limits
            // @tuning per-machine
            // Maximum following speed in RPM.  X42S free-protocol speed fields
            // are limited to 3000 RPM, so values above that are rejected.
            handler.item("speed_rpm", _speedRpm, 1, 3000);

            // @config accel
            // @default 80
            // @ignore_drift protocol header owns the symbolic default/limits
            // @tuning per-machine
            // Driver acceleration. For Emm this is the native 0..255 ramp
            // setting; for X it is the native RPM/s value (0..65535).
            handler.item("accel", _accel, 0, 65535);

            // @config update_ms
            // @default 20
            // @ignore_drift protocol header owns the symbolic default/limits
            // @tuning typical
            // Interval between streamed relative-position commands.
            handler.item("update_ms", _updateMs, 2, 1000);

            // @config mode
            // @default "position_stream"
            // @tuning per-machine
            // Motion contract for this motor.  Only position_stream is
            // implemented: the driver receives a relative 0xFD position
            // increment every update_ms.  Future modes must fail closed.
            handler.item("mode", _modeName, 1, 32);

            // @config position_mode
            // @default "relative_previous"
            // @tuning per-machine
            // Position command base.  Only relative_previous (the ZDT 0xFD
            // mode-0 behavior used by the current smooth streaming path) is
            // implemented; absolute/current-position modes are rejected.
            handler.item("position_mode", _positionModeName, 1, 32);

            // @config speed_source
            // @default "fixed"
            // @tuning per-machine
            // fixed: send speed_rpm on every command, exactly as before.
            // planner: derive the ZDT following speed from the actual
            // executed step delta, so G-code feed/override changes and
            // homing/jog ramps are reflected without changing the position
            // stream.  speed_rpm remains the upper clamp.  planner is an
            // experimental Emm-only mode; fixed preserves the historical
            // smooth position stream byte-for-byte.
            handler.item("speed_source", _speedSourceName, 1, 16);

            // @config speed_margin_pct
            // @default 25
            // @ignore_drift protocol header owns the symbolic default/limits
            // @tuning per-machine
            // Catch-up margin used only by speed_source=planner.  The
            // following speed is raised by this percentage before the
            // speed_rpm clamp, so a short following lag can recover without
            // making the drive run at a fixed speed.  Ignored by fixed.
            handler.item("speed_margin_pct", _speedMarginPct, 0, 100);
        }

        bool set_homing_mode(bool isHoming) override { return true; }
        // ZDT free protocol has its own 0x9A homing command, but this driver
        // does not implement the 9A/3B state machine.  Do not tell FluidNC that
        // the drive can home without external limit switches.
        bool can_self_home() override { return false; }
        void set_axis(axis_t axis) override { _axis = axis; }
        void set_block_target(axis_t axis, int32_t targetSteps, float feedMmMin, float accelMmSec2) override {}
        void stop_motion() override;
        void resume_motion() override;
        void hold_at_limit() override { _stopRequested = true; }
        void set_disable(bool disable) override;
        void on_stepping_reset() override;

        IsrStepFn isr_step_fn() override;
        IsrDirFn  isr_dir_fn() override;

        uint8_t bus_address() const override { return uint8_t(_address); }
        void    can_poll(uint32_t now_ms) override;
        bool    is_synced() const override { return _synced; }
        void    on_position_zeroed() override;

        static void IRAM_ATTR step_thunk(MotorDriver* driver);
        static void IRAM_ATTR dir_thunk(MotorDriver* driver, bool dir);

        void IRAM_ATTR step_isr();
        void IRAM_ATTR set_direction_isr(bool dir);

    private:
        bool           send_enable(bool enable);
        bool           send_stop();
        bool           send_move(int32_t delta_counts, uint16_t speed_rpm);
        bool           transact(const uint8_t* request, size_t length, uint8_t function, uint8_t* response, size_t& response_len);
        void           report_motion_fault(const char* reason);

        Machine::CanBus* _bus = nullptr;
        axis_t           _axis = X_AXIS;

        int32_t    _address      = 1;
        std::string _firmwareName    = "Emm";
        std::string _checksumName    = "6B";
        std::string _modeName        = "position_stream";
        std::string _positionModeName = "relative_previous";
        std::string _speedSourceName = "fixed";
        int32_t  _countsPerRev = 0;  // 0 = derive from axis mechanics
        float    _countsPerStep = 1.0f;
        int32_t  _speedRpm     = ZdtProtocol::Defaults::SPEED_RPM;
        int32_t  _speedMarginPct = ZdtProtocol::Defaults::SPEED_MARGIN_PCT;
        int32_t  _accel        = ZdtProtocol::Defaults::ACCEL_EMM;
        uint32_t _updateMs     = ZdtProtocol::Defaults::UPDATE_MS;
        bool     _invertDirection = false;

        ZdtFirmware     _firmware     = ZdtFirmware::Emm;
        ZdtChecksum     _checksum     = ZdtChecksum::Fixed6B;
        ZdtPositionMode _positionMode = ZdtPositionMode::RelativePrevious;
        bool            _plannerSpeed = false;

        volatile int32_t _stepCount = 0;
        volatile bool    _direction = false;
        volatile bool    _disableRequested = false;
        volatile bool    _stopRequested = false;

        int32_t  _lastSentSteps = 0;
        double   _residual      = 0.0;
        double   _xAngleResidual = 0.0;
        bool     _enabled       = false;
        bool     _synced        = false;
        bool     _lastSendOk    = false;
        bool     _lastAckError  = false;
        uint32_t _stopFailures  = 0;
        uint32_t _ackFailures   = 0;
        uint32_t _txFailures    = 0;
        uint32_t _nextPollMs    = 0;
        uint32_t _lastSendMs    = 0;
        uint32_t _lastAckWarnMs = 0;
    };
}

#endif
