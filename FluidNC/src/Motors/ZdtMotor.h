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
            // @default 3200
            // @tuning per-machine
            // Driver position units per motor revolution.  3200 is the factory
            // 16-microstep value for a 1.8 degree motor.
            handler.item("counts_per_rev", _countsPerRev, 1, 1000000);

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
            // @tuning per-machine
            // Maximum following speed in RPM.
            handler.item("speed_rpm", _speedRpm, 1, 6000);

            // @config accel
            // @default 80
            // @tuning per-machine
            // Driver acceleration. For Emm this is the 0..200 ramp setting; for
            // X it is converted to RPM/s. Keep it high enough to follow the planner.
            handler.item("accel", _accel, 0, 200);

            // @config update_ms
            // @default 20
            // @tuning typical
            // Interval between streamed relative-position commands.
            handler.item("update_ms", _updateMs, 2, 1000);
        }

        bool set_homing_mode(bool isHoming) override { return true; }
        bool can_self_home() override { return true; }
        void set_axis(axis_t axis) override { _axis = axis; }
        void set_block_target(axis_t axis, int32_t targetSteps, float feedMmMin, float accelMmSec2) override {}
        void stop_motion() override;
        void resume_motion() override;
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
        bool           send_move(int32_t delta_counts);
        bool           transact(const uint8_t* request, size_t length, uint8_t function, uint8_t* response, size_t& response_len);

        Machine::CanBus* _bus = nullptr;
        axis_t           _axis = X_AXIS;

        int32_t    _address      = 1;
        std::string _firmwareName = "Emm";
        std::string _checksumName = "6B";
        int32_t  _countsPerRev = 3200;
        float    _countsPerStep = 1.0f;
        int32_t  _speedRpm     = 600;
        int32_t  _accel        = 80;
        uint32_t _updateMs     = 20;
        bool     _invertDirection = false;

        ZdtFirmware _firmware = ZdtFirmware::Emm;
        ZdtChecksum _checksum = ZdtChecksum::Fixed6B;

        volatile int32_t _stepCount = 0;
        volatile bool    _direction = false;
        volatile bool    _disableRequested = false;
        volatile bool    _stopRequested = false;

        int32_t  _lastSentSteps = 0;
        double   _residual      = 0.0;
        bool     _enabled       = false;
        bool     _synced        = false;
        uint32_t _nextPollMs    = 0;
    };
}

#endif
