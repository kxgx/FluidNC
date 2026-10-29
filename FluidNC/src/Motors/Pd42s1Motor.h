// Copyright (c) 2026 - FluidNC contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

/*
    Motor driver for ALIENTEK (正点原子) PD42S1 closed-loop stepper drivers
    driven over CAN.

    FluidNC still generates its normal step/direction stream; StepStick-style
    pin toggling is replaced by a small cumulative step counter updated from
    the step ISR.  A worker task in the CAN bus turns that counter into
    absolute position commands (0xF2) that the driver's own closed-loop
    controller follows, so the machine keeps Grbl-style kinematics, homing and
    limit handling while the motor driver handles current control and encoder
    feedback.

    Wiring and configuration notes are in Pd42s1Motor.md.
*/

#pragma once

#include "MotorDriver.h"

#if MAX_N_CAN

#    include "Machine/CanBus.h"

namespace MotorDrivers {
    class Pd42s1Motor final : public MotorDriver, public Machine::CanBusClient {
    public:
        Pd42s1Motor(const char* name) : MotorDriver(name) { Machine::CanBus::mark_driver_configured(); }
        ~Pd42s1Motor() override { Machine::CanBus::unmark_driver_configured(); }

        // MotorDriver overrides
        void init() override;
        bool set_homing_mode(bool isHoming) override { return true; }
        bool can_self_home() override { return false; }
        void set_disable(bool disable) override;
        void validate() override;

        // CanBusClient override -- called from the CAN worker task.
        void    can_poll(uint32_t now_ms) override;
        uint8_t bus_address() const override { return uint8_t(_address); }

        // Step ISR entry points.  These are non-virtual and IRAM_ATTR for the
        // same reason as UnipolarMotor's: the linker puts virtual function
        // tables in flash, and the step ISR can run while flash is disabled.
        void      step_isr();
        void      set_direction_isr(bool dir);
        IsrStepFn isr_step_fn() override;
        IsrDirFn  isr_dir_fn() override;

        void group(Configuration::HandlerBase& handler) override {
            // @config address
            // @default 1
            // @tuning per-machine
            // CAN slave address of this driver, 1..255.  Must match the value
            // written to the driver with setting 0x60 (set slave address).
            // Address 0 is the broadcast address and cannot be used here.
            handler.item("address", _address, 1, 255);

            // @config counts_per_step
            // @default 1.0
            // @tuning per-machine
            // Driver position units per FluidNC step.  The PD42S1 position
            // commands use 51200 units per revolution, so this is
            // 51200 / (steps_per_mm * mm_per_rev) for the axis.  Set it to 1.0
            // when steps_per_mm has been chosen to match the driver units.
            handler.item("counts_per_step", _countsPerStep, 0.000001f, 1000000.0f);

            // @config invert_direction
            // @default false
            // @tuning typical
            // Reverses the direction in which this motor follows the step
            // stream; use it instead of swapping motor phases on a closed-loop
            // driver that cannot be rewired.
            handler.item("invert_direction", _invertDirection);

            // @config speed_rpm
            // @default 600
            // @tuning per-machine
            // Speed in RPM that the driver is allowed to use while following
            // the position stream.  Must be at least the axis's maximum speed
            // (max_rate_mm_per_min converted to motor RPM) or the motor will
            // lag behind the commanded position.  The firmware derives the
            // required RPM from max_rate_mm_per_min, steps_per_mm and
            // counts_per_step and raises this value if it is too low.
            handler.item("speed_rpm", _speedRpm, 1, 6000);

            // @config accel
            // @default 200
            // @tuning per-machine
            // Acceleration in revolutions/second^2 (0..200) that the driver is
            // allowed to use while following the position stream.  The
            // firmware streams small position increments, so a high value here
            // keeps the motor tracking closely.  The firmware derives the
            // required value from acceleration_mm_per_sec2, steps_per_mm and
            // counts_per_step and raises this value if it is too low.
            handler.item("accel", _accel, 0, 200);

            // @config update_ms
            // @default 20
            // @tuning typical
            // Interval in milliseconds between position commands.  The driver
            // interpolates between them.  Smaller values track the step stream
            // more closely at the cost of more CAN traffic.
            handler.item("update_ms", _updateMs, 2, 1000);

            // @config status_poll_ms
            // @default 1000
            // @tuning typical
            // Interval in milliseconds between driver position readbacks, used
            // for diagnostics.  Set to 0 to disable readbacks.
            handler.item("status_poll_ms", _statusPollMs, 0, 60000);
        }

    protected:
        void config_message() override;
        void debug_message() override;

    private:
        bool sync();
        bool transact(
            const uint8_t* request, size_t requestLen, uint8_t* response, size_t responseCapacity, size_t& responseLen, uint32_t timeoutMs);
        bool transact_retry(
            const uint8_t* request, size_t requestLen, uint8_t* response, size_t responseCapacity, size_t& responseLen, uint32_t timeoutMs, uint8_t attempts);
        bool clear_fault();
        void     apply_disable(uint32_t nowMs);
        bool     send_target(uint32_t target, uint32_t timeoutMs);
        bool     send_target_nowait(uint32_t target);
        bool     read_position(int32_t& position);
        void     verify_final_position(uint32_t nowMs);
        void     poll_position();
        uint32_t target_position() const;
        uint32_t planner_target_position() const;
        void     warn_throttled(uint32_t nowMs, const char* message);

        Machine::CanBus* _bus = nullptr;

        // Configuration
        int32_t  _address         = 1;
        float    _countsPerStep   = 1.0f;
        bool     _invertDirection = false;
        int32_t  _speedRpm        = 600;
        int32_t  _accel           = 200;
        uint32_t _updateMs        = 20;
        uint32_t _statusPollMs    = 1000;

        // State written by the step ISR.  These are plain members of a small
        // heap object (internal DRAM on the ESP32), so reading and writing
        // them from the ISR does not touch flash.
        volatile int32_t _stepCount = 0;
        volatile bool    _direction = true;

        // State shared between the ISR-ish disable request and the CAN task.
        volatile bool _disableRequested = false;
        volatile bool _disablePending   = false;

        // CAN task state
        bool     _synced         = false;
        bool     _everSynced     = false;  // first sync adopts the driver position
        bool     _online         = false;
        bool     _disabled       = false;
        uint32_t _nextSyncMs     = 0;
        uint32_t _nextUpdateMs   = 0;
        uint32_t _nextStatusMs   = 0;
        uint32_t _nextDisableMs  = 0;
        uint32_t _lastWarnMs     = 0;
        uint32_t _basePosition   = 0;
        int32_t  _baseStepCount  = 0;  // step counter value at the last sync
        int32_t  _basePlannerSteps = 0;  // planner step position at the last sync
        uint32_t _lastTarget     = 0;
        uint32_t _lastTargetChangeMs = 0;
        bool     _finalTargetAcked   = false;
        bool     _targetDirty    = true;
        int32_t  _driverPosition = 0;

        // Fast final-position check: after the last target is acknowledged,
        // read the driver's actual position and re-send the target until it
        // matches.  This runs only when the target has stopped changing.
        bool     _verifyPending  = false;
        uint32_t _nextVerifyMs   = 0;
        uint8_t  _verifyAttempts = 0;

        // Consecutive failed transactions.  A marginal CAN bus drops the odd
        // response; only after several failures in a row is the driver
        // considered lost and re-synchronised.
        uint8_t _writeFailures = 0;
        uint8_t _readFailures  = 0;

        // Status byte of the last non-OK response (0xE1..0xE6); zero when the
        // last transaction succeeded.  Used to trigger automatic clearing of
        // the driver's abnormal state.
        uint8_t _lastErrorCode = 0;
    };
}

#endif  // MAX_N_CAN
