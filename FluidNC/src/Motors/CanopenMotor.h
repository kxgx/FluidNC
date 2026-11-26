// Copyright (c) 2026 - FluidNC contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

/*
    Motor driver for a standard CANopen (CiA301) / CiA402 drive.

    This is a standard-protocol driver: it assumes the CiA402 object dictionary
    and the standard COB-IDs, so it can drive any vendor whose product really
    speaks CANopen.  The motion model is the same as the other closed-loop CAN
    motors on this branch:

      * FluidNC still generates its normal step/direction stream; the step ISR
        only updates a cumulative step counter.
      * The CAN worker periodically converts the step counter to an absolute
        position in the drive's position units and writes it to the CiA402
        target position object (0x607A by default) with an expedited SDO
        download.
      * The CiA402 actual position (0x6064) and statusword (0x6041) are polled
        for diagnostics and to detect a lost / faulted / disabled drive, which
        forces a resynchronisation before another target is sent.
      * Enable/disable use the CiA402 state machine
        (shutdown -> switch on -> enable operation, and shutdown for disable).

    Only expedited (<= 4 byte) SDO transfers are used and only 11-bit standard
    CANopen frames; both are what CiA301 devices must support for the objects
    used here.

    IMPORTANT / UNVERIFIED: the manuals available for the Zhang Datou (张大头)
    X42S V2.0 and X28S V2.0 do not include a CANopen tutorial or an object
    dictionary.  This driver therefore assumes the standard CiA402 defaults.
    It has NOT been tested against real hardware.  Verify the object dictionary
    on the actual drive before moving any axis; the object indices and SDO
    COB-IDs can be overridden per motor.  Homing is FluidNC step-stream homing
    (set_homing_mode() is a no-op), not CiA402 homing mode.
*/

#pragma once

#include "Config.h"

#if MAX_N_CAN

#    include "Can/CanBus.h"
#    include "Can/Protocols/Standard/Canopen/CanopenProtocol.h"
#    include "Can/Drivers/Canopen/CanopenClient.h"
#    include "Motors/MotorDriver.h"

#    include <cstdint>

namespace MotorDrivers {
    class CanopenMotor final : public MotorDriver, public Machine::CanBusClient {
    public:
        CanopenMotor(const char* name);
        ~CanopenMotor() override;

        void init() override;
        void validate() override;

        bool set_homing_mode(bool isHoming) override { return true; }
        bool can_self_home() override { return false; }
        void set_axis(axis_t axis) override { _axis = axis; }
        void set_disable(bool disable) override;
        void stop_motion() override;
        void resume_motion() override;
        void on_stepping_reset() override;
        void on_position_zeroed() override;
        void on_diagnostic_complete() override;

        void    can_poll(uint32_t now_ms) override;
        uint8_t bus_address() const override { return _node.node_id; }
        bool    is_synced() const override { return _synced; }

        // Non-virtual entry points for the step ISR: the linker puts virtual
        // function tables in flash, which the ISR must not touch.  The IRAM_ATTR
        // attribute lives on the out-of-line definitions (see the .cpp), same as
        // the other motor drivers.
        void      step_isr();
        void      set_direction_isr(bool dir);
        IsrStepFn isr_step_fn() override;
        IsrDirFn  isr_dir_fn() override;

        void group(Configuration::HandlerBase& handler) override {
            // @config address
            // @default 1
            // @tuning per-machine
            // CANopen node id of this drive.  CiA301 calls this node_id; it must
            // be unique on the bus and must match the id configured in the
            // drive.  This firmware's shared CAN address bookkeeping limits the
            // value to 1..16 even though CiA301 allows 1..127.  Address 0 is the
            // NMT broadcast id and is not valid here.
            handler.item("address", _address, 1, 16);

            // @config node_id
            // @default 0
            // @tuning per-machine
            // Optional alias for address.  0 uses address; any other value must
            // equal address.  Vendor manuals that call the setting node_id can
            // use this key instead of address.
            handler.item("node_id", _nodeId, 0, 16);

            // @config counts_per_step
            // @default 1.0
            // @tuning per-machine
            // Drive position units (CiA402 position increments, usually encoder
            // counts) per FluidNC step.  Set it to 1.0 when steps_per_mm has
            // already been chosen in these units.
            handler.item("counts_per_step", _countsPerStep, 0.000001f, 1000000.0f);

            // @config counts_per_rev
            // @default 10000
            // @tuning per-machine
            // Drive position increments per motor revolution.  Used to convert
            // speed_rpm to the CiA402 profile velocity object and to size the
            // idle following-error window.  Check this against the drive's
            // encoder/multi-turn setting; the X42S/X28S manuals do not state a
            // CANopen default.
            handler.item("counts_per_rev", _countsPerRev, 1, 1000000);

            // @config invert_direction
            // @default false
            // @tuning typical
            // Reverses the direction in which the motor follows the step stream,
            // without swapping motor phases.
            handler.item("invert_direction", _invertDirection);

            // @config update_ms
            // @default 20
            // @tuning typical
            // Interval between streamed absolute target-position SDO writes.
            // Smaller values follow the step stream more closely at the cost of
            // more CAN traffic (each write is one 8-byte frame plus a response).
            handler.item("update_ms", _updateMs, 2, 1000);

            // @config status_poll_ms
            // @default 1000
            // @tuning typical
            // Interval between actual-position (0x6064) and statusword (0x6041)
            // readbacks, used for diagnostics and resynchronisation.  Set to 0
            // to disable background polling.
            handler.item("status_poll_ms", _statusPollMs, 0, 60000);

            // @config speed_rpm
            // @default 0
            // @tuning per-machine
            // Optional maximum profile speed in RPM, written to the CiA402
            // profile velocity object (0x6081) as counts_per_rev * rpm / 60
            // position units per second.  0 disables the write and leaves the
            // drive's own profile velocity untouched.  This assumes the drive
            // expresses profile velocity in position units/s; verify before use.
            handler.item("speed_rpm", _speedRpm, 0, 6000);

            // @config setpoint_toggle
            // @default false
            // @tuning per-machine
            // Also toggle CiA402 controlword bit 4 (new set-point) after each
            // target position write.  Some profile position implementations
            // only latch 0x607A on that edge; others (cyclic synchronous
            // position, vendor auto-latch) follow the object directly.  The
            // manual does not say which behaviour the X42S/X28S uses, so this
            // is opt-in rather than assumed.
            handler.item("setpoint_toggle", _setpointToggle);

            // @config controlword_index
            // @default 0x6040
            // @tuning per-machine
            // CiA402 controlword object index.  Override only when the drive
            // uses a non-standard object dictionary.
            handler.item("controlword_index", _controlwordIndex, 0, 0xFFFF);

            // @config statusword_index
            // @default 0x6041
            // @tuning per-machine
            // CiA402 statusword object index.
            handler.item("statusword_index", _statuswordIndex, 0, 0xFFFF);

            // @config mode_index
            // @default 0x6060
            // @tuning per-machine
            // CiA402 modes-of-operation object index.  The firmware reads the
            // mode display (mode_index + 1) for diagnostics but does not write
            // the mode; the drive must already be configured for a position
            // mode that accepts target-position updates.
            handler.item("mode_index", _modeIndex, 0, 0xFFFF);

            // @config target_position_index
            // @default 0x607A
            // @tuning per-machine
            // CiA402 target position object index (absolute, drive position
            // units).
            handler.item("target_position_index", _targetPositionIndex, 0, 0xFFFF);

            // @config actual_position_index
            // @default 0x6064
            // @tuning per-machine
            // CiA402 position actual value object index (readback).
            handler.item("actual_position_index", _actualPositionIndex, 0, 0xFFFF);

            // @config sdo_request_cobid
            // @default 0
            // @tuning per-machine
            // 11-bit COB-ID the controller uses to send SDO requests (CiA301
            // default 0x600 + node id).  0 selects the standard default.
            handler.item("sdo_request_cobid", _sdoRequestCobid, 0, 0x7FF);

            // @config sdo_response_cobid
            // @default 0
            // @tuning per-machine
            // 11-bit COB-ID the drive uses for SDO responses (CiA301 default
            // 0x580 + node id).  0 selects the standard default.
            handler.item("sdo_response_cobid", _sdoResponseCobid, 0, 0x7FF);
        }

    protected:
        void config_message() override;

    private:
        bool sync();
        bool apply_enable(bool enable);
        bool send_target(int32_t target);
        void poll_status(uint32_t now_ms);
        void resync_base(int32_t actual, const char* reason);
        void note_failure(const char* reason);
        void clear_failure();
        void report_driver_fault(const char* reason);
        void warn_throttled(uint32_t now_ms, const char* message);
        int32_t target_position() const;
        bool    write_profile_velocity(Machine::Canopen::Client& client);

        Machine::CanBus* _bus  = nullptr;
        axis_t           _axis = X_AXIS;

        // Configuration (validated by validate()).
        int32_t  _address            = 1;
        int32_t  _nodeId             = 0;
        float    _countsPerStep      = 1.0f;
        int32_t  _countsPerRev       = 10000;
        bool     _invertDirection    = false;
        uint32_t _updateMs           = 20;
        uint32_t _statusPollMs       = 1000;
        int32_t  _speedRpm           = 0;
        bool     _setpointToggle     = false;
        int32_t  _controlwordIndex   = 0x6040;
        int32_t  _statuswordIndex    = 0x6041;
        int32_t  _modeIndex          = 0x6060;
        int32_t  _targetPositionIndex = 0x607A;
        int32_t  _actualPositionIndex = 0x6064;
        int32_t  _sdoRequestCobid    = 0;
        int32_t  _sdoResponseCobid   = 0;

        // Effective node configuration, resolved in validate().
        Machine::Canopen::NodeConfig _node;
        bool                          _registered = false;
        int32_t                       _followingErrorLimit = 1000;

        // Step ISR state: only touched from the ISR and read by the CAN task.
        volatile int32_t _stepCount = 0;
        volatile bool    _direction = false;

        // Requests posted from the step/control path (never run bus I/O there).
        volatile bool _disableRequested = false;
        volatile bool _stopRequested    = false;
    volatile bool _holdLatched      = false;

        // CAN task state.
        bool     _enabled      = false;
        bool     _synced       = false;
        bool     _online       = false;
        bool     _faultLatched = false;
        bool     _velocityWriteWarned = false;
        bool     _targetDirty  = true;
        bool     _modeReported = false;
        int8_t   _mode         = 0;
        int32_t  _basePosition = 0;
        int32_t  _baseStepCount = 0;
        int32_t  _lastTarget   = 0;
        int32_t  _driverPosition = 0;
        int32_t  _lastStreamedSteps = 0;
        uint32_t _nextSyncMs   = 0;
        uint32_t _nextUpdateMs = 0;
        uint32_t _nextStatusMs = 0;
        uint32_t _nextEnableMs = 0;
        uint32_t _lastWarnMs   = 0;
        uint32_t _idleSinceMs  = 0;

        // Consecutive failed transactions.  A marginal bus drops the odd
        // response; only several failures in a row are treated as a lost drive.
        uint8_t _consecutiveFailures = 0;
    };
}

#endif  // MAX_N_CAN

