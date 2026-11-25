// Copyright (c) 2021 -  Stefan de Bruijn
// Copyright (c) 2021 -  Mitch Bradley
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include "Configuration/Configurable.h"
// #include "Axes.h"
#include "Motor.h"
#include "Homing.h"

namespace MotorDrivers {
    class MotorDriver;
}

namespace Machine {
    class Axis : public Configuration::Configurable {
        axis_t  _axis;
        motor_t motorsWithSwitches();

    public:
        Axis(axis_t currentAxis) : _axis(currentAxis) {
            for (motor_t motor = 0; motor < MAX_MOTORS_PER_AXIS; ++motor) {
                _motors[motor] = nullptr;
            }
        }

        static const int MAX_MOTORS_PER_AXIS = 2;

        Motor*  _motors[MAX_MOTORS_PER_AXIS];
        Homing* _homing = nullptr;

        float _stepsPerMm   = 80.0f;
        float _maxRate      = 1000.0f;
        float _acceleration = 25.0f;
        float _maxTravel    = 1000.0f;

        // Optional human-readable mechanics.  When all three are supplied the
        // firmware derives steps_per_mm/deg, so users do not hand-compute the
        // controller's step resolution.  distance_per_rev is mm for linear
        // axes and degrees for rotary axes (default 360 for rotary).
        int32_t _fullStepsPerRev = 0;  // 200 for 1.8deg, 400 for 0.9deg
        int32_t _microsteps      = 0;  // 0 means "not specified"
        float   _distancePerRev  = 0;  // 0 means "not specified"

        // Human speed/acceleration names.  The legacy max_rate_mm_per_min /
        // acceleration_mm_per_sec2 remain accepted; for a rotary axis these
        // human fields use degrees.
        float _maxSpeedLinear = 0;
        float _maxSpeedRotary = 0;
        float _accelRotary    = 0;
        bool  _softLimits   = false;
        bool  _idleDisable  = true;

        // Optional independent soft-limit range.  When enabled, limitsMinPosition()
        // and limitsMaxPosition() use these absolute machine coordinates instead of
        // deriving the range from homing.mpos_mm and max_travel_mm.  This allows a
        // homed position of 0 with travel on both sides of it.
        bool  _useSoftLimitRange = false;
        float _softLimitMin      = 0.0f;
        float _softLimitMax      = 0.0f;

        // Configuration system helpers:
        void group(Configuration::HandlerBase& handler) override;
        void afterParse() override;
        void validate() override;

        // Checks if a motor matches this axis:
        bool hasMotor(const MotorDrivers::MotorDriver* const driver) const;
        bool hasDualMotor();

        float commonPulloff();
        float extraPulloff();

        void init();
        void config_motors();
        bool can_home();

        ~Axis();
    };
}
