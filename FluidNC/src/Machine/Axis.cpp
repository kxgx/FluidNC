#include "Axes.h"
#include "Axis.h"
#include "MachineConfig.h"  // config

#include <cstring>

namespace Machine {
    void Axis::group(Configuration::HandlerBase& handler) {
        // @config steps_per_mm
        // @default 80.0
        // @tuning per-machine
        // Controller-side step-pulse resolution for this axis -- really "steps per GCode
        // unit," despite the name: in G21 (mm) mode, a one-unit move issues this many step
        // pulses; for a linear (XYZ) axis in G20 (inches) mode, the step count is instead
        // multiplied by 25.4. A rotary (ABC) axis always issues this many pulses per unit,
        // regardless of G20/G21. If using a microstepping driver, this already needs to
        // include the microstep multiplier.
        handler.item("steps_per_mm", _stepsPerMm, 0.001, 100000.0);

        // @config full_steps_per_rev
        // @default 0
        // @tuning per-machine
        // Optional human-readable motor mechanics: full steps per revolution
        // (200 for a 1.8 degree motor, 400 for 0.9 degree).  When this,
        // microsteps and distance_per_rev are all supplied, the firmware
        // derives steps_per_mm; steps_per_mm is then optional.
        handler.item("full_steps_per_rev", _fullStepsPerRev, 0, 10000);

        // @config microsteps
        // @default 0
        // @tuning per-machine
        // Optional driver microstep setting.  This is the value the physical
        // driver is configured for; FluidNC uses it to derive step resolution.
        handler.item("microsteps", _microsteps, 0, 256);

        // @config distance_per_rev
        // @default 0
        // @tuning per-machine
        // Optional mechanical travel per motor revolution: mm for linear axes,
        // degrees for rotary axes (rotary defaults to 360 when omitted).
        handler.item("distance_per_rev", _distancePerRev, 0, 1000000.0);

        // @config max_speed_mm_per_min
        // @default 0
        // @tuning per-machine
        // Human name for a linear axis's maximum speed.  Overrides the legacy
        // max_rate_mm_per_min when supplied.
        handler.item("max_speed_mm_per_min", _maxSpeedLinear, 0, 250000.0);

        // @config max_speed_deg_per_min
        // @default 0
        // @tuning per-machine
        // Human name for a rotary axis's maximum speed, in degrees/minute.
        handler.item("max_speed_deg_per_min", _maxSpeedRotary, 0, 250000.0);

        // @config acceleration_deg_per_sec2
        // @default 0
        // @tuning per-machine
        // Human acceleration for a rotary axis, in degrees/sec^2.  Linear axes
        // use the existing acceleration_mm_per_sec2 field.
        handler.item("acceleration_deg_per_sec2", _accelRotary, 0, 100000.0);

        // @config max_rate_mm_per_min
        // @default 1000.0
        // @tuning per-machine
        // Maximum feed rate (rapids and feed moves alike are capped here) for this axis.
        handler.item("max_rate_mm_per_min", _maxRate, 0.001, 250000.0);

        // @config acceleration_mm_per_sec2
        // @default 25.0
        // @tuning per-machine
        // Acceleration used for this axis's motion ramps.
        handler.item("acceleration_mm_per_sec2", _acceleration, 0.001, 100000.0);

        // @config max_travel_mm
        // @default 1000.0
        // @tuning per-machine
        // Working length of the axis, measured from its position immediately after homing
        // pull-off. If a second limit switch exists at the far end of travel (for hard
        // limits), make sure this value stays short enough that a soft-limit alarm trips
        // before that second switch would be physically reached.
        handler.item("max_travel_mm", _maxTravel, 0.1, 10000000.0);

        // @config soft_limits
        // @default false
        // @tuning typical
        // When true, a move that would exceed max_travel_mm is aborted before it starts;
        // jogs are instead constrained to stop at the travel limit rather than alarming.
        // Relies on accurate machine position, so the axis should be homed first -- always
        // home before jogging or running GCode when soft limits are enabled.
        handler.item("soft_limits", _softLimits);

        // @config use_soft_limit_range
        // @default false
        // @tuning per-machine
        // When true, use the explicit soft_limit_min_mm / soft_limit_max_mm range
        // instead of deriving the range from homing.mpos_mm and max_travel_mm.
        // The explicit range is an absolute machine-coordinate range, so a homed
        // position of 0 can have travel on both the negative and positive sides.
        handler.item("use_soft_limit_range", _useSoftLimitRange);

        // @config soft_limit_min_mm
        // @default 0.0
        // @tuning per-machine
        // Absolute machine-coordinate lower bound used when use_soft_limit_range is true.
        handler.item("soft_limit_min_mm", _softLimitMin, -10000000.0, 10000000.0);

        // @config soft_limit_max_mm
        // @default 0.0
        // @tuning per-machine
        // Absolute machine-coordinate upper bound used when use_soft_limit_range is true.
        handler.item("soft_limit_max_mm", _softLimitMax, -10000000.0, 10000000.0);

        // @config idle_disable
        // @default true
        // Whether this axis participates in stepping.idle_ms auto-disable. Set to false to
        // keep this axis's motor(s) always enabled regardless of the global idle timeout --
        // e.g. a Z axis that would otherwise fall, or an RC servo axis that needs to stay
        // enabled to hold position.
        handler.item("idle_disable", _idleDisable);

        handler.section("homing", _homing);

        char tmp[7];
        tmp[0] = 0;
        strcat(tmp, "motor");

        for (size_t i = 0; i < MAX_MOTORS_PER_AXIS; ++i) {
            tmp[5] = char(i + '0');
            tmp[6] = '\0';
            handler.section(tmp, _motors[i], _axis, i);
        }
    }

    void Axis::afterParse() {
        if (_motors[0] == nullptr) {
            _motors[0] = new Machine::Motor(_axis, 0);
        }

        if (is_linear(_axis)) {
            if (_maxSpeedLinear > 0.0f) {
                _maxRate = _maxSpeedLinear;
            }
        } else {
            if (_maxSpeedRotary > 0.0f) {
                _maxRate = _maxSpeedRotary;
            }
            if (_accelRotary > 0.0f) {
                _acceleration = _accelRotary;
            }
        }

        // Human-readable mechanics win over the raw steps_per_mm when complete.
        // A rotary axis may omit distance_per_rev (defaults to 360 degrees).
        const bool anyMechanics = _fullStepsPerRev || _microsteps || _distancePerRev > 0.0f;
        const bool allMechanics = _fullStepsPerRev && _microsteps && (_distancePerRev > 0.0f || !is_linear(_axis));
        if (allMechanics) {
            const float distance = _distancePerRev > 0.0f ? _distancePerRev : 360.0f;
            _stepsPerMm          = float(_fullStepsPerRev) * float(_microsteps) / distance;
            if (_distancePerRev == 0.0f) {
                _distancePerRev = 360.0f;
            }
            log_info("Axis " << Axes::axisName(_axis) << " derived steps_per_mm " << _stepsPerMm << " from "
                             << _fullStepsPerRev << " full steps x " << _microsteps << " microsteps / " << distance
                             << " per rev");
        } else if (anyMechanics) {
            log_config_error("Axis " << Axes::axisName(_axis)
                                     << " human mechanics fields are incomplete; supply all of full_steps_per_rev, "
                                        "microsteps and (for linear axes) distance_per_rev");
        }
    }

    void Axis::validate() {
        const bool anyMechanics = _fullStepsPerRev || _microsteps || _distancePerRev > 0.0f;
        const bool allMechanics = _fullStepsPerRev && _microsteps && (_distancePerRev > 0.0f || !is_linear(_axis));
        Assert(!anyMechanics || allMechanics,
               "axis mechanics fields are incomplete; steps_per_mm cannot be derived");
        if (allMechanics || _fullStepsPerRev || _microsteps) {
            Assert(_fullStepsPerRev > 0 && _microsteps > 0,
                   "full_steps_per_rev and microsteps must both be positive when supplied");
            Assert(!is_linear(_axis) || _distancePerRev > 0.0f,
                   "distance_per_rev must be positive for a linear axis when using human mechanics fields");
        }
        if (allMechanics) {
            Assert(_stepsPerMm >= 0.001f && _stepsPerMm <= 100000.0f,
                   "derived steps_per_mm is outside the supported range");
        }
        if (is_linear(_axis)) {
            Assert(_maxSpeedRotary == 0.0f && _accelRotary == 0.0f,
                   "rotary max_speed_deg_per_min/acceleration_deg_per_sec2 used on a linear axis");
        } else {
            Assert(_maxSpeedLinear == 0.0f,
                   "max_speed_mm_per_min used on a rotary axis; use max_speed_deg_per_min");
        }
    }

    void Axis::init() {
        uint32_t stepRate = uint32_t(_stepsPerMm * _maxRate / 60.0);
        auto     maxRate  = Stepping::maxPulsesPerSec();
        if (stepRate > maxRate) {
            log_config_error("Axis " << Axes::axisName(_axis) << " stepping rate " << stepRate
                                     << " steps/sec exceeds the maximum rate " << maxRate);
        }

        for (size_t i = 0; i < Axis::MAX_MOTORS_PER_AXIS; i++) {
            auto m = _motors[i];
            if (m) {
                log_info("  Motor" << i);
                m->init();
            }
        }
        if (_homing && _homing->_cycle >= 0) {
            _homing->init();
            set_bitnum(Axes::homingMask, _axis);
        }

        if (!_motors[0] && _motors[1]) {
            log_config_error("motor1 defined without motor0");
        }

        // If dual motors and only one motor has switches, this is the configuration
        // for a POG style squaring. The switch should report as being on both axes
        if (hasDualMotor() && (motorsWithSwitches() == 1)) {
            _motors[0]->makeDualSwitches();
            _motors[1]->makeDualSwitches();
        }

        // see if the configured switches support the homing direction.
        if (_homing) {
            bool homing_dir_supported = false;
            auto direction            = _homing->_positiveDirection;
            for (motor_t i = 0; i < MAX_MOTORS_PER_AXIS; i++) {
                auto m = _motors[i];
                if (m && m->supports_homing_dir(direction)) {
                    homing_dir_supported = true;
                    break;
                }
            }
            if (!homing_dir_supported) {
                log_warn("  Limit switches do not support " << (direction ? "positive" : "negative") << " homing dir");
            }
        }
    }

    void Axis::config_motors() {
        for (motor_t motor = 0; motor < Axis::MAX_MOTORS_PER_AXIS; ++motor) {
            auto mot = _motors[motor];
            if (mot)
                mot->config_motor();
        }
    }

    // Checks if a motor matches this axis:
    bool Axis::hasMotor(const MotorDrivers::MotorDriver* const driver) const {
        for (size_t i = 0; i < MAX_MOTORS_PER_AXIS; i++) {
            auto m = _motors[i];
            if (m && m->_driver == driver) {
                return true;
            }
        }
        return false;
    }

    // Does this axis have 2 motors?
    bool Axis::hasDualMotor() {
        return _motors[0] && _motors[0]->isReal() && _motors[1] && _motors[1]->isReal();
    }

    // How many motors have switches defined?
    motor_t Axis::motorsWithSwitches() {
        motor_t count = 0;
        for (motor_t i = 0; i < MAX_MOTORS_PER_AXIS; i++) {
            auto m = _motors[i];
            if (m && m->hasSwitches()) {
                count++;
            }
        }
        return count;
    }

    float Axis::commonPulloff() {
        auto motor0Pulloff = _motors[0]->_pulloff;
        if (hasDualMotor()) {
            auto motor1Pulloff = _motors[1]->_pulloff;
            return std::min(motor0Pulloff, motor1Pulloff);
        } else {
            return motor0Pulloff;
        }
    }

    // returns the offset between the pulloffs
    // value is positive when motor1 has a larger pulloff
    float Axis::extraPulloff() {
        if (hasDualMotor()) {
            return _motors[1]->_pulloff - _motors[0]->_pulloff;
        } else {
            return 0.0f;
        }
    }

    bool Axis::can_home() {
        for (motor_t i = 0; i < MAX_MOTORS_PER_AXIS; i++) {
            if (_motors[i]) {
                if (_motors[i]->can_home()) {
                    return true;
                }
            }
        }
        return false;
    }

    Axis::~Axis() {
        for (size_t i = 0; i < MAX_MOTORS_PER_AXIS; i++) {
            if (_motors[i]) {
                delete _motors[i];
            }
        }
    }
}
