/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *       AP_MotorsHeli.cpp - ArduCopter motors library
 *       Code by RandyMackay. DIYDrones.com
 *
 */
#include <stdlib.h>
#include "AP_MotorsHeli.h"

#include "ap_hal.h"
extern AP_HAL hal;

//
// public methods
//

// init
void AP_MotorsHeli::init(motor_frame_class frame_class, motor_frame_type frame_type)
{
    // set update rate
    set_update_rate(_speed_hz);

    // load boot-up servo test cycles into counter to be consumed
    _servo_test_cycle_counter = _servo_test;

    // ensure inputs are not passed through to servos on start-up
    _servo_mode = SERVO_CONTROL_MODE_AUTOMATED;

    // initialise radio passthrough for collective to middle
    _throttle_radio_passthrough = 0.5f;

    // initialise Servo/PWM ranges and endpoints
    if (!init_outputs()) {
        // don't set initialised_ok
        return;
    }

    // calculate all scalars
    calculate_scalars();

    // record successful initialisation if what we setup was the desired frame_class
    _flags.initialised_ok = (frame_class == MOTOR_FRAME_HELI);
}

// set frame class (i.e. quad, hexa, heli) and type (i.e. x, plus)
void AP_MotorsHeli::set_frame_class_and_type(motor_frame_class frame_class, motor_frame_type frame_type)
{
    _flags.initialised_ok = (frame_class == MOTOR_FRAME_HELI);
}

// output_min - sets servos to neutral point with motors stopped
void AP_MotorsHeli::output_min()
{
    // move swash to mid
    move_actuators(0.0f,0.0f,0.5f,0.0f);

    update_motor_control(ROTOR_CONTROL_STOP);

    // override limits flags
    limit.roll_pitch = true;
    limit.yaw = true;
    limit.throttle_lower = true;
    limit.throttle_upper = false;
}

// output - sends commands to the servos
void AP_MotorsHeli::output()
{
    // update throttle filter
    update_throttle_filter();

    if (_flags.armed) {
        calculate_armed_scalars();
        if (!_flags.interlock) {
            output_armed_zero_throttle();
        } else {
            output_armed_stabilizing();
        }
    } else {
        output_disarmed();
    }
};

// sends commands to the motors
void AP_MotorsHeli::output_armed_stabilizing()
{
    // if manual override active after arming, deactivate it and reinitialize servos
    if (_servo_mode != SERVO_CONTROL_MODE_AUTOMATED) {
        reset_flight_controls();
    }

    move_actuators(_roll_in, _pitch_in, get_throttle(), _yaw_in);

    update_motor_control(ROTOR_CONTROL_ACTIVE);
}

// output_armed_zero_throttle - sends commands to the motors
void AP_MotorsHeli::output_armed_zero_throttle()
{
    // if manual override active after arming, deactivate it and reinitialize servos
    if (_servo_mode != SERVO_CONTROL_MODE_AUTOMATED) {
        reset_flight_controls();
    }

    move_actuators(_roll_in, _pitch_in, get_throttle(), _yaw_in);

    update_motor_control(ROTOR_CONTROL_IDLE);
}

// output_disarmed - sends commands to the motors
void AP_MotorsHeli::output_disarmed()
{
    if (_servo_test_cycle_counter > 0){
        // perform boot-up servo test cycle if enabled
        servo_test();
    } else {
        // manual override (i.e. when setting up swash)
        switch (_servo_mode) {
            case SERVO_CONTROL_MODE_MANUAL_PASSTHROUGH:
                // pass pilot commands straight through to swash
                _roll_in = _roll_radio_passthrough;
                _pitch_in = _pitch_radio_passthrough;
                _throttle_filter.reset(_throttle_radio_passthrough);
                _yaw_in = _yaw_radio_passthrough;
                break;
            case SERVO_CONTROL_MODE_MANUAL_CENTER:
                // fixate mid collective
                _roll_in = 0.0f;
                _pitch_in = 0.0f;
                _throttle_filter.reset(_collective_mid_pct);
                _yaw_in = 0.0f;
                break;
            case SERVO_CONTROL_MODE_MANUAL_MAX:
                // fixate max collective
                _roll_in = 0.0f;
                _pitch_in = 0.0f;
                _throttle_filter.reset(1.0f);
                _yaw_in = 1.0f;
                break;
            case SERVO_CONTROL_MODE_MANUAL_MIN:
                // fixate min collective
                _roll_in = 0.0f;
                _pitch_in = 0.0f;
                _throttle_filter.reset(0.0f);
                _yaw_in = -1.0f;
                break;
            case SERVO_CONTROL_MODE_MANUAL_OSCILLATE:
                // use servo_test function from child classes
                servo_test();
                break;
            default:
                // no manual override
                break;
        }
    }

    // ensure swash servo endpoints haven't been moved
    init_outputs();

    // continuously recalculate scalars to allow setup
    calculate_scalars();

    // helicopters always run stabilizing flight controls
    move_actuators(_roll_in, _pitch_in, get_throttle(), _yaw_in);

    update_motor_control(ROTOR_CONTROL_STOP);
}

// parameter_check - check if helicopter specific parameters are sensible
bool AP_MotorsHeli::parameter_check(bool display_msg) const
{
    // returns false if _rsc_setpoint is not higher than _rsc_critical as this would not allow rotor_runup_complete to ever return true
    if (_rsc_critical >= _rsc_setpoint) {
        if (display_msg) {
            //GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL, "PreArm: H_RSC_CRITICAL too large");
        }
        return false;
    }

    // returns false if RSC Mode is not set to a valid control mode
    if (_rsc_mode <= (int8_t)ROTOR_CONTROL_MODE_DISABLED || _rsc_mode > (int8_t)ROTOR_CONTROL_MODE_CLOSED_LOOP_POWER_OUTPUT) {
        if (display_msg) {
            //GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL, "PreArm: H_RSC_MODE invalid");
        }
        return false;
    }

    // returns false if RSC Runup Time is less than Ramp time as this could cause undesired behaviour of rotor speed estimate
    if (_rsc_runup_time <= _rsc_ramp_time){
        if (display_msg) {
            //GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL, "PreArm: H_RUNUP_TIME too small");
        }
        return false;
    }

    // returns false if idle output is higher than critical rotor speed as this could block runup_complete from going false
    if ( _rsc_idle_output >=  _rsc_critical){
        if (display_msg) {
            //GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL, "PreArm: H_RSC_IDLE too large");
        }
        return false;
    }

    // all other cases parameters are OK
    return true;
}

// reset_swash_servo
void AP_MotorsHeli::reset_swash_servo(SRV_Channel *servo)
{
    servo->set_range(1000);

    // swash servos always use full endpoints as restricting them would lead to scaling errors
    servo->set_output_min(1000);
    servo->set_output_max(2000);
}

// update the throttle input filter
void AP_MotorsHeli::update_throttle_filter()
{
    _throttle_filter.apply(_throttle_in, 1.0f/_loop_rate);

	// constrain filtered throttle 油门值保护
    if (_throttle_filter.get() < 0.0f) {
        _throttle_filter.reset(0.0f);
    }
    if (_throttle_filter.get() > 1.0f) {
        _throttle_filter.reset(1.0f);
    }
}

// reset_flight_controls - resets all controls and scalars to flight status
void AP_MotorsHeli::reset_flight_controls()
{
    _servo_mode = SERVO_CONTROL_MODE_AUTOMATED;
    init_outputs();
    calculate_scalars();
}
