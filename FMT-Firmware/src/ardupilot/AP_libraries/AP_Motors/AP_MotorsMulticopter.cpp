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
 *       AP_MotorsMulticopter.cpp - ArduCopter multicopter motors library
 *       Code by Randy Mackay and Robert Lefebvre. DIYDrones.com
 *
 */

#include "AP_MotorsMulticopter.h"

#include "ap_hal.h"

extern AP_HAL hal;

// Constructor
AP_MotorsMulticopter::AP_MotorsMulticopter(uint16_t loop_rate, uint16_t speed_hz) :
    AP_Motors(loop_rate, speed_hz),
    _spool_mode(SHUT_DOWN),
    _spin_up_ratio(0.0f),
    _batt_voltage_resting(0.0f),
    _batt_current_resting(0.0f),
    _batt_resistance(0.0f),
    _batt_timer(0),
    _lift_max(1.0f),
    _throttle_limit(1.0f),
    _throttle_thrust_max(0.0f),
    _disarm_safety_timer(0)

{

    // disable all motors by default
    memset(motor_enabled, false, sizeof(motor_enabled));

    // setup battery voltage filtering
    _batt_voltage_filt.set_cutoff_frequency(AP_MOTORS_BATT_VOLT_FILT_HZ);
    _batt_voltage_filt.reset(1.0f);

    // default throttle range
    _throttle_radio_min = 1100;
    _throttle_radio_max = 1900;

//para init
    _yaw_headroom = (AP_MOTORS_YAW_HEADROOM_DEFAULT);
    _thrust_curve_expo = (AP_MOTORS_THST_EXPO_DEFAULT);
    _spin_max = (AP_MOTORS_SPIN_MAX_DEFAULT);
    _batt_voltage_max = (AP_MOTORS_BAT_VOLT_MAX_DEFAULT);
    _batt_voltage_min = (AP_MOTORS_BAT_VOLT_MIN_DEFAULT);
    _batt_current_max = (AP_MOTORS_BAT_CURR_MAX_DEFAULT);
    _pwm_type = (PWM_TYPE_NORMAL);
    _pwm_min = (0);
    _pwm_max = (0);
    _spin_min = (AP_MOTORS_SPIN_MIN_DEFAULT);
    _spin_arm = (AP_MOTORS_SPIN_ARM_DEFAULT);
    _batt_current_time_constant = (AP_MOTORS_BAT_CURR_TC_DEFAULT);
    _throttle_hover = (AP_MOTORS_THST_HOVER_DEFAULT);
    _throttle_hover_learn = (HOVER_LEARN_DISABLED);
    _disarm_disable_pwm = (0);
    _spool_up_time = (AP_MOTORS_SPOOL_UP_TIME_DEFAULT);
};

// output - sends commands to the motors
void AP_MotorsMulticopter::output()
{
    // update throttle filter
    update_throttle_filter();

    // run spool logic
    output_logic();

    // calculate thrust
    output_armed_stabilizing();

    // convert rpy_thrust values to pwm
    output_to_motors();
};

// sends minimum values out to the motors
void AP_MotorsMulticopter::output_min()
{
    set_desired_spool_state(DESIRED_SHUT_DOWN);
    _spool_mode = SHUT_DOWN;
    output();
}

// update the throttle input filter
void AP_MotorsMulticopter::update_throttle_filter()
{
    if (armed()) {
        _throttle_filter.apply(_throttle_in, 1.0f/_loop_rate);
        // constrain filtered throttle
        if (_throttle_filter.get() < 0.0f) {
            _throttle_filter.reset(0.0f);
        }
        if (_throttle_filter.get() > 1.0f) {
            _throttle_filter.reset(1.0f);
        }
    } else {
        _throttle_filter.reset(0.0f);
    }
}

// return current_limit as a number from 0 ~ 1 in the range throttle_min to throttle_max
float AP_MotorsMulticopter::get_current_limit_max_throttle()
{
    // return maximum if current limiting is disabled
    _throttle_limit = 1.0f;
    return 1.0f;
}

// apply_thrust_curve_and_volt_scaling - returns throttle in the range 0 ~ 1
float AP_MotorsMulticopter::apply_thrust_curve_and_volt_scaling(float thrust) const
{
    float throttle_ratio = thrust;
    // apply thrust curve - domain 0.0 to 1.0, range 0.0 to 1.0
    if (_thrust_curve_expo > 0.0f){
        throttle_ratio = ((_thrust_curve_expo-1.0f) + safe_sqrt((1.0f-_thrust_curve_expo)*(1.0f-_thrust_curve_expo) + 4.0f*_thrust_curve_expo*_lift_max*thrust))/(2.0f*_thrust_curve_expo);
    }

    return apm_constrain_float(throttle_ratio, 0.0f, 1.0f);
}

// update_lift_max from battery voltage - used for voltage compensation
void AP_MotorsMulticopter::update_lift_max_from_batt_voltage()
{
    // sanity check battery_voltage_min is not too small
    // if disabled or misconfigured exit immediately
    _batt_voltage_filt.reset(1.0f);
    _lift_max = 1.0f;
    return;
}

// update_battery_resistance - calculate battery resistance when throttle is above hover_out
void AP_MotorsMulticopter::update_battery_resistance()
{
    // if disarmed reset resting voltage and current
    if (!_flags.armed) {
        _batt_voltage_resting = _batt_voltage;
        _batt_current_resting = _batt_current;
        _batt_timer = 0;
    } else if (_batt_voltage_resting > _batt_voltage && _batt_current_resting < _batt_current) {
        // update battery resistance when throttle is over hover throttle
        float batt_resistance = (_batt_voltage_resting-_batt_voltage)/(_batt_current-_batt_current_resting);
        if ((_batt_timer < 400) && ((_batt_current_resting*2.0f) < _batt_current)) {
            if (get_throttle() >= get_throttle_hover()) {
                // filter reaches 90% in 1/4 the test time
                _batt_resistance += 0.05f*(batt_resistance - _batt_resistance);
                _batt_timer += 1;
            } else {
                // initialize battery resistance to prevent change in resting voltage estimate
                _batt_resistance = batt_resistance;
            }
        }
        // make sure battery resistance value doesn't result in the predicted battery voltage exceeding the resting voltage
        if(batt_resistance < _batt_resistance){
            _batt_resistance = batt_resistance;
        }
    }
}

float AP_MotorsMulticopter::get_compensation_gain() const
{
    // avoid divide by zero
    if (_lift_max <= 0.0f) {
        return 1.0f;
    }

    float ret = 1.0f / _lift_max;

#if AP_MOTORS_DENSITY_COMP == 1
    // air density ratio is increasing in density / decreasing in altitude
    if (_air_density_ratio > 0.3f && _air_density_ratio < 1.5f) {
        ret *= 1.0f / apm_constrain_float(_air_density_ratio,0.5f,1.25f);
    }
#endif
    return ret;
}

int16_t AP_MotorsMulticopter::calc_thrust_to_pwm(float thrust_in) const
{
    thrust_in = apm_constrain_float(thrust_in, 0.0f, 1.0f);
    return get_pwm_output_min() + (get_pwm_output_max()-get_pwm_output_min()) * (_spin_min + (_spin_max-_spin_min)*apply_thrust_curve_and_volt_scaling(thrust_in));
}

int16_t AP_MotorsMulticopter::calc_spin_up_to_pwm() const
{
    return get_pwm_output_min() + apm_constrain_float(_spin_up_ratio, 0.0f, 1.0f) * _spin_min * (get_pwm_output_max()-get_pwm_output_min());
}
// get minimum or maximum pwm value that can be output to motors
int16_t AP_MotorsMulticopter::get_pwm_output_min() const
{
    // return _pwm_min if both PWM_MIN and PWM_MAX parameters are defined and valid
    if ((_pwm_min > 0) && (_pwm_max > 0) && (_pwm_max > _pwm_min)) {
        return _pwm_min;
    }
    return _throttle_radio_min;
}

// get maximum pwm value that can be output to motors
int16_t AP_MotorsMulticopter::get_pwm_output_max() const
{
    // return _pwm_max if both PWM_MIN and PWM_MAX parameters are defined and valid
    if ((_pwm_min > 0) && (_pwm_max > 0) && (_pwm_max > _pwm_min)) {
        return _pwm_max;
    }
    return _throttle_radio_max;
}

// set_throttle_range - sets the minimum throttle that will be sent to the engines when they're not off (i.e. to prevents issues with some motors spinning and some not at very low throttle)
// also sets throttle channel minimum and maximum pwm
void AP_MotorsMulticopter::set_throttle_range(int16_t radio_min, int16_t radio_max)
{
    // sanity check
    if (radio_max <= radio_min) {
        return;
    }

    _throttle_radio_min = radio_min;
    _throttle_radio_max = radio_max;
}

// update the throttle input filter.  should be called at 100hz
void AP_MotorsMulticopter::update_throttle_hover(float dt)
{
/*    if (_throttle_hover_learn != HOVER_LEARN_DISABLED) {
        // we have chosen to constrain the hover throttle to be within the range reachable by the third order expo polynomial.
        _throttle_hover = apm_constrain_float(_throttle_hover + (dt/(dt+AP_MOTORS_THST_HOVER_TC))*(get_throttle()-_throttle_hover), AP_MOTORS_THST_HOVER_MIN, AP_MOTORS_THST_HOVER_MAX);
    }*/
    ;
}

// run spool logic
void AP_MotorsMulticopter::output_logic()
{
    if (_flags.armed) {
        _disarm_safety_timer = 100;
    } else if (_disarm_safety_timer != 0) {
        _disarm_safety_timer--;
    }

    // force desired and current spool mode if disarmed or not interlocked
    if (!_flags.armed || !_flags.interlock) {
        _spool_desired = DESIRED_SHUT_DOWN;
        _spool_mode = SHUT_DOWN;
    }

    if (_spool_up_time < 0.05) {
        // prevent float exception
        _spool_up_time = (0.05);
    }

    switch (_spool_mode) {
        case SHUT_DOWN:
            // Motors should be stationary.
            // Servos set to their trim values or in a test condition.

            // set limits flags
            limit.roll_pitch = true;
            limit.yaw = true;
            limit.throttle_lower = true;
            limit.throttle_upper = true;

            // make sure the motors are spooling in the correct direction
            if (_spool_desired != DESIRED_SHUT_DOWN) {
                _spool_mode = SPIN_WHEN_ARMED;
                break;
            }

            // set and increment ramp variables
            _spin_up_ratio = 0.0f;
            _throttle_thrust_max = 0.0f;
            break;

        case SPIN_WHEN_ARMED: {
            // Motors should be stationary or at spin when armed.
            // Servos should be moving to correct the current attitude.

            // set limits flags
            limit.roll_pitch = true;
            limit.yaw = true;
            limit.throttle_lower = true;
            limit.throttle_upper = true;

            // set and increment ramp variables
            float spool_step = 1.0f/(_spool_up_time*_loop_rate);
            if (_spool_desired == DESIRED_SHUT_DOWN){
                _spin_up_ratio -= spool_step;
                // constrain ramp value and update mode
                if (_spin_up_ratio <= 0.0f) {
                    _spin_up_ratio = 0.0f;
                    _spool_mode = SHUT_DOWN;
                }
            } else if(_spool_desired == DESIRED_THROTTLE_UNLIMITED) {
                _spin_up_ratio += spool_step;
                // constrain ramp value and update mode
                if (_spin_up_ratio >= 1.0f) {
                    _spin_up_ratio = 1.0f;
                    _spool_mode = SPOOL_UP;
                }
            } else {    // _spool_desired == SPIN_WHEN_ARMED
                float spin_up_armed_ratio = 0.0f;
                if (_spin_min > 0.0f) {
                    spin_up_armed_ratio = _spin_arm / _spin_min;
                }
                _spin_up_ratio += apm_constrain_float(spin_up_armed_ratio-_spin_up_ratio, -spool_step, spool_step);
            }
            _throttle_thrust_max = 0.0f;
            break;
        }
        case SPOOL_UP:
            // Maximum throttle should move from minimum to maximum.
            // Servos should exhibit normal flight behavior.

            // initialize limits flags
            limit.roll_pitch = false;
            limit.yaw = false;
            limit.throttle_lower = false;
            limit.throttle_upper = false;

            // make sure the motors are spooling in the correct direction
            if (_spool_desired != DESIRED_THROTTLE_UNLIMITED ){
                _spool_mode = SPOOL_DOWN;
                break;
            }

            // set and increment ramp variables
            _spin_up_ratio = 1.0f;
            _throttle_thrust_max += 1.0f/(_spool_up_time*_loop_rate);

            // constrain ramp value and update mode
            if (_throttle_thrust_max >= MIN(get_throttle(), get_current_limit_max_throttle())) {
                _throttle_thrust_max = get_current_limit_max_throttle();
                _spool_mode = THROTTLE_UNLIMITED;
            } else if (_throttle_thrust_max < 0.0f) {
                _throttle_thrust_max = 0.0f;
            }
            break;

        case THROTTLE_UNLIMITED:
            // Throttle should exhibit normal flight behavior.
            // Servos should exhibit normal flight behavior.

            // initialize limits flags
            limit.roll_pitch = false;
            limit.yaw = false;
            limit.throttle_lower = false;
            limit.throttle_upper = false;

            // make sure the motors are spooling in the correct direction
            if (_spool_desired != DESIRED_THROTTLE_UNLIMITED) {
                _spool_mode = SPOOL_DOWN;
                break;
            }

            // set and increment ramp variables
            _spin_up_ratio = 1.0f;
            _throttle_thrust_max = get_current_limit_max_throttle();
            break;

        case SPOOL_DOWN:
            // Maximum throttle should move from maximum to minimum.
            // Servos should exhibit normal flight behavior.

            // initialize limits flags
            limit.roll_pitch = false;
            limit.yaw = false;
            limit.throttle_lower = false;
            limit.throttle_upper = false;

            // make sure the motors are spooling in the correct direction
            if (_spool_desired == DESIRED_THROTTLE_UNLIMITED) {
                _spool_mode = SPOOL_UP;
                break;
            }

            // set and increment ramp variables
            _spin_up_ratio = 1.0f;
            _throttle_thrust_max -= 1.0f/(_spool_up_time*_loop_rate);

            // constrain ramp value and update mode
            if (_throttle_thrust_max <= 0.0f){
                _throttle_thrust_max = 0.0f;
            }
            if (_throttle_thrust_max >= get_current_limit_max_throttle()) {
                _throttle_thrust_max = get_current_limit_max_throttle();
            } else if (is_zero(_throttle_thrust_max)) {
                _spool_mode = SPIN_WHEN_ARMED;
            }
            break;
    }
}

// passes throttle directly to all motors for ESC calibration.
//   throttle_input is in the range of 0 ~ 1 where 0 will send get_pwm_output_min() and 1 will send get_pwm_output_max()
void AP_MotorsMulticopter::set_throttle_passthrough_for_esc_calibration(float throttle_input)
{
    if (armed()) {
        uint16_t pwm_out = get_pwm_output_min() + apm_constrain_float(throttle_input, 0.0f, 1.0f) * (get_pwm_output_max() - get_pwm_output_min());
        // send the pilot's input directly to each enabled motor
        for (uint16_t i=0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
            if (motor_enabled[i]) {
                rc_write(i, pwm_out);
            }
        }
    }
}

// output a thrust to all motors that match a given motor mask. This
// is used to control tiltrotor motors in forward flight. Thrust is in
// the range 0 to 1
void AP_MotorsMulticopter::output_motor_mask(float thrust, uint8_t mask)
{
    for (uint8_t i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            int16_t motor_out;
            if (mask & (1U<<i)) {
                motor_out = calc_thrust_to_pwm(thrust);
            } else {
                motor_out = get_pwm_output_min();
            }
            rc_write(i, motor_out);
        }
    }
}

// save parameters as part of disarming
void AP_MotorsMulticopter::save_params_on_disarm()
{
    // save hover throttle
    if (_throttle_hover_learn == HOVER_LEARN_AND_SAVE) {
        ;//_throttle_hover.save();
    }
}
