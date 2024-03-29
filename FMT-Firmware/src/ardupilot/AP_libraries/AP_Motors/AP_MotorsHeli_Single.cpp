/*
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdlib.h>
#include "SRV_Channel.h"
#include "AP_MotorsHeli_Single.h"

#include "ap_hal.h"
extern AP_HAL hal;

// set update rate to motors - a value in hertz
void AP_MotorsHeli_Single::set_update_rate( uint16_t speed_hz )
{
    // record requested speed
    _speed_hz = speed_hz;

    // setup fast channels
    uint32_t mask = 
        1U << AP_MOTORS_MOT_1 |
        1U << AP_MOTORS_MOT_2 |
        1U << AP_MOTORS_MOT_3 |
        1U << AP_MOTORS_MOT_4;
    rc_set_freq(mask, _speed_hz);
}

// enable - starts allowing signals to be sent to motors and servos
void AP_MotorsHeli_Single::enable()
{
    // enable output channels
    rc_enable_ch(AP_MOTORS_MOT_1);    // swash servo 1
    rc_enable_ch(AP_MOTORS_MOT_2);    // swash servo 2
    rc_enable_ch(AP_MOTORS_MOT_3);    // swash servo 3
    rc_enable_ch(AP_MOTORS_MOT_4);    // yaw
    rc_enable_ch(AP_MOTORS_HELI_SINGLE_AUX);                                 // output for gyro gain or direct drive variable pitch tail motor
    rc_enable_ch(AP_MOTORS_HELI_SINGLE_RSC);                                 // output for main rotor esc
}

// init_outputs - initialise Servo/PWM ranges and endpoints
bool AP_MotorsHeli_Single::init_outputs()
{
    if (!_flags.initialised_ok) {
        _swash_servo_1 = SRV_Channels::get_channel_for(SRV_Channel::k_motor1, CH_1);
        _swash_servo_2 = SRV_Channels::get_channel_for(SRV_Channel::k_motor2, CH_2);
        _swash_servo_3 = SRV_Channels::get_channel_for(SRV_Channel::k_motor3, CH_3);
        _yaw_servo = SRV_Channels::get_channel_for(SRV_Channel::k_motor4, CH_4);
        if (_tail_type == AP_MOTORS_HELI_SINGLE_TAILTYPE_DIRECTDRIVE_VARPITCH) {
            _tail_rotor.init_servo();
            if (!_swash_servo_1 || !_swash_servo_2 || !_swash_servo_3 || !_yaw_servo) {
                return false;
            }
        } else {
            _servo_aux = SRV_Channels::get_channel_for(SRV_Channel::k_motor7, CH_7);
            if (!_swash_servo_1 || !_swash_servo_2 || !_swash_servo_3 || !_yaw_servo || !_servo_aux) {
                return false;
            }
        }
    }

    // reset swash servo range and endpoints
    reset_swash_servo (_swash_servo_1);
    reset_swash_servo (_swash_servo_2);
    reset_swash_servo (_swash_servo_3);

    _yaw_servo->set_angle(4500);

    // set main rotor servo range
    // tail rotor servo use range as set in vehicle code for rc7
    _main_rotor.init_servo();

    return true;
}

// output_test - spin a motor at the pwm value specified
//  motor_seq is the motor's sequence number from 1 to the number of motors on the frame
//  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
void AP_MotorsHeli_Single::output_test(uint8_t motor_seq, int16_t pwm)
{
    // exit immediately if not armed
    if (!armed()) {
        return;
    }

    // output to motors and servos
    switch (motor_seq) {
        case 1:
            // swash servo 1
            rc_write(AP_MOTORS_MOT_1, pwm);
            break;
        case 2:
            // swash servo 2
            rc_write(AP_MOTORS_MOT_2, pwm);
            break;
        case 3:
            // swash servo 3
            rc_write(AP_MOTORS_MOT_3, pwm);
            break;
        case 4:
            // external gyro & tail servo
            if (_tail_type == AP_MOTORS_HELI_SINGLE_TAILTYPE_SERVO_EXTGYRO) {
                if (_acro_tail && _ext_gyro_gain_acro > 0) {
                    write_aux(_ext_gyro_gain_acro/1000.0f);
                } else {
                    write_aux(_ext_gyro_gain_std/1000.0f);
                }
            }
            rc_write(AP_MOTORS_MOT_4, pwm);
            break;
        case 5:
            // main rotor
            rc_write(AP_MOTORS_HELI_SINGLE_RSC, pwm);
            break;
        default:
            // do nothing
            break;
    }
}

// set_desired_rotor_speed
void AP_MotorsHeli_Single::set_desired_rotor_speed(float desired_speed)
{
    _main_rotor.set_desired_speed(desired_speed);

    // always send desired speed to tail rotor control, will do nothing if not DDVPT not enabled
    _tail_rotor.set_desired_speed(_direct_drive_tailspeed/1000.0f);
}

// calculate_scalars - recalculates various scalers used.
void AP_MotorsHeli_Single::calculate_armed_scalars()
{
    _main_rotor.set_ramp_time(_rsc_ramp_time);//控制起转时间
    _main_rotor.set_runup_time(_rsc_runup_time);//实际起转时间
    _main_rotor.set_critical_speed(_rsc_critical/1000.0f);//必要转速
    _main_rotor.set_idle_output(_rsc_idle_output/1000.0f);//怠速
    _main_rotor.set_power_output_range(_rsc_power_low/1000.0f, _rsc_power_high/1000.0f, _rsc_power_negc/1000.0f, (uint16_t)_rsc_slewrate);
}


// calculate_scalars - recalculates various scalers used.
void AP_MotorsHeli_Single::calculate_scalars()
{
    // range check collective min, max and mid
    if( _collective_min >= _collective_max ) {
        _collective_min = AP_MOTORS_HELI_COLLECTIVE_MIN;
        _collective_max = AP_MOTORS_HELI_COLLECTIVE_MAX;
    }
    _collective_mid = apm_constrain_int16(_collective_mid, _collective_min, _collective_max);

    // calculate collective mid point as a number from 0 to 1
    _collective_mid_pct = ((float)(_collective_mid-_collective_min))/((float)(_collective_max-_collective_min));

    // calculate factors based on swash type and servo position
    calculate_roll_pitch_collective_factors();

    // send setpoints to main rotor controller and trigger recalculation of scalars
    _main_rotor.set_control_mode(static_cast<RotorControlMode>(_rsc_mode));
    calculate_armed_scalars();
    
    // send setpoints to tail rotor controller and trigger recalculation of scalars
    if (_tail_type == AP_MOTORS_HELI_SINGLE_TAILTYPE_DIRECTDRIVE_VARPITCH) {
        _tail_rotor.set_control_mode(ROTOR_CONTROL_MODE_SPEED_SETPOINT);
        _tail_rotor.set_ramp_time(AP_MOTORS_HELI_SINGLE_DDVPT_RAMP_TIME);
        _tail_rotor.set_runup_time(AP_MOTORS_HELI_SINGLE_DDVPT_RUNUP_TIME);
        _tail_rotor.set_critical_speed(_rsc_critical/1000.0f);
        _tail_rotor.set_idle_output(_rsc_idle_output/1000.0f);
    } else {
        _tail_rotor.set_control_mode(ROTOR_CONTROL_MODE_DISABLED);
        _tail_rotor.set_ramp_time(0);
        _tail_rotor.set_runup_time(0);
        _tail_rotor.set_critical_speed(0);
        _tail_rotor.set_idle_output(0);
    }
}

// calculate_roll_pitch_collective_factors - calculate factors based on swash type and servo position  影响因子计算
void AP_MotorsHeli_Single::calculate_roll_pitch_collective_factors()
{
    if (_swash_type == AP_MOTORS_HELI_SINGLE_SWASH_CCPM) {                     //CCPM Swashplate, perform control mixing

        // roll factors
        _rollFactor[CH_1] = cosf(radians(_servo1_pos + 90 - _phase_angle));
        _rollFactor[CH_2] = cosf(radians(_servo2_pos + 90 - _phase_angle));
        _rollFactor[CH_3] = cosf(radians(_servo3_pos + 90 - _phase_angle));

        // pitch factors
        _pitchFactor[CH_1] = cosf(radians(_servo1_pos - _phase_angle));
        _pitchFactor[CH_2] = cosf(radians(_servo2_pos - _phase_angle));
        _pitchFactor[CH_3] = cosf(radians(_servo3_pos - _phase_angle));

        // collective factors
        _collectiveFactor[CH_1] = 1;
        _collectiveFactor[CH_2] = 1;
        _collectiveFactor[CH_3] = 1;

    }else{              //H1 Swashplate, keep servo outputs separated

        // roll factors
        _rollFactor[CH_1] = 1;
        _rollFactor[CH_2] = 0;
        _rollFactor[CH_3] = 0;

        // pitch factors
        _pitchFactor[CH_1] = 0;
        _pitchFactor[CH_2] = 1;
        _pitchFactor[CH_3] = 0;

        // collective factors
        _collectiveFactor[CH_1] = 0;
        _collectiveFactor[CH_2] = 0;
        _collectiveFactor[CH_3] = 1;
    }
}

// get_motor_mask - returns a bitmask of which outputs are being used for motors or servos (1 means being used)
//  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
uint16_t AP_MotorsHeli_Single::get_motor_mask()
{
    // heli uses channels 1,2,3,4,7 and 8
    return rc_map_mask(1U << 0 | 1U << 1 | 1U << 2 | 1U << 3 | 1U << AP_MOTORS_HELI_SINGLE_AUX | 1U << AP_MOTORS_HELI_SINGLE_RSC);
}

// update_motor_controls - sends commands to motor controllers
void AP_MotorsHeli_Single::update_motor_control(RotorControlState state)
{
    // Send state update to motors
    _tail_rotor.output(state);
    _main_rotor.output(state);

    if (state == ROTOR_CONTROL_STOP){
        // set engine run enable aux output to not run position to kill engine when disarmed
        SRV_Channels::set_output_limit(SRV_Channel::k_engine_run_enable, SRV_Channel::SRV_CHANNEL_LIMIT_MIN);
    } else {
        // else if armed, set engine run enable output to run position
        SRV_Channels::set_output_limit(SRV_Channel::k_engine_run_enable, SRV_Channel::SRV_CHANNEL_LIMIT_MAX);
    }

    // Check if both rotors are run-up, tail rotor controller always returns true if not enabled
    _heliflags.rotor_runup_complete = ( _main_rotor.is_runup_complete() && _tail_rotor.is_runup_complete() );
}

//
// move_actuators - moves swash plate and tail rotor
//                 - expected ranges:
//                       roll : -1 ~ +1
//                       pitch: -1 ~ +1
//                       collective: 0 ~ 1
//                       yaw:   -1 ~ +1
//

float aaa= 370,bbb=100,ccc=1000;
void AP_MotorsHeli_Single::move_actuators(float roll_out, float pitch_out, float coll_in, float yaw_out)
{
    float yaw_offset = 0.0f;

    // initialize limits flag
    limit.roll_pitch = false;
    limit.yaw = false;
    limit.throttle_lower = false;
    limit.throttle_upper = false;

    // rescale roll_out and pitch_out into the min and max ranges to provide linear motion
    // across the input range instead of stopping when the input hits the constrain value
    // these calculations are based on an assumption of the user specified cyclic_max
    // coming into this equation at 4500 or less
    float total_out = norm(pitch_out, roll_out);

    if (total_out > (_cyclic_max/4500.0f)) {
        float ratio = (float)(_cyclic_max/4500.0f) / total_out;
        roll_out *= ratio;
        pitch_out *= ratio;
        limit.roll_pitch = true;
    }

    // constrain collective input
    float collective_out = coll_in;
    if (collective_out <= 0.0f) {
        collective_out = 0.0f;
        limit.throttle_lower = true;
    }
    if (collective_out >= 1.0f) {
        collective_out = 1.0f;
        limit.throttle_upper = true;
    }

    // ensure not below landed/landing collective
    if (_heliflags.landing_collective && collective_out < (_land_collective_min/1000.0f)) {
        collective_out = (_land_collective_min/1000.0f);
        limit.throttle_lower = true;
    }

    // if servo output not in manual mode, process pre-compensation factors，总距发生变化时，尾桨补偿
    if (_servo_mode == SERVO_CONTROL_MODE_AUTOMATED) {
        // rudder feed forward based on collective
        // the feed-forward is not required when the motor is stopped or at idle, and thus not creating torque
        // also not required if we are using external gyro
        if ((_main_rotor.get_control_output() > _main_rotor.get_idle_output()) && _tail_type != AP_MOTORS_HELI_SINGLE_TAILTYPE_SERVO_EXTGYRO) {
            // sanity check collective_yaw_effect
            _collective_yaw_effect = apm_constrain_float(_collective_yaw_effect, -AP_MOTORS_HELI_SINGLE_COLYAW_RANGE, AP_MOTORS_HELI_SINGLE_COLYAW_RANGE);
            // the 4.5 scaling factor is to bring the values in line with previous releases
            yaw_offset = _collective_yaw_effect * fabsf(collective_out - _collective_mid_pct) / 4.5f;
					aaa= _collective_yaw_effect;
        }
    } else {
        yaw_offset = 0.0f;
    }

    // feed power estimate into main rotor controller 升降时补偿主电机油门
    // ToDo: include tail rotor power?
    // ToDo: add main rotor cyclic power?
    if (collective_out > _collective_mid_pct) {
        // +ve motor load for +ve collective
        _main_rotor.set_motor_load((collective_out - _collective_mid_pct) / (1.0f - _collective_mid_pct));
    } else {
        // -ve motor load for -ve collective
        _main_rotor.set_motor_load((collective_out - _collective_mid_pct) / _collective_mid_pct);
    }

    // swashplate servos  控制分配 _pitchFactor俯仰响应因子  0.45保护参数
    float collective_scalar = ((float)(_collective_max-_collective_min))/1000.0f;
    float coll_out_scaled = collective_out * collective_scalar + (_collective_min - 1000)/1000.0f;
/*  直接这样也可以
		float servo1_out = ((_rollFactor[CH_1] * roll_out) + (0.5 * pitch_out))*0.45f + 1 * coll_out_scaled;
    float servo2_out = ((_rollFactor[CH_2] * roll_out) + (0.5 * pitch_out))*0.45f + 1 * coll_out_scaled;
    if (_swash_type == AP_MOTORS_HELI_SINGLE_SWASH_H1) {
        servo1_out += 0.5f;
        servo2_out += 0.5f;
    }
    float servo3_out = ((0 * roll_out) + (-1 * pitch_out))*0.45f + 1 * coll_out_scaled;

*/
    float servo1_out = ((_rollFactor[CH_1] * roll_out) + (_pitchFactor[CH_1] * pitch_out))*0.45f + _collectiveFactor[CH_1] * coll_out_scaled;
    float servo2_out = ((_rollFactor[CH_2] * roll_out) + (_pitchFactor[CH_2] * pitch_out))*0.45f + _collectiveFactor[CH_2] * coll_out_scaled;
    if (_swash_type == AP_MOTORS_HELI_SINGLE_SWASH_H1) {
        servo1_out += 0.5f;
        servo2_out += 0.5f;
    }
    float servo3_out = ((_rollFactor[CH_3] * roll_out) + (_pitchFactor[CH_3] * pitch_out))*0.45f + _collectiveFactor[CH_3] * coll_out_scaled;

    // rescale from -1..1, so we can use the pwm calc that includes trim    从0-1缩放到-1到+1
    servo1_out = 2.0f*servo1_out - 1.0f;
    servo2_out = 2.0f*servo2_out - 1.0f;
    servo3_out = 2.0f*servo3_out - 1.0f;
    
    // actually move the servos     输出到实际舵机
    rc_write(AP_MOTORS_MOT_1, calc_pwm_output_1to1(servo1_out, _swash_servo_1));
    rc_write(AP_MOTORS_MOT_2, calc_pwm_output_1to1(servo2_out, _swash_servo_2));
    rc_write(AP_MOTORS_MOT_3, calc_pwm_output_1to1(servo3_out, _swash_servo_3));

    // update the yaw rate using the tail rotor/servo
    move_yaw(yaw_out + yaw_offset);
}


// move_yaw
void AP_MotorsHeli_Single::move_yaw(float yaw_out)
{
    // sanity check yaw_out
    if (yaw_out < -1.0f) {
        yaw_out = -1.0f;
        limit.yaw = true;
    }
    if (yaw_out > 1.0f) {
        yaw_out = 1.0f;
        limit.yaw = true;
    }

    if (_tail_type == AP_MOTORS_HELI_SINGLE_TAILTYPE_DIRECTDRIVE_FIXEDPITCH){
        if (_main_rotor.get_desired_speed() > 0.0f && hal.get_soft_armed()) {
            // constrain output so that motor never fully stops
            yaw_out = apm_constrain_float(yaw_out, -0.9f, 1.0f);
            // output yaw servo to tail rsc
            rc_write(AP_MOTORS_MOT_4, calc_pwm_output_1to1(yaw_out, _yaw_servo));
        } else {
            // output zero speed to tail rsc
            rc_write(AP_MOTORS_MOT_4, calc_pwm_output_1to1(-1.0f, _yaw_servo));
        }
    } else {
        rc_write(AP_MOTORS_MOT_4, calc_pwm_output_1to1(yaw_out, _yaw_servo));
    }
    if (_tail_type == AP_MOTORS_HELI_SINGLE_TAILTYPE_SERVO_EXTGYRO) {
        // output gain to exernal gyro
        if (_acro_tail && _ext_gyro_gain_acro > 0) {
            write_aux(_ext_gyro_gain_acro/1000.0f);
        } else {
            write_aux(_ext_gyro_gain_std/1000.0f);
        }
    }
}

// write_aux - converts servo_out parameter value (0 to 1 range) to pwm and outputs to aux channel (ch7)
void AP_MotorsHeli_Single::write_aux(float servo_out)
{
    if (_servo_aux) {
        rc_write(AP_MOTORS_HELI_SINGLE_AUX, calc_pwm_output_0to1(servo_out, _servo_aux));
    }
}

// servo_test - move servos through full range of movement
void AP_MotorsHeli_Single::servo_test()
{
    _servo_test_cycle_time += 1.0f / _loop_rate;

    if ((_servo_test_cycle_time >= 0.0f && _servo_test_cycle_time < 0.5f)||                                   // Tilt swash back
        (_servo_test_cycle_time >= 6.0f && _servo_test_cycle_time < 6.5f)){
        _pitch_test += (1.0f / (_loop_rate / 2.0f));
        _oscillate_angle += 8 * M_PI / _loop_rate;
        _yaw_test = 0.5f * sinf(_oscillate_angle);
    } else if ((_servo_test_cycle_time >= 0.5f && _servo_test_cycle_time < 4.5f)||                            // Roll swash around
               (_servo_test_cycle_time >= 6.5f && _servo_test_cycle_time < 10.5f)){
        _oscillate_angle += M_PI / (2 * _loop_rate);
        _roll_test = sinf(_oscillate_angle);
        _pitch_test = cosf(_oscillate_angle);
        _yaw_test = sinf(_oscillate_angle);
    } else if ((_servo_test_cycle_time >= 4.5f && _servo_test_cycle_time < 5.0f)||                            // Return swash to level
               (_servo_test_cycle_time >= 10.5f && _servo_test_cycle_time < 11.0f)){
        _pitch_test -= (1.0f / (_loop_rate / 2.0f));
        _oscillate_angle += 8 * M_PI / _loop_rate;
        _yaw_test = 0.5f * sinf(_oscillate_angle);
    } else if (_servo_test_cycle_time >= 5.0f && _servo_test_cycle_time < 6.0f){                              // Raise swash to top
        _collective_test = 1.0f;
        _oscillate_angle += 2 * M_PI / _loop_rate;
        _yaw_test = sinf(_oscillate_angle);
    } else if (_servo_test_cycle_time >= 11.0f && _servo_test_cycle_time < 12.0f){                            // Lower swash to bottom
        _collective_test = 0.0f;
        _oscillate_angle += 2 * M_PI / _loop_rate;
        _yaw_test = sinf(_oscillate_angle);
    } else {                                                                                                  // reset cycle
        _servo_test_cycle_time = 0.0f;
        _oscillate_angle = 0.0f;
        _collective_test = 0.0f;
        _roll_test = 0.0f;
        _pitch_test = 0.0f;
        _yaw_test = 0.0f;
        // decrement servo test cycle counter at the end of the cycle
        if (_servo_test_cycle_counter > 0){
            _servo_test_cycle_counter--;
        }
    }

    // over-ride servo commands to move servos through defined ranges
    _throttle_filter.reset(_collective_test);
    _roll_in = _roll_test;
    _pitch_in = _pitch_test;
    _yaw_in = _yaw_test;
}

// parameter_check - check if helicopter specific parameters are sensible
bool AP_MotorsHeli_Single::parameter_check(bool display_msg) const
{
    // returns false if Phase Angle is outside of range 
    if ((_phase_angle > 90) || (_phase_angle < -90)){
        if (display_msg) {
            //GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL, "PreArm: H_PHANG out of range");
        }
        return false;
    }

    // returns false if Acro External Gyro Gain is outside of range
    if ((_ext_gyro_gain_acro < 0) || (_ext_gyro_gain_acro > 1000)){
        if (display_msg) {
            //GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL, "PreArm: H_GYR_GAIN_ACRO out of range");
        }
        return false;
    }

    // returns false if Standard External Gyro Gain is outside of range
    if ((_ext_gyro_gain_std < 0) || (_ext_gyro_gain_std > 1000)){
        if (display_msg) {
            //GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL, "PreArm: H_GYR_GAIN out of range");
        }
        return false;
    }

    // check parent class parameters
    return AP_MotorsHeli::parameter_check(display_msg);
}
