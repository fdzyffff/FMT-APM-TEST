#include "AC_AttitudeControl_Multi.h"
#include "AP_Math.h"

AC_AttitudeControl_Multi::AC_AttitudeControl_Multi(AP_AHRS_View &ahrs, const AP_Vehicle::MultiCopter &aparm, AP_MotorsMulticopter& motors, float dt) :
    AC_AttitudeControl(ahrs, aparm, motors, dt),
    _pid_rate_roll(AC_ATC_MULTI_RATE_RP_P, AC_ATC_MULTI_RATE_RP_I, AC_ATC_MULTI_RATE_RP_D, AC_ATC_MULTI_RATE_RP_IMAX, AC_ATC_MULTI_RATE_RP_FILT_HZ, dt),
    _pid_rate_pitch(AC_ATC_MULTI_RATE_RP_P, AC_ATC_MULTI_RATE_RP_I, AC_ATC_MULTI_RATE_RP_D, AC_ATC_MULTI_RATE_RP_IMAX, AC_ATC_MULTI_RATE_RP_FILT_HZ, dt),
    _pid_rate_yaw(AC_ATC_MULTI_RATE_YAW_P, AC_ATC_MULTI_RATE_YAW_I, AC_ATC_MULTI_RATE_YAW_D, AC_ATC_MULTI_RATE_YAW_IMAX, AC_ATC_MULTI_RATE_YAW_FILT_HZ, dt),
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    _thr_mix_man(AC_ATTITUDE_CONTROL_MAN_DEFAULT),
    _thr_mix_min(AC_ATTITUDE_CONTROL_MIN_DEFAULT),
    _thr_mix_max(AC_ATTITUDE_CONTROL_MAX_DEFAULT),
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    _motors_multi(motors)
{
}

// Update Alt_Hold angle maximum
void AC_AttitudeControl_Multi::update_althold_lean_angle_max(float throttle_in)
{
    // calc maximum tilt angle based on throttle
    float thr_max = _motors_multi.get_throttle_thrust_max();

    // divide by zero check
    if (is_zero(thr_max)) {
        _althold_lean_angle_max = 0.0f;
        return;
    }

    float althold_lean_angle_max = acos(apm_constrain_float(_throttle_in/(AC_ATTITUDE_CONTROL_ANGLE_LIMIT_THROTTLE_MAX * thr_max), 0.0f, 1.0f));
    _althold_lean_angle_max = _althold_lean_angle_max + (_dt/(_dt+_angle_limit_tc))*(althold_lean_angle_max-_althold_lean_angle_max);
}

void AC_AttitudeControl_Multi::set_throttle_out(float throttle_in, bool apply_angle_boost, float filter_cutoff)
{
    _throttle_in = throttle_in;
    update_althold_lean_angle_max(throttle_in);
    _motors.set_throttle_filter_cutoff(filter_cutoff);
    if (apply_angle_boost) {
        // Apply angle boost
        throttle_in = get_throttle_boosted(throttle_in);
    }else{
        // Clear angle_boost for logging purposes
        _angle_boost = 0.0f;
    }
    _motors.set_throttle(throttle_in);
    _motors.set_throttle_avg_max(get_throttle_avg_max(MAX(throttle_in, _throttle_in)));
}

// returns a throttle including compensation for roll/pitch angle
// throttle value should be 0 ~ 1
float AC_AttitudeControl_Multi::get_throttle_boosted(float throttle_in)
{
    if (!_angle_boost_enabled) {
        _angle_boost = 0.0f;
        return throttle_in;
    }
    // inverted_factor is 1 for tilt angles below 60 degrees
    // inverted_factor reduces from 1 to 0 for tilt angles between 60 and 90 degrees

    float cos_tilt = _ahrs.cos_pitch() * _ahrs.cos_roll();
    float inverted_factor = apm_constrain_float(2.0f*cos_tilt, 0.0f, 1.0f);
    float boost_factor = 1.0f/apm_constrain_float(cos_tilt, 0.5f, 1.0f);

    float throttle_out = throttle_in*inverted_factor*boost_factor;
    _angle_boost = apm_constrain_float(throttle_out - throttle_in,-1.0f,1.0f);
    return throttle_out;
}

// returns a throttle including compensation for roll/pitch angle
// throttle value should be 0 ~ 1
float AC_AttitudeControl_Multi::get_throttle_avg_max(float throttle_in)
{
    throttle_in = apm_constrain_float(throttle_in, 0.0f, 1.0f);
    return MAX(throttle_in, throttle_in*MAX(0.0f,1.0f-_throttle_rpy_mix)+_motors.get_throttle_hover()*_throttle_rpy_mix);
}

// update_throttle_rpy_mix - slew set_throttle_rpy_mix to requested value
void AC_AttitudeControl_Multi::update_throttle_rpy_mix()
{
    // slew _throttle_rpy_mix to _throttle_rpy_mix_desired
    if (_throttle_rpy_mix < _throttle_rpy_mix_desired) {
        // increase quickly (i.e. from 0.1 to 0.9 in 0.4 seconds)
        _throttle_rpy_mix += MIN(2.0f*_dt, _throttle_rpy_mix_desired-_throttle_rpy_mix);
    } else if (_throttle_rpy_mix > _throttle_rpy_mix_desired) {
        // reduce more slowly (from 0.9 to 0.1 in 1.6 seconds)
        _throttle_rpy_mix -= MIN(0.5f*_dt, _throttle_rpy_mix-_throttle_rpy_mix_desired);
    }
    _throttle_rpy_mix = apm_constrain_float(_throttle_rpy_mix, 0.1f, AC_ATTITUDE_CONTROL_MAX);
}

void AC_AttitudeControl_Multi::rate_controller_run()
{
    // move throttle vs attitude mixing towards desired (called from here because this is conveniently called on every iteration)
    update_throttle_rpy_mix();

    Vector3f gyro_latest = _ahrs.get_gyro_latest();
    _motors.set_roll(rate_target_to_motor_roll(gyro_latest.x, _rate_target_ang_vel.x));
    _motors.set_pitch(rate_target_to_motor_pitch(gyro_latest.y, _rate_target_ang_vel.y));
    _motors.set_yaw(rate_target_to_motor_yaw(gyro_latest.z, _rate_target_ang_vel.z));

    //control_monitor_update();
}

// sanity check parameters.  should be called once before takeoff
void AC_AttitudeControl_Multi::parameter_sanity_check()
{
    // sanity check throttle mix parameters
    if (_thr_mix_man < 0.1f || _thr_mix_man > 4.0f) {
        // parameter description recommends thr-mix-man be no higher than 0.9 but we allow up to 4.0
        // which can be useful for very high powered copters with very low hover throttle
        _thr_mix_man = (AC_ATTITUDE_CONTROL_MAN_DEFAULT);
    }
    if (_thr_mix_min < 0.1f || _thr_mix_min > 0.25f) {
        _thr_mix_min = (AC_ATTITUDE_CONTROL_MIN_DEFAULT);
    }
    if (_thr_mix_max < 0.5f || _thr_mix_max > AC_ATTITUDE_CONTROL_MAX) {
        // parameter description recommends thr-mix-max be no higher than 0.9 but we allow up to 5.0
        // which can be useful for very high powered copters with very low hover throttle
        _thr_mix_max = (AC_ATTITUDE_CONTROL_MAX_DEFAULT);
    }
    if (_thr_mix_min > _thr_mix_max) {
        _thr_mix_min = (AC_ATTITUDE_CONTROL_MIN_DEFAULT);
        _thr_mix_max = (AC_ATTITUDE_CONTROL_MAX_DEFAULT);
    }
}
