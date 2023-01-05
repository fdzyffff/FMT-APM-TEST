/// @file	PID.cpp
/// @brief	Generic PID algorithm

#include <cmath>

#include "PID.h"
#include "AP_Math.h"

extern uint32_t millis();

float PID::get_pid(float error, float scaler)
{
    uint32_t tnow = millis();
    uint32_t dt = tnow - _last_t;
    float output            = 0;
    float delta_time;

    if (_last_t == 0 || dt > 1000) {
        dt = 0;

		// if this PID hasn't been used for a full second then zero
		// the intergator term. This prevents I buildup from a
		// previous fight mode from causing a massive return before
		// the integrator gets a chance to correct itself
		reset_I();
    }
    _last_t = tnow;

    delta_time = (float)dt / 1000.0f;

    // Compute proportional component
    //_pid_info.P = error * _kp;
    output += error * _kp;

    // Compute derivative component if time has elapsed
    if ((fabsf(_kd) > 0) && (dt > 0)) {
        float derivative;

		if (isnan(_last_derivative)) {
			// we've just done a reset, suppress the first derivative
			// term as we don't want a sudden change in input to cause
			// a large D output change			
			derivative = 0;
			_last_derivative = 0;
		} else {
			derivative = (error - _last_error) / delta_time;
		}

        // discrete low pass filter, cuts out the
        // high frequency noise that can drive the controller crazy
        float RC = 1/(2*M_PI*_fCut);
        derivative = _last_derivative +
                     ((delta_time / (RC + delta_time)) *
                      (derivative - _last_derivative));

        // update state
        _last_error             = error;
        _last_derivative    = derivative;

        // add in derivative component
        //_pid_info.D = _kd * derivative;
        output                          += _kd * derivative;
    }

    // scale the P and D components
    output *= scaler;
    //_pid_info.D *= scaler;
    //_pid_info.P *= scaler;

    // Compute integral component if time has elapsed
    if ((fabsf(_ki) > 0) && (dt > 0)) {
        _integrator             += (error * _ki) * scaler * delta_time;
        if (_integrator < -_imax) {
            _integrator = -_imax;
        } else if (_integrator > _imax) {
            _integrator = _imax;
        }
        //_pid_info.I = _integrator;
        output                          += _integrator;
    }

    //_pid_info.desired = output;
    return output;
}

void
PID::reset_I()
{
    _integrator = 0;
	// we use NAN (Not A Number) to indicate that the last 
	// derivative value is not valid
    _last_derivative = NAN;
    //_pid_info.I = 0;
}

