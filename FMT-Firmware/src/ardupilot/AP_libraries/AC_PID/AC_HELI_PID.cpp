/// @file	AC_HELI_PID.cpp
/// @brief	Generic PID algorithm

#include "AP_Math.h"
#include "AC_HELI_PID.h"

/// Constructor for PID
AC_HELI_PID::AC_HELI_PID(float initial_p, float initial_i, float initial_d, float initial_imax, float initial_filt_hz, float dt, float initial_ff) :
    AC_PID(initial_p, initial_i, initial_d, initial_imax, initial_filt_hz, dt),
    _leak_min(AC_PID_LEAK_MIN)
{
    _ff = initial_ff;
    _last_requested_rate = 0;
}

// This is an integrator which tends to decay to zero naturally
// if the error is zero.

float AC_HELI_PID::get_leaky_i(float leak_rate)
{
    if(!is_zero(_ki) && !is_zero(_dt)){

        // integrator does not leak down below Leak Min
        if (_integrator > _leak_min){
            _integrator -= (float)(_integrator - _leak_min) * leak_rate;
        } else if (_integrator < -_leak_min) {
            _integrator -= (float)(_integrator + _leak_min) * leak_rate;
        }

        _integrator += ((float)_input * _ki) * _dt;
        if (_integrator < -_imax) {
            _integrator = -_imax;
            } else if (_integrator > _imax) {
            _integrator = _imax;
        }

        //_pid_info.I = _integrator;
        return _integrator;
    }
    return 0;
}
