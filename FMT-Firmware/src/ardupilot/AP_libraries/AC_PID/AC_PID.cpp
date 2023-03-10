/// @file	AC_PID.cpp
/// @brief	Generic PID algorithm

#include "AP_Math.h"
#include "AC_PID.h"

// Constructor
AC_PID::AC_PID(float initial_p, float initial_i, float initial_d, float initial_imax, float initial_filt_hz, float dt) :
    _kp(0.0f),
    _ki(0.0f),
    _kd(0.0f),
    _imax(0.0f),
    _filt_hz(AC_PID_FILT_HZ_DEFAULT),
    _ff(0.0f),
    _dt(dt),
    _integrator(0.0f),
    _input(0.0f),
    _derivative(0.0f)
{

    _kp = initial_p;
    _ki = initial_i;
    _kd = initial_d;
    _imax = fabsf(initial_imax);
    filt_hz(initial_filt_hz);

    // reset input filter to first value received
    _flags._reset_filter = true;
}

// set_dt - set time step in seconds
void AC_PID::set_dt(float dt)
{
    // set dt and calculate the input filter alpha
    _dt = dt;
}

// filt_hz - set input filter hz
void AC_PID::filt_hz(float hz)
{
    _filt_hz = (fabsf(hz));

    // sanity check _filt_hz
    _filt_hz = MAX(_filt_hz, AC_PID_FILT_HZ_MIN);
}

// set_input_filter_all - set input to PID controller
//  input is filtered before the PID controllers are run
//  this should be called before any other calls to get_p, get_i or get_d
void AC_PID::set_input_filter_all(float input)
{
    // don't process inf or NaN
    if (!isfinite(input)) {
        return;
    }

    // reset input filter to value received
    if (_flags._reset_filter) {
        _flags._reset_filter = false;
        _input = input;
        _derivative = 0.0f;
    }

    // update filter and calculate derivative
    float input_filt_change = get_filt_alpha() * (input - _input);
    _input = _input + input_filt_change;
    if (_dt > 0.0f) {
        _derivative = input_filt_change / _dt;
    }
}

// set_input_filter_d - set input to PID controller
//  only input to the D portion of the controller is filtered
//  this should be called before any other calls to get_p, get_i or get_d
void AC_PID::set_input_filter_d(float input)
{
    // don't process inf or NaN
    if (!isfinite(input)) {
        return;
    }

    // reset input filter to value received
    if (_flags._reset_filter) {
        _flags._reset_filter = false;
        _derivative = 0.0f;
    }

    // update filter and calculate derivative
    if (_dt > 0.0f) {
        float derivative = (input - _input) / _dt;
        _derivative = _derivative + get_filt_alpha() * (derivative-_derivative);
    }

    _input = input;
}

float AC_PID::get_p()
{
    return (_input * _kp);
}

float AC_PID::get_i()
{
    if(!is_zero(_ki) && !is_zero(_dt)) {
        _integrator += ((float)_input * _ki) * _dt;
        if (_integrator < -_imax) {
            _integrator = -_imax;
        } else if (_integrator > _imax) {
            _integrator = _imax;
        }
        return _integrator;
    }
    return 0;
}

float AC_PID::get_d()
{
    // derivative component
    return (_kd * _derivative);
}

float AC_PID::get_ff(float requested_rate)
{
    return (float)requested_rate * _ff;
}


float AC_PID::get_pi()
{
    return get_p() + get_i();
}

float AC_PID::get_pid()
{
    return get_p() + get_i() + get_d();
}

void AC_PID::reset_I()
{
    _integrator = 0;
}

/// Overload the function call operator to permit easy initialisation
void AC_PID::operator() (float p, float i, float d, float imaxval, float input_filt_hz, float dt)
{
    _kp = p;
    _ki = i;
    _kd = d;
    _imax = fabsf(imaxval);
    _filt_hz = input_filt_hz;
    _dt = dt;
}

// calc_filt_alpha - recalculate the input filter alpha
float AC_PID::get_filt_alpha() const
{
    if (is_zero(_filt_hz)) {
        return 1.0f;
    }

    // calculate alpha
    float rc = 1/(M_2PI*_filt_hz);
    return _dt / (_dt + rc);
}
