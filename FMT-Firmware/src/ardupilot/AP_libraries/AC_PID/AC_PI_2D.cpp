/// @file	AC_PI_2D.cpp
/// @brief	Generic PID algorithm

#include "AP_Math.h"
#include "AC_PI_2D.h"

// Constructor
AC_PI_2D::AC_PI_2D(float initial_p, float initial_i, float initial_imax, float initial_filt_hz, float dt) :
    _kp(0.0f),
    _ki(0.0f),
    _imax(0.0f),
    _filt_hz(AC_PI_2D_FILT_HZ_DEFAULT),
    _dt(dt)
{
    _kp = initial_p;
    _ki = initial_i;
    _imax = fabsf(initial_imax);
    filt_hz(initial_filt_hz);

    // reset input filter to first value received
    _flags._reset_filter = true;
}

// set_dt - set time step in seconds
void AC_PI_2D::set_dt(float dt)
{
    // set dt and calculate the input filter alpha
    _dt = dt;
    calc_filt_alpha();
}

// filt_hz - set input filter hz
void AC_PI_2D::filt_hz(float hz)
{
    _filt_hz = (fabsf(hz));

    // sanity check _filt_hz
    _filt_hz = MAX(_filt_hz, AC_PI_2D_FILT_HZ_MIN);

    // calculate the input filter alpha
    calc_filt_alpha();
}

// set_input - set input to PID controller
//  input is filtered before the PID controllers are run
//  this should be called before any other calls to get_p, get_i or get_d
void AC_PI_2D::set_input(const Vector2f &input)
{
    // don't process inf or NaN
    if (!isfinite(input.x) || !isfinite(input.y)) {
        return;
    }

    // reset input filter to value received
    if (_flags._reset_filter) {
        _flags._reset_filter = false;
        _input = input;
    }

    // update filter and calculate derivative
    Vector2f input_filt_change = (input - _input) * _filt_alpha;
    _input = _input + input_filt_change;
}

Vector2f AC_PI_2D::get_p() const
{
    return (_input * _kp);
}

Vector2f AC_PI_2D::get_i()
{
    if(!is_zero(_ki) && !is_zero(_dt)) {
        _integrator += (_input * _ki) * _dt;
        float integrator_length = _integrator.length();
        if ((integrator_length > _imax) && (integrator_length > 0)) {
            _integrator *= (_imax / integrator_length);
        }
        return _integrator;
    }
    return Vector2f();
}

// get_i_shrink - get_i but do not allow integrator to grow in length (it may shrink)
Vector2f AC_PI_2D::get_i_shrink()
{
    if (!is_zero(_ki) && !is_zero(_dt)) {
        float integrator_length_orig = MIN(_integrator.length(),_imax);
        _integrator += (_input * _ki) * _dt;
        float integrator_length_new = _integrator.length();
        if ((integrator_length_new > integrator_length_orig) && (integrator_length_new > 0)) {
            _integrator *= (integrator_length_orig / integrator_length_new);
        }
        return _integrator;
    }
    return Vector2f();
}

Vector2f AC_PI_2D::get_pi()
{
    return get_p() + get_i();
}

void AC_PI_2D::reset_I()
{
    _integrator.zero();
}

/// Overload the function call operator to permit easy initialisation
void AC_PI_2D::operator() (float p, float i, float imaxval, float input_filt_hz, float dt)
{
    _kp = p;
    _ki = i;
    _imax = fabsf(imaxval);
    _filt_hz = input_filt_hz;
    _dt = dt;
    // calculate the input filter alpha
    calc_filt_alpha();
}

// calc_filt_alpha - recalculate the input filter alpha
void AC_PI_2D::calc_filt_alpha()
{
    if (is_zero(_filt_hz)) {
        _filt_alpha = 1.0f;
        return;
    }
  
    // calculate alpha
    float rc = 1/(M_2PI*_filt_hz);
    _filt_alpha = _dt / (_dt + rc);
}
