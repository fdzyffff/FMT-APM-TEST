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
 *       RC_Channel.cpp - class for one RC channel input
 */

#include "stdlib.h"
#include <cmath>

#include "AP_Math.h"

#include "RC_Channel.h"

extern AP_HAL hal;

// constructor
RC_Channel::RC_Channel(void) :
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    radio_min(1000),
    radio_trim(1500),
    radio_max(2000),
    reversed(0),
    dead_zone(0)
{
}

void
RC_Channel::set_range(uint16_t high)
{
    type_in = RC_CHANNEL_TYPE_RANGE;
    high_in = high;
}

void
RC_Channel::set_angle(uint16_t angle)
{
    type_in = RC_CHANNEL_TYPE_ANGLE;
    high_in = angle;
}

void
RC_Channel::set_default_dead_zone(int16_t dzone)
{
    dead_zone = (abs(dzone));
}

bool
RC_Channel::get_reverse(void) const
{
    return bool(reversed);
}

// read input from APM_RC - create a control_in value
void
RC_Channel::set_pwm(int16_t pwm)
{
    radio_in = pwm;

    if (type_in == RC_CHANNEL_TYPE_RANGE) {
        control_in = pwm_to_range();
    } else {
        //RC_CHANNEL_TYPE_ANGLE
        control_in = pwm_to_angle();
    }
}

// read input from APM_RC - create a control_in value, but use a 
// zero value for the dead zone. When done this way the control_in
// value can be used as servo_out to give the same output as input
void
RC_Channel::set_pwm_no_deadzone(int16_t pwm)
{
    radio_in = pwm;

    if (type_in == RC_CHANNEL_TYPE_RANGE) {
        control_in = pwm_to_range_dz(0);
    } else {
        //RC_CHANNEL_ANGLE
        control_in = pwm_to_angle_dz(0);
    }
}

/*
  return the center stick position expressed as a control_in value
  used for thr_mid in copter
 */
int16_t RC_Channel::get_control_mid() const
{
    if (type_in == RC_CHANNEL_TYPE_RANGE) {
        int16_t r_in = (radio_min + radio_max)/2;

        if (reversed) {
            r_in = radio_max - (r_in - radio_min);
        }

        int16_t radio_trim_low  = radio_min + dead_zone;

        return (((int32_t)(high_in) * (int32_t)(r_in - radio_trim_low)) / (int32_t)(radio_max - radio_trim_low));
    } else {
        return 0;
    }
}

// ------------------------------------------

/*
  return an "angle in centidegrees" (normally -4500 to 4500) from
  the current radio_in value using the specified dead_zone
 */
int16_t
RC_Channel::pwm_to_angle_dz_trim(uint16_t _dead_zone, uint16_t _trim)
{
    int16_t radio_trim_high = _trim + _dead_zone;
    int16_t radio_trim_low  = _trim - _dead_zone;

    // prevent div by 0
    if ((radio_trim_low - radio_min) == 0 || (radio_max - radio_trim_high) == 0)
        return 0;

    int16_t reverse_mul = (reversed?-1:1);
    if (radio_in > radio_trim_high) {
        return reverse_mul * ((int32_t)high_in * (int32_t)(radio_in - radio_trim_high)) / (int32_t)(radio_max  - radio_trim_high);
    } else if (radio_in < radio_trim_low) {
        return reverse_mul * ((int32_t)high_in * (int32_t)(radio_in - radio_trim_low)) / (int32_t)(radio_trim_low - radio_min);
    } else {
        return 0;
    }
}

/*
  return an "angle in centidegrees" (normally -4500 to 4500) from
  the current radio_in value using the specified dead_zone
 */
int16_t
RC_Channel::pwm_to_angle_dz(uint16_t _dead_zone)
{
    return pwm_to_angle_dz_trim(_dead_zone, radio_trim);
}

/*
  return an "angle in centidegrees" (normally -4500 to 4500) from
  the current radio_in value
 */
int16_t
RC_Channel::pwm_to_angle()
{
	return pwm_to_angle_dz(dead_zone);
}


/*
  convert a pulse width modulation value to a value in the configured
  range, using the specified deadzone
 */
int16_t
RC_Channel::pwm_to_range_dz(uint16_t _dead_zone)
{
    int16_t r_in = apm_constrain_int16(radio_in, radio_min, radio_max);

    if (reversed) {
	    r_in = radio_max - (r_in - radio_min);
    }

    int16_t radio_trim_low  = radio_min + _dead_zone;

    if (r_in > radio_trim_low) {
        return (((int32_t)(high_in) * (int32_t)(r_in - radio_trim_low)) / (int32_t)(radio_max - radio_trim_low));
    }
    return 0;
}

/*
  convert a pulse width modulation value to a value in the configured
  range
 */
int16_t
RC_Channel::pwm_to_range()
{
    return pwm_to_range_dz(dead_zone);
}


int16_t RC_Channel::get_control_in_zero_dz(void)
{
    if (type_in == RC_CHANNEL_TYPE_RANGE) {
        return pwm_to_range_dz(0);
    }
    return pwm_to_angle_dz(0);
}

// ------------------------------------------

float
RC_Channel::norm_input()
{
    float ret;
    int16_t reverse_mul = (reversed?-1:1);
    if (radio_in < radio_trim) {
        if (radio_min >= radio_trim) {
            return 0.0f;
        }
        ret = reverse_mul * (float)(radio_in - radio_trim) / (float)(radio_trim - radio_min);
    } else {
        if (radio_max <= radio_trim) {
            return 0.0f;
        }
        ret = reverse_mul * (float)(radio_in - radio_trim) / (float)(radio_max  - radio_trim);
    }
    return apm_constrain_float(ret, -1.0f, 1.0f);
}

float
RC_Channel::norm_input_dz()
{
    int16_t dz_min = radio_trim - dead_zone;
    int16_t dz_max = radio_trim + dead_zone;
    float ret;
    int16_t reverse_mul = (reversed?-1:1);
    if (radio_in < dz_min && dz_min > radio_min) {
        ret = reverse_mul * (float)(radio_in - dz_min) / (float)(dz_min - radio_min);
    } else if (radio_in > dz_max && radio_max > dz_max) {
        ret = reverse_mul * (float)(radio_in - dz_max) / (float)(radio_max  - dz_max);
    } else {
        ret = 0;
    }
    return apm_constrain_float(ret, -1.0f, 1.0f);
}

/*
  get percentage input from 0 to 100. This ignores the trim value.
 */
uint8_t
RC_Channel::percent_input()
{
    if (radio_in <= radio_min) {
        return reversed?100:0;
    }
    if (radio_in >= radio_max) {
        return reversed?0:100;
    }
    uint8_t ret = 100.0f * (radio_in - radio_min) / (float)(radio_max - radio_min);
    if (reversed) {
        ret = 100 - ret;
    }
    return ret;
}

void
RC_Channel::input()
{
    ;
}

uint16_t
RC_Channel::read() const
{
    return 0;
}

/*
  Return true if the channel is at trim and within the DZ
*/
bool RC_Channel::in_trim_dz()
{
    return is_bounded_int32(radio_in, radio_trim - dead_zone, radio_trim + dead_zone);
}

