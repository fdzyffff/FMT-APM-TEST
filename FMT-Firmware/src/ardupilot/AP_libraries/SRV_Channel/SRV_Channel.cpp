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
  SRV_Channel.cpp - object to separate input and output channel
  ranges, trim and reversal
 */

#include "AP_Math.h"
#include "AP_Vehicle.h"
#include "AP_Math.h"
#include "SRV_Channel.h"
SRV_Channel::servo_mask_t SRV_Channel::have_pwm_mask;

SRV_Channel::SRV_Channel(void) :
    servo_min(1100),
    servo_max(1900),
    servo_trim(1500),
    reversed(0),
    function(0)
{
    // start with all pwm at zero
    have_pwm_mask = ~uint16_t(0);
}


// convert a 0..range_max to a pwm
uint16_t SRV_Channel::pwm_from_range(int16_t scaled_value) const
{
    if (servo_max <= servo_min || high_out == 0) {
        return servo_min;
    }
    if (scaled_value >= high_out) {
        scaled_value = high_out;
    }
    if (scaled_value < 0) {
        scaled_value = 0;
    }
    if (reversed) {
        scaled_value = high_out - scaled_value;
    }
    return servo_min + ((int32_t)scaled_value * (int32_t)(servo_max - servo_min)) / (int32_t)high_out;
}

// convert a -angle_max..angle_max to a pwm
uint16_t SRV_Channel::pwm_from_angle(int16_t scaled_value) const
{
    if (reversed) {
        scaled_value = -scaled_value;
    }
    if (scaled_value > 0) {
        return servo_trim + ((int32_t)scaled_value * (int32_t)(servo_max - servo_trim)) / (int32_t)high_out;
    } else {
        return servo_trim - (-(int32_t)scaled_value * (int32_t)(servo_trim - servo_min)) / (int32_t)high_out;
    }
}

void SRV_Channel::calc_pwm(int16_t output_scaled)
{
    if (have_pwm_mask & (1U<<ch_num)) {
        return;
    }
    uint16_t pwm;
    if (type_angle) {
        pwm = pwm_from_angle(output_scaled);
    } else {
        pwm = pwm_from_range(output_scaled);
    }
    set_output_pwm(pwm);
}

void SRV_Channel::set_output_pwm(uint16_t pwm)
{
    output_pwm = pwm;
    have_pwm_mask |= (1U<<ch_num);
}

// set angular range of scaled output
void SRV_Channel::set_angle(int16_t angle)
{
    type_angle = true;
    high_out = angle;    
    type_setup = true;
}

// set range of scaled output
void SRV_Channel::set_range(uint16_t high)
{
    type_angle = false;
    high_out = high;
    type_setup = true;
}

/*
  get normalised output from -1 to 1, assuming 0 at mid point of servo_min/servo_max
 */
float SRV_Channel::get_output_norm(void)
{
    uint16_t mid = (servo_max + servo_min) / 2;
    float ret;
    if (mid <= servo_min) {
        return 0;
    }
    if (output_pwm < mid) {
        ret = (float)(output_pwm - mid) / (float)(mid - servo_min);
    } else if (output_pwm > mid) {
        ret = (float)(output_pwm - mid) / (float)(servo_max  - mid);
    } else {
        ret = 0;
    }
    if (get_reversed()) {
           ret = -ret;
    }
    return ret;
}

uint16_t SRV_Channel::get_limit_pwm(LimitValue limit) const
{
    switch (limit) {
    case SRV_CHANNEL_LIMIT_TRIM:
        return servo_trim;
    case SRV_CHANNEL_LIMIT_MIN:
        return servo_min;
    case SRV_CHANNEL_LIMIT_MAX:
        return servo_max;
    case SRV_CHANNEL_LIMIT_ZERO_PWM:
    default:
        return 0;
    }
}

