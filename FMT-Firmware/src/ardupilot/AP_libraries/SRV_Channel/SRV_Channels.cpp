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
#include "SRV_Channel.h"
#include "ap_hal.h"

extern AP_HAL hal;

SRV_Channel *SRV_Channels::channels;
bool SRV_Channels::disabled_passthrough;
bool SRV_Channels::initialised;
Bitmask SRV_Channels::function_mask{SRV_Channel::k_nr_aux_servo_functions};
SRV_Channels::srv_function SRV_Channels::functions[SRV_Channel::k_nr_aux_servo_functions];
   
/*
  constructor
 */
SRV_Channels::SRV_Channels(void)
{
    printf(" Init: SRV_Channels\n");
    channels = obj_channels;
    
    function_mask = *(new Bitmask(SRV_Channel::k_nr_aux_servo_functions));
    // set defaults from the parameter table

    // setup ch_num on channels
    for (uint8_t i=0; i<NUM_SERVO_CHANNELS; i++) {
        channels[i].ch_num = i;
    }
}

/*
  save adjusted trims
 */
void SRV_Channels::save_trim(void)
{
    for (uint8_t i=0; i<NUM_SERVO_CHANNELS; i++) {
        if (trimmed_mask & (1U<<i)) {
            ;//channels[i].servo_trim.set_and_save(channels[i].servo_trim.get());
        }
    }
    trimmed_mask = 0;
}

void SRV_Channels::output_trim_all(void)
{
    for (uint8_t i=0; i<NUM_SERVO_CHANNELS; i++) {
        channels[i].set_output_pwm(channels[i].servo_trim);
    }
}

void SRV_Channels::setup_failsafe_trim_all(void)
{
    ;
}

/*
  run calc_pwm for all channels
 */
void SRV_Channels::calc_pwm(void)
{
    ;
}

// set output value for a specific function channel as a pwm value
void SRV_Channels::set_output_pwm_chan(uint8_t chan, uint16_t value)
{
    if (chan < NUM_SERVO_CHANNELS) {
        channels[chan].set_output_pwm(value);
    }
}
