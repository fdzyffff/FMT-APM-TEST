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
 *       RC_Channels.cpp - class containing an array of RC_Channel objects
 *
 */

#include "stdlib.h"
#include <cmath>

#include "AP_Math.h"

#include "RC_Channel.h"

#include "ap_hal.h"

RC_Channel *RC_Channels::channels;

/*
  channels group object constructor
 */
RC_Channels::RC_Channels(void)
{
    printf(" Init: RC_Channels\n");
    channels = obj_channels;

    // setup ch_in on channels
    for (uint8_t i=0; i<NUM_RC_CHANNELS; i++) {
        channels[i].ch_in = i;
    }
}

/*
  call read() and set_pwm() on all channels
 */
void
RC_Channels::set_pwm_all(void)
{
    for (uint8_t i=0; i<NUM_RC_CHANNELS; i++) {
        channels[i].set_pwm(channels[i].read());
    }
}
