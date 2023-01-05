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

#include "AP_Notify.h"
#include "stdio.h"
#include "string.h"

#define CONFIG_NOTIFY_DEVICES_COUNT 5

// Default constructor
AP_Notify::AP_Notify(){}

// static flags, to allow for direct class update from device drivers
struct AP_Notify::notify_flags_and_values_type AP_Notify::flags;
struct AP_Notify::notify_events_type AP_Notify::events;

// initialisation
void AP_Notify::init(bool enable_external_leds)
{
    // clear all flags and events
    memset(&AP_Notify::flags, 0, sizeof(AP_Notify::flags));
    memset(&AP_Notify::events, 0, sizeof(AP_Notify::events));

    // clear flight mode string and text buffer
    memset(_flight_mode_str, 0, sizeof(_flight_mode_str));
    memset(_send_text, 0, sizeof(_send_text));
    _send_text_updated_millis = 0;

    AP_Notify::flags.external_leds = enable_external_leds;

/*    for (uint8_t i = 0; i < CONFIG_NOTIFY_DEVICES_COUNT; i++) {
        if (_devices[i] != nullptr) {
            _devices[i]->pNotify = this;
            _devices[i]->init();
        }
    }*/
}

// main update function, called at 50Hz
void AP_Notify::update(void)
{
/*    for (uint8_t i = 0; i < CONFIG_NOTIFY_DEVICES_COUNT; i++) {
        if (_devices[i] != nullptr) {
            _devices[i]->update();
        }
    }

    //reset the events
    memset(&AP_Notify::events, 0, sizeof(AP_Notify::events));*/
}

// handle a LED_CONTROL message
/*void AP_Notify::handle_led_control(mavlink_message_t *msg)
{
    for (uint8_t i = 0; i < CONFIG_NOTIFY_DEVICES_COUNT; i++) {
        if (_devices[i] != nullptr) {
            _devices[i]->handle_led_control(msg);
        }
    }
}

// handle a PLAY_TUNE message
void AP_Notify::handle_play_tune(mavlink_message_t *msg)
{
    for (uint8_t i = 0; i < CONFIG_NOTIFY_DEVICES_COUNT; i++) {
        if (_devices[i] != nullptr) {
            _devices[i]->handle_play_tune(msg);
        }
    }
}*/

// set flight mode string
void AP_Notify::set_flight_mode_str(const char *str)
{
/*    strncpy(_flight_mode_str, str, 4);
    _flight_mode_str[sizeof(_flight_mode_str)-1] = 0;*/
}

void AP_Notify::send_text(const char *str)
{
/*    strncpy(_send_text, str, sizeof(_send_text));
    _send_text[sizeof(_send_text)-1] = 0;
    _send_text_updated_millis = millis();*/
}
