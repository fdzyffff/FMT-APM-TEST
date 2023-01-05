#pragma once

#include "inttypes.h"
#include "AP_Common.h"

class RCMapper
{
public:
    /// Constructor
    ///
    RCMapper();

    /// roll - return input channel number for roll / aileron input
    uint8_t roll() const { return _ch_roll; }

    /// pitch - return input channel number for pitch / elevator input
    uint8_t pitch() const { return _ch_pitch; }

    /// throttle - return input channel number for throttle input
    uint8_t throttle() const { return _ch_throttle; }

    /// yaw - return input channel number for yaw / rudder input
    uint8_t yaw() const { return _ch_yaw; }

private:
    // channel mappings
    int8_t _ch_roll;
    int8_t _ch_pitch;
    int8_t _ch_throttle;
    int8_t _ch_yaw;
};
