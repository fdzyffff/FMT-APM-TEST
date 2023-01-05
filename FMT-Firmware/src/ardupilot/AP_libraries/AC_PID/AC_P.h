#pragma once

/// @file	AC_PD.h
/// @brief	Generic PID algorithm, with EEPROM-backed storage of constants.

#include "AP_Common.h"
#include "stdlib.h"
#include <cmath>

/// @class	AC_P
/// @brief	Object managing one P controller
class AC_P {
public:

    /// Constructor for P that saves its settings to EEPROM
    ///
    /// @note	PIs must be named to avoid either multiple parameters with the
    ///			same name, or an overly complex constructor.
    ///
    /// @param  initial_p       Initial value for the P term.
    ///
    AC_P(
        const float &   initial_p = 0.0f) :
        _kp(0.0f)
    {
        _kp = initial_p;
    }

    /// Iterate the P controller, return the new control value
    ///
    /// Positive error produces positive output.
    ///
    /// @param error	The measured error value
    /// @param dt		The time delta in milliseconds (note
    ///					that update interval cannot be more
    ///					than 65.535 seconds due to limited range
    ///					of the data type).
    ///
    /// @returns		The updated control output.
    ///
    float       get_p(float error) const;

    /// @name	parameter accessors
    //@{

    /// Overload the function call operator to permit relatively easy initialisation
    void operator() (const float p) { _kp = p; }

    // accessors
    float       kP() { return _kp; }
    void        kP(const float v) { _kp = v; }

private:
    float        _kp;
};
