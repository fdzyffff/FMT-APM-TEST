#include "Copter.h"

// home_is_set - returns true if home positions has been set (to GPS location, armed location or EKF origin)
bool Copter::home_is_set()
{
    return (ap.home_state == HOME_SET_NOT_LOCKED || ap.home_state == HOME_SET_AND_LOCKED);
}

// set_home_state - update home state
void Copter::set_home_state(enum HomeState new_home_state)
{
    // if no change, exit immediately
    if (ap.home_state == new_home_state)
        return;

    // update state
    ap.home_state = new_home_state;
}
// ---------------------------------------------
void Copter::set_auto_armed(bool b)
{
    // if no change, exit immediately
    if( ap.auto_armed == b )
        return;

    ap.auto_armed = b;
}

// ---------------------------------------------
void Copter::set_failsafe_radio(bool b)
{
    // only act on changes
    // -------------------
    if(failsafe.radio != b) {

        // store the value so we don't trip the gate twice
        // -----------------------------------------------
        failsafe.radio = b;

        if (failsafe.radio == false) {
            // We've regained radio contact
            // ----------------------------
            //failsafe_radio_off_event();
            ;
        }else{
            // We've lost radio contact
            // ------------------------
            //failsafe_radio_on_event();
            ;
        }

        // update AP_Notify
        AP_Notify::flags.failsafe_radio = b;
    }
}

// ---------------------------------------------

void Copter::update_using_interlock()
{
    // check if we are using motor interlock control on an aux switch or are in throw mode
    // which uses the interlock to stop motors while the copter is being thrown
    ap.using_interlock = check_if_auxsw_mode_used(AUXSW_MOTOR_INTERLOCK);
}

void Copter::set_motor_emergency_stop(bool b)
{
    if(ap.motor_emergency_stop != b) {
        ap.motor_emergency_stop = b;
    }
}
