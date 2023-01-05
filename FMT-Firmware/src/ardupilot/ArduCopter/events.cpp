#include "Copter.h"

/*
 *       This event will be called when the failsafe changes
 *       boolean failsafe reflects the current state
 */
void Copter::failsafe_radio_on_event()
{
    // if motors are not armed there is nothing to do
    if( !motors->armed() ) {
        return;
    }

    if (should_disarm_on_failsafe()) {
        init_disarm_motors();
    }/* else {
        if (control_mode == AUTO && g.failsafe_throttle == FS_THR_ENABLED_CONTINUE_MISSION) {
            // continue mission
        } else if (control_mode == LAND && g.failsafe_battery_enabled == FS_BATT_LAND && failsafe.battery) {
            // continue landing
        } else {
            if (g.failsafe_throttle == FS_THR_ENABLED_ALWAYS_LAND) {
                set_mode_land_with_pause(MODE_REASON_RADIO_FAILSAFE);
            } else {
                set_mode_RTL_or_land_with_pause(MODE_REASON_RADIO_FAILSAFE);
            }
        }
    }

    // log the error to the dataflash
    Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_RADIO, ERROR_CODE_FAILSAFE_OCCURRED);*/

}

bool Copter::should_disarm_on_failsafe() {
    if (ap.in_arming_delay) {
        return true;
    }

    switch(control_mode) {
        case STABILIZE:
        case ACRO:
            // if throttle is zero OR vehicle is landed disarm motors
            return ap.throttle_zero || ap.land_complete;
        case AUTO:
            // if mission has not started AND vehicle is landed, disarm motors
            return !ap.auto_armed && ap.land_complete;
        default:
            // used for AltHold, Guided, Loiter, RTL, Circle, Drift, Sport, Flip, Autotune, PosHold
            // if landed disarm
            return ap.land_complete;
    }
}

// set terrain data status (found or not found)
void Copter::failsafe_terrain_set_status(bool data_ok)
{
    uint32_t now = millis();

    // record time of first and latest failures (i.e. duration of failures)
    if (!data_ok) {
        failsafe.terrain_last_failure_ms = now;
        if (failsafe.terrain_first_failure_ms == 0) {
            failsafe.terrain_first_failure_ms = now;
        }
    } else {
        // failures cleared after 0.1 seconds of persistent successes
        if (now - failsafe.terrain_last_failure_ms > 100) {
            failsafe.terrain_last_failure_ms = 0;
            failsafe.terrain_first_failure_ms = 0;
        }
    }
}


// terrain failsafe action
void Copter::failsafe_terrain_on_event()
{
    failsafe.terrain = true;

    set_mode_land_with_pause(MODE_REASON_TERRAIN_FAILSAFE);
}
