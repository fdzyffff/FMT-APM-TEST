#include "Copter.h"

/*
 * High level calls to set and update flight modes logic for individual
 * flight modes is in control_acro.cpp, control_stabilize.cpp, etc
 */

// set_mode - change flight mode and perform any necessary initialisation
// optional force parameter used to force the flight mode change (used only first time mode is set)
// returns true if mode was successfully set
// ACRO, STABILIZE, ALTHOLD, LAND, DRIFT and SPORT can always be set successfully but the return state of other flight modes should be checked and the caller should deal with failures appropriately
bool Copter::set_mode(control_mode_t mode, mode_reason_t reason)
{
    // boolean to record if flight mode could be set
    bool success = false;
    bool ignore_checks = !motors->armed();   // allow switching to any mode if disarmed.  We rely on the arming check to perform

    // return immediately if we are already in the desired mode
    if (mode == control_mode) {
        prev_control_mode = control_mode;
        prev_control_mode_reason = control_mode_reason;

        control_mode_reason = reason;
        return true;
    }

    switch (mode) {
//        case ACRO:
//            success = acro_init(ignore_checks);
//            break;

        case STABILIZE:
            success = stabilize_init(ignore_checks);
            break;

        case ALT_HOLD:
            success = althold_init(ignore_checks);
            break;

        case AUTO:
        case TESTSTAR:
            success = auto_init(ignore_checks);
            break;

        case CIRCLE:
            success = circle_init(ignore_checks);
            break;

        case LOITER:
            success = loiter_init(ignore_checks);
            break;

        case GUIDED:
            success = guided_init(ignore_checks);
            break;

        case LAND:
           success = land_init(ignore_checks);
           break;

        case RTL:
           success = rtl_init(ignore_checks);
           break;
//
//        case DRIFT:
//            success = drift_init(ignore_checks);
//            break;
//
//        case SPORT:
//            success = sport_init(ignore_checks);
//            break;
//
//        case FLIP:
//            success = flip_init(ignore_checks);
//            break;
//
//        case BRAKE:
//            success = brake_init(ignore_checks);
//            break;
//
//        case THROW:
//            success = throw_init(ignore_checks);
//            break;
//
//        case AVOID_ADSB:
//            success = avoid_adsb_init(ignore_checks);
//            break;
//
//        case GUIDED_NOGPS:
//            success = guided_nogps_init(ignore_checks);
//            break;

        default:
            success = false;
            break;
    }
    
    // update flight mode
    if (success) {
        // perform any cleanup required by previous flight mode
        exit_mode(control_mode, mode);
        
        prev_control_mode = control_mode;
        prev_control_mode_reason = control_mode_reason;

        control_mode = mode;
        control_mode_reason = reason;

    } else {
        // Log error that we failed to enter desired flight mode
        ;
    }

    // update notify object
    if (success) {
        notify_flight_mode(control_mode);
    }

    // return success or failure
    return success;
}

// update_flight_mode - calls the appropriate attitude controllers based on flight mode
// called at 100hz or more
void Copter::update_flight_mode()
{
    // Update EKF speed limit - used to limit speed when we are using optical flow
    ahrs.getEkfControlLimits(ekfGndSpdLimit, ekfNavVelGainScaler);

    switch (control_mode) {
//        case ACRO:
//            acro_run();
//            break;
//
        case STABILIZE:
            stabilize_run();
            break;

        case ALT_HOLD:
            althold_run();
            break;

        case AUTO:
        case TESTSTAR:
            auto_run();
            break;

        case CIRCLE:
            circle_run();
            break;

        case LOITER:
            loiter_run();
            break;

        case GUIDED:
            guided_run();
            break;

        case LAND:
            land_run();
            break;

        case RTL:
            rtl_run();
            break;

//        case DRIFT:
//            drift_run();
//            break;
//
//        case SPORT:
//            sport_run();
//            break;
//
//        case FLIP:
//            flip_run();
//            break;
//
//        case BRAKE:
//            brake_run();
//            break;
//
//        case THROW:
//            throw_run();
//            break;
//
//        case AVOID_ADSB:
//            avoid_adsb_run();
//            break;
//
//        case GUIDED_NOGPS:
//            guided_nogps_run();
//            break;
            
        default:
            break;
    }
}

// exit_mode - high level call to organise cleanup as a flight mode is exited
void Copter::exit_mode(control_mode_t old_control_mode, control_mode_t new_control_mode)
{
    // smooth throttle transition when switching from manual to automatic flight modes
    if (mode_has_manual_throttle(old_control_mode) && !mode_has_manual_throttle(new_control_mode) && motors->armed() && !ap.land_complete) {
        // this assumes all manual flight modes use get_pilot_desired_throttle to translate pilot input to output throttle
        set_accel_throttle_I_from_pilot_throttle();
    }

    if (old_control_mode == GUIDED) {
        guided_mode = Guided_TakeOff;
    }

    // cancel any takeoffs in progress
    takeoff_stop();
}

// returns true or false whether mode requires GPS
bool Copter::mode_requires_GPS(control_mode_t mode)
{
    switch (mode) {
        case AUTO:
        case GUIDED:
        case LOITER:
        case RTL:
        case CIRCLE:
        case DRIFT:
        case POSHOLD:
        case BRAKE:
        case AVOID_ADSB:
        case THROW:
        case TESTSTAR:
            return true;
        default:
            return false;
    }
}

// mode_has_manual_throttle - returns true if the flight mode has a manual throttle (i.e. pilot directly controls throttle)
bool Copter::mode_has_manual_throttle(control_mode_t mode)
{
    switch (mode) {
        case ACRO:
        case STABILIZE:
            return true;
        default:
            return false;
    }
}

// mode_allows_arming - returns true if vehicle can be armed in the specified mode
//  arming_from_gcs should be set to true if the arming request comes from the ground station
bool Copter::mode_allows_arming(control_mode_t mode, bool arming_from_gcs)
{
    if (mode_has_manual_throttle(mode) || mode == LOITER || mode == ALT_HOLD || mode == POSHOLD || mode == DRIFT || mode == SPORT || mode == THROW || (arming_from_gcs && (mode == GUIDED || mode == GUIDED_NOGPS))) {
        return true;
    }
    return false;
}

// notify_flight_mode - sets notify object based on flight mode.  Only used for OreoLED notify device
void Copter::notify_flight_mode(control_mode_t mode)
{
    AP_Notify::flags.flight_mode = mode;

    switch (mode) {
        case AUTO:
        case GUIDED:
        case RTL:
        case CIRCLE:
        case AVOID_ADSB:
        case GUIDED_NOGPS:
        case LAND:
        case TESTSTAR:
            // autopilot modes
            AP_Notify::flags.autopilot_mode = true;
            break;
        default:
            // all other are manual flight modes
            AP_Notify::flags.autopilot_mode = false;
            break;
    }
/*
    // set flight mode string
    switch (mode) {
        case STABILIZE:
            notify.set_flight_mode_str("STAB");
            break;
        case ACRO:
            notify.set_flight_mode_str("ACRO");
            break;
        case ALT_HOLD:
            notify.set_flight_mode_str("ALTH");
            break;
        case AUTO:
            notify.set_flight_mode_str("AUTO");
            break;
        case GUIDED:
            notify.set_flight_mode_str("GUID");
            break;
        case LOITER:
            notify.set_flight_mode_str("LOIT");
            break;
        case RTL:
            notify.set_flight_mode_str("RTL ");
            break;
        case CIRCLE:
            notify.set_flight_mode_str("CIRC");
            break;
        case LAND:
            notify.set_flight_mode_str("LAND");
            break;
        case DRIFT:
            notify.set_flight_mode_str("DRIF");
            break;
        case SPORT:
            notify.set_flight_mode_str("SPRT");
            break;
        case FLIP:
            notify.set_flight_mode_str("FLIP");
            break;
        case AUTOTUNE:
            notify.set_flight_mode_str("ATUN");
            break;
        case POSHOLD:
            notify.set_flight_mode_str("PHLD");
            break;
        case BRAKE:
            notify.set_flight_mode_str("BRAK");
            break;
        case THROW:
            notify.set_flight_mode_str("THRW");
            break;
        case AVOID_ADSB:
            notify.set_flight_mode_str("AVOI");
            break;
        case GUIDED_NOGPS:
            notify.set_flight_mode_str("GNGP");
            break;
        default:
            notify.set_flight_mode_str("----");
            break;
    }*/
}

