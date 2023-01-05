#include "Copter.h"

#define ARM_DELAY               20  // called at 10hz so 2 seconds
#define DISARM_DELAY            20  // called at 10hz so 2 seconds
#define AUTO_TRIM_DELAY         100 // called at 10hz so 10 seconds
#define LOST_VEHICLE_DELAY      10  // called at 10hz so 1 second

static uint32_t auto_disarm_begin;

#include "ap_hal.h"
extern AP_HAL hal;
// arm_motors_check - checks for pilot input to arm or disarm the copter
// called at 10hz
void Copter::arm_motors_check()
{
    static int16_t arming_counter;

    // ensure throttle is down
    if (channel_throttle->get_control_in() > 0) {
        arming_counter = 0;
        return;
    }

    int16_t tmp_thr = channel_throttle->get_control_in(); // range to 1000
    int16_t tmp_roll = channel_roll->get_control_in();
    int16_t tmp_pitch = channel_pitch->get_control_in();
    int16_t tmp_yaw = channel_yaw->get_control_in();

    bool arm_flag = (tmp_thr < 100 && tmp_yaw > 4000 && tmp_pitch > 4000 && tmp_roll < -4000);
    bool disarm_flag = (tmp_thr < 100 && tmp_yaw < -4000 && tmp_pitch > 4000 && tmp_roll > 4000);

    // full right
    if (arm_flag) {

        // increase the arming counter to a maximum of 1 beyond the auto trim counter
        if( arming_counter <= ARM_DELAY ) {
            arming_counter++;
        }

        // arm the motors and configure for flight
        if (arming_counter == ARM_DELAY && !motors->armed()) {
            // reset arming counter if arming fail
            if (!init_arm_motors(false)) {
                arming_counter = 0;
            }
        }

    // full left
    }else if (disarm_flag) {
        if (!mode_has_manual_throttle(control_mode) && !ap.land_complete) {
            arming_counter = 0;
            return;
        }

        // increase the counter to a maximum of 1 beyond the disarm delay
        if( arming_counter <= DISARM_DELAY ) {
            arming_counter++;
        }

        // disarm the motors
        if (arming_counter == DISARM_DELAY && motors->armed()) {
            init_disarm_motors();
        }

    // Yaw is centered so reset arming counter
    }else{
        arming_counter = 0;
    }
}

// auto_disarm_check - disarms the copter if it has been sitting on the ground in manual mode with throttle low for at least 15 seconds
void Copter::auto_disarm_check()
{
    uint32_t tnow_ms = millis();
    uint32_t disarm_delay_ms = 1000*apm_constrain_int16(g.disarm_delay, 0, 127);

    // exit immediately if we are already disarmed, or if auto
    // disarming is disabled
    if (!motors->armed() || disarm_delay_ms == 0 || control_mode == THROW) {
        auto_disarm_begin = tnow_ms;
        return;
    }
    // always allow auto disarm if using interlock switch or motors are Emergency Stopped
    if ((ap.using_interlock && !motors->get_interlock()) || ap.motor_emergency_stop) {

        // use a shorter delay if using throttle interlock switch or Emergency Stop, because it is less
        // obvious the copter is armed as the motors will not be spinning
        disarm_delay_ms /= 2;
    } else {
        bool sprung_throttle_stick = (g.throttle_behavior & THR_BEHAVE_FEEDBACK_FROM_MID_STICK) != 0;
        bool thr_low;
        if (mode_has_manual_throttle(control_mode) || !sprung_throttle_stick) {
            thr_low = ap.throttle_zero;
        } else {
            float deadband_top = channel_throttle->get_control_mid() + g.throttle_deadzone;
            thr_low = channel_throttle->get_control_in() <= deadband_top;
        }

        if (!thr_low || !ap.land_complete) {
            // reset timer
            auto_disarm_begin = tnow_ms;
        }
    }

    // disarm once timer expires
    if ((tnow_ms-auto_disarm_begin) >= disarm_delay_ms) {
        init_disarm_motors();
        auto_disarm_begin = tnow_ms;
    }
}

// init_arm_motors - performs arming process including initialisation of barometer and gyros
//  returns false if arming failed because of pre-arm checks, arming checks or a gyro calibration failure
bool Copter::init_arm_motors(bool arming_from_gcs)
{
    static bool in_arm_motors = false;

    // exit immediately if already in this function
    if (in_arm_motors) {
        return false;
    }
    in_arm_motors = true;

    // return true if already armed
    if (motors->armed()) {
        in_arm_motors = false;
        return true;
    }

/*    // run pre-arm-checks and display failures
    if (!arming.all_checks_passing(arming_from_gcs)) {
        AP_Notify::events.arming_failed = true;
        in_arm_motors = false;
        return false;
    }*/

    // disable cpu failsafe because initialising everything takes a while
    //failsafe_disable();

    // reset battery failsafe
    //set_failsafe_battery(false);

    // notify that arming will occur (we do this early to give plenty of warning)
    AP_Notify::flags.armed = true;
    // call update_notify a few times to ensure the message gets out
/*    for (uint8_t i=0; i<=10; i++) {
        update_notify();
    }*/

    // Remember Orientation
    // --------------------

    initial_armed_bearing = ahrs.yaw_sensor;

//    if (ap.home_state == HOME_UNSET) {
        // Reset EKF altitude if home hasn't been set yet (we use EKF altitude as substitute for alt above home)
//        ahrs.resetHeightDatum();
//    } else 
    if (ap.home_state == HOME_UNSET || ap.home_state == HOME_SET_NOT_LOCKED) {
        // Reset home position if it has already been set before (but not locked)
        set_home_to_current_location();
    }
    //calc_distance_and_bearing();

    // enable gps velocity based centrefugal force compensation
    ahrs.set_correct_centrifugal(true);

    // enable output to motors
    enable_motor_output();

    // finally actually arm the motors
    motors->armed(true);

    // flag exiting this function
    in_arm_motors = false;

    // Log time stamp of arming event
    arm_time_ms = millis();

    // Start the arming delay
    ap.in_arming_delay = true;

    // return success
    return true;
}

// init_disarm_motors - disarm motors
void Copter::init_disarm_motors()
{
    // return immediately if we are already disarmed
    if (!motors->armed()) {
        return;
    }

    // we are not in the air
    set_land_complete(true);
    set_land_complete_maybe(true);

    // send disarm command to motors
    motors->armed(false);

    // disable gps velocity based centrefugal force compensation
    ahrs.set_correct_centrifugal(false);

    ap.in_arming_delay = false;
}

// motors_output - send output to motors library which will adjust and send to ESCs and servos
void Copter::motors_output()
{
    // Update arming delay state
    if (ap.in_arming_delay && (!motors->armed() || millis()-arm_time_ms > ARMING_DELAY_SEC*1.0e3f || control_mode == THROW)) {
        ap.in_arming_delay = false;
    }

    // output any servo channels
    SRV_Channels::calc_pwm();

    // cork now, so that all channel outputs happen at once
    /*hal.rcout->cork();*/
    
    // update output on any aux channels, for manual passthru
    SRV_Channels::output_ch_all();
    
    // check if we are performing the motor test
    if (ap.motor_test) {
        ;
    } else {
        bool interlock = motors->armed() && !ap.in_arming_delay && (!ap.using_interlock || ap.motor_interlock_switch) && !ap.motor_emergency_stop;
        if (!motors->get_interlock() && interlock) {
            motors->set_interlock(true);
        } else if (motors->get_interlock() && !interlock) {
            motors->set_interlock(false);
        }

        // send output signals to motors
        motors->output();
    }
}
