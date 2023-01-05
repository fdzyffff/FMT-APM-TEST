#include "Copter.h"

/*****************************************************************************
*   The init_ardupilot function processes everything we need for an in - air restart
*        We will determine later if we are actually on the ground and process a
*        ground start in that case.
*
*****************************************************************************/
void Copter::init_ardupilot()
{
    ap.usb_connected = false;

    // update motor interlock state
    update_using_interlock();

    init_rc_in();               // sets up rc channels from radio

    // allocate the motors class
    allocate_motors();

    // sets up motors and output to escs
    init_rc_out();

    // motors initialised so parameters can be sent
    ap.initialised_params = true;

    // init Location class
    Location_Class::set_ahrs(&ahrs);

    attitude_control->parameter_sanity_check();
    pos_control->set_dt(MAIN_LOOP_SECONDS);

    // initialise the flight mode and aux switch
    // ---------------------------
    reset_control_switch();
    init_aux_switches();

    startup_INS_ground();

    // set landed flags
    set_land_complete(true);
    set_land_complete_maybe(true);

    // enable output to motors
    ap.pre_arm_rc_check = true;
    enable_motor_output();

    set_mode(AUTO, MODE_REASON_TX_COMMAND);
    // flag that initialisation has completed
    ap.initialised = true;
}


//******************************************************************************
//This function does all the calibrations, etc. that we need during a ground start
//******************************************************************************
void Copter::startup_INS_ground()
{
    // initialise ahrs (may push imu calibration into the mpu6000 if using that device).
    ahrs.init();
    ahrs.set_vehicle_class(AHRS_VEHICLE_COPTER);
    ahrs.reset();
}


// position_ok - returns true if the horizontal absolute position is ok and home position is set
bool Copter::position_ok()
{
    // return false if ekf failsafe has triggered
/*    if (failsafe.ekf) {
        return false;
    }*/

    // check ekf position estimate
    return (ekf_position_ok());
}

// ekf_position_ok - returns true if the ekf claims it's horizontal absolute position estimate is ok and home position is set
bool Copter::ekf_position_ok()
{
    return 0;
}

// update_auto_armed - update status of auto_armed flag
void Copter::update_auto_armed()
{
    // disarm checks
    if(ap.auto_armed){
        // if motors are disarmed, auto_armed should also be false
        if(!motors->armed()) {
            set_auto_armed(false);
            return;
        }
        // if in stabilize or acro flight mode and throttle is zero, auto-armed should become false
        if(mode_has_manual_throttle(control_mode) && ap.throttle_zero && !failsafe.radio) {
            set_auto_armed(false);
        }

    }else{
        // arm checks
        // if motors are armed and throttle is above zero auto_armed should be true
        // if motors are armed and we are in throw mode, then auto_ermed should be true
        if(motors->armed() && (!ap.throttle_zero || control_mode == THROW)) {
            set_auto_armed(true);
        }
    }
}

/*
  allocate the motors class
 */
void Copter::allocate_motors(void)
{
    switch ((AP_Motors::motor_frame_class)g2.frame_class) {
        case AP_Motors::MOTOR_FRAME_QUAD:
        case AP_Motors::MOTOR_FRAME_HEXA:
        case AP_Motors::MOTOR_FRAME_Y6:
        case AP_Motors::MOTOR_FRAME_OCTA:
        case AP_Motors::MOTOR_FRAME_OCTAQUAD:
        default:
            motors = new AP_MotorsMatrix(MAIN_LOOP_RATE);
            break;
    }


    AP_AHRS_View *ahrs_view = ahrs.create_view(ROTATION_NONE);

    attitude_control = new AC_AttitudeControl_Multi(*ahrs_view, aparm, *motors, MAIN_LOOP_SECONDS);
        
    pos_control = new AC_PosControl(*ahrs_view, inertial_nav, *motors, *attitude_control,
                                    g.p_alt_hold, g.p_vel_z, g.pid_accel_z,
                                    g.p_pos_xy, g.pi_vel_xy);

    wp_nav = new AC_WPNav(inertial_nav, *ahrs_view, *pos_control, *attitude_control);

    circle_nav = new AC_Circle(inertial_nav, *ahrs_view, *pos_control);

    
    // now setup some frame-class specific defaults
    switch ((AP_Motors::motor_frame_class)g2.frame_class) {
    case AP_Motors::MOTOR_FRAME_Y6:
/*        attitude_control->get_rate_roll_pid().kP() = (0.1);
        attitude_control->get_rate_roll_pid().kD() = (0.006);
        attitude_control->get_rate_pitch_pid().kP() = (0.1);
        attitude_control->get_rate_pitch_pid().kD() = (0.006);
        attitude_control->get_rate_yaw_pid().kP() = (0.15);
        attitude_control->get_rate_yaw_pid().kI() = (0.015);*/
        break;
    case AP_Motors::MOTOR_FRAME_TRI:
        //attitude_control->get_rate_yaw_pid().filt_hz() = (100);
        break;
    default:
        break;
    }
}
