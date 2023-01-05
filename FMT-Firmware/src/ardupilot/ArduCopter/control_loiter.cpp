#include "Copter.h"

/*
 * Init and run calls for loiter flight mode
 */

// loiter_init - initialise loiter controller
bool Copter::loiter_init(bool ignore_checks)
{
    if (position_ok() || ignore_checks) {

        // set target to current position
        wp_nav->init_loiter_target();

        // initialize vertical speed and acceleration
        pos_control->set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
        pos_control->set_accel_z(g.pilot_accel_z);

        // initialise position and desired velocity
        if (!pos_control->is_active_z()) {
            pos_control->set_alt_target_to_current_alt();
            pos_control->set_desired_velocity_z(inertial_nav.get_velocity_z());
        }        
        Loiter_unit_vel_dir.x = 0.0f;
        Loiter_unit_vel_dir.y = 0.0f;
        Loiter_sub_state = Loiter_sub_Flying;
        Loiter_vel_trig = 0.0f;

        return true;
    }else{
        return false;
    }
}

// loiter_run - runs the loiter controller
// should be called at 100hz or more
void Copter::loiter_run()
{
    LoiterModeState loiter_state;
    float target_yaw_rate = 0.0f;
    float target_climb_rate = 0.0f;
    float takeoff_climb_rate = 0.0f;

    // initialize vertical speed and acceleration
    pos_control->set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
    pos_control->set_accel_z(g.pilot_accel_z);

    // process pilot inputs unless we are in radio failsafe
    if (!failsafe.radio) {
        // process pilot's roll and pitch input
        wp_nav->set_pilot_desired_acceleration(channel_roll->get_control_in(), channel_pitch->get_control_in());

        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

        // get pilot desired climb rate
        target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
        target_climb_rate = apm_constrain_float(target_climb_rate, -g.pilot_velocity_z_max, g.pilot_velocity_z_max);
    } else {
        // clear out pilot desired acceleration in case radio failsafe event occurs and we do not switch to RTL for some reason
        wp_nav->clear_pilot_desired_acceleration();
    }

    // relax loiter target if we might be landed
    if (ap.land_complete_maybe) {
        wp_nav->loiter_soften_for_landing();
    }

    bool takeoff_triggered = ap.land_complete && (target_climb_rate > 0.0f);

    // Loiter State Machine Determination
    if (!motors->armed() || !motors->get_interlock()) {
        loiter_state = Loiter_MotorStopped;
    } else if (takeoff_state.running || takeoff_triggered) {
        loiter_state = Loiter_Takeoff;
    } else if (!ap.auto_armed || ap.land_complete) {
        loiter_state = Loiter_Landed;
    } else {
        loiter_state = Loiter_Flying;
    }

    // Loiter State Machine
    switch (loiter_state) {

    case Loiter_MotorStopped:

        motors->set_desired_spool_state(AP_Motors::DESIRED_SHUT_DOWN);

        wp_nav->init_loiter_target();
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->set_yaw_target_to_current_heading();
        pos_control->relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero

        wp_nav->update_loiter(ekfGndSpdLimit, ekfNavVelGainScaler);
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), target_yaw_rate, get_smoothing_gain());
        pos_control->update_z_controller();
        break;

    case Loiter_Takeoff:
        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

        // initiate take-off
        if (!takeoff_state.running) {
            takeoff_timer_start(apm_constrain_float(g.pilot_takeoff_alt,0.0f,1000.0f));
            // indicate we are taking off
            set_land_complete(false);
            // clear i term when we're taking off
            set_throttle_takeoff();
        }

        // get takeoff adjusted pilot and takeoff climb rates
        takeoff_get_climb_rates(target_climb_rate, takeoff_climb_rate);

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // run loiter controller
        wp_nav->update_loiter(ekfGndSpdLimit, ekfNavVelGainScaler);

        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), target_yaw_rate, get_smoothing_gain());

        // update altitude target and call position controller
        pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
        pos_control->add_takeoff_climb_rate(takeoff_climb_rate, G_Dt);
        pos_control->update_z_controller();
        break;

    case Loiter_Landed:
        // set motors to spin-when-armed if throttle below deadzone, otherwise full range (but motors will only spin at min throttle)
        if (target_climb_rate < 0.0f) {
            motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        } else {
            motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
        }
        wp_nav->init_loiter_target();
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->set_yaw_target_to_current_heading();
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0, 0, 0, get_smoothing_gain());
        pos_control->relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero
        pos_control->update_z_controller();
        break;

    case Loiter_Flying:

        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

        // run loiter controller
        switch (Loiter_sub_state) {
            case Loiter_sub_Flying: {
                // run loiter controller
                wp_nav->update_loiter(ekfGndSpdLimit, ekfNavVelGainScaler);
                if (channel_roll->get_control_in() == 0 && channel_pitch->get_control_in() == 0) {
                    Loiter_sub_state = Loiter_sub_Stopping;
                    Loiter_unit_vel_dir.x = pos_control->get_desired_velocity().x;
                    Loiter_unit_vel_dir.y = pos_control->get_desired_velocity().y;
                    Loiter_vel_trig = Loiter_unit_vel_dir.length();
                    Loiter_unit_vel_dir.normalize();
                }
                }
                break;
            case Loiter_sub_Stopping: {
                wp_nav->update_loiter_stopping(ekfGndSpdLimit, ekfNavVelGainScaler);
                if (channel_roll->get_control_in() != 0 || channel_pitch->get_control_in() != 0) {
                    Loiter_sub_state = Loiter_sub_Flying;
                }
                Vector2f curr_vel(inertial_nav.get_velocity().x,inertial_nav.get_velocity().y);
                float current_vel = curr_vel.x * Loiter_unit_vel_dir.x + curr_vel.y * Loiter_unit_vel_dir.y;
                if (current_vel < Loiter_vel_trig * 0.1f) {
                    wp_nav->init_loiter_target(inertial_nav.get_position(), true);
                    pos_control->init_xy_controller_zero();
                    Loiter_sub_state = Loiter_sub_Holding;
                }
                }
                break;
            case Loiter_sub_Holding: {
                wp_nav->update_loiter(ekfGndSpdLimit, ekfNavVelGainScaler);
                if (channel_roll->get_control_in() != 0 || channel_pitch->get_control_in() != 0) {
                    Loiter_sub_state = Loiter_sub_Flying;
                }
                }
                break;
            default:
                break;
        }

        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), target_yaw_rate, get_smoothing_gain());

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // update altitude target and call position controller
        pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
        pos_control->update_z_controller();
        break;
    }
}
