#include "Copter.h"

/*
 * Init and run calls for guided flight mode
 */

#ifndef GUIDED_LOOK_AT_TARGET_MIN_DISTANCE_CM
 # define GUIDED_LOOK_AT_TARGET_MIN_DISTANCE_CM     500     // point nose at target if it is more than 5m away
#endif

#define GUIDED_POSVEL_TIMEOUT_MS    3000    // guided mode's position-velocity controller times out after 3seconds with no new updates
#define GUIDED_ATTITUDE_TIMEOUT_MS  1000    // guided mode's attitude controller times out after 1 second with no new updates

static Vector3f guided_pos_target_cm;       // position target (used by posvel controller only)
static Vector3f guided_vel_target_cms;      // velocity target (used by velocity controller and posvel controller)
static uint32_t posvel_update_time_ms;      // system time of last target update to posvel controller (i.e. position and velocity update)
static uint32_t vel_update_time_ms;         // system time of last target update to velocity controller
static bool fp_use_vel_z;
static float fp_target_cms_z;

struct {
    uint32_t update_time_ms;
    float roll_cd;
    float pitch_cd;
    float yaw_cd;
    float yaw_rate_cds;
    float climb_rate_cms;
    bool use_yaw_rate;
    float throttle_in;
} static guided_angle_state;

struct Guided_Limit {
    uint32_t timeout_ms;  // timeout (in seconds) from the time that guided is invoked
    float alt_min_cm;   // lower altitude limit in cm above home (0 = no limit)
    float alt_max_cm;   // upper altitude limit in cm above home (0 = no limit)
    float horiz_max_cm; // horizontal position limit in cm from where guided mode was initiated (0 = no limit)
    uint32_t start_time;// system time in milliseconds that control was handed to the external computer
    Vector3f start_pos; // start position as a distance from home in cm.  used for checking horiz_max limit
} guided_limit;

// guided_init - initialise guided controller
bool Copter::guided_init(bool ignore_checks)
{
    guided_yaw_base_angle_filt.set_cutoff_frequency(RANGEFINDER_WPNAV_FILT_HZ);
    if (position_ok() || ignore_checks) {
        // initialise yaw
        set_auto_yaw_mode(get_default_auto_yaw_mode(false));
        // start in position control mode
        guided_pos_control_start();
        return true;
    } else if (guided_mode == Guided_Angle) {
        guided_angle_control_start();
        return true;
    } else if (guided_mode == Guided_Angle_Stab) {
        guided_angle_stab_start();
        return true;
    } else{
        return false;
    }
}


// guided_takeoff_start - initialises waypoint controller to implement take-off
bool Copter::guided_takeoff_start(float final_alt_above_home)
{
    guided_mode = Guided_TakeOff;

    // initialise wpnav destination
    Location_Class target_loc = current_loc;
    target_loc.set_alt_cm(final_alt_above_home, Location_Class::ALT_FRAME_ABOVE_HOME);

    if (!wp_nav->set_wp_destination(target_loc)) {
        // failure to set destination can only be because of missing terrain data
        // failure is propagated to GCS with NAK
        return false;
    }

    // initialise yaw
    set_auto_yaw_mode(AUTO_YAW_HOLD);

    // clear i term when we're taking off
    set_throttle_takeoff();

    // get initial alt for WP_NAVALT_MIN
    auto_takeoff_set_start_alt();
    
    return true;
}

// initialise guided mode's position controller
void Copter::guided_pos_control_start()
{
    // set to position control mode
    guided_mode = Guided_WP;

    // initialise waypoint and spline controller
    wp_nav->wp_and_spline_init();
	
	// initialize vertical speeds and acceleration
    pos_control->set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
    pos_control->set_accel_z(g.pilot_accel_z);

    // initialise wpnav to stopping point
    Vector3f stopping_point;
    wp_nav->get_wp_stopping_point(stopping_point);

    // no need to check return status because terrain data is not used
    wp_nav->set_wp_destination(stopping_point, false);

    // initialise yaw
    set_auto_yaw_mode(get_default_auto_yaw_mode(false));
}

// initialise guided mode's velocity controller
void Copter::guided_vel_control_start()
{
    // set guided_mode to velocity controller
    guided_mode = Guided_Velocity;

    // initialise horizontal speed, acceleration and jerk
    pos_control->set_speed_xy(wp_nav->get_speed_xy());
    pos_control->set_accel_xy(wp_nav->get_wp_acceleration());
    pos_control->set_jerk_xy_to_default();

    // initialize vertical speeds and acceleration
    pos_control->set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
    pos_control->set_accel_z(g.pilot_accel_z);

    // initialise velocity controller
    pos_control->init_vel_controller_xyz();
}

// initialise guided mode's posvel controller
void Copter::guided_posvel_control_start()
{
    // set guided_mode to velocity controller
    guided_mode = Guided_PosVel;

    pos_control->init_xy_controller();

    // set speed and acceleration from wpnav's speed and acceleration
    pos_control->set_speed_xy(wp_nav->get_speed_xy());
    pos_control->set_accel_xy(wp_nav->get_wp_acceleration());
    pos_control->set_jerk_xy_to_default();

    const Vector3f& curr_pos = inertial_nav.get_position();
    const Vector3f& curr_vel = inertial_nav.get_velocity();

    // set target position and velocity to current position and velocity
    pos_control->set_xy_target(curr_pos.x, curr_pos.y);
    pos_control->set_desired_velocity_xy(curr_vel.x, curr_vel.y);

    // set vertical speed and acceleration
    pos_control->set_speed_z(wp_nav->get_speed_down(), wp_nav->get_speed_up());
    pos_control->set_accel_z(wp_nav->get_accel_z());

    // pilot always controls yaw
    set_auto_yaw_mode(AUTO_YAW_HOLD);
}

// initialise guided mode's angle controller
void Copter::guided_angle_control_start()
{
    // set guided_mode to velocity controller
    guided_mode = Guided_Angle;

    // set vertical speed and acceleration
    pos_control->set_speed_z(wp_nav->get_speed_down(), wp_nav->get_speed_up());
    pos_control->set_accel_z(wp_nav->get_accel_z());

    // initialise position and desired velocity
    if (!pos_control->is_active_z()) {
        pos_control->set_alt_target_to_current_alt();
        pos_control->set_desired_velocity_z(inertial_nav.get_velocity_z());
    }

    // initialise targets
    guided_angle_state.update_time_ms = millis();
    guided_angle_state.roll_cd = ahrs.roll_sensor;
    guided_angle_state.pitch_cd = ahrs.pitch_sensor;
    guided_angle_state.yaw_cd = ahrs.yaw_sensor;
    guided_angle_state.climb_rate_cms = 0.0f;
    guided_angle_state.yaw_rate_cds = 0.0f;
    guided_angle_state.use_yaw_rate = false;
    guided_angle_state.throttle_in = 0.0f;

    // pilot always controls yaw
    set_auto_yaw_mode(AUTO_YAW_HOLD);
}

// initialise guided mode's angle controller
void Copter::guided_angle_stab_start()
{
    // set guided_mode to velocity controller
    guided_mode = Guided_Angle_Stab;

    // initialise targets
    guided_angle_state.update_time_ms = millis();
    guided_angle_state.roll_cd = ahrs.roll_sensor;
    guided_angle_state.pitch_cd = ahrs.pitch_sensor;
    guided_angle_state.yaw_cd = ahrs.yaw_sensor;
    guided_angle_state.climb_rate_cms = 0.0f;
    guided_angle_state.yaw_rate_cds = 0.0f;
    guided_angle_state.use_yaw_rate = false;
    guided_angle_state.throttle_in = 0.0f;

    // pilot always controls yaw
    set_auto_yaw_mode(AUTO_YAW_HOLD);
}
// guided_set_destination - sets guided mode's target destination
// Returns true if the fence is enabled and guided waypoint is within the fence
// else return false if the waypoint is outside the fence
bool Copter::guided_set_destination(const Vector3f& destination, bool use_vel_z, float target_z, bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_yaw)
{
    // ensure we are in position control mode
    if (guided_mode != Guided_WP) {
        guided_pos_control_start();
    }

		fp_use_vel_z = use_vel_z;
		fp_target_cms_z = target_z;
    // set yaw state
    guided_set_yaw_state(use_yaw, yaw_cd, use_yaw_rate, yaw_rate_cds, relative_yaw);

    // no need to check return status because terrain data is not used
    wp_nav->set_wp_destination(destination, false);

    return true;
}

// sets guided mode's target from a Location object
// returns false if destination could not be set (probably caused by missing terrain data)
// or if the fence is enabled and guided waypoint is outside the fence
bool Copter::guided_set_destination(const Location_Class& dest_loc, bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_yaw)
{
    // ensure we are in position control mode
    if (guided_mode != Guided_WP) {
        guided_pos_control_start();
    }
    if (!wp_nav->set_wp_destination(dest_loc)) {
        // failure is propagated to GCS with NAK
        return false;
    }
		
    // set yaw state
    guided_set_yaw_state(use_yaw, yaw_cd, use_yaw_rate, yaw_rate_cds, relative_yaw);

    return true;
}

// guided_set_velocity - sets guided mode's target velocity
void Copter::guided_set_velocity(const Vector3f& velocity, bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_yaw)
{
    // check we are in velocity control mode
    if (guided_mode != Guided_Velocity) {
        guided_vel_control_start();
    }

    // set yaw state
    guided_set_yaw_state(use_yaw, yaw_cd, use_yaw_rate, yaw_rate_cds, relative_yaw);

    // record velocity target
    guided_vel_target_cms = velocity;
    vel_update_time_ms = millis();
}

// set guided mode posvel target
// set guided mode posvel target
void Copter::guided_set_destination_posvel(const Vector3f& destination, const Vector3f& velocity, bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_yaw)
{
    // check we are in velocity control mode
    if (guided_mode != Guided_PosVel) {
        guided_posvel_control_start();
    }

    // set yaw state
    guided_set_yaw_state(use_yaw, yaw_cd, use_yaw_rate, yaw_rate_cds, relative_yaw);

    posvel_update_time_ms = millis();
    guided_pos_target_cm = destination;
    guided_vel_target_cms = velocity;

    pos_control->set_pos_target(guided_pos_target_cm);
}

// set guided mode angle target
void Copter::guided_set_angle(const float roll, const float pitch, const float yaw, float climb_rate_cms, bool use_yaw_rate, float yaw_rate_rads)
{
    // check we are in velocity control mode
    if (guided_mode != Guided_Angle) {
        guided_angle_control_start();
    }

    // convert quaternion to euler angles
    guided_angle_state.roll_cd = roll * 100.0f;
    guided_angle_state.pitch_cd = pitch * 100.0f;
    guided_angle_state.yaw_cd = wrap_180_cd(yaw) * 100.0f;
    guided_angle_state.yaw_rate_cds = ToDeg(yaw_rate_rads) * 100.0f;
    guided_angle_state.use_yaw_rate = use_yaw_rate;

    guided_angle_state.climb_rate_cms = climb_rate_cms;
    guided_angle_state.update_time_ms = millis();

    // interpret positive climb rate as triggering take-off
    if (motors->armed() && !ap.auto_armed && (guided_angle_state.climb_rate_cms > 0.0f)) {
        set_auto_armed(true);
    }
}


// set guided mode angle target
void Copter::guided_set_angle_stab(const float roll, const float pitch, const float yaw, float thr_in, bool use_yaw_rate, float yaw_rate_rads)
{
    // check we are in velocity control mode
    if (guided_mode != Guided_Angle_Stab) {
        //guided_angle_control_start();
		guided_angle_stab_start();
    }

    // convert quaternion to euler angles
    guided_angle_state.roll_cd = roll * 100.0f;
    guided_angle_state.pitch_cd = pitch * 100.0f;
    guided_angle_state.yaw_cd = wrap_180_cd(yaw) * 100.0f;
    guided_angle_state.yaw_rate_cds = ToDeg(yaw_rate_rads) * 100.0f;
    guided_angle_state.use_yaw_rate = use_yaw_rate;

    guided_angle_state.throttle_in = thr_in;
    guided_angle_state.update_time_ms = millis();

    // interpret positive climb rate as triggering take-off
    if (motors->armed() && !ap.auto_armed && (guided_angle_state.climb_rate_cms > 0.0f)) {
        set_auto_armed(true);
    }
}
// guided_run - runs the guided controller
// should be called at 100hz or more
void Copter::guided_run()
{
    // call the correct auto controller
    switch (guided_mode) {

    case Guided_TakeOff:
        // run takeoff controller
        guided_takeoff_run();
        break;

    case Guided_WP:
        // run position controller
        guided_pos_control_run();
        break;

    case Guided_Velocity:
        // run velocity controller
        guided_vel_control_run();
        break;

    case Guided_PosVel:
        // run position-velocity controller
        guided_posvel_control_run();
        break;

    case Guided_Angle:
        // run angle controller
        guided_angle_control_run();
        break;

    case Guided_Angle_Stab:
        // run angle controller
        guided_angle_stab_run();
        break;
    }
 }

// guided_takeoff_run - takeoff in guided mode
//      called by guided_run at 100hz or more
void Copter::guided_takeoff_run()
{
    // if not auto armed or motor interlock not enabled set throttle to zero and exit immediately
    if (!motors->armed() || !ap.auto_armed || !motors->get_interlock()) {
        // initialise wpnav targets
        wp_nav->shift_wp_origin_to_current_pos();
        // multicopters do not stabilize roll/pitch/yaw when disarmed
        motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        // reset attitude control targets
        attitude_control->set_throttle_out_unstabilized(0,true,g.throttle_filt);
        // clear i term when we're taking off
        set_throttle_takeoff();
        return;
    }

    // process pilot's yaw input
    float target_yaw_rate = 0;
    if (!failsafe.radio) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
    }

    set_land_complete(false);

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // run waypoint controller
    failsafe_terrain_set_status(wp_nav->update_wpnav());

    // call z-axis position controller (wpnav should have already updated it's alt target)
    pos_control->update_z_controller();

    // call attitude controller
    auto_takeoff_attitude_run(target_yaw_rate);
}

// guided_pos_control_run - runs the guided position controller
// called from guided_run
void Copter::guided_pos_control_run()
{
    // if not auto armed or motors not enabled set throttle to zero and exit immediately
    if (!motors->armed() || !ap.auto_armed || !motors->get_interlock() || ap.land_complete) {
        motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        // multicopters do not stabilize roll/pitch/yaw when disarmed
        attitude_control->set_throttle_out_unstabilized(0,true,g.throttle_filt);
        return;
    }

    // process pilot's yaw input
    float target_yaw_rate = 0;
    if (!failsafe.radio) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
        if (!is_zero(target_yaw_rate)) {
            set_auto_yaw_mode(AUTO_YAW_HOLD);
        }
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // run waypoint controller
    failsafe_terrain_set_status(wp_nav->update_wpnav());
		
		if (fp_use_vel_z) {
        pos_control->set_alt_target_from_climb_rate_ff(fp_target_cms_z, G_Dt, false);
    } 
		
    // call z-axis position controller (wpnav should have already updated it's alt target)
    pos_control->update_z_controller();

    // call attitude controller
    if (auto_yaw_mode == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), target_yaw_rate, get_smoothing_gain());
    } else if (auto_yaw_mode == AUTO_YAW_RATE) {
        // roll & pitch from waypoint controller, yaw rate from mavlink command or mission item
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), get_auto_yaw_rate_cds(), get_smoothing_gain());
    } else {
        // roll, pitch from waypoint controller, yaw heading from GCS or auto_heading()
        attitude_control->input_euler_angle_roll_pitch_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), get_auto_heading(), true, get_smoothing_gain());
    }
}

// guided_vel_control_run - runs the guided velocity controller
// called from guided_run
void Copter::guided_vel_control_run()
{
    // if not auto armed or motors not enabled set throttle to zero and exit immediately
    if (!motors->armed() || !ap.auto_armed || !motors->get_interlock() || ap.land_complete) {
        // initialise velocity controller
        pos_control->init_vel_controller_xyz();

        motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        // multicopters do not stabilize roll/pitch/yaw when disarmed
        attitude_control->set_throttle_out_unstabilized(0,true,g.throttle_filt);
        return;
    }

    // process pilot's yaw input
    float target_yaw_rate = 0;
    if (!failsafe.radio) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
        if (!is_zero(target_yaw_rate)) {
            set_auto_yaw_mode(AUTO_YAW_HOLD);
        }
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // set velocity to zero and stop rotating if no updates received for 3 seconds
    uint32_t tnow = millis();
    if (tnow - vel_update_time_ms > GUIDED_POSVEL_TIMEOUT_MS) {
        if (!pos_control->get_desired_velocity().is_zero()) {
            guided_set_desired_velocity_with_accel_and_fence_limits(Vector3f(0.0f, 0.0f, 0.0f));
        }
        if (auto_yaw_mode == AUTO_YAW_RATE) {
            set_auto_yaw_rate(0.0f);
        }
    } else {
        guided_set_desired_velocity_with_accel_and_fence_limits(guided_vel_target_cms);
    }

    // call velocity controller which includes z axis controller
    pos_control->update_vel_controller_xyz(ekfNavVelGainScaler);

    // call attitude controller
    if (auto_yaw_mode == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(pos_control->get_roll(), pos_control->get_pitch(), target_yaw_rate, get_smoothing_gain());
    } else if (auto_yaw_mode == AUTO_YAW_RATE) {
        // roll & pitch from velocity controller, yaw rate from mavlink command or mission item
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(pos_control->get_roll(), pos_control->get_pitch(), get_auto_yaw_rate_cds(), get_smoothing_gain());
    } else {
        // roll, pitch from waypoint controller, yaw heading from GCS or auto_heading()
        attitude_control->input_euler_angle_roll_pitch_yaw(pos_control->get_roll(), pos_control->get_pitch(), get_auto_heading(), true, get_smoothing_gain());
    }
}

// guided_posvel_control_run - runs the guided spline controller
// called from guided_run
void Copter::guided_posvel_control_run()
{
    // if not auto armed or motors not enabled set throttle to zero and exit immediately
    if (!motors->armed() || !ap.auto_armed || !motors->get_interlock() || ap.land_complete) {
        // set target position and velocity to current position and velocity
        pos_control->set_pos_target(inertial_nav.get_position());
        pos_control->set_desired_velocity(Vector3f(0,0,0));

        motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        // multicopters do not stabilize roll/pitch/yaw when disarmed
        attitude_control->set_throttle_out_unstabilized(0,true,g.throttle_filt);
        return;
    }

    // process pilot's yaw input
    float target_yaw_rate = 0;

    if (!failsafe.radio) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
        if (!is_zero(target_yaw_rate)) {
            set_auto_yaw_mode(AUTO_YAW_HOLD);
        }
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // set velocity to zero and stop rotating if no updates received for 3 seconds
    uint32_t tnow = millis();
    if (tnow - posvel_update_time_ms > GUIDED_POSVEL_TIMEOUT_MS) {
        guided_vel_target_cms.zero();
        if (auto_yaw_mode == AUTO_YAW_RATE) {
            set_auto_yaw_rate(0.0f);
        }
    }

    // calculate dt
    float dt = pos_control->time_since_last_xy_update();

    // update at poscontrol update rate
    if (dt >= pos_control->get_dt_xy()) {
        // sanity check dt
        if (dt >= 0.2f) {
            dt = 0.0f;
        }

        // advance position target using velocity target
        guided_pos_target_cm.x += guided_vel_target_cms.x * dt;
        guided_pos_target_cm.y += guided_vel_target_cms.y * dt;

        // send position and velocity targets to position controller
        pos_control->set_pos_target(guided_pos_target_cm);
        pos_control->set_desired_velocity_xy(guided_vel_target_cms.x, guided_vel_target_cms.y);

        // run position controller
        pos_control->update_xy_controller(AC_PosControl::XY_MODE_POS_AND_VEL_FF, ekfNavVelGainScaler, false);
    }

    pos_control->update_z_controller();
		
    // call attitude controller
    if (auto_yaw_mode == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(pos_control->get_roll(), pos_control->get_pitch(), target_yaw_rate, get_smoothing_gain());
    } else if (auto_yaw_mode == AUTO_YAW_RATE) {
        // roll & pitch from position-velocity controller, yaw rate from mavlink command or mission item
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(pos_control->get_roll(), pos_control->get_pitch(), get_auto_yaw_rate_cds(), get_smoothing_gain());
    } else {
        // roll, pitch from waypoint controller, yaw heading from GCS or auto_heading()
        attitude_control->input_euler_angle_roll_pitch_yaw(pos_control->get_roll(), pos_control->get_pitch(), get_auto_heading(), true, get_smoothing_gain());
    }
}

// guided_angle_control_run - runs the guided angle controller
// called from guided_run
void Copter::guided_angle_control_run()
{
    // if not auto armed or motors not enabled set throttle to zero and exit immediately
    if (!motors->armed() || !ap.auto_armed || !motors->get_interlock() || (ap.land_complete && guided_angle_state.climb_rate_cms <= 0.0f)) {
        motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        // multicopters do not stabilize roll/pitch/yaw when disarmed
        attitude_control->set_throttle_out_unstabilized(0.0f,true,g.throttle_filt);
        pos_control->relax_alt_hold_controllers(0.0f);
        return;
    }

    // constrain desired lean angles
    float roll_in = guided_angle_state.roll_cd;
    float pitch_in = guided_angle_state.pitch_cd;
    float total_in = norm(roll_in, pitch_in);
    float angle_max = MIN(attitude_control->get_althold_lean_angle_max(), aparm.angle_max);
    if (total_in > angle_max) {
        float ratio = angle_max / total_in;
        roll_in *= ratio;
        pitch_in *= ratio;
    }

    // wrap yaw request
    float yaw_in = wrap_180_cd(guided_angle_state.yaw_cd);
    float yaw_rate_in = wrap_180_cd(guided_angle_state.yaw_rate_cds);

    // constrain climb rate
    float climb_rate_cms = apm_constrain_float(guided_angle_state.climb_rate_cms, -fabsf(wp_nav->get_speed_down()), wp_nav->get_speed_up());

    // get avoidance adjusted climb rate
    climb_rate_cms = get_avoidance_adjusted_climbrate(climb_rate_cms);

    // check for timeout - set lean angles and climb rate to zero if no updates received for 3 seconds
    uint32_t tnow = millis();
    if (tnow - guided_angle_state.update_time_ms > GUIDED_ATTITUDE_TIMEOUT_MS) {
        roll_in = 0.0f;
        pitch_in = 0.0f;
        climb_rate_cms = 0.0f;
        yaw_rate_in = 0.0f;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // call attitude controller
    if (guided_angle_state.use_yaw_rate) {
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(roll_in, pitch_in, yaw_rate_in, get_smoothing_gain());
    } else {
        attitude_control->input_euler_angle_roll_pitch_yaw(roll_in, pitch_in, yaw_in, true, get_smoothing_gain());
    }

    // call position controller
    pos_control->set_alt_target_from_climb_rate_ff(climb_rate_cms, G_Dt, false);
    pos_control->update_z_controller();
}

// guided_angle_control_run - runs the guided angle controller
// called from guided_run
void Copter::guided_angle_stab_run()
{
    // if not auto armed or motors not enabled set throttle to zero and exit immediately
    if (!motors->armed() || !ap.auto_armed || !motors->get_interlock() || (ap.land_complete && get_pilot_desired_throttle((int16_t)guided_angle_state.throttle_in) > get_non_takeoff_throttle()) ) {
        motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        // multicopters do not stabilize roll/pitch/yaw when disarmed
        attitude_control->set_throttle_out_unstabilized(0.0f,true,g.throttle_filt);
        pos_control->relax_alt_hold_controllers(0.0f);
        return;
    }

    // constrain desired lean angles
    float roll_in = guided_angle_state.roll_cd;
    float pitch_in = guided_angle_state.pitch_cd;
    float total_in = norm(roll_in, pitch_in);
    float angle_max = MIN(attitude_control->get_althold_lean_angle_max(), aparm.angle_max);
    float throttle_in = guided_angle_state.throttle_in;
    if (total_in > angle_max) {
        float ratio = angle_max / total_in;
        roll_in *= ratio;
        pitch_in *= ratio;
    }
    float pilot_throttle_scaled;

    // wrap yaw request
    float yaw_in = wrap_180_cd(guided_angle_state.yaw_cd);
    float yaw_rate_in = wrap_180_cd(guided_angle_state.yaw_rate_cds);

    // check for timeout - set lean angles and climb rate to zero if no updates received for 3 seconds
    uint32_t tnow = millis();
    if (tnow - guided_angle_state.update_time_ms > GUIDED_ATTITUDE_TIMEOUT_MS) {
        roll_in = 0.0f;
        pitch_in = 0.0f;
        yaw_rate_in = 0.0f;
        throttle_in = 0.0f;
        guided_angle_control_start();
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // get pilot's desired throttle
    pilot_throttle_scaled = get_pilot_desired_throttle((int16_t)throttle_in);

    // call attitude controller
    if (guided_angle_state.use_yaw_rate) {
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(roll_in, pitch_in, yaw_rate_in, get_smoothing_gain());
    } else {
        attitude_control->input_euler_angle_roll_pitch_yaw(roll_in, pitch_in, yaw_in, true, get_smoothing_gain());
    }

    // output pilot's throttle
    attitude_control->set_throttle_out(pilot_throttle_scaled, true, g.throttle_filt);
}

// helper function to update position controller's desired velocity while respecting acceleration limits
void Copter::guided_set_desired_velocity_with_accel_and_fence_limits(const Vector3f& vel_des)
{
    // get current desired velocity
    Vector3f curr_vel_des = pos_control->get_desired_velocity();

    // exit immediately if already equal
    if (curr_vel_des == vel_des) {
        return;
    }

    // get change in desired velocity
    Vector3f vel_delta = vel_des - curr_vel_des;

    // limit xy change
    float vel_delta_xy = safe_sqrt(sq(vel_delta.x)+sq(vel_delta.y));
    float vel_delta_xy_max = G_Dt * pos_control->get_accel_xy();
    float ratio_xy = 1.0f;
    if (!is_zero(vel_delta_xy) && (vel_delta_xy > vel_delta_xy_max)) {
        ratio_xy = vel_delta_xy_max / vel_delta_xy;
    }
    curr_vel_des.x += (vel_delta.x * ratio_xy);
    curr_vel_des.y += (vel_delta.y * ratio_xy);

    // limit z change
    float vel_delta_z_max = G_Dt * pos_control->get_accel_z();
    curr_vel_des.z += apm_constrain_float(vel_delta.z, -vel_delta_z_max, vel_delta_z_max);

    // update position controller with new target
    pos_control->set_desired_velocity(curr_vel_des);
}

// helper function to set yaw state and targets
void Copter::guided_set_yaw_state(bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_angle)
{
    if (use_yaw) {
        set_auto_yaw_look_at_heading(yaw_cd / 100.0f, 0.0f, 0, relative_angle);
    } else if (use_yaw_rate) {
        set_auto_yaw_rate(yaw_rate_cds);
    }
}

// Guided Limit code

// guided_limit_clear - clear/turn off guided limits
void Copter::guided_limit_clear()
{
    guided_limit.timeout_ms = 0;
    guided_limit.alt_min_cm = 0.0f;
    guided_limit.alt_max_cm = 0.0f;
    guided_limit.horiz_max_cm = 0.0f;
}

// guided_limit_set - set guided timeout and movement limits
void Copter::guided_limit_set(uint32_t timeout_ms, float alt_min_cm, float alt_max_cm, float horiz_max_cm)
{
    guided_limit.timeout_ms = timeout_ms;
    guided_limit.alt_min_cm = alt_min_cm;
    guided_limit.alt_max_cm = alt_max_cm;
    guided_limit.horiz_max_cm = horiz_max_cm;
}

// guided_limit_init_time_and_pos - initialise guided start time and position as reference for limit checking
//  only called from AUTO mode's auto_nav_guided_start function
void Copter::guided_limit_init_time_and_pos()
{
    // initialise start time
    guided_limit.start_time = millis();

    // initialise start position from current position
    guided_limit.start_pos = inertial_nav.get_position();
}

// guided_limit_check - returns true if guided mode has breached a limit
//  used when guided is invoked from the NAV_GUIDED_ENABLE mission command
bool Copter::guided_limit_check()
{
    // check if we have passed the timeout
    if ((guided_limit.timeout_ms > 0) && (millis() - guided_limit.start_time >= guided_limit.timeout_ms)) {
        return true;
    }

    // get current location
    const Vector3f& curr_pos = inertial_nav.get_position();

    // check if we have gone below min alt
    if (!is_zero(guided_limit.alt_min_cm) && (curr_pos.z < guided_limit.alt_min_cm)) {
        return true;
    }

    // check if we have gone above max alt
    if (!is_zero(guided_limit.alt_max_cm) && (curr_pos.z > guided_limit.alt_max_cm)) {
        return true;
    }

    // check if we have gone beyond horizontal limit
    if (guided_limit.horiz_max_cm > 0.0f) {
        float horiz_move = pv_get_horizontal_distance_cm(guided_limit.start_pos, curr_pos);
        if (horiz_move > guided_limit.horiz_max_cm) {
            return true;
        }
    }

    // if we got this far we must be within limits
    return false;
}

void Copter::guided_cal_posvel(const Vector3f base_pos, const float base_vel_xy, const float base_yaw_angle, const bool do_land)
{
    static uint32_t last_update_ms = 0;
    static uint8_t follow_yaw_state = 0;
    static uint8_t follow_hclbr_state = 0;
    const float speed_threshold = 50.f;
    bool use_yaw = false;
    float yaw_cd = 0.0f;
    bool use_yaw_rate = false;
    float yaw_rate_cds = 0.0f;
    bool relative_yaw = false;
    uint32_t now = millis();
    float base_dist_xy_length = Vector2f(base_pos.x - inertial_nav.get_position().x, base_pos.y - inertial_nav.get_position().y).length();
    float base_vel_xy_length = base_vel_xy;
    float bearing_to_base_cd = degrees( atan2f(base_pos.y - inertial_nav.get_position().y, base_pos.x - inertial_nav.get_position().x) ) * 100.f;
    bool heading_invalid = base_vel_xy_length < speed_threshold;

    Vector3f base_vel = Vector3f(base_vel_xy*cosf(base_yaw_angle*0.01745), base_vel_xy*sinf(base_yaw_angle*0.01745), 0.0f);

    if (now - last_update_ms > 3000) {
        follow_yaw_state = 0;
        follow_hclbr_state = 0;
    }
    last_update_ms = now;

    // state for heading
    switch (follow_yaw_state) {
        case 0:
            if (base_dist_xy_length < 500.f) {
                follow_yaw_state = 1;
            } else {
                if (heading_invalid) {
                    use_yaw = true;
                    yaw_cd = bearing_to_base_cd;
                    use_yaw_rate = false;
                    yaw_rate_cds = 0.0f;
                } else {
                    use_yaw = true;
                    yaw_cd = base_yaw_angle*100.f;
                    use_yaw_rate = false;
                    yaw_rate_cds = 0.0f;
                }
            }
            break;
        case 1:
            if (base_dist_xy_length > 800.f) {
                follow_yaw_state = 0;
            } else {
                if (heading_invalid) {
                    use_yaw = false;
                    yaw_cd = base_yaw_angle*100.f;
                    use_yaw_rate = true;
                    yaw_rate_cds = 0.0f;
                } else {
                    use_yaw = true;
                    yaw_cd = base_yaw_angle*100.f;
                    use_yaw_rate = false;
                    yaw_rate_cds = 0.0f;
                }
            }
            break;
        default:
            break;
    }

    //state for position
    const float L1_length = 1000.f;
    Matrix3f body_to_ned;
    body_to_ned.from_euler(0.0f, 0.0f, radians(base_yaw_angle));
    Matrix3f ned_to_body = body_to_ned.transposed();
    Vector3f dest_pos = base_pos;
    Vector3f dest_vel = base_vel;
    Vector3f dest_vel_ff = Vector3f(0.0f, 0.0f, 0.0f);
    Vector3f delta_pos = dest_pos - inertial_nav.get_position();
    Vector3f tmp_delta_pos = delta_pos;
    float delta_length = Vector2f(delta_pos.x, delta_pos.y).length();

    if (heading_invalid) {
        ;
    } else {
        tmp_delta_pos = body_to_ned * guided_L1_xy(ned_to_body * delta_pos, L1_length);
    }

    float v_ff_factor = 0.5f;
    float tmp_length = Vector2f(tmp_delta_pos.x, tmp_delta_pos.y).length();
    if (tmp_length > L1_length) {
        tmp_delta_pos.x *= L1_length/tmp_length;
        tmp_delta_pos.y *= L1_length/tmp_length;
    }

    dest_pos = inertial_nav.get_position() + tmp_delta_pos;


    if (heading_invalid) {
        float error = MAX(delta_length - 1.5f*L1_length, 0.0f);
        float speed_max = guided_max_speed(error, pos_control->get_pos_xy_kP(), pos_control->get_accel_xy());
        dest_vel_ff.x = apm_constrain_float(speed_max * tmp_delta_pos.x/L1_length, -pos_control->get_speed_xy(), pos_control->get_speed_xy());
        dest_vel_ff.y = apm_constrain_float(speed_max * tmp_delta_pos.y/L1_length, -pos_control->get_speed_xy(), pos_control->get_speed_xy());
    } else {
        tmp_delta_pos = (ned_to_body * tmp_delta_pos);

        float error_x = MAX(fabsf((ned_to_body * delta_pos).x) - 1.5f*L1_length, 0.0f);
        float speed_max_x = guided_max_speed(error_x, pos_control->get_pos_xy_kP(), pos_control->get_accel_xy());
        float error_y = MAX(fabsf((ned_to_body * delta_pos).y) - 1.5f*L1_length, 0.0f);
        float speed_max_y = guided_max_speed(error_y, pos_control->get_pos_xy_kP(), pos_control->get_accel_xy());

        dest_vel_ff.x = apm_constrain_float(speed_max_x * tmp_delta_pos.x/L1_length, -pos_control->get_speed_xy(), pos_control->get_speed_xy());
        dest_vel_ff.y = apm_constrain_float(speed_max_y * tmp_delta_pos.y/L1_length, -pos_control->get_speed_xy(), pos_control->get_speed_xy());
        
        dest_vel_ff = body_to_ned * dest_vel_ff;
    }

//    float error = MAX(delta_length - L1_length, 0.0f);
//    float speed_max = guided_max_speed(error, pos_control->get_pos_xy_kP(), pos_control->get_accel_xy());
//    dest_vel_ff.x = apm_constrain_float(speed_max * delta_pos.x/delta_length, -pos_control->get_speed_xy(), pos_control->get_speed_xy());
//    dest_vel_ff.y = apm_constrain_float(speed_max * delta_pos.y/delta_length, -pos_control->get_speed_xy(), pos_control->get_speed_xy());

    float dest_vel_ff_length = Vector2f(dest_vel_ff.x, dest_vel_ff.y).length();
    if (dest_vel_ff_length > pos_control->get_speed_xy()) {
        dest_vel_ff.x *= pos_control->get_speed_xy()/dest_vel_ff_length;
        dest_vel_ff.y *= pos_control->get_speed_xy()/dest_vel_ff_length;
    }
    dest_vel_ff = dest_vel_ff * v_ff_factor;
    //dest_vel_ff.y *= v_ff_factor;

    // state for height and decent rate
    dest_pos.z = 1000.0f;
    dest_vel.z = 0.0f;
    switch (follow_hclbr_state) {
        case 0:
            if (base_dist_xy_length < 300.f) {
                follow_hclbr_state = 1;
            } else {
                if (heading_invalid) {
                    dest_vel.x = 0.0f;
                    dest_vel.y = 0.0f;
                }
            }
            break;
        case 1:
            if (base_dist_xy_length > 600.f) {
                follow_hclbr_state = 0;
            } else {
                if (heading_invalid) {
                    dest_vel.x = 0.0f;
                    dest_vel.y = 0.0f;
                }

                if (do_land) {
                    dest_pos.z = -99.0f; //disable pos_target_z input
                    if (inertial_nav.get_position().z > 300.f) {
                        dest_vel.z = -100.f;
                    } else if (inertial_nav.get_position().z > 50.f) {
                        dest_vel.z = -50.f;
                    }
                }
            }
            break;
        default:
            break;
    }

    dest_vel += dest_vel_ff;

    guided_pos_target_cm = dest_pos;
    guided_vel_target_cms = dest_vel;

    // set yaw state
    guided_set_yaw_state(use_yaw, yaw_cd, use_yaw_rate, yaw_rate_cds, relative_yaw);
}

Vector3f Copter::guided_L1_xy(Vector3f pos_p, const float L1_length ) {
    bool xp_limit = (fabsf(pos_p.x) < L1_length);
    bool yp_limit = (fabsf(pos_p.y) < L1_length);
    //gcs_send_text_fmt(MAV_SEVERITY_INFO, "x %0.2f , y %0.2f", pos_p.x, pos_p.y);
    if (xp_limit && yp_limit) {
        //gcs_send_text(MAV_SEVERITY_INFO, "T0");
        ;
    } else if (xp_limit || pos_p.x < 0.0f) {
        //gcs_send_text(MAV_SEVERITY_INFO, "T1");
        ;
    } else if (!xp_limit && yp_limit) {
        pos_p.x = safe_sqrt(L1_length * L1_length - pos_p.y * pos_p.y);
        //gcs_send_text(MAV_SEVERITY_INFO, "T2");
    } else {
        pos_p.x = 0.0f;
        //gcs_send_text(MAV_SEVERITY_INFO, "T3");
    }

    return pos_p;
}

float Copter::guided_max_speed(float distance, float p, float accel_cmss) {
    return AC_AttitudeControl::sqrt_controller(distance, p, accel_cmss);
}