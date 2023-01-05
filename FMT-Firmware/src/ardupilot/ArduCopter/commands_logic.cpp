#include "Copter.h"

// start_command - this function will be called when the ap_mission lib wishes to start a new command
bool Copter::start_command(const AP_Mission::Mission_Command& cmd)
{
    if (control_mode != AUTO) {
        return false;
    }
    copter_current_cmd = cmd;
    // To-Do: logging when new commands start/end

    switch(cmd.id) {

    ///
    /// navigation commands
    ///
    case MAV_CMD_NAV_TAKEOFF:                   // 22
        do_takeoff(cmd);
        break;

    case MAV_CMD_NAV_WAYPOINT:                  // 16  Navigate to Waypoint
        do_nav_wp(cmd);
        break;

    case MAV_CMD_NAV_LAND:              // 21 LAND to Waypoint
        do_land(cmd);
        break;

    case MAV_CMD_NAV_PAYLOAD_PLACE:              // 94 place at Waypoint
        //do_payload_place(cmd);
        break;

    case MAV_CMD_NAV_LOITER_UNLIM:              // 17 Loiter indefinitely
        do_loiter_unlimited(cmd);
        break;

    case MAV_CMD_NAV_LOITER_TURNS:              //18 Loiter N Times
        do_circle(cmd);
        break;

    case MAV_CMD_NAV_LOITER_TIME:              // 19
        do_loiter_time(cmd);
        break;

    case MAV_CMD_NAV_RETURN_TO_LAUNCH:             //20
        do_RTL();
        break;

    case MAV_CMD_NAV_SPLINE_WAYPOINT:           // 82  Navigate to Waypoint using spline
        //do_spline_wp(cmd);
        break;

    case MAV_CMD_NAV_DELAY:                    // 94 Delay the next navigation command
        do_nav_delay(cmd);
        break;

    //
    // conditional commands
    //
    case MAV_CMD_CONDITION_DELAY:             // 112
        do_wait_delay(cmd);
        break;

    case MAV_CMD_CONDITION_DISTANCE:             // 114
        do_within_distance(cmd);
        break;

    case MAV_CMD_CONDITION_YAW:             // 115
        do_yaw(cmd);
        break;

    ///
    /// do commands
    ///
    case MAV_CMD_DO_CHANGE_SPEED:             // 178
        do_change_speed(cmd);
        break;

    case MAV_CMD_DO_SET_HOME:             // 179
        do_set_home(cmd);
        break;

    case MAV_CMD_DO_SET_SERVO:
        //ServoRelayEvents.do_set_servo(cmd.content.servo.channel, cmd.content.servo.pwm);
        break;
        
    case MAV_CMD_DO_SET_RELAY:
        //ServoRelayEvents.do_set_relay(cmd.content.relay.num, cmd.content.relay.state);
        break;
        
    case MAV_CMD_DO_REPEAT_SERVO:
        //ServoRelayEvents.do_repeat_servo(cmd.content.repeat_servo.channel, cmd.content.repeat_servo.pwm,
        //                                 cmd.content.repeat_servo.repeat_count, cmd.content.repeat_servo.cycle_time * 1000.0f);
        break;
        
    case MAV_CMD_DO_REPEAT_RELAY:
        //ServoRelayEvents.do_repeat_relay(cmd.content.repeat_relay.num, cmd.content.repeat_relay.repeat_count,
        //                                 cmd.content.repeat_relay.cycle_time * 1000.0f);
        break;

    case MAV_CMD_DO_SET_ROI:                // 201
        // point the copter and camera at a region of interest (ROI)
        do_roi(cmd);
        break;

    case MAV_CMD_DO_MOUNT_CONTROL:          // 205
        // point the camera to a specified angle
        //do_mount_control(cmd);
        break;

    default:
        // do nothing with unrecognized MAVLink messages
        break;
    }

    // always return success
    return true;
}

/********************************************************************************/
// Verify command Handlers
/********************************************************************************/

// verify_command_callback - callback function called from ap-mission at 10hz or higher when a command is being run
//      we double check that the flight mode is AUTO to avoid the possibility of ap-mission triggering actions while we're not in AUTO mode
bool Copter::verify_command_callback(const AP_Mission::Mission_Command& cmd)
{
    if (control_mode == AUTO || control_mode == TESTSTAR) {
        bool cmd_complete = verify_command(cmd);
        return cmd_complete;
    }
    return false;
}

/*******************************************************************************
Verify command Handlers

Each type of mission element has a "verify" operation. The verify
operation returns true when the mission element has completed and we
should move onto the next mission element.
Return true if we do not recognize the command so that we move on to the next command
*******************************************************************************/

bool Copter::verify_command(const AP_Mission::Mission_Command& cmd)
{
    switch(cmd.id) {
    //
    // navigation commands
    //
    case MAV_CMD_NAV_TAKEOFF:
        return verify_takeoff();

    case MAV_CMD_NAV_WAYPOINT:
        return verify_nav_wp(cmd);

    case MAV_CMD_NAV_LAND:
        return verify_land();

    case MAV_CMD_NAV_PAYLOAD_PLACE:
        //return verify_payload_place();

    case MAV_CMD_NAV_LOITER_UNLIM:
        return verify_loiter_unlimited();

    case MAV_CMD_NAV_LOITER_TURNS:
        return verify_circle(cmd);

    case MAV_CMD_NAV_LOITER_TIME:
        return verify_loiter_time();

    case MAV_CMD_NAV_RETURN_TO_LAUNCH:
        //return verify_RTL();

    case MAV_CMD_NAV_SPLINE_WAYPOINT:
        return verify_spline_wp(cmd);

     case MAV_CMD_NAV_DELAY:
        return verify_nav_delay(cmd);

    ///
    /// conditional commands
    ///
    case MAV_CMD_CONDITION_DELAY:
        return verify_wait_delay();

    case MAV_CMD_CONDITION_DISTANCE:
        return verify_within_distance();

    case MAV_CMD_CONDITION_YAW:
        return verify_yaw();

    // do commands (always return true)
    case MAV_CMD_DO_CHANGE_SPEED:
    case MAV_CMD_DO_SET_HOME:
    case MAV_CMD_DO_SET_SERVO:
    case MAV_CMD_DO_SET_RELAY:
    case MAV_CMD_DO_REPEAT_SERVO:
    case MAV_CMD_DO_REPEAT_RELAY:
    case MAV_CMD_DO_SET_ROI:
    case MAV_CMD_DO_MOUNT_CONTROL:
    case MAV_CMD_DO_CONTROL_VIDEO:
    case MAV_CMD_DO_DIGICAM_CONFIGURE:
    case MAV_CMD_DO_DIGICAM_CONTROL:
    case MAV_CMD_DO_SET_CAM_TRIGG_DIST:
    case MAV_CMD_DO_PARACHUTE:  // assume parachute was released successfully
    case MAV_CMD_DO_GRIPPER:
    case MAV_CMD_DO_GUIDED_LIMITS:
    case MAV_CMD_DO_FENCE_ENABLE:
        return true;

    default:
        // return true if we do not recognize the command so that we move on to the next command
        return true;
    }
}

/********************************************************************************/
//
/********************************************************************************/

// do_RTL - start Return-to-Launch
void Copter::do_RTL(void)
{
    // start rtl in auto flight mode
    auto_rtl_start();
}

/********************************************************************************/
//	Nav (Must) commands
/********************************************************************************/

// do_takeoff - initiate takeoff navigation command
void Copter::do_takeoff(const AP_Mission::Mission_Command& cmd)
{
    // Set wp navigation target to safe altitude above current position
    auto_takeoff_start(cmd.content.location);
}

// do_nav_wp - initiate move to next waypoint
void Copter::do_nav_wp(const AP_Mission::Mission_Command& cmd)
{
    Location_Class target_loc(cmd.content.location);
    // use current lat, lon if zero
    if (target_loc.lat == 0 && target_loc.lng == 0) {
        target_loc.lat = current_loc.lat;
        target_loc.lng = current_loc.lng;
    }
    // use current altitude if not provided
    if (target_loc.alt == 0) {
        // set to current altitude but in command's alt frame
        int32_t curr_alt;
        if (current_loc.get_alt_cm(target_loc.get_alt_frame(),curr_alt)) {
            target_loc.set_alt_cm(curr_alt, target_loc.get_alt_frame());
        } else {
            // default to current altitude as alt-above-home
            target_loc.set_alt_cm(current_loc.alt, current_loc.get_alt_frame());
        }
    }
    
    // this will be used to remember the time in millis after we reach or pass the WP.
    loiter_time = 0;
    // this is the delay, stored in seconds
    loiter_time_max = cmd.p1;

    // Set wp navigation target
    auto_wp_start(target_loc);

    // if no delay set the waypoint as "fast"
    if (loiter_time_max == 0 ) {
        wp_nav->set_fast_waypoint(true);
    }
}

// terrain_adjusted_location: returns a Location with lat/lon from cmd
// and altitude from our current altitude adjusted for location
Location_Class Copter::terrain_adjusted_location(const AP_Mission::Mission_Command& cmd) const
{
    // convert to location class
    Location_Class target_loc(cmd.content.location);

    // decide if we will use terrain following
    int32_t curr_terr_alt_cm, target_terr_alt_cm;
    if (current_loc.get_alt_cm(Location_Class::ALT_FRAME_ABOVE_TERRAIN, curr_terr_alt_cm) &&
        target_loc.get_alt_cm(Location_Class::ALT_FRAME_ABOVE_TERRAIN, target_terr_alt_cm)) {
        curr_terr_alt_cm = MAX(curr_terr_alt_cm,200);
        // if using terrain, set target altitude to current altitude above terrain
        target_loc.set_alt_cm(curr_terr_alt_cm, Location_Class::ALT_FRAME_ABOVE_TERRAIN);
    } else {
        // set target altitude to current altitude above home
        target_loc.set_alt_cm(current_loc.alt, Location_Class::ALT_FRAME_ABOVE_HOME);
    }
    return target_loc;
}

// do_land - initiate landing procedure
void Copter::do_land(const AP_Mission::Mission_Command& cmd)
{
    // To-Do: check if we have already landed

    // if location provided we fly to that location at current altitude
    if (cmd.content.location.lat != 0 || cmd.content.location.lng != 0) {
        // set state to fly to location
        land_state = LandStateType_FlyToLocation;

        Location_Class target_loc = terrain_adjusted_location(cmd);

        auto_wp_start(target_loc);
    }else{
        // set landing state
        land_state = LandStateType_Descending;

        // initialise landing controller
        auto_land_start();
    }
}

// do_loiter_unlimited - start loitering with no end conditions
// note: caller should set yaw_mode
void Copter::do_loiter_unlimited(const AP_Mission::Mission_Command& cmd)
{
    // convert back to location
    Location_Class target_loc(cmd.content.location);

    // use current location if not provided
    if (target_loc.lat == 0 && target_loc.lng == 0) {
        // To-Do: make this simpler
        Vector3f temp_pos;
        wp_nav->get_wp_stopping_point_xy(temp_pos);
        Location_Class temp_loc(temp_pos);
        target_loc.lat = temp_loc.lat;
        target_loc.lng = temp_loc.lng;
    }

    // use current altitude if not provided
    // To-Do: use z-axis stopping point instead of current alt
    if (target_loc.alt == 0) {
        // set to current altitude but in command's alt frame
        int32_t curr_alt;
        if (current_loc.get_alt_cm(target_loc.get_alt_frame(),curr_alt)) {
            target_loc.set_alt_cm(curr_alt, target_loc.get_alt_frame());
        } else {
            // default to current altitude as alt-above-home
            target_loc.set_alt_cm(current_loc.alt, current_loc.get_alt_frame());
        }
    }

    // start way point navigator and provide it the desired location
    auto_wp_start(target_loc);
}

// do_circle - initiate moving in a circle
void Copter::do_circle(const AP_Mission::Mission_Command& cmd)
{
    Location_Class circle_center(cmd.content.location);

    // default lat/lon to current position if not provided
    // To-Do: use stopping point or position_controller's target instead of current location to avoid jerk?
    if (circle_center.lat == 0 && circle_center.lng == 0) {
        circle_center.lat = current_loc.lat;
        circle_center.lng = current_loc.lng;
    }

    // default target altitude to current altitude if not provided
    if (circle_center.alt == 0) {
        int32_t curr_alt;
        if (current_loc.get_alt_cm(circle_center.get_alt_frame(),curr_alt)) {
            // circle altitude uses frame from command
            circle_center.set_alt_cm(curr_alt,circle_center.get_alt_frame());
        } else {
            // default to current altitude above origin
            circle_center.set_alt_cm(current_loc.alt, current_loc.get_alt_frame());
        }
    }

    // calculate radius
    uint8_t circle_radius_m = HIGHBYTE(cmd.p1); // circle radius held in high byte of p1

    // move to edge of circle (verify_circle) will ensure we begin circling once we reach the edge
    auto_circle_movetoedge_start(circle_center, circle_radius_m);
}

// do_loiter_time - initiate loitering at a point for a given time period
// note: caller should set yaw_mode
void Copter::do_loiter_time(const AP_Mission::Mission_Command& cmd)
{
    // re-use loiter unlimited
    do_loiter_unlimited(cmd);

    // setup loiter timer
    loiter_time     = 0;
    loiter_time_max = cmd.p1;     // units are (seconds)
}


// do_nav_delay - Delay the next navigation command
void Copter::do_nav_delay(const AP_Mission::Mission_Command& cmd)
{
    nav_delay_time_start = millis();

    if (cmd.content.nav_delay.seconds > 0) {
        // relative delay
        nav_delay_time_max = cmd.content.nav_delay.seconds * 1000; // convert seconds to milliseconds
    } else {
        // absolute delay to utc time
        nav_delay_time_max = 0;
    }
}


/********************************************************************************/
//	Verify Nav (Must) commands
/********************************************************************************/

// verify_takeoff - check if we have completed the takeoff
bool Copter::verify_takeoff()
{
    // have we reached our target altitude?
    return wp_nav->reached_wp_destination();
}

// verify_land - returns true if landing has been completed
bool Copter::verify_land()
{
    bool retval = false;

    switch (land_state) {
        case LandStateType_FlyToLocation:
            // check if we've reached the location
            if (wp_nav->reached_wp_destination()) {
                // get destination so we can use it for loiter target
                Vector3f dest = wp_nav->get_wp_destination();

                // initialise landing controller
                auto_land_start(dest);

                // advance to next state
                land_state = LandStateType_Descending;
            }
            break;

        case LandStateType_Descending:
            // rely on THROTTLE_LAND mode to correctly update landing status
            retval = ap.land_complete;
            break;

        default:
            // this should never happen
            // TO-DO: log an error
            retval = true;
            break;
    }

    // true is returned if we've successfully landed
    return retval;
}

// verify_nav_wp - check if we have reached the next way point
bool Copter::verify_nav_wp(const AP_Mission::Mission_Command& cmd)
{
    // check if we have reached the waypoint
    if( !wp_nav->reached_wp_destination() ) {
        return false;
    }

    // play a tone
    AP_Notify::events.waypoint_complete = 1;

    // start timer if necessary
    if(loiter_time == 0) {
        loiter_time = millis();
    }

    // check if timer has run out
    if (((millis() - loiter_time) / 1000) >= loiter_time_max) {
        return true;
    }else{
        return false;
    }
}

bool Copter::verify_loiter_unlimited()
{
    return false;
}

// verify_loiter_time - check if we have loitered long enough
bool Copter::verify_loiter_time()
{
    // return immediately if we haven't reached our destination
    if (!wp_nav->reached_wp_destination()) {
        return false;
    }

    // start our loiter timer
    if( loiter_time == 0 ) {
        loiter_time = millis();
    }

    // check if loiter timer has run out
    return (((millis() - loiter_time) / 1000) >= loiter_time_max);
}

// verify_circle - check if we have circled the point enough
bool Copter::verify_circle(const AP_Mission::Mission_Command& cmd)
{
    // check if we've reached the edge
    if (auto_mode == Auto_CircleMoveToEdge) {
        if (wp_nav->reached_wp_destination()) {
            Vector3f curr_pos = inertial_nav.get_position();
            Vector3f circle_center = pv_location_to_vector(cmd.content.location);

            // set target altitude if not provided
            if (is_zero(circle_center.z)) {
                circle_center.z = curr_pos.z;
            }

            // set lat/lon position if not provided
            if (cmd.content.location.lat == 0 && cmd.content.location.lng == 0) {
                circle_center.x = curr_pos.x;
                circle_center.y = curr_pos.y;
            }

            // start circling
            auto_circle_start();
        }
        return false;
    }

    // check if we have completed circling
    return fabsf(circle_nav->get_angle_total()/M_2PI) >= LOWBYTE(cmd.p1);
}

// verify_RTL - handles any state changes required to implement RTL
// do_RTL should have been called once first to initialise all variables
// returns true with RTL has completed successfully
bool Copter::verify_RTL()
{
    return (rtl_state_complete && (rtl_state == RTL_FinalDescent || rtl_state == RTL_Land));
}

// verify_spline_wp - check if we have reached the next way point using spline
bool Copter::verify_spline_wp(const AP_Mission::Mission_Command& cmd)
{
    // check if we have reached the waypoint
    if( !wp_nav->reached_wp_destination() ) {
        return false;
    }

    // start timer if necessary
    if(loiter_time == 0) {
        loiter_time = millis();
    }

    // check if timer has run out
    if (((millis() - loiter_time) / 1000) >= loiter_time_max) {
        return true;
    }else{
        return false;
    }
}


// verify_nav_delay - check if we have waited long enough
bool Copter::verify_nav_delay(const AP_Mission::Mission_Command& cmd)
{
    if (millis() - nav_delay_time_start > (uint32_t)MAX(nav_delay_time_max,0)) {
        nav_delay_time_max = 0;
        return true;
    }
    return false;
}


/********************************************************************************/
//	Condition (May) commands
/********************************************************************************/

void Copter::do_wait_delay(const AP_Mission::Mission_Command& cmd)
{
    condition_start = millis();
    condition_value = cmd.content.delay.seconds * 1000;     // convert seconds to milliseconds
}

void Copter::do_within_distance(const AP_Mission::Mission_Command& cmd)
{
    condition_value  = cmd.content.distance.meters * 100;
}

void Copter::do_yaw(const AP_Mission::Mission_Command& cmd)
{
	set_auto_yaw_look_at_heading(
		cmd.content.yaw.angle_deg,
		cmd.content.yaw.turn_rate_dps,
		cmd.content.yaw.direction,
		cmd.content.yaw.relative_angle > 0);
}


/********************************************************************************/
// Verify Condition (May) commands
/********************************************************************************/

bool Copter::verify_wait_delay()
{
    if (millis() - condition_start > (uint32_t)MAX(condition_value,0)) {
        condition_value = 0;
        return true;
    }
    return false;
}

bool Copter::verify_within_distance()
{
    // update distance calculation
    calc_wp_distance();
    if (wp_distance < (uint32_t)MAX(condition_value,0)) {
        condition_value = 0;
        return true;
    }
    return false;
}

// verify_yaw - return true if we have reached the desired heading
bool Copter::verify_yaw()
{
    // set yaw mode if it has been changed (the waypoint controller often retakes control of yaw as it executes a new waypoint command)
    if (auto_yaw_mode != AUTO_YAW_LOOK_AT_HEADING) {
        set_auto_yaw_mode(AUTO_YAW_LOOK_AT_HEADING);
    }

    // check if we are within 2 degrees of the target heading
    float tmp_delta_heading = (float)(ahrs.yaw_sensor-yaw_look_at_heading);
    if (fabsf(wrap_180_cd(tmp_delta_heading)) <= 200.f) {
        return true;
    }else{
        return false;
    }
}

/********************************************************************************/
//	Do (Now) commands
/********************************************************************************/


void Copter::do_change_speed(const AP_Mission::Mission_Command& cmd)
{
    if (cmd.content.speed.target_ms > 0) {
        wp_nav->set_speed_xy(cmd.content.speed.target_ms * 100.0f);
    }
}

void Copter::do_set_home(const AP_Mission::Mission_Command& cmd)
{
    if(cmd.p1 == 1 || (cmd.content.location.lat == 0 && cmd.content.location.lng == 0 && cmd.content.location.alt == 0)) {
        set_home_to_current_location();
    } else {
        set_home(cmd.content.location);
    }
}

// do_roi - starts actions required by MAV_CMD_DO_SET_ROI
//          this involves either moving the camera to point at the ROI (region of interest)
//          and possibly rotating the copter to point at the ROI if our mount type does not support a yaw feature
//	TO-DO: add support for other features of MAV_CMD_DO_SET_ROI including pointing at a given waypoint
void Copter::do_roi(const AP_Mission::Mission_Command& cmd)
{
    set_auto_yaw_roi(cmd.content.location);
}

// point the camera to a specified angle
void Copter::do_mount_control(const AP_Mission::Mission_Command& cmd)
{
;
}
