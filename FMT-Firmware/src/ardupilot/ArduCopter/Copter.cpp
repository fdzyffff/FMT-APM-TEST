#include "Copter.h"
/*
  constructor for main Copter class
 */
Copter::Copter(void) :
    control_mode(STABILIZE),
    scaleLongDown(1),
    wp_bearing(0),
    home_bearing(0),
    home_distance(0),
    wp_distance(0),
    guided_mode(Guided_TakeOff),
    rtl_state(RTL_InitialClimb),
    rtl_state_complete(false),
    circle_pilot_yaw_override(false),
    initial_armed_bearing(0),
    loiter_time_max(0),
    loiter_time(0),
    climb_rate(0),
    target_rangefinder_alt(0.0f),
    baro_alt(0),
    baro_climbrate(0.0f),
    land_accel_ef_filter(LAND_DETECTOR_ACCEL_LPF_CUTOFF),
    rc_throttle_control_in_filter(1.0f),
    auto_yaw_mode(AUTO_YAW_LOOK_AT_NEXT_WP),
    yaw_look_at_WP_bearing(0.0f),
    yaw_look_at_heading(0),
    yaw_look_at_heading_slew(0),
    yaw_look_ahead_bearing(0.0f),
    condition_value(0),
    condition_start(0),
    G_Dt(MAIN_LOOP_SECONDS),
    inertial_nav(ahrs),
    pmTest1(0),
    fast_loopTimer(0),
    mainLoop_count(0),
    rtl_loiter_start_time(0),
    auto_trim_counter(0),
    in_mavlink_delay(false),
    gcs_out_of_time(false)
{
    memset(&current_loc, 0, sizeof(current_loc));
    aparm.angle_max = DEFAULT_ANGLE_MAX;
    flight_modes[0] = FLIGHT_MODE_1;
    flight_modes[1] = FLIGHT_MODE_2;
    flight_modes[2] = FLIGHT_MODE_3;
    flight_modes[3] = FLIGHT_MODE_4;
    flight_modes[4] = FLIGHT_MODE_5;
    flight_modes[5] = FLIGHT_MODE_6;
    test_value_p1 = (100.f);
    printf(" Init: Copter\n");
}
