#pragma once

#include "AP_Common.h"

// Global parameter class.
//
class Parameters {
public:

    float        throttle_filt;
    int16_t        throttle_behavior;
    int16_t        takeoff_trigger_dz;
    float        pilot_takeoff_alt;

    int16_t        rtl_altitude;
    int16_t        rtl_speed_cms;
    float        rtl_cone_slope;
    int16_t        rtl_alt_final;
    int16_t        rtl_climb_min;              // rtl minimum climb in cm

    int8_t         wp_yaw_behavior;            // controls how the autopilot controls yaw during missions
    int8_t         rc_feel_rp;                 // controls vehicle response to user input with 0 being extremely soft and 100 begin extremely crisp

    // Waypoints
    //
    int32_t        rtl_loiter_time;
    int16_t        land_speed;
    int16_t        land_speed_high;
    int16_t        pilot_velocity_z_max;        // maximum vertical velocity the pilot may request
    int16_t        pilot_accel_z;               // vertical acceleration the pilot may request

    // Throttle
    //
    int8_t         failsafe_throttle;
    int16_t        failsafe_throttle_value;
    int16_t        throttle_deadzone;

    // Misc
    //
    int8_t         frame_type;
    int8_t         ch7_option;
    int8_t         ch8_option;
    int8_t         ch9_option;
    int8_t         ch10_option;
    int8_t         ch11_option;
    int8_t         ch12_option;
    int8_t         disarm_delay;

    int8_t         land_repositioning;

    int16_t                rc_speed; // speed of fast RC Channels in Hz

    // PI/D controllers
    AC_PI_2D                pi_vel_xy;

    AC_P                    p_vel_z;
    AC_PID                  pid_accel_z;

    AC_P                    p_pos_xy;
    AC_P                    p_alt_hold;

    // Autotune
    int8_t              autotune_axis_bitmask;
    float               autotune_aggressiveness;
    float               autotune_min_d;
    int8_t              fs_crash_check;
    float               acro_yaw_p;
    uint32_t            follow_land_time;
    float               follow_land_safe_height;
    float               follow_land_safe_throttle;
	uint32_t            offset_distance;
	float               first_stage_vel;
	float               final_stage_vel;
	float               altitude_offset;
	float               cross_offset;
	float               fore_offset;
	float               stall_high;    

    // Note: keep initializers here in the same order as they are declared
    // above.
    Parameters() :
        throttle_filt(0),
        throttle_behavior(0),
        takeoff_trigger_dz(THR_DZ_DEFAULT),
        pilot_takeoff_alt(PILOT_TKOFF_ALT_DEFAULT),
        rtl_altitude(RTL_ALT),
        rtl_speed_cms(0),
        rtl_cone_slope(RTL_CONE_SLOPE_DEFAULT),
        rtl_alt_final(RTL_ALT_FINAL),
        rtl_climb_min(RTL_CLIMB_MIN_DEFAULT),
        wp_yaw_behavior(WP_YAW_BEHAVIOR_DEFAULT),
        rc_feel_rp(RC_FEEL_RP_MEDIUM),
        rtl_loiter_time(RTL_LOITER_TIME),
        land_speed(LAND_SPEED),
        land_speed_high(0),
        pilot_velocity_z_max(PILOT_VELZ_MAX),
        pilot_accel_z(PILOT_ACCEL_Z_DEFAULT),
        failsafe_throttle(FS_THR_ENABLED_ALWAYS_RTL),
        failsafe_throttle_value(FS_THR_VALUE_DEFAULT),
        throttle_deadzone(THR_DZ_DEFAULT),
        frame_type(AP_Motors::MOTOR_FRAME_TYPE_X),
        ch7_option(AUXSW_DO_NOTHING),
        ch8_option(AUXSW_DO_NOTHING),
        ch9_option(AUXSW_DO_NOTHING),
        ch10_option(AUXSW_DO_NOTHING),
        ch11_option(AUXSW_DO_NOTHING),
        ch12_option(AUXSW_DO_NOTHING),
        disarm_delay(AUTO_DISARMING_DELAY),
        land_repositioning(LAND_REPOSITION_DEFAULT),
        rc_speed(RC_FAST_SPEED),
        // PID controller       initial P         initial I         initial D       initial imax        initial filt hz     pid rate
        //---------------------------------------------------------------------------------------------------------------------------------
        pi_vel_xy               (VEL_XY_P,        VEL_XY_I,                         VEL_XY_IMAX,        VEL_XY_FILT_HZ,     WPNAV_LOITER_UPDATE_TIME),

        p_vel_z                 (VEL_Z_P),
        pid_accel_z             (ACCEL_Z_P,       ACCEL_Z_I,        ACCEL_Z_D,      ACCEL_Z_IMAX,       ACCEL_Z_FILT_HZ,    MAIN_LOOP_SECONDS),

        // P controller         initial P
        //----------------------------------------------------------------------
        p_pos_xy                (POS_XY_P),

        p_alt_hold              (ALT_HOLD_P),

        fs_crash_check(0),
        acro_yaw_p(ACRO_YAW_P),
        follow_land_time(3000),
        follow_land_safe_height(500.f),
        follow_land_safe_throttle(0.2f),
		offset_distance(200.f),
		first_stage_vel(-100.f),
		final_stage_vel(-50.f),
		altitude_offset(0.0f),
		cross_offset(200.f),
		fore_offset(300.f),
		stall_high(50.f)
    {
        printf(" Init: Parameters\n");
    }
};

/*
  2nd block of parameters, to avoid going past 256 top level keys
 */
class ParametersG2 {
public:

    // RC input channels
    RC_Channels rc_channels;
    
    // control over servo output ranges
    SRV_Channels servo_channels;

    // altitude at which nav control can start in takeoff
    float wp_navalt_min;

    // ground effect compensation enable/disable
    int8_t gndeffect_comp_enabled;

    // frame class
    int8_t frame_class;

    float acro_y_expo;

    ParametersG2() : 
    wp_navalt_min(10),
    gndeffect_comp_enabled(0),
    frame_class(1),
    acro_y_expo(ACRO_Y_EXPO_DEFAULT)
    {
        printf(" Init: ParametersG2\n");
    }
};

