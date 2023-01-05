#pragma once

/*
  This is the main Copter class
 */

////////////////////////////////////////////////////////////////////////////////
// Header includes
////////////////////////////////////////////////////////////////////////////////
// #include "board.h"
//#include <cmath>
#include "stdio.h"
#include "stdarg.h"
#include "stdint.h"
// #include <firmament.h>

#include "ap_hal.h"
// Common dependencies
#include "AP_Common.h"
#include "Location.h"
#include "AP_Motors.h"
#include "AP_Notify.h"
#include "AC_PID.h"             // PID library
#include "AC_PI_2D.h"           // PID library (2-axis)
#include "AC_P.h"               // P library
#include "Filter.h"             // Filter library
#include "AC_WPNav.h"           // ArduCopter waypoint navigation library
#include "AC_Circle.h"          // circle navigation library
#include "RC_Channel.h"         // RC Channel Library
#include "AP_AHRS_NavEKF.h"
// #include "AP_Math.h"
#include "AP_InertialNav_NavEKF.h"
// Configuration
#include "defines.h"
#include "config.h"
#include "Parameters.h"

#include "AC_AttitudeControl_Multi.h" // Attitude control library
#include "AC_PosControl.h"      // Position control library

#include "AP_Mission.h"
#include "ardupilotmega.h"
#include "AP_Scheduler.h"       // main loop scheduler

// libraries which are dependent on #defines in defines.h and/or config.h

class Copter_pre {
public:
    virtual void setup();
    virtual void loop();
    virtual void rc_loop();
};

class Copter : public Copter_pre {
public:

    Copter(void);

    //parameters 
    void setup() override;
    void loop() override;
    void fast_loop();
    void rc_loop();
    void throttle_loop();
    void ten_hz_loop();
    void one_hz_loop();
    void arm_motors_check();
    void auto_disarm_check();
    void read_AHRS(void);
    void update_home_from_EKF();
    void read_inertia();

    bool set_mode(control_mode_t mode, mode_reason_t reason);
    void update_flight_mode();
    void exit_mode(control_mode_t old_control_mode, control_mode_t new_control_mode);
    bool mode_requires_GPS(control_mode_t mode);
    bool mode_has_manual_throttle(control_mode_t mode);
    bool mode_allows_arming(control_mode_t mode, bool arming_from_gcs);

    void update_land_and_crash_detectors();
    void update_land_detector();
    void update_throttle_thr_mix();
    void update_using_interlock();
    void default_dead_zones();
    void init_rc_in();
    void init_rc_out();
    void allocate_motors(void);
    void enable_motor_output();
    void startup_INS_ground();
    void set_land_complete(bool b);
    void set_land_complete_maybe(bool b);
    bool position_ok();
    void set_auto_armed(bool b);
    void failsafe_terrain_set_status(bool data_ok);

    bool stabilize_init(bool ignore_checks);
    void stabilize_run();

    void read_radio();
    void read_control_switch();
    void set_throttle_and_failsafe(uint16_t throttle_pwm);
    void set_throttle_zero_flag(int16_t throttle_control);
    void radio_passthrough_to_motors();

    void set_failsafe_radio(bool b);
    void failsafe_radio_on_event();
    void failsafe_terrain_on_event();

    bool init_arm_motors(bool arming_from_gcs);
    void init_disarm_motors();
    void motors_output();
    void update_auto_armed();
    bool set_home_to_current_location();
    void set_home_to_current_location_inflight();
    bool set_home_to_current_location_and_lock();
    bool set_home(const Location& loc);
    void set_home_state(enum HomeState new_home_state);
    void set_motor_emergency_stop(bool b);
    bool far_from_EKF_origin(const Location& loc);
    void set_system_time_from_GPS();

    bool should_disarm_on_failsafe();
    void init_ardupilot();
    bool home_is_set();
    void crash_check();
    float get_non_takeoff_throttle();
    float get_pilot_desired_throttle(int16_t throttle_control, float thr_mid = 0.0f);
    float get_pilot_desired_climb_rate(float throttle_control);
    float get_smoothing_gain();
    void get_pilot_desired_lean_angles(float roll_in, float pitch_in, float &roll_out, float &pitch_out, float angle_max);
    float get_pilot_desired_yaw_rate(int16_t stick_angle);
    Vector3f pv_location_to_vector(const Location& loc);
    float pv_alt_above_origin(float alt_above_home_cm);
    float pv_alt_above_home(float alt_above_origin_cm);
    float pv_get_bearing_cd(const Vector3f &origin, const Vector3f &destination);
    float pv_get_horizontal_distance_cm(const Vector3f &origin, const Vector3f &destination);
    float pv_distance_to_home_cm(const Vector3f &destination);
    bool check_if_auxsw_mode_used(uint8_t auxsw_mode_check);
    bool check_duplicate_auxsw(void);
    void reset_control_switch();
    uint8_t read_3pos_switch(uint8_t chan);
    void read_aux_switches();
    void init_aux_switches();
    void init_aux_switch_function(int8_t ch_option, uint8_t ch_flag);
    void do_aux_switch_function(int8_t ch_function, uint8_t ch_flag);
    void notify_flight_mode(control_mode_t mode);
    void set_accel_throttle_I_from_pilot_throttle();
    void read_aux_switch(uint8_t chan, uint8_t flag, int8_t option);

    bool current_mode_has_user_takeoff(bool must_navigate);
    bool do_user_takeoff(float takeoff_alt_cm, bool must_navigate);
    void takeoff_timer_start(float alt_cm);
    void takeoff_stop();
    void takeoff_get_climb_rates(float& pilot_climb_rate, float& takeoff_climb_rate);
    void auto_takeoff_set_start_alt(void);
    void auto_takeoff_attitude_run(float target_yaw_rate);
    float get_auto_heading(void);
    float get_auto_yaw_rate_cds();
    uint8_t get_default_auto_yaw_mode(bool rtl);
    void set_auto_yaw_mode(uint8_t yaw_mode);
    void set_auto_yaw_look_at_heading(float angle_deg, float turn_rate_dps, int8_t direction, bool relative_angle);
    void set_auto_yaw_roi(const Location &roi_location);
    void set_auto_yaw_rate(float turn_rate_cds);
    bool auto_init(bool ignore_checks);
    void auto_run();
    void auto_takeoff_start(const Location& dest_loc);
    void auto_takeoff_run();
    void auto_wp_start(const Vector3f& destination);
    void auto_wp_start(const Location_Class& dest_loc);
    void auto_wp_run();
    void auto_spline_run();
    void auto_land_start();
    void auto_land_start(const Vector3f& destination);
    void auto_land_run();
    void auto_payload_place_start();
    void auto_payload_place_start(const Vector3f& destination);
    void auto_payload_place_run();
    bool auto_payload_place_run_should_run();
    void auto_payload_place_run_loiter();
    void auto_payload_place_run_descend();
    void auto_payload_place_run_release();
    void auto_rtl_start();
    void auto_rtl_run();
    void auto_circle_movetoedge_start(const Location_Class &circle_center, float radius_m);
    void auto_circle_start();
    void auto_circle_run();
    void auto_nav_guided_start();
    void auto_nav_guided_run();
    bool auto_loiter_start();
    void auto_loiter_run();
    void auto_spline_start(const Location_Class& destination, bool stopped_at_start, AC_WPNav::spline_segment_end_type seg_end_type, const Location_Class& next_destination);

    bool start_command(const AP_Mission::Mission_Command& cmd);
    bool verify_command(const AP_Mission::Mission_Command& cmd);
    bool verify_command_callback(const AP_Mission::Mission_Command& cmd);
    Location_Class terrain_adjusted_location(const AP_Mission::Mission_Command& cmd) const;

    bool do_guided(const AP_Mission::Mission_Command& cmd);
    void do_takeoff(const AP_Mission::Mission_Command& cmd);
    void do_nav_wp(const AP_Mission::Mission_Command& cmd);
    void do_land(const AP_Mission::Mission_Command& cmd);
    void do_loiter_unlimited(const AP_Mission::Mission_Command& cmd);
    void do_circle(const AP_Mission::Mission_Command& cmd);
    void do_loiter_time(const AP_Mission::Mission_Command& cmd);
    void do_spline_wp(const AP_Mission::Mission_Command& cmd);
#if NAV_GUIDED == ENABLED
    void do_nav_guided_enable(const AP_Mission::Mission_Command& cmd);
    void do_guided_limits(const AP_Mission::Mission_Command& cmd);
#endif
    void do_nav_delay(const AP_Mission::Mission_Command& cmd);
    void do_wait_delay(const AP_Mission::Mission_Command& cmd);
    void do_within_distance(const AP_Mission::Mission_Command& cmd);
    void do_yaw(const AP_Mission::Mission_Command& cmd);
    void do_change_speed(const AP_Mission::Mission_Command& cmd);
    void do_set_home(const AP_Mission::Mission_Command& cmd);
    void do_roi(const AP_Mission::Mission_Command& cmd);
    void do_mount_control(const AP_Mission::Mission_Command& cmd);
    void do_RTL(void);
    bool verify_nav_wp(const AP_Mission::Mission_Command& cmd);
    bool verify_circle(const AP_Mission::Mission_Command& cmd);
    bool verify_spline_wp(const AP_Mission::Mission_Command& cmd);
#if NAV_GUIDED == ENABLED
    bool verify_nav_guided_enable(const AP_Mission::Mission_Command& cmd);
#endif
    bool verify_nav_delay(const AP_Mission::Mission_Command& cmd);
    bool verify_takeoff();
    bool verify_land();
    bool verify_payload_place();
    bool verify_loiter_unlimited();
    bool verify_loiter_time();
    bool verify_RTL();
    bool verify_wait_delay();
    bool verify_within_distance();
    bool verify_yaw();

    void run_nav_updates(void);
    void calc_distance_and_bearing();
    void calc_wp_distance();
    void calc_wp_bearing();
    void calc_home_distance_and_bearing();

    bool ekf_position_ok();
    float get_roi_yaw();
    float get_look_ahead_yaw();
    void update_throttle_hover();
    void set_throttle_takeoff();
    float get_surface_tracking_climb_rate(int16_t target_rate, float current_alt_target, float dt);
    float get_avoidance_adjusted_climbrate(float target_rate);
    void rotate_body_frame_to_NE(float &x, float &y);

    bool althold_init(bool ignore_checks);
    void althold_run();

    bool loiter_init(bool ignore_checks);
    void loiter_run();

    bool circle_init(bool ignore_checks);
    void circle_run();

    bool guided_init(bool ignore_checks);
    bool guided_takeoff_start(float final_alt_above_home);
    void guided_pos_control_start();
    void guided_vel_control_start();
    void guided_posvel_control_start();
    void guided_angle_control_start();
    void guided_angle_stab_start();
    bool guided_set_destination(const Vector3f& destination, bool use_vel_z, float target_z, bool use_yaw = false, float yaw_cd = 0.0, bool use_yaw_rate = false, float yaw_rate_cds = 0.0, bool yaw_relative = false);
    bool guided_set_destination(const Location_Class& dest_loc, bool use_yaw = false, float yaw_cd = 0.0, bool use_yaw_rate = false, float yaw_rate_cds = 0.0, bool yaw_relative = false);
    void guided_set_velocity(const Vector3f& velocity, bool use_yaw = false, float yaw_cd = 0.0, bool use_yaw_rate = false, float yaw_rate_cds = 0.0, bool yaw_relative = false);
    void guided_set_destination_posvel(const Vector3f& destination, const Vector3f& velocity, bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_yaw);
    void guided_set_angle(const float roll, const float pitch, const float yaw, float climb_rate_cms, bool use_yaw_rate, float yaw_rate_rads);
    void guided_set_angle_stab(const float roll, const float pitch, const float yaw, float thr_in, bool use_yaw_rate, float yaw_rate_rads);
    void guided_run();
    void guided_takeoff_run();
    void guided_pos_control_run();
    void guided_vel_control_run();
    void guided_posvel_control_run();
    void guided_angle_control_run();
    void guided_angle_stab_run();
    void guided_set_desired_velocity_with_accel_and_fence_limits(const Vector3f& vel_des);
    void guided_limit_clear();
    void guided_limit_set(uint32_t timeout_ms, float alt_min_cm, float alt_max_cm, float horiz_max_cm);
    void guided_limit_init_time_and_pos();
    bool guided_limit_check();
    void guided_set_yaw_state(bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_angle);
    Vector3f guided_align(Vector3f& rel_posneu_vector);
    bool guided_nogps_init(bool ignore_checks);
    void guided_nogps_run();
    void guided_cal_posvel(const Vector3f base_pos, const float base_vel_xy, const float base_yaw_angle, const bool do_land);
    Vector3f guided_L1_xy(Vector3f pos_p, const float L1_length );
    float guided_max_speed(float distance, float p, float accel_cmss);
    LowPassFilterFloat guided_yaw_base_angle_filt;

    bool rtl_init(bool ignore_checks);
    void rtl_restart_without_terrain();
    void rtl_run();
    void rtl_climb_start();
    void rtl_return_start();
    void rtl_climb_return_run();
    void rtl_loiterathome_start();
    void rtl_loiterathome_run();
    void rtl_descent_start();
    void rtl_descent_run();
    void rtl_land_start();
    void rtl_land_run();
    void rtl_build_path(bool terrain_following_allowed);
    void rtl_compute_return_target(bool terrain_following_allowed);

    bool land_init(bool ignore_checks);
    void land_run();
    void land_gps_run();
    void land_nogps_run();
    int32_t land_get_alt_above_ground(void);
    void land_run_vertical_control(bool pause_descent = false);
    void land_run_horizontal_control();
    void land_do_not_use_GPS();
    void set_mode_land_with_pause(mode_reason_t reason);
    bool landing_with_GPS();

    bool rangefinder_alt_ok();

    void navigation_init();
    void navigation_update();
    void navigation_next();

    void update_gcs_cmd();
    void update_fmt_bus();


    bool fp_init(bool ignore_checks);
    void fp_run();
    void fp_posvel_control_start();
    void fp_set_destination_posvel(const Vector3f& destination, const float velocity_xy, const float base_yaw_angle, const bool do_land);
    void fp_cal_posvel(const Vector3f base_pos, const float base_vel_xy, const float base_yaw_angle, const bool do_land);
    Vector3f fp_L1_xy(Vector3f pos_p, const float L1_length );
    float fp_max_speed(float distance, float p, float accel_cmss);

    bool fv_init(bool ignore_checks);
    void fv_run();
    void fv_vel_control_start();
    void fv_set_destination_vel(const Vector3f& destination, const float velocity_xy, const float base_yaw_angle, const bool do_land);
    void fv_set_desired_velocity_with_accel_and_fence_limits(const Vector3f& vel_des);
    void fv_cal_posvel(const Vector3f base_pos, const float base_vel_xy, const float base_yaw_angle, const bool do_land);
    float fv_max_speed(float distance, float kp, float accel_cmss, float offset=1.0f);

    int8_t follow_yaw_state;
    Vector3f follow_pos_base_cm;       // position target (used by posvel controller only)
    LowPassFilterFloat follow_yaw_base_angle_filt;		
		float target_cross_offset;
    float target_fore_offset;
    float target_high_offset;
		bool IsStall;
    
    Vector3f get_destination();
    // Global parameters are all contained within the 'g' class.
    Parameters g;
    ParametersG2 g2;
    // main loop scheduler
    AP_Scheduler scheduler;

    int8_t flight_modes[6];

    // This is the state of the flight control system
    // There are multiple states defined such as STABILIZE, ACRO,
    control_mode_t control_mode;
    mode_reason_t control_mode_reason = MODE_REASON_UNKNOWN;

    control_mode_t prev_control_mode;
    mode_reason_t prev_control_mode_reason = MODE_REASON_UNKNOWN;

    // Attitude, Position and Waypoint navigation objects
    // To-Do: move inertial nav up or other navigation variables down here
    AC_AttitudeControl *attitude_control;
    AC_PosControl *pos_control;
    AC_WPNav *wp_nav;
    AC_Circle *circle_nav;
    // Motor Output
    AP_MotorsMulticopter *motors;
    
    // Inertial Navigation EKF
    AP_AHRS_NavEKF ahrs;

    int32_t last_pilot_heading;
    uint32_t last_pilot_yaw_input_ms;

    float my_target_roll;
    float my_target_pitch;
    float my_target_yaw_rate;
    int8_t my_yaw_ctrl_state;

    // key aircraft parameters passed to multiple libraries
    AP_Vehicle::MultiCopter aparm;

    AP_Mission::Mission_Command copter_current_cmd;
    // primary input control channels
    RC_Channel *channel_roll;
    RC_Channel *channel_pitch;
    RC_Channel *channel_throttle;
    RC_Channel *channel_yaw;

    struct {
        bool enabled:1;
        bool alt_healthy:1; // true if we can trust the altitude from the rangefinder
        int16_t alt_cm;     // tilt compensated altitude (in cm) from rangefinder
        uint32_t last_healthy_ms;
        LowPassFilterFloat alt_cm_filt; // altitude filter
        int8_t glitch_count;
    } rangefinder_state = { false, false, 0, 0 };

    struct {
        int16_t id;
        Vector3f start_pos;
        float radius_cm;
        float start_yaw_angle;
        bool do_next;
    } TestStar;

    // gnd speed limit required to observe optical flow sensor limits
    float ekfGndSpdLimit;

    // scale factor applied to velocity controller gain to prevent optical flow noise causing excessive angle demand noise
    float ekfNavVelGainScaler;

    // system time in milliseconds of last recorded yaw reset from ekf
    uint32_t ekfYawReset_ms = 0;
    int8_t ekf_primary_core;

    // Documentation of GLobals:
    union {
        struct {
            uint8_t unused1                 : 1; // 0
            uint8_t simple_mode             : 2; // 1,2     // This is the state of simple mode : 0 = disabled ; 1 = SIMPLE ; 2 = SUPERSIMPLE
            uint8_t pre_arm_rc_check        : 1; // 3       // true if rc input pre-arm checks have been completed successfully
            uint8_t pre_arm_check           : 1; // 4       // true if all pre-arm checks (rc, accel calibration, gps lock) have been performed
            uint8_t auto_armed              : 1; // 5       // stops auto missions from beginning until throttle is raised
            uint8_t logging_started         : 1; // 6       // true if dataflash logging has started
            uint8_t land_complete           : 1; // 7       // true if we have detected a landing
            uint8_t new_radio_frame         : 1; // 8       // Set true if we have new PWM data to act on from the Radio
            uint8_t usb_connected           : 1; // 9       // true if APM is powered from USB connection
            uint8_t rc_receiver_present     : 1; // 10      // true if we have an rc receiver present (i.e. if we've ever received an update
            uint8_t compass_mot             : 1; // 11      // true if we are currently performing compassmot calibration
            uint8_t motor_test              : 1; // 12      // true if we are currently performing the motors test
            uint8_t initialised             : 1; // 13      // true once the init_ardupilot function has completed.  Extended status to GCS is not sent until this completes
            uint8_t land_complete_maybe     : 1; // 14      // true if we may have landed (less strict version of land_complete)
            uint8_t throttle_zero           : 1; // 15      // true if the throttle stick is at zero, debounced, determines if pilot intends shut-down when not using motor interlock
            uint8_t system_time_set         : 1; // 16      // true if the system time has been set from the GPS
            uint8_t gps_glitching           : 1; // 17      // true if the gps is glitching
            enum HomeState home_state       : 2; // 18,19   // home status (unset, set, locked)
            uint8_t using_interlock         : 1; // 20      // aux switch motor interlock function is in use
            uint8_t motor_emergency_stop    : 1; // 21      // motor estop switch, shuts off motors when enabled
            uint8_t land_repo_active        : 1; // 22      // true if the pilot is overriding the landing position
            uint8_t motor_interlock_switch  : 1; // 23      // true if pilot is requesting motor interlock enable
            uint8_t in_arming_delay         : 1; // 24      // true while we are armed but waiting to spin motors
            uint8_t initialised_params      : 1; // 25      // true when the all parameters have been initialised. we cannot send parameters to the GCS until this is done
        };
        uint32_t value;
    } ap;

    // Structure used to detect changes in the flight mode control switch
    struct {
        int8_t debounced_switch_position;   // currently used switch position
        int8_t last_switch_position;        // switch position in previous iteration
        uint32_t last_edge_time_ms;         // system time that switch position was last changed
    } control_switch_state;

    struct {
        bool running;
        float max_speed;
        float alt_delta;
        uint32_t start_ms;
    } takeoff_state;

    // altitude below which we do no navigation in auto takeoff
    float auto_takeoff_no_nav_alt_cm;
    
    RCMapper rcmap;

    // receiver RSSI
    uint8_t receiver_rssi;

    // Failsafe
    struct {
        uint8_t rc_override_active  : 1; // 0   // true if rc control are overwritten by ground station
        uint8_t radio               : 1; // 1   // A status flag for the radio failsafe
        uint8_t battery             : 1; // 2   // A status flag for the battery failsafe
        uint8_t gcs                 : 1; // 4   // A status flag for the ground station failsafe
        uint8_t ekf                 : 1; // 5   // true if ekf failsafe has occurred
        uint8_t terrain             : 1; // 6   // true if the missing terrain data failsafe has occurred
        uint8_t adsb                : 1; // 7   // true if an adsb related failsafe has occurred

        int8_t radio_counter;            // number of iterations with throttle below throttle_fs_value

        uint32_t last_heartbeat_ms;      // the time when the last HEARTBEAT message arrived from a GCS - used for triggering gcs failsafe
        uint32_t terrain_first_failure_ms;  // the first time terrain data access failed - used to calculate the duration of the failure
        uint32_t terrain_last_failure_ms;   // the most recent time terrain data access failed
    } failsafe;

    // sensor health for logging
    struct {
        uint8_t baro        : 1;    // true if baro is healthy
        uint8_t compass     : 1;    // true if compass is healthy
        uint8_t primary_gps;        // primary gps index
    } sensor_health;

    // GPS variables
    // Sometimes we need to remove the scaling for distance calcs
    float scaleLongDown;

    // Location & Navigation
    int32_t wp_bearing;
    // The location of home in relation to the copter in centi-degrees
    int32_t home_bearing;
    // distance between plane and home in cm
    int32_t home_distance;
    // distance between plane and next waypoint in cm.
    uint32_t wp_distance;
    LandStateType land_state = LandStateType_FlyToLocation; // records state of land (flying to location, descending)

    struct {
        PayloadPlaceStateType state = PayloadPlaceStateType_Calibrating_Hover_Start; // records state of place (descending, releasing, released, ...)
        uint32_t hover_start_timestamp; // milliseconds
        float hover_throttle_level;
        uint32_t descend_start_timestamp; // milliseconds
        uint32_t place_start_timestamp; // milliseconds
        float descend_throttle_level;
        float descend_start_altitude;
        float descend_max; // centimetres
    } nav_payload_place;

    // Auto
    AutoMode auto_mode;   // controls which auto controller is run

    // Guided
    GuidedMode guided_mode;  // controls which controller is run (pos or vel)

    // RTL
    RTLState rtl_state;  // records state of rtl (initial climb, returning home, etc)
    bool rtl_state_complete; // set to true if the current state is completed

    struct {
        // NEU w/ Z element alt-above-ekf-origin unless use_terrain is true in which case Z element is alt-above-terrain
        Location_Class origin_point;
        Location_Class climb_target;
        Location_Class return_target;
        Location_Class descent_target;
        bool land;
        bool terrain_used;
    } rtl_path;

    // Circle
    bool circle_pilot_yaw_override; // true if pilot is overriding yaw

    // Stores initial bearing when armed - initial simple bearing is modified in super simple mode so not suitable
    int32_t initial_armed_bearing;

    // Loiter control
    uint16_t loiter_time_max;                // How long we should stay in Loiter Mode for mission scripting (time in seconds)
    uint32_t loiter_time;                    // How long have we been loitering - The start time in millis
    Vector2f Loiter_unit_vel_dir;
    float Loiter_vel_trig;
    enum Loiter_sub_state_t{
        Loiter_sub_Flying = 0,
        Loiter_sub_Stopping,
        Loiter_sub_Holding
    } Loiter_sub_state;
    
    // Brake
    uint32_t brake_timeout_start;
    uint32_t brake_timeout_ms;

    // Delay the next navigation command
    int32_t nav_delay_time_max;  // used for delaying the navigation commands (eg land,takeoff etc.)
    uint32_t nav_delay_time_start;

    // throw mode state
    struct {
        ThrowModeStage stage;
        ThrowModeStage prev_stage;
        uint32_t last_log_ms;
        bool nextmode_attempted;
        uint32_t free_fall_start_ms;    // system time free fall was detected
        float free_fall_start_velz;     // vertical velocity when free fall was detected
    } throw_state = {Throw_Disarmed, Throw_Disarmed, 0, false, 0, 0.0f};

    // Altitude
    // The cm/s we are moving up or down based on filtered data - Positive = UP
    int16_t climb_rate;
    float target_rangefinder_alt;   // desired altitude in cm above the ground
    int32_t baro_alt;            // barometer altitude in cm above home
    float baro_climbrate;        // barometer climbrate in cm/s
    LowPassFilterVector3f land_accel_ef_filter; // accelerations for land and crash detector tests

    // filtered pilot's throttle input used to cancel landing if throttle held high
    LowPassFilterFloat rc_throttle_control_in_filter;

    // 3D Location vectors
    // Current location of the copter (altitude is relative to home)
    Location_Class current_loc;

    // Navigation Yaw control
    // auto flight mode's yaw mode
    uint8_t auto_yaw_mode;

    // Yaw will point at this location if auto_yaw_mode is set to AUTO_YAW_ROI
    Vector3f roi_WP;

    // bearing from current location to the yaw_look_at_WP
    float yaw_look_at_WP_bearing;

    // yaw used for YAW_LOOK_AT_HEADING yaw_mode
    int32_t yaw_look_at_heading;

    // Deg/s we should turn
    int16_t yaw_look_at_heading_slew;

    // heading when in yaw_look_ahead_bearing
    float yaw_look_ahead_bearing;

    // turn rate (in cds) when auto_yaw_mode is set to AUTO_YAW_RATE
    float auto_yaw_rate_cds;

    // Delay Mission Scripting Command
    int32_t condition_value;  // used in condition commands (eg delay, change alt, etc.)
    uint32_t condition_start;

    // IMU variables
    // Integration time (in seconds) for the gyros (DCM algorithm)
    // Updated with the fast loop
    float G_Dt;

    // Inertial Navigation
    AP_InertialNav_NavEKF inertial_nav;

    // Performance monitoring
    int16_t pmTest1;

    // System Timers
    // --------------
    // Time in microseconds of main control loop
    uint64_t fast_loopTimer;
    // Counter of main loop executions.  Used for performance monitoring and failsafe processing
    uint16_t mainLoop_count;
    // Loiter timer - Records how long we have been in loiter
    uint32_t rtl_loiter_start_time;
    // arm_time_ms - Records when vehicle was armed. Will be Zero if we are disarmed.
    uint32_t arm_time_ms;

    // Used to exit the roll and pitch auto trim function
    uint8_t auto_trim_counter;

    // use this to prevent recursion during sensor init
    bool in_mavlink_delay;

    // true if we are out of time in our event timeslice
    bool gcs_out_of_time;

    // last valid RC input time
    uint32_t last_radio_update_ms;

    // last esc calibration notification update
    uint32_t esc_calibration_notify_update_ms;

    // Top-level logic

    struct FMT_mission_t {
        bool current_mission_verified;
    } FMT_mission;

    float test_value_p1;


private:

    // ground effect detector
    struct {
        bool takeoff_expected;
        bool touchdown_expected;
        uint32_t takeoff_time_ms;
        float takeoff_alt_cm;
    } gndeffect_state;

    // static const AP_Scheduler::Task scheduler_tasks[];
};


extern Copter* copter;

