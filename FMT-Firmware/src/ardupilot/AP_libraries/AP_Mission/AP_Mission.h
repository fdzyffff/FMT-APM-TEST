/// @file    AP_Mission.h
/// @brief   Handles the MAVLINK command mission stack.  Reads and writes mission to storage.

/*
 *   The AP_Mission library:
 *   - responsible for managing a list of commands made up of "nav", "do" and "conditional" commands
 *   - reads and writes the mission commands to storage.
 *   - provides easy access to current, previous and upcoming waypoints
 *   - calls main program's command execution and verify functions.
 *   - accounts for the DO_JUMP command
 *
 */
#pragma once

#include "AP_Vehicle.h"
#include "AP_Math.h"
#include "AP_Common.h"

/// @class    AP_Mission
/// @brief    Object managing Mission
class AP_Mission {

public:
    // jump command structure
    struct PACKED Jump_Command {
        uint16_t target;        // target command id
        int16_t num_times;      // num times to repeat.  -1 = repeat forever
    };

    // condition delay command structure
    struct PACKED Conditional_Delay_Command {
        float seconds;          // period of delay in seconds
    };

    // condition delay command structure
    struct PACKED Conditional_Distance_Command {
        float meters;           // distance from next waypoint in meters
    };

    // condition yaw command structure
    struct PACKED Yaw_Command {
        float angle_deg;        // target angle in degrees (0=north, 90=east)
        float turn_rate_dps;    // turn rate in degrees / second (0=use default)
        int8_t direction;       // -1 = ccw, +1 = cw
        uint8_t relative_angle; // 0 = absolute angle, 1 = relative angle
    };

    // change speed command structure
    struct PACKED Change_Speed_Command {
        uint8_t speed_type;     // 0=airspeed, 1=ground speed
        float target_ms;        // target speed in m/s, -1 means no change
        float throttle_pct;     // throttle as a percentage (i.e. 0 ~ 100), -1 means no change
    };

    // set relay command structure
    struct PACKED Set_Relay_Command {
        uint8_t num;            // relay number from 1 to 4
        uint8_t state;          // on = 3.3V or 5V (depending upon board), off = 0V.  only used for do-set-relay, not for do-repeat-relay
    };

    // repeat relay command structure
    struct PACKED Repeat_Relay_Command {
        uint8_t num;            // relay number from 1 to 4
        int16_t repeat_count;   // number of times to trigger the relay
        float cycle_time;       // cycle time in seconds (the time between peaks or the time the relay is on and off for each cycle?)
    };

    // set servo command structure
    struct PACKED Set_Servo_Command {
        uint8_t channel;        // servo channel
        uint16_t pwm;           // pwm value for servo
    };

    // repeat servo command structure
    struct PACKED Repeat_Servo_Command {
        uint8_t channel;        // servo channel
        uint16_t pwm;           // pwm value for servo
        int16_t repeat_count;   // number of times to move the servo (returns to trim in between)
        float cycle_time;       // cycle time in seconds (the time between peaks or the time the servo is at the specified pwm value for each cycle?)
    };

    // mount control command structure
    struct PACKED Mount_Control {
        float pitch;            // pitch angle in degrees
        float roll;             // roll angle in degrees
        float yaw;              // yaw angle (relative to vehicle heading) in degrees
    };

    // digicam control command structure
    struct PACKED Digicam_Configure {
        uint8_t shooting_mode;  // ProgramAuto = 1, AV = 2, TV = 3, Man=4, IntelligentAuto=5, SuperiorAuto=6
        uint16_t shutter_speed;
        uint8_t aperture;       // F stop number * 10
        uint16_t ISO;           // 80, 100, 200, etc
        uint8_t exposure_type;
        uint8_t cmd_id;
        float engine_cutoff_time;   // seconds
    };

    // digicam control command structure
    struct PACKED Digicam_Control {
        uint8_t session;        // 1 = on, 0 = off
        uint8_t zoom_pos;
        int8_t zoom_step;       // +1 = zoom in, -1 = zoom out
        uint8_t focus_lock;
        uint8_t shooting_cmd;
        uint8_t cmd_id;
    };

    // set cam trigger distance command structure
    struct PACKED Cam_Trigg_Distance {
        float meters;           // distance
    };

    // gripper command structure
    struct PACKED Gripper_Command {
        uint8_t num;            // gripper number
        uint8_t action;         // action (0 = release, 1 = grab)
    };

    // high altitude balloon altitude wait
    struct PACKED Altitude_Wait {
        float altitude; // meters
        float descent_rate; // m/s
        uint8_t wiggle_time; // seconds
    };

    // nav guided command
    struct PACKED Guided_Limits_Command {
        // max time is held in p1 field
        float alt_min;          // min alt below which the command will be aborted.  0 for no lower alt limit
        float alt_max;          // max alt above which the command will be aborted.  0 for no upper alt limit
        float horiz_max;        // max horizontal distance the vehicle can move before the command will be aborted.  0 for no horizontal limit
    };

    // do VTOL transition
    struct PACKED Do_VTOL_Transition {
        uint8_t target_state;
    };

     // navigation delay command structure
    struct PACKED Navigation_Delay_Command {
        float seconds; // period of delay in seconds
        int8_t hour_utc; // absolute time's hour (utc)
        int8_t min_utc; // absolute time's min (utc)
        int8_t sec_utc; // absolute time's sec (utc)
    };

    // DO_ENGINE_CONTROL support
    struct PACKED Do_Engine_Control {
        bool start_control; // start or stop engine
        bool cold_start; // use cold start procedure
        uint16_t height_delay_cm; // height delay for start
    };
    
    union PACKED Content {
        // jump structure
        Jump_Command jump;

        // conditional delay
        Conditional_Delay_Command delay;

        // conditional distance
        Conditional_Distance_Command distance;

        // conditional yaw
        Yaw_Command yaw;

        // change speed
        Change_Speed_Command speed;

        // do-set-relay
        Set_Relay_Command relay;

        // do-repeat-relay
        Repeat_Relay_Command repeat_relay;

        // do-set-servo
        Set_Servo_Command servo;

        // do-repeate-servo
        Repeat_Servo_Command repeat_servo;

        // mount control
        Mount_Control mount_control;

        // camera configure
        Digicam_Configure digicam_configure;

        // camera control
        Digicam_Control digicam_control;

        // cam trigg distance
        Cam_Trigg_Distance cam_trigg_dist;

        // do-gripper
        Gripper_Command gripper;

        // do-guided-limits
        Guided_Limits_Command guided_limits;

        // cam trigg distance
        Altitude_Wait altitude_wait;

        // do vtol transition
        Do_VTOL_Transition do_vtol_transition;

        // DO_ENGINE_CONTROL
        Do_Engine_Control do_engine_control;
        
        // location
        Location location;      // Waypoint location

        // navigation delay
        Navigation_Delay_Command nav_delay;

        // raw bytes, for reading/writing to eeprom. Note that only 10 bytes are available
        // if a 16 bit command ID is used
        uint8_t bytes[12];
    };

    // command structure
    struct Mission_Command {
        uint16_t index;             // this commands position in the command list
        uint16_t id;                // mavlink command id
        uint16_t p1;                // general purpose parameter 1
        Content content;
    };

    // mission state enumeration
    enum mission_state {
        MISSION_STOPPED=0,
        MISSION_RUNNING=1,
        MISSION_COMPLETE=2
    };

    /// constructor
    AP_Mission(){}
};
