#pragma once

/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *  AHRS (Attitude Heading Reference System) interface for ArduPilot
 *
 */

#include "AP_Math.h"
#include "inttypes.h"
#include "AP_InertialSensor.h"

class OpticalFlow;
#define AP_AHRS_TRIM_LIMIT 10.0f        // maximum trim angle in degrees
#define AP_AHRS_RP_P_MIN   0.05f        // minimum value for AHRS_RP_P parameter
#define AP_AHRS_YAW_P_MIN  0.05f        // minimum value for AHRS_YAW_P parameter

enum AHRS_VehicleClass {
    AHRS_VEHICLE_UNKNOWN,
    AHRS_VEHICLE_GROUND,
    AHRS_VEHICLE_COPTER,
    AHRS_VEHICLE_FIXED_WING,
    AHRS_VEHICLE_SUBMARINE,
};


// forward declare view class
class AP_AHRS_View;

class AP_AHRS
{
public:
    friend class AP_AHRS_View;
    
    // Constructor
    AP_AHRS() :
        roll(0.0f),
        pitch(0.0f),
        yaw(0.0f),
        roll_sensor(0),
        pitch_sensor(0),
        yaw_sensor(0),
        _vehicle_class(AHRS_VEHICLE_UNKNOWN),
        _compass_last_update(0),
        _cos_roll(1.0f),
        _cos_pitch(1.0f),
        _cos_yaw(1.0f),
        _sin_roll(0.0f),
        _sin_pitch(0.0f),
        _sin_yaw(0.0f),
        _active_accel_instance(0)
    {

        // base the ki values by the sensors maximum drift
        // rate.

        // enable centrifugal correction by default
        _flags.correct_centrifugal = true;

        // initialise _home
        _home.options    = 0;
        _home.alt        = 0;
        _home.lng        = 0;
        _home.lat        = 0;

        _last_trim = Vector3f(0.0f,0.0f,0.0f);
        _rotation_autopilot_body_to_vehicle_body.from_euler(0.0f, 0.0f, 0.0f);
        _rotation_vehicle_body_to_autopilot_body = _rotation_autopilot_body_to_vehicle_body.transposed();
    }

    // empty virtual destructor
    ~AP_AHRS() {};

    // init sets up INS board orientation
    void init() {
        //set_orientation();
    };

    // Accessors
    void set_fly_forward(bool b) {
        _flags.fly_forward = b;
    }

    bool get_fly_forward(void) const {
        return _flags.fly_forward;
    }

    AHRS_VehicleClass get_vehicle_class(void) const {
        return _vehicle_class;
    }

    void set_vehicle_class(AHRS_VehicleClass vclass) {
        _vehicle_class = vclass;
    }

    void set_wind_estimation(bool b) {
        _flags.wind_estimation = b;
    }



   
    // Euler angles (radians)
    float roll;
    float pitch;
    float yaw;

    // integer Euler angles (Degrees * 100)
    int32_t roll_sensor;
    int32_t pitch_sensor;
    int32_t yaw_sensor;


    virtual const Matrix3f &get_rotation_body_to_ned(void) const = 0;
    const Matrix3f& get_rotation_autopilot_body_to_vehicle_body(void) const { return _rotation_autopilot_body_to_vehicle_body; }
    const Matrix3f& get_rotation_vehicle_body_to_autopilot_body(void) const { return _rotation_vehicle_body_to_autopilot_body; }

    // returns the expected NED magnetic field
    bool get_expected_mag_field_NED(Vector3f &ret) const {
        return false;
    }



    // return true if yaw has been initialised
    bool yaw_initialised(void) const {
        return _flags.have_initial_yaw;
    }

    // set the correct centrifugal flag
    // allows arducopter to disable corrections when disarmed
    void set_correct_centrifugal(bool setting) {
        _flags.correct_centrifugal = setting;
    }

    // get the correct centrifugal flag
    bool get_correct_centrifugal(void) const {
        return _flags.correct_centrifugal;
    }

    // helper trig value accessors
    float cos_roll() const  {
        return _cos_roll;
    }
    float cos_pitch() const {
        return _cos_pitch;
    }
    float cos_yaw() const   {
        return _cos_yaw;
    }
    float sin_roll() const  {
        return _sin_roll;
    }
    float sin_pitch() const {
        return _sin_pitch;
    }
    float sin_yaw() const   {
        return _sin_yaw;
    }

    // get the home location. This is const to prevent any changes to
    // home without telling AHRS about the change
    const struct Location &get_home(void) const {
        return _home;
    }

    // return the active accelerometer instance
    uint8_t get_active_accel_instance(void) const {
        return _active_accel_instance;
    }

    // get the selected ekf type, for allocation decisions
    int8_t get_ekf_type(void) const {
        return _ekf_type;
    }


    // settable parameters
    // these are public for ArduCopter
    float _kp_yaw;//para
    float _kp;//para

    float beta;//para
    int8_t _wind_max;//para
    int8_t _board_orientation;//para
    int8_t _ekf_type;//para
    // a vector to capture the difference between the controller and body frames
    Vector3f         _trim;
    
protected:
    AHRS_VehicleClass _vehicle_class;


    // flags structure
    struct ahrs_flags {
        uint8_t have_initial_yaw        : 1;    // whether the yaw value has been intialised with a reference
        uint8_t fly_forward             : 1;    // 1 if we can assume the aircraft will be flying forward on its X axis
        uint8_t correct_centrifugal     : 1;    // 1 if we should correct for centrifugal forces (allows arducopter to turn this off when motors are disarmed)
        uint8_t wind_estimation         : 1;    // 1 if we should do wind estimation
    } _flags;

    // calculate sin/cos of roll/pitch/yaw from rotation
    void calc_trig(const Matrix3f &rot,
                   float &cr, float &cp, float &cy,
                   float &sr, float &sp, float &sy) const;
    
    // update_trig - recalculates _cos_roll, _cos_pitch, etc based on latest attitude
    //      should be called after _dcm_matrix is updated
    void update_trig(void);

    // update roll_sensor, pitch_sensor and yaw_sensor
    void update_cd_values(void);

    // time in microseconds of last compass update
    uint32_t _compass_last_update;

    // note: we use ref-to-pointer here so that our caller can change the GPS without our noticing
    //       IMU under us without our noticing.


    // cached trim rotations
    Vector3f _last_trim;
    Matrix3f _rotation_autopilot_body_to_vehicle_body;
    Matrix3f _rotation_vehicle_body_to_autopilot_body;

    // the limit of the gyro drift claimed by the sensors, in
    // radians/s/s
    float _gyro_drift_limit;

    // accelerometer values in the earth frame in m/s/s
    Vector3f        _accel_ef[INS_MAX_INSTANCES];
    Vector3f        _accel_ef_blended;

    // Declare filter states for HPF and LPF used by complementary
    // filter in AP_AHRS::groundspeed_vector
    Vector2f _lp; // ground vector low-pass filter
    Vector2f _hp; // ground vector high-pass filter
    Vector2f _lastGndVelADS; // previous HPF input

    // reference position for NED positions
    struct Location _home;

    // helper trig variables
    float _cos_roll, _cos_pitch, _cos_yaw;
    float _sin_roll, _sin_pitch, _sin_yaw;

    // which accelerometer instance is active
    uint8_t _active_accel_instance;

    // optional view class
    AP_AHRS_View *_view;

    // AOA and SSA
    float _AOA, _SSA;
    uint32_t _last_AOA_update_ms;
};

