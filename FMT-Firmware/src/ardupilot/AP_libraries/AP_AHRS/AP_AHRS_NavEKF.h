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
 *  NavEKF based AHRS (Attitude Heading Reference System) interface for
 *  ArduPilot
 *
 */

#include "AP_AHRS.h"
#include "ap_hal.h"

#include "AP_Nav_Common.h"              // definitions shared by inertial and ekf nav filters

#define AP_AHRS_NAVEKF_AVAILABLE 1
#define AP_AHRS_NAVEKF_SETTLE_TIME_MS 20000     // time in milliseconds the ekf needs to settle after being started

class AP_AHRS_NavEKF : public AP_AHRS
{
public:
    enum Flags {
        FLAG_NONE = 0,
        FLAG_ALWAYS_USE_EKF = 0x1,
    };

    // Constructor
    AP_AHRS_NavEKF(Flags flags = FLAG_NONE);

    // return the smoothed gyro vector corrected for drift
    const Vector3f &get_gyro(void) const;
    const Matrix3f &get_rotation_body_to_ned(void) const override;

    // return the current drift correction integrator value
    const Vector3f &get_gyro_drift(void) const;

    const Vector3f get_gyro_latest(void) const;

    // reset the current gyro drift estimate
    //  should be called if gyro offsets are recalculated
    void reset_gyro_drift();

    void            update(bool skip_ins_update=false);
    void            reset(bool recover_eulers = false);

    // reset the current attitude, used on new IMU calibration
    void reset_attitude(const float &roll, const float &pitch, const float &yaw);

    // dead-reckoning support
    bool get_position(struct Location &loc) const;

    // get latest altitude estimate above ground level in meters and validity flag
    bool get_hagl(float &hagl) const;

    // status reporting of estimated error
    float           get_error_rp() {return 0.0f;}
    float           get_error_yaw() {return 0.0f;}

    // return a wind estimation vector, in m/s
    Vector3f wind_estimate();

    // return an airspeed estimate if available. return true
    // if we have an estimate
    bool airspeed_estimate(float *airspeed_ret) const;

    // true if compass is being used
    bool use_compass() {return true;}

    // EKF has a better ground speed vector estimate
    Vector2f groundspeed_vector();

    const Vector3f &get_accel_ef(uint8_t i) const;
    const Vector3f &get_accel_ef() const;

    // Retrieves the corrected NED delta velocity in use by the inertial navigation
    void getCorrectedDeltaVelocityNED(Vector3f& ret, float& dt) const ;

    // blended accelerometer values in the earth frame in m/s/s
    const Vector3f &get_accel_ef_blended() const;

    // set home location
    void set_home(const Location &loc);

    // set the EKF's origin location in 10e7 degrees.  This should only
    // be called when the EKF has no absolute position reference (i.e. GPS)
    // from which to decide the origin on its own
    bool set_origin(const Location &loc);

    // returns the inertial navigation origin in lat/lon/alt
    bool get_origin(Location &ret) const;

    bool have_inertial_nav() const;

    bool get_velocity_NED(Vector3f &vec) const;

    // return the relative position NED to either home or origin
    // return true if the estimate is valid
    bool get_relative_position_NED_home(Vector3f &vec) const;
    bool get_relative_position_NED_origin(Vector3f &vec) const;

    // return the relative position NE to either home or origin
    // return true if the estimate is valid
    bool get_relative_position_NE_home(Vector2f &posNE) const;
    bool get_relative_position_NE_origin(Vector2f &posNE) const;

    // return the relative position down to either home or origin
    // baro will be used for the _home relative one if the EKF isn't
    void get_relative_position_D_home(float &posD) const;
    bool get_relative_position_D_origin(float &posD) const;

    // Get a derivative of the vertical position in m/s which is kinematically consistent with the vertical position is required by some control loops.
    // This is different to the vertical velocity from the EKF which is not always consistent with the vertical position due to the various errors that are being corrected for.
    bool get_vert_pos_rate(float &velocity) const;

    // inhibit GPS usage
    uint8_t setInhibitGPS(void);

    // get speed limit
    void getEkfControlLimits(float &ekfGndSpdLimit, float &ekfNavVelGainScaler);

    void set_ekf_use(bool setting);

    // is the AHRS subsystem healthy?
    bool healthy();

    // true if the AHRS has completed initialisation
    bool initialised();

    // get_filter_status - returns filter status as a series of flags
    bool get_filter_status(nav_filter_status &status) const;

    // get compass offset estimates
    // true if offsets are valid
    bool getMagOffsets(uint8_t mag_idx, Vector3f &magOffsets);

    // return the amount of yaw angle change due to the last yaw angle reset in radians
    // returns the time of the last yaw angle reset or 0 if no reset has ever occurred
    uint32_t getLastYawResetAngle(float &yawAng);

    // return the amount of NE position change in meters due to the last reset
    // returns the time of the last reset or 0 if no reset has ever occurred
    uint32_t getLastPosNorthEastReset(Vector2f &pos);

    // return the amount of NE velocity change in meters/sec due to the last reset
    // returns the time of the last reset or 0 if no reset has ever occurred
    uint32_t getLastVelNorthEastReset(Vector2f &vel);

    // return the amount of vertical position change due to the last reset in meters
    // returns the time of the last reset or 0 if no reset has ever occurred
    uint32_t getLastPosDownReset(float &posDelta);

    // Resets the baro so that it reads zero at the current height
    // Resets the EKF height to zero
    // Adjusts the EKf origin height so that the EKF height + origin height is the same as before
    // Returns true if the height datum reset has been performed
    // If using a range finder for height no reset is performed and it returns false
    bool resetHeightDatum();
    
    // get_hgt_ctrl_limit - get maximum height to be observed by the control loops in meters and a validity flag
    // this is used to limit height during optical flow navigation
    // it will return invalid when no limiting is required
    bool get_hgt_ctrl_limit(float &limit) const;

    // get_llh - updates the provided location with the latest calculated location including absolute altitude
    //  returns true on success (i.e. the EKF knows it's latest position), false on failure
    bool get_location(struct Location &loc) const;

    // get_variances - provides the innovations normalised using the innovation variance where a value of 0
    // indicates perfect consistency between the measurement and the EKF solution and a value of of 1 is the maximum
    // inconsistency that will be accepted by the filter
    // boolean false is returned if variances are not available
    bool get_variances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar, Vector2f &offset);

    // returns the expected NED magnetic field
    bool get_mag_field_NED(Vector3f& ret) const;

    // returns the estimated magnetic field offsets in body frame
    bool get_mag_field_correction(Vector3f &ret) const;

    void setTakeoffExpected(bool val);
    void setTouchdownExpected(bool val);

    bool getGpsGlitchStatus();

    // used by Replay to force start at right timestamp
    void force_ekf_start(void) { _force_ekf = true; }

    // get the index of the current primary accelerometer sensor
    uint8_t get_primary_accel_index(void);

    // get the index of the current primary gyro sensor
    uint8_t get_primary_gyro_index(void);

    // get yaw rate in earth frame in radians/sec
    float get_yaw_rate_earth(void) {
        return get_gyro() * get_rotation_body_to_ned().c;
    }

    // return ground speed estimate in meters/second. Used by ground vehicles.
    float groundspeed(void) {
        return groundspeed_vector().length();
    }

    // create a view
    AP_AHRS_View *create_view(enum Rotation rotation);
private:
    enum EKF_TYPE {EKF_TYPE_NONE=0,
                   EKF_TYPE_SITL=10
    };
    EKF_TYPE active_EKF_type(void) const;

    bool always_use_EKF() const {
        return _ekf_flags & FLAG_ALWAYS_USE_EKF;
    }

    bool _ekf2_started;
    bool _ekf3_started;
    bool _force_ekf;
    Matrix3f _dcm_matrix;
    Vector3f _dcm_attitude;
    Vector3f _gyro_drift;
    Vector3f _gyro_estimate;
    Vector3f _accel_ef_ekf[INS_MAX_INSTANCES];
    Vector3f _accel_ef_ekf_blended;
    const uint16_t startup_delay_ms = 1000;
    uint32_t start_time_ms = 0;
    Flags _ekf_flags;

    uint8_t ekf_type(void) const;
    void update_DCM(bool skip_ins_update);

    // get the index of the current primary IMU
    uint8_t get_primary_IMU_index(void);
    
    void update_SITL(void);
};
