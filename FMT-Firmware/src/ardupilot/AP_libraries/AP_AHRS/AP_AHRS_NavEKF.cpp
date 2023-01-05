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
#include "AP_AHRS_NavEKF.h"
#include "AP_AHRS_View.h"
#include "AP_Vehicle.h"

// constructor
AP_AHRS_NavEKF::AP_AHRS_NavEKF(Flags flags) :
    AP_AHRS(),
    _ekf2_started(false),
    _ekf3_started(false),
    _force_ekf(false),
    _ekf_flags(flags)
{
    _ekf_type = 10;
    _dcm_matrix.identity();
}

// return the smoothed gyro vector corrected for drift
const Vector3f &AP_AHRS_NavEKF::get_gyro(void) const
{
    return _gyro_estimate;
}

const Matrix3f &AP_AHRS_NavEKF::get_rotation_body_to_ned(void) const
{
    return _dcm_matrix;
}

const Vector3f &AP_AHRS_NavEKF::get_gyro_drift(void) const
{
    return _gyro_drift;
}

// reset the current gyro drift estimate
//  should be called if gyro offsets are recalculated
void AP_AHRS_NavEKF::reset_gyro_drift(void)
{
}

void AP_AHRS_NavEKF::update(bool skip_ins_update)
{
    update_SITL();
    
    if (_view != nullptr) {
        // update optional alternative attitude view
        _view->update(skip_ins_update);
    }
}

// return a ground speed estimate in m/s
const Vector3f AP_AHRS_NavEKF::get_gyro_latest(void) const
{
    return Vector3f();
}


void AP_AHRS_NavEKF::update_SITL(void)
{
    ;
}

// accelerometer values in the earth frame in m/s/s
const Vector3f &AP_AHRS_NavEKF::get_accel_ef(uint8_t i) const
{
    return _accel_ef_ekf[i];
}

// blended accelerometer values in the earth frame in m/s/s
const Vector3f &AP_AHRS_NavEKF::get_accel_ef_blended(void) const
{
    return _accel_ef_ekf_blended;
}

void AP_AHRS_NavEKF::reset(bool recover_eulers)
{
    ;
}

// reset the current attitude, used on new IMU calibration
void AP_AHRS_NavEKF::reset_attitude(const float &_roll, const float &_pitch, const float &_yaw)
{
    ;
}

// dead-reckoning support
bool AP_AHRS_NavEKF::get_position(struct Location &loc) const
{
    return false;
}

// return a wind estimation vector, in m/s
Vector3f AP_AHRS_NavEKF::wind_estimate(void)
{
    Vector3f wind;
    wind.zero();
    return wind;
}

// return an airspeed estimate if available. return true
// if we have an estimate
bool AP_AHRS_NavEKF::airspeed_estimate(float *airspeed_ret) const
{
    return false;
}

// EKF has a better ground speed vector estimate
Vector2f AP_AHRS_NavEKF::groundspeed_vector(void)
{
    return Vector2f();
}

void AP_AHRS_NavEKF::set_home(const Location &loc)
{
    _home = loc;
    _home.options = 0;
}

// set the EKF's origin location in 10e7 degrees.  This should only
// be called when the EKF has no absolute position reference (i.e. GPS)
// from which to decide the origin on its own
bool AP_AHRS_NavEKF::set_origin(const Location &loc)
{
    // return success if active EKF's origin was set
    return false;
}

// return true if inertial navigation is active
bool AP_AHRS_NavEKF::have_inertial_nav(void) const
{
    return false;
}

// return a ground velocity in meters/second, North/East/Down
// order. Must only be called if have_inertial_nav() is true
bool AP_AHRS_NavEKF::get_velocity_NED(Vector3f &vec) const
{
    return false;
}

// returns the expected NED magnetic field
bool AP_AHRS_NavEKF::get_mag_field_NED(Vector3f &vec) const
{
    return false;
}

// returns the estimated magnetic field offsets in body frame
bool AP_AHRS_NavEKF::get_mag_field_correction(Vector3f &vec) const
{
    return false;
}

// Get a derivative of the vertical position which is kinematically consistent with the vertical position is required by some control loops.
// This is different to the vertical velocity from the EKF which is not always consistent with the verical position due to the various errors that are being corrected for.
bool AP_AHRS_NavEKF::get_vert_pos_rate(float &velocity) const
{
    return false;
}

// get latest height above ground level estimate in metres and a validity flag
bool AP_AHRS_NavEKF::get_hagl(float &height) const
{
    return false;
}

// return a relative ground position to the origin in meters
// North/East/Down order.
bool AP_AHRS_NavEKF::get_relative_position_NED_origin(Vector3f &vec) const
{
    return false;
}

// return a relative ground position to the home in meters
// North/East/Down order.
bool AP_AHRS_NavEKF::get_relative_position_NED_home(Vector3f &vec) const
{
    Location originLLH;
    Vector3f originNED;
    if (!get_relative_position_NED_origin(originNED) ||
        !get_origin(originLLH)) {
        return false;
    }

    Vector3f offset = location_3d_diff_NED(originLLH, _home);

    vec.x = originNED.x - offset.x;
    vec.y = originNED.y - offset.y;
    vec.z = originNED.z - offset.z;
    return true;
}

// write a relative ground position estimate to the origin in meters, North/East order
// return true if estimate is valid
bool AP_AHRS_NavEKF::get_relative_position_NE_origin(Vector2f &posNE) const
{
    Location loc;
    get_position(loc);
    posNE = location_diff(get_home(), loc);
    return true;
}

// return a relative ground position to the home in meters
// North/East order.
bool AP_AHRS_NavEKF::get_relative_position_NE_home(Vector2f &posNE) const
{
    Location originLLH;
    Vector2f originNE;
    if (!get_relative_position_NE_origin(originNE) ||
        !get_origin(originLLH)) {
        return false;
    }

    Vector2f offset = location_diff(originLLH, _home);

    posNE.x = originNE.x - offset.x;
    posNE.y = originNE.y - offset.y;
    return true;
}

// write a relative ground position estimate to the origin in meters, North/East order


// write a relative ground position to the origin in meters, Down
// return true if the estimate is valid
bool AP_AHRS_NavEKF::get_relative_position_D_origin(float &posD) const
{
    return false;
}

// write a relative ground position to home in meters, Down
// will use the barometer if the EKF isn't available
void AP_AHRS_NavEKF::get_relative_position_D_home(float &posD) const
{
    return;
}
/*
  canonicalise _ekf_type, forcing it to be 0, 2 or 3
  type 1 has been deprecated
 */
uint8_t AP_AHRS_NavEKF::ekf_type(void) const
{
    uint8_t type = _ekf_type;
    return type;
}

/*
  check if the AHRS subsystem is healthy
*/
bool AP_AHRS_NavEKF::healthy(void) 
{
    // If EKF is started we switch away if it reports unhealthy. This could be due to bad
    // sensor data. If EKF reversion is inhibited, we only switch across if the EKF encounters
    // an internal processing error, but not for bad sensor data.
    return true;
}

// true if the AHRS has completed initialisation
bool AP_AHRS_NavEKF::initialised(void) 
{
    return true;
}

// get_filter_status : returns filter status as a series of flags
bool AP_AHRS_NavEKF::get_filter_status(nav_filter_status &status) const
{
    memset(&status, 0, sizeof(status));
    status.flags.attitude = 1;
    status.flags.horiz_vel = 1;
    status.flags.vert_vel = 1;
    status.flags.horiz_pos_rel = 1;
    status.flags.horiz_pos_abs = 1;
    status.flags.vert_pos = 1;
    status.flags.pred_horiz_pos_rel = 1;
    status.flags.pred_horiz_pos_abs = 1;
    status.flags.using_gps = 1;
    return true;
}

// inhibit GPS usage
uint8_t AP_AHRS_NavEKF::setInhibitGPS(void)
{
    return false;
}

// get speed limit
void AP_AHRS_NavEKF::getEkfControlLimits(float &ekfGndSpdLimit, float &ekfNavVelGainScaler)
{
    ekfGndSpdLimit = 400.0f;
    ekfNavVelGainScaler = 1.0f;
}

// get compass offset estimates
// true if offsets are valid
bool AP_AHRS_NavEKF::getMagOffsets(uint8_t mag_idx, Vector3f &magOffsets)
{
    magOffsets.zero();
    return true;
}

// return the amount of yaw angle change due to the last yaw angle reset in radians
// returns the time of the last yaw angle reset or 0 if no reset has ever occurred
uint32_t AP_AHRS_NavEKF::getLastYawResetAngle(float &yawAng) 
{
    return 0;
}

// return the amount of NE position change in metres due to the last reset
// returns the time of the last reset or 0 if no reset has ever occurred
uint32_t AP_AHRS_NavEKF::getLastPosNorthEastReset(Vector2f &pos) 
{
    return 0;
}

// return the amount of NE velocity change in metres/sec due to the last reset
// returns the time of the last reset or 0 if no reset has ever occurred
uint32_t AP_AHRS_NavEKF::getLastVelNorthEastReset(Vector2f &vel) 
{
    return 0;
}


// return the amount of vertical position change due to the last reset in meters
// returns the time of the last reset or 0 if no reset has ever occurred
uint32_t AP_AHRS_NavEKF::getLastPosDownReset(float &posDelta) 
{
    return 0;
}

// Resets the baro so that it reads zero at the current height
// Resets the EKF height to zero
// Adjusts the EKf origin height so that the EKF height + origin height is the same as before
// Returns true if the height datum reset has been performed
// If using a range finder for height no reset is performed and it returns false
bool AP_AHRS_NavEKF::resetHeightDatum(void)
{
    return false;
}

// passes a reference to the location of the inertial navigation origin
// in WGS-84 coordinates
// returns a boolean true when the inertial navigation origin has been set
bool AP_AHRS_NavEKF::get_origin(Location &ret) const
{
    ret = get_home();
    return ret.lat != 0 || ret.lng != 0;
}

// get_hgt_ctrl_limit - get maximum height to be observed by the control loops in metres and a validity flag
// this is used to limit height during optical flow navigation
// it will return invalid when no limiting is required
bool AP_AHRS_NavEKF::get_hgt_ctrl_limit(float& limit) const
{
    return false;
}

// get_location - updates the provided location with the latest calculated location
//  returns true on success (i.e. the EKF knows it's latest position), false on failure
bool AP_AHRS_NavEKF::get_location(struct Location &loc) const
{
    return get_position(loc);
}

// get_variances - provides the innovations normalised using the innovation variance where a value of 0
// indicates prefect consistency between the measurement and the EKF solution and a value of of 1 is the maximum
// inconsistency that will be accpeted by the filter
// boolean false is returned if variances are not available
bool AP_AHRS_NavEKF::get_variances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar, Vector2f &offset)
{
    velVar = 0;
    posVar = 0;
    hgtVar = 0;
    magVar.zero();
    tasVar = 0;
    offset.zero();
    return true;
}

void AP_AHRS_NavEKF::setTakeoffExpected(bool val)
{
    ;
}

void AP_AHRS_NavEKF::setTouchdownExpected(bool val)
{
    ;
}

bool AP_AHRS_NavEKF::getGpsGlitchStatus()
{
    nav_filter_status ekf_status {};
    if (!get_filter_status(ekf_status)) {
        return false;
    }
    return ekf_status.flags.gps_glitching;
}

// get the index of the current primary IMU
uint8_t AP_AHRS_NavEKF::get_primary_IMU_index()
{
    return 0;
}

// get earth-frame accel vector for primary IMU
const Vector3f &AP_AHRS_NavEKF::get_accel_ef() const
{
    return get_accel_ef(0);
}


// get the index of the current primary accelerometer sensor
uint8_t AP_AHRS_NavEKF::get_primary_accel_index(void)
{
    return 0;
}

// get the index of the current primary gyro sensor
uint8_t AP_AHRS_NavEKF::get_primary_gyro_index(void)
{
    return 0;
}

/*
  create a rotated view of AP_AHRS
 */
AP_AHRS_View *AP_AHRS_NavEKF::create_view(enum Rotation rotation)
{
    if (_view != nullptr) {
        // can only have one
        return nullptr;
    }
    _view = new AP_AHRS_View(*this, rotation);
    return _view;
}
