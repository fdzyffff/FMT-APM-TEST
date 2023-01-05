/*
  APM_AHRS.cpp

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
#include "AP_AHRS.h"
#include "AP_AHRS_View.h"

/*
  calculate sin and cos of roll/pitch/yaw from a body_to_ned rotation matrix
 */
void AP_AHRS::calc_trig(const Matrix3f &rot,
                        float &cr, float &cp, float &cy,
                        float &sr, float &sp, float &sy) const
{
    Vector2f yaw_vector;

    yaw_vector.x = rot.a.x;
    yaw_vector.y = rot.b.x;
    if (fabsf(yaw_vector.x) > 0 ||
        fabsf(yaw_vector.y) > 0) {
        yaw_vector.normalize();
    }
    sy = apm_constrain_float(yaw_vector.y, -1.0, 1.0);
    cy = apm_constrain_float(yaw_vector.x, -1.0, 1.0);

    // sanity checks
    if (yaw_vector.is_inf() || yaw_vector.is_nan()) {
        sy = 0.0f;
        cy = 1.0f;
    }
    
    float cx2 = rot.c.x * rot.c.x;
    if (cx2 >= 1.0f) {
        cp = 0;
        cr = 1.0f;
    } else {
        cp = safe_sqrt(1 - cx2);
        cr = rot.c.z / cp;
    }
    cp = apm_constrain_float(cp, 0, 1.0);
    cr = apm_constrain_float(cr, -1.0, 1.0); // this relies on apm_constrain_float() of infinity doing the right thing

    sp = -rot.c.x;

    if (!is_zero(cp)) {
        sr = rot.c.y / cp;
    }

    if (is_zero(cp) || std::isinf(cr) || std::isnan(cr) || std::isinf(sr) || std::isnan(sr)) {
        float r, p, y;
        rot.to_euler(&r, &p, &y);
        cr = cosf(r);
        sr = sinf(r);
    }
}

// update_trig - recalculates _cos_roll, _cos_pitch, etc based on latest attitude
//      should be called after _dcm_matrix is updated
void AP_AHRS::update_trig(void)
{
    calc_trig(get_rotation_body_to_ned(),
              _cos_roll, _cos_pitch, _cos_yaw,
              _sin_roll, _sin_pitch, _sin_yaw);
}

/*
  update the centi-degree values
 */
void AP_AHRS::update_cd_values(void)
{
    roll_sensor  = degrees(roll) * 100;
    pitch_sensor = degrees(pitch) * 100;
    yaw_sensor   = degrees(yaw) * 100;
    if (yaw_sensor < 0)
        yaw_sensor += 36000;
}


