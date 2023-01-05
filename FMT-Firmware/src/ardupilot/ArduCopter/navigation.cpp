#include "Copter.h"

// #include <firmament.h>

// run_nav_updates - top level call for the autopilot
// ensures calculations such as "distance to waypoint" are calculated before autopilot makes decisions
// To-Do - rename and move this function to make it's purpose more clear
void Copter::run_nav_updates(void)
{
    // calculate distance and bearing for reporting and autopilot decisions
    calc_distance_and_bearing();

    navigation_update();
}

// calc_distance_and_bearing - calculate distance and bearing to next waypoint and home
void Copter::calc_distance_and_bearing()
{
    calc_wp_distance();
    calc_wp_bearing();
    calc_home_distance_and_bearing();
}

// calc_wp_distance - calculate distance to next waypoint for reporting and autopilot decisions
void Copter::calc_wp_distance()
{
    // get target from loiter or wpinav controller
    switch (control_mode) {
    case LOITER:
    case CIRCLE:
        wp_distance = wp_nav->get_loiter_distance_to_target();
        break;

    case AUTO:
    case RTL:
        wp_distance = wp_nav->get_wp_distance_to_destination();
        break;

    case GUIDED:
        if (guided_mode == Guided_WP) {
            wp_distance = wp_nav->get_wp_distance_to_destination();
            break;
        }
        // no break
    default:
        wp_distance = 0;
        break;
    }
}

// calc_wp_bearing - calculate bearing to next waypoint for reporting and autopilot decisions
void Copter::calc_wp_bearing()
{
    // get target from loiter or wpinav controller
    switch (control_mode) {
    case LOITER:
    case CIRCLE:
        wp_bearing = wp_nav->get_loiter_bearing_to_target();
        break;

    case AUTO:
    case RTL:
        wp_bearing = wp_nav->get_wp_bearing_to_destination();
        break;

    case GUIDED:
        if (guided_mode == Guided_WP) {
            wp_bearing = wp_nav->get_wp_bearing_to_destination();
            break;
        }
        // no break
    default:
        wp_bearing = 0;
        break;
    }
}

// calc_home_distance_and_bearing - calculate distance and bearing to home for reporting and autopilot decisions
void Copter::calc_home_distance_and_bearing()
{
    // calculate home distance and bearing
    if (position_ok()) {
        Vector3f home = pv_location_to_vector(ahrs.get_home());
        Vector3f curr = inertial_nav.get_position();
        home_distance = pv_get_horizontal_distance_cm(curr, home);
        home_bearing = pv_get_bearing_cd(curr,home);
    }
}

void Copter::navigation_init() {
    FMT_mission.current_mission_verified = true;
}

void Copter::navigation_update()
{
       return;
}

void Copter::navigation_next() {
    return;
}

Vector3f Copter::get_destination() {
    Vector3f dest_pos = Vector3f(0.0f, 0.0f, 0.0f);
    switch (control_mode) {
    case AUTO:
    case RTL:
        dest_pos = wp_nav->get_wp_destination();
        break;

    case GUIDED:
        if (guided_mode == Guided_WP) {
            dest_pos = wp_nav->get_wp_destination();
        } else {
            dest_pos = pos_control->get_pos_target();
        }
        break;
        // no break

    case LOITER:
    case CIRCLE:
        dest_pos = pos_control->get_pos_target();
        break;

    default:
        break;
    }
    return dest_pos;
}