#include "Copter.h"

/*
 * Init and run calls for guided_nogps flight mode
 */

Vector3f Copter::guided_align(Vector3f& rel_posneu_vector) {
    Vector3f curr_pos = inertial_nav.get_position();
    Vector3f ret_vector = Vector3f(curr_pos.x-rel_posneu_vector.x, curr_pos.y-rel_posneu_vector.y, auto_takeoff_no_nav_alt_cm);
    return ret_vector;
}
