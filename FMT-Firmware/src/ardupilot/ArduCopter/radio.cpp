#include "Copter.h"

// Function that will read the radio data, limit servos and trigger a failsafe
// ----------------------------------------------------------------------------

void Copter::default_dead_zones()
{
    channel_roll->set_default_dead_zone(20);
    channel_pitch->set_default_dead_zone(20);
    channel_throttle->set_default_dead_zone(30);
    channel_yaw->set_default_dead_zone(20);
    RC_Channels::rc_channel(CH_6)->set_default_dead_zone(0);
}

void Copter::init_rc_in()
{
    channel_roll     = RC_Channels::rc_channel(rcmap.roll()-1);
    channel_pitch    = RC_Channels::rc_channel(rcmap.pitch()-1);
    channel_throttle = RC_Channels::rc_channel(rcmap.throttle()-1);
    channel_yaw      = RC_Channels::rc_channel(rcmap.yaw()-1);

    if (channel_roll == nullptr) {
        printf(" Err! channel_roll Fail, check compiler\n");
    }
    // set rc channel ranges
    channel_roll->set_angle(ROLL_PITCH_YAW_INPUT_MAX);
    channel_pitch->set_angle(ROLL_PITCH_YAW_INPUT_MAX);
    channel_yaw->set_angle(ROLL_PITCH_YAW_INPUT_MAX);
    channel_throttle->set_range(1000);

    //set auxiliary servo ranges
    RC_Channels::rc_channel(CH_5)->set_range(1000);
    RC_Channels::rc_channel(CH_6)->set_range(1000);
    RC_Channels::rc_channel(CH_7)->set_range(1000);
    RC_Channels::rc_channel(CH_8)->set_range(1000);

    // set default dead zones
    default_dead_zones();

    // initialise throttle_zero flag
    ap.throttle_zero = true;
}

 // init_rc_out -- initialise motors and check if pilot wants to perform ESC calibration
void Copter::init_rc_out()
{
    motors->set_update_rate(g.rc_speed);
    motors->set_loop_rate(400);
    motors->init((AP_Motors::motor_frame_class)g2.frame_class, (AP_Motors::motor_frame_type)g.frame_type);

    motors->set_throttle_range(channel_throttle->get_radio_min(), channel_throttle->get_radio_max());

    // refresh auxiliary channel to function map
    SRV_Channels::update_aux_servo_function();
}


// enable_motor_output() - enable and output lowest possible value to motors
void Copter::enable_motor_output()
{
    motors->output_min();
}

void Copter::read_radio()
{
    ;
}

#define FS_COUNTER 3        // radio failsafe kicks in after 3 consecutive throttle values below failsafe_throttle_value
void Copter::set_throttle_and_failsafe(uint16_t throttle_pwm)
{
    // if failsafe not enabled pass through throttle and exit
    if(g.failsafe_throttle == FS_THR_DISABLED) {
        channel_throttle->set_pwm(throttle_pwm);
        return;
    }

    //check for low throttle value
    if (throttle_pwm < (uint16_t)g.failsafe_throttle_value) {

        // if we are already in failsafe or motors not armed pass through throttle and exit
        if (failsafe.radio || !(ap.rc_receiver_present || motors->armed())) {
            channel_throttle->set_pwm(throttle_pwm);
            return;
        }

        // check for 3 low throttle values
        // Note: we do not pass through the low throttle until 3 low throttle values are received
        failsafe.radio_counter++;
        if( failsafe.radio_counter >= FS_COUNTER ) {
            failsafe.radio_counter = FS_COUNTER;  // check to ensure we don't overflow the counter
            set_failsafe_radio(true);
            channel_throttle->set_pwm(throttle_pwm);   // pass through failsafe throttle
        }
    }else{
        // we have a good throttle so reduce failsafe counter
        failsafe.radio_counter--;
        if( failsafe.radio_counter <= 0 ) {
            failsafe.radio_counter = 0;   // check to ensure we don't underflow the counter

            // disengage failsafe after three (nearly) consecutive valid throttle values
            if (failsafe.radio) {
                set_failsafe_radio(false);
            }
        }
        // pass through throttle
        channel_throttle->set_pwm(throttle_pwm);
    }
}

#define THROTTLE_ZERO_DEBOUNCE_TIME_MS 400
// set_throttle_zero_flag - set throttle_zero flag from debounced throttle control
// throttle_zero is used to determine if the pilot intends to shut down the motors
// Basically, this signals when we are not flying.  We are either on the ground
// or the pilot has shut down the copter in the air and it is free-falling
void Copter::set_throttle_zero_flag(int16_t throttle_control)
{
    static uint32_t last_nonzero_throttle_ms = 0;
    uint32_t tnow_ms = millis();

    // if not using throttle interlock and non-zero throttle and not E-stopped,
    // or using motor interlock and it's enabled, then motors are running, 
    // and we are flying. Immediately set as non-zero
    if ((!ap.using_interlock && (throttle_control > 0) && !ap.motor_emergency_stop) || (ap.using_interlock && motors->get_interlock())) {
        last_nonzero_throttle_ms = tnow_ms;
        ap.throttle_zero = false;
    } else if (tnow_ms - last_nonzero_throttle_ms > THROTTLE_ZERO_DEBOUNCE_TIME_MS) {
        ap.throttle_zero = true;
    }
}

// pass pilot's inputs to motors library (used to allow wiggling servos while disarmed on heli, single, coax copters)
void Copter::radio_passthrough_to_motors()
{
    motors->set_radio_passthrough(channel_roll->norm_input(),
                                  channel_pitch->norm_input(),
                                  channel_throttle->get_control_in_zero_dz()*0.001,
                                  channel_yaw->norm_input());
}
