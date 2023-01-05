// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file   AP_MotorsHeli_Dual.h
/// @brief  Motor control class for dual heli (tandem or transverse)
/// @author Fredrik Hedberg

#ifndef __AP_MOTORS_HELI_DUAL_H__
#define __AP_MOTORS_HELI_DUAL_H__

#include "AP_Common.h"
#include "AP_Math.h"
#include "RC_Channel.h"

#include "AP_MotorsHeli.h"
#include "AP_MotorsHeli_RSC.h"

// servo position defaults
#define AP_MOTORS_HELI_DUAL_SERVO1_POS               -60
#define AP_MOTORS_HELI_DUAL_SERVO2_POS                60
#define AP_MOTORS_HELI_DUAL_SERVO3_POS               180
#define AP_MOTORS_HELI_DUAL_SERVO4_POS               -60
#define AP_MOTORS_HELI_DUAL_SERVO5_POS                60
#define AP_MOTORS_HELI_DUAL_SERVO6_POS               180

// rsc function output channel
#define AP_MOTORS_HELI_DUAL_RSC                     CH_8

// tandem modes
#define AP_MOTORS_HELI_DUAL_MODE_TANDEM                0 // tandem mode (rotors front and aft)
#define AP_MOTORS_HELI_DUAL_MODE_TRANSVERSE            1 // transverse mode (rotors side by side)

// default differential-collective-pitch scaler
#define AP_MOTORS_HELI_DUAL_DCP_SCALER             0.25f

// maximum number of swashplate servos
#define AP_MOTORS_HELI_DUAL_NUM_SWASHPLATE_SERVOS    6

// default collective min, max and midpoints for the rear swashplate
#define AP_MOTORS_HELI_DUAL_COLLECTIVE2_MIN 1250
#define AP_MOTORS_HELI_DUAL_COLLECTIVE2_MAX 1750
#define AP_MOTORS_HELI_DUAL_COLLECTIVE2_MID 1500

/// @class AP_MotorsHeli_Dual
class AP_MotorsHeli_Dual : public AP_MotorsHeli {
public:
    // constructor
    AP_MotorsHeli_Dual(uint16_t loop_rate,
                       uint16_t speed_hz = AP_MOTORS_HELI_SPEED_DEFAULT) :
        AP_MotorsHeli(loop_rate, speed_hz),
        _collective2_min(AP_MOTORS_HELI_DUAL_COLLECTIVE2_MIN),
        _collective2_max(AP_MOTORS_HELI_DUAL_COLLECTIVE2_MAX),
        _collective2_mid(AP_MOTORS_HELI_DUAL_COLLECTIVE2_MID),
        _servo1_pos(AP_MOTORS_HELI_DUAL_SERVO1_POS),
        _servo2_pos( AP_MOTORS_HELI_DUAL_SERVO2_POS),
        _servo3_pos( AP_MOTORS_HELI_DUAL_SERVO3_POS),
        _servo4_pos(AP_MOTORS_HELI_DUAL_SERVO4_POS),
        _servo5_pos(AP_MOTORS_HELI_DUAL_SERVO5_POS),
        _servo6_pos(AP_MOTORS_HELI_DUAL_SERVO6_POS),
        _swash1_phase_angle(0),
        _swash2_phase_angle(0),
        _dual_mode(AP_MOTORS_HELI_DUAL_MODE_TANDEM),
        _dcp_scaler(AP_MOTORS_HELI_DUAL_DCP_SCALER),
        _dcp_yaw_effect(0),
        _yaw_scaler(1.0f),
        _rotor(SRV_Channel::k_heli_rsc, AP_MOTORS_HELI_DUAL_RSC)
    {
        ;
    };


    // set_update_rate - set update rate to motors
    void set_update_rate( uint16_t speed_hz ) override;

    // enable - starts allowing signals to be sent to motors
    void enable() override;

    // output_test - spin a motor at the pwm value specified
    void output_test(uint8_t motor_seq, int16_t pwm) override;

    // set_desired_rotor_speed - sets target rotor speed as a number from 0 ~ 1000
    void set_desired_rotor_speed(float desired_speed) override;

    // get_estimated_rotor_speed - gets estimated rotor speed as a number from 0 ~ 1000
    float get_main_rotor_speed() const  override { return _rotor.get_rotor_speed(); }

    // get_desired_rotor_speed - gets target rotor speed as a number from 0 ~ 1000
    float get_desired_rotor_speed() const  override { return _rotor.get_rotor_speed(); }

    // rotor_speed_above_critical - return true if rotor speed is above that critical for flight
    bool rotor_speed_above_critical() const  override { return _rotor.get_rotor_speed() > _rotor.get_critical_speed(); }

    // calculate_scalars - recalculates various scalars used
    void calculate_scalars() override;

    // calculate_armed_scalars - recalculates scalars that can change while armed
    void calculate_armed_scalars() override;

    // get_motor_mask - returns a bitmask of which outputs are being used for motors or servos (1 means being used)
    uint16_t get_motor_mask() override;

    // has_flybar - returns true if we have a mechical flybar
    bool has_flybar() const  override { return AP_MOTORS_HELI_NOFLYBAR; }

    // supports_yaw_passthrought - returns true if we support yaw passthrough
    bool supports_yaw_passthrough() const  override { return false; }

    // servo_test - move servos through full range of movement
    void servo_test() override;

    // parameters
    int16_t        _collective2_min;               // Lowest possible servo position for the rear swashplate
    int16_t        _collective2_max;               // Highest possible servo position for the rear swashplate
    int16_t        _collective2_mid;               // Swash servo position corresponding to zero collective pitch for the rear swashplate (or zero lift for Asymmetrical blades)
    int16_t        _servo1_pos;                    // angular location of swash servo #1
    int16_t        _servo2_pos;                    // angular location of swash servo #2
    int16_t        _servo3_pos;                    // angular location of swash servo #3
    int16_t        _servo4_pos;                    // angular location of swash servo #4
    int16_t        _servo5_pos;                    // angular location of swash servo #5
    int16_t        _servo6_pos;                    // angular location of swash servo #6
    int16_t        _swash1_phase_angle;            // phase angle correction for 1st swash.
    int16_t        _swash2_phase_angle;            // phase angle correction for 2nd swash.
    int8_t         _dual_mode;                     // which dual mode the heli is
    float          _dcp_scaler;                    // scaling factor applied to the differential-collective-pitch
    float          _dcp_yaw_effect;                // feed-forward compensation to automatically add yaw input when differential collective pitch is applied.
    float          _yaw_scaler;                    // scaling factor applied to the yaw mixing

    SRV_Channel    *_swash_servo_1;
    SRV_Channel    *_swash_servo_2;
    SRV_Channel    *_swash_servo_3;
    SRV_Channel    *_swash_servo_4;
    SRV_Channel    *_swash_servo_5;
    SRV_Channel    *_swash_servo_6;


protected:

    // init_outputs
    bool init_outputs () override;

    // update_motor_controls - sends commands to motor controllers
    void update_motor_control(RotorControlState state) override;

    // calculate_roll_pitch_collective_factors - calculate factors based on swash type and servo position
    void calculate_roll_pitch_collective_factors () override;

    // move_actuators - moves swash plate to attitude of parameters passed in
    void move_actuators(float roll_out, float pitch_out, float coll_in, float yaw_out)  override;

    //  objects we depend upon
    AP_MotorsHeli_RSC           _rotor;             // main rotor controller

    // internal variables
    float _oscillate_angle = 0.0f;                  // cyclic oscillation angle, used by servo_test function
    float _servo_test_cycle_time = 0.0f;            // cycle time tracker, used by servo_test function
    float _collective_test = 0.0f;                  // over-ride for collective output, used by servo_test function
    float _roll_test = 0.0f;                        // over-ride for roll output, used by servo_test function
    float _pitch_test = 0.0f;                       // over-ride for pitch output, used by servo_test function


    // internal variables
    float           _collective2_mid_pct = 0.0f;      // collective mid parameter value for rear swashplate converted to 0 ~ 1 range
    float           _rollFactor[AP_MOTORS_HELI_DUAL_NUM_SWASHPLATE_SERVOS];
    float           _pitchFactor[AP_MOTORS_HELI_DUAL_NUM_SWASHPLATE_SERVOS];
    float           _collectiveFactor[AP_MOTORS_HELI_DUAL_NUM_SWASHPLATE_SERVOS];
    float           _yawFactor[AP_MOTORS_HELI_DUAL_NUM_SWASHPLATE_SERVOS];
};

#endif  // AP_MotorsHeli_Dual
