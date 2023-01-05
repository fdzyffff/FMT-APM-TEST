#ifndef AP_HAL_H
#define AP_HAL_H

#include "AP_Math.h"
#include <firmament.h>
#include "APM.h"

#pragma once

#define RC_OUTPUT_MIN_PULSEWIDTH 400
#define RC_OUTPUT_MAX_PULSEWIDTH 2100

#ifndef RC_INPUT_MAX_CHANNELS
#define RC_INPUT_MAX_CHANNELS 18
#endif

/* Define the CH_n names, indexed from 1, if we don't have them already */
#ifndef CH_1
#define CH_1 0
#define CH_2 1
#define CH_3 2
#define CH_4 3
#define CH_5 4
#define CH_6 5
#define CH_7 6
#define CH_8 7
#define CH_9 8
#define CH_10 9
#define CH_11 10
#define CH_12 11
#define CH_13 12
#define CH_14 13
#define CH_15 14
#define CH_16 15
#define CH_17 16
#define CH_18 17
#define CH_NONE 255
#endif

extern uint32_t millis();
extern uint64_t micro64();

struct my_temp_log_t {
    float pos_x;
    float pos_y;
    float pos_z;
    float pos_x_des;
    float pos_y_des;
    float pos_z_des;
    float vel_x_des;  //x速度cm/s
    float vel_y_des;  //y速度cm/s
    float vel_xy_des;  //水平速度cm/s
    float vel_z_des;  //升降速度cm/s
	
    float ang_roll;
    float ang_roll_des;
    float ang_pitch;
    float ang_pitch_des;
    float ang_yaw;
    float ang_yaw_des;

    float rate_roll;
    float rate_roll_des;
    float rate_pitch;
    float rate_pitch_des;
    float rate_yaw;
    float rate_yaw_des;

    float rate_roll_kP;
    float rate_roll_kI;
    float rate_roll_kD;

    float rate_pitch_kP;
    float rate_pitch_kI;
    float rate_pitch_kD;

    float rate_yaw_kP;
    float rate_yaw_kI;
    float rate_yaw_kD;

    int16_t rc1_in;
    int16_t rc2_in;
    int16_t rc3_in;
    int16_t rc4_in;
    int16_t rc5_in;

    int16_t rc1_out;
    int16_t rc2_out;
    int16_t rc3_out;
    int16_t rc4_out;

    int8_t my_flt_mode;
    int16_t throttle_out;
    int16_t roll_out;
    int16_t pitch_out;
    int16_t yaw_out;      
	
    uint16_t current_cmd_id;
    uint8_t land_complete_state;
    int8_t my_home_state;
    struct Location my_home_loc;
    int8_t my_version;
};


class AP_HAL{
public:
    bool get_soft_armed() {return false;}
    uint32_t micros() {return (uint32_t)micro64();}
    void info();
    void update();
    void update_rc();
    void update_mission();
    void update_inertial();
};

extern AP_HAL hal;

#endif
