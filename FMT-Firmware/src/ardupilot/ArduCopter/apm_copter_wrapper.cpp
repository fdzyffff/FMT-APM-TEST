#include "Copter.h"
#include "APM.h"

// extern uint32_t APM_millis(void);
// extern APM_Params_t apm_params;

Copter* copter;
AP_HAL hal;
my_temp_log_t my_temp_log;

// int16_t RC_in_data[20];
// int16_t RC_out_data[20];

uint32_t millis() {return systime_now_ms();}
uint64_t micro64() {return systime_now_us();} 

extern "C" {

void APM_Copter_Init(void)  //飞控初始化
{
    // Copter copter_temp;
    printf("----------------------------------------------------------------------------------------------------\n");
    printf("                                               APM init                                             \n");
    printf("----------------------------------------------------------------------------------------------------\n");
    copter = new Copter();
    hal =*(new AP_HAL());
}

void APM_Copter_Setup(void)  //飞控初始化
{
    copter->setup();
    copter->test_value_p1 = 100.f;
    // memset(RC_in_data, 0, sizeof(RC_in_data));
    // memset(RC_out_data, 0, sizeof(RC_out_data));
    printf("----------------------------------------------------------------------------------------------------\n");
}

void APM_Copter_Main(void)  //飞控主循环，不小于400Hz
{
    // the Copter main loop, includes fast_loop and loops defined in ArduCopter.cpp by AP_scheduler
    copter->loop();
}

void APM_Copter_init_para(void)
{
    FMT_CHECK(param_link_variable(PARAM_GET(APM, USER_TEST_P1), &copter->test_value_p1));
    // copter->g.wp_yaw_behavior = 0;
    // copter->p1 = apm_params.user_test_p1;
}

//     my_temp_log.pos_x = copter.inertial_nav.get_position().x;
//     my_temp_log.pos_y = copter.inertial_nav.get_position().y;
//     my_temp_log.pos_z = copter.inertial_nav.get_position().z;

//     my_temp_log.pos_x_des = copter.get_destination().x;
//     my_temp_log.pos_y_des = copter.get_destination().y;
//     my_temp_log.pos_z_des = copter.get_destination().z;
//     my_temp_log.vel_x_des = copter.pos_control->get_vel_target().x;  //get_vel_target_z()
//     my_temp_log.vel_y_des = copter.pos_control->get_vel_target().y;  //get_vel_target_z()
//     my_temp_log.vel_z_des = copter.pos_control->get_vel_target().z;  //get_vel_target_z()
//     my_temp_log.vel_xy_des = safe_sqrt(copter.pos_control->get_vel_target().x*copter.pos_control->get_vel_target().x + copter.pos_control->get_vel_target().y*copter.pos_control->get_vel_target().y);

//     my_temp_log.ang_roll = degrees(copter.ahrs.roll);
//     my_temp_log.ang_roll_des = copter.attitude_control->get_att_target_euler_cd().x*0.01f;
//     my_temp_log.ang_pitch = degrees(copter.ahrs.pitch);
//     my_temp_log.ang_pitch_des = copter.attitude_control->get_att_target_euler_cd().y*0.01f;
//     my_temp_log.ang_yaw = degrees(copter.ahrs.yaw);
//     if(copter.attitude_control->get_att_target_euler_cd().z<0.0f) {
//         my_temp_log.ang_yaw_des = copter.attitude_control->get_att_target_euler_cd().z*0.01f;
//         my_temp_log.ang_yaw_des+=360.0f;
//     } else{
//         my_temp_log.ang_yaw_des = copter.attitude_control->get_att_target_euler_cd().z*0.01f;
//     }
    
//     my_temp_log.rate_roll = degrees(copter.ahrs.get_gyro_latest().x);
//     my_temp_log.rate_roll_des = degrees(copter.attitude_control->rate_bf_targets().x);
//     my_temp_log.rate_pitch = degrees(copter.ahrs.get_gyro_latest().y);
//     my_temp_log.rate_pitch_des = degrees(copter.attitude_control->rate_bf_targets().y);
//     my_temp_log.rate_yaw = degrees(copter.ahrs.get_gyro_latest().z);
//     my_temp_log.rate_yaw_des = degrees(copter.attitude_control->rate_bf_targets().z);

//     my_temp_log.rate_roll_kP = copter.attitude_control->get_rate_roll_pid().kP();
//     my_temp_log.rate_roll_kI = copter.attitude_control->get_rate_roll_pid().kI();
//     my_temp_log.rate_roll_kD = copter.attitude_control->get_rate_roll_pid().kD();

//     my_temp_log.rate_pitch_kP = copter.attitude_control->get_rate_pitch_pid().kP();
//     my_temp_log.rate_pitch_kI = copter.attitude_control->get_rate_pitch_pid().kI();
//     my_temp_log.rate_pitch_kD = copter.attitude_control->get_rate_pitch_pid().kD();

//     my_temp_log.rate_yaw_kP = copter.attitude_control->get_rate_yaw_pid().kP();
//     my_temp_log.rate_yaw_kI = copter.attitude_control->get_rate_yaw_pid().kI();
//     my_temp_log.rate_yaw_kD = copter.attitude_control->get_rate_yaw_pid().kD();

//     my_temp_log.rc1_in = (int16_t)copter.g2.rc_channels.rc_channel(0)->get_radio_in();
//     my_temp_log.rc2_in = (int16_t)copter.g2.rc_channels.rc_channel(1)->get_radio_in();
//     my_temp_log.rc3_in = (int16_t)copter.g2.rc_channels.rc_channel(2)->get_radio_in();
//     my_temp_log.rc4_in = (int16_t)copter.g2.rc_channels.rc_channel(3)->get_radio_in();
//     my_temp_log.rc5_in = (int16_t)copter.g2.rc_channels.rc_channel(4)->get_radio_in();

//     my_temp_log.rc1_out = (int16_t)hal.rcout.read(0);
//     my_temp_log.rc2_out = (int16_t)hal.rcout.read(1);
//     my_temp_log.rc3_out = (int16_t)hal.rcout.read(2);
//     my_temp_log.rc4_out = (int16_t)hal.rcout.read(3);

//     my_temp_log.my_flt_mode = copter.control_mode;
//     my_temp_log.my_home_state = copter.ap.home_state;
//     my_temp_log.my_home_loc = copter.ahrs.get_home();

//     my_temp_log.throttle_out = (int16_t)(copter.motors->get_throttle()*1000.f);  
//     my_temp_log.roll_out = (int16_t)(copter.motors->get_roll()*1000.f);
//     my_temp_log.pitch_out = (int16_t)(copter.motors->get_pitch()*1000.f);
//     my_temp_log.yaw_out = (int16_t)(copter.motors->get_yaw()*1000.f);

//     my_temp_log.current_cmd_id = copter.copter_current_cmd.id;

//     my_temp_log.land_complete_state = copter.ap.land_complete;
//     my_temp_log.my_version = 1;
// }

// void copter_loop(void)
// {
//     copter.loop();
// }

// bool copter_set_mode(control_mode_t mode)
// {
// //    STABILIZE =     0,  // manual airframe angle with manual throttle
// //    ACRO =          1,  // manual body-frame angular rate with manual throttle
// //    ALT_HOLD =      2,  // manual airframe angle with automatic throttle
// //    AUTO =          3,  // fully automatic waypoint control using mission commands
// //    GUIDED =        4,  // fully automatic fly to coordinate or fly at velocity/direction using GCS immediate commands
// //    LOITER =        5,  // automatic horizontal acceleration with automatic throttle
// //    RTL =           6,  // automatic return to launching point
// //    CIRCLE =        7,  // automatic circular flight with automatic throttle
// //    LAND =          9,  // automatic landing with horizontal position control
// //    DRIFT =        11,  // semi-automous position, yaw and throttle control
// //    SPORT =        13,  // manual earth-frame angular rate control with manual throttle
// //    FLIP =         14,  // automatically flip the vehicle on the roll axis
// //    AUTOTUNE =     15,  // automatically tune the vehicle's roll and pitch gains
// //    POSHOLD =      16,  // automatic position hold with manual override, with automatic throttle
// //    BRAKE =        17,  // full-brake using inertial/GPS system, no pilot input
// //    THROW =        18,  // throw to launch mode using inertial/GPS system, no pilot input
// //    AVOID_ADSB =   19,  // automatic avoidance of obstacles in the macro scale - e.g. full-sized aircraft
// //    GUIDED_NOGPS = 20,  // guided mode but only accepts attitude and altitude
//     return copter.set_mode(mode, MODE_REASON_TX_COMMAND);
// }

// bool copter_set_home_and_lock()
// {
//     return copter.set_home_to_current_location_and_lock();
// }

void copter_init_para(void)
{
    // //roll
    // copter.g2.rc_channels.rc_channel(0)->radio_min = C_st0;
    // copter.g2.rc_channels.rc_channel(0)->radio_trim = C_st1;
    // copter.g2.rc_channels.rc_channel(0)->radio_max = C_st2;
    // //pitch
    // copter.g2.rc_channels.rc_channel(1)->radio_min = C_st0;
    // copter.g2.rc_channels.rc_channel(1)->radio_trim = C_st1;
    // copter.g2.rc_channels.rc_channel(1)->radio_max = C_st2;
    // //throttle
    // copter.g2.rc_channels.rc_channel(2)->radio_min = C_st0;
    // copter.g2.rc_channels.rc_channel(2)->radio_trim = C_st0;//油门没有trim，跟最小保持一致
    // copter.g2.rc_channels.rc_channel(2)->radio_max = C_st2;
    // //yaw
    // copter.g2.rc_channels.rc_channel(3)->radio_min = C_st0;
    // copter.g2.rc_channels.rc_channel(3)->radio_trim = C_st1;
    // copter.g2.rc_channels.rc_channel(3)->radio_max = C_st2;
    // //ch5 fltmode
    // copter.g2.rc_channels.rc_channel(4)->radio_min = C_st0;
    // copter.g2.rc_channels.rc_channel(4)->radio_trim = C_st1;
    // copter.g2.rc_channels.rc_channel(4)->radio_max = C_st2;
    // //ch6 nothing
    // copter.g2.rc_channels.rc_channel(5)->radio_min = C_st0;
    // copter.g2.rc_channels.rc_channel(5)->radio_trim = C_st0;
    // copter.g2.rc_channels.rc_channel(5)->radio_max = C_st2;
    // //ch7 ch7_option
    // copter.g2.rc_channels.rc_channel(6)->radio_min = C_st0;
    // copter.g2.rc_channels.rc_channel(6)->radio_trim = C_st0;
    // copter.g2.rc_channels.rc_channel(6)->radio_max = C_st2;
    // //ch8 ch8_option
    // copter.g2.rc_channels.rc_channel(7)->radio_min = C_st0;
    // copter.g2.rc_channels.rc_channel(7)->radio_trim = C_st0;
    // copter.g2.rc_channels.rc_channel(7)->radio_max = C_st2;

    // copter.g.frame_type = AP_Motors::MOTOR_FRAME_TYPE_X;

    // copter.flight_modes[0] = 0;//STABILIZE
    // copter.flight_modes[1] = 0;//STABILIZE
    // copter.flight_modes[2] = 0;//STABILIZE
    // copter.flight_modes[3] = 0;//STABILIZE
    // copter.flight_modes[4] = 0;//STABILIZE
    // copter.flight_modes[5] = 0;//STABILIZE

    // copter.g.disarm_delay = 0;
}

void copter_update_para(void)
{
    copter_init_para();

    // copter.attitude_control->get_rate_roll_pid().kP(C_RollPara.KR1[Vn]);    //0.12f
    // copter.attitude_control->get_rate_roll_pid().kI(C_RollPara.KRI[Vn]);     //0.08f
    // copter.attitude_control->get_rate_roll_pid().kD(C_RollPara.KRD[Vn]);   //0.008f
    // copter.attitude_control->get_rate_roll_pid().imax(C_RollPara.KR2[Vn]); //0.5f

    // copter.attitude_control->get_rate_pitch_pid().kP(C_PitchPara.KP1[Vn]); //0.12f
    // copter.attitude_control->get_rate_pitch_pid().kI(C_PitchPara.KPI[Vn]);  //0.08f
    // copter.attitude_control->get_rate_pitch_pid().kD(C_PitchPara.KPD[Vn]); // 
    // copter.attitude_control->get_rate_pitch_pid().imax(C_PitchPara.KP2[Vn]); //0.5f

    // copter.attitude_control->get_rate_yaw_pid().kP(C_YawPara.KY1[Vn]);  //0.12f
    // copter.attitude_control->get_rate_yaw_pid().kI(C_YawPara.KYI[Vn]);   //0.08f
    // copter.attitude_control->get_rate_yaw_pid().kD(C_YawPara.KYD[Vn]);  //0.008f
    // copter.attitude_control->get_rate_yaw_pid().imax(C_YawPara.KY2[Vn]); //0.5f

    // copter.attitude_control->get_angle_roll_p().kP(C_RollPara.KRT1[Vn]);   // 4.5f
    // copter.attitude_control->get_angle_pitch_p().kP(C_PitchPara.KPT1[Vn]);   // 4.5f
    // copter.attitude_control->get_angle_yaw_p().kP(C_YawPara.KYT1[Vn]);     // 4.5f

    //copter.aparm.angle_max = C_RollPara.KSlimit_P[Vn]*10;   //DEFAULT_ANGLE_MAX 最大角度  厘度 4500
    // copter.aparm.angle_max = C_YawPara.KClimit_P[Vn]*10;   //DEFAULT_ANGLE_MAX 最大角度  厘度 4500
    // copter.attitude_control->_slew_yaw = AC_ATTITUDE_CONTROL_SLEW_YAW_DEFAULT_CDS;  // 偏航最大变化角速度 厘度/s   2500
    // copter.attitude_control->_accel_roll_max = AC_ATTITUDE_CONTROL_ACCEL_RP_MAX_DEFAULT_CDSS;  //最大角加速度Roll  厘度/s/s  110000.0
    // copter.attitude_control->_accel_pitch_max = AC_ATTITUDE_CONTROL_ACCEL_RP_MAX_DEFAULT_CDSS;  //最大角加速度Pitch  厘度/s/s  110000.0
    // copter.attitude_control->_accel_yaw_max = AC_ATTITUDE_CONTROL_ACCEL_Y_MAX_DEFAULT_CDSS;  // 最大角加速度Yaw  厘度/s/s 12000.0

    // copter.pos_control->_p_pos_z.kP(C_PitchPara.KH1[Vn]);  //ALT_HOLD_P  垂直位置到速度P    1.0
    // copter.pos_control->_p_vel_z.kP(C_PitchPara.KH2[Vn]);        //VEL_Z_P 垂直速度到加速度 5.0f
    // copter.pos_control->_pid_accel_z.kP(C_PitchPara.KH3[Vn]);   // ACCEL_Z_P 垂直加速度到电机P  0.5
    // copter.pos_control->_pid_accel_z.kI(C_PitchPara.KHI[Vn]);    //ACCEL_Z_I 垂直加速度到电机I   1.00f
    // copter.pos_control->_pid_accel_z.kD(C_PitchPara.KHD[Vn]);  //ACCEL_Z_D 垂直加速度到电机D   0.0f
    // copter.pos_control->_p_pos_xy.kP(C_RollPara.KS1[Vn]);   // POS_XY_P 水平位置到速度P   1.0f
    // copter.pos_control->_pi_vel_xy.kP(C_RollPara.KS2[Vn]);   //VEL_XY_P 水平速度到姿态P   1.0f
    // copter.pos_control->_pi_vel_xy.kI(C_RollPara.KSI[Vn]);    //VEL_XY_I 水平速度到姿态I   0.5f

    // copter.wp_nav->_wp_speed_cms = WPNAV_WP_SPEED;   //航点最大速度以及跟飞的最大速度 厘米/s  500.0f
    // copter.wp_nav->_wp_radius_cm = WPNAV_WP_RADIUS;  //到点的判断圆半径  厘米   200
    // //copter.wp_nav->_wp_speed_up_cms = WPNAV_WP_SPEED_UP;   //航点最大上升速度 厘米/s    250.0f
    // copter.wp_nav->_wp_speed_up_cms = ((F32)SysPara.cdHeiP_Climb*10.0f);   //航点最大上升速度 厘米/s    250.0f    
    // //copter.wp_nav->_wp_speed_down_cms = WPNAV_WP_SPEED_DOWN;   //航点最大下降速度 厘米/s    150.0f
    // copter.wp_nav->_wp_speed_down_cms = ((F32)SysPara.cdHeiP*10.0f);   //航点最大下降速度 厘米/s    150.0f
    // copter.wp_nav->_loiter_speed_cms = WPNAV_LOITER_SPEED;  //留待最大水平速度 厘米/s   500
    // copter.wp_nav->_wp_accel_cms = WPNAV_ACCELERATION;    // 航点水平加速度  厘米/s/s     100   改为80
    // copter.wp_nav->_wp_accel_z_cms = WPNAV_WP_ACCEL_Z_DEFAULT;   // 航点垂直加速度  厘米/s/s     100
    // copter.wp_nav->_loiter_jerk_max_cmsss = WPNAV_LOITER_JERK_MAX_DEFAULT;  //留待加加速度 厘米/s/s/s     500
    // copter.wp_nav->_loiter_accel_cmss = WPNAV_LOITER_ACCEL;   //留待加速度/  厘米/s/s     250  WPNAV_LOITER_ACCEL
    // copter.wp_nav->_loiter_accel_min_cmss = WPNAV_LOITER_ACCEL_MIN;   //留待最小加速度/  厘米/s/s      
    // copter.g.pilot_velocity_z_max = PILOT_VELZ_MAX; // 留待最大垂直速度   250

    // copter.g.follow_land_time = 3000;                        //稳定跟飞时间      ms
    // copter.g.follow_land_safe_height = 500.f;                //安全高度    cm
    // copter.g.follow_land_safe_throttle = 0.2f;               //安全油门    0~0.3f
    // copter.g.offset_distance = 200.f;                        //跟飞偏置距离     cm
    // copter.g.first_stage_vel = -100.f;                       //降落第一阶段下降速度    cm
    // copter.g.final_stage_vel = -50.f;                        //降落第二阶段下降速度    cm
    // copter.g.altitude_offset = SysPara.T1F3;                 //跟飞降落过程的高度偏置      cm            向上为正  
    // copter.g.fore_offset = SysPara.T1F1;                     //跟飞降落过程纵向偏置距离     cm           运动过程中，车前偏置为正    车后为负 x
    // copter.g.cross_offset = SysPara.T1F2;                    //跟飞降落过程横向偏置距离     cm           运动过程中，车右偏置为负值    左侧为正值 y  
    // copter.g.stall_high = SysPara.T1F4;                      //熄火高度    cm    默认50cm

    // copter.attitude_control->bf_feedforward(true);
    // copter.attitude_control->use_ff_and_input_shaping(true);
}

}
