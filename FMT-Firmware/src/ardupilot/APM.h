#ifndef APM_H
#define APM_H

#include <firmament.h>

// modify for apm interface
#ifdef __cplusplus
extern "C" {
#endif

#define _EXT_DTCM0      __attribute__((section(".cf_dtcm0_ekf2")))
#define _EXT_ITCM1      __attribute__((section(".cf_ext_itcm1")))
#define _EXT_DTCM1      __attribute__((section(".dtcm1")))
#define _EXT_DTCM1_ROD  __attribute__((section(".dtcm1_rodata")))
#define _EXT_DTCM1_BSS  __attribute__((section(".dtcm1_bss")))

#ifndef DEFINED_TYPEDEF_FOR_Mission_Data_Bus_
    #define DEFINED_TYPEDEF_FOR_Mission_Data_Bus_

typedef struct {
    uint32_t timestamp;
    uint16_t valid_items;
    uint16_t reserved;

    /* Start from 0 */
    uint16_t seq[8];
    uint16_t command[8];
    uint8_t  frame[8];
    uint8_t  current[8];
    uint8_t  autocontinue[8];
    uint8_t  mission_type[8];
    float    param1[8];
    float    param2[8];
    float    param3[8];
    float    param4[8];
    int32_t  x[8];
    int32_t  y[8];
    float    z[8];
} Mission_Data_Bus;

#endif

#ifndef DEFINED_TYPEDEF_FOR_FMS_Out_Bus_
    #define DEFINED_TYPEDEF_FOR_FMS_Out_Bus_

typedef struct {
    uint32_t  timestamp;    //   ms  fms output timestamp
    float     p_cmd;        //   rad/s   roll rate command in body frame
    float     q_cmd;        //   rad/s   pitch rate command in body frame
    float     r_cmd;        //   rad/s   yaw rate command in body frame
    float     phi_cmd;      //   rad     roll command in body frame
    float     theta_cmd;    //   rad     pitch command in body frame
    float     psi_rate_cmd; //   rad/s   yaw rate command in body frame
    float     u_cmd;        //   m/s     velocity x command in control frame
    float     v_cmd;        //   m/s     velocity y command in control frame
    float     w_cmd;        //   m/s     velocity z command in control frame
    uint32_t  throttle_cmd; //   [1000 2000]     throttle command
    uint16_t  actuator_cmd[16];//    [1000 2000]     actuator command
    uint8_t   status;       //   enum VehicleStatus  vehicle status: 0: None, 1: Disarm, 2: Standby, 3: Arm, 
    uint8_t   state;        //   enum VehicleState   vehicle state: 0: None, 1: Disarm, 2: Standby, 3: Offboard, 4: Mission, 5: InvalidAutoMode, 6: Hold, 7: Acro, 8: Stabilize, 9: Altitude, 10: Position, 11: InvalidAssistMode, 12: Manual, 13: InvalidManualMode, 14: InvalidArmMode, 15: Land, 16: Return, 17: Takeoff
    uint8_t   ctrl_mode;    //   enum ControlMode    control mode: 0: None, 1: Manual, 2: Acro, 3: Stabilize, 4: ALTCTL, 5: POSCTL
    uint8_t   reset;         //   enum PilotMode  pilot mode: 0: None, 1: Manual, 2: Acro, 3: Stabilize, 4: Altitude, 5: Position, 6: Mission, 7: Offboard
    uint8_t   mode;        //   reset controller
    uint8_t   reserved1;
    uint8_t   wp_consume;   //  consumed waypoints
    uint8_t   wp_current;   //  current waypoint
} FMS_Out_Bus;

#endif

#ifndef DEFINED_TYPEDEF_FOR_Control_Out_Bus_
    #define DEFINED_TYPEDEF_FOR_Control_Out_Bus_

typedef struct {
    uint32_t timestamp;
    uint16_t actuator_cmd[16];
} Control_Out_Bus;

#endif

// #ifndef DEFINED_TYPEDEF_FOR_IMU_Bus_
//     #define DEFINED_TYPEDEF_FOR_IMU_Bus_

// typedef struct {
//     uint32_t timestamp;
//     float gyr_x;
//     float gyr_y;
//     float gyr_z;
//     float acc_x;
//     float acc_y;
//     float acc_z;
// } IMU_Bus;

// #endif

// #ifndef DEFINED_TYPEDEF_FOR_MAG_Bus_
//     #define DEFINED_TYPEDEF_FOR_MAG_Bus_

// typedef struct {
//     uint32_t timestamp;
//     float mag_x;
//     float mag_y;
//     float mag_z;
// } MAG_Bus;

// #endif

// #ifndef DEFINED_TYPEDEF_FOR_Barometer_Bus_
//     #define DEFINED_TYPEDEF_FOR_Barometer_Bus_

// typedef struct {
//     uint32_t timestamp;
//     float pressure;
//     float temperature;
// } Barometer_Bus;

// #endif

// #ifndef DEFINED_TYPEDEF_FOR_GPS_uBlox_Bus_
//     #define DEFINED_TYPEDEF_FOR_GPS_uBlox_Bus_

// typedef struct {
//     uint32_t timestamp;
//     uint32_t iTOW;
//     uint16_t year;
//     uint8_t month;
//     uint8_t day;
//     uint8_t hour;
//     uint8_t min;
//     uint8_t sec;
//     uint8_t valid;
//     uint32_t tAcc;
//     int32_t nano;
//     uint8_t fixType;
//     uint8_t flags;
//     uint8_t reserved1;
//     uint8_t numSV;
//     int32_t lon;
//     int32_t lat;
//     int32_t height;
//     int32_t hMSL;
//     uint32_t hAcc;
//     uint32_t vAcc;
//     int32_t velN;
//     int32_t velE;
//     int32_t velD;
//     int32_t gSpeed;
//     int32_t heading;
//     uint32_t sAcc;
//     uint32_t headingAcc;
//     uint16_t pDOP;
//     uint16_t reserved2;
// } GPS_uBlox_Bus;

// #endif

// #ifndef DEFINED_TYPEDEF_FOR_Rangefinder_Bus_
//     #define DEFINED_TYPEDEF_FOR_Rangefinder_Bus_

// typedef struct {
//     uint32_t timestamp;
//     float distance_m;
// } Rangefinder_Bus;

// #endif

// #ifndef DEFINED_TYPEDEF_FOR_Optical_Flow_Bus_
//     #define DEFINED_TYPEDEF_FOR_Optical_Flow_Bus_

// typedef struct {
//     uint32_t timestamp;
//     float vx;
//     float vy;
//     uint32_t valid;
// } Optical_Flow_Bus;

// #endif

#ifndef DEFINED_TYPEDEF_FOR_INS_Out_Bus_
    #define DEFINED_TYPEDEF_FOR_INS_Out_Bus_

typedef struct {
    uint32_t timestamp;
    float phi;
    float theta;
    float psi;
    float quat[4];
    float p;
    float q;
    float r;
    float ax;
    float ay;
    float az;
    float vn;
    float ve;
    float vd;
    float reserved;
    double lat;
    double lon;
    double alt;
    double lat_0;
    double lon_0;
    double alt_0;
    float x_R;
    float y_R;
    float h_R;
    float h_AGL;
    uint32_t flag;
    uint32_t status;
} INS_Out_Bus;

#endif

// #ifndef DEFINED_TYPEDEF_FOR_Pilot_cmd_Bus_
//     #define DEFINED_TYPEDEF_FOR_Pilot_cmd_Bus_

// typedef struct {
//     uint32_t timestamp;
//     float phi;
//     float theta;
//     float psi;
//     float quat[4];
//     float p;
//     float q;
//     float r;
//     float ax;
//     float ay;
//     float az;
//     float vn;
//     float ve;
//     float vd;
//     float reserved;
//     double lat;
//     double lon;
//     double alt;
//     double lat_0;
//     double lon_0;
//     double alt_0;
//     float x_R;
//     float y_R;
//     float h_R;
//     float h_AGL;
//     uint32_t flag;
//     uint32_t status;
// } Pilot_Cmd_Bus;

// #endif

// parameters
// typedef struct {
//     float user_test_p1;             //test parameter p1
// } APM_Params_t;

// typedef struct {
//     int16_t radio_in[16];
// } my_radio_in;


#ifndef DEFINED_TYPEDEF_FOR_GCS_Cmd_Bus_
    #define DEFINED_TYPEDEF_FOR_GCS_Cmd_Bus_

typedef struct {
    uint32_t timestamp;
    uint32_t mode;

    /* Operation channel 1 */
    uint32_t cmd_1;

    /* Operation channel 2 */
    uint32_t cmd_2;
} GCS_Cmd_Bus;

#endif

// enum defination
#ifndef DEFINED_TYPEDEF_FOR_VehicleStatus_
    #define DEFINED_TYPEDEF_FOR_VehicleStatus_

/* enumeration of vehicle status */
typedef enum {
    VehicleStatus_None = 0, /* Default value */
    VehicleStatus_Disarm,
    VehicleStatus_Standby,
    VehicleStatus_Arm
} VehicleStatus;

#endif

#ifndef DEFINED_TYPEDEF_FOR_NAV_Cmd_
    #define DEFINED_TYPEDEF_FOR_NAV_Cmd_

/* enumeration of navigation command */
typedef enum {
    NAV_Cmd_None = 0, /* Default value */
    NAV_Cmd_Waypoint = 16,
    NAV_Cmd_Return = 20,
    NAV_Cmd_Land,
    NAV_Cmd_Takeoff
} NAV_Cmd;

#endif


#ifndef DEFINED_TYPEDEF_FOR_FMS_Cmd_
    #define DEFINED_TYPEDEF_FOR_FMS_Cmd_

/* enumeration of FMS command */
typedef enum {
    FMS_Cmd_None = 0, /* Default value */
    FMS_Cmd_PreArm = 1000,
    FMS_Cmd_Arm,
    FMS_Cmd_Disarm,
    FMS_Cmd_Takeoff,
    FMS_Cmd_Land,
    FMS_Cmd_Return,
    FMS_Cmd_Pause,
    FMS_Cmd_Continue
} FMS_Cmd;

#endif

#ifndef DEFINED_TYPEDEF_FOR_PilotMode_
    #define DEFINED_TYPEDEF_FOR_PilotMode_

/* enumeration of pilot mode */
typedef enum {
    PilotMode_None = 0, /* Default value */
    PilotMode_Manual,
    PilotMode_Acro,
    PilotMode_Stabilize,
    PilotMode_Altitude,
    PilotMode_Position,
    PilotMode_Mission,
    PilotMode_Offboard
} PilotMode;

#endif

#ifndef DEFINED_TYPEDEF_FOR_VehicleState_
    #define DEFINED_TYPEDEF_FOR_VehicleState_

/* enumeration to track active leaf state of FMS/FMS State Machine/Vehicle */
typedef enum {
    VehicleState_None = 0, /* Default value */
    VehicleState_Disarm,
    VehicleState_Standby,
    VehicleState_Offboard,
    VehicleState_Mission,
    VehicleState_InvalidAutoMode,
    VehicleState_Hold,
    VehicleState_Acro,
    VehicleState_Stabilize,
    VehicleState_Altitude,
    VehicleState_Position,
    VehicleState_InvalidAssistMode,
    VehicleState_Manual,
    VehicleState_InValidManualMode,
    VehicleState_InvalidArmMode,
    VehicleState_Land,
    VehicleState_Return,
    VehicleState_Takeoff
} VehicleState;

#endif

extern uint8_t apm_pilot_cmd_updated;
extern uint8_t apm_gcs_cmd_updated;
extern uint8_t apm_mission_data_updated;
extern uint8_t apm_pilot_cmd_log;
extern uint8_t apm_gcs_cmd_log;
extern uint8_t apm_mission_data_log;

extern int16_t rcChannel_msg[16];
extern INS_Out_Bus ins_out_msg;
extern Mission_Data_Bus mission_data_msg;
extern FMS_Out_Bus fms_out_msg;
extern Control_Out_Bus control_out_msg;
extern GCS_Cmd_Bus gcs_cmd_msg;
extern Control_Out_Bus control_out_msg;

void APM_init(void);                //上电初始化
void APM_init_para(void);           //参数初始化
void APM_loop(void);                //ardupilot主循环
// void APM_update_para(void);      //参数更新

void APM_update_para(void);
void APM_update_rc(void);
void APM_update_inertial(void);

// extern APM_Params_t apm_params;

#ifdef __cplusplus
}
#endif

#endif