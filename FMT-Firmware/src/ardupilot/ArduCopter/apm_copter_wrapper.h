#ifndef APM_COPTER_WRAPPER_H
#define APM_COPTER_WRAPPER_H

// #include "ap_hal.h"

void APM_Copter_Init(void);    //上电初始化
void APM_Copter_Setup(void);   //上电初始化
void APM_Copter_Main(void);    //飞控主循环，不小于400Hz
void APM_Copter_init_para(void);
void APM_Copter_update_para(void);

#endif