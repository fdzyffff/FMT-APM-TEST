#include <APM.h>
#include <firmament.h>

#include "apm_copter_wrapper.h"

// APM_Params_t apm_params = { 0 };

void APM_init(void)                //上电初始化
{
    APM_Copter_Init();
    APM_Copter_init_para();
    APM_Copter_Setup();
}

void APM_init_para(void)           //参数初始化
{
    // APM_Copter_init_para();
    // copter.p1 = apm_params.p1;
}

void APM_loop(void)               //ardupilot主循环
{
    // APM_update_rc();
    // APM_update_para();
    // APM_update_inertial();
    APM_Copter_Main();
}


void APM_update_para(void){
    APM_init_para();
}

// uint32_t APM_millis(void)
// {
//     return systime_now_ms();
// }

// uint64_t APM_micros(void)
// {
//     return systime_now_us();
// }


