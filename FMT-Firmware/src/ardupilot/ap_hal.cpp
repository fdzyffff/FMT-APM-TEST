#include "ap_hal.h"

extern uint32_t millis();
extern uint64_t micro64();

extern int16_t RC_in_data[20];
extern int16_t RC_out_data[20];

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void AP_HAL::info() {
    printf("This APM hal\n");
}

void AP_HAL::update() {
    update_inertial();
    update_rc();
    update_mission();
}

void AP_HAL::update_inertial() {
    ;
}

void AP_HAL::update_rc(void)
{
    ;
}

void AP_HAL::update_mission(void)
{
    ;
}
