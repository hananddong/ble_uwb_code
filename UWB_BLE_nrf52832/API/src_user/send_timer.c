#include "send_timer.h"
#include "blue.h"
#include "nrf_drv_timer.h"


bool flag_256_Hz = false;
bool flag_64_Hz = false;
bool flag_detection_power = false;
bool flag_16_Hz = false;

uint32_t aun_red_buffer = 0;
uint32_t aun_ir_buffer = 0;

const nrfx_timer_t TIMER = NRF_DRV_TIMER_INSTANCE(1);

