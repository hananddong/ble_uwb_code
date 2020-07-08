#ifndef __SEND_TIMER_H__
#define __SEND_TIMER_H__

#include "nrf_drv_timer.h"

extern bool flag_256_Hz;
extern bool flag_64_Hz;
extern bool flag_detection_power;
extern bool flag_16_Hz;

extern uint32_t aun_red_buffer;
extern uint32_t aun_ir_buffer;

#endif  // __SEND_TIMER_H__

