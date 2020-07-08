#ifndef	__DEV_FINGER_H__
#define __DEV_FINGER_H__

#include "stdint.h"
#include "stdbool.h"

extern void dev_init_finger(void);
extern void tag_data_profess_64hz_in_main_loop_finger(void);
extern void dev_send_64hz_in_rtc_finger(void);
extern bool uart_rx_handle_finger(uint8_t* p_rx, uint16_t len);

#endif	// __DEV_FINGER_H__

