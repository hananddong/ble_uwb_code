#ifndef	__DEV_CHEST_H__
#define __DEV_CHEST_H__

#include "stdint.h"
#include "stdbool.h"

extern void dev_init_dwm1000(void);
extern void tag_data_profess_64hz_in_main_loop_dwm1000(void);
extern void dev_sample_256hz_in_main_loop_chest(void);
extern void dev_send_64hz_in_rtc_dwm1000(void);
extern bool uart_rx_handle_dwm1000(uint8_t* p_rx, uint16_t len);

#endif	// __DEV_CHEST_H__

