#ifndef	__DEV_EAR_H__
#define __DEV_EAR_H__

#include "stdint.h"

extern void dev_init_ear(void);
extern void tag_data_profess_64hz_in_main_loop_ear(void);
extern void dev_send_16hz_in_rtc_ear(void);

#endif	// __DEV_EAR_H__

