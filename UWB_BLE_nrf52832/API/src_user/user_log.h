#ifndef	__USER_LOG_H__
#define __USER_LOG_H__

#include "stdint.h"
#include "string.h"

#include "SEGGER_RTT.h"
#include "SEGGER_RTT_Conf.h"

#define LOG_BUF_LEN                 200

extern uint8_t LogBuf[LOG_BUF_LEN];

#define LOG(format, ...)                do{ sprintf((char*)LogBuf, format, ##__VA_ARGS__);  \
                                            SEGGER_RTT_Write(0, LogBuf, strlen((const char*)LogBuf));   }while(0)

#define log_msg(x, ...)             	    LOG(x, ##__VA_ARGS__)
#define log_ble_evt(x, ...)         	    //LOG(x, ##__VA_ARGS__)
#define log_nus(x, ...)             	    //LOG(x, ##__VA_ARGS__)
#define log_rssi(x, ...)            	    //LOG(x, ##__VA_ARGS__)
#define log_connect_pc(x, ...)      	    //LOG(x, ##__VA_ARGS__)
#define log_cfg(x, ...)             	    LOG(x, ##__VA_ARGS__)
#define log_dbg(x, ...)             	    //LOG(x, ##__VA_ARGS__)
#define log_dbg_ear(x, ...)         	    //LOG(x, ##__VA_ARGS__)

#define log_dbg_dis_cnct(x, ...)    	    //LOG(x, ##__VA_ARGS__)
                                        
#define log_finger_skt(x, ...)      	    //LOG(x, ##__VA_ARGS__)
#define log_dbg_bat(x, ...)         	    //LOG(x, ##__VA_ARGS__)
#define log_dbg_speed(x, ...)       	    //LOG(x, ##__VA_ARGS__)
#define log_dbg_chest_st(x, ...)    	    //LOG(x, ##__VA_ARGS__)
#define log_dbg_chest_ads1292(x, ...)       //LOG(x, ##__VA_ARGS__)

#define log_dbg_uwb_raw(x, ...)             LOG(x, ##__VA_ARGS__)
#define log_dbg_uwb_uart(x, ...)		  //  LOG(x, ##__VA_ARGS__)
#define log_dbg_uwb_send(x, ...)            LOG(x, ##__VA_ARGS__)
#define log_dbg_uwb_int16(x, ...)         //  LOG(x, ##__VA_ARGS__)
#define log_dbg_uwb_loaclation(x, ...)    //  LOG(x, ##__VA_ARGS__)		
#define log_dwm1000_init(x, ...)             LOG(x, ##__VA_ARGS__)
										
										
void log_queue_hex( uint8_t *pointer_que_hex, uint16_t hex_lenght );
										
#endif	// __USER_LOG_H__

