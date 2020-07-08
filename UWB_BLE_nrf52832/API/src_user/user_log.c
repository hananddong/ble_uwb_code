#include "user_log.h"

uint8_t LogBuf[LOG_BUF_LEN] = {0};


void log_queue_hex( uint8_t *pointer_que_hex, uint16_t hex_lenght )
{
		for(int i=0 ;i<hex_lenght;i++)
		{
				log_msg("%02X ",*pointer_que_hex);
				pointer_que_hex++;
		}
		log_msg("\r\n");

}
