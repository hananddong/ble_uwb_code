
#ifndef ant_H
#define ant_H
#include "time_stamp.h"
#include "trilateration.h"

#include "stdint.h"

typedef struct
{
		uint8_t anchor0_head_id0;
	  uint8_t anchor0_head_id1;
	  uint8_t anchor0_head_id3;
		uint8_t anchor0_head_tag_id;
		uint16_t index ;
		uint16_t distance_a0;
		uint16_t distance_a1;
		uint16_t distance_a2;
		uint16_t distance_a3;
		uint8_t  anchor0_tail_n;
		uint8_t  anchor0_tail_r;
	
}anchor0_send_pc_t;


void dwm1000_receive_data_test(void) ;
void Anchor_Array_Init(void);
void anchor_function_loop(void);
float dwGetReceivePower(void) ;
float calculatePower(float base, float N, uint8_t pulseFrequency) ;

#endif

