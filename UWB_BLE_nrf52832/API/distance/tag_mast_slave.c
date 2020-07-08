#include "tag_mast_slave.h"
#include "ant.h"
#include "tag.h"
#include "time_stamp.h"
#include "deca_regs.h"
#include <stdio.h>
#include "trilateration.h"
#include "SEGGER_RTT.h"

extern uint8 anthor_index ;
extern uint8 tag_index ;

extern int8 frame_len;

uint8 Semaphore[MAX_SLAVE_TAG];
uint8 Semaphore_Enable = 0 ;

uint8 Waiting_TAG_Release_Semaphore = 0;

void Semaphore_Init(void)
{
    int tag_index = 0 ;
    for(tag_index = 0; tag_index <MAX_SLAVE_TAG; tag_index++)
    {
        Semaphore[tag_index]  = 0;
    }
}

int Sum_Tag_Semaphore_request(void)
{
    int tag_index = 0 ;
    int sum_request = 0;
    for(tag_index = SLAVE_TAG_START_INDEX; tag_index <MAX_SLAVE_TAG; tag_index++)
    {
        sum_request+=Semaphore[tag_index];
    }
    return sum_request;
}


void tag_function_loop(void)
{

     single_tag_loop();//measuer distance between tag and all anthor

}


