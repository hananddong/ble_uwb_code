#ifndef TAG_MAST_SLAVE_H
#define TAG_MAST_SLAVE_H
#include "time_stamp.h"
#include "trilateration.h"


extern uint8 Semaphore_Enable;

extern uint8 Waiting_TAG_Release_Semaphore;
void tag_function_loop(void);


#endif

