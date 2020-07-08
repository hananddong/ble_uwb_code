
#ifndef time_stamp_H
#define time_stamp_H
#include "deca_device_api.h"

#define FINAL_MSG_TS_LEN 4

typedef signed long long int64;
typedef unsigned long long uint64;
uint64 get_tx_timestamp_u64(void);
uint64 get_rx_timestamp_u64(void);
void final_msg_get_ts(const uint8 *ts_field, uint32 *ts);
void final_msg_set_ts(uint8 *ts_field, uint64 ts);

#endif

