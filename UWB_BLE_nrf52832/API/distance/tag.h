#ifndef __TAG_H__
#define __TAG_H__

#include "stdint.h"
#include "time_stamp.h"
#include "trilateration.h"
//#define TX_ANT_DLY 16436
//#define RX_ANT_DLY 16436
#define TX_ANT_DLY 0
#define RX_ANT_DLY 32950

// 选择基站模式还是标签模式


#ifndef  USE_ANTHOR_MODE
    #define TAG     // 定义标签宏
#else
    #define ANTHOR  // 定义基站宏  
#endif


#define TAG_ID 0x06               // 
#define MASTER_TAG 0x06           // 
#define MAX_SLAVE_TAG 0x01
#define SLAVE_TAG_START_INDEX 0x01// 

#define ANCHOR_MAX_NUM 3
#define ANCHOR_IND 1  // 0 1 2
//#define ANCHOR_IND ANCHOR_NUM



#define ANCHOR_REFRESH_COUNT 4



static dwt_config_t config_dwm1000 =
{
    2,               /* Channel number. */
    DWT_PRF_64M,     /* Pulse repetition frequency. */
    DWT_PLEN_1024,   /* Preamble length. */
    DWT_PAC32,       /* Preamble acquisition chunk size. Used in RX only. */
    9,               /* TX preamble code. Used in TX only. */
    9,               /* RX preamble code. Used in RX only. */
    1,               /* Use non-standard SFD (Boolean) */
    DWT_BR_110K,     /* Data rate.  数据速率 */
    DWT_PHRMODE_STD, /* PHY header mode. */
    (1025 + 64 - 32) /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};


/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor.
 * 1 uus = 512 / 499.2 ? and 1 ? = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME 65536

/* Delay between frames, in UWB microseconds. See NOTE 4 below. */
/* This is the delay from Frame RX timestamp to TX reply timestamp used for calculating/setting the DW1000's delayed TX function. This includes the
 * frame length of approximately 2.46 ms with above configuration. */
#define POLL_RX_TO_RESP_TX_DLY_UUS 2600
/* This is the delay from the end of the frame transmission to the enable of the receiver, as programmed for the DW1000's wait for response feature. */
#define RESP_TX_TO_FINAL_RX_DLY_UUS 100
/* Receive final timeout. See NOTE 5 below. */
#define FINAL_RX_TIMEOUT_UUS 5000

/* Delay between frames, in UWB microseconds. See NOTE 4 below. */
/* This is the delay from the end of the frame transmission to the enable of the receiver, as programmed for the DW1000's wait for response feature. */
#define POLL_TX_TO_RESP_RX_DLY_UUS 150
/* This is the delay from Frame RX timestamp to TX reply timestamp used for calculating/setting the DW1000's delayed TX function. This includes the
 * frame length of approximately 2.66 ms with above configuration. */
#define RESP_RX_TO_FINAL_TX_DLY_UUS 4500 //2700 will fail
/* Receive response timeout. See NOTE 5 below. */
#define RESP_RX_TIMEOUT_UUS 3800

/* Length of the common part of the message (up to and including the function code, see NOTE 2 below). */
#define ALL_MSG_COMMON_LEN 10
/* Index to access some of the fields in the frames involved in the process. */
#define ALL_MSG_SN_IDX 2
#define ALL_MSG_TAG_IDX 3
#define FINAL_MSG_POLL_TX_TS_IDX 10
#define FINAL_MSG_RESP_RX_TS_IDX 14
#define FINAL_MSG_FINAL_TX_TS_IDX 18
#define DISTANCE3 0.27

#define ANGLE_MSG_IDX 10
#define LOCATION_FLAG_IDX 11
#define LOCATION_INFO_LEN_IDX 12
#define LOCATION_INFO_START_IDX 13
#define ANGLE_MSG_MAX_LEN 30

#define RX_BUF_LEN 35
extern  uint8 rx_buffer[RX_BUF_LEN];
extern  int Anthordistance[ANCHOR_MAX_NUM]; 
extern  int Anthordistance_count[ANCHOR_MAX_NUM];

extern 	vec3d AnchorList[ANCHOR_MAX_NUM];
extern  vec3d tag_best_solution;
extern uint64 poll_rx_ts;
extern uint64 resp_tx_ts;
extern uint64 final_rx_ts; 

extern uint64 poll_tx_ts;
extern uint64 resp_rx_ts;
extern uint64 final_tx_ts;

extern char dist_str[];

/* Frames used in the ranging process. See NOTE 2 below. */
extern uint8 rx_poll_msg[12] ;
extern uint8 tx_resp_msg[15];
extern uint8 rx_final_msg[24];
extern uint8 distance_msg[15] ;
extern uint8 tx_poll_msg[12];
extern uint8 rx_resp_msg[15];
extern uint8 tx_final_msg[24] ;
extern uint8 angle_msg[] ;
extern uint8 Semaphore_Release[];
extern uint8 Tag_Statistics[] ;
extern uint8 Master_Release_Semaphore[];
extern uint8 Tag_Statistics_response[];
extern uint8 Master_Release_Semaphore_comfirm[];



extern double final_distance ;
extern int  first_distance ; 
extern uint8 frame_seq_nb;
extern uint32_t status_reg;
void dwm1000_send_data_test(void);
void single_tag_loop(void);
void compute_angle_send_to_anthor0(int distance1, int distance2,int distance3);
void distance_mange(void);
void distance_mange_anthor(int distance_data,uint8_t dis_index);

#endif  // __TAG_H__

