#include "dev.h"
#include "tag.h"
#include "ant.h"
#include "port_platform.h"
#include "time_stamp.h"
#include "deca_regs.h"
#include <stdio.h>
#include "trilateration.h"
#include <string.h>
#include "SEGGER_RTT.h"
#include "data_fitting.h"
#include "kf_vhub.h"

vec3d AnchorList[ANCHOR_MAX_NUM];
vec3d tag_best_solution;


extern  int16_t uwb_rssi ;
/* Frames used in the ranging process. See NOTE 2 below. */
uint8 rx_poll_msg[] =  {0x41, 0x88, 0, 0x0, 0xDE, 'W', 'A', 'V', 'E', 0x21, 0, 0};
uint8 tx_resp_msg[] =  {0x41, 0x88, 0, 0x0, 0xDE, 'V', 'E', 'W', 'A', 0x10, 0x02, 0, 0, 0, 0};
uint8 rx_final_msg[] = {0x41, 0x88, 0, 0x0, 0xDE, 'W', 'A', 'V', 'E', 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint8 distance_msg[] = {0x41, 0x88, 0, 0x0, 0xDE, 'W', 'A', 'V', 'E', 0xAA, 0, 0,0, 0, 0};
uint8 tx_poll_msg[] =  {0x41, 0x88, 0, 0x0, 0xDE, 'W', 'A', 'V', 'E', 0x21, 0, 0};
//uint8 tx_poll_msg[] =  {0x41, 0x88, 0, 0x0, 0xDE, 0x12, 0x12, 0x12, 0x12, 0x21, 0, 0};
uint8 rx_resp_msg[] =  {0x41, 0x88, 0, 0x0, 0xDE, 'V', 'E', 'W', 'A', 0x10, 0x02, 0, 0, 0, 0};
uint8 tx_final_msg[] = {0x41, 0x88, 0, 0x0, 0xDE, 'W', 'A', 'V', 'E', 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint8 angle_msg[] =    {0x41, 0x88, 0, 0x0, 0xDE, 'W', 'A', 'V', 'E', 0xFE, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint8 Semaphore_Release[] =    {0x41, 0x88, 0, 0x0, 0xDE, 'W', 'A', 'V', 'E', 0xE0, 0, 0, 0};
uint8 Tag_Statistics[] =                      {0x41, 0x88, 0, 0x0, 0xDE, 'W', 'A', 'V', 'E', 0xE1, 0, 0, 0};
uint8 Master_Release_Semaphore[] =            {0x41, 0x88, 0, 0x0, 0xDE, 'W', 'A', 'V', 'E', 0xE2, 0, 0, 0};
uint8 Tag_Statistics_response[] =             {0x41, 0x88, 0, 0x0, 0xDE, 'W', 'A', 'V', 'E', 0xE3, 0, 0, 0};
uint8 Master_Release_Semaphore_comfirm[] =    {0x41, 0x88, 0, 0x0, 0xDE, 'W', 'A', 'V', 'E', 0xE4, 0, 0, 0};



//    AnchorList[0].x =0.12;
//    AnchorList[0].y =0.34;
//    AnchorList[0].z =0;

//    AnchorList[1].x =0.25;
//    AnchorList[1].y =0;
//    AnchorList[1].z =0;

//    AnchorList[2].x =0;
//    AnchorList[2].y =0;
//    AnchorList[2].z =0;
//    int rx_ant_delay =32880;
//    int index = 0 ;


/* Buffer to store received messages.
 * Its size is adjusted to longest frame that this example code is supposed to handle. */

uint8 rx_buffer[RX_BUF_LEN];

int Anthordistance[ANCHOR_MAX_NUM];
int Anthordistance_count[ANCHOR_MAX_NUM];
int16_t Anthorrssi[ANCHOR_MAX_NUM];
char dist_str[16] = {0};


double final_distance ;
int  first_distance ; 


/* Timestamps of frames transmission/reception.
 * As they are 40-bit wide, we need to define a 64-bit int type to handle them. */
typedef signed long long int64;
typedef unsigned long long uint64;
uint64 poll_rx_ts;
uint64 resp_tx_ts;
uint64 final_rx_ts;

uint64 poll_tx_ts;
uint64 resp_rx_ts;
uint64 final_tx_ts;


#ifdef DWM1000_TEST

void dwm1000_send_data_test(void)
{
    uint8 dest_anthor = 0,frame_len = 0;
    float final_distance = 0;
		while(1)
		{
				for(dest_anthor = 0 ;  dest_anthor<ANCHOR_MAX_NUM; dest_anthor++)
				{
						
						dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
						dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
						/* Write frame data to DW1000 and prepare transmission. See NOTE 7 below. */
						tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
						tx_poll_msg[ALL_MSG_TAG_IDX] = TAG_ID;//基站收到标签的信息，里面有TAG_ID,在基站回复标签的时候，也需要指定TAG_ID,只有TAG_ID一致才做处理

						dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0);
						dwt_writetxfctrl(sizeof(tx_poll_msg), 0);

						/* Start transmission, indicating that a response is expected so that reception is enabled automatically after the frame is sent and the delay
						 * set by dwt_setrxaftertxdelay() has elapsed. */
						dwt_starttx(DWT_START_TX_IMMEDIATE| DWT_RESPONSE_EXPECTED); // DWT_START_TX_IMMEDIATE 

						//GPIO_SetBits(GPIOA,GPIO_Pin_2);
						//TODO
						//000000000000000000000000000000000000000000000deca_sleep(20);
						/* Execute a delay between ranging exchanges. */
						deca_sleep(2);
						//frame_seq_nb++;
				}
		}
		
}

#endif // DWM1000_TEST


/* Frame sequence number, incremented after each transmission. */
uint8 frame_seq_nb = 0;
/* Hold copy of status register state here for reference, so reader can examine it at a breakpoint. */
uint32_t status_reg = 0;
void single_tag_loop(void)
{
	static uint8_t err_cnt = 0;
	
    uint8 dest_anthor = 0,frame_len = 0;
    float final_distance = 0;
    for(dest_anthor = 0 ;  dest_anthor < ANCHOR_MAX_NUM; dest_anthor++)
    {
        dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
        dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
				
        /* Write frame data to DW1000 and prepare transmission. See NOTE 7 below. */
        tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
        tx_poll_msg[ALL_MSG_TAG_IDX] = TAG_ID;//基站收到标签的信息，里面有TAG_ID,在基站回复标签的时候，也需要指定TAG_ID,只有TAG_ID一致才做处理
				
        dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0);
        dwt_writetxfctrl(sizeof(tx_poll_msg), 0);
        
        /* Start transmission, indicating that a response is expected so that reception is enabled automatically after the frame is sent and the delay
         * set by dwt_setrxaftertxdelay() has elapsed. */
        int dwt_start_status_one = dwt_starttx(DWT_START_TX_IMMEDIATE| DWT_RESPONSE_EXPECTED);
				//log_msg("%d aone is %d \r\n",dest_anthor,dwt_start_status_one);
        //GPIO_SetBits(GPIOA,GPIO_Pin_2);
        //TODO
        dwt_rxenable(0);//这个后加的，默认tx后应该自动切换rx，但是目前debug 发现并没有自动打开，这里强制打开rx
        
        /* We assume that the transmission is achieved correctly, poll for reception of a frame or error/timeout. See NOTE 8 below. */
        // dwt_read32bitoffsetreg
				
        while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)))
        {
        };
        //    GPIO_SetBits(GPIOA,GPIO_Pin_1);
        // 此处的引脚拉低需确定具体功能
        //log_msg("s %x \r\n",status_reg );
        if (status_reg & SYS_STATUS_RXFCG)
        {
            /* Clear good RX frame event and TX frame sent in the DW1000 status register. */
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);
						
            /* A frame has been received, read it into the local buffer. */
					  
            frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;  
						
			// log_msg("f %d\r\n",frame_len);
						
            if(frame_len <= RX_BUF_LEN)
            {
                dwt_readrxdata(rx_buffer, frame_len, 0);
            }
														
            #ifdef LOG_TEST
            while (1)  
            {
                    uint32_t deviceid;
                    // reset_DW1000();
                    deviceid = dwt_readdevid();
                    log_dwm1000_init("init  %x   !!\r\n",deviceid);
                    //deca_sleep(50);
            }
            #endif
								
            if(rx_buffer[ALL_MSG_TAG_IDX] != TAG_ID)//检测TAG_ID
            {
                log_msg("continue ! \r\n");
                continue;
            }
            rx_buffer[ALL_MSG_TAG_IDX] = 0;
						
            /* As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
            rx_buffer[ALL_MSG_SN_IDX] = 0;
            //
            if (memcmp(rx_buffer, rx_resp_msg, ALL_MSG_COMMON_LEN) == 0)
            {
                uint32 final_tx_time;
								//log_msg("s %x \r\n",status_reg );
                /* Retrieve poll transmission and response reception timestamp. */
                poll_tx_ts = get_tx_timestamp_u64();
                resp_rx_ts = get_rx_timestamp_u64();
								
                /* Compute final message transmission time. See NOTE 9 below. */
                final_tx_time = (resp_rx_ts + (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
								
                dwt_setdelayedtrxtime(final_tx_time);
								
                /* Final TX timestamp is the transmission time we programmed plus the TX antenna delay. */
                final_tx_ts = (((uint64)(final_tx_time & 0xFFFFFFFE)) << 8) + TX_ANT_DLY;
								//log_msg("final time is %lld \r\n",final_tx_ts);
                /* Write all timestamps in the final message. See NOTE 10 below. */
                #ifndef FAKE_DATA
                final_msg_set_ts(&tx_final_msg[FINAL_MSG_POLL_TX_TS_IDX], poll_tx_ts);
                final_msg_set_ts(&tx_final_msg[FINAL_MSG_RESP_RX_TS_IDX], resp_rx_ts);
                final_msg_set_ts(&tx_final_msg[FINAL_MSG_FINAL_TX_TS_IDX], final_tx_ts);
                #else
                tx_final_msg[FINAL_MSG_POLL_TX_TS_IDX] = 0x0a;
                tx_final_msg[FINAL_MSG_POLL_TX_TS_IDX+1] = 0x0a;
                tx_final_msg[FINAL_MSG_POLL_TX_TS_IDX+2] = 0x0a;
                tx_final_msg[FINAL_MSG_POLL_TX_TS_IDX+3] = 0x0a;								
                    
                tx_final_msg[FINAL_MSG_RESP_RX_TS_IDX] = 0x0b;
                tx_final_msg[FINAL_MSG_RESP_RX_TS_IDX+1] = 0x0b;
                tx_final_msg[FINAL_MSG_RESP_RX_TS_IDX+2] = 0x0b;
                tx_final_msg[FINAL_MSG_RESP_RX_TS_IDX+3] = 0x0b;	
                    
                tx_final_msg[FINAL_MSG_FINAL_TX_TS_IDX] = 0x05;
                tx_final_msg[FINAL_MSG_FINAL_TX_TS_IDX+1] = 0x05;
                tx_final_msg[FINAL_MSG_FINAL_TX_TS_IDX+2] = 0x05;
                tx_final_msg[FINAL_MSG_FINAL_TX_TS_IDX+3] = 0x05;	
                #endif
                /* Write and send final message. See NOTE 7 below. */
                tx_final_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
                tx_final_msg[ALL_MSG_TAG_IDX] = TAG_ID;
                int dwt_start_status = dwt_writetxdata(sizeof(tx_final_msg), tx_final_msg, 0); // 写TX长数据
				//log_msg("w is %d \r\n",dwt_start_status);
                dwt_start_status = dwt_writetxfctrl(sizeof(tx_final_msg), 0);                  // 写TX控制
				//log_msg("a is %d \r\n",dwt_start_status);  

                do
                {
                    err_cnt++;
                    dwt_start_status = dwt_starttx( DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED );// 开始发送数据
                    //log_msg("d is %d err_cnt is %d \r\n",dwt_start_status,err_cnt);
                    //deca_sleep(10);
                }while((DWT_SUCCESS != dwt_start_status) && (err_cnt < 1));
                err_cnt = 0;
								
                //log_queue_hex(tx_final_msg,sizeof(tx_final_msg));

                //dwt_rxenable(0);//
                //log_msg("f%lld\r\n",final_tx_ts );  
                while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR))) // 开始读状态 
                {
                    
                };

                //log_msg("s is%x \r\n",status_reg);
                /* Increment frame sequence number after transmission of the poll message (modulo 256). */
                if (status_reg & SYS_STATUS_RXFCG) // 0x00004000UL
                {
                    /* Clear good/fail RX frame event in the DW1000 status register. */
                    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);
                    /* A frame has been received, read it into the local buffer. */
                    frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
                    if (frame_len <= RX_BUF_LEN)
                    {
                        dwt_readrxdata(rx_buffer, frame_len, 0);
                    }
										
                    if(rx_buffer[ALL_MSG_TAG_IDX] != TAG_ID)
                        continue;
                    rx_buffer[ALL_MSG_TAG_IDX] = 0;

                    /*As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
                    rx_buffer[ALL_MSG_SN_IDX] = 0;

                    if (memcmp(rx_buffer, distance_msg, ALL_MSG_COMMON_LEN) == 0)
                    {
                        // final_distance = rx_buffer[10] + (float)rx_buffer[11]/100;
                        Anthordistance[rx_buffer[12]] +=(rx_buffer[10]*1000 + rx_buffer[11]*10);
                        Anthordistance_count[rx_buffer[12]] ++;

                        Anthorrssi[rx_buffer[12]]  =uwb_rssi;

                        uwb_rssi = (int16_t)(dwGetReceivePower()*100);
                        //log_msg("\n\r an  %d  rssi  %d  d\n\r" , \
                                                            (uint8_t)(rx_buffer[12]) ,uwb_rssi);
                        Anthorrssi[rx_buffer[12]]  =uwb_rssi;
                        {
                            int Anchor_Index = 0;
                            while(Anchor_Index < ANCHOR_MAX_NUM)
                            {
                                if(Anthordistance_count[Anchor_Index] >=ANCHOR_REFRESH_COUNT ) // ANCHOR_REFRESH_COUNT作为算平均数的除数 
                                {
                                    distance_mange();
                                    Anchor_Index = 0;
                                    //clear all
                                    while(Anchor_Index < ANCHOR_MAX_NUM)
                                    {
                                        Anthordistance_count[Anchor_Index] = 0;
                                        Anthordistance[Anchor_Index] = 0;
                                        Anchor_Index++;
                                    }
                                    break;
                                }
                                Anchor_Index++;
                            }
                        }
                    }
                }
                else  
                {
                    /* Clear RX error events in the DW1000 status register. */
                    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
                }
            }
        }
        else
        {
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
        }
        /* Execute a delay between ranging exchanges. */
        // deca_sleep(RNG_DELAY_MS);
       frame_seq_nb++;
    }

}

int filter(int input, int fliter_idx )
{
	
    #define Filter_N 3  //max filter use in this system  滤波的组数 

    #ifdef  USE_TAG
    #define Filter_D 5  //each filter contain "Filter_D" data  窗口长度 
    #else
    #define Filter_D 7 
    #endif
    static int Value_Buf[Filter_N][Filter_D]= {0};
    static int filter_index[Filter_N] = {0};
			
    char count = 0;
    int  sum = 0;
    if(input > 0)
    {
        Value_Buf[fliter_idx][filter_index[fliter_idx]++]=input;
        if(filter_index[fliter_idx] == Filter_D) filter_index[fliter_idx] = 0;

        for(count = 0; count<Filter_D; count++)
        {
            sum += Value_Buf[fliter_idx][count];
        }
        return (int)(sum/Filter_D);
    }
    else
    {
        for(count = 0; count<Filter_D; count++)
        {
            sum += Value_Buf[fliter_idx][count];
        }
        return (int)(sum/Filter_D);
    }
}

void angle_send_to_ble(int distance1, int distance2,int distance3,int rssi1,int rssi2,int rssi3);
void distance_mange(void)
{
	static int mange_index = 0;
    static double  f_Anthordistance[3];
    {
        int Anchor_Index = 0;
        while(Anchor_Index < ANCHOR_MAX_NUM)
        {
            if(Anthordistance_count[Anchor_Index] > 0 )
            {
                Anthordistance[Anchor_Index] =filter((int)(Anthordistance[Anchor_Index]/Anthordistance_count[Anchor_Index]),Anchor_Index);
            }
            Anchor_Index++;
        }
    }
    
    #ifndef SEGGER_SEE_DATA
    f_Anthordistance[0] = (float)Anthordistance[0]/1000;
    f_Anthordistance[1] = (float)Anthordistance[1]/1000;
    f_Anthordistance[2] = (float)Anthordistance[2]/1000;
    log_msg("l[0] is %f l[1] is %f l[2] is %f \r\n",f_Anthordistance[0],f_Anthordistance[1],f_Anthordistance[2]);
    range_calibration(f_Anthordistance);
	log_msg("t[0] is %f t[1] is %f t[2] is %f \r\n",f_Anthordistance[0],f_Anthordistance[1],f_Anthordistance[2]);
    #endif 
    
    Anthordistance[0] = f_Anthordistance[0]*100 ;
    Anthordistance[1] = f_Anthordistance[1]*100 ;
    Anthordistance[2] = f_Anthordistance[2]*100 ;
    
    if(Report_Data == ReportType)
    {
        tag_data_profess_64hz_in_main_loop();
        dev_send_64hz_in_rtc();
        //deca_sleep(30);
    }
    		
    if(first_distance == 1)
    {
         first_distance = 0;
    }
    
    if(Anthordistance_count[0]>0)
    {
        
        //sprintf(dist_str, "an0:%3.2fm", (float)Anthordistance[0]/1000);       
		//SEGGER_RTT_printf(0,"\n\r an0  %s  \n\r" , dist_str );
    }

    if(Anthordistance_count[1]>0)
    {
        //sprintf(dist_str, "an1:%3.2fm", (float)Anthordistance[1]/1000);   
		//SEGGER_RTT_printf(0,"\n\r         an1  %s  \n\r" , dist_str );
    }

    if(Anthordistance_count[2]>0)
    {
        //sprintf(dist_str, "an2:%3.2fm", (float)Anthordistance[2]/1000);   
        //SEGGER_RTT_printf(0,"\n\r                  an2  %s  \n\r" , dist_str );		
    }
}



void distance_mange_anthor(int distance_data,uint8_t dis_index)
{
	static uint32_t  num_index = 0 ;
    {
        //int Anchor_Index = 0;
		Anthordistance[dis_index] = filter(distance_data,dis_index);
    }
		
		//SEGGER_RTT_printf(0,"\n\r distance_mange  %d  index %d \n\r",Anthordistance[0] , num_index++);

    if(first_distance == 1)
    {
        first_distance = 0;
    }
		
    if(dis_index == 0)
    {
			  
        sprintf(dist_str, "an0:%3.2fm", (float)Anthordistance[0]/100);       
			  //SEGGER_RTT_printf(0,"\n\r an0  %s  index %d \n\r",dist_str , num_index++);
    }

    if(dis_index == 1)
    {
        sprintf(dist_str, "an1:%3.2fm", (float)Anthordistance[1]/100); 
    }

    if(dis_index == 2)
    {
        sprintf(dist_str, "an2:%3.2fm", (float)Anthordistance[2]/100);  
    }
}
void USART_putc(char c);

void angle_send_to_ble(int distance1, int distance2,int distance3,int rssi1,int rssi2,int rssi3)
{
    #define BLE_SEND_DATA_LENGHT 16
    static int angle_send_to_ble_countx = 0 ;
    static uint8 diasance12[BLE_SEND_DATA_LENGHT];
    uint8 len = 0;
    
    diasance12[0] = 0xf0;
    diasance12[1] = TAG_ID;//TAG ID

    diasance12[2] = (uint8)(angle_send_to_ble_countx&0xFF);
    diasance12[3] = (uint8)((angle_send_to_ble_countx>>8)&0xFF);

    diasance12[4] = (uint8)((distance1/10)&0xFF);
    diasance12[5] = (uint8)((distance1/10 >>8)&0xFF);

    diasance12[6] =  (uint8)((distance2/10)&0xFF);
    diasance12 [7] =  (uint8)((distance2/10 >>8)&0xFF);

    diasance12[8] =  (uint8)((distance3/10)&0xFF);
    diasance12[9] =  (uint8)((distance3/10 >>8)&0xFF);



	diasance12[10] = (uint8)((rssi1)&0xFF);
	diasance12[11] = (uint8)((rssi1>>8)&0xFF);
	diasance12[12] =  (uint8)((rssi2)&0xFF);
	diasance12[13] =  (uint8)((rssi2>>8)&0xFF);
	diasance12[14] =  (uint8)((rssi3)&0xFF);
	diasance12[15] =  (uint8)((rssi3>>8)&0xFF);
    //if(send_flag)
    {
    //	send_flag=0;
    //	USART_puts((uint8_t*)diasance12,16);
    }//deca_sleep(100);
    angle_send_to_ble_countx++;

}




//**************************************************************//
//distance1 anthor0 <--> TAG  mm
//distance2 anthor1 <--> TAG  mm
//distance3 anthor2 <--> TAG  mm
//**************************************************************//
void compute_angle_send_to_anthor0(int distance1, int distance2,int distance3)
{
    static int framenum = 0 ;
    //location
    {
        uint8 len = 0;
				uint8 diasance12[2];
        angle_msg[LOCATION_FLAG_IDX] = 1;

        angle_msg[LOCATION_INFO_START_IDX + (len++)] = 'm';
        angle_msg[LOCATION_INFO_START_IDX + (len++)] = 'r';

        angle_msg[LOCATION_INFO_START_IDX + (len++)] = 0x02;
        angle_msg[LOCATION_INFO_START_IDX + (len++)] = TAG_ID;//TAG ID

        angle_msg[LOCATION_INFO_START_IDX + (len++)] = (uint8)(framenum&0xFF);
        angle_msg[LOCATION_INFO_START_IDX + (len++)] = (uint8)((framenum>>8)&0xFF);

        angle_msg[LOCATION_INFO_START_IDX + (len++)] = (uint8)((distance1/10)&0xFF);
        angle_msg[LOCATION_INFO_START_IDX + (len++)] = (uint8)((distance1/10 >>8)&0xFF);
				diasance12[0]=(uint8)((distance1/10)&0xFF);
				diasance12[1]=(uint8)((distance1/10 >>8)&0xFF);

        angle_msg[LOCATION_INFO_START_IDX + (len++)] =  (uint8)((distance2/10)&0xFF);
        angle_msg[LOCATION_INFO_START_IDX + (len++)] =  (uint8)((distance2/10 >>8)&0xFF);

        angle_msg[LOCATION_INFO_START_IDX + (len++)] =  (uint8)((distance3/10)&0xFF);
        angle_msg[LOCATION_INFO_START_IDX + (len++)] =  (uint8)((distance3/10 >>8)&0xFF);

        if(ANCHOR_MAX_NUM > 3)
        {
            angle_msg[LOCATION_INFO_START_IDX + (len++)] = (uint8)((Anthordistance[3]/10)&0xFF);
            angle_msg[LOCATION_INFO_START_IDX + (len++)] = (uint8)((Anthordistance[3]/10 >>8)&0xFF);
        }
        else
        {
            angle_msg[LOCATION_INFO_START_IDX + (len++)] = (uint8)((distance1/10)&0xFF);
            angle_msg[LOCATION_INFO_START_IDX + (len++)] = (uint8)((distance1/10 >>8)&0xFF);
        }

        angle_msg[LOCATION_INFO_START_IDX + (len++)] = '\n';
        angle_msg[LOCATION_INFO_START_IDX + (len++)] = '\r';


        angle_msg[LOCATION_INFO_LEN_IDX] = len;
        // MAX LEN
        if(LOCATION_INFO_START_IDX + len -2 >ANGLE_MSG_MAX_LEN)
        {
            while(1);
        }
        // USART_puts((char*)angle_msg,sizeof (angle_msg));
			//	USART_puts((uint8_t*)diasance12,sizeof (diasance12));
    }
    // only anthor0 recive angle message
    angle_msg[ALL_MSG_SN_IDX] = framenum;
    angle_msg[ALL_MSG_TAG_IDX] = TAG_ID;

    dwt_writetxdata(sizeof(angle_msg), angle_msg, 0);
    dwt_writetxfctrl(sizeof(angle_msg), 0);

    /* Start transmission, indicating that a response is expected so that reception is enabled automatically after the frame is sent and the delay
     * set by dwt_setrxaftertxdelay() has elapsed. */
    dwt_starttx(DWT_START_TX_IMMEDIATE );
    while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
    { };

    framenum++;
}



