#include "ant.h"
#include "tag.h"
#include "time_stamp.h"
#include "deca_regs.h"
#include <stdio.h>
#include "trilateration.h"
#include <string.h>
#include "SEGGER_RTT.h"
#include <math.h>
void USART_puts(uint8_t *s,uint8_t len);
anchor0_send_pc_t anchor0_s_p_exam;
uint8 anthor_index = 0;
uint8 tag_index = 0;

int8 frame_len = 0;


int16_t uwb_rssi ;


float calculatePower(float base, float N, uint8_t pulseFrequency) 
{
		float A, corrFac;

    if(DWT_PRF_16M == pulseFrequency) 
		{
        A = 115.72;
        corrFac = 2.3334;
    } 
		else 
		{
        A = 121.74;
        corrFac = 1.1667;
    }

    float estFpPwr = 10.0 * log10(base / (N * N)) - A;

    if(estFpPwr <= -88)
    {
        return estFpPwr;
    } 
    else 
    {
        // approximation of Fig. 22 in user manual for dbm correction
        estFpPwr += (estFpPwr + 88) * corrFac;
    }

    return estFpPwr;
}


float dwGetReceivePower(void) 
{
    dwt_rxdiag_t *diagnostics;
    dwt_readdiagnostics(diagnostics);
    float C = (&diagnostics->stdNoise)[3];
    float N = diagnostics->rxPreamCount;

    float twoPower17 = 131072.0;
    return calculatePower(C * twoPower17, N, config_dwm1000.prf);
}


void Anchor_Array_Init(void)
{
    int anchor_index = 0;
    for(anchor_index = 0; anchor_index < ANCHOR_MAX_NUM; anchor_index++)
    {
        Anthordistance[anchor_index] = 0;
        Anthordistance_count[anchor_index] = 0;
    }
    // sprintf(dist_str, "    ANTHOR:%02X", ANCHOR_IND);
}


static uint8 tx_msg[] = {0x41, 0x88, 0, 0x0, 0xDE, 'W', 'A', 'V', 'E', 0xFE, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};


#ifdef DWM1000_TEST

void dwm1000_receive_data_test(void)
{
		
	  static vec3d use_2d_cal[3] = {{0,0},
																	{0,2.2},
																	{1.8,0}};
	  int anthor_main_index = 0;
		Anchor_Array_Init();
    /* Loop forever initiating ranging exchanges. */

		sprintf(dist_str, "    ANTHOR:%02X", ANCHOR_IND);

    while (1)
    {
        /* Clear reception timeout to start next ranging process. */
        dwt_setrxtimeout(0);
        /* Activate reception immediately. */
        dwt_rxenable(0);

        /* Poll for reception of a frame or error/timeout. See NOTE 7 below. */
        while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)))
        { };

        if (status_reg & SYS_STATUS_RXFCG)
        {
            /* Clear good RX frame event in the DW1000 status register. */
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);

            /* A frame has been received, read it into the local buffer. */
            frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
            if (frame_len <= RX_BUFFER_LEN)
            {
                dwt_readrxdata(rx_buffer, frame_len, 0);
            }
						for(uint8_t i=0;i<=frame_len;i++)
						{
							 log_msg("%02X ",rx_buffer[i]);
						}
						log_msg("\r\n");
						
            /* Check that the frame is a poll sent by "DS TWR initiator" example.
             * As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */

            if(rx_buffer[ALL_MSG_SN_IDX]%ANCHOR_MAX_NUM != ANCHOR_IND)
                continue;

            anthor_index = rx_buffer[ALL_MSG_SN_IDX]%ANCHOR_MAX_NUM;
            tag_index = rx_buffer[ALL_MSG_TAG_IDX];

            rx_buffer[ALL_MSG_SN_IDX] = 0;
            rx_buffer[ALL_MSG_TAG_IDX] = 0;


        }
        else
        {
            /* Clear RX error events in the DW1000 status register. */
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
        }
    }
}
#endif // DWM1000_TEST




void anchor_function_loop(void)
{
	
    static vec3d use_2d_cal[3] =   {{0,0},
                                    {0,2.2},
                                    {1.8,0}};
    static int anthor_main_index = 0;

    
    /* Loop forever initiating ranging exchanges. */
    
    anchor_id_error_label:
    //log_msg("Y\r\n");
    {
        /* Clear reception timeout to start next ranging process. */
        dwt_setrxtimeout(0);
        /* Activate reception immediately. */
        dwt_rxenable(0);

        /* Poll for reception of a frame or error/timeout. See NOTE 7 below. */
        while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)))
        { };

        if (status_reg & SYS_STATUS_RXFCG)
        {
            /* Clear good RX frame event in the DW1000 status register. */
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);

            /* A frame has been received, read it into the local buffer. */
            frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
            if (frame_len <= RX_BUFFER_LEN)
            {
                dwt_readrxdata(rx_buffer, frame_len, 0);
            }
            
            /* Check that the frame is a poll sent by "DS TWR initiator" example.
             * As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
			
            if(rx_buffer[ALL_MSG_SN_IDX]%ANCHOR_MAX_NUM != ANCHOR_IND)
            {
                //log_msg("frame_len is %d \r\n",frame_len);
                //log_queue_hex(rx_buffer,frame_len);
                goto anchor_id_error_label;
            }

            anthor_index = rx_buffer[ALL_MSG_SN_IDX]%ANCHOR_MAX_NUM;
            tag_index = rx_buffer[ALL_MSG_TAG_IDX]; 
            
            rx_buffer[ALL_MSG_SN_IDX] = 0;
            rx_buffer[ALL_MSG_TAG_IDX] = 0;
			
		
            if (memcmp(rx_buffer, rx_poll_msg, ALL_MSG_COMMON_LEN) == 0)
            {
                /* Retrieve poll reception timestamp. */
                poll_rx_ts = get_rx_timestamp_u64();

                /* Set expected delay and timeout for final message reception. */
                dwt_setrxaftertxdelay(RESP_TX_TO_FINAL_RX_DLY_UUS);
                dwt_setrxtimeout(FINAL_RX_TIMEOUT_UUS);

                /* Write and send the response message. See NOTE 9 below.*/
                tx_resp_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
                tx_resp_msg[ALL_MSG_TAG_IDX] = tag_index;
                int dwt_start_status = dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg, 0);
				//log_msg("w is %d \r\n",dwt_start_status); 
                dwt_start_status = dwt_writetxfctrl(sizeof(tx_resp_msg), 0);
				//log_msg("s is %d \r\n",dwt_start_status); 
                dwt_start_status = dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
				//log_msg("a is %d \r\n",dwt_start_status); 
                while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)))
                { };

                //log_msg("a is %x \r\n",status_reg); 
                #ifdef LOG_TEST
                while (1)
                {
                        uint32_t deviceid;
                        // reset_DW1000();
                        deviceid = dwt_readdevid();
                        log_dwm1000_init("init  %x   !!\r\n",deviceid);
                        deca_sleep(50);
                }
								#endif
                if (status_reg & SYS_STATUS_RXFCG)
                {
                    /* Clear good RX frame event and TX frame sent in the DW1000 status register. */
                    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);
                    
                    /* A frame has been received, read it into the local buffer. */
                    frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
                    if (frame_len <= RX_BUF_LEN)
                    {
                        dwt_readrxdata(rx_buffer, frame_len, 0);
                    }
					//log_msg("qq is %x \r\n",status_reg); 					
                    rx_buffer[ALL_MSG_SN_IDX] = 0;
                    if(tag_index != rx_buffer[ALL_MSG_TAG_IDX])
                    {     
                        log_msg("go to two \r\n");
                        goto anchor_id_error_label;
                    }
                    rx_buffer[ALL_MSG_TAG_IDX] = 0;
                    // log_queue_hex(rx_buffer,ALL_MSG_COMMON_LEN);
                    if (memcmp(rx_buffer, rx_final_msg, ALL_MSG_COMMON_LEN) == 0)
                    {
                        double distance;
                        double tof;
                        uint32 poll_tx_ts, resp_rx_ts, final_tx_ts;
                        uint32 poll_rx_ts_32, resp_tx_ts_32, final_rx_ts_32;
                        double Ra, Rb, Da, Db;
                        int64 tof_dtu;
                        // log_msg("qq is %x \r\n",status_reg); 	
                        /* Retrieve response transmission and final reception timestamps. */
                        resp_tx_ts = get_tx_timestamp_u64();
                        final_rx_ts = get_rx_timestamp_u64();

                        /* Get timestamps embedded in the final message. */
                        final_msg_get_ts(&rx_buffer[FINAL_MSG_POLL_TX_TS_IDX], &poll_tx_ts);
                        final_msg_get_ts(&rx_buffer[FINAL_MSG_RESP_RX_TS_IDX], &resp_rx_ts);
                        final_msg_get_ts(&rx_buffer[FINAL_MSG_FINAL_TX_TS_IDX], &final_tx_ts);

                        /* Compute time of flight. 32-bit subtractions give correct answers even if clock has wrapped. See NOTE 10 below. */
                        poll_rx_ts_32 = (uint32)poll_rx_ts;
                        resp_tx_ts_32 = (uint32)resp_tx_ts;
                        final_rx_ts_32 = (uint32)final_rx_ts;
                        Ra = (double)(resp_rx_ts - poll_tx_ts);
                        Rb = (double)(final_rx_ts_32 - resp_tx_ts_32);
                        Da = (double)(final_tx_ts - resp_rx_ts);
                        Db = (double)(resp_tx_ts_32 - poll_rx_ts_32);
                        tof_dtu = (int64)((Ra * Rb - Da * Db) / (Ra + Rb + Da + Db));

                        tof = tof_dtu * DWT_TIME_UNITS;
                        distance = tof * SPEED_OF_LIGHT;
                        distance = distance - dwt_getrangebias(config_dwm1000.chan,(float)distance, config_dwm1000.prf);//距离减去矫正系数
                        //将计算结果发送给TAG
                        int temp = (int)(distance*100);
                        distance_msg[10] = temp/100;
                        // a=x;  //自动类型转换，取整数部分
                        distance_msg[11] = temp%100;  //乘100后对100取余，得到2位小数点后数字
                        distance_msg[12] = anthor_index;

                        distance_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
                        distance_msg[ALL_MSG_TAG_IDX] = tag_index;
                        dwt_writetxdata(sizeof(distance_msg), distance_msg, 0);
                        dwt_writetxfctrl(sizeof(distance_msg), 0);

                        /* Start transmission, indicating that a response is expected so that reception is enabled automatically after the frame is sent and the delay
                         * set by dwt_setrxaftertxdelay() has elapsed. */
                        dwt_starttx(DWT_START_TX_IMMEDIATE );
                        
                        //distance_mange_anthor(temp,anthor_index); /* 这个滤波和显示的函数最好放在发送后的末尾，\
                                                                     以避免影响twr通讯过程中的实时性 */
                        //uwb_rssi = (int16_t)(dwGetReceivePower()*100);
                        //log_msg("\n\r an  %d  rssi  %d  %d\n\r" , \
                                 (uint16_t)(distance*100) ,uwb_rssi, anthor_main_index++);

                    }
                    else
                    {
                        //log_msg("compy error !\r\n");
                    }
										
                }
                else
                {
                    /* Clear RX error events in the DW1000 status register. */
                    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
                }
            }
			
            else if (memcmp(rx_buffer, angle_msg, ALL_MSG_COMMON_LEN) == 0 && ANCHOR_IND == 0)
            {
                if(rx_buffer[LOCATION_FLAG_IDX] == 1)//location infomartion
                {
                    rx_buffer[ALL_MSG_TAG_IDX] = tag_index;
                    memcpy(&anchor0_s_p_exam,&rx_buffer[LOCATION_INFO_START_IDX],rx_buffer[LOCATION_INFO_LEN_IDX]); 
                    //USART_puts((uint8_t *)&anchor0_s_p_exam.anchor0_head_id0,sizeof(anchor0_s_p_exam));

                    //SEGGER_RTT_printf(0,"\n\r    %d     %d    %d  \n\r" , \
                    anchor0_s_p_exam.distance_a0 ,anchor0_s_p_exam.distance_a1 , anchor0_s_p_exam.distance_a2);
                    int distance_array_cal[3] = {anchor0_s_p_exam.distance_a0 ,anchor0_s_p_exam.distance_a1 , anchor0_s_p_exam.distance_a2};
                    //printf(" 1 %f 2 %f 3 %f 4 %f 5 %f 6 %f \r\n",use_2d_cal[0].x,use_2d_cal[0].y,use_2d_cal[1].x,use_2d_cal[1].y,use_2d_cal[2].x,use_2d_cal[2].y); 
                    Th_Location2(use_2d_cal,distance_array_cal);
                }
                else //follow car
                {
                    //putchar(rx_buffer[10]);
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





