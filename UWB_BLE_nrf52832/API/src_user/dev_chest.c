#include "dev_chest.h"

#include "blue.h"
#include "dev.h"
#include "battery.h"
#include "trilateration.h"
#include "drv_ads1292.h"
#include "user_log.h"
#include "dw1001_dev.h"
#include "deca_device_api.h"
#include "port_platform.h"
#include "boards.h"
#include "tag.h"
//-----------------dw1000----------------------------

//static dwt_config_t config = {
//    5,                /* Channel number. */
//    DWT_PRF_64M,      /* Pulse repetition frequency. */
//    DWT_PLEN_128,     /* Preamble length. Used in TX only. */
//    DWT_PAC8,         /* Preamble acquisition chunk size. Used in RX only. */
//    10,               /* TX preamble code. Used in TX only. */
//    10,               /* RX preamble code. Used in RX only. */
//    0,                /* 0 to use standard SFD, 1 to use non-standard SFD. */
//    DWT_BR_6M8,       /* Data rate. */
//    DWT_PHRMODE_STD,  /* PHY header mode. */
//    (129 + 8 - 8)     /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
//};

/* Preamble timeout, in multiple of PAC size. See NOTE 3 below. */
#define PRE_TIMEOUT 1000

/* Delay between frames, in UWB microseconds. See NOTE 1 below. */
//#define POLL_TX_TO_RESP_RX_DLY_UUS 100 

/*Should be accurately calculated during calibration*/

//--------------dw1000---end---------------



typedef struct
{
    bool PB_Pressed;
    uint16_t temperature;
}DevChest_ST_t;

DevChest_ST_t DevChest_ST;
bool Got_Chest_ST_Uart_Data = false;

void chest_st_cmd_start_sensor_data(void);

void dev_init_dwm1000(void)
{
    #define   DWT_LED_STATUS   1          // 写1使能dwm1000led，写0失能dwm1000led 
    
	nrf_gpio_cfg_input(DW1000_IRQ, NRF_GPIO_PIN_NOPULL); 		 // Setup DW1000 IRQ pin
	reset_DW1000(); 
	port_set_dw1000_slowrate();	                                 // Set SPI clock to 2MHz 
	log_dwm1000_init("spi_low_init!!\r\n");
	int dwt_loaducode_initialise = dwt_initialise(DWT_LOADUCODE);// Init the DW1000
    if (dwt_loaducode_initialise == DWT_ERROR)                   // DWT_SUCCESS  DWT_ERROR
    {
        uint32_t deviceid; // Init of DW1000 Failed
        reset_DW1000();
        while (1)
        {
            deviceid = dwt_readdevid();
            log_dwm1000_init("init_fail  %x   !!\r\n",deviceid);
            nrf_delay_ms(50);
        }
    }
	log_dwm1000_init("spi_low_init!!  %x  \r\n",dwt_loaducode_initialise);
    port_set_dw1000_fastrate();                                    // port_set_SPI to 8MHz clock
    #ifdef  FAST_SPI_TEST
    if (dwt_loaducode_initialise == DWT_SUCCESS)  //DWT_SUCCESS  DWT_ERROR
    {
        //Init of DW1000 Failed
        while (1)
        {
            uint32_t deviceid;
            reset_DW1000();
            deviceid = dwt_readdevid();
            log_dwm1000_init("success  %x   !!\r\n",deviceid);
            //nrf_delay_ms(50);
        }
    }
    #endif
    dwt_configure(&config_dwm1000);                         // Configure DW1000.
    dwt_setleds(DWT_LED_STATUS);                                         // 设置led灯闪烁 
    dwt_setrxantennadelay(RX_ANT_DLY);                      // Apply default antenna delay value. See NOTE 2 below. 
    dwt_settxantennadelay(TX_ANT_DLY);
    /* Set preamble timeout for expected frames. See NOTE 3 below. */
    //dwt_setpreambledetecttimeout(0); // PRE_TIMEOUT
    /* Set expected response's delay and timeout. 
    * As this example only handles one incoming frame with always the same delay and timeout, those values can be set here once for all. */
    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
    // dwt_setrxtimeout(65000);                             // Maximum value timeout with DW1000 is 65ms  
    battery_check_init();
    power_en_ctrl_start();
    
}


typedef struct vec3d vec3d;
double distanceArray[3];
vec3d report;
uint8_t count_x=0;
uint16_t count_x1=0;

void process_data()
{
    
    uint8_t LB, HB;
    uint16_t  d0, d1, d2;
	int16_t r0,r1,r2;

	log_dbg_uwb_raw(    "[d0:%d][d1:%d][d2:% 4d]\r\n",
                        Anthordistance[0],
                        Anthordistance[1],
                        Anthordistance[2]  );

    log_dbg_uwb_raw(    "[d0:% 4d][d1:% 4d][d2:% 4d]\r\n",
                        d0,
                        d1,
                        d2  );
    
    MsgToMaster[13] = (uint8_t)((Anthordistance[0]>> 8)&0xFF);			
    MsgToMaster[14] = (uint8_t)((Anthordistance[0])&0xFF);
    
    MsgToMaster[15] = (uint8_t)((Anthordistance[1]>> 8)&0xFF);			
    MsgToMaster[16] = (uint8_t)((Anthordistance[1])&0xFF);
    
    MsgToMaster[17] = (uint8_t)((Anthordistance[2]>> 8)&0xFF);			
    MsgToMaster[18] = (uint8_t)((Anthordistance[2])&0xFF);
	
	MsgToMaster[19] = (uint8_t)((r0>> 8)&0xFF);			
    MsgToMaster[20] = (uint8_t)((r0)&0xFF);
    
    MsgToMaster[21] = (uint8_t)((r1>> 8)&0xFF);			
    MsgToMaster[22] = (uint8_t)((r1)&0xFF);
    
    MsgToMaster[23] = (uint8_t)((r2>> 8)&0xFF);			
    MsgToMaster[24] = (uint8_t)((r2)&0xFF);
    
	AnchorList[0].x = 0.39;
	AnchorList[0].y = 0;
	AnchorList[0].z = 0;

	AnchorList[1].x = 5.14;
	AnchorList[1].y = 0.00;
	AnchorList[1].z = 0.00;

	AnchorList[2].x = 0.45;
	AnchorList[2].y = 4.83;
	AnchorList[2].z = 0.00;	
    
	Th_Location2(AnchorList, Anthordistance);
    
	count_x1++;
    
    if(count_x1%1000==1)
        count_x++;
}

void tag_data_profess_64hz_in_main_loop_dwm1000(void)
{
    uint8_t tmp_uint8 = 0;
    
    MsgToMaster[8] = tmp_uint8;                     // Battery
    
    if(flag_ble_connected)
    {
		
		process_data();
		
		MsgToMaster[0] = 0xFF;                          // Header
        MsgToMaster[1] = 0x02;
    
        MsgToMaster[2] = _rssi_;                        // 2020-01-13 两个字节的 ID 改为 RSSI 和 ID
        MsgToMaster[3] = UserFlashCfg.Pkg.DevID;
    }
}


void dev_send_64hz_in_rtc_dwm1000(void)
{
	static uint32_t Cnt_64Hz =0;
    static uint32_t Cnt_Send = 0;
    
//    double  x_d;
//    float   x_f;
//    int16_t x_i;
//    
//    double  y_d;
//    float   y_f;
//    int16_t y_i;
    
    int16_t x, y;
    uint16_t d0, d1, d2;
    
    Cnt_64Hz++;
	//if(1 == local_data_ready)
	//if(0 == (Cnt_64Hz%8))
	{
        Cnt_Send++;
        
//        x_d = 12.34;
//        y_d = -98.76;
//        
//        x_f = x_d;
//        y_f = y_d;
//        
////        x_i = -100;
//        x_i = x_f;
//        y_i = y_f;
////        
//        log_dbg_uwb_int16("x_d=%f y_d=%f ",     x_d,   y_d);
//        log_dbg_uwb_int16("x_f=%f y_f=%f " ,    x_f,    y_f);
//        log_dbg_uwb_int16("x_i=%d y_i=%d ",     x_i,    y_i);
//        
//        MsgToMaster[9]  = x_i >> 8;
//        MsgToMaster[10] = x_i & 0xFF;
//        
//        MsgToMaster[11] = y_i >> 8;
//        MsgToMaster[12] = y_i & 0xFF;
        
        x = MsgToMaster[9];
        x <<= 8;
        x |= MsgToMaster[10];
        
        y = MsgToMaster[11];
        y <<= 8;
        y |= MsgToMaster[12];
        
        d0 = MsgToMaster[13];
        d0 <<= 8;
        d0 |= MsgToMaster[14];
        
        d1 = MsgToMaster[15];
        d1 <<= 8;
        d1 |= MsgToMaster[16];
        
        d2 = MsgToMaster[17];
        d2 <<= 8;
        d2 |= MsgToMaster[18];
        
		log_dbg_uwb_send(   "%03d: d0=%04d d1=%04d d2=%04d (x=%+ 4d, y=%+ 4d)\r\n",
                            Cnt_Send,
                            d0,
                            d1,
                            d2,
                            x,
                            y   );
        
		send_data_to_ergolab();
		local_data_ready=0;
	}
}


bool uart_rx_handle_dwm1000(uint8_t* p_rx, uint16_t len)
{     
//    /*
//        0    1    2       3       4
//        [FF] [PB] [Tmp_H] [Tmp_L] [CRC]
//        
//        CRC = 前面所有自己按位异或
//    */
//    uint16_t tmp_uint16 = 0;
//    
//    //log_dbg_chest_st("%02X\r\n", p_rx[len-1]);
//    
//    if(1 == len)                        // FF 开头
//    {
//        if(0xFF != p_rx[0])
//            return true;
//    }
//    else if(5 == len)                   // 完整长度 4 字节
//    {
//        if(p_rx[4] == (p_rx[0] ^ p_rx[1] ^ p_rx[2] ^ p_rx[3]))
//        {
//            Got_Chest_ST_Uart_Data = true;
//            
//            DevChest_ST.PB_Pressed = p_rx[1] ? true : false;
//            
//            
//            #if EN_Get_Power_PB_Sta
//                PB_Press_Sta = DevChest_ST.PB_Pressed;
//            #endif  // EN_Get_Power_PB_Sta
//            
//            tmp_uint16 = p_rx[2];           // temperature
//            tmp_uint16 <<= 8;
//            tmp_uint16 |= p_rx[3];
//            DevChest_ST.temperature = tmp_uint16;
//        }
//        
//        return true;
//    }
    
    return false;
}

