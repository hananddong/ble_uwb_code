#include "nrf_drv_rtc.h"
#include "blue.h"
#include "send_timer.h"
#include "nrf_drv_clock.h"
#include "battery.h"
#include "kf_vhub.h"
#include "user_log.h"
#include "user_cfg.h"
#include "user_wdt.h"
#include "user_timer.h"
#include "tag.h"
#include "ant.h"
#include "tag_mast_slave.h"
#include "dev.h"
#include "trilateration.h"

#include "nrfx_wdt.h"

#define COMPARE_COUNTERTIME  (3UL)  // Get Compare event COMPARE_TIME seconds after the counter starts from 0.

const nrf_drv_rtc_t rtc = NRF_DRV_RTC_INSTANCE(2);

void rtc_handler(nrf_drv_rtc_int_type_t int_type);
void lfclk_config(void);
void rtc_config(void);

/*
    测试 RSSI
    BLE_UUID_NUS_SERVICE        扫描时 UUID_1
    BLE_UUID_NUS_RX_CHARACTERISTIC
    BLE_UUID_KF_`EGL_SCAN        扫描时 UUID_2
    BLE_UUID_KF_EGL_SCAN_TYPE
    m_adv_uuids
    ble_nus_init
*/

/*
    uart_event_handle       接收 UWB 板数据
    log_dbg_uwb_loaclation  用来输出调试信息
*/

/*
	TAG
	ANTHOR
	ANCHOR_MAX_NUM
	ANCHOR_REFRESH_COUNT
*/

int main(void)
{
    //#ifdef TAG
    log_init();                         // JLink 调试信息输出初始化
    power_en_ctrl_start();              // 开机的开始，就把ps_hold保持住，防止因为后面的初始化错误导致的堵塞，电源开启不成功
    log_msg("power_en_ctrl_start ！ \r\n");
	user_cfg_fs_init();                 // Flash 操作初始化
    read_user_cfg_in_flash();           // 从 Flash 读取配置信息
    
    uart_init();                        // 指尖、胸带 通过 UART 和 STM32F1 通信
	printf("uart ！ \r\n");
    
	update_user_cfg_by_flash_record();  // 根据保存的配置信息设置其他参数
	user_wdt_init();                    // 开启看门狗 防止异常死机
    lfclk_config();                     // 协议栈用到的低频时钟初始化
    rtc_config();                       // rtc_handler
    
    
    ble_init();                         // ble_event_handler    nus_data_handler

	sta_led_init();                     // 指示灯初始化
    Timer_ms_Service_Init();            // Timer_ms_IRQ_Handle
	log_msg("Timer_ms_Service_Init ! \r\n");
    dev_init();                         // 传感器设备初始化 根据不同设备执行不同操作
  
	log_msg("slave_tag_is_ready ! \r\n"); 
     
//	#endif
//	#ifdef ANTHOR
//	log_init();                         // JLink 调试信息输出初始化
//  uart_init();                        // 指尖、胸带 通过 UART 和 STM32F1 通信
//	dev_init();                         // 传感器设备初始化 根据不同设备执行不同操作
//  log_msg("ant_is_ready\r\n");
//	#endif
    #ifdef ANTHOR
    Anchor_Array_Init();
    #endif // ANTHOR
    while(1)
    {
        
        #ifdef ANTHOR
            anchor_function_loop();
        #endif // ANTHOR
        
        #ifdef TAG
            single_tag_loop();
            user_cfg_write_service();
        #endif // TAG
        if(true == flag_detection_power)
        {
            flag_detection_power = false; 
            LED_DEBUG_Toogle;
            get_battery_percent_val();
        }
        if(flag_SysReset)               // 部分配置操作需要重启
        {
            log_msg("reset !!! \r\n");
            NVIC_SystemReset();
        }
    }
}

// RTC 中断处理函数 64 Hz
void rtc_handler(nrf_drv_rtc_int_type_t int_type)
{
	static uint32_t tickCnt_256_Hz = 0;
	
    if (int_type == NRF_DRV_RTC_INT_TICK)
    {
		tickCnt_256_Hz++;
        flag_256_Hz = true;
        
        if(0 == (tickCnt_256_Hz % 4))
        {
            flag_64_Hz = true;
        
            if(Report_Data == ReportType)
            {
                //dev_send_64hz_in_rtc();
                
                if(0 == (tickCnt_256_Hz % 16))
                {
                    flag_16_Hz = true;
                    //dev_send_16hz_in_rtc();
                }
            }
        }
        if(0 == (tickCnt_256_Hz % 42))
        {
            
            flag_detection_power = true;
        }
    }
}

void lfclk_config(void)
{
    ret_code_t err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);
    
    nrf_drv_clock_lfclk_request(NULL);
}

void rtc_config(void)
{
    uint32_t err_code;
    
    nrf_drv_rtc_config_t config = NRF_DRV_RTC_DEFAULT_CONFIG;
    
    
    #if EN_TEST_MAX_SPEED
        config.prescaler = 512 / 2;
    #else
    config.prescaler = 32768 / 256;   //512;     // 32768 / 512 = 64
    #endif  // EN_TEST_MAX_SPEED
    
    err_code = nrf_drv_rtc_init(&rtc, &config, rtc_handler);
    APP_ERROR_CHECK(err_code);
    
    nrf_drv_rtc_tick_enable(&rtc, true);
    err_code = nrf_drv_rtc_cc_set(&rtc, 0, COMPARE_COUNTERTIME * 8, true);
    APP_ERROR_CHECK(err_code);
    
    nrf_drv_rtc_enable(&rtc);
}

