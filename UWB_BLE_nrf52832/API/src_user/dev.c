#include "dev.h"

#include "dev_chest.h"
#include "dev_ear.h"
#include "dev_finger.h"
#include "dev_wrist.h"
#include "dev_wrist_v3.h"
#include "dev_ear_v2.h"
#include "dev_test_max_speed.h"

#include "nrf_gpio.h"
#include "blue.h"
#include "battery.h"

//#if (Dev_Indoor_Localization == CFG_DevType)
    p_dev_init dev_init                                             = dev_init_dwm1000;
    p_tag_data_profess_64hz_in_main_loop tag_data_profess_64hz_in_main_loop     = tag_data_profess_64hz_in_main_loop_dwm1000;
    p_dev_sample_256hz_in_main_loop dev_sample_256hz_in_main_loop   = do_nothing;
    p_dev_send_16hz_in_rtc dev_send_16hz_in_rtc                     = do_nothing;
    p_dev_send_64hz_in_rtc dev_send_64hz_in_rtc                     = dev_send_64hz_in_rtc_dwm1000;
    p_uart_rx_handle uart_rx_handle                                 = uart_rx_handle_dwm1000;
    
    DevInfo_t DevInfo_Chest = 
    {
        .DevType    = CFG_DevType,
//        .DevID      = CFG_DevID,
        .RptFreq    = 64,
        .Battery    = 100,
        .Amp        = 100,
        .FrameIndex = 0,
    };
    
    DevInfo_t* pDevInfo = &DevInfo_Chest;
  
  // (Dev_Indoor_Localization == CFG_DevType)


    #define LED_Sta_Off     do{ nrf_gpio_pin_set(STA_LED_PIN);      }while(0)
    #define LED_Sta_On      do{ nrf_gpio_pin_clear(STA_LED_PIN);    }while(0)
    #define LED_Sta_Toogle  do{ nrf_gpio_pin_toggle(STA_LED_PIN);   }while(0)
    

volatile uint8_t MsgToMaster[MSG_TO_MASTER_LEN] = {0};
uint8_t _rssi_ = 0;
bool PB_Press_Sta = false;

void do_nothing(void)
{
    ;
}

bool do_nothing_uart(uint8_t* p_rx, uint16_t len)
{
    return true;
}

void msg_to_master_clear(void)
{
    uint8_t i;
    
    for(i=0; i<MSG_TO_MASTER_LEN; i++)
        MsgToMaster[i] = 0;
}

void sta_led_init(void)
{
    nrf_gpio_cfg_output(DEBUG_LED_PIN);
    nrf_gpio_cfg_output(STA_LED_PIN);
    LED_Sta_On;
}

void power_en_ctrl_start(void)
{
    #ifdef EN_POWER_CTRL
    nrf_gpio_cfg_output(POWER_CTRL_PH_HOLD);    // PS_HOLD
    nrf_gpio_pin_set(POWER_CTRL_PH_HOLD);
    
    nrf_gpio_cfg_input(POWER_CTRL_INT, NRF_GPIO_PIN_NOPULL);
    
    #endif // EN_POWER_CTRL
}

void sta_led_ms_service(void)
{
    #define MS_LED_STA_BAT_LOW_ON                       (100)
    #define MS_LED_STA_BAT_LOW_OFF                      (200)
    #define MS_LED_STA_BAT_LOW_BLINK_TIMES_PCT_010      (2)
    #define MS_LED_STA_BAT_LOW_BLINK_TIMES_PCT_000      (3)
    #define MS_LED_STA_BAT_LOW_SILENT                   (1000)
    
    #define MS_LED_STA_BAT_LOW_PERIOD_BLINK_EACH        (MS_LED_STA_BAT_LOW_ON + MS_LED_STA_BAT_LOW_OFF)
    #define MS_LED_STA_BAT_LOW_PERIOD_BLINK_PCT_010     (MS_LED_STA_BAT_LOW_PERIOD_BLINK_EACH * MS_LED_STA_BAT_LOW_BLINK_TIMES_PCT_010)
    #define MS_LED_STA_BAT_LOW_PERIOD_BLINK_PCT_000     (MS_LED_STA_BAT_LOW_PERIOD_BLINK_EACH * MS_LED_STA_BAT_LOW_BLINK_TIMES_PCT_000)
    #define MS_LED_STA_BAT_LOW_PERIOD_PCT_010           (MS_LED_STA_BAT_LOW_PERIOD_BLINK_PCT_010 + MS_LED_STA_BAT_LOW_SILENT)
    #define MS_LED_STA_BAT_LOW_PERIOD_PCT_000           (MS_LED_STA_BAT_LOW_PERIOD_BLINK_PCT_000 + MS_LED_STA_BAT_LOW_SILENT)
    
    static uint32_t msCnt_LED_Sta = 0;
    static uint32_t msCnt_Power_PB_Release = 0;    // 释放pb按键
    
    msCnt_LED_Sta++;
    
    
    if(msCnt_Power_PB_Release)
    {
        msCnt_Power_PB_Release--;
    }
    
    #if defined(EN_POWER_CTRL)
        if(!nrf_gpio_pin_read(POWER_CTRL_INT))
        {
            LED_Sta_On;
            log_msg("       LED_Sta_On\r\n");
            msCnt_Power_PB_Release = 1000;
            msCnt_LED_Sta = 0;
        }
        else
        {
            if(msCnt_Power_PB_Release)
                LED_Sta_Off;
            else
            {
                if(0 == get_battery_percent_val_real())         // 彻底没电了 每隔 1 秒闪 3 次
                {
                    if(msCnt_LED_Sta < MS_LED_STA_BAT_LOW_PERIOD_PCT_000)
                    {
                        if(msCnt_LED_Sta < MS_LED_STA_BAT_LOW_PERIOD_BLINK_PCT_000)
                        {
                            if((msCnt_LED_Sta % MS_LED_STA_BAT_LOW_PERIOD_BLINK_EACH) < MS_LED_STA_BAT_LOW_ON)
                                LED_Sta_On;
                            else
                                LED_Sta_Off;
                        }
                        else
                            LED_Sta_Off;
                    }
                    else
                    {
                        msCnt_LED_Sta = 0;
                    }
                }
                else if(get_battery_percent_val_real() < 10)     // 电量低 每隔 1 秒闪 2 次
                {
                    if(msCnt_LED_Sta < MS_LED_STA_BAT_LOW_PERIOD_PCT_010)
                    {
                        if(msCnt_LED_Sta < MS_LED_STA_BAT_LOW_PERIOD_BLINK_PCT_010)
                        {
                            if((msCnt_LED_Sta % MS_LED_STA_BAT_LOW_PERIOD_BLINK_EACH) < MS_LED_STA_BAT_LOW_ON)
                                LED_Sta_On;
                            else
                                LED_Sta_Off;
                        }
                        else
                            LED_Sta_Off;
                    }
                    else
                    {
                        msCnt_LED_Sta = 0;
                    }
                }
                else if(flag_ble_connected)                 // 正常连接 每秒闪一次
                {
                    if(msCnt_LED_Sta < 1000)
                    {
                        if(msCnt_LED_Sta < 100)
                            LED_Sta_On;
                        else
                            LED_Sta_Off;
                    }
                    else
                        msCnt_LED_Sta = 0;
                }
                else                                        // 广播中 不断快速闪烁
                {
                    if(0 == (msCnt_LED_Sta % 40))
                        LED_Sta_Toogle;
                }
            }
        }
    #elif EN_Get_Power_PB_Sta
        if(PB_Press_Sta)
        {
            LED_Sta_On;
            msCnt_Power_PB_Release = 1000;
            msCnt_LED_Sta = 0;
        }
        else
        {
            if(msCnt_Power_PB_Release)
                LED_Sta_Off;
            else
            {
//                if(0 == get_battery_percent_val_real())         // 彻底没电了 每隔 1 秒闪 3 次
//                {
//                    if(msCnt_LED_Sta < MS_LED_STA_BAT_LOW_PERIOD_PCT_000)
//                    {
//                        if(msCnt_LED_Sta < MS_LED_STA_BAT_LOW_PERIOD_BLINK_PCT_000)
//                        {
//                            if((msCnt_LED_Sta % MS_LED_STA_BAT_LOW_PERIOD_BLINK_EACH) < MS_LED_STA_BAT_LOW_ON)
//                                LED_Sta_On;
//                            else
//                                LED_Sta_Off;
//                        }
//                        else
//                            LED_Sta_Off;
//                    }
//                    else
//                    {
//                        msCnt_LED_Sta = 0;
//                    }
//                }
//                else if(get_battery_percent_val_real() < 10)     // 电量低 每隔 1 秒闪 2 次
//                {
//                    if(msCnt_LED_Sta < MS_LED_STA_BAT_LOW_PERIOD_PCT_010)
//                    {
//                        if(msCnt_LED_Sta < MS_LED_STA_BAT_LOW_PERIOD_BLINK_PCT_010)
//                        {
//                            if((msCnt_LED_Sta % MS_LED_STA_BAT_LOW_PERIOD_BLINK_EACH) < MS_LED_STA_BAT_LOW_ON)
//                                LED_Sta_On;
//                            else
//                                LED_Sta_Off;
//                        }
//                        else
//                            LED_Sta_Off;
//                    }
//                    else
//                    {
//                        msCnt_LED_Sta = 0;
//                    }
//                }
//                else if(flag_ble_connected)                 // 正常连接 每秒闪一次
                if(flag_ble_connected)
                {
                    if(msCnt_LED_Sta < 1000)
                    {
                        if(msCnt_LED_Sta < 100)
                            LED_Sta_On;
                        else
                            LED_Sta_Off;
                    }
                    else
                        msCnt_LED_Sta = 0;
                }
                else                                        // 广播中 不断快速闪烁
                {
                    if(0 == (msCnt_LED_Sta % 40))
                        LED_Sta_Toogle;
                }
            }
        }
    #else
        
        if(0 == get_battery_percent_val_real())         // 彻底没电了 每隔 1 秒闪 3 次
        {
            if(msCnt_LED_Sta < MS_LED_STA_BAT_LOW_PERIOD_PCT_000)
            {
                if(msCnt_LED_Sta < MS_LED_STA_BAT_LOW_PERIOD_BLINK_PCT_000)
                {
                    if((msCnt_LED_Sta % MS_LED_STA_BAT_LOW_PERIOD_BLINK_EACH) < MS_LED_STA_BAT_LOW_ON)
                        LED_Sta_On;
                    else
                        LED_Sta_Off;
                }
                else
                    LED_Sta_Off;
            }
            else
            {
                msCnt_LED_Sta = 0;
            }
        }
        else if(get_battery_percent_val_real() < 10)     // 电量低 每隔 1 秒闪 2 次
        {
            if(msCnt_LED_Sta < MS_LED_STA_BAT_LOW_PERIOD_PCT_010)
            {
                if(msCnt_LED_Sta < MS_LED_STA_BAT_LOW_PERIOD_BLINK_PCT_010)
                {
                    if((msCnt_LED_Sta % MS_LED_STA_BAT_LOW_PERIOD_BLINK_EACH) < MS_LED_STA_BAT_LOW_ON)
                        LED_Sta_On;
                    else
                        LED_Sta_Off;
                }
                else
                    LED_Sta_Off;
            }
            else
            {
                msCnt_LED_Sta = 0;
            }
        }
        else if(flag_ble_connected)                 // 正常连接 每秒闪一次
        {
            if(msCnt_LED_Sta < 1000)
            {
                if(msCnt_LED_Sta < 100)
                    LED_Sta_On;
                else
                    LED_Sta_Off;
            }
            else
                msCnt_LED_Sta = 0;
        }
        else                                        // 广播中 不断快速闪烁
        {
            if(0 == (msCnt_LED_Sta % 40))
                LED_Sta_Toogle;
        }
    
    #endif  // defined(EN_POWER_CTRL)
    
}

