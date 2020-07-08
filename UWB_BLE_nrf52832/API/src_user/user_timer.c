#include "user_timer.h"
#include "nrf_drv_timer.h"
#include "kf_vhub.h"
#include "user_log.h"
#include "user_cfg.h"
#include "user_wdt.h"
#include "dev.h"
#include "dev_test_max_speed.h"

#include "dev_ear.h"
#include "nrf_gpio.h"
#include "ble_gap.h"

const nrf_drv_timer_t Timer_ms = NRF_DRV_TIMER_INSTANCE(1);

void Timer_ms_IRQ_Handle(nrf_timer_event_t event_type, void *p_context);

void Timer_ms_Service_Init(void)
{
    uint32_t time_ms = 1;   // 产生中断的时间间隔
    uint32_t time_ticks;
    uint32_t err_code = NRF_SUCCESS;
    
    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    err_code = nrf_drv_timer_init(&Timer_ms, &timer_cfg, Timer_ms_IRQ_Handle);
    APP_ERROR_CHECK(err_code);
    
    time_ticks = nrf_drv_timer_ms_to_ticks(&Timer_ms, time_ms);
    
    // 定时器通道 0 比较匹配产生中断
    nrf_drv_timer_extended_compare(&Timer_ms, NRF_TIMER_CC_CHANNEL0, time_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);
    nrfx_timer_enable(&Timer_ms);
}

void Timer_ms_IRQ_Handle(nrf_timer_event_t event_type, void *p_context)
{
    #if EN_TEST_MAX_SPEED
        static uint32_t msCnt_TestMaxSpeed = 0;
    #endif  // EN_TEST_MAX_SPEED
    
    static uint32_t msCnt_SysReset = 0;
    static uint32_t msCnt_Feed_WDT = 0;
    
    #ifdef  EN_POWER_CTRL
    static uint32_t msCnt_Int_Low = 0;
    #endif  // EN_POWER_CTRL
    
    uint8_t i;
    
    if(NRF_TIMER_EVENT_COMPARE0 == event_type)
    {
        msCnt_Feed_WDT++;
        if(msCnt_Feed_WDT >= 500)
        {
            msCnt_Feed_WDT = 0;
            
            user_wdt_feed();
        }
        
        #if EN_TEST_MAX_SPEED
        msCnt_TestMaxSpeed++;
        if(msCnt_TestMaxSpeed >= 1000)
        {
            msCnt_TestMaxSpeed = 0;
            test_max_speed_index_reset();
        }
        #endif  // EN_TEST_MAX_SPEED
        
        if(msCnt_SysReset)
        {
            msCnt_SysReset--;
            if(0 == msCnt_SysReset)
            {
                flag_SysReset = true;
            }
        }
        
        if(msCnt_ModifyGroupID)
        {
            msCnt_ModifyGroupID--;
            
            if(0 == msCnt_ModifyGroupID)
            {
                msCnt_SysReset = 100;
                
                // 断开蓝牙连接
                //sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
                for(i = 0; i < NRF_SDH_BLE_PERIPHERAL_LINK_COUNT; i++)
                    sd_ble_gap_disconnect(i, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
                
                log_msg("NVIC_SystemReset_after %d ms\r\n", msCnt_SysReset);
            }
        }
        
        sta_led_ms_service();
        
        #ifdef  EN_POWER_CTRL
        
        if(!nrf_gpio_pin_read(POWER_CTRL_INT))
        {
            msCnt_Int_Low++;
            
            if(msCnt_Int_Low > 2000)
            {
                msCnt_Int_Low = 0;
                
                //log_dbg("power_off\r\n");
                
                nrf_gpio_pin_clear(POWER_CTRL_PH_HOLD);
            }
        }
        else
        {
            msCnt_Int_Low = 0;
        }
        
        #endif  // EN_POWER_CTRL
    }
}

