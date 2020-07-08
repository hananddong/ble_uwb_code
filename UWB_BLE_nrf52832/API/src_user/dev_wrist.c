#include "dev_wrist.h"

#include "max11213_wrist.h"
#include "mpu9250.h"
#include "battery.h"
#include "acquisition.h"
#include "blue.h"
#include "BPM.h"
#include "dev.h"

void dev_init_wrist(void)
{
    spi_pin_init_wrist();
    spi_init();
    GSR_hardware_detect_pin_init();
    mpu9250_init(IIC_SCL_PIN, IIC_SDA_PIN);
    max11213_init_GSR();
    max11213_init_PPG();
    battery_check_init(NRF_SAADC_INPUT_AIN4);
}    

void dev_sample_64hz_in_main_loop_wrist(void)
{
    uint8_t tmp_uint8;
    
    tmp_uint8 = get_battery_percent_val();
    MsgToMaster[8] = tmp_uint8;                     // Battery
    
    if(flag_ble_connected)
    {
        update_report_data_wrist();
    }
}

void dev_send_64hz_in_rtc_wrist(void)
{
    uint32_t tmp_uint32;
    
    tmp_uint32 = ++pDevInfo->FrameIndex;            // FrameIndex
    MsgToMaster[4] = tmp_uint32 >> 24;
    MsgToMaster[5] = tmp_uint32 >> 16;
    MsgToMaster[6] = tmp_uint32 >> 8;
    MsgToMaster[7] = tmp_uint32 >> 0;
    
    send_data_to_chen();
}
