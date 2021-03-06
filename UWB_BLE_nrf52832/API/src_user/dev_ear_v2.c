#include "dev_ear_v2.h"
#include "nrf_gpio.h"
#include "nrf_drv_saadc.h"
#include "battery.h"
#include "blue.h"
#include "nrf_delay.h"
#include "BPM.h"
#include "dev.h"
#include "user_log.h"

#define SPI_CS_Pin14        14
#define SPI_CK_Pin05        5
#define SPI_MO_Pin04        4
#define SPI_MI_Pin11        11

void dev_ear_v2_spi_pin_init(void);
void dev_ear_v2_update_ppg(void);
void dev_ear_v2_update_bpm(void);

uint16_t dev_ear_v2_AD7685_read(void);

typedef struct
{
    uint8_t idx;
    uint16_t ppg_adc;
    uint16_t ppg;
    uint16_t ppg_report[4];
    uint16_t bpm;
    uint16_t bpm_report[4];
}DevEarData_t;

DevEarData_t DevEarV2Data = {0};

extern void dev_init_ear_v2(void)
{
    dev_ear_v2_spi_pin_init();
    battery_check_init(NRF_SAADC_INPUT_AIN5);
    
    #ifdef EN_POWER_CTRL
    nrf_gpio_cfg_output(POWER_CTRL_PH_HOLD);    // PS_HOLD
    nrf_gpio_pin_set(POWER_CTRL_PH_HOLD);
    
    nrf_gpio_cfg_input(POWER_CTRL_INT, NRF_GPIO_PIN_NOPULL);
    
    #endif // EN_POWER_CTRL
}

extern void dev_sample_64hz_in_main_loop_ear_v2(void)
{
    uint8_t tmp_uint8 = 0;
    uint16_t tmp_uint16 = 0;
//    uint32_t tmp_uint32 = 0;
    
    tmp_uint8 = get_battery_percent_val();
    MsgToMaster[8] = tmp_uint8;                     // Battery
    
    if(flag_ble_connected)
    {
        dev_ear_v2_update_ppg();
        dev_ear_v2_update_bpm();
    
        DevEarV2Data.idx++;
        if(DevEarV2Data.idx >= 4)
        {
            DevEarV2Data.idx = 0;
        
            tmp_uint16 = DevEarV2Data.ppg_report[0];          // PPG_0
            MsgToMaster[9] = tmp_uint16 >> 8;
            MsgToMaster[10] = tmp_uint16;
            
            tmp_uint16 = DevEarV2Data.ppg_report[1];          // PPG_1
            MsgToMaster[11] = tmp_uint16 >> 8;
            MsgToMaster[12] = tmp_uint16;
            
            tmp_uint16 = DevEarV2Data.ppg_report[2];          // PPG_2
            MsgToMaster[13] = tmp_uint16 >> 8;
            MsgToMaster[14] = tmp_uint16;
            
            tmp_uint16 = DevEarV2Data.ppg_report[3];          // PPG_3
            MsgToMaster[15] = tmp_uint16 >> 8;
            MsgToMaster[16] = tmp_uint16;
            
            tmp_uint16 = DevEarV2Data.bpm_report[0];          // BPM_0
            MsgToMaster[17] = tmp_uint16 >> 8;
            MsgToMaster[18] = tmp_uint16;
            
            tmp_uint16 = DevEarV2Data.bpm_report[1];          // BPM_1
            MsgToMaster[19] = tmp_uint16 >> 8;
            MsgToMaster[20] = tmp_uint16;
            
            tmp_uint16 = DevEarV2Data.bpm_report[2];          // BPM_2
            MsgToMaster[21] = tmp_uint16 >> 8;
            MsgToMaster[22] = tmp_uint16;
            
            tmp_uint16 = DevEarV2Data.bpm_report[3];          // BPM_3
            MsgToMaster[23] = tmp_uint16 >> 8;
            MsgToMaster[24] = tmp_uint16;
        }
        
        MsgToMaster[0] = 0xFF;                          // Header
        MsgToMaster[1] = 0x02;
    
        MsgToMaster[2] = _rssi_;                        // 2020-01-13 两个字节的 ID 改为 RSSI 和 ID
        MsgToMaster[3] = UserFlashCfg.Pkg.DevID;
        
//        tmp_uint32 = ++pDevInfo->FrameIndex;            // FrameIndex
//        MsgToMaster[4] = tmp_uint32 >> 24;
//        MsgToMaster[5] = tmp_uint32 >> 16;
//        MsgToMaster[6] = tmp_uint32 >> 8;
//        MsgToMaster[7] = tmp_uint32 >> 0;
    }
}

extern void dev_send_16hz_in_rtc_ear_v2(void)
{
    uint32_t tmp_uint32;
    
    tmp_uint32 = ++pDevInfo->FrameIndex;            // FrameIndex
    MsgToMaster[4] = tmp_uint32 >> 24;
    MsgToMaster[5] = tmp_uint32 >> 16;
    MsgToMaster[6] = tmp_uint32 >> 8;
    MsgToMaster[7] = tmp_uint32 >> 0;
    
    send_data_to_chen();
}

void dev_ear_v2_spi_pin_init(void)
{
    nrf_gpio_cfg_output(SPI_CS_Pin14);
    nrf_gpio_cfg_output(SPI_MO_Pin04);
    nrf_gpio_cfg_output(SPI_CK_Pin05);
    nrf_gpio_cfg_input(SPI_MI_Pin11, NRF_GPIO_PIN_PULLDOWN);
    nrf_gpio_pin_clear(SPI_CK_Pin05);
}

void dev_ear_v2_update_ppg(void)
{
    DevEarV2Data.ppg_adc = dev_ear_v2_AD7685_read();
    
    //DevEarV2Data.ppg = Filter_MovingAverage3(DevEarV2Data.ppg_adc);   // 耳夹的信号比较好 不用滤波
    
    DevEarV2Data.ppg_report[DevEarV2Data.idx] = (DevEarV2Data.ppg_adc - 16300) * 4 / 6;
}

void dev_ear_v2_update_bpm(void)
{
    DevEarV2Data.bpm = update_bpm_by_ppg(DevEarV2Data.ppg_adc);
    DevEarV2Data.bpm_report[DevEarV2Data.idx] = DevEarV2Data.bpm;//DevEarV2Data.bpm * 100;
}

uint16_t dev_ear_v2_AD7685_read(void)
{
    uint16_t DataBuffer = 0;
    uint8_t i;
    
    nrf_gpio_pin_clear(SPI_CS_Pin14);     //cnv  0
    nrf_gpio_pin_set(SPI_MO_Pin04);         //sdi 1
    nrf_delay_us(1);
    
    nrf_gpio_pin_set(SPI_CS_Pin14);       //cnv   1
    nrf_delay_us(1);
    
    nrf_gpio_pin_set(SPI_MO_Pin04);         //sdi 1
    nrf_delay_us(5);
    
    //DataBuffer = AD7685_ReadData_Time();
    for(i = 0; i < 16; i++)
    {
        nrf_gpio_pin_set(SPI_CK_Pin05);
        
        nrf_gpio_pin_clear(SPI_CS_Pin14);
        DataBuffer <<= 1;
        
        if(nrf_gpio_pin_read(SPI_MI_Pin11))
            DataBuffer = DataBuffer | 0x01;
            
        nrf_gpio_pin_clear(SPI_CK_Pin05);
    }
    
    nrf_gpio_pin_set(SPI_MO_Pin04);         //sdi 1
    nrf_gpio_pin_clear(SPI_MO_Pin04);       //sdi 0
    nrf_gpio_pin_set(SPI_CS_Pin14);       //cnv   1
    
    return DataBuffer;
}

