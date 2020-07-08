#include "dev_wrist_v3.h"

#include "max11213_wrist.h"
#include "mpu9250.h"
#include "battery.h"
#include "acquisition.h"
#include "blue.h"
#include "BPM.h"
#include "dev.h"
#include "son1421.h"
#include "stdlib.h"
#include "mpu9250.h"
#include "user_log.h"

#define SPI_CK_Pin11    11
#define SPI_MI_Pin12    12
#define SPI_MO_Pin13    13
#define SPI_CS_Pin14    14

uint16_t get_data_wrist_v3_GSR(void);
uint16_t dev_wrist_v3_ad7685_read(void);
void dev_wrist_v3_ad7685_spi_pin_init(void);

void dev_wrist_v3_ad7685_spi_pin_init(void)
{
    nrf_gpio_cfg_output(11);
    nrf_gpio_cfg_input(12, NRF_GPIO_PIN_PULLUP);
    nrf_gpio_cfg_output(13);
}

void dev_init_wrist_v3(void)
{
    spi_pin_init_wrist();
    spi_init();
    GSR_hardware_detect_pin_init();
    mpu9250_init(IIC_SCL_PIN, IIC_SDA_PIN);
    //max11213_init_GSR();      // 手环 V3 GSR 将 MAX11213 换成了 AD7685
    max11213_init_PPG();
    
    spi_un_init();
    
    battery_check_init(NRF_SAADC_INPUT_AIN4);
    
    #ifdef EN_POWER_CTRL
    nrf_gpio_cfg_output(POWER_CTRL_PH_HOLD);    // PS_HOLD
    nrf_gpio_pin_set(POWER_CTRL_PH_HOLD);
    
    nrf_gpio_cfg_input(POWER_CTRL_INT, NRF_GPIO_PIN_NOPULL);
    
    #endif // EN_POWER_CTRL
}    

void dev_sample_64hz_in_main_loop_wrist_v3(void)
{
    accel_values_t acc;
    
    uint8_t tmp_uint8 = 0;
    uint16_t tmp_uint16 = 0;
//    uint32_t tmp_uint32 = 0;
        
    tmp_uint8 = get_battery_percent_val();
    MsgToMaster[8] = tmp_uint8;                             // Battery
    
    if(flag_ble_connected)
    {
        MsgToMaster[0] = 0xFF;                              // Header
        MsgToMaster[1] = 0x02;
    
        MsgToMaster[2] = _rssi_;                        // 2020-01-13 两个字节的 ID 改为 RSSI 和 ID
        MsgToMaster[3] = UserFlashCfg.Pkg.DevID;
        
//        tmp_uint32 = ++pDevInfo->FrameIndex;                // FrameIndex
//        MsgToMaster[4] = tmp_uint32 >> 24;
//        MsgToMaster[5] = tmp_uint32 >> 16;
//        MsgToMaster[6] = tmp_uint32 >> 8;
//        MsgToMaster[7] = tmp_uint32 >> 0;
        
        spi_init();
        tmp_uint16 = update_ppg_wrist();                    // PPG
        MsgToMaster[9] = tmp_uint16 >> 8;
        MsgToMaster[10] = tmp_uint16;
        
        spi_un_init();
        dev_wrist_v3_ad7685_spi_pin_init();
        GSR_hardware_detect_pin_init();
        tmp_uint16 = get_data_wrist_v3_GSR();               // GSR 皮电
        MsgToMaster[11] = tmp_uint16 >> 8;
        MsgToMaster[12] = tmp_uint16;
        
        tmp_uint16 = SON1421_Read_Temperature(8) * 100;     // SKT
        MsgToMaster[13] = tmp_uint16 >> 8;
        MsgToMaster[14] = tmp_uint16;
        
        mpu9250_read_accel(&acc);
        tmp_uint16 = acc.x;                                 // ACC_X
        MsgToMaster[15] = tmp_uint16 >> 8;
        MsgToMaster[16] = tmp_uint16;
        
        tmp_uint16 = acc.y;                                 // ACC_Y
        MsgToMaster[17] = tmp_uint16 >> 8;
        MsgToMaster[18] = tmp_uint16;
        
        tmp_uint16 = acc.z;                                 // ACC_Z
        MsgToMaster[19] = tmp_uint16 >> 8;
        MsgToMaster[20] = tmp_uint16;
        
        tmp_uint16 = update_bpm_by_ppg(PPG_Val);// * 100;   // BPM
        MsgToMaster[21] = tmp_uint16 >> 8;
        MsgToMaster[22] = tmp_uint16;
    }
}

void dev_send_64hz_in_rtc_wrist_v3(void)
{
    uint32_t tmp_uint32;
    
    tmp_uint32 = ++pDevInfo->FrameIndex;            // FrameIndex
    MsgToMaster[4] = tmp_uint32 >> 24;
    MsgToMaster[5] = tmp_uint32 >> 16;
    MsgToMaster[6] = tmp_uint32 >> 8;
    MsgToMaster[7] = tmp_uint32 >> 0;
    
    send_data_to_chen();
}

uint16_t dev_wrist_v3_ad7685_read(void)
{
    uint16_t DataRead = 0;
    uint8_t i;
    
    nrf_gpio_pin_clear(SPI_CS_Pin14);
    nrf_gpio_pin_set(SPI_MO_Pin13);
    nrf_delay_us(1);
    
    nrf_gpio_pin_set(SPI_CS_Pin14);
    nrf_delay_us(1);
    
    nrf_gpio_pin_set(SPI_MO_Pin13);
    nrf_delay_us(5);
    
    for(i = 0; i < 16; i++)
    {
        nrf_gpio_pin_set(SPI_CK_Pin11);
        
        nrf_gpio_pin_clear(SPI_CS_Pin14);
        DataRead <<= 1;
        
        if(nrf_gpio_pin_read(SPI_MI_Pin12))
            DataRead = DataRead | 0x01;
            
        nrf_gpio_pin_clear(SPI_CK_Pin11);
    }
    
    nrf_gpio_pin_set(SPI_MO_Pin13);
    nrf_gpio_pin_clear(SPI_MO_Pin13);
    nrf_gpio_pin_set(SPI_CS_Pin14);
    
    return DataRead;
}

uint16_t get_data_wrist_v3_GSR(void)
{
    /*
        GSR 
            Galvanic Skin Reflect   皮电反映
            是测量人体两点之间的电导率(电阻值的倒数)
        
        电导率的点位是：西门子
            如人体两个手指间电阻值是 1Ω     则电导率为 1/1 S = 1S
            如人体两个手指间电阻值是 100Ω   则电导率为 1/100 S = 0.01S
        
        GSR 测量电路由 放大器、跟随器、ADC 组成
        
        测量原理：
        一个手指间施加特定值的电压 Vi 有人体测量点之间 电阻 R20 和 放大器电阻 R23 组成特定倍数的放大器
        通过测量输入电压和输出电压的放大倍数，推导出 人体两个测试点之间的电阻值(倒数就是 GSR)
        
        放大器满足：
        1. 输入电压     Vi = 2 * Vref                   // 硬件设计实现特定倍数关系
        2. 输出电压     Vo = (ADC / 65536) * Vref       // 16位 ADC 满量程值为 65536
        3. 放大比例     Vo = Vi / (2 + (R23 / R20))     // R23 是固定值电阻 R20 是测量点之间的电阻
        4. R23 值为 1MΩ                                 // 方便计算
    
        所以：
            Vo  = Vi / (2 + (R23 / R20))
                = (2 * Vref) / (2 + (R23 / R20))
                = (2 * Vref) / (2 + (1000000 / R20))
            
        又因： Vo = (ADC / 65536) * Vref
        所以： (2 * Vref) / (2 + (1000000 / R20)) = (ADC / 65536) * Vref
        即：
            2 / (2 + (1000000 / R20)) = ADC / 65536
            (2 + (1000000 / R20))   = 2 / (ADC / 65536)
                                    = (65536 * 2) / ADC
                                    
            (1000000 / R20) = ((2 * 65536) / ADC) - 2
             1 / R20 = (((2 * 65536) / ADC) - 2) / 1000000
             
         1 / R20 即要计算的电导率值 两遍同时除以 1000000 单位由 S 变为 uS
            = ((2 * 65536) / ADC) - 2
    */
    
    static uint16_t GSR_Last = 0;
    
    uint16_t  GSR_ADC = 0;
    uint32_t  GSR_Val = 0;
    
    
    // 检测皮电测量使用的硬件(手环底部的金属触点 或 外接触点连接到手环 USB 口)
    if(nrf_gpio_pin_read(Pin04_GSR_Hardware_Detect))
    {
        nrf_gpio_pin_set(Pin02_GSR_Hardware_Vi_N_Sel);
        nrf_gpio_pin_set(Pin03_GSR_Hardware_Vi_P_Sel);
    }
    else
    {
        nrf_gpio_pin_clear(Pin03_GSR_Hardware_Vi_P_Sel);
        nrf_gpio_pin_clear(Pin02_GSR_Hardware_Vi_N_Sel);
    }
    
    GSR_ADC = dev_wrist_v3_ad7685_read();
    
//    log_dbg("gsr_adc = %d\r\n", GSR_ADC);
    
    if(GSR_ADC >= 0xF3C0)
        return 0;               // human <= 10M,10M is F3C
    
    /*
          ((2 * 65536) / ADC) - 2
        = ((2 * 65536) / ADC) - (2 * (ADC / ADC))
        = ((2 * 65536) - (2 * ADC)) / ADC
        = 2((65536 - ADC) / ADC)
    */
//    GSR_Val = 65536 - (GSR_ADC);
//    GSR_Val = 2000 * GSR_Val;           // 此处将 GSR 的原始值放大了 1000 倍
//    GSR_Val = GSR_Val / GSR_ADC;
    
    // 手环 V3 硬件电路 Vi = Vref(原来是 Vi = 2 * Vref)
    GSR_Val = 65536 - (2 * GSR_ADC);
    GSR_Val = 1000 * GSR_Val;           // 此处将 GSR 的原始值放大了 1000 倍
    GSR_Val = GSR_Val / GSR_ADC;
    
    if(abs(GSR_Val - GSR_Last) > 21000)
        GSR_Val = GSR_Last;                // 过滤掉皮电的尖峰
        
    else
        GSR_Last = GSR_Val ;
        
    if(GSR_Val >= 0xFFFF)
        GSR_Val = 0xFFFF;
        
    return (uint16_t)GSR_Val;
}

