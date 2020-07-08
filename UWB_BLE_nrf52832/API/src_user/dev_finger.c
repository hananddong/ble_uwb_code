#include "dev_finger.h"
#include "blue.h"
#include "dev.h"
#include "battery.h"
#include "nrf_delay.h"
#include "stdlib.h"
#include "son1421.h"
#include "user_log.h"

#define SPI_CK_Pin15    15
#define SPI_MI_Pin16    16
#define SPI_MO_Pin17    17
#define SPI_CS_Pin18    18

void dev_finger_spi_pin_init(void);
uint16_t dev_finger_ad7685_read(void);

bool Got_Finger_ST_Uart_Data = false;

typedef struct
{
    uint16_t bpm;
    uint16_t ppg;
    uint16_t spo2;
    uint16_t skt;
    uint16_t gsr;
}DevFinger_ST_t;

DevFinger_ST_t DevFinger_ST = {0};

void finger_st_cmd_start_sensor_data(void);
uint16_t get_data_Finger_GSR(void);

void dev_init_finger(void)
{
//    dev_finger_spi_pin_init();
//    battery_check_init(NRF_SAADC_INPUT_AIN4);
//    finger_st_cmd_start_sensor_data();
//    
//    nrf_gpio_cfg_output(SPI_CS_Pin18);
}

void dev_sample_64hz_in_main_loop_finger(void)
{
    uint8_t tmp_uint8 = 0;
    uint16_t tmp_uint16 = 0;
//    uint32_t tmp_uint32 = 0;
    float tmp_float = 0;
        
    tmp_uint8 = get_battery_percent_val();
    MsgToMaster[8] = tmp_uint8;                         // Battery
    
    if(flag_ble_connected)
    {
        MsgToMaster[0] = 0xFF;                          // Header
        MsgToMaster[1] = 0x02;
    
        MsgToMaster[2] = _rssi_;                        // 2020-01-13 两个字节的 ID 改为 RSSI 和 ID
        MsgToMaster[3] = UserFlashCfg.Pkg.DevID;
        
//        tmp_uint32 = ++pDevInfo->FrameIndex;            // FrameIndex
//        MsgToMaster[4] = tmp_uint32 >> 24;
//        MsgToMaster[5] = tmp_uint32 >> 16;
//        MsgToMaster[6] = tmp_uint32 >> 8;
//        MsgToMaster[7] = tmp_uint32 >> 0;
        
        if(!Got_Finger_ST_Uart_Data)
            finger_st_cmd_start_sensor_data();
        
        tmp_uint16 = DevFinger_ST.bpm;                 // BPM
        MsgToMaster[9] = tmp_uint16 >> 8;
        MsgToMaster[10] = tmp_uint16;
        
        tmp_uint16 = DevFinger_ST.ppg * 4;              // PPG  // 2020-01-11 15:50:28 认为调整了从指尖 ST 收到的 PPG 数据
                                                                // 2020-01-14 10:29:35 结项前暂定由 832 控制放大 4 倍
        MsgToMaster[11] = tmp_uint16 >> 8;
        MsgToMaster[12] = tmp_uint16;
        
        tmp_uint16 = DevFinger_ST.spo2;                // SPO2
        MsgToMaster[13] = tmp_uint16 >> 8;
        MsgToMaster[14] = tmp_uint16;
        
        tmp_float = SON1421_Read_Temperature(9);        // SKT
        tmp_float *= 100;
        tmp_uint16 = tmp_float;
        MsgToMaster[15] = tmp_uint16 >> 8;
        MsgToMaster[16] = tmp_uint16;
        
        tmp_uint16 = get_data_Finger_GSR();             // GSR
        MsgToMaster[17] = tmp_uint16 >> 8;
        MsgToMaster[18] = tmp_uint16;
    }
}

void dev_send_64hz_in_rtc_finger(void)
{
    uint32_t tmp_uint32;
    
    tmp_uint32 = ++pDevInfo->FrameIndex;            // FrameIndex
    MsgToMaster[4] = tmp_uint32 >> 24;
    MsgToMaster[5] = tmp_uint32 >> 16;
    MsgToMaster[6] = tmp_uint32 >> 8;
    MsgToMaster[7] = tmp_uint32 >> 0;
    
    send_data_to_chen();
}

bool uart_rx_handle_finger(uint8_t* p_rx, uint16_t len)
{
    /*
        00  01  02  03  04  05  06  07  08
        5F  07  BH  BL  PH  PL  SH  SL  CRC
        
        BPM PPG SPO2
        CRC = 07 + BH + BL + PH + PL + SH + SL
    */
    
    uint8_t PkgCRC = 0;
    uint16_t i;
    uint16_t tmp_uint16 = 0;
    
    if(1 == len)                // 5F 开头
    {
        if(0x5F != p_rx[0])
            return true;
    }
    else if(2 == len)           // 第二字节为长度 固定为 07
    {
        if(7 != p_rx[1])
            return true;
    }
    else if(9 == len)           // 完整长度 9 字节
    {
        PkgCRC = 0;
        
        for(i=1; i<8; i++)
            PkgCRC += p_rx[i];
        
        if(PkgCRC == p_rx[8])
        {
            Got_Finger_ST_Uart_Data = true;
            
            tmp_uint16 = p_rx[2];
            tmp_uint16 <<= 8;
            tmp_uint16 |= p_rx[3];
            DevFinger_ST.bpm = tmp_uint16;
            
            tmp_uint16 = p_rx[4];
            tmp_uint16 <<= 8;
            tmp_uint16 |= p_rx[5];
            DevFinger_ST.ppg = tmp_uint16;
            
            tmp_uint16 = p_rx[6];
            tmp_uint16 <<= 8;
            tmp_uint16 |= p_rx[7];
            DevFinger_ST.spo2 = tmp_uint16;
        }
        
        return true;
    }
    
    return false;
}

void dev_finger_spi_pin_init(void)
{
    nrf_gpio_cfg_output(SPI_CS_Pin18);
    nrf_gpio_cfg_output(SPI_MO_Pin17);
    nrf_gpio_cfg_output(SPI_CK_Pin15);
    nrf_gpio_cfg_input(SPI_MI_Pin16, NRF_GPIO_PIN_PULLDOWN);
    nrf_gpio_pin_clear(SPI_CK_Pin15);
}

uint16_t dev_finger_ad7685_read(void)
{
    uint16_t DataRead = 0;
    uint8_t i;
    
    nrf_gpio_pin_clear(SPI_CS_Pin18);
    nrf_gpio_pin_set(SPI_MO_Pin17);
    nrf_delay_us(1);
    
    nrf_gpio_pin_set(SPI_CS_Pin18);
    nrf_delay_us(1);
    
    nrf_gpio_pin_set(SPI_MO_Pin17);
    nrf_delay_us(5);
    
    for(i = 0; i < 16; i++)
    {
        nrf_gpio_pin_set(SPI_CK_Pin15);
        
        nrf_gpio_pin_clear(SPI_CS_Pin18);
        DataRead <<= 1;
        
        if(nrf_gpio_pin_read(SPI_MI_Pin16))
            DataRead = DataRead | 0x01;
            
        nrf_gpio_pin_clear(SPI_CK_Pin15);
    }
    
    nrf_gpio_pin_set(SPI_MO_Pin17);
    nrf_gpio_pin_clear(SPI_MO_Pin17);
    nrf_gpio_pin_set(SPI_CS_Pin18);
    
    return DataRead;
}

void finger_st_cmd_start_sensor_data(void)
{
    static uint8_t Cmd_St[4] = {0x43, 0x01, 0x02, 0x40};
    uint8_t i;
    
    for(i=0; i<4; i++)
        app_uart_put(Cmd_St[i]);
}

// uint16_t get_data_Wrist_GSR(void)
uint16_t get_data_Finger_GSR(void)
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
    
    GSR_ADC = dev_finger_ad7685_read();
    
    //log_dbg("gsr_adc = %d\r\n", GSR_ADC);
    
    if(GSR_ADC >= 0xF3C0)
        return 0;               // human <= 10M,10M is F3C
    
    /*
          ((2 * 65536) / ADC) - 2
        = ((2 * 65536) / ADC) - (2 * (ADC / ADC))
        = ((2 * 65536) - (2 * ADC)) / ADC
        = 2((65536 - ADC) / ADC)
    */
    GSR_Val = 65536 - (GSR_ADC);
    GSR_Val = 2000 * GSR_Val;           // 此处将 GSR 的原始值放大了 1000 倍
    GSR_Val = GSR_Val / GSR_ADC;
    
    if(abs(GSR_Val - GSR_Last) > 21000)
        GSR_Val = GSR_Last;                // 过滤掉皮电的尖峰
        
    else
        GSR_Last = GSR_Val ;
        
    if(GSR_Val >= 0xFFFF)
        GSR_Val = 0xFFFF;
        
    return (uint16_t)GSR_Val;
}

