#include "battery.h"
#include "dev.h"
#include "user_log.h"
#include "nrf_drv_saadc.h"
#include "myiic.h"

uint8_t BatPct_uint8 = 0;


#ifdef   USE_SSADC

    #define SAMPLES_BUFFER_LEN 2

    static nrf_saadc_value_t m_buffer_pool[2][SAMPLES_BUFFER_LEN];


    float BatPct_float = 0.0;

    uint16_t Bat_ADC_Raw = 0;

    #define BatPct_Chart_Len        (230)

    const uint8_t BatPct_Chart[BatPct_Chart_Len] =
    {
         0,    0,   1,   1,   1,   1,   1,   1,   1,   1,
         1,    1,   1,   1,   1,   1,   1,   1,   1,   1,
         2,    2,   2,   2,   2,   2,   2,   2,   2,   2,
         2,    2,   3,   3,   3,   3,   3,   3,   3,   3,
         3,    4,   4,   4,   4,   4,   4,   4,   4,   4,
         5,    5,   5,   5,   5,   6,   6,   6,   6,   6,
         6,    6,   6,   7,   7,   7,   7,   7,   7,   8,
         8,    8,   8,   8,   8,   8,   9,   9,   9,   9,
         10,  10,  10,  10,  10,  10,  11,  11,  11,  11,
         12,  12,  12,  12,  13,  13,  13,  13,  14,  14,
         14,  15,  15,  15,  15,  16,  16,  17,  17,  18,
         18,  19,  19,  19,  20,  21,  21,  22,  23,  23,
         24,  24,  25,  26,  27,  28,  29,  30,  31,  32,
         33,  34,  36,  37,  38,  41,  42,  43,  49,  51,
         51,  51,  52,  52,  57,  57,  63,  65,  65,  67,
         70,  71,  74,  74,  80,  82,  82,  84,  86,  91,
         91,  98, 100, 101, 106, 106, 106, 115, 118, 122,
        126, 129, 133, 140, 141, 144, 148, 149, 154, 155,
        155, 156, 161, 164, 167, 170, 172, 173, 175, 176,
        177, 179, 182, 184, 185, 188, 189, 189, 192, 195,
        198, 199, 202, 205, 206, 207, 212, 214, 220, 220,
        225, 230, 231, 231, 234, 235, 239, 240, 242, 243,
        244, 245, 246, 246, 247, 249, 249, 249, 250, 250
    };

    void saadc_callback_battery(nrfx_saadc_evt_t const *p_event)
    {
        if(p_event->type == NRFX_SAADC_EVT_DONE)
        {
            ret_code_t err_code;
            int i;
            
            err_code = nrfx_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_BUFFER_LEN);
            APP_ERROR_CHECK(err_code);
            
            for(i = 0; i < SAMPLES_BUFFER_LEN; i++)
            {
                Bat_ADC_Raw = p_event->data.done.p_buffer[i];
            }
        }
    }

    uint8_t battery_adc_to_percent(void)
    {
        #define Bat_ADC_Raw_Buf_Len         (50)
        
        #define Bat_Pct_100_Voltage_mV      (4200.0f)
        #define Bat_Pct_000_Voltage_mV      (3300.0f)
        
        #define Bat_Pct_100_ADC_Avg         (3360)
        #define Bat_Pct_000_ADC_Avg         (2620)
        
        static uint8_t i;
        static uint32_t Bat_ADC_Raw_Sum = 0;
        static uint16_t Bat_ADC_Raw_Buf[Bat_ADC_Raw_Buf_Len] = {0};
        uint16_t Bat_ADC_Avg = 0;
        float Bat_Voltage_mV = 0.0f;
        float BatPct_Decimal = 0.0;
        uint8_t BatPct_ChartVal = 0;
        
        log_dbg_bat("ADC_Raw=%04d", Bat_ADC_Raw);
        
        for(i=0; i<Bat_ADC_Raw_Buf_Len-1; i++)
            Bat_ADC_Raw_Buf[i] = Bat_ADC_Raw_Buf[i+1];
        
        Bat_ADC_Raw_Buf[i] = Bat_ADC_Raw;
        
        Bat_ADC_Raw_Sum = 0;
        
        for(i=0; i<Bat_ADC_Raw_Buf_Len; i++)
            Bat_ADC_Raw_Sum += Bat_ADC_Raw_Buf[i];
        
        Bat_ADC_Avg = Bat_ADC_Raw_Sum / Bat_ADC_Raw_Buf_Len;
        
        log_dbg_bat("  ADC_Avg=%04d", Bat_ADC_Avg);
        
        // 用两点式直线公式 根据 ADC 求 电压值 A(Bat_Pct_000_ADC_Avg, Bat_Pct_000_Voltage_mV) B(Bat_Pct_100_ADC_Avg, Bat_Pct_100_Voltage_mV)
        Bat_Voltage_mV =    (       (Bat_Pct_100_Voltage_mV - Bat_Pct_000_Voltage_mV)
                                /   (Bat_Pct_100_ADC_Avg - Bat_Pct_000_ADC_Avg)         )
                        *   (Bat_ADC_Avg - Bat_Pct_000_ADC_Avg)
                        +   Bat_Pct_000_Voltage_mV;
        
        log_dbg_bat("  Bat_mV = %5.2f", Bat_Voltage_mV);

        if(Bat_Voltage_mV >= Bat_Pct_100_Voltage_mV)        // 电池电压大于等于 100% 点的电压 则认为满电
            BatPct_ChartVal = 250;
        else if(Bat_Voltage_mV >= Bat_Pct_000_Voltage_mV)   // 电池电压大于等于 0% 点的电压 则通过查表得到电量
            BatPct_ChartVal = BatPct_Chart[ (uint8_t)(      (       (BatPct_Chart_Len - 1)
                                                                *   ((float)Bat_ADC_Avg - (float)Bat_Pct_000_ADC_Avg))
                                                        /   ((float)Bat_Pct_100_ADC_Avg - (float)Bat_Pct_000_ADC_Avg)   )   ];
        else                                                // 电池电压低于 0% 点 均视为没电了
            BatPct_ChartVal = 0;
        
        BatPct_float = BatPct_ChartVal * 100.0f / 250;      // 电量百分比 浮点型值
        BatPct_uint8 = BatPct_float;                        // 电量百分比 整数部分值   
        BatPct_Decimal = BatPct_float - BatPct_uint8;       // 点亮百分比 小数部分值
        
        if(BatPct_Decimal >= 0.5f)                          // 做四舍五入
        {
            BatPct_uint8 += 1;
            if(BatPct_uint8 > 100)
                BatPct_uint8 = 100;
        }
        
        log_dbg_bat("  Bat_Pct= %03d%%  Rpt=%03d\r\n", BatPct_uint8, BatPct_ChartVal);
        
        return BatPct_ChartVal;
    }

    void battery_ssadc_init(nrf_saadc_input_t ch)
    {
        ret_code_t err_code;
        uint8_t i;
        
        for(i=0; i<BatPct_Chart_Len; i++)
        {
            log_dbg_bat("i=%03d    ChartVal=%03d    Pct=%03d%%\r\n", i, BatPct_Chart[i], (uint8_t)(BatPct_Chart[i] * 100.0f / 250));
        }
        
        log_dbg_bat("\r\n\r\n");
        
        nrf_saadc_channel_config_t channel_config = NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(ch);
        
        err_code = nrf_drv_saadc_init(NULL, saadc_callback_battery);
        APP_ERROR_CHECK(err_code);
        
        NRF_SAADC->RESOLUTION = SAADC_RESOLUTION_VAL_12bit;         // 强制改为12位分辨率
        err_code = nrfx_saadc_channel_init(ch-1, &channel_config);  // 注意这里的参数要和选择的通道对应
        APP_ERROR_CHECK(err_code);
        
        err_code = nrfx_saadc_buffer_convert(m_buffer_pool[0], SAMPLES_BUFFER_LEN);
        APP_ERROR_CHECK(err_code);
        
        err_code = nrfx_saadc_buffer_convert(m_buffer_pool[1], SAMPLES_BUFFER_LEN);
        APP_ERROR_CHECK(err_code);
    }



    void set_battery_percent_for_test(void)
    {
        BatPct_uint8 = 250 / 2;
    }

#endif  // USE_SSADC
    
    
#ifdef  USE_MAX17043_IIC

    void max17403_iic_gpio_init()
    {
        nrf_gpio_cfg_output(IIC_SDA_PIN);
        nrf_gpio_cfg_output(IIC_SCL_PIN);
    }

    void MAX_17043_init(void)
    {
        Software_Max17043Write(0xfe,0x54,0x00);
        Software_Max17043Write(0x0c,0x97,0x1f);
        Software_Max17043Write(0x06,0x40,0x00);
    }

    void Software_Max17043Read(uint8_t Max17043_Address,uint8_t *Data)
    {
        uint8_t IIC_bufferH=0x00;
        uint8_t IIC_bufferL=0x00;
        
        i2c_Start();
        i2c_SendByte(0x6c);
        while(i2c_WaitAck())
        {
            log_msg("IIC NO ack!\r\n");
        }
        i2c_SendByte(Max17043_Address);
        while(i2c_WaitAck());
        i2c_Start();
        i2c_SendByte(0x6d);
        while(i2c_WaitAck())
        {
            log_msg("IIC NO ack!\r\n");
        }
        
        IIC_bufferH=i2c_ReadByte_2(1);
        IIC_bufferL=i2c_ReadByte_2(0);
        i2c_Stop();
        
        Data[0]=IIC_bufferH;
        Data[1]=IIC_bufferL;
    }

    void Software_Max17043Write(uint8_t Max17043_Address,uint8_t Data0,uint8_t Data1)
    {
        i2c_Start();
        i2c_SendByte(0x6c);
        while(i2c_WaitAck());
        i2c_SendByte(Max17043_Address);
        
        while(i2c_WaitAck());
        i2c_SendByte(Data0);
        while(i2c_WaitAck());
        i2c_SendByte(Data1);
        while(i2c_WaitAck());
        
        i2c_Stop();
    }

    void Software_Max17043_awaken()
    {
        SCL_LOW(IIC_SCL_PIN);
        nrf_delay_us(2);
        SCL_HIGH(IIC_SCL_PIN);
    }

    float Software_Max17043_ReadSOC(void)
    {
        uint8_t Buffer[2];
        Software_Max17043Read(SOC_REGISTER,Buffer);
        return (float)Buffer[1]/256+(float)Buffer[0];
    }
    uint8_t Software_Max17043_readVoltage()
    {
        #define RATE_OF_POWER_DATA  (255/100)
        uint8_t Pc_Bat_percent;
        uint8_t Buffer[2];
        uint16_t Voltage_int;
        float voltage_float,voltage_percent;
        
        Software_Max17043Read(VELL_REGISTER,Buffer);
        voltage_percent = Software_Max17043_ReadSOC();
        
        Voltage_int = ((Buffer[0]<<8)|Buffer[1])>>4;
        voltage_float = 1.25*Voltage_int;
        
        #if defined (lichao_test)
        if(voltage_float >= 4100.0f)
        {
            BatPct_uint8 = 100;
            Pc_Bat_percent = 250;
        }
        else if(voltage_float >=3350.0f && voltage_float <=3450.0f)  
        {
            BatPct_uint8 = 4;
            Pc_Bat_percent = 10;
        }
        else if(voltage_float >=3250.0f && voltage_float <3350.0f)
        {
            BatPct_uint8 = 2;
            Pc_Bat_percent = 6;
        }
        else if(voltage_float > 3180.0f && voltage_float <3250.0f)
        {
            BatPct_uint8 = 1;
            Pc_Bat_percent = 2;
        }
        else if(voltage_float <= 3180.0f)
        {
            BatPct_uint8 = 0;
            Pc_Bat_percent = 0;
            //nrf_gpio_pin_clear(POWER_CTRL_PH_HOLD);  // 电量过低,关闭电源
        }
        else
        {
            BatPct_uint8 = voltage_percent;
            Pc_Bat_percent = (uint8_t)(250*voltage_percent/100 + 10.5f);
        }	
        #else
        BatPct_uint8 = (uint8_t)voltage_percent;  // get_battery_percent_val_real(); 函数的中的返回值在这里更新 
        Pc_Bat_percent = voltage_percent*RATE_OF_POWER_DATA;
        //log_msg("voltage_float is %f  Pc_Bat_percent %d  BatPct_uint8 is %d  \r\n",voltage_float,Pc_Bat_percent,BatPct_uint8);
        //log_msg("BAT %d\r\n",BatPct_uint8);
        #endif
        //log_finger_bat("voltage = %f,percent = %f, pc_percent = %d\r\n",voltage_float,voltage_percent,Pc_Bat_percent);
        return Pc_Bat_percent;
    }


#endif //USE_MAX17043_IIC
    

void battery_check_init(void)
{
    #ifdef   USE_SSADC
    battery_ssadc_init(PIN_BAT_ADC);
    #endif   //USE_SSADC
    
    #ifdef  USE_MAX17043_IIC
    max17403_iic_gpio_init();
    MAX_17043_init();
    #endif //USE_MAX17043_IIC
        
}

uint8_t get_battery_percent_val(void)
{
    #ifdef   USE_SSADC
    nrfx_saadc_sample();                // 启动电池监测 ADC 转换
    return battery_adc_to_percent();
    #endif   //USE_SSADC
    
    #ifdef  USE_MAX17043_IIC
    return Software_Max17043_readVoltage();
    #endif //USE_MAX17043_IIC
}

float get_battery_percent_val_real(void)
{
       
    return BatPct_uint8; // 数值在Software_Max17043_readVoltage();函数中更新。

}










