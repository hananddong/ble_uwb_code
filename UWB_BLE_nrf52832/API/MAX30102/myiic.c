#include "myiic.h"
#include "nrf_gpio.h"

void i2c_Delay(void)
{
    uint8_t i;
    
    /*  ��
        CPU��Ƶ168MHzʱ�����ڲ�Flash����, MDK���̲��Ż�����̨ʽʾ�����۲Ⲩ�Ρ�
        ѭ������Ϊ5ʱ��SCLƵ�� = 1.78MHz (����ʱ: 92ms, ��д������������ʾ����̽ͷ���ϾͶ�дʧ�ܡ�ʱ��ӽ��ٽ�)
        ѭ������Ϊ10ʱ��SCLƵ�� = 1.1MHz (����ʱ: 138ms, ���ٶ�: 118724B/s)
        ѭ������Ϊ30ʱ��SCLƵ�� = 440KHz�� SCL�ߵ�ƽʱ��1.0us��SCL�͵�ƽʱ��1.2us
    
        ��������ѡ��2.2Kŷʱ��SCL������ʱ��Լ0.5us�����ѡ4.7Kŷ����������Լ1us
    
        ʵ��Ӧ��ѡ��400KHz���ҵ����ʼ���
    */
    for(i = 0; i < 10; i++);
}

void SDA_Input_mode(void)
{
    nrf_gpio_cfg_input(SDA_PIN, NRF_GPIO_PIN_NOPULL);
}

void SDA_Output_mode(void)
{
    nrf_gpio_cfg_output(SDA_PIN);
}

void SCL_Output_mode(void)
{
    nrf_gpio_cfg_output(SCL_PIN);
}

void Output_mode(void)
{
    nrf_gpio_cfg_output(_PIN);
}

void i2c_Start(void)
{
    SDA_Output_mode();     //sda�����
    
    SDA_HIGH(SDA_PIN);
    
    SCL_HIGH(SCL_PIN);
    
    i2c_Delay();
    SDA_LOW(SDA_PIN);//START:when CLK is high,DATA change form high to low
    i2c_Delay();
    
    SCL_LOW(SCL_PIN);//ǯסI2C���ߣ�׼�����ͻ��������
}

void i2c_Stop(void)
{
    SDA_Output_mode();//sda�����
    
    SCL_LOW(SCL_PIN);
    
    SDA_LOW(SDA_PIN);//STOP:when CLK is high DATA change form low to high
    i2c_Delay();
    
    SCL_HIGH(SCL_PIN);
    
    SDA_HIGH(SDA_PIN);//����I2C���߽����ź�
    i2c_Delay();
}

void i2c_SendByte(uint8_t _ucByte)
{
    uint8_t t;
    SDA_Output_mode();
    
    SCL_LOW(SCL_PIN);//����ʱ�ӿ�ʼ���ݴ���
    
    for(t = 0; t < 8; t++)
    {
        if(_ucByte & 0x80)
            SDA_HIGH(SDA_PIN);
            
        else
            SDA_LOW(SDA_PIN);
            
        i2c_Delay();
        
        SCL_HIGH(SCL_PIN);
        
        i2c_Delay();
        
        SCL_LOW(SCL_PIN);
        
        i2c_Delay();
        _ucByte <<= 1;
    }
}

uint8_t i2c_ReadByte(void)
{
    uint8_t i = 0;
    uint8_t value = 0;
    SDA_Input_mode();
    
    for(i = 0; i < 8 ; i++)
    {
        SCL_LOW(SCL_PIN);
        i2c_Delay();
        SCL_HIGH(SCL_PIN);
        value <<= 1;
        
        if(nrf_gpio_pin_read(SDA_PIN))
            value++;
            
        i2c_Delay();
    }
    
    return value;
}

uint8_t i2c_WaitAck(void)
{
    uint8_t  delay_timer = 0;
    
    SDA_Input_mode();
    
    SCL_HIGH(SCL_PIN);
    
    i2c_Delay();
    SDA_HIGH(SDA_PIN);
    i2c_Delay();
    
    while(nrf_gpio_pin_read(SDA_PIN))
    {
        delay_timer++;
        
        if(delay_timer >= 200)
        {
            i2c_Stop();
            return 1;
        }
    }
    
    SCL_LOW(SCL_PIN);
    i2c_Delay();
    
    return 0;
}

void i2c_Ack(void)
{
    SCL_LOW(SCL_PIN);
    
    SDA_Output_mode();
    SDA_LOW(SDA_PIN);
    i2c_Delay();
    
    SCL_HIGH(SCL_PIN);
    
    i2c_Delay();
    
    SCL_LOW(SCL_PIN);
}

void i2c_NAck(void)
{
    SCL_LOW(SCL_PIN);
    
    SDA_Output_mode();
    SDA_HIGH(SDA_PIN);
    i2c_Delay();
    
    SCL_HIGH(SCL_PIN);
    i2c_Delay();
    SCL_LOW(SCL_PIN);
}

