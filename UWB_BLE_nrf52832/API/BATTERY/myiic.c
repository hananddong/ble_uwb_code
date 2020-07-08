#include "myiic.h"
#include "nrf_gpio.h"

void i2c_Delay(void)
{
    #define  IIC_DELAY_TIME 5
    uint8_t i;
    
    /*  　
        CPU主频168MHz时，在内部Flash运行, MDK工程不优化。用台式示波器观测波形。
        循环次数为5时，SCL频率 = 1.78MHz (读耗时: 92ms, 读写正常，但是用示波器探头碰上就读写失败。时序接近临界)
        循环次数为10时，SCL频率 = 1.1MHz (读耗时: 138ms, 读速度: 118724B/s)
        循环次数为30时，SCL频率 = 440KHz， SCL高电平时间1.0us，SCL低电平时间1.2us
    
        上拉电阻选择2.2K欧时，SCL上升沿时间约0.5us，如果选4.7K欧，则上升沿约1us
    
        实际应用选择400KHz左右的速率即可
    */
    for(i = 0; i < IIC_DELAY_TIME; i++);
}

void SDA_Input_mode(void)
{
    nrf_gpio_cfg_input(IIC_SDA_PIN, NRF_GPIO_PIN_PULLUP);
}

void SDA_Output_mode(void)
{
    nrf_gpio_cfg_output(IIC_SDA_PIN);
}

void SCL_Output_mode(void)
{
    nrf_gpio_cfg_output(IIC_SCL_PIN);
}

void Output_mode(void)
{
    nrf_gpio_cfg_output(_PIN);
}

void i2c_Start(void)
{
    SDA_Output_mode();     //sda线输出
    
    SDA_HIGH(IIC_SDA_PIN);
    
    SCL_HIGH(IIC_SCL_PIN);
    
    i2c_Delay();
    SDA_LOW(IIC_SDA_PIN);//START:when CLK is high,DATA change form high to low
    i2c_Delay();
    
    SCL_LOW(IIC_SCL_PIN);//钳住I2C总线，准备发送或接收数据
}

void i2c_Stop(void)
{
    SDA_Output_mode();//sda线输出
    
    SCL_LOW(IIC_SCL_PIN);
    
    SDA_LOW(IIC_SDA_PIN);//STOP:when CLK is high DATA change form low to high
    i2c_Delay();
    
    SCL_HIGH(IIC_SCL_PIN);
    
    SDA_HIGH(IIC_SDA_PIN);//发送I2C总线结束信号
    i2c_Delay();
}

void i2c_SendByte(uint8_t _ucByte)
{
    uint8_t t;
    SDA_Output_mode();
    
    SCL_LOW(IIC_SCL_PIN);//拉低时钟开始数据传输
    
    for(t = 0; t < 8; t++)
    {
        if(_ucByte & 0x80)
            SDA_HIGH(IIC_SDA_PIN);
            
        else
            SDA_LOW(IIC_SDA_PIN);
            
        i2c_Delay();
        
        SCL_HIGH(IIC_SCL_PIN);
        
        i2c_Delay();
        
        SCL_LOW(IIC_SCL_PIN);
        
        i2c_Delay();
        _ucByte <<= 1;
    }
}

uint8_t i2c_ReadByte_2(uint8_t ack)
{
    uint8_t i = 0;
    uint8_t value = 0;
    SDA_Input_mode();
    
    for(i = 0; i < 8 ; i++)
    {
        SCL_LOW(IIC_SCL_PIN);
        i2c_Delay();
        SCL_HIGH(IIC_SCL_PIN);
        value <<= 1;
        
        if(nrf_gpio_pin_read(IIC_SDA_PIN))
            value++;
            
        i2c_Delay();
    }
	if(!ack){
		i2c_NAck();
	}
	else{
		i2c_Ack();
	}
    
    return value;
}

uint8_t i2c_ReadByte(void)
{
    uint8_t i = 0;
    uint8_t value = 0;
    SDA_Input_mode();
    
    for(i = 0; i < 8 ; i++)
    {
        SCL_LOW(IIC_SCL_PIN);
        i2c_Delay();
        SCL_HIGH(IIC_SCL_PIN);
        value <<= 1;
        
        if(nrf_gpio_pin_read(IIC_SDA_PIN))
            value++;
            
        i2c_Delay();
    }
    
    return value;
}

uint8_t i2c_WaitAck(void)
{
    uint8_t  delay_timer = 0;
      
    SDA_HIGH(IIC_SDA_PIN);  //SDA_HIGH(SDA_PIN);
    i2c_Delay();
	SDA_Input_mode();
    SCL_HIGH(IIC_SCL_PIN);
    i2c_Delay();
    
    while(nrf_gpio_pin_read(IIC_SDA_PIN))
    {
        delay_timer++;
        
        if(delay_timer >= 250)
        {
            i2c_Stop();
            return 1;
        }
    }
    
    SCL_LOW(IIC_SCL_PIN);
    i2c_Delay();
    
    return 0;
}

void i2c_Ack(void)
{
    SCL_LOW(IIC_SCL_PIN);
    
    SDA_Output_mode();
    SDA_LOW(IIC_SDA_PIN);
    i2c_Delay();
    
    SCL_HIGH(IIC_SCL_PIN);
    
    i2c_Delay();
    
    SCL_LOW(IIC_SCL_PIN);
}

void i2c_NAck(void)
{
    SCL_LOW(IIC_SCL_PIN);
    
    SDA_Output_mode();
    SDA_HIGH(IIC_SDA_PIN);
    i2c_Delay();
    
    SCL_HIGH(IIC_SCL_PIN);
    i2c_Delay();
    SCL_LOW(IIC_SCL_PIN);
}

