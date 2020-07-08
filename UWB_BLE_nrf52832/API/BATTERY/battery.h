#ifndef __BATTERY_H__
#define __BATTERY_H__

#include "stdint.h"
#include "nrf_saadc.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
//#define USE_SSADC
#define  USE_MAX17043_IIC

#ifdef   USE_SSADC

    extern void battery_ssadc_init(nrf_saadc_input_t ch);
    extern void set_battery_percent_for_test(void);
    
#endif // USE_SSADC

#ifdef  USE_MAX17043_IIC

    #define VELL_REGISTER       0x02
    #define  SOC_REGISTER       0x04
    #define MODE_REGISTER       0x06
    #define VERSION_REGISTER    0x08
    #define  CONFIG_REGISTER    0x0C
    #define COMMAND_REGISTER    0xFE
    /*****************************************
     IIC_SCL    PIN22 
     IIC_SDA    PIN23 	 
    *****************************************/
    extern void max17403_iic_gpio_init(void);
    extern void MAX_17043_init(void);
    extern void  Software_Max17043Read(uint8_t Max17043_Address,uint8_t *Data);
    extern void  Software_Max17043Write(uint8_t Max17043_Address,uint8_t Data0,uint8_t Data1);
    extern float Software_Max17043_ReadSOC(void);
    extern uint8_t Software_Max17043_readVoltage(void);
    extern void  Software_Max17043_awaken(void);

#endif //USE_MAX17043_IIC

extern uint8_t get_battery_percent_val(void);
extern void battery_check_init(void);
extern float get_battery_percent_val_real(void);
#endif

