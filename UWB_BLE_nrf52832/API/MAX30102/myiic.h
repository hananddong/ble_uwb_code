#ifndef __MYIIC_H__
#define __MYIIC_H__

#include "stdint.h"

#define SDA_PIN     7
#define SCL_PIN     8

#define _PIN        25

#define SDA_HIGH(n)     nrf_gpio_pin_set(n)
#define SDA_LOW(n)      nrf_gpio_pin_clear(n)

#define SCL_HIGH(n)     nrf_gpio_pin_set(n)
#define SCL_LOW(n)      nrf_gpio_pin_clear(n)
#define _PIN_HIGH(n)    nrf_gpio_pin_set(n)
#define _PIN_LOW(n)     nrf_gpio_pin_clear(n)

void i2c_Start(void);
void i2c_Stop(void);
void i2c_SendByte(uint8_t _ucByte);
uint8_t i2c_ReadByte(void);
uint8_t i2c_WaitAck(void);
void i2c_Ack(void);
void i2c_NAck(void);
void i2c_Delay(void);
void SDA_Input_mode(void);
void SDA_Output_mode(void);
void SCL_Output_mode(void);
void Output_mode(void);

#endif  // __MYIIC_H__

