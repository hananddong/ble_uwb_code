#ifndef	__DEV_H__
#define __DEV_H__

#include "tag.h"
#include "stdint.h"
#include "user_cfg.h"
#include "stdbool.h"

typedef void (*p_dev_init)(void);
typedef void (*p_tag_data_profess_64hz_in_main_loop)(void);
typedef void (*p_dev_sample_256hz_in_main_loop)(void);
typedef void (*p_dev_send_16hz_in_rtc)(void);
typedef void (*p_dev_send_64hz_in_rtc)(void);
typedef bool (*p_uart_rx_handle)(uint8_t* p_rx, uint16_t len);

//#pragma pack(1)
//#pragma pack()

typedef struct
{
    uint8_t DevType;
//    uint8_t NameIndex;      // 2020-01-13 17:51:34 修改
//    uint8_t DevID;
    uint8_t RptFreq;
    uint8_t Battery;
    uint8_t Amp;
    uint32_t FrameIndex;
}DevInfo_t;

extern DevInfo_t* pDevInfo;

#define MSG_TO_MASTER_LEN   (32)

#if (Dev_Chest == CFG_DevType)
    #define PIN_BAT_ADC                     (NRF_SAADC_INPUT_AIN2)  // 电量检测引脚
    
    #define STA_LED_PIN                     (20)
    #define STA_LED_LEV_ON                  (0)

    #define UART_PIN_RX                     8
    #define UART_PIN_TX                     6
//    #define UART_PIN_RX                     10
//    #define UART_PIN_TX                     12
    #define UART_BAUD                      NRF_UART_BAUDRATE_4800
    
    #define IIC_SCL_MAX17043                25
    #define IIC_SDA_MAX17043                26
    
    #define IIC_SCL_PIN                     IIC_SCL_MAX17043
    #define IIC_SDA_PIN                     IIC_SDA_MAX17043
    
    #define EN_Get_Power_PB_Sta       1
    
    #define EN_POWER_CTRL
    #define POWER_CTRL_INT                  16
    #define POWER_CTRL_PH_HOLD              15

//    #define Pin_SPI_MI                      26
//    #define Pin_SPI_MO                      28
//    #define Pin_SPI_CK                      27

#elif (Dev_Wrist == CFG_DevType)
    #define PIN_BAT_ADC                     (NRF_SAADC_INPUT_AIN4)  // 电量检测引脚
    
    #define STA_LED_PIN                     (19)
    #define STA_LED_LEV_ON                  (1)

    #define UART_PIN_RX                     27
    #define UART_PIN_TX                     26
    #define UART_BAUD                       NRF_UART_BAUDRATE_115200
    
    #define IIC_SCL_Wrist_MPU9250_Pin30     30
    #define IIC_SDA_Wrist_MPU9250_Pin29     29
    
    #define IIC_SCL_PIN                     IIC_SCL_Wrist_MPU9250_Pin30
    #define IIC_SDA_PIN                     IIC_SDA_Wrist_MPU9250_Pin29
    
#elif (Dev_Finger == CFG_DevType)
    #define PIN_BAT_ADC                     (NRF_SAADC_INPUT_AIN4)  // 电量检测引脚
    
    #define STA_LED_PIN                     (19)
    #define STA_LED_LEV_ON                  (0)

    #define UART_PIN_RX                     7
    #define UART_PIN_TX                     6
    #define UART_BAUD                       NRF_UART_BAUDRATE_115200
    
    #define IIC_SCL_PIN                     27
    #define IIC_SDA_PIN                     26
    
#elif (Dev_Ear == CFG_DevType)
    #define PIN_BAT_ADC                     (NRF_SAADC_INPUT_AIN5)  // 电量检测引脚
    
    #define STA_LED_PIN                     (17)
    #define STA_LED_LEV_ON                  (1)

    #define UART_PIN_RX                     8
    #define UART_PIN_TX                     6
    #define UART_BAUD                       NRF_UART_BAUDRATE_115200
    
    #define IIC_SCL_PIN                     27
    #define IIC_SDA_PIN                     26

#elif (Dev_Wrist_V3 == CFG_DevType)
    #define PIN_BAT_ADC                     (NRF_SAADC_INPUT_AIN4)  // 电量检测引脚
    
    #define STA_LED_PIN                     (18)
    #define STA_LED_LEV_ON                  (1)

    #define UART_PIN_RX                     27
    #define UART_PIN_TX                     26
    #define UART_BAUD                       NRF_UART_BAUDRATE_115200
    
    #define EN_POWER_CTRL
    #define POWER_CTRL_INT                  22
    #define POWER_CTRL_PH_HOLD              23
    
    #define IIC_SCL_Wrist_MPU9250_Pin30     30
    #define IIC_SDA_Wrist_MPU9250_Pin29     29
    
    #define IIC_SCL_PIN                     IIC_SCL_Wrist_MPU9250_Pin30
    #define IIC_SDA_PIN                     IIC_SDA_Wrist_MPU9250_Pin29
    
#elif (Dev_Ear_V2 == CFG_DevType)
    #define PIN_BAT_ADC                     (NRF_SAADC_INPUT_AIN5)  // 电量检测引脚
    
    #define STA_LED_PIN                     (18)
    #define STA_LED_LEV_ON                  (1)

    #define UART_PIN_RX                     27
    #define UART_PIN_TX                     26
    #define UART_BAUD                       NRF_UART_BAUDRATE_115200
    
    #define IIC_SCL_PIN                     27
    #define IIC_SDA_PIN                     26
    
    #define EN_POWER_CTRL
    #define POWER_CTRL_INT                  22
    #define POWER_CTRL_PH_HOLD              23
    
#elif (Dev_Test_MaxSpeed == CFG_DevType)
    // Test max speed
    #define PIN_BAT_ADC                     (NRF_SAADC_INPUT_AIN5)  // 电量检测引脚
    
    #define STA_LED_PIN                     (2)
    #define STA_LED_LEV_ON                  (0)

    #define UART_PIN_RX                     27
    #define UART_PIN_TX                     26
    #define UART_BAUD                       NRF_UART_BAUDRATE_115200
    
    #define IIC_SCL_PIN                     27
    #define IIC_SDA_PIN                     26

#elif (Dev_Indoor_Localization == CFG_DevType)
    #define PIN_BAT_ADC                     (NRF_SAADC_INPUT_AIN2)  // 电量检测引脚
    
    #define DEBUG_LED_PIN                   (17)
    #define STA_LED_PIN                     (20)
    #define STA_LED_LEV_ON                  (0)


    #define UART_PIN_RX                     6 //8   6
    #define UART_PIN_TX                     7 //6   7


//    #define UART_PIN_RX                     10
//    #define UART_PIN_TX                     12
    #define UART_BAUD                      NRF_UART_BAUDRATE_115200
    
    #define IIC_SCL_MAX17043                25
    #define IIC_SDA_MAX17043                26
    
    #define IIC_SCL_PIN                     IIC_SCL_MAX17043
    #define IIC_SDA_PIN                     IIC_SDA_MAX17043
    
    #define EN_Get_Power_PB_Sta       1
    
    #define EN_POWER_CTRL
    #define POWER_CTRL_INT                  16
    #define POWER_CTRL_PH_HOLD              15

    #define LED_DEBUG_Off     do{ nrf_gpio_pin_set(DEBUG_LED_PIN);      }while(0)
    #define LED_DEBUG_On      do{ nrf_gpio_pin_clear(DEBUG_LED_PIN);    }while(0)
    #define LED_DEBUG_Toogle  do{ nrf_gpio_pin_toggle(DEBUG_LED_PIN);   }while(0)
//    #define Pin_SPI_MI                      26
//    #define Pin_SPI_MO                      28
//    #define Pin_SPI_CK                      27
#else
    #error "you must set CFG_DevType and CFG_DevID"
    
#endif  // (Dev_Chest == CFG_DevType)

#if EN_Get_Power_PB_Sta
    extern bool PB_Press_Sta;
#endif  // EN_Get_Power_PB_Sta

extern volatile uint8_t MsgToMaster[MSG_TO_MASTER_LEN];
extern uint8_t _rssi_;

extern p_dev_init                       dev_init;
extern p_tag_data_profess_64hz_in_main_loop   tag_data_profess_64hz_in_main_loop;
extern p_dev_sample_256hz_in_main_loop  dev_sample_256hz_in_main_loop;
extern p_dev_send_16hz_in_rtc           dev_send_16hz_in_rtc;
extern p_dev_send_64hz_in_rtc           dev_send_64hz_in_rtc;
extern p_uart_rx_handle                 uart_rx_handle;

extern void do_nothing(void);
extern bool do_nothing_uart(uint8_t* p_rx, uint16_t len);
extern void msg_to_master_clear(void);

extern void power_en_ctrl_start(void);
extern void sta_led_init(void);
extern void sta_led_ms_service(void);

#endif	// __DEV_H__

