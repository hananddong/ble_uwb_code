#include "drv_ads1292.h"
#include "nrf_drv_spi.h"
#include "dev.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "sdk_config.h"
#include "user_log.h"

#define Pin_SPI_MI      26
#define Pin_SPI_MO      28
#define Pin_SPI_CK      27

void spi_event_handler(nrf_drv_spi_evt_t const *p_event, void *p_context);

//static const nrf_drv_spi_t SPI_1 = NRF_DRV_SPI_INSTANCE(1); // SPI instance
//static const nrf_drv_spi_t SPI_0 = NRF_DRV_SPI_INSTANCE(0); // SPI instance
static volatile bool spi_xfer_done;                         // Flag used to indicate that SPI instance completed the transfer

uint8_t m_tx_buf[9] = {0};
uint8_t m_rx_buf[9] = {0};

void drv_spi_init(void)
{
//    nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
//    
//    spi_config.mode = NRF_DRV_SPI_MODE_1;   // 
//    spi_config.frequency = NRF_DRV_SPI_FREQ_500K;//NRF_DRV_SPI_FREQ_4M;
//    spi_config.ss_pin   = NRF_DRV_SPI_PIN_NOT_USED;
//    spi_config.miso_pin = Pin_SPI_MI;
//    spi_config.mosi_pin = Pin_SPI_MO;
//    spi_config.sck_pin  = Pin_SPI_CK;
//    
//    APP_ERROR_CHECK(nrf_drv_spi_init(&SPI_1, &spi_config, spi_event_handler, NULL));
    
    
    
    #define MYCONFIG   *((volatile unsigned int *) 0x554)
    
    nrf_gpio_cfg_input(Pin_SPI_CK, NRF_GPIO_PIN_PULLUP);
    nrf_gpio_cfg_input(Pin_SPI_MO, NRF_GPIO_PIN_PULLUP);
	nrf_gpio_cfg_output(Pin_SPI_MI);

    NRF_SPI0->PSEL.SCK = Pin_SPI_CK;
    NRF_SPI0->PSEL.MOSI = Pin_SPI_MO;
    NRF_SPI0->PSEL.MISO = Pin_SPI_MI;
  
    MYCONFIG &= ~(0 << 0);
	MYCONFIG |= ~(0 << 0);
	NRF_SPI0->CONFIG = (SPIM_CONFIG_CPOL_ActiveHigh << SPIM_CONFIG_CPOL_Pos) | (SPIM_CONFIG_CPHA_Trailing << SPIM_CONFIG_CPHA_Pos);
    
    NRF_SPI0->FREQUENCY = NRF_DRV_SPI_FREQ_500K;//SPIM_FREQUENCY_FREQUENCY_M8;//NRF_SPIM_FREQ_500K;

	NRF_SPI0->ENABLE = (SPIM_ENABLE_ENABLE_Enabled << SPIM_ENABLE_ENABLE_Pos);
}

void spi_event_handler(nrf_drv_spi_evt_t const *p_event, void *p_context)
{
    spi_xfer_done = true;
}

uint8_t spiReadWrite(uint8_t data)
{
//    // 这种操作方式会引起不断地复位
//    static uint8_t tx_buf[1] = {0};
//    static uint8_t rx_buf[1] = {0};
//    
//    tx_buf[0] = data;
//    
//    spi_xfer_done = false;
//    
//    // spi_tx_rx_PPG(txbuf, length + 1, rxbuf, 0);
//    APP_ERROR_CHECK(nrf_drv_spi_transfer(&SPI_0, tx_buf, 1, rx_buf, 1));
//    
//    while(!spi_xfer_done)
//        __WFE();
//    
//    return rx_buf[0];
    
    // 直接操作寄存器
	NRF_SPI0->TXD = (uint32_t)data;
	NRF_SPI0->ENABLE = 1;
    
	while(NRF_SPI0->EVENTS_READY != 1)
        ; 
	nrf_delay_us(10);    // 2020-03-19 00:11:17 gzj SCLK 8M 延时 1us 即可
                        // 2020-03-19 11:26:14 gzj 对比用户寄存器内容变化时发现 1us 不稳定 改为 2us 已经稳定
    
	return NRF_SPI0->RXD;
}

