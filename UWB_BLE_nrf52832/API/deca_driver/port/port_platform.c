/*! ----------------------------------------------------------------------------
 * @file    port_platform.c
 * @brief   HW specific definitions and functions for portability
 *
 * @attention
 *
 * Copyright 2016 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author DecaWave
 */
#include "dw1001_dev.h"
#include "port_platform.h"
#include "deca_device_api.h"
#include "user_log.h"
/****************************************************************************//**
 *
 *                              APP global variables
 *
 *******************************************************************************/


/****************************************************************************//**
 *
 *                  Port private variables and function prototypes
 *
 *******************************************************************************/
static volatile uint32_t signalResetDone;

/****************************************************************************//**
 *
 *                              Time section
 *
 *******************************************************************************/

/****************************************************************************//**
 *
 *                              END OF Time section
 *
 *******************************************************************************/

/****************************************************************************//**
 *
 *                              Configuration section
 *
 *******************************************************************************/

/****************************************************************************//**
 *
 *                          End of configuration section
 *
 *******************************************************************************/

/****************************************************************************//**
 *
 *                          DW1000 port section
 *
 *******************************************************************************/
 // YB : STM HAL based function have to be updated using NRF drivers


/* @fn      setup_DW1000RSTnIRQ
 * @brief   setup the DW_RESET pin mode
 *          0 - output Open collector mode
 *          !0 - input mode with connected EXTI0 IRQ
 * */
void setup_DW1000RSTnIRQ(int enable)
{
}



/* @fn      port_wakeup_dw1000
 * @brief   "slow" waking up of DW1000 using DW_CS only
 * */
void port_wakeup_dw1000(void)
{
}

/* @fn      port_wakeup_dw1000_fast
 * @brief   waking up of DW1000 using DW_CS and DW_RESET pins.
 *          The DW_RESET signalling that the DW1000 is in the INIT state.
 *          the total fast wakeup takes ~2.2ms and depends on crystal startup time
 * */
void port_wakeup_dw1000_fast(void)
{
}


/**
 * @brief SPI user event handler.
 * @param event
 */
void spi_event_handler(nrf_drv_spi_evt_t const * p_event, void * p_context)
{
    spi_xfer_done = true;
}
#define DATALEN1 200
//================================================================================================
int readfromspi(uint16 headerLength,
                const uint8 *headerBuffer,
                uint32 readlength,
                uint8 *readBuffer)
{
    uint8 idatabuf[DATALEN1]={0};
    uint8 itempbuf[DATALEN1]={0};

    uint8 * p1;
    uint32 idatalength=0;

    memset(idatabuf, 0, DATALEN1);
    memset(itempbuf, 0, DATALEN1);

    p1=idatabuf;
    memcpy(p1,headerBuffer, headerLength);

    p1 += headerLength;
    memset(p1,0x00,readlength);

    idatalength= headerLength + readlength;

    spi_xfer_done = false;
    nrf_drv_spi_transfer(&spi, idatabuf, idatalength, itempbuf, idatalength);
    uint8_t wait_spi_data_back_cnt = 0;
    while(!spi_xfer_done)
    {
        wait_spi_data_back_cnt++;
        nrf_delay_us(1);
        if(wait_spi_data_back_cnt > 20)
        {
            spi_xfer_done = true;
        }
    }
    p1=itempbuf + headerLength;
    memcpy(readBuffer, p1, readlength);

    return 0;
}




int writetospi( uint16 headerLength,
                const uint8 *headerBuffer,
                uint32 bodylength,
                const uint8 *bodyBuffer)
{
    uint8 idatabuf[DATALEN1]={0};
    uint8 itempbuf[DATALEN1]={0};
		
    uint8 * p1;
    uint32 idatalength=0;

    memset(idatabuf, 0, DATALEN1);
    memset(itempbuf, 0, DATALEN1);

    p1=idatabuf;
    memcpy(p1,headerBuffer, headerLength);
    p1 += headerLength;
    memcpy(p1,bodyBuffer,bodylength);

    idatalength= headerLength + bodylength;

    spi_xfer_done = false;
    nrf_drv_spi_transfer(&spi, idatabuf, idatalength, itempbuf, idatalength);
    uint8_t wait_spi_data_back_cnt = 0;
    while(!spi_xfer_done)
    {
        wait_spi_data_back_cnt++;
        nrf_delay_us(1);
        if(wait_spi_data_back_cnt > 20)
        {
            spi_xfer_done = true;
        }
    }

		//log_msg("T[ "); log_queue_hex(idatabuf,idatalength);
		//log_msg("R{ "); log_queue_hex(itempbuf,idatalength);
    return 0;
}
//------------------------------other---------------------------

#define NRF_DRV_SPI_DEFAULT_CONFIG_2M(id)                       \
{                                                            \
    .sck_pin      = CONCAT_3(SPIM, id, _SCK_PIN),      \
    .mosi_pin     = CONCAT_3(SPIM, id, _MOSI_PIN),     \
    .miso_pin     = CONCAT_3(SPIM, id, _MISO_PIN),     \
    .ss_pin       = NRF_DRV_SPI_PIN_NOT_USED,                \
    .irq_priority = CONCAT_3(SPIM, id, _IRQ_PRIORITY), \
    .orc          = 0xFF,                                    \
    .frequency    = NRF_DRV_SPI_FREQ_2M,                     \
    .mode         = NRF_DRV_SPI_MODE_3,                      \
    .bit_order    = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST,         \
}


#define NRF_DRV_SPI_DEFAULT_CONFIG_8M(id)                       \
{                                                            \
    .sck_pin      = CONCAT_3(SPIM, id, _SCK_PIN),      \
    .mosi_pin     = CONCAT_3(SPIM, id, _MOSI_PIN),     \
    .miso_pin     = CONCAT_3(SPIM, id, _MISO_PIN),     \
    .ss_pin       = NRF_DRV_SPI_PIN_NOT_USED,                \
    .irq_priority = CONCAT_3(SPIM, id, _IRQ_PRIORITY), \
    .orc          = 0xFF,                                    \
    .frequency    = NRF_DRV_SPI_FREQ_8M,                     \
    .mode         = NRF_DRV_SPI_MODE_0,                      \
    .bit_order    = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST,         \
}

/* @fn      reset_DW1000
 * @brief   DW_RESET pin on DW1000 has 2 functions
 *          In general it is output, but it also can be used to reset the digital
 *          part of DW1000 by driving this pin low.
 *          Note, the DW_RESET pin should not be driven high externally.
 * */
void reset_DW1000(void)
{
    nrf_gpio_cfg_output(DW1000_RST);   
    nrf_gpio_pin_clear(DW1000_RST); 
    log_dwm1000_init("dwm1000 reset_init!!\r\n");  
    nrf_gpio_cfg_input(DW1000_RST, NRF_GPIO_PIN_NOPULL); 
    nrf_delay_ms(2); 
}

/* @fn      port_set_dw1000_slowrate
 * @brief   set 2MHz
 *          n
 * */
void port_set_dw1000_slowrate(void)
{
    nrf_drv_spi_config_t  spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
    spi_config.sck_pin  = SPI_dw_SCK_PIN;
    spi_config.mosi_pin = SPI_dw_MOSI_PIN ;
    spi_config.miso_pin = SPI_dw_MISO_PIN;
    spi_config.ss_pin   = SPI_dw_CS_PIN;
    spi_config.irq_priority =  SPI_DEFAULT_CONFIG_IRQ_PRIORITY; 
    spi_config.orc          = 0xFF;                                    
    spi_config.frequency    = NRF_DRV_SPI_FREQ_2M;                     
    spi_config.mode         = NRF_DRV_SPI_MODE_0;                      
    spi_config.bit_order    = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST;
		APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler, NULL) );
		nrf_delay_ms(2);
//	nrf_drv_spi_config_t  spi_config = NRF_DRV_SPI_DEFAULT_CONFIG_2M(SPI_INSTANCE);
//	spi_config.ss_pin = SPI_CS_PIN;
//	APP_ERROR_CHECK( nrf_drv_spi_init(&spi, &spi_config, spi_event_handler, NULL) );
//	nrf_delay_ms(2);	
}

/*

	nrf_drv_spi_config_t  spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
	spi_config.sck_pin = SPI_dw_SCK_PIN;
	spi_config.mosi_pin = SPI_dw_MOSI_PIN ;
	spi_config.miso_pin = SPI_dw_MISO_PIN;
	spi_config.ss_pin = SPI_dw_CS_PIN;
	spi_config.irq_priority =  SPI_DEFAULT_CONFIG_IRQ_PRIORITY; 
    spi_config.orc          = 0xFF;                                    
    spi_config.frequency    = NRF_DRV_SPI_FREQ_2M;                     
    spi_config.mode         = NRF_DRV_SPI_MODE_0;                      
    spi_config.bit_order    = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST;
	APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler, NULL) );
	nrf_delay_ms(2);
*/

/* @fn      port_set_dw1000_fastrate
 * @brief   set 8MHz
 *         
 * */
void port_set_dw1000_fastrate(void)
{ 
#ifdef kunkun_use
    nrf_drv_spi_uninit(&spi);
    nrf_drv_spi_config_t  spi_config = NRF_DRV_SPI_DEFAULT_CONFIG_8M(SPI_INSTANCE);
    spi_config.ss_pin = SPI_CS_PIN;
    APP_ERROR_CHECK( nrf_drv_spi_init(&spi, &spi_config, spi_event_handler,NULL) );
    nrf_delay_ms(2);	
#else
    nrf_drv_spi_uninit(&spi);
    nrf_drv_spi_config_t  spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
    spi_config.sck_pin  = SPI_dw_SCK_PIN;
    spi_config.mosi_pin = SPI_dw_MOSI_PIN ;
    spi_config.miso_pin = SPI_dw_MISO_PIN;
    spi_config.ss_pin   = SPI_dw_CS_PIN;
    spi_config.irq_priority =  SPI_DEFAULT_CONFIG_IRQ_PRIORITY; 
    spi_config.orc          = 0xFF;                                    
    spi_config.frequency    = NRF_DRV_SPI_FREQ_8M;                     
    spi_config.mode         = NRF_DRV_SPI_MODE_0;                      
    spi_config.bit_order    = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST;
    APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler, NULL) );
    nrf_delay_ms(2);
#endif

}

void deca_sleep(unsigned int time_ms)
{
    nrf_delay_ms(time_ms);
}


// currently do nothing
decaIrqStatus_t decamutexon(void)           
{
//	u16 j = (u16)(NVIC->ISER[0] & (1 << 5));

	//	if(j) 
//  {
//		NVIC->ISER[0] &= ~(1 << 5); //disable the external interrupt line
//	}
//	return j ; 

		return 0;	
}


// currently do nothing
void decamutexoff(decaIrqStatus_t s)       
{
//	if(j) 

	//	{                 
//		NVIC->ISER[0] |= (1 << 5);;
//	}
		;	
}

/****************************************************************************//**
 *
 *                          End APP port section
 *
 *******************************************************************************/



/****************************************************************************//**
 *
 *                              IRQ section
 *
 *******************************************************************************/

/****************************************************************************//**
 *
 *                              END OF IRQ section
 *
 *******************************************************************************/

/****************************************************************************//**
 *
 *******************************************************************************/

