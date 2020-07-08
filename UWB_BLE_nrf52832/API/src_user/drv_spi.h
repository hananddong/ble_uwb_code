#ifndef	__DRV_SPI_H__
#define __DRV_SPI_H__

#include "stdint.h"

extern uint8_t m_tx_buf[9];
extern uint8_t m_rx_buf[9];

extern void drv_spi_init(void);
extern uint8_t spiReadWrite(uint8_t data);

#endif	// __DRV_SPI_H__

