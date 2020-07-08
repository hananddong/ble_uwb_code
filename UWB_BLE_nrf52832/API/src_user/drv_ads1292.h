#ifndef	__DRV_ADS1292_H__
#define __DRV_ADS1292_H__

#include "stdbool.h"
#include "stdint.h"

typedef struct
{
    int16_t ADS1292_ECG[4];
    
    int16_t ADS1292_RESP;
    int16_t ADS1292_BPM;
    int16_t ADS1292_RPM;
}ADS1292_Data_Pkg_t;

extern ADS1292_Data_Pkg_t ADS1292_Data_Pkg;
extern bool read_adc_flag;
extern uint8_t ADS1292_ECG_Update_Index;

extern void ads1292_init(void);
extern void ads1292_read_adc(void);
uint16_t ads1292_read_adc_test(void);
void ads1292_read_addr_len(uint8_t addr, uint8_t* p_data);
void ads1292_load_ecg(void);

#endif	// __DRV_ADS1292_H__

