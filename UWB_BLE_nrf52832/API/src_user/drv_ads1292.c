#include "drv_ads1292.h"
#include "drv_spi.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrf_drv_gpiote.h"
#include "user_log.h"
//#include "heart_rate.h"
#include "ECG_Processing.h"
#include "RESP_Processing.h"
#include "dev.h"

#define CSPIN

#define ADS1292_Pin_RDY         25
#define ADS1292_Pin_CS          29
#define ADS1292_Pin_Start       30
#define ADS1292_Pin_Reset       31

#define SPI_MI_Pin26            26
#define SPI_CK_Pin27            27
#define SPI_MO_Pin28            28

#define Pin_H(n)                nrf_gpio_pin_set(n);
#define Pin_L(n)                nrf_gpio_pin_clear(n);

#define ADS1292_Pin_CS_H        Pin_H(ADS1292_Pin_CS)
#define ADS1292_Pin_CS_L        Pin_L(ADS1292_Pin_CS)

#define ADS1292_Pin_Start_H     Pin_H(ADS1292_Pin_Start)
#define ADS1292_Pin_Start_L     Pin_L(ADS1292_Pin_Start)

#define ADS1292_Pin_Reset_H     Pin_H(ADS1292_Pin_Reset)
#define ADS1292_Pin_Reset_L     Pin_L(ADS1292_Pin_Reset)

// System Commands
#define ADS1292R_CMD_WAKEUP     (0x02)  // Wake-up from standby mode
#define ADS1292R_CMD_STANDBY    (0x04)  // Enter standby mode
#define ADS1292R_CMD_RESET      (0x06)  // Reset the device
#define ADS1292R_CMD_START      (0x08)  // Start or restart (synchronize) conversions
#define ADS1292R_CMD_STOP       (0x0A)  // Stop conversion
#define ADS1292R_CMD_OFFSETCAL  (0x1A)  // Channel offset calibration

// Data Read Commands
#define ADS1292R_CMD_RDATAC     (0x10)  // Enable Read Data Continuous mode.
                                        // This mode is the default mode at power-up.
                                        // When in RDATAC mode, the RREG command is ignored.
#define ADS1292R_CMD_SDATAC     (0x11)  // Stop Read Data Continuously mode
#define ADS1292R_CMD_RDATA      (0x12)  // Read data by command; supports multiple read back.

// Register Read Commands
// n nnnn = number of registers to be read or written – 1.
// For example, to read or write three registers,
// set n nnnn = 0 (0010). r rrrr = starting register address
// for read and write opcodes.
//#define ADS1292R_CMD_RREG       (001r rrrr) (000n nnnn)
//#define ADS1292R_CMD_WREG       (010R rrrr) (000n nnnn)

#define CONFIG_SPI_MASTER_DUMMY 0xFF

// Register Read Commands
#define RREG        0x20    // Read n nnnn registers starting at address r rrrr
                            // first byte 001r rrrr (2xh)(2) - second byte 000n nnnn(2)

#define WREG        0x40    // Write n nnnn registers starting at address r rrrr
                            // first byte 010r rrrr (2xh)(2) - second byte 000n nnnn(2)

#define START       0x08        //Start/restart (synchronize) conversions
#define STOP        0x0A        //Stop conversion
#define RDATAC      0x10        //Enable Read Data Continuous mode. 

//This mode is the default mode at power-up.
#define SDATAC      0x11        //Stop Read Data Continuously mode
#define RDATA       0x12        //Read data by command; supports multiple read back.

// register address
#define ADS1292R_Reg_00_ID          (0x00)
#define ADS1292R_Reg_01_CONFIG1     (0x01)
#define ADS1292R_Reg_02_CONFIG2     (0x02)
#define ADS1292R_Reg_03_LOFF        (0x03)
#define ADS1292R_Reg_04_CH1SET      (0x04)
#define ADS1292R_Reg_05_CH2SET      (0x05)
#define ADS1292R_Reg_06_RLD_SENS    (0x06)
#define ADS1292R_Reg_07_LOFF_SENS   (0x07)
#define ADS1292R_Reg_08_LOFF_STAT   (0x08)
#define ADS1292R_Reg_09_RESP1       (0x09)
#define ADS1292R_Reg_0A_RESP2       (0x0A)
#define ADS1292R_Reg_0B_GPIO        (0x0B)


//#define FILTERORDER                 (161)
///* DC Removal Numerator Coeff*/
//#define NRCOEFF                     (0.999)//(0.992)
//#define LENGTH_SEND                 (15)

//#define SAMPLING_RATE               (500)
//#define TWO_SEC_SAMPLES             (2 * SAMPLING_RATE)

//#define RESP_MAX_PEAK_TO_SEARCH     5
//#define RESP_MAXIMA_SEARCH_WINDOW   8
//#define RESP_MINIMUM_SKIP_WINDOW    80

//#define RESP_SAMPLING_RATE          (100)
//#define RESP_TWO_SEC_SAMPLES        (2 * RESP_SAMPLING_RATE)

//#define MAX_PEAK_TO_SEARCH          5
//#define MAXIMA_SEARCH_WINDOW        40
//#define MINIMUM_SKIP_WINDOW         50

ADS1292_Data_Pkg_t ADS1292_Data_Pkg = {0};
bool read_adc_flag = false;
bool ADS1292R_Init_Finished = false;
uint8_t ADS1292_ECG_Update_Index = 0;

volatile char *SPI_RX_Buff_Ptr;
static uint8_t spi_dummy_buff[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
volatile uint8_t SPI_RX_Buff[15];
volatile static int SPI_RX_Buff_Count = 0;
volatile bool ads1292dataReceived = false;
unsigned long uecgtemp = 0;
signed long secgtemp = 0;
volatile signed long s32DaqVals[8];

#define LENGTH_SEND         (15)
uint8_t DataPacketHeader[LENGTH_SEND] = {0};

short qunimade1 = 0;
short qunimade2 = 0;
short qunimade3 = 0;
short qunimade4 = 0;

int16_t ecg_wave_buff[1], ecg_filterout[1];
int16_t resp_wave_buff[1], resp_filterout[1];
long status_byte = 0;
uint8_t LeadStatus = 0;
bool leadoff_deteted = true;

uint8_t RegVal = 0;

void asd1292_pin_init(void);
void ads1292_pin_reset_high_low_high_100ms(void);
void ads1292_pin_start_low_20ms(void);
void ads1292_pin_start_high_20ms(void);
void ads1292_pin_start_low_100ms(void);
void ads1292_send_cmd_start(void);
void ads1292_send_cmd(uint8_t data_in);
void ads1292_send_cmd_stop(void);
void ads1292_send_cmd_stop_read_data_continuous(void);
void ads1292_send_cmd_wakeup(void);
void ads1292_regs_read(uint8_t addr_st, uint8_t* p_data, uint8_t len);
uint8_t ads1292_reg_read(uint8_t addr);
void ads1292_regs_write(uint8_t addr_st, uint8_t* p_data, uint8_t len);
void ads1292_Reg_Write(uint8_t addr, uint8_t data);
void ads1292_send_cmd_read_data_continuous(void);
void ads1292_ready_pin_interrupt_init(void);
void in_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action);
char *ads1292_Read_Data(void);


#define NZEROS 6
#define NPOLES 6
#define GAIN   1.008760041e+02
//static float xv[NZEROS+1], yv[NPOLES+1];
//static float xv[NZEROS+1], yv[NPOLES+1];

//iir filter 
int16_t bandIIRFilter0HZ5_40HZ(int16_t dataIn)
{ 
//	xv[0] = xv[1];
//	xv[1] = xv[2];
//	xv[2] = xv[3];
//	xv[3] = xv[4]; 
//	xv[4] = xv[5];
//	xv[5] = xv[6]; 
//	xv[6] = dataIn / GAIN;
//	yv[0] = yv[1]; 
//	yv[1] = yv[2]; 
//	yv[2] = yv[3]; 
//	yv[3] = yv[4];
//	yv[4] = yv[5]; 
//	yv[5] = yv[6]; 
//	yv[6] = (xv[6] - xv[0]) + 3 * (xv[2] - xv[4])
//		   + ( -0.3665332263 * yv[0]) + (  2.5568853665 * yv[1])
//		   + ( -7.4804429297 * yv[2]) + ( 11.7643366740 * yv[3])
//		   + (-10.4822135550 * yv[4]) + (  5.0079676498 * yv[5]);
//	return (int16_t)yv[6];
    return 0;
}

#define RNZEROS 3
#define RNPOLES 3
#define RGAIN   5.166746126e+05

static double rxv[RNZEROS+1], ryv[RNPOLES+1];


int16_t lowIIRFilter2HZ(int16_t dataIn)
{ 
	rxv[0] = rxv[1];
	rxv[1] = rxv[2];
	rxv[2] = rxv[3]; 
	rxv[3] = dataIn / RGAIN;
	ryv[0] = ryv[1]; 
	ryv[1] = ryv[2];
	ryv[2] = ryv[3]; 
	ryv[3] =   (rxv[0] + rxv[3]) + 3 * (rxv[1] + rxv[2])
			+ (  0.9509756650 * ryv[0]) + ( -2.9007269884 * ryv[1])
			+ (  2.9497358397 * ryv[2]);
	return (int16_t)(ryv[3] * 100 );

}

#define B      -0.999

int16_t HighPass(int16_t input)
{
    static float xv[NZEROS], yv[NPOLES];
    xv[0] = xv[1];
    xv[1] = input;
    yv[0] = yv[1];
    yv[1] = (xv[1] - xv[0]) - (B * yv[0]);
    return (yv[1]);
}

void ads1292_init(void)
{
    #define ADS1292R_UserReg_Num        (12)    // 00 ~ 0B
//    uint8_t mcu_id = 0;
    
//    uint8_t UserRegVal_Default[ADS1292R_UserReg_Num] = {0};
//    uint8_t UserRegVal_UserCfg[ADS1292R_UserReg_Num] = {0};
    
    ADS1292R_Init_Finished = false;
    
    // ads1292_io_init();
    asd1292_pin_init();
    
    drv_spi_init();
    
    //log_dbg_chest_ads1292("ads1292_reg_init\r\n");
    
    // Analog/Digital Power-Up (Follow Power-Up Sequencing)
    
    // Set CLKSEL Pin = 1 and Wait for Oscillator to Wake Up
    
    // ads1292_Reset();
    // Set PWDN/RESET = 1 Wait for 1s for Power-On Reset
    ADS1292_Pin_Reset_H;
    nrf_delay_ms(1);
    ADS1292_Pin_Reset_L;
    nrf_delay_ms(1);
    ADS1292_Pin_Reset_H;
    nrf_delay_ms(1000);
    
//    // nrf_delay_ms(100);
//    nrf_delay_ms(100);
    
//    // ads1292_Disable_Start();
	ADS1292_Pin_Start_L;
	nrf_delay_ms(20);
//    
//    // ads1292_Enable_Start();
	ADS1292_Pin_Start_H;
	nrf_delay_ms(20);
//    
//    // ads1292_Hard_Stop();();
	ADS1292_Pin_Start_L;
	nrf_delay_ms(100);
    
//    // ads1292_Start_Data_Conv_Command();
    ads1292_send_cmd(START);
    
//    // ads1292_Soft_Stop();
    ads1292_send_cmd(STOP);
    
//    // nrf_delay_ms(50);
    nrf_delay_ms(50);
    
    // ads1292_Stop_Read_Data_Continuous();
    ads1292_send_cmd(SDATAC);
    nrf_delay_ms(1);
    
    
//    // Issue Reset Pulse, Wait for 18 tCLKs
//    ADS1292_Pin_Reset_H;
//    nrf_delay_ms(5);
//    ADS1292_Pin_Reset_L;
//    nrf_delay_ms(5);
//    ADS1292_Pin_Reset_H;
//    nrf_delay_ms(5);
//    
//    // Send SDATAC Command (Device Wakes Up in RDATAC Mode, so Send SDATAC Command so Registers can be Written)
//    //ads1292_send_cmd_stop_read_data_continuous();
//    log_dbg_chest_ads1292("SDATAC\r\n");
//    ads1292_send_cmd(SDATAC);
//    nrf_delay_ms(10);

//    ads1292_regs_read(0, UserRegVal_Default, ADS1292R_UserReg_Num);
//    
//    //log_dbg_chest_ads1292("ADS1292R_Reg_00_ID\r\n");
//    mcu_id = ads1292_reg_read(ADS1292R_Reg_00_ID);      // 读取芯片 ID ADS1292R ID 为 0x73
//    log_dbg_chest_ads1292("\r\nADS1292R_ID = 0x%02X (Should be 0x73)\r\n", mcu_id);
    
    ads1292_Reg_Write(ADS1292R_Reg_01_CONFIG1, 0x02);   // Set sampling rate to 500 SPS
    nrf_delay_ms(10);
    
    ads1292_Reg_Write(ADS1292R_Reg_02_CONFIG2, 0xE0);   // Lead-off comp on, test signal disabled,Reference buffer is enabled
    nrf_delay_ms(10);
    
    ads1292_Reg_Write(ADS1292R_Reg_03_LOFF, 0xF0);      // Lead-off defaults
    nrf_delay_ms(10);
    
    ads1292_Reg_Write(ADS1292R_Reg_04_CH1SET, 0x00);    // Ch 1 enabled, gain 6, connected to electrode in
    nrf_delay_ms(10);
    
    ads1292_Reg_Write(ADS1292R_Reg_05_CH2SET, 0x00);    // Ch 2 enabled, gain 6, connected to electrode in
    nrf_delay_ms(10);
    
    ads1292_Reg_Write(ADS1292R_Reg_06_RLD_SENS, 0x2c);  // RLD settings: fmod/16, RLD enabled, RLD inputs from Ch2 only
    nrf_delay_ms(10);
    
    ads1292_Reg_Write(ADS1292R_Reg_07_LOFF_SENS, 0x0F); // LOFF settings: all disabled
    nrf_delay_ms(10);
    
    ads1292_Reg_Write(ADS1292R_Reg_08_LOFF_STAT, 0x00); // Skip register 8, LOFF Settings default
    nrf_delay_ms(10);
    
    ads1292_Reg_Write(ADS1292R_Reg_09_RESP1, 0xf2);     // first two bits inverted Respiration: MOD/DEMOD turned only, phase 0
    nrf_delay_ms(10);
    
    ads1292_Reg_Write(ADS1292R_Reg_0A_RESP2, 0x03);     // Respiration: Calib OFF, respiration freq defaults
    nrf_delay_ms(10);
    
//    ads1292_regs_read(0, UserRegVal_UserCfg, ADS1292R_UserReg_Num);
    
    ads1292_send_cmd_read_data_continuous();
    nrf_delay_ms(10);
    
    ads1292_pin_start_high_20ms();
    
    ads1292_ready_pin_interrupt_init();
    
//    log_dbg_chest_ads1292("\r\nADS1292R Reg(Default -> Congfig  [Changed])\r\n");
//    for(i=0; i<ADS1292R_UserReg_Num; i++)
//    {
//        log_dbg_chest_ads1292("Reg_0x%02X: 0x%02X -> 0x%02X  ",
//                                i,
//                                UserRegVal_Default[i],
//                                UserRegVal_UserCfg[i]);
//        
//        if(UserRegVal_Default[i] == UserRegVal_UserCfg[i])
//            log_dbg_chest_ads1292(".\r\n");
//        else
//            log_dbg_chest_ads1292("Yes\r\n");
//    }
//    log_dbg_chest_ads1292("\r\n");

    ADS1292R_Init_Finished = true;
}

void ads1292_read_adc(void)
{
//    int16_t tmp_int16 = 0;
	uint8_t i = 0,j = 0;
	uint16_t tmp = 0;
    
	if(ADS1292R_Init_Finished && read_adc_flag)
	{
        read_adc_flag = false;
        
        SPI_RX_Buff_Count = 0;
		SPI_RX_Buff_Ptr = ads1292_Read_Data();
		for(i = 0; i < 9; i++)
		{
			SPI_RX_Buff[SPI_RX_Buff_Count++] = *(SPI_RX_Buff_Ptr + i);  // store the result data in array
		}

		for (i = 3; i < 9; i += 3)                          // data outputs is (24 status bits + 24 bits Respiration data +  24 bits ECG data)
		{
			uecgtemp = (unsigned long)(((unsigned long)SPI_RX_Buff[i + 0] << 16) | ( (unsigned long) SPI_RX_Buff[i + 1] << 8) |  (unsigned long) SPI_RX_Buff[i + 2]);
			uecgtemp = (unsigned long)(uecgtemp << 8);
			secgtemp = (signed long)(uecgtemp);
			secgtemp = (signed long)(secgtemp >> 8);

			s32DaqVals[j++] = secgtemp;                     //s32DaqVals[0] is Resp data and s32DaqVals[1] is ECG data
		}
    
		status_byte = (long)((long)SPI_RX_Buff[2] | ((long) SPI_RX_Buff[1]) <<8 | ((long) SPI_RX_Buff[0])<<16); // First 3 bytes represents the status
		status_byte  = (status_byte & 0x0f8000) >> 15;      // bit15 gives the lead status
		LeadStatus = (unsigned char ) status_byte ;  
		
		if(!((LeadStatus & 0x1f) == 0 ))
		{
			leadoff_deteted  = true; 
		}
		else
		{
			leadoff_deteted  = false;
		}
		
		ecg_wave_buff[0] = (int16_t)(s32DaqVals[1]);   // ignore the lower 8 bits out of 24bits
		//QRS_Algorithm_Interface(ecg_wave_buff[0]);
		resp_wave_buff[0] = (int16_t)(s32DaqVals[0]);
        
        //log_dbg_chest_ads1292("ecg=%+08d  resp=%+08d\r\n", ecg_wave_buff[0]+100000, resp_wave_buff[0]+100000);

		//if(leadoff_deteted == false) 

		// printf("aa %d\r\n",s32DaqVals[1]);
		//filter out the line noise @40Hz cutoff 161 order
		//ECG_ProcessCurrSample(&ecg_wave_buff[0], &ecg_filterout[0]); 
//		ecg_filterout[0] = bandIIRFilter0HZ5_40HZ(ecg_wave_buff[0]);
        //log_dbg_chest_ads1292("ecg_filterout[0]=%+08d\r\n", ecg_filterout[0]);
        
		//ecg_filterout[0] = test;
		//test++; 
		QRS_Algorithm_Interface(ecg_filterout[0] * 50);
		resp_filterout[0] = lowIIRFilter2HZ(HighPass(resp_wave_buff[0]));
		RESP_Algorithm_Interface(resp_filterout[0]);
		//Resp_ProcessCurrSample(&resp_wave_buff[0], &resp_filterout[0]);
        
        ADS1292_Data_Pkg.ADS1292_ECG[ADS1292_ECG_Update_Index] = ecg_filterout[0] * ((32768.0 / 6000.0) * 5);
        
        log_dbg_chest_ads1292("ecg_%d\r\n", ADS1292_ECG_Update_Index);
        
        if(ADS1292_ECG_Update_Index == 0)
		{	
			DataPacketHeader[6] = ecg_filterout[0] >> 8;
			DataPacketHeader[7] = ecg_filterout[0];
			qunimade1 = (((DataPacketHeader[6] << 8) & 0xff00) | (DataPacketHeader[7] & 0x00ff) & 0xffff);
//            ADS1292_Data_Pkg.ADS1292_ECG[ADS1292_ECG_Update_Index] = qunimade1 * ((32768.0 / 6000.0) * 5);
//			update_data();
//      update_data_flag = true;
		}
        else if(ADS1292_ECG_Update_Index == 1)
		{	
			DataPacketHeader[0] = ecg_filterout[0] >> 8; 
			DataPacketHeader[1] = ecg_filterout[0];   
			qunimade2 = (((DataPacketHeader[0] << 8) & 0xff00) | (DataPacketHeader[1] & 0x00ff) & 0xffff);
//            ADS1292_Data_Pkg.ADS1292_ECG[ADS1292_ECG_Update_Index] = qunimade2 * ((32768.0 / 6000.0) * 5);
			//update_data();
		}

		//ecg_filterout[0]*=49.9;   //mv    电极片
		//ecg_filterout[0]*=2.132;   //mv    胸带   ori
		//ecg_filterout[0] = Respiration_Rate * 5.4613;

		else if(ADS1292_ECG_Update_Index == 2)
		{	
			DataPacketHeader[2] = ecg_filterout[0] >> 8;
			DataPacketHeader[3] = ecg_filterout[0];
			qunimade3 = (((DataPacketHeader[2] << 8) & 0xff00) | (DataPacketHeader[3] & 0x00ff) & 0xffff); 
//            ADS1292_Data_Pkg.ADS1292_ECG[ADS1292_ECG_Update_Index] = qunimade3 * ((32768.0 / 6000.0) * 5);
			//update_data();
		}
		// resp_filterout[0]*=49.9;  //NODE    
		// resp_filterout[0]*=9.9;       // 电极片
		//resp_filterout[0]*=100;   //或者在源头放大    //mine2
		//resp_filterout[0] = 100;

		else if(ADS1292_ECG_Update_Index == 3)
		{	
			DataPacketHeader[4] = ecg_filterout[0] >> 8;
			DataPacketHeader[5] = ecg_filterout[0];
			qunimade4 = (((DataPacketHeader[4] << 8) & 0xff00) | (DataPacketHeader[5] & 0x00ff) & 0xffff);
//            ADS1292_Data_Pkg.ADS1292_ECG[ADS1292_ECG_Update_Index] = qunimade4 * ((32768.0 / 6000.0) * 5);
			//update_data();
            
            //ads1292_load_ecg();
            
            //log_dbg_chest_ads1292("ECG_val_100\r\n");
            //log_dbg_chest_ads1292("MsgToMaster[9:10] = %02X %02X\r\n", MsgToMaster[9], MsgToMaster[10]);
		}
        
        ADS1292_ECG_Update_Index++;
        if(ADS1292_ECG_Update_Index >= 4)
            ADS1292_ECG_Update_Index = 0;

//		else if(ecg_flag == 10)
//		{
//			
//			update_data();
//			update_data_flag = true;
//			//send();
//		}		
//		else
//		{}


		DataPacketHeader[8] = resp_filterout[0] >> 8;
		DataPacketHeader[9] = resp_filterout[0];
		tmp = QRS_Heart_Rate * 100;
		DataPacketHeader[10] = tmp >> 8;
		DataPacketHeader[11] = tmp;

		tmp = Respiration_Rate;
		DataPacketHeader[12] = tmp >> 8;
		DataPacketHeader[13] = tmp;	

//		mpu9250_read_accel(&AccValue);

//		accx = AccValue.x;
//		accy = AccValue.y;
//		accz = AccValue.z;
		
//		if(leadoff_deteted == true)                         // lead in not connected
//		{

//		}
//		else
//		{

//		}
//		checkSum = 0;
//		for(i=0;i<LENGTH_SEND-1;i++)
//		{
//			checkSum += DataPacketHeader[i];
//		}
//		DataPacketHeader[LENGTH_SEND-1] = checkSum;
//		for (i = 0; i < LENGTH_SEND;i++)
//		{
//			//usartSendChar(DataPacketHeader[i]);             // transmit the data over USB
//			//DataPacketHeader[0] = 5;
//			SendToErg(DataPacketHeader[i]);
//		}
	}
}

void ads1292_ready_pin_interrupt_init(void)
{
    nrf_drv_gpiote_init();
    
    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
    
    //in_config.pull = NRF_GPIO_PIN_PULLUP;
    nrf_drv_gpiote_in_init(ADS1292_Pin_RDY, &in_config, in_pin_handler);
    
    nrf_drv_gpiote_in_event_enable(ADS1292_Pin_RDY, true);
}

void asd1292_pin_init(void)
{
    nrf_gpio_cfg_output(ADS1292_Pin_CS);
    nrf_gpio_cfg_output(ADS1292_Pin_Start);
    nrf_gpio_cfg_output(ADS1292_Pin_Reset);
    
    nrf_gpio_cfg_input(ADS1292_Pin_RDY, NRF_GPIO_PIN_PULLUP);
}

void ads1292_pin_reset_high_low_high_100ms(void)
{
    ADS1292_Pin_Reset_H;
    nrf_delay_ms(100);
    
    ADS1292_Pin_Reset_L;
    nrf_delay_ms(100);
    
    ADS1292_Pin_Reset_H;
    nrf_delay_ms(100);
}

void ads1292_pin_start_low_20ms(void)
{
    ADS1292_Pin_Start_L;
    nrf_delay_ms(20);
}

void ads1292_pin_start_high_20ms(void)
{
    ADS1292_Pin_Start_H;
    nrf_delay_ms(20);
}

void ads1292_pin_start_low_100ms(void)
{
    ADS1292_Pin_Start_L;
    nrf_delay_ms(100);
}

void ads1292_send_cmd_start(void)
{
    ads1292_send_cmd(START);                     // Send 0x08 to the ADS1x9x
}

void ads1292_send_cmd_stop(void)
{
    ads1292_send_cmd(STOP);                      // Send 0x0A to the ADS1x9x
}

void ads1292_send_cmd(uint8_t data_in)
{
    #ifdef CSPIN
    ADS1292_Pin_CS_L ;
    nrf_delay_ms(2);
    
    ADS1292_Pin_CS_H;
    nrf_delay_ms(2);
    
    ADS1292_Pin_CS_L ;
    nrf_delay_ms(2);
    #endif
    
    spiReadWrite(data_in);
    
    #ifdef CSPIN
    nrf_delay_ms(2);
    ADS1292_Pin_CS_H;
    #endif
}

void ads1292_send_cmd_stop_read_data_continuous(void)
{
    ads1292_send_cmd(SDATAC);
}

void ads1292_send_cmd_wakeup(void)
{
    ads1292_send_cmd(ADS1292R_CMD_WAKEUP);
}

void ads1292_regs_read(uint8_t addr_st, uint8_t* p_data, uint8_t len)
{
    /*
        First opcode byte: 001r rrrr, where r rrrr is the starting register address.
        Second opcode byte: 000n nnnn, where n nnnn is the number of registers to read – 1.
    */
    
    uint8_t opcode_1;
    uint8_t opcode_2;
    uint8_t i;
	
    opcode_1 = addr_st;
    opcode_1 &= 0x1F;		// 0001 1111
    opcode_1 |= 0x20;		// 0010 0000
    
    opcode_2 = len -1;
    opcode_2 &= 0x1F;		// 0001 1111
    
    #ifdef CSPIN
        ADS1292_Pin_CS_L ;
        nrf_delay_ms(2);
        ADS1292_Pin_CS_H;
        nrf_delay_ms(2);
        ADS1292_Pin_CS_L ;
        nrf_delay_ms(2);
    #endif
    
    spiReadWrite(opcode_1);
    nrf_delay_us(50);
    
    spiReadWrite(opcode_2);
    nrf_delay_us(50);
    
    for(i=0; i<len; i++)
    {
        p_data[i] = spiReadWrite(0xFF);
        //log_dbg_chest_ads1292("p_data[%d] = 0x%02X\r\n", i, p_data[i]);
        nrf_delay_us(50);
    }
    
    #ifdef CSPIN
        nrf_delay_ms(2);
        ADS1292_Pin_CS_H;
    #endif
}

uint8_t ads1292_reg_read(uint8_t addr)
{
    //log_dbg_chest_ads1292("\r\nval=0x%02X\r\n", RegVal);
    ads1292_regs_read(addr, &RegVal, 1);
    //log_dbg_chest_ads1292("  ->  val=0x%02X\r\n", RegVal);
    
    return RegVal;
}

void ads1292_regs_write(uint8_t addr_st, uint8_t* p_data, uint8_t len)
{
    /*
        First opcode byte: 010r rrrr, where r rrrr is the starting register address.
        Second opcode byte: 000n nnnn, where n nnnn is the number of registers to write – 1.
    */
    
    uint8_t opcode_1;
    uint8_t opcode_2;
    uint8_t i;
	
    opcode_1 = addr_st;
    opcode_1 &= 0x1F;		// 0001 1111
    opcode_1 |= 0x40;		// 0100 0000
    
    opcode_2 = len -1;
    opcode_2 &= 0x1F;		// 0001 1111
    
    #ifdef CSPIN
        ADS1292_Pin_CS_L ;
        nrf_delay_ms(2);
        ADS1292_Pin_CS_H;
        nrf_delay_ms(2);
        ADS1292_Pin_CS_L ;
        nrf_delay_ms(2);
    #endif
    
    spiReadWrite(opcode_1);
    nrf_delay_us(50);
    
    spiReadWrite(opcode_2);
    nrf_delay_us(50);
    
    for(i=0; i<len; i++)
    {
        spiReadWrite(p_data[i]);
        nrf_delay_us(50);
    }
    
    #ifdef CSPIN
        nrf_delay_ms(2);
        ADS1292_Pin_CS_H;
    #endif
}

void ads1292_Reg_Write(uint8_t addr, uint8_t data)
{
    // 某些寄存器的特定位必须为 0 或 1
    switch(addr)
    {
        case 0x01:  data &= 0x87;                   break;
        case 0x02:  data &= 0xFB;   data |= 0x80;   break;
        case 0x03:  data &= 0xFD;   data |= 0x10;   break;
        case 0x07:  data &= 0x3F;                   break;
        case 0x08:  data &= 0x5F;                   break;
        case 0x09:  data |= 0x02;                   break;
        case 0x0A:  data &= 0x87;   data |= 0x01;   break;
        case 0x0B:  data &= 0x0F;                   break;
        default:                                    break;
    }
    
    #ifdef CSPIN
        ADS1292_Pin_CS_L ;
        nrf_delay_ms(2);
        ADS1292_Pin_CS_H;
        nrf_delay_ms(2);
        ADS1292_Pin_CS_L ;
        nrf_delay_ms(2);
    #endif
    //  uint8_t spiReadWrite(uint8_t data)
    
    uint8_t dataToSend = addr | WREG;
    
    spiReadWrite(dataToSend);   // Send register location
    nrf_delay_us(50);
    
    spiReadWrite(0x00);         // number of register to wr
    nrf_delay_us(50);
    
    spiReadWrite(data);         // Send value to record into register
    nrf_delay_us(50);
    
    #ifdef CSPIN
        nrf_delay_ms(2);
        ADS1292_Pin_CS_H;
    #endif
}

void ads1292_send_cmd_read_data_continuous(void)
{
    ads1292_send_cmd(RDATAC);                            // Send 0x10 to the ADS1x9x
}

void in_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    read_adc_flag = true;
    
//    log_dbg_chest_ads1292("DRDY\r\n");
}

char *ads1292_Read_Data(void)
{
    #ifdef CSPIN
    ADS1292_Pin_CS_L ;
    
    #endif
    
    for(int i = 0; i < 9; i++)
    {
        spi_dummy_buff[i] = spiReadWrite(CONFIG_SPI_MASTER_DUMMY);
        nrf_delay_us(50);
    }
    
    #ifdef CSPIN
    ADS1292_Pin_CS_H;
    #endif
    
    return (char *)spi_dummy_buff;
}

void ads1292_load_ecg(void)
{
    #define CNT_DBG_UPDATE      do{ cnt_dbg = (cnt_dbg + 100) % 10000;  }while(0)
    
    static uint32_t cnt_dbg = 0;
    int16_t tmp_int16;
    
    ADS1292_ECG_Update_Index = 0;
    
    log_dbg_chest_ads1292("ecg_update\r\n");
    
    tmp_int16 = ADS1292_Data_Pkg.ADS1292_ECG[0];
    MsgToMaster[9] = tmp_int16 >> 8;
    MsgToMaster[10] = tmp_int16;
    
    tmp_int16 = ADS1292_Data_Pkg.ADS1292_ECG[1];
    MsgToMaster[11] = tmp_int16 >> 8;
    MsgToMaster[12] = tmp_int16;
    
    tmp_int16 = ADS1292_Data_Pkg.ADS1292_ECG[2];
    MsgToMaster[13] = tmp_int16 >> 8;
    MsgToMaster[14] = tmp_int16;
    
    tmp_int16 = ADS1292_Data_Pkg.ADS1292_ECG[3];
    MsgToMaster[15] = tmp_int16 >> 8;
    MsgToMaster[16] = tmp_int16;
    
    CNT_DBG_UPDATE;
    MsgToMaster[9] = cnt_dbg >> 8;
    MsgToMaster[10] = cnt_dbg;
//    
//    CNT_DBG_UPDATE;
//    MsgToMaster[11] = cnt_dbg >> 8;
//    MsgToMaster[12] = cnt_dbg;
//    
//    CNT_DBG_UPDATE;
//    MsgToMaster[13] = cnt_dbg >> 8;
//    MsgToMaster[14] = cnt_dbg;
//    
//    CNT_DBG_UPDATE;
//    MsgToMaster[15] = cnt_dbg >> 8;
//    MsgToMaster[16] = cnt_dbg;
}    

