






#ifndef _RESP_PROCESSING_H__
#define _RESP_PROCESSING_H__




#define RESP_MAX_PEAK_TO_SEARCH 			5
#define RESP_MAXIMA_SEARCH_WINDOW			8
#define RESP_MINIMUM_SKIP_WINDOW			80

#define RESP_SAMPLING_RATE				100
#define RESP_TWO_SEC_SAMPLES  			2 * RESP_SAMPLING_RATE



extern unsigned short Respiration_Rate;
void RESP_Algorithm_Interface(short CurrSample);
void Respiration_Rate_Detection(short Resp_wave);



#endif











