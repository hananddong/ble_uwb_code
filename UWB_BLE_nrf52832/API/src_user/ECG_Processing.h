







#ifndef _ECG_PROCESSING_H__
#define _ECG_PROCESSING_H__




#define MAX_PEAK_TO_SEARCH 				5
#define MAXIMA_SEARCH_WINDOW			40
#define MINIMUM_SKIP_WINDOW				50

#define SAMPLING_RATE					500
#define TWO_SEC_SAMPLES  				2 * SAMPLING_RATE

#define TRUE	1
#define FALSE	0
extern unsigned short QRS_Heart_Rate;

void QRS_Algorithm_Interface(short CurrSample);

void QRS_process_buffer(void);

void QRS_check_sample_crossing_threshold( short scaled_result ) ;






#endif














