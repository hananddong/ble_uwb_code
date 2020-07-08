











#include "ECG_Processing.h"

int QRS_Second_Prev_Sample = 0 ;
int QRS_Prev_Sample = 0 ;
int QRS_Current_Sample = 0 ;
int QRS_Next_Sample = 0 ;
int QRS_Second_Next_Sample = 0 ;

unsigned char Start_Sample_Count_Flag = 0;
unsigned char first_peak_detect = FALSE ;
unsigned int sample_count = 0 ;
unsigned int sample_index[MAX_PEAK_TO_SEARCH+2] = {0};
static unsigned short QRS_B4_Buffer_ptr = 0 ;

short QRS_Threshold_Old = 0;
short QRS_Threshold_New = 0;

unsigned short QRS_Heart_Rate = 0 ;
unsigned char HR_flag;

void QRS_check_sample_crossing_threshold(short scaled_result )
{
	/* array to hold the sample indexes S1,S2,S3 etc */
	
	static unsigned short s_array_index = 0 ;
	static unsigned short m_array_index = 0 ;
	
	static unsigned char threshold_crossed = FALSE ;
	static unsigned short maxima_search = 0 ;
	static unsigned char peak_detected = FALSE ;
	static unsigned short skip_window = 0 ;
	static long maxima_sum = 0 ;
	static unsigned int peak = 0;
	static unsigned int sample_sum = 0;
	static unsigned int nopeak=0;
	unsigned short max = 0 ;
	unsigned short HRAvg;

	
	if( TRUE == threshold_crossed  )
	{
		/*
		Once the sample value crosses the threshold check for the
		maxima value till MAXIMA_SEARCH_WINDOW samples are received
		*/
		sample_count ++ ;
		maxima_search ++ ;

		if( scaled_result > peak )
		{
			peak = scaled_result ;
		}

		if( maxima_search >= MAXIMA_SEARCH_WINDOW )
		{
			// Store the maxima values for each peak
			maxima_sum += peak;
			maxima_search = 0;

			threshold_crossed = FALSE;
			peak_detected = TRUE;
		}

	}
	else if( TRUE == peak_detected )
	{
		/*
		Once the sample value goes below the threshold
		skip the samples untill the SKIP WINDOW criteria is meet
		*/
		sample_count ++ ;
		skip_window ++ ;

		if( skip_window >= MINIMUM_SKIP_WINDOW )
		{
			skip_window = 0 ;
			peak_detected = FALSE ;
		}

		if( m_array_index == MAX_PEAK_TO_SEARCH )
		{
			sample_sum = sample_sum / (MAX_PEAK_TO_SEARCH - 1);
			HRAvg =  (unsigned short) sample_sum  ;
#if 0
			if((LeadStatus & 0x0005)== 0x0000)
			{
				
			QRS_Heart_Rate = (unsigned short) 60 *  SAMPLING_RATE;
			QRS_Heart_Rate =  QRS_Heart_Rate/ HRAvg ;
				if(QRS_Heart_Rate > 250)
					QRS_Heart_Rate = 250 ;
			}
			else
			{
				QRS_Heart_Rate = 0;
			}
#else
			// Compute HR without checking LeadOffStatus
			QRS_Heart_Rate = (unsigned short) 60 *  SAMPLING_RATE;
			QRS_Heart_Rate =  QRS_Heart_Rate/ HRAvg ; 
			if(QRS_Heart_Rate > 250)
				QRS_Heart_Rate = 250 ;
#endif

			/* Setting the Current HR value in the ECG_Info structure*/

			HR_flag = 1;

			maxima_sum =  maxima_sum / MAX_PEAK_TO_SEARCH;
			max = (short) maxima_sum ;
			/*  calculating the new QRS_Threshold based on the maxima obtained in 4 peaks */
			maxima_sum = max * 7;
			maxima_sum = maxima_sum/10;
			QRS_Threshold_New = (short)maxima_sum;

			/* Limiting the QRS Threshold to be in the permissible range*/
			if(QRS_Threshold_New > (4 * QRS_Threshold_Old))
			{
				QRS_Threshold_New = QRS_Threshold_Old;
	 		}

	 		sample_count = 0;
	 		s_array_index = 0;
	 		m_array_index = 0;
	 		maxima_sum = 0;
			sample_index[0] = 0;
			sample_index[1] = 0;
			sample_index[2] = 0 ;
			sample_index[3] = 0 ;
			Start_Sample_Count_Flag = 0;

			sample_sum = 0;
		}
	}
	else if( scaled_result > QRS_Threshold_New )
	{
		/*
			If the sample value crosses the threshold then store the sample index
		*/
		Start_Sample_Count_Flag = 1;
		sample_count ++ ;
		m_array_index++;
		threshold_crossed = TRUE ;
		peak = scaled_result ;
		nopeak = 0;

		/*	storing sample index*/
	   	sample_index[ s_array_index ] = sample_count;
		if( s_array_index >= 1 )
		{
			sample_sum += sample_index[ s_array_index ] - sample_index[ s_array_index - 1 ] ;
		}
		s_array_index ++ ;
	}

	else if(( scaled_result < QRS_Threshold_New ) && (Start_Sample_Count_Flag == 1))
	{
		sample_count ++ ;
        nopeak++;	
        if (nopeak > (3 * SAMPLING_RATE))
        { 
        	sample_count = 0 ;
	 		s_array_index = 0 ;
	 		m_array_index = 0 ;
	 		maxima_sum = 0 ;
			sample_index[0] = 0 ;
			sample_index[1] = 0 ;
			sample_index[2] = 0 ;
			sample_index[3] = 0 ;
			Start_Sample_Count_Flag = 0;
			peak_detected = FALSE ;
			sample_sum = 0;
        	    	
        	first_peak_detect = FALSE;
	      	nopeak=0;

			QRS_Heart_Rate = 0;
			HR_flag = 1;
        }
	}
   else
   {
    nopeak++;	
   	if (nopeak > (3 * SAMPLING_RATE))
     { 
		/* Reset heart rate computation sate variable in case of no peak found in 3 seconds */
 		sample_count = 0 ;
 		s_array_index = 0 ;
 		m_array_index = 0 ;
 		maxima_sum = 0 ;
		sample_index[0] = 0 ;
		sample_index[1] = 0 ;
		sample_index[2] = 0 ;
		sample_index[3] = 0 ;
		Start_Sample_Count_Flag = 0;
		peak_detected = FALSE ;
		sample_sum = 0;
     	first_peak_detect = FALSE;
	 	nopeak = 0;
		QRS_Heart_Rate = 0;
		HR_flag = 1;

     }
   }

}

void QRS_process_buffer( void )
{

	short first_derivative = 0 ;
	short scaled_result = 0 ;

	static short max = 0 ;

	/* calculating first derivative*/
	first_derivative = QRS_Next_Sample - QRS_Prev_Sample  ;

	/*taking the absolute value*/

	if(first_derivative < 0)
	{
		first_derivative = -(first_derivative);
	}

	scaled_result = first_derivative;

	if( scaled_result > max )
	{
		max = scaled_result ;
	}

	QRS_B4_Buffer_ptr++;
	if (QRS_B4_Buffer_ptr ==  TWO_SEC_SAMPLES)
	{
		QRS_Threshold_Old = ((max *7) /10 ) ;
		QRS_Threshold_New = QRS_Threshold_Old ;
		if(max > 50)
		    first_peak_detect = TRUE ;
		max = 0;
		QRS_B4_Buffer_ptr = 0;
	}


	if( TRUE == first_peak_detect )
	{
		QRS_check_sample_crossing_threshold(scaled_result) ;
	}
}


void QRS_Algorithm_Interface(short CurrSample)
{
//	static FILE *fp = fopen("ecgData.txt", "w");
	static short prev_data[32] ={0};
	short i;
	long Mac=0;
	prev_data[0] = CurrSample;
	for ( i=31; i > 0; i--)
	{
		Mac += prev_data[i];
		prev_data[i] = prev_data[i-1];

	}
	Mac += CurrSample;
	Mac = Mac >> 2;
	CurrSample = (short) Mac;
	QRS_Second_Prev_Sample = QRS_Prev_Sample ;
	QRS_Prev_Sample = QRS_Current_Sample ;
	QRS_Current_Sample = QRS_Next_Sample ;
	QRS_Next_Sample = QRS_Second_Next_Sample ;
	QRS_Second_Next_Sample = CurrSample ;
	QRS_process_buffer();
}




