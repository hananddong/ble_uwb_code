








#include "RESP_Processing.h"
#include "stdio.h"
#include "math.h"
int RESP_Second_Prev_Sample = 0 ;
int RESP_Prev_Sample = 0 ;
int RESP_Current_Sample = 0 ;
int RESP_Next_Sample = 0 ;
int RESP_Second_Next_Sample = 0 ;

unsigned short Respiration_Rate = 0 ;


void Respiration_Rate_Detection(short Resp_wave)
{

	static int abs_value = 0;
	static unsigned short skipCount = 0, SampleCount = 0,TimeCnt=0, SampleCountNtve=0, PtiveCnt =0,NtiveCnt=0 ;
	static short MinThreshold = 0x7FFF, MaxThreshold = 0x8000, PrevSample = 0, PrevPrevSample = 0, PrevPrevPrevSample =0;
	static short MinThresholdNew = 0x7FFF, MaxThresholdNew = 0x8000, AvgThreshold = 0;
	static unsigned char startCalc=0, PtiveEdgeDetected=0, NtiveEdgeDetected=0, peakCount = 0;
	static unsigned short PeakCount[8];
	
	SampleCount++;
	SampleCountNtve++;
	TimeCnt++; 
	if (Resp_wave < MinThresholdNew) MinThresholdNew = Resp_wave;
	if (Resp_wave > MaxThresholdNew) MaxThresholdNew = Resp_wave;
	
	if (SampleCount > 800)
	{
		SampleCount =0;
	}
	if (SampleCountNtve > 800)
	{
		SampleCountNtve =0;
	}
	

	if ( startCalc == 1)
	{
		if (TimeCnt >= 500)
		{
			TimeCnt =0;
			if ( (MaxThresholdNew - MinThresholdNew) > 400)
			{
				MaxThreshold = MaxThresholdNew; 
				MinThreshold =  MinThresholdNew;
				AvgThreshold = MaxThreshold + MinThreshold;
				AvgThreshold = AvgThreshold >> 1;
			}
			else
			{
				startCalc = 0;
				Respiration_Rate = 0;
			}
		}

		PrevPrevPrevSample = PrevPrevSample;
		PrevPrevSample = PrevSample;
		PrevSample = Resp_wave;
		if ( skipCount == 0)
		{
			if (PrevPrevPrevSample < AvgThreshold && Resp_wave > AvgThreshold)
			{
				if ( SampleCount > 40 &&  SampleCount < 700)
				{
//						Respiration_Rate = 6000/SampleCount;	// 60 * 100/SampleCount;
					PtiveEdgeDetected = 1;
					PtiveCnt = SampleCount;
					skipCount = 4;
				}
				SampleCount = 0;
			}
			if (PrevPrevPrevSample < AvgThreshold && Resp_wave > AvgThreshold)
			{
				if ( SampleCountNtve > 40 &&  SampleCountNtve < 700)
				{
					NtiveEdgeDetected = 1;
					NtiveCnt = SampleCountNtve;
					skipCount = 4;
				}
				SampleCountNtve = 0;
			}
			
			if (PtiveEdgeDetected ==1 && NtiveEdgeDetected ==1)
			{
				PtiveEdgeDetected = 0;
				NtiveEdgeDetected =0;
				abs_value = PtiveCnt - NtiveCnt;
				if(abs_value < 0)
				{
					abs_value = -abs_value;
				}	
				
				if ( abs_value < 5)
				{
					PeakCount[peakCount++] = PtiveCnt;
					PeakCount[peakCount++] = NtiveCnt;
					if( peakCount == 8)
					{
						peakCount = 0;
						PtiveCnt = PeakCount[0] + PeakCount[1] + PeakCount[2] + PeakCount[3] + 
								PeakCount[4] + PeakCount[5] + PeakCount[6] + PeakCount[7];
						PtiveCnt = PtiveCnt >> 3;
						Respiration_Rate = (60 * RESP_SAMPLING_RATE)/PtiveCnt;	// 60 * 100/SampleCount;
						if(Respiration_Rate > 60)
						{
							Respiration_Rate = 60;
						}	
					}
				}
			}
		}
		else
		{
			skipCount--;
		}
		
	}
	else
	{
		TimeCnt++;
		if (TimeCnt >= 500)
		{
			TimeCnt = 0;
			if ( (MaxThresholdNew - MinThresholdNew) > 400)
			{
				startCalc = 1;
				MaxThreshold = MaxThresholdNew; 
				MinThreshold =  MinThresholdNew;
				AvgThreshold = MaxThreshold + MinThreshold;
				AvgThreshold = AvgThreshold >> 1;
				PrevPrevPrevSample = Resp_wave;
				PrevPrevSample = Resp_wave;
				PrevSample = Resp_wave;

			}
		}
	}
}

void RESP_Algorithm_Interface(short CurrSample)
{
//	static FILE *fp = fopen("RESPData.txt", "w");
	static short prev_data[64] ={0};
	static unsigned char Decimeter = 0;
	char i;
	long Mac=0;
	prev_data[0] = CurrSample;
	for ( i=63; i > 0; i--)
	{
		Mac += prev_data[i];
		prev_data[i] = prev_data[i-1];

	}
	Mac += CurrSample;
//	Mac = Mac;
	CurrSample = (short) Mac >> 1;
	RESP_Second_Prev_Sample = RESP_Prev_Sample ;
	RESP_Prev_Sample = RESP_Current_Sample ;
	RESP_Current_Sample = RESP_Next_Sample ;
	RESP_Next_Sample = RESP_Second_Next_Sample ;
	RESP_Second_Next_Sample = CurrSample;// << 3 ;
//	fprintf(fp,"%d\n", CurrSample);
	Decimeter++;
	//Resp_Rr_val = RESP_Second_Next_Sample;
	if ( Decimeter == 5)
	{
		Decimeter = 0;
//		RESP_process_buffer();
		Respiration_Rate_Detection(RESP_Second_Next_Sample);
	}
}



















