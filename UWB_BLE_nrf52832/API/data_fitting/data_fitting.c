#include "data_fitting.h"

#include <string.h>
#include "math.h"
#include <time.h>
#include "user_log.h"
#include "deca_device_api.h"
#include "deca_regs.h"
#include "trilateration.h"
#include <math.h>



#define ParaBuffer(Buffer,Row,Col) (*(Buffer + (Row) * (SizeSrc + 1) + (Col)))
/***********************************************************************************
***********************************************************************************/
static int GetXY(const char* FileName, double* X, double* Y, int* Amount)
{
    FILE* File = fopen(FileName, "r");
    if (!File)
            return -1;
    for (*Amount = 0; !feof(File); X++, Y++, (*Amount)++)
            if (2 != fscanf(File, (const char*)"%lf %lf", X, Y))
        break;
    fclose(File);
    return 0;
}

/***********************************************************************************
***********************************************************************************/
static int PrintPara(double* Para, int SizeSrc)
{
    int i, j;
    for (i = 0; i < SizeSrc; i++)
    {
        for (j = 0; j <= SizeSrc; j++)
        //printf("%10.6lf ", ParaBuffer(Para, i, j));
        //printf("\r\n");
        ;
    }
    //printf("\r\n");
    return 0;
}
/***********************************************************************************
***********************************************************************************/
static int ParalimitRow(double* Para, int SizeSrc, int Row)
{
    int i;
    double Max, Min, Temp;
    for (Max = abs(ParaBuffer(Para, Row, 0)), Min = Max, i = SizeSrc; i; i--)
    {
        Temp = abs(ParaBuffer(Para, Row, i));
        if (Max < Temp)
            Max = Temp;
        if (Min > Temp)
            Min = Temp;
    }
    Max = (Max + Min) * 0.000005;
    for (i = SizeSrc; i >= 0; i--)
        ParaBuffer(Para, Row, i) /= Max;
    return 0;
}
/***********************************************************************************
***********************************************************************************/
static int Paralimit(double* Para, int SizeSrc)
{
    int i;
    for (i = 0; i < SizeSrc; i++)
        if (ParalimitRow(Para, SizeSrc, i))
            return -1;
    return 0;
}
/***********************************************************************************
***********************************************************************************/
static int ParaPreDealA(double* Para, int SizeSrc, int Size)
{
     int i, j;
     for (Size -= 1, i = 0; i < Size; i++)
     {
         for (j = 0; j < Size; j++)
         ParaBuffer(Para, i, j) = ParaBuffer(Para, i, j) * ParaBuffer(Para, Size, Size) - ParaBuffer(Para, Size, j) * ParaBuffer(Para, i, Size);
         ParaBuffer(Para, i, SizeSrc) = ParaBuffer(Para, i, SizeSrc) * ParaBuffer(Para, Size, Size) - ParaBuffer(Para, Size, SizeSrc) * ParaBuffer(Para, i, Size);
         ParaBuffer(Para, i, Size) = 0;
         ParalimitRow(Para, SizeSrc, i);
     }
     return 0;
}
/***********************************************************************************
***********************************************************************************/
static int ParaDealA(double* Para, int SizeSrc)
{
     int i;
     for (i = SizeSrc; i; i--)
         if (ParaPreDealA(Para, SizeSrc, i))
             return -1;
     return 0;
}
/***********************************************************************************
***********************************************************************************/
static int ParaPreDealB(double* Para, int SizeSrc, int OffSet)
{
     int i, j;
     for (i = OffSet + 1; i < SizeSrc; i++)
     {
         for (j = OffSet + 1; j <= i; j++)
         ParaBuffer(Para, i, j) *= ParaBuffer(Para, OffSet, OffSet);
         ParaBuffer(Para, i, SizeSrc) = ParaBuffer(Para, i, SizeSrc) * ParaBuffer(Para, OffSet, OffSet) - ParaBuffer(Para, i, OffSet) * ParaBuffer(Para, OffSet, SizeSrc);
         ParaBuffer(Para, i, OffSet) = 0;
         ParalimitRow(Para, SizeSrc, i);
     }
     return 0;
}
/***********************************************************************************
***********************************************************************************/
static int ParaDealB(double* Para, int SizeSrc)
{
     int i;
     for (i = 0; i < SizeSrc; i++)
             if (ParaPreDealB(Para, SizeSrc, i))
                     return -1;
     for (i = 0; i < SizeSrc; i++)
     {
         if (ParaBuffer(Para, i, i))
         {
             ParaBuffer(Para, i, SizeSrc) /= ParaBuffer(Para, i, i);
             ParaBuffer(Para, i, i) = 1.0;
         }
     }
     return 0;
}
/***********************************************************************************
***********************************************************************************/
static int ParaDeal(double* Para, int SizeSrc)
{
     PrintPara(Para, SizeSrc);
     Paralimit(Para, SizeSrc);
     PrintPara(Para, SizeSrc);
     if (ParaDealA(Para, SizeSrc))
             return -1;
     PrintPara(Para, SizeSrc);
     if (ParaDealB(Para, SizeSrc))
             return -1;
     return 0;
}
/***********************************************************************************
    函数名：GetParaBuffer
    参数：	double* Para          
          const double* X
          const double* Y
          int Amount
          int SizeSrc
***********************************************************************************/
static int GetParaBuffer(double* Para, const double* X, const double* Y, int Amount, int SizeSrc)
{
	 int i, j;
	 for (i = 0; i < SizeSrc; i++)
					 for (ParaBuffer(Para, 0, i) = 0, j = 0; j < Amount; j++)
									 ParaBuffer(Para, 0, i) += pow(*(X + j), 2 * (SizeSrc - 1) - i);
	 for (i = 1; i < SizeSrc; i++)
					 for (ParaBuffer(Para, i, SizeSrc - 1) = 0, j = 0; j < Amount; j++)
									 ParaBuffer(Para, i, SizeSrc - 1) += pow(*(X + j), SizeSrc - 1 - i);
	 for (i = 0; i < SizeSrc; i++)
					 for (ParaBuffer(Para, i, SizeSrc) = 0, j = 0; j < Amount; j++)
									 ParaBuffer(Para, i, SizeSrc) += (*(Y + j)) * pow(*(X + j), SizeSrc - 1 - i);
	 for (i = 1; i < SizeSrc; i++)
					 for (j = 0; j < SizeSrc - 1; j++)
									 ParaBuffer(Para, i, j) = ParaBuffer(Para, i - 1, j + 1);
	 return 0;
}
/***********************************************************************************
    函数名：Cal
    参数：  const double* BufferX		x轴参数
          const double* BufferY		y轴参数
          int Amount    					参数对数
          int SizeSrc   					拟合次数
          double* ParaResK				拟合多项式系数数组指针
***********************************************************************************/ 
int Cal(const double* BufferX, const double* BufferY, int Amount, int SizeSrc, double* ParaResK)
{
	 double* ParaK = (double*)malloc(SizeSrc * (SizeSrc + 1) * sizeof(double)); // 动态分配内存  
	 GetParaBuffer(ParaK, BufferX, BufferY, Amount, SizeSrc);
	 ParaDeal(ParaK, SizeSrc);
	 for (Amount = 0; Amount < SizeSrc; Amount++, ParaResK++)
					 *ParaResK = ParaBuffer(ParaK, Amount, SizeSrc);
	 free(ParaK);
	 return 0;
}
 /***********************************************************************************
 ***********************************************************************************/


double buff_x_f[30];   // X轴序列  
double buff_y_f[30];   // Y轴序列  
char storing_to_eeprom_anthor_num = 0;  
#define  A0_id                '0'
#define  A1_id                '1'
#define  A2_id                '2'
#define  A0_eeprom_parak_adr  (0)
#define  A1_eeprom_parak_adr  (50)
#define  A2_eeprom_parak_adr  (100)
//double   A0_g_parak[6],A1_g_parak[6],A2_g_parak[6];

int data_fit_apply(void)
{
    // SEGGER_RTT_printf  
    int Amount = 20;
    clock_t StartTime, FinishTime;	//	声明两个时间变量
    double DiffTime = -13.24;
    int16_t int_timedata;
    int_timedata  = DiffTime;
    StartTime = clock();		//	开始计时  
    /*
    BufferX[50] = {0.995119 ,2.001185 ,2.999068 ,4.001035 ,4.999859 ,6.004461 ,6.999335 ,7.999433 ,9.002257 ,10.003888,
                     11.004076,12.001602,13.003390,14.001623,15.003034,16.002561,17.003010,18.003897,19.002563,20.003530	 }, 
          BufferY[50] = {-7.620000,-2.460000,108.760000,625.020000,2170.500000,5814.580000,13191.840000,26622.060000,49230.220000,85066.500000 ,  
                      139226.280000,217970.140000,328843.860000,480798.420000,684310.000000,951499.980000,1296254.940000,1734346.660000,2283552.120000,2963773.500000}, 
    */
    double ParaK[6];	//	5次拟合, 一共6个系数(包含常数项)											
    //	读入要拟合的数据   

    log_msg("hello world!!");
    //if (GetXY((const char*)"test.txt", (double*)BufferX, (double*)BufferY, &Amount))
      //return 0;
	 
    printf("Amount: %d\n", Amount);
    Cal((const double*)buff_x_f, (const double*)buff_y_f, Amount, sizeof(ParaK) / sizeof(double), (double*)ParaK);
    printf("拟合系数为:\n");
    printf("按升序排列\n");
    for (Amount = 0; Amount < sizeof(ParaK) / sizeof(double); Amount++)
    printf("ParaK[%d] = %lf\r\n", Amount, ParaK[Amount]);

    if(storing_to_eeprom_anthor_num == A0_id)
    {
        //eeprom_data_storing_write((void*)ParaK,A0_eeprom_parak_adr,);
        memcpy((void *)UserFlashCfg.Pkg.polynomial_coeff_ant_0.g_parak,ParaK,sizeof(ParaK));
        user_cfg_write();
    }
    else if(storing_to_eeprom_anthor_num == A1_id)
    {
        //eeprom_data_storing_write((void*)ParaK,A1_eeprom_parak_adr,sizeof(ParaK));
        memcpy((void *)UserFlashCfg.Pkg.polynomial_coeff_ant_1.g_parak,ParaK,sizeof(ParaK));
        user_cfg_write();
    }
    else if(storing_to_eeprom_anthor_num == A2_id)
    {
        //eeprom_data_storing_write((void*)ParaK,A2_eeprom_parak_adr,sizeof(ParaK));
        memcpy((void *)UserFlashCfg.Pkg.polynomial_coeff_ant_2.g_parak,ParaK,sizeof(ParaK));
        user_cfg_write();
    }

    FinishTime = clock();	//	结束计时
    DiffTime = FinishTime - StartTime;	//拟合时间
    printf("拟合时间为: %lf\n  double lenght is %d \n\r ", DiffTime, sizeof(ParaK[0]));
    return 0;
	 
}



/**
    历史数据 	： 	,0.995155,2.001185,2.999068,4.001035,4.999859,6.004461,6.999335,7.999433,9.002257,10.003888,X 
    函数名称	： 	store_floating_point_data
    参数     	： 	buff       输入参数，要处理的字符串数组。
                        ：	buff_x_ff  输出参数，处理后形成的浮点型数据数组。
    功能    	：  提取包含浮点数据的字符串中的浮点数据，并存储到相应的输出数组中。格式如上所示
*/
static inline uint16_t store_floating_point_data(char* buff , double * buff_x_ff)
{
    uint16_t num = 0;
    uint8_t f_str_buf[30];
    char *p  = buff ,
         *pt = p+1;  
    uint8_t buf_counter = 0;
    do
    {
        p++;
        log_msg("num  %d  lengght %d  *p %c   \n\r",num,(p-pt),*p);
        if( (p-pt) > 0 )
        {
            memcpy(f_str_buf,(pt),(p-pt-1));
            char *ptr = NULL ;
            buff_x_ff[buf_counter++] = strtod((void*)f_str_buf,&ptr); //  atof(f_str_buf);  //
            log_msg("lf is %lf *ptr is %s  \n\r",buff_x_ff[buf_counter-1],ptr);
            log_msg(" %s \n\r",f_str_buf);
            memset(f_str_buf,0,sizeof(f_str_buf));
            
        }
        num ++;
        pt = p ;
    }
    while((p = strstr(p,",")) != NULL);
    printf("%s \n\r",buff);   
    return (num-1);
}
/**
    函数名称：  cb_data_fitting_uart_handler
    参数    ：  res  返回的字节   
    功能    ：  串口1rx中断的回调函数 

    说明    ：
    x轴数据  ,0.995119,2.001185,2.999068,4.001035,4.999859,6.004461,6.999335,7.999433,9.002257,10.003888,11.004076,12.001602,13.003390,14.001623,15.003034,16.002561,17.003010,18.003897,19.002563,20.003530,X
    y轴数据  ,-7.620000,-2.460000,108.760000,625.020000,2170.500000,5814.580000,13191.840000,26622.060000,49230.220000,85066.500000,139226.280000,217970.140000,328843.860000,480798.420000,684310.000000,951499.980000,1296254.940000,1734346.660000,2283552.120000,2963773.500000,Y
    计算多项式系数并存储命令
                    ,RS0
                    ,RS1
                    ,RS2

    读取eeprom数据命令

                    ,RT0
                    ,RT1
                    ,RT2

    清除eeprom命令

                    ,RTC
                    
    历史数据: 
                ,0.30,0.50,1.00,2.00,3.00,4.00,5.00,6.00,7.00,8.00,9.00,10.00,Y
        ant0    ,0.37,0.59,1.11,2.14,3.21,4.23,5.24,6.21,7.29,8.33,9.39,10.37,X
        ant1    ,0.36,0.58,1.09,2.11,3.09,4.14,5.17,6.12,7.12,8.24,9.22,10.20,X
        ant2    ,0.37,0.59,1.11,2.17,3.14,4.15,5.18,6.16,7.23,8.28,9.27,10.21,X
*/
void cb_data_fitting_uart_handler(uint8_t res)
{
    #define cache_bytes_lenght    400         // 串口中断接收数据buf的长度
    #define uart_counter_max      300
    #define start_cal_cmd_lenght  4           
    #define cam_max_lenght        4
    #define clear_eeprom_cmd      'C'
    static char buf[cache_bytes_lenght];      // 串口中断接收数据buf
    static uint16_t uart_counter = 0, x_ele_num = 0 , y_ele_num = 0;  // 接收数据的字节个数
    
    buf[uart_counter++] = res;         // 接收数据
    //log_msg("%c",res);
    //log_msg("%02x %02x  ",buf[uart_counter - 2],buf[uart_counter - 1]);
    if( ',' == buf[0])                     // 判断包头是否等于 ','
    {
          
        if('X' == buf[uart_counter - 1])  // 判断包尾，作为接收一组数据的结束标志结束标志
        {
            x_ele_num = store_floating_point_data(buf,buff_x_f);  // 数据处理部分，将字符串中的浮点数提取出来 
            memset(buf,0,uart_counter);
            uart_counter = 0;
        }
        if('Y' == buf[uart_counter - 1])  // 判断包尾，作为接收一组数据的结束标志结束标志
        {
            y_ele_num = store_floating_point_data(buf,buff_y_f);  // 数据处理部分，将字符串中的浮点数提取出来 
            memset(buf,0,uart_counter);
            uart_counter = 0;
        }
        if('R' == buf[1] ) // ,RSX
        {
            if('S' == buf[2])
            {
                log_msg("(buf[3] is %c ！" ,buf[3]);
                if((buf[3]>=A0_id)&&(buf[3]<=A2_id))
                {
                    if(x_ele_num == y_ele_num) // 判断返回的元素的个数是否相等，相等子进行最小二乘法，多项式系数计算 
                    {
                        storing_to_eeprom_anthor_num = buf[3];
                        data_fit_apply();
                    }
                    else                       // 若返回的数量不相等，则清除浮点数组 
                    {
                        printf("element num fail ！");
                        memset(buff_x_f,0,x_ele_num);
                        memset(buff_y_f,0,y_ele_num);
                    }
                    memset(buf,0,uart_counter);
                    uart_counter = 0;
                }
                else if(uart_counter >= cam_max_lenght )
                {
                    memset(buf,0,uart_counter);
                    uart_counter = 0;
                                                    
                }
            }
            else if('T' == buf[2])  // ,RTX
            {
                if(((buf[3]>=A0_id)&&(buf[3]<=A2_id))||(buf[3]==clear_eeprom_cmd))
                {
                    double kkk[6];
                    if(A0_id == buf[3] )
                    {
                        //eeprom_data_storing_read((void*)kkk,A0_eeprom_parak_adr,sizeof(kkk));

                    }
                    else if(A1_id == buf[3] )
                    {
                        //eeprom_data_storing_read((void*)kkk,A1_eeprom_parak_adr,sizeof(kkk));

                    }
                    else if(A2_id == buf[3] )
                    {
                        //eeprom_data_storing_read((void*)kkk,A2_eeprom_parak_adr,sizeof(kkk));

                    }
                    else if(clear_eeprom_cmd == buf[3])
                    {
                        //ee_Erase();  
                    }
                    log_msg("0 %lf \n\r 1 %lf \n\r 2 %lf \n\r 3 %lf \n\r 4 %lf \n\r 5 %lf \n\r ！",\
                    kkk[0],kkk[1],kkk[2],kkk[3],kkk[4],kkk[5] );
                    memset(buf,0,uart_counter);
                    uart_counter = 0;
                    
                }
                else if(uart_counter >= cam_max_lenght )
                {
                    memset(buf,0,uart_counter);
                    uart_counter = 0;
                                                    
                }
            }
            else if( uart_counter >= start_cal_cmd_lenght)
            {
                memset(buf,0,uart_counter);
                uart_counter = 0;
            }
        }
    }
    else
    {
        memset(buf,0,uart_counter);
        uart_counter = 0;
    }
    if(uart_counter >= uart_counter_max)
    {
        memset(buf,0,uart_counter);
        uart_counter = 0;
    }
		 
}

uint8_t  a0_eeprom_parameters_exist_flag = false,
         a1_eeprom_parameters_exist_flag = false,
         a2_eeprom_parameters_exist_flag = false;  
void  get_eepprom_parameters( void )
{
    double   minus_zore_data  = -0.0001 ; 
    double   zero_data        = 0.00001 ;
    #define  zore              minus_zore_data   
    #define  float_to_int_mul  (1000000)
		
	// 读取eeprom中基站0的最小二乘法的六阶参数  
    //eeprom_data_storing_read((void*)A0_g_parak,A0_eeprom_parak_adr,sizeof(A0_g_parak));
    log_msg("0 %lf \n\r 1 %lf \n\r 2 %lf \n\r 3 %lf \n\r 4 %lf \n\r 5 %lf \n\r ！",\
                                UserFlashCfg.Pkg.polynomial_coeff_ant_0.g_parak[0],\
                                UserFlashCfg.Pkg.polynomial_coeff_ant_0.g_parak[1],\
                                UserFlashCfg.Pkg.polynomial_coeff_ant_0.g_parak[2],\
                                UserFlashCfg.Pkg.polynomial_coeff_ant_0.g_parak[3],\
                                UserFlashCfg.Pkg.polynomial_coeff_ant_0.g_parak[4],\
                                UserFlashCfg.Pkg.polynomial_coeff_ant_0.g_parak[5] );
    // 判断
    if((abs((int)(UserFlashCfg.Pkg.polynomial_coeff_ant_0.g_parak[0]*float_to_int_mul)) > zero_data)||\
       (abs((int)(UserFlashCfg.Pkg.polynomial_coeff_ant_0.g_parak[1]*float_to_int_mul)) > zero_data)||\
       (abs((int)(UserFlashCfg.Pkg.polynomial_coeff_ant_0.g_parak[2]*float_to_int_mul)) > zero_data)||\
       (abs((int)(UserFlashCfg.Pkg.polynomial_coeff_ant_0.g_parak[3]*float_to_int_mul)) > zero_data)||\
       (abs((int)(UserFlashCfg.Pkg.polynomial_coeff_ant_0.g_parak[4]*float_to_int_mul)) > zero_data)||\
       (abs((int)(UserFlashCfg.Pkg.polynomial_coeff_ant_0.g_parak[5]*float_to_int_mul)) > zero_data))
    {
        a0_eeprom_parameters_exist_flag = true;    
        log_msg("A0_g_parak");    
    }
    // 读取eeprom中基站1的最小二乘法的六阶参数  
    //eeprom_data_storing_read((void*)A1_g_parak,A1_eeprom_parak_adr,sizeof(A1_g_parak));
    log_msg("0 %lf \n\r 1 %lf \n\r 2 %lf \n\r 3 %lf \n\r 4 %lf \n\r 5 %lf \n\r ！",\
                                UserFlashCfg.Pkg.polynomial_coeff_ant_1.g_parak[0],\
                                UserFlashCfg.Pkg.polynomial_coeff_ant_1.g_parak[1],\
                                UserFlashCfg.Pkg.polynomial_coeff_ant_1.g_parak[2],\
                                UserFlashCfg.Pkg.polynomial_coeff_ant_1.g_parak[3],\
                                UserFlashCfg.Pkg.polynomial_coeff_ant_1.g_parak[4],\
                                UserFlashCfg.Pkg.polynomial_coeff_ant_1.g_parak[5] );
    if((abs((int)(UserFlashCfg.Pkg.polynomial_coeff_ant_1.g_parak[0]*float_to_int_mul)) > zero_data)||\
       (abs((int)(UserFlashCfg.Pkg.polynomial_coeff_ant_1.g_parak[1]*float_to_int_mul)) > zero_data)||\
       (abs((int)(UserFlashCfg.Pkg.polynomial_coeff_ant_1.g_parak[2]*float_to_int_mul)) > zero_data)||\
       (abs((int)(UserFlashCfg.Pkg.polynomial_coeff_ant_1.g_parak[3]*float_to_int_mul)) > zero_data)||\
       (abs((int)(UserFlashCfg.Pkg.polynomial_coeff_ant_1.g_parak[4]*float_to_int_mul)) > zero_data)||\
       (abs((int)(UserFlashCfg.Pkg.polynomial_coeff_ant_1.g_parak[5]*float_to_int_mul)) > zero_data))
    {
        a1_eeprom_parameters_exist_flag = true;
        log_msg("A1_g_parak");
    }
    // 读取eeprom中基站2的最小二乘法的六阶参数  
    //eeprom_data_storing_read((void*)A2_g_parak,A2_eeprom_parak_adr,sizeof(A2_g_parak));
    
    log_msg("0 %lf \n\r 1 %lf \n\r 2 %lf \n\r 3 %lf \n\r 4 %lf \n\r 5 %lf \n\r ！",\
                                UserFlashCfg.Pkg.polynomial_coeff_ant_2.g_parak[0],\
                                UserFlashCfg.Pkg.polynomial_coeff_ant_2.g_parak[1],\
                                UserFlashCfg.Pkg.polynomial_coeff_ant_2.g_parak[2],\
                                UserFlashCfg.Pkg.polynomial_coeff_ant_2.g_parak[3],\
                                UserFlashCfg.Pkg.polynomial_coeff_ant_2.g_parak[4],\
                                UserFlashCfg.Pkg.polynomial_coeff_ant_2.g_parak[5] );
    //A2_g_parak[0] = -0.00000001;
    if((abs((int)(UserFlashCfg.Pkg.polynomial_coeff_ant_2.g_parak[0]*float_to_int_mul)) > zero_data)||\
       (abs((int)(UserFlashCfg.Pkg.polynomial_coeff_ant_2.g_parak[1]*float_to_int_mul)) > zero_data)||\
       (abs((int)(UserFlashCfg.Pkg.polynomial_coeff_ant_2.g_parak[2]*float_to_int_mul)) > zero_data)||\
       (abs((int)(UserFlashCfg.Pkg.polynomial_coeff_ant_2.g_parak[3]*float_to_int_mul)) > zero_data)||\
       (abs((int)(UserFlashCfg.Pkg.polynomial_coeff_ant_2.g_parak[4]*float_to_int_mul)) > zero_data)||\
       (abs((int)(UserFlashCfg.Pkg.polynomial_coeff_ant_2.g_parak[5]*float_to_int_mul)) > zero_data))
    {
        a2_eeprom_parameters_exist_flag = true;  
        log_msg("A2_g_parak");
    }
    
}

void  range_calibration( double *a_f_data )
{
    static double  dd;
    double  a_f_data_buf[3] = {a_f_data[0],a_f_data[1],a_f_data[2]};
    
    if(true == a0_eeprom_parameters_exist_flag)
    {
        a_f_data[0] =   UserFlashCfg.Pkg.polynomial_coeff_ant_0.g_parak[5] + \
                        UserFlashCfg.Pkg.polynomial_coeff_ant_0.g_parak[4]*a_f_data_buf[0] + \
                        UserFlashCfg.Pkg.polynomial_coeff_ant_0.g_parak[3]*pow(a_f_data_buf[0],2)+ \
                        UserFlashCfg.Pkg.polynomial_coeff_ant_0.g_parak[2]*pow(a_f_data_buf[0],3)+ \
                        UserFlashCfg.Pkg.polynomial_coeff_ant_0.g_parak[1]*pow(a_f_data_buf[0],4) + \
                        UserFlashCfg.Pkg.polynomial_coeff_ant_0.g_parak[0]*pow(a_f_data_buf[0],5);
    }
    if(true == a1_eeprom_parameters_exist_flag)
    {
        a_f_data[1] =   UserFlashCfg.Pkg.polynomial_coeff_ant_1.g_parak[5] + \
                        UserFlashCfg.Pkg.polynomial_coeff_ant_1.g_parak[4]*a_f_data_buf[1] + \
                        UserFlashCfg.Pkg.polynomial_coeff_ant_1.g_parak[3]*pow(a_f_data_buf[1],2)+ \
                        UserFlashCfg.Pkg.polynomial_coeff_ant_1.g_parak[2]*pow(a_f_data_buf[1],3)+ \
                        UserFlashCfg.Pkg.polynomial_coeff_ant_1.g_parak[1]*pow(a_f_data_buf[1],4)+ \
                        UserFlashCfg.Pkg.polynomial_coeff_ant_1.g_parak[0]*pow(a_f_data_buf[1],5);
    }
    if(true == a2_eeprom_parameters_exist_flag)
    {
        a_f_data[2] =   UserFlashCfg.Pkg.polynomial_coeff_ant_2.g_parak[5] + \
                        UserFlashCfg.Pkg.polynomial_coeff_ant_2.g_parak[4]*a_f_data_buf[2] + \
                        UserFlashCfg.Pkg.polynomial_coeff_ant_2.g_parak[3]*pow(a_f_data_buf[2],2) + \
                        UserFlashCfg.Pkg.polynomial_coeff_ant_2.g_parak[2]*pow(a_f_data_buf[2],3) + \
                        UserFlashCfg.Pkg.polynomial_coeff_ant_2.g_parak[1]*pow(a_f_data_buf[2],4) + \
                        UserFlashCfg.Pkg.polynomial_coeff_ant_2.g_parak[0]*pow(a_f_data_buf[2],5);
    }
}




#if 0 

#endif


