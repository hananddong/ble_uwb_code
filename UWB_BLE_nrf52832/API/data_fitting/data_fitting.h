#ifndef _DATA_FITTING_H_
#define _DATA_FITTING_H_
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

//typedef enum 
//{
//    FALSE = 0, 
//    TRUE = !FALSE
//}Bool ;
typedef struct
{
    double g_parak[6];

}Least_square_polynomial_coefficient;



int data_fit_apply(void);
void cb_data_fitting_uart_handler(uint8_t res);
void  get_eepprom_parameters( void );
void  range_calibration( double  *a_f_data );
//inline void store_floating_point_data(char* buff );



#endif


