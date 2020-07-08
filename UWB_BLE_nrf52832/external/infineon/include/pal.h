/**
    \copyright
    Copyright (c) 2018, Infineon Technologies AG
    All rights reserved.

    This software is provided with terms and conditions as specified in OPTIGA(TM) Trust X Evaluation Kit License Agreement.
    \endcopyright

    \author Infineon AG

    \file

    \brief This file implements the prototype declarations of platform abstraction layer

    \addtogroup  grPAL
    @{
*/


#ifndef _PAL_H_
#define _PAL_H_

/**********************************************************************************************************************
    HEADER FILES
 *********************************************************************************************************************/
#include <Datatypes.h>

/**********************************************************************************************************************
    pal.h
 *********************************************************************************************************************/

/**********************************************************************************************************************
    MACROS
 *********************************************************************************************************************/

/// PAL API execution is successful
#define PAL_STATUS_SUCCESS      (0x0000)
/// PAL API execution failed
#define PAL_STATUS_FAILURE      (0x0001)
/// PAL I2C is busy
#define PAL_STATUS_I2C_BUSY     (0x0002)

/**********************************************************************************************************************
    ENUMS
 *********************************************************************************************************************/
/**
    \brief PAL return status.
*/
typedef uint16_t pal_status_t;

/**********************************************************************************************************************
    API Prototypes
 *********************************************************************************************************************/

#endif /* _PAL_H_ */

/**
    @}
*/
