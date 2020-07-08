/**
    Copyright (c) 2017 - 2018, Nordic Semiconductor ASA

    All rights reserved.

    Redistribution and use in source and binary forms, with or without modification,
    are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this
      list of conditions and the following disclaimer.

    2. Redistributions in binary form, except as embedded into a Nordic
      Semiconductor ASA integrated circuit in a product or a software update for
      such product, must reproduce the above copyright notice, this list of
      conditions and the following disclaimer in the documentation and/or other
      materials provided with the distribution.

    3. Neither the name of Nordic Semiconductor ASA nor the names of its
      contributors may be used to endorse or promote products derived from this
      software without specific prior written permission.

    4. This software, with or without modification, must only be used with a
      Nordic Semiconductor ASA integrated circuit.

    5. Any software provided in binary form under this license must not be reverse
      engineered, decompiled, modified and/or disassembled.

    THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
    OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
    OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
    LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
    CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
    GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
    HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
    OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/


#ifndef APPLY_OLD_CONFIG_H__
#define APPLY_OLD_CONFIG_H__

//------------------------------------------------------------------------------
// Peripheral Resource Sharing (PRS)

#if defined(PERIPHERAL_RESOURCE_SHARING_ENABLED)
    
    #define NRFX_PRS_ENABLED  PERIPHERAL_RESOURCE_SHARING_ENABLED
    #define NRFX_PRS_BOX_0_ENABLED  PERIPHERAL_RESOURCE_SHARING_ENABLED
    #define NRFX_PRS_BOX_1_ENABLED  PERIPHERAL_RESOURCE_SHARING_ENABLED
    #define NRFX_PRS_BOX_2_ENABLED  PERIPHERAL_RESOURCE_SHARING_ENABLED
    #define NRFX_PRS_BOX_3_ENABLED  PERIPHERAL_RESOURCE_SHARING_ENABLED
    #define NRFX_PRS_BOX_4_ENABLED  PERIPHERAL_RESOURCE_SHARING_ENABLED
    
    #if defined(COMMON_CONFIG_LOG_ENABLED)
        #undef NRFX_PRS_CONFIG_LOG_ENABLED
        #define NRFX_PRS_CONFIG_LOG_ENABLED  COMMON_CONFIG_LOG_ENABLED
    #endif
    #if defined(COMMON_CONFIG_LOG_LEVEL)
        #undef NRFX_PRS_CONFIG_LOG_LEVEL
        #define NRFX_PRS_CONFIG_LOG_LEVEL  COMMON_CONFIG_LOG_LEVEL
    #endif
    #if defined(COMMON_CONFIG_INFO_COLOR)
        #undef NRFX_PRS_CONFIG_INFO_COLOR
        #define NRFX_PRS_CONFIG_INFO_COLOR  COMMON_CONFIG_INFO_COLOR
    #endif
    #if defined(COMMON_CONFIG_DEBUG_COLOR)
        #undef NRFX_PRS_CONFIG_DEBUG_COLOR
        #define NRFX_PRS_CONFIG_DEBUG_COLOR  COMMON_CONFIG_DEBUG_COLOR
    #endif
    
#endif // defined(PERIPHERAL_RESOURCE_SHARING_ENABLED)

//------------------------------------------------------------------------------
// CLOCK

#if defined(NRF_CLOCK_ENABLED)
    
    #undef NRFX_CLOCK_ENABLED
    #define NRFX_CLOCK_ENABLED  NRF_CLOCK_ENABLED
    
    #if defined(CLOCK_CONFIG_LF_SRC)
        #undef NRFX_CLOCK_CONFIG_LF_SRC
        #define NRFX_CLOCK_CONFIG_LF_SRC  CLOCK_CONFIG_LF_SRC
    #endif
    #if defined(CLOCK_CONFIG_IRQ_PRIORITY)
        #undef NRFX_CLOCK_CONFIG_IRQ_PRIORITY
        #define NRFX_CLOCK_CONFIG_IRQ_PRIORITY  CLOCK_CONFIG_IRQ_PRIORITY
    #endif
    
    #if defined(CLOCK_CONFIG_LOG_ENABLED)
        #undef NRFX_CLOCK_CONFIG_LOG_ENABLED
        #define NRFX_CLOCK_CONFIG_LOG_ENABLED  CLOCK_CONFIG_LOG_ENABLED
    #endif
    #if defined(CLOCK_CONFIG_LOG_LEVEL)
        #undef NRFX_CLOCK_CONFIG_LOG_LEVEL
        #define NRFX_CLOCK_CONFIG_LOG_LEVEL  CLOCK_CONFIG_LOG_LEVEL
    #endif
    #if defined(CLOCK_CONFIG_INFO_COLOR)
        #undef NRFX_CLOCK_CONFIG_INFO_COLOR
        #define NRFX_CLOCK_CONFIG_INFO_COLOR  CLOCK_CONFIG_INFO_COLOR
    #endif
    #if defined(CLOCK_CONFIG_DEBUG_COLOR)
        #undef NRFX_CLOCK_CONFIG_DEBUG_COLOR
        #define NRFX_CLOCK_CONFIG_DEBUG_COLOR  CLOCK_CONFIG_DEBUG_COLOR
    #endif
    
#endif // defined(NRF_CLOCK_ENABLED)

//------------------------------------------------------------------------------
// COMP

#if defined(COMP_ENABLED)
    
    #undef NRFX_COMP_ENABLED
    #define NRFX_COMP_ENABLED  COMP_ENABLED
    
    #if defined(COMP_CONFIG_REF)
        #undef NRFX_COMP_CONFIG_REF
        #define NRFX_COMP_CONFIG_REF  COMP_CONFIG_REF
    #endif
    #if defined(COMP_CONFIG_MAIN_MODE)
        #undef NRFX_COMP_CONFIG_MAIN_MODE
        #define NRFX_COMP_CONFIG_MAIN_MODE  COMP_CONFIG_MAIN_MODE
    #endif
    #if defined(COMP_CONFIG_SPEED_MODE)
        #undef NRFX_COMP_CONFIG_SPEED_MODE
        #define NRFX_COMP_CONFIG_SPEED_MODE  COMP_CONFIG_SPEED_MODE
    #endif
    #if defined(COMP_CONFIG_HYST)
        #undef NRFX_COMP_CONFIG_HYST
        #define NRFX_COMP_CONFIG_HYST  COMP_CONFIG_HYST
    #endif
    #if defined(COMP_CONFIG_ISOURCE)
        #undef NRFX_COMP_CONFIG_ISOURCE
        #define NRFX_COMP_CONFIG_ISOURCE  COMP_CONFIG_ISOURCE
    #endif
    #if defined(COMP_CONFIG_INPUT)
        #undef NRFX_COMP_CONFIG_INPUT
        #define NRFX_COMP_CONFIG_INPUT  COMP_CONFIG_INPUT
    #endif
    #if defined(COMP_CONFIG_IRQ_PRIORITY)
        #undef NRFX_COMP_CONFIG_IRQ_PRIORITY
        #define NRFX_COMP_CONFIG_IRQ_PRIORITY  COMP_CONFIG_IRQ_PRIORITY
    #endif
    
    #if defined(COMP_CONFIG_LOG_ENABLED)
        #undef NRFX_COMP_CONFIG_LOG_ENABLED
        #define NRFX_COMP_CONFIG_LOG_ENABLED  COMP_CONFIG_LOG_ENABLED
    #endif
    #if defined(COMP_CONFIG_LOG_LEVEL)
        #undef NRFX_COMP_CONFIG_LOG_LEVEL
        #define NRFX_COMP_CONFIG_LOG_LEVEL  COMP_CONFIG_LOG_LEVEL
    #endif
    #if defined(COMP_CONFIG_INFO_COLOR)
        #undef NRFX_COMP_CONFIG_INFO_COLOR
        #define NRFX_COMP_CONFIG_INFO_COLOR  COMP_CONFIG_INFO_COLOR
    #endif
    #if defined(COMP_CONFIG_DEBUG_COLOR)
        #undef NRFX_COMP_CONFIG_DEBUG_COLOR
        #define NRFX_COMP_CONFIG_DEBUG_COLOR  COMP_CONFIG_DEBUG_COLOR
    #endif
    
#endif // defined(COMP_ENABLED)

//------------------------------------------------------------------------------
// GPIOTE

#if defined(GPIOTE_ENABLED)
    
    #undef NRFX_GPIOTE_ENABLED
    #define NRFX_GPIOTE_ENABLED  GPIOTE_ENABLED
    
    #if defined(GPIOTE_CONFIG_NUM_OF_LOW_POWER_EVENTS)
        #undef NRFX_GPIOTE_CONFIG_NUM_OF_LOW_POWER_EVENTS
        #define NRFX_GPIOTE_CONFIG_NUM_OF_LOW_POWER_EVENTS  GPIOTE_CONFIG_NUM_OF_LOW_POWER_EVENTS
    #endif
    
    #if defined(GPIOTE_CONFIG_IRQ_PRIORITY)
        #undef NRFX_GPIOTE_CONFIG_IRQ_PRIORITY
        #define NRFX_GPIOTE_CONFIG_IRQ_PRIORITY  GPIOTE_CONFIG_IRQ_PRIORITY
    #endif
    
    #if defined(GPIOTE_CONFIG_LOG_ENABLED)
        #undef NRFX_GPIOTE_CONFIG_LOG_ENABLED
        #define NRFX_GPIOTE_CONFIG_LOG_ENABLED  GPIOTE_CONFIG_LOG_ENABLED
    #endif
    #if defined(GPIOTE_CONFIG_LOG_LEVEL)
        #undef NRFX_GPIOTE_CONFIG_LOG_LEVEL
        #define NRFX_GPIOTE_CONFIG_LOG_LEVEL  GPIOTE_CONFIG_LOG_LEVEL
    #endif
    #if defined(GPIOTE_CONFIG_INFO_COLOR)
        #undef NRFX_GPIOTE_CONFIG_INFO_COLOR
        #define NRFX_GPIOTE_CONFIG_INFO_COLOR  GPIOTE_CONFIG_INFO_COLOR
    #endif
    #if defined(GPIOTE_CONFIG_DEBUG_COLOR)
        #undef NRFX_GPIOTE_CONFIG_DEBUG_COLOR
        #define NRFX_GPIOTE_CONFIG_DEBUG_COLOR  GPIOTE_CONFIG_DEBUG_COLOR
    #endif
    
#endif // defined(GPIOTE_ENABLED)

//------------------------------------------------------------------------------
// I2S

#if defined(I2S_ENABLED)
    
    #undef NRFX_I2S_ENABLED
    #define NRFX_I2S_ENABLED  I2S_ENABLED
    
    #if defined(I2S_CONFIG_SCK_PIN)
        #undef NRFX_I2S_CONFIG_SCK_PIN
        #define NRFX_I2S_CONFIG_SCK_PIN  I2S_CONFIG_SCK_PIN
    #endif
    #if defined(I2S_CONFIG_LRCK_PIN)
        #undef NRFX_I2S_CONFIG_LRCK_PIN
        #define NRFX_I2S_CONFIG_LRCK_PIN  I2S_CONFIG_LRCK_PIN
    #endif
    #if defined(I2S_CONFIG_MCK_PIN)
        #undef NRFX_I2S_CONFIG_MCK_PIN
        #define NRFX_I2S_CONFIG_MCK_PIN  I2S_CONFIG_MCK_PIN
    #endif
    #if defined(I2S_CONFIG_SDOUT_PIN)
        #undef NRFX_I2S_CONFIG_SDOUT_PIN
        #define NRFX_I2S_CONFIG_SDOUT_PIN  I2S_CONFIG_SDOUT_PIN
    #endif
    #if defined(I2S_CONFIG_SDIN_PIN)
        #undef NRFX_I2S_CONFIG_SDIN_PIN
        #define NRFX_I2S_CONFIG_SDIN_PIN  I2S_CONFIG_SDIN_PIN
    #endif
    
    #if defined(I2S_CONFIG_MASTER)
        #undef NRFX_I2S_CONFIG_MASTER
        #define NRFX_I2S_CONFIG_MASTER  I2S_CONFIG_MASTER
    #endif
    #if defined(I2S_CONFIG_FORMAT)
        #undef NRFX_I2S_CONFIG_FORMAT
        #define NRFX_I2S_CONFIG_FORMAT  I2S_CONFIG_FORMAT
    #endif
    #if defined(I2S_CONFIG_ALIGN)
        #undef NRFX_I2S_CONFIG_ALIGN
        #define NRFX_I2S_CONFIG_ALIGN  I2S_CONFIG_ALIGN
    #endif
    #if defined(I2S_CONFIG_SWIDTH)
        #undef NRFX_I2S_CONFIG_SWIDTH
        #define NRFX_I2S_CONFIG_SWIDTH  I2S_CONFIG_SWIDTH
    #endif
    #if defined(I2S_CONFIG_CHANNELS)
        #undef NRFX_I2S_CONFIG_CHANNELS
        #define NRFX_I2S_CONFIG_CHANNELS  I2S_CONFIG_CHANNELS
    #endif
    #if defined(I2S_CONFIG_MCK_SETUP)
        #undef NRFX_I2S_CONFIG_MCK_SETUP
        #define NRFX_I2S_CONFIG_MCK_SETUP  I2S_CONFIG_MCK_SETUP
    #endif
    #if defined(I2S_CONFIG_RATIO)
        #undef NRFX_I2S_CONFIG_RATIO
        #define NRFX_I2S_CONFIG_RATIO  I2S_CONFIG_RATIO
    #endif
    #if defined(I2S_CONFIG_IRQ_PRIORITY)
        #undef NRFX_I2S_CONFIG_IRQ_PRIORITY
        #define NRFX_I2S_CONFIG_IRQ_PRIORITY  I2S_CONFIG_IRQ_PRIORITY
    #endif
    
    #if defined(I2S_CONFIG_LOG_ENABLED)
        #undef NRFX_I2S_CONFIG_LOG_ENABLED
        #define NRFX_I2S_CONFIG_LOG_ENABLED  I2S_CONFIG_LOG_ENABLED
    #endif
    #if defined(I2S_CONFIG_LOG_LEVEL)
        #undef NRFX_I2S_CONFIG_LOG_LEVEL
        #define NRFX_I2S_CONFIG_LOG_LEVEL  I2S_CONFIG_LOG_LEVEL
    #endif
    #if defined(I2S_CONFIG_INFO_COLOR)
        #undef NRFX_I2S_CONFIG_INFO_COLOR
        #define NRFX_I2S_CONFIG_INFO_COLOR  I2S_CONFIG_INFO_COLOR
    #endif
    #if defined(I2S_CONFIG_DEBUG_COLOR)
        #undef NRFX_I2S_CONFIG_DEBUG_COLOR
        #define NRFX_I2S_CONFIG_DEBUG_COLOR  I2S_CONFIG_DEBUG_COLOR
    #endif
    
#endif // defined(I2S_ENABLED)

//------------------------------------------------------------------------------
// LPCOMP

#if defined(LPCOMP_ENABLED)
    
    #undef NRFX_LPCOMP_ENABLED
    #define NRFX_LPCOMP_ENABLED  LPCOMP_ENABLED
    
    #if defined(LPCOMP_CONFIG_REFERENCE)
        #undef NRFX_LPCOMP_CONFIG_REFERENCE
        #define NRFX_LPCOMP_CONFIG_REFERENCE  LPCOMP_CONFIG_REFERENCE
    #endif
    #if defined(LPCOMP_CONFIG_DETECTION)
        #undef NRFX_LPCOMP_CONFIG_DETECTION
        #define NRFX_LPCOMP_CONFIG_DETECTION  LPCOMP_CONFIG_DETECTION
    #endif
    #if defined(LPCOMP_CONFIG_INPUT)
        #undef NRFX_LPCOMP_CONFIG_INPUT
        #define NRFX_LPCOMP_CONFIG_INPUT  LPCOMP_CONFIG_INPUT
    #endif
    #if defined(LPCOMP_CONFIG_HYST)
        #undef NRFX_LPCOMP_CONFIG_HYST
        #define NRFX_LPCOMP_CONFIG_HYST  LPCOMP_CONFIG_HYST
    #endif
    #if defined(LPCOMP_CONFIG_IRQ_PRIORITY)
        #undef NRFX_LPCOMP_CONFIG_IRQ_PRIORITY
        #define NRFX_LPCOMP_CONFIG_IRQ_PRIORITY  LPCOMP_CONFIG_IRQ_PRIORITY
    #endif
    
    #if defined(LPCOMP_CONFIG_LOG_ENABLED)
        #undef NRFX_LPCOMP_CONFIG_LOG_ENABLED
        #define NRFX_LPCOMP_CONFIG_LOG_ENABLED  LPCOMP_CONFIG_LOG_ENABLED
    #endif
    #if defined(LPCOMP_CONFIG_LOG_LEVEL)
        #undef NRFX_LPCOMP_CONFIG_LOG_LEVEL
        #define NRFX_LPCOMP_CONFIG_LOG_LEVEL  LPCOMP_CONFIG_LOG_LEVEL
    #endif
    #if defined(LPCOMP_CONFIG_INFO_COLOR)
        #undef NRFX_LPCOMP_CONFIG_INFO_COLOR
        #define NRFX_LPCOMP_CONFIG_INFO_COLOR  LPCOMP_CONFIG_INFO_COLOR
    #endif
    #if defined(LPCOMP_CONFIG_DEBUG_COLOR)
        #undef NRFX_LPCOMP_CONFIG_DEBUG_COLOR
        #define NRFX_LPCOMP_CONFIG_DEBUG_COLOR  LPCOMP_CONFIG_DEBUG_COLOR
    #endif
    
#endif // defined(LPCOMP_ENABLED)

//------------------------------------------------------------------------------
// PDM

#if defined(PDM_ENABLED)
    
    #undef NRFX_PDM_ENABLED
    #define NRFX_PDM_ENABLED  PDM_ENABLED
    
    #if defined(PDM_CONFIG_MODE)
        #undef NRFX_PDM_CONFIG_MODE
        #define NRFX_PDM_CONFIG_MODE  PDM_CONFIG_MODE
    #endif
    #if defined(PDM_CONFIG_EDGE)
        #undef NRFX_PDM_CONFIG_EDGE
        #define NRFX_PDM_CONFIG_EDGE  PDM_CONFIG_EDGE
    #endif
    #if defined(PDM_CONFIG_CLOCK_FREQ)
        #undef NRFX_PDM_CONFIG_CLOCK_FREQ
        #define NRFX_PDM_CONFIG_CLOCK_FREQ  PDM_CONFIG_CLOCK_FREQ
    #endif
    #if defined(PDM_CONFIG_IRQ_PRIORITY)
        #undef NRFX_PDM_CONFIG_IRQ_PRIORITY
        #define NRFX_PDM_CONFIG_IRQ_PRIORITY  PDM_CONFIG_IRQ_PRIORITY
    #endif
    
    #if defined(PDM_CONFIG_LOG_ENABLED)
        #undef NRFX_PDM_CONFIG_LOG_ENABLED
        #define NRFX_PDM_CONFIG_LOG_ENABLED  PDM_CONFIG_LOG_ENABLED
    #endif
    #if defined(PDM_CONFIG_LOG_LEVEL)
        #undef NRFX_PDM_CONFIG_LOG_LEVEL
        #define NRFX_PDM_CONFIG_LOG_LEVEL  PDM_CONFIG_LOG_LEVEL
    #endif
    #if defined(PDM_CONFIG_INFO_COLOR)
        #undef NRFX_PDM_CONFIG_INFO_COLOR
        #define NRFX_PDM_CONFIG_INFO_COLOR  PDM_CONFIG_INFO_COLOR
    #endif
    #if defined(PDM_CONFIG_DEBUG_COLOR)
        #undef NRFX_PDM_CONFIG_DEBUG_COLOR
        #define NRFX_PDM_CONFIG_DEBUG_COLOR  PDM_CONFIG_DEBUG_COLOR
    #endif
    
#endif // defined(PDM_ENABLED)

//------------------------------------------------------------------------------
// POWER

#if defined(POWER_ENABLED)
    
    #undef NRFX_POWER_ENABLED
    #define NRFX_POWER_ENABLED  POWER_ENABLED
    
    #if defined(POWER_CONFIG_IRQ_PRIORITY)
        #undef NRFX_POWER_CONFIG_IRQ_PRIORITY
        #define NRFX_POWER_CONFIG_IRQ_PRIORITY  POWER_CONFIG_IRQ_PRIORITY
    #endif
    
    #if defined(POWER_CONFIG_DEFAULT_DCDCEN)
        #undef NRFX_POWER_CONFIG_DEFAULT_DCDCEN
        #define NRFX_POWER_CONFIG_DEFAULT_DCDCEN  POWER_CONFIG_DEFAULT_DCDCEN
    #endif
    #if defined(POWER_CONFIG_DEFAULT_DCDCENHV)
        #undef NRFX_POWER_CONFIG_DEFAULT_DCDCENHV
        #define NRFX_POWER_CONFIG_DEFAULT_DCDCENHV  POWER_CONFIG_DEFAULT_DCDCENHV
    #endif
    
#endif // defined(POWER_ENABLED)

//------------------------------------------------------------------------------
// PPI

#if defined(PPI_ENABLED)
    
    #undef NRFX_PPI_ENABLED
    #define NRFX_PPI_ENABLED  PPI_ENABLED
    
    #if defined(PPI_CONFIG_LOG_ENABLED)
        #undef NRFX_PPI_CONFIG_LOG_ENABLED
        #define NRFX_PPI_CONFIG_LOG_ENABLED  PPI_CONFIG_LOG_ENABLED
    #endif
    #if defined(PPI_CONFIG_LOG_LEVEL)
        #undef NRFX_PPI_CONFIG_LOG_LEVEL
        #define NRFX_PPI_CONFIG_LOG_LEVEL  PPI_CONFIG_LOG_LEVEL
    #endif
    #if defined(PPI_CONFIG_INFO_COLOR)
        #undef NRFX_PPI_CONFIG_INFO_COLOR
        #define NRFX_PPI_CONFIG_INFO_COLOR  PPI_CONFIG_INFO_COLOR
    #endif
    #if defined(PPI_CONFIG_DEBUG_COLOR)
        #undef NRFX_PPI_CONFIG_DEBUG_COLOR
        #define NRFX_PPI_CONFIG_DEBUG_COLOR  PPI_CONFIG_DEBUG_COLOR
    #endif
    
#endif // defined(PPI_ENABLED)

//------------------------------------------------------------------------------
// PWM

#if defined(PWM_ENABLED)
    
    #undef NRFX_PWM_ENABLED
    #define NRFX_PWM_ENABLED  PWM_ENABLED
    
    #if defined(PWM0_ENABLED)
        #undef NRFX_PWM0_ENABLED
        #define NRFX_PWM0_ENABLED  PWM0_ENABLED
    #endif
    #if defined(PWM1_ENABLED)
        #undef NRFX_PWM1_ENABLED
        #define NRFX_PWM1_ENABLED  PWM1_ENABLED
    #endif
    #if defined(PWM2_ENABLED)
        #undef NRFX_PWM2_ENABLED
        #define NRFX_PWM2_ENABLED  PWM2_ENABLED
    #endif
    #if defined(PWM3_ENABLED)
        #undef NRFX_PWM3_ENABLED
        #define NRFX_PWM3_ENABLED  PWM3_ENABLED
    #endif
    
    #if defined(PWM_DEFAULT_CONFIG_OUT0_PIN)
        #undef NRFX_PWM_DEFAULT_CONFIG_OUT0_PIN
        #define NRFX_PWM_DEFAULT_CONFIG_OUT0_PIN  PWM_DEFAULT_CONFIG_OUT0_PIN
    #endif
    #if defined(PWM_DEFAULT_CONFIG_OUT1_PIN)
        #undef NRFX_PWM_DEFAULT_CONFIG_OUT1_PIN
        #define NRFX_PWM_DEFAULT_CONFIG_OUT1_PIN  PWM_DEFAULT_CONFIG_OUT1_PIN
    #endif
    #if defined(PWM_DEFAULT_CONFIG_OUT2_PIN)
        #undef NRFX_PWM_DEFAULT_CONFIG_OUT2_PIN
        #define NRFX_PWM_DEFAULT_CONFIG_OUT2_PIN  PWM_DEFAULT_CONFIG_OUT2_PIN
    #endif
    #if defined(PWM_DEFAULT_CONFIG_OUT3_PIN)
        #undef NRFX_PWM_DEFAULT_CONFIG_OUT3_PIN
        #define NRFX_PWM_DEFAULT_CONFIG_OUT3_PIN  PWM_DEFAULT_CONFIG_OUT3_PIN
    #endif
    #if defined(PWM_DEFAULT_CONFIG_BASE_CLOCK)
        #undef NRFX_PWM_DEFAULT_CONFIG_BASE_CLOCK
        #define NRFX_PWM_DEFAULT_CONFIG_BASE_CLOCK  PWM_DEFAULT_CONFIG_BASE_CLOCK
    #endif
    #if defined(PWM_DEFAULT_CONFIG_COUNT_MODE)
        #undef NRFX_PWM_DEFAULT_CONFIG_COUNT_MODE
        #define NRFX_PWM_DEFAULT_CONFIG_COUNT_MODE  PWM_DEFAULT_CONFIG_COUNT_MODE
    #endif
    #if defined(PWM_DEFAULT_CONFIG_TOP_VALUE)
        #undef NRFX_PWM_DEFAULT_CONFIG_TOP_VALUE
        #define NRFX_PWM_DEFAULT_CONFIG_TOP_VALUE  PWM_DEFAULT_CONFIG_TOP_VALUE
    #endif
    #if defined(PWM_DEFAULT_CONFIG_LOAD_MODE)
        #undef NRFX_PWM_DEFAULT_CONFIG_LOAD_MODE
        #define NRFX_PWM_DEFAULT_CONFIG_LOAD_MODE  PWM_DEFAULT_CONFIG_LOAD_MODE
    #endif
    #if defined(PWM_DEFAULT_CONFIG_STEP_MODE)
        #undef NRFX_PWM_DEFAULT_CONFIG_STEP_MODE
        #define NRFX_PWM_DEFAULT_CONFIG_STEP_MODE  PWM_DEFAULT_CONFIG_STEP_MODE
    #endif
    #if defined(PWM_DEFAULT_CONFIG_IRQ_PRIORITY)
        #undef NRFX_PWM_DEFAULT_CONFIG_IRQ_PRIORITY
        #define NRFX_PWM_DEFAULT_CONFIG_IRQ_PRIORITY  PWM_DEFAULT_CONFIG_IRQ_PRIORITY
    #endif
    
    #if defined(PWM_CONFIG_LOG_ENABLED)
        #undef NRFX_PWM_CONFIG_LOG_ENABLED
        #define NRFX_PWM_CONFIG_LOG_ENABLED  PWM_CONFIG_LOG_ENABLED
    #endif
    #if defined(PWM_CONFIG_LOG_LEVEL)
        #undef NRFX_PWM_CONFIG_LOG_LEVEL
        #define NRFX_PWM_CONFIG_LOG_LEVEL  PWM_CONFIG_LOG_LEVEL
    #endif
    #if defined(PWM_CONFIG_INFO_COLOR)
        #undef NRFX_PWM_CONFIG_INFO_COLOR
        #define NRFX_PWM_CONFIG_INFO_COLOR  PWM_CONFIG_INFO_COLOR
    #endif
    #if defined(PWM_CONFIG_DEBUG_COLOR)
        #undef NRFX_PWM_CONFIG_DEBUG_COLOR
        #define NRFX_PWM_CONFIG_DEBUG_COLOR  PWM_CONFIG_DEBUG_COLOR
    #endif
    
    #if defined(PWM_NRF52_ANOMALY_109_WORKAROUND_ENABLED)
        #undef NRFX_PWM_NRF52_ANOMALY_109_WORKAROUND_ENABLED
        #define NRFX_PWM_NRF52_ANOMALY_109_WORKAROUND_ENABLED  PWM_NRF52_ANOMALY_109_WORKAROUND_ENABLED
    #endif
    #if defined(PWM_NRF52_ANOMALY_109_EGU_INSTANCE)
        #undef NRFX_PWM_NRF52_ANOMALY_109_EGU_INSTANCE
        #define NRFX_PWM_NRF52_ANOMALY_109_EGU_INSTANCE  PWM_NRF52_ANOMALY_109_EGU_INSTANCE
    #endif
    
#endif // defined(PWM_ENABLED)

//------------------------------------------------------------------------------
// QDEC

#if defined(QDEC_ENABLED)
    
    #undef NRFX_QDEC_ENABLED
    #define NRFX_QDEC_ENABLED  QDEC_ENABLED
    
    #if defined(QDEC_CONFIG_REPORTPER)
        #undef NRFX_QDEC_CONFIG_REPORTPER
        #define NRFX_QDEC_CONFIG_REPORTPER  QDEC_CONFIG_REPORTPER
    #endif
    #if defined(QDEC_CONFIG_SAMPLEPER)
        #undef NRFX_QDEC_CONFIG_SAMPLEPER
        #define NRFX_QDEC_CONFIG_SAMPLEPER  QDEC_CONFIG_SAMPLEPER
    #endif
    #if defined(QDEC_CONFIG_PIO_A)
        #undef NRFX_QDEC_CONFIG_PIO_A
        #define NRFX_QDEC_CONFIG_PIO_A  QDEC_CONFIG_PIO_A
    #endif
    #if defined(QDEC_CONFIG_PIO_B)
        #undef NRFX_QDEC_CONFIG_PIO_B
        #define NRFX_QDEC_CONFIG_PIO_B  QDEC_CONFIG_PIO_B
    #endif
    #if defined(QDEC_CONFIG_PIO_LED)
        #undef NRFX_QDEC_CONFIG_PIO_LED
        #define NRFX_QDEC_CONFIG_PIO_LED  QDEC_CONFIG_PIO_LED
    #endif
    #if defined(QDEC_CONFIG_LEDPRE)
        #undef NRFX_QDEC_CONFIG_LEDPRE
        #define NRFX_QDEC_CONFIG_LEDPRE  QDEC_CONFIG_LEDPRE
    #endif
    #if defined(QDEC_CONFIG_LEDPOL)
        #undef NRFX_QDEC_CONFIG_LEDPOL
        #define NRFX_QDEC_CONFIG_LEDPOL  QDEC_CONFIG_LEDPOL
    #endif
    #if defined(QDEC_CONFIG_DBFEN)
        #undef NRFX_QDEC_CONFIG_DBFEN
        #define NRFX_QDEC_CONFIG_DBFEN  QDEC_CONFIG_DBFEN
    #endif
    #if defined(QDEC_CONFIG_SAMPLE_INTEN)
        #undef NRFX_QDEC_CONFIG_SAMPLE_INTEN
        #define NRFX_QDEC_CONFIG_SAMPLE_INTEN  QDEC_CONFIG_SAMPLE_INTEN
    #endif
    #if defined(QDEC_CONFIG_IRQ_PRIORITY)
        #undef NRFX_QDEC_CONFIG_IRQ_PRIORITY
        #define NRFX_QDEC_CONFIG_IRQ_PRIORITY  QDEC_CONFIG_IRQ_PRIORITY
    #endif
    
    #if defined(QDEC_CONFIG_LOG_ENABLED)
        #undef NRFX_QDEC_CONFIG_LOG_ENABLED
        #define NRFX_QDEC_CONFIG_LOG_ENABLED  QDEC_CONFIG_LOG_ENABLED
    #endif
    #if defined(QDEC_CONFIG_LOG_LEVEL)
        #undef NRFX_QDEC_CONFIG_LOG_LEVEL
        #define NRFX_QDEC_CONFIG_LOG_LEVEL  QDEC_CONFIG_LOG_LEVEL
    #endif
    #if defined(QDEC_CONFIG_INFO_COLOR)
        #undef NRFX_QDEC_CONFIG_INFO_COLOR
        #define NRFX_QDEC_CONFIG_INFO_COLOR  QDEC_CONFIG_INFO_COLOR
    #endif
    #if defined(QDEC_CONFIG_DEBUG_COLOR)
        #undef NRFX_QDEC_CONFIG_DEBUG_COLOR
        #define NRFX_QDEC_CONFIG_DEBUG_COLOR  QDEC_CONFIG_DEBUG_COLOR
    #endif
    
#endif // defined(QDEC_ENABLED)

//------------------------------------------------------------------------------
// QSPI

#if defined(QSPI_ENABLED)
    
    #undef NRFX_QSPI_ENABLED
    #define NRFX_QSPI_ENABLED  QSPI_ENABLED
    
    #if defined(QSPI_CONFIG_SCK_DELAY)
        #undef NRFX_QSPI_CONFIG_SCK_DELAY
        #define NRFX_QSPI_CONFIG_SCK_DELAY  QSPI_CONFIG_SCK_DELAY
    #endif
    #if defined(QSPI_CONFIG_XIP_OFFSET)
        #undef NRFX_QSPI_CONFIG_XIP_OFFSET
        #define NRFX_QSPI_CONFIG_XIP_OFFSET  QSPI_CONFIG_XIP_OFFSET
    #endif
    #if defined(QSPI_CONFIG_READOC)
        #undef NRFX_QSPI_CONFIG_READOC
        #define NRFX_QSPI_CONFIG_READOC  QSPI_CONFIG_READOC
    #endif
    #if defined(QSPI_CONFIG_WRITEOC)
        #undef NRFX_QSPI_CONFIG_WRITEOC
        #define NRFX_QSPI_CONFIG_WRITEOC  QSPI_CONFIG_WRITEOC
    #endif
    #if defined(QSPI_CONFIG_ADDRMODE)
        #undef NRFX_QSPI_CONFIG_ADDRMODE
        #define NRFX_QSPI_CONFIG_ADDRMODE  QSPI_CONFIG_ADDRMODE
    #endif
    #if defined(QSPI_CONFIG_MODE)
        #undef NRFX_QSPI_CONFIG_MODE
        #define NRFX_QSPI_CONFIG_MODE  QSPI_CONFIG_MODE
    #endif
    #if defined(QSPI_CONFIG_FREQUENCY)
        #undef NRFX_QSPI_CONFIG_FREQUENCY
        #define NRFX_QSPI_CONFIG_FREQUENCY  QSPI_CONFIG_FREQUENCY
    #endif
    #if defined(QSPI_CONFIG_IRQ_PRIORITY)
        #undef NRFX_QSPI_CONFIG_IRQ_PRIORITY
        #define NRFX_QSPI_CONFIG_IRQ_PRIORITY  QSPI_CONFIG_IRQ_PRIORITY
    #endif
    
    #if defined(QSPI_PIN_SCK)
        #undef NRFX_QSPI_PIN_SCK
        #define NRFX_QSPI_PIN_SCK  QSPI_PIN_SCK
    #endif
    #if defined(QSPI_PIN_CSN)
        #undef NRFX_QSPI_PIN_CSN
        #define NRFX_QSPI_PIN_CSN  QSPI_PIN_CSN
    #endif
    #if defined(QSPI_PIN_IO0)
        #undef NRFX_QSPI_PIN_IO0
        #define NRFX_QSPI_PIN_IO0  QSPI_PIN_IO0
    #endif
    #if defined(QSPI_PIN_IO0)
        #undef NRFX_QSPI_PIN_IO0
        #define NRFX_QSPI_PIN_IO0  QSPI_PIN_IO0
    #endif
    #if defined(QSPI_PIN_IO1)
        #undef NRFX_QSPI_PIN_IO1
        #define NRFX_QSPI_PIN_IO1  QSPI_PIN_IO1
    #endif
    #if defined(QSPI_PIN_IO2)
        #undef NRFX_QSPI_PIN_IO2
        #define NRFX_QSPI_PIN_IO2  QSPI_PIN_IO2
    #endif
    #if defined(QSPI_PIN_IO3)
        #undef NRFX_QSPI_PIN_IO3
        #define NRFX_QSPI_PIN_IO3  QSPI_PIN_IO3
    #endif
    
#endif // defined(QSPI_ENABLED)

//------------------------------------------------------------------------------
// RNG

#if defined(RNG_ENABLED)
    
    #undef NRFX_RNG_ENABLED
    #define NRFX_RNG_ENABLED  RNG_ENABLED
    
    #if defined(RNG_CONFIG_ERROR_CORRECTION)
        #undef NRFX_RNG_CONFIG_ERROR_CORRECTION
        #define NRFX_RNG_CONFIG_ERROR_CORRECTION  RNG_CONFIG_ERROR_CORRECTION
    #endif
    
    #if defined(RNG_CONFIG_IRQ_PRIORITY)
        #undef NRFX_RNG_CONFIG_IRQ_PRIORITY
        #define NRFX_RNG_CONFIG_IRQ_PRIORITY  RNG_CONFIG_IRQ_PRIORITY
    #endif
    
    #if defined(RNG_CONFIG_LOG_ENABLED)
        #undef NRFX_RNG_CONFIG_LOG_ENABLED
        #define NRFX_RNG_CONFIG_LOG_ENABLED  RNG_CONFIG_LOG_ENABLED
    #endif
    #if defined(RNG_CONFIG_LOG_LEVEL)
        #undef NRFX_RNG_CONFIG_LOG_LEVEL
        #define NRFX_RNG_CONFIG_LOG_LEVEL  RNG_CONFIG_LOG_LEVEL
    #endif
    #if defined(RNG_CONFIG_INFO_COLOR)
        #undef NRFX_RNG_CONFIG_INFO_COLOR
        #define NRFX_RNG_CONFIG_INFO_COLOR  RNG_CONFIG_INFO_COLOR
    #endif
    #if defined(RNG_CONFIG_DEBUG_COLOR)
        #undef NRFX_RNG_CONFIG_DEBUG_COLOR
        #define NRFX_RNG_CONFIG_DEBUG_COLOR  RNG_CONFIG_DEBUG_COLOR
    #endif
    
#endif // defined(RNG_ENABLED)

//------------------------------------------------------------------------------
// RTC

#if defined(RTC_ENABLED)
    
    #undef NRFX_RTC_ENABLED
    #define NRFX_RTC_ENABLED  RTC_ENABLED
    
    #if defined(RTC0_ENABLED)
        #undef NRFX_RTC0_ENABLED
        #define NRFX_RTC0_ENABLED  RTC0_ENABLED
    #endif
    #if defined(RTC1_ENABLED)
        #undef NRFX_RTC1_ENABLED
        #define NRFX_RTC1_ENABLED  RTC1_ENABLED
    #endif
    #if defined(RTC2_ENABLED)
        #undef NRFX_RTC2_ENABLED
        #define NRFX_RTC2_ENABLED  RTC2_ENABLED
    #endif
    
    #if defined(RTC_DEFAULT_CONFIG_FREQUENCY)
        #undef NRFX_RTC_DEFAULT_CONFIG_FREQUENCY
        #define NRFX_RTC_DEFAULT_CONFIG_FREQUENCY  RTC_DEFAULT_CONFIG_FREQUENCY
    #endif
    #if defined(RTC_DEFAULT_CONFIG_RELIABLE)
        #undef NRFX_RTC_DEFAULT_CONFIG_RELIABLE
        #define NRFX_RTC_DEFAULT_CONFIG_RELIABLE  RTC_DEFAULT_CONFIG_RELIABLE
    #endif
    #if defined(RTC_DEFAULT_CONFIG_IRQ_PRIORITY)
        #undef NRFX_RTC_DEFAULT_CONFIG_IRQ_PRIORITY
        #define NRFX_RTC_DEFAULT_CONFIG_IRQ_PRIORITY  RTC_DEFAULT_CONFIG_IRQ_PRIORITY
    #endif
    
    #if defined(NRF_MAXIMUM_LATENCY_US)
        #undef NRFX_RTC_MAXIMUM_LATENCY_US
        #define NRFX_RTC_MAXIMUM_LATENCY_US  NRF_MAXIMUM_LATENCY_US
    #endif
    
    #if defined(RTC_CONFIG_LOG_ENABLED)
        #undef NRFX_RTC_CONFIG_LOG_ENABLED
        #define NRFX_RTC_CONFIG_LOG_ENABLED  RTC_CONFIG_LOG_ENABLED
    #endif
    #if defined(RTC_CONFIG_LOG_LEVEL)
        #undef NRFX_RTC_CONFIG_LOG_LEVEL
        #define NRFX_RTC_CONFIG_LOG_LEVEL  RTC_CONFIG_LOG_LEVEL
    #endif
    #if defined(RTC_CONFIG_INFO_COLOR)
        #undef NRFX_RTC_CONFIG_INFO_COLOR
        #define NRFX_RTC_CONFIG_INFO_COLOR  RTC_CONFIG_INFO_COLOR
    #endif
    #if defined(RTC_CONFIG_DEBUG_COLOR)
        #undef NRFX_RTC_CONFIG_DEBUG_COLOR
        #define NRFX_RTC_CONFIG_DEBUG_COLOR  RTC_CONFIG_DEBUG_COLOR
    #endif
    
#endif // defined(RTC_ENABLED)

//------------------------------------------------------------------------------
// SAADC

#if defined(SAADC_ENABLED)
    
    #undef NRFX_SAADC_ENABLED
    #define NRFX_SAADC_ENABLED  SAADC_ENABLED
    
    #if defined(SAADC_CONFIG_RESOLUTION)
        #undef NRFX_SAADC_CONFIG_RESOLUTION
        #define NRFX_SAADC_CONFIG_RESOLUTION  SAADC_CONFIG_RESOLUTION
    #endif
    #if defined(SAADC_CONFIG_OVERSAMPLE)
        #undef NRFX_SAADC_CONFIG_OVERSAMPLE
        #define NRFX_SAADC_CONFIG_OVERSAMPLE  SAADC_CONFIG_OVERSAMPLE
    #endif
    #if defined(SAADC_CONFIG_LP_MODE)
        #undef NRFX_SAADC_CONFIG_LP_MODE
        #define NRFX_SAADC_CONFIG_LP_MODE  SAADC_CONFIG_LP_MODE
    #endif
    #if defined(SAADC_CONFIG_IRQ_PRIORITY)
        #undef NRFX_SAADC_CONFIG_IRQ_PRIORITY
        #define NRFX_SAADC_CONFIG_IRQ_PRIORITY  SAADC_CONFIG_IRQ_PRIORITY
    #endif
    
    #if defined(SAADC_CONFIG_LOG_ENABLED)
        #undef NRFX_SAADC_CONFIG_LOG_ENABLED
        #define NRFX_SAADC_CONFIG_LOG_ENABLED  SAADC_CONFIG_LOG_ENABLED
    #endif
    #if defined(SAADC_CONFIG_LOG_LEVEL)
        #undef NRFX_SAADC_CONFIG_LOG_LEVEL
        #define NRFX_SAADC_CONFIG_LOG_LEVEL  SAADC_CONFIG_LOG_LEVEL
    #endif
    #if defined(SAADC_CONFIG_INFO_COLOR)
        #undef NRFX_SAADC_CONFIG_INFO_COLOR
        #define NRFX_SAADC_CONFIG_INFO_COLOR  SAADC_CONFIG_INFO_COLOR
    #endif
    #if defined(SAADC_CONFIG_DEBUG_COLOR)
        #undef NRFX_SAADC_CONFIG_DEBUG_COLOR
        #define NRFX_SAADC_CONFIG_DEBUG_COLOR  SAADC_CONFIG_DEBUG_COLOR
    #endif
    
#endif // defined(SAADC_ENABLED)

//------------------------------------------------------------------------------
// SPI

#if defined(SPI_ENABLED)

#undef NRFX_SPI_ENABLED
#define NRFX_SPI_ENABLED \
    (SPI_ENABLED && (NRFX_SPI0_ENABLED  || NRFX_SPI1_ENABLED  || NRFX_SPI2_ENABLED))
#undef NRFX_SPIM_ENABLED
#define NRFX_SPIM_ENABLED \
    (SPI_ENABLED && (NRFX_SPIM0_ENABLED || NRFX_SPIM1_ENABLED || NRFX_SPIM2_ENABLED))

#if defined(SPI_PRESENT) && !defined(SPIM_PRESENT)
    
    #undef NRFX_SPI0_ENABLED
    #define NRFX_SPI0_ENABLED   SPI0_ENABLED
    #undef NRFX_SPIM0_ENABLED
    #define NRFX_SPIM0_ENABLED  0
    
    #undef NRFX_SPI1_ENABLED
    #define NRFX_SPI1_ENABLED   SPI1_ENABLED
    #undef NRFX_SPIM1_ENABLED
    #define NRFX_SPIM1_ENABLED  0
    
    #undef NRFX_SPI2_ENABLED
    #define NRFX_SPI2_ENABLED   SPI2_ENABLED
    #undef NRFX_SPIM2_ENABLED
    #define NRFX_SPIM2_ENABLED  0
    
#elif !defined(SPI_PRESENT) && defined(SPIM_PRESENT)
    
    #undef NRFX_SPI0_ENABLED
    #define NRFX_SPI0_ENABLED   0
    #undef NRFX_SPIM0_ENABLED
    #define NRFX_SPIM0_ENABLED  SPI0_ENABLED
    
    #undef NRFX_SPI1_ENABLED
    #define NRFX_SPI1_ENABLED   0
    #undef NRFX_SPIM1_ENABLED
    #define NRFX_SPIM1_ENABLED  SPI1_ENABLED
    
    #undef NRFX_SPI2_ENABLED
    #define NRFX_SPI2_ENABLED   0
    #undef NRFX_SPIM2_ENABLED
    #define NRFX_SPIM2_ENABLED  SPI2_ENABLED
    
#else // -> defined(SPI_PRESENT) && defined(SPIM_PRESENT)
    
    #undef NRFX_SPI0_ENABLED
    #define NRFX_SPI0_ENABLED   (SPI0_ENABLED && !SPI0_USE_EASY_DMA)
    #undef NRFX_SPIM0_ENABLED
    #define NRFX_SPIM0_ENABLED  (SPI0_ENABLED && SPI0_USE_EASY_DMA)
    
    #undef NRFX_SPI1_ENABLED
    #define NRFX_SPI1_ENABLED   (SPI1_ENABLED && !SPI1_USE_EASY_DMA)
    #undef NRFX_SPIM1_ENABLED
    #define NRFX_SPIM1_ENABLED  (SPI1_ENABLED && SPI1_USE_EASY_DMA)
    
    #undef NRFX_SPI2_ENABLED
    #define NRFX_SPI2_ENABLED   (SPI2_ENABLED && !SPI2_USE_EASY_DMA)
    #undef NRFX_SPIM2_ENABLED
    #define NRFX_SPIM2_ENABLED  (SPI2_ENABLED && SPI2_USE_EASY_DMA)
    
#endif // -> defined(SPI_PRESENT) && defined(SPIM_PRESENT)

#if defined(NRF_SPI_DRV_MISO_PULLUP_CFG)
    #undef NRFX_SPI_MISO_PULL_CFG
    #define NRFX_SPI_MISO_PULL_CFG  NRF_SPI_DRV_MISO_PULLUP_CFG
    #undef NRFX_SPIM_MISO_PULL_CFG
    #define NRFX_SPIM_MISO_PULL_CFG  NRF_SPI_DRV_MISO_PULLUP_CFG
#endif

#if defined(SPI_DEFAULT_CONFIG_IRQ_PRIORITY)
    #undef NRFX_SPI_DEFAULT_CONFIG_IRQ_PRIORITY
    #define NRFX_SPI_DEFAULT_CONFIG_IRQ_PRIORITY  SPI_DEFAULT_CONFIG_IRQ_PRIORITY
    #undef NRFX_SPIM_DEFAULT_CONFIG_IRQ_PRIORITY
    #define NRFX_SPIM_DEFAULT_CONFIG_IRQ_PRIORITY  SPI_DEFAULT_CONFIG_IRQ_PRIORITY
#endif

#if defined(SPI_CONFIG_LOG_ENABLED)
    #undef NRFX_SPI_CONFIG_LOG_ENABLED
    #define NRFX_SPI_CONFIG_LOG_ENABLED  SPI_CONFIG_LOG_ENABLED
    #undef NRFX_SPIM_CONFIG_LOG_ENABLED
    #define NRFX_SPIM_CONFIG_LOG_ENABLED  SPI_CONFIG_LOG_ENABLED
#endif
#if defined(SPI_CONFIG_LOG_LEVEL)
    #undef NRFX_SPI_CONFIG_LOG_LEVEL
    #define NRFX_SPI_CONFIG_LOG_LEVEL  SPI_CONFIG_LOG_LEVEL
    #undef NRFX_SPIM_CONFIG_LOG_LEVEL
    #define NRFX_SPIM_CONFIG_LOG_LEVEL  SPI_CONFIG_LOG_LEVEL
#endif
#if defined(SPI_CONFIG_INFO_COLOR)
    #undef NRFX_SPI_CONFIG_INFO_COLOR
    #define NRFX_SPI_CONFIG_INFO_COLOR  SPI_CONFIG_INFO_COLOR
    #undef NRFX_SPIM_CONFIG_INFO_COLOR
    #define NRFX_SPIM_CONFIG_INFO_COLOR  SPI_CONFIG_INFO_COLOR
#endif
#if defined(SPI_CONFIG_DEBUG_COLOR)
    #undef NRFX_SPI_CONFIG_DEBUG_COLOR
    #define NRFX_SPI_CONFIG_DEBUG_COLOR  SPI_CONFIG_DEBUG_COLOR
    #undef NRFX_SPIM_CONFIG_DEBUG_COLOR
    #define NRFX_SPIM_CONFIG_DEBUG_COLOR  SPI_CONFIG_DEBUG_COLOR
#endif

#if defined(SPIM_NRF52_ANOMALY_109_WORKAROUND_ENABLED)
    #undef NRFX_SPIM_NRF52_ANOMALY_109_WORKAROUND_ENABLED
    #define NRFX_SPIM_NRF52_ANOMALY_109_WORKAROUND_ENABLED  SPIM_NRF52_ANOMALY_109_WORKAROUND_ENABLED
#endif

#endif // defined(SPI_ENABLED)

//------------------------------------------------------------------------------
// SPIS

#if defined(SPIS_ENABLED)
    
    #undef NRFX_SPIS_ENABLED
    #define NRFX_SPIS_ENABLED  SPIS_ENABLED
    
    #if defined(SPIS0_ENABLED)
        #undef NRFX_SPIS0_ENABLED
        #define NRFX_SPIS0_ENABLED  SPIS0_ENABLED
    #endif
    #if defined(SPIS1_ENABLED)
        #undef NRFX_SPIS1_ENABLED
        #define NRFX_SPIS1_ENABLED  SPIS1_ENABLED
    #endif
    #if defined(SPIS2_ENABLED)
        #undef NRFX_SPIS2_ENABLED
        #define NRFX_SPIS2_ENABLED  SPIS2_ENABLED
    #endif
    
    #if defined(SPIS_DEFAULT_CONFIG_IRQ_PRIORITY)
        #undef NRFX_SPIS_DEFAULT_CONFIG_IRQ_PRIORITY
        #define NRFX_SPIS_DEFAULT_CONFIG_IRQ_PRIORITY  SPIS_DEFAULT_CONFIG_IRQ_PRIORITY
    #endif
    #if defined(SPIS_DEFAULT_MODE)
        #undef NRFX_SPIS_DEFAULT_MODE
        #define NRFX_SPIS_DEFAULT_MODE  SPIS_DEFAULT_MODE
    #endif
    #if defined(SPIS_DEFAULT_BIT_ORDER)
        #undef NRFX_SPIS_DEFAULT_BIT_ORDER
        #define NRFX_SPIS_DEFAULT_BIT_ORDER  SPIS_DEFAULT_BIT_ORDER
    #endif
    #if defined(SPIS_DEFAULT_DEF)
        #undef NRFX_SPIS_DEFAULT_DEF
        #define NRFX_SPIS_DEFAULT_DEF  SPIS_DEFAULT_DEF
    #endif
    #if defined(SPIS_DEFAULT_ORC)
        #undef NRFX_SPIS_DEFAULT_ORC
        #define NRFX_SPIS_DEFAULT_ORC  SPIS_DEFAULT_ORC
    #endif
    
    #if defined(SPIS_CONFIG_LOG_ENABLED)
        #undef NRFX_SPIS_CONFIG_LOG_ENABLED
        #define NRFX_SPIS_CONFIG_LOG_ENABLED  SPIS_CONFIG_LOG_ENABLED
    #endif
    #if defined(SPIS_CONFIG_LOG_LEVEL)
        #undef NRFX_SPIS_CONFIG_LOG_LEVEL
        #define NRFX_SPIS_CONFIG_LOG_LEVEL  SPIS_CONFIG_LOG_LEVEL
    #endif
    #if defined(SPIS_CONFIG_INFO_COLOR)
        #undef NRFX_SPIS_CONFIG_INFO_COLOR
        #define NRFX_SPIS_CONFIG_INFO_COLOR  SPIS_CONFIG_INFO_COLOR
    #endif
    #if defined(SPIS_CONFIG_DEBUG_COLOR)
        #undef NRFX_SPIS_CONFIG_DEBUG_COLOR
        #define NRFX_SPIS_CONFIG_DEBUG_COLOR  SPIS_CONFIG_DEBUG_COLOR
    #endif
    
    #if defined(SPIS_NRF52_ANOMALY_109_WORKAROUND_ENABLED)
        #undef NRFX_SPIS_NRF52_ANOMALY_109_WORKAROUND_ENABLED
        #define NRFX_SPIS_NRF52_ANOMALY_109_WORKAROUND_ENABLED  SPIS_NRF52_ANOMALY_109_WORKAROUND_ENABLED
    #endif
    
#endif // defined(SPIS_ENABLED)

//------------------------------------------------------------------------------
// SWI

#if defined(SWI_DISABLE0)
    #undef NRFX_SWI0_DISABLED
    #define NRFX_SWI0_DISABLED  1
#endif
#if defined(SWI_DISABLE1)
    #undef NRFX_SWI1_DISABLED
    #define NRFX_SWI1_DISABLED  1
#endif
#if defined(SWI_DISABLE2)
    #undef NRFX_SWI2_DISABLED
    #define NRFX_SWI2_DISABLED  1
#endif
#if defined(SWI_DISABLE3)
    #undef NRFX_SWI3_DISABLED
    #define NRFX_SWI3_DISABLED  1
#endif
#if defined(SWI_DISABLE4)
    #undef NRFX_SWI4_DISABLED
    #define NRFX_SWI4_DISABLED  1
#endif
#if defined(SWI_DISABLE5)
    #undef NRFX_SWI5_DISABLED
    #define NRFX_SWI5_DISABLED  1
#endif

#if defined(EGU_ENABLED)
    #undef NRFX_EGU_ENABLED
    #define NRFX_EGU_ENABLED  EGU_ENABLED
#endif

#if defined(SWI_CONFIG_LOG_ENABLED)
    #undef NRFX_SWI_CONFIG_LOG_ENABLED
    #define NRFX_SWI_CONFIG_LOG_ENABLED  SWI_CONFIG_LOG_ENABLED
#endif
#if defined(SWI_CONFIG_LOG_LEVEL)
    #undef NRFX_SWI_CONFIG_LOG_LEVEL
    #define NRFX_SWI_CONFIG_LOG_LEVEL  SWI_CONFIG_LOG_LEVEL
#endif
#if defined(SWI_CONFIG_INFO_COLOR)
    #undef NRFX_SWI_CONFIG_INFO_COLOR
    #define NRFX_SWI_CONFIG_INFO_COLOR  SWI_CONFIG_INFO_COLOR
#endif
#if defined(SWI_CONFIG_DEBUG_COLOR)
    #undef NRFX_SWI_CONFIG_DEBUG_COLOR
    #define NRFX_SWI_CONFIG_DEBUG_COLOR  SWI_CONFIG_DEBUG_COLOR
#endif

//------------------------------------------------------------------------------
// SysTick

#if defined(SYSTICK_ENABLED)
    
    #undef NRFX_SYSTICK_ENABLED
    #define NRFX_SYSTICK_ENABLED  SYSTICK_ENABLED
    
#endif // defined(SYSTICK_ENABLED)

//------------------------------------------------------------------------------
// TIMER

#if defined(TIMER_ENABLED)
    
    #undef NRFX_TIMER_ENABLED
    #define NRFX_TIMER_ENABLED  TIMER_ENABLED
    
    #if defined(TIMER0_ENABLED)
        #undef NRFX_TIMER0_ENABLED
        #define NRFX_TIMER0_ENABLED  TIMER0_ENABLED
    #endif
    #if defined(TIMER1_ENABLED)
        #undef NRFX_TIMER1_ENABLED
        #define NRFX_TIMER1_ENABLED  TIMER1_ENABLED
    #endif
    #if defined(TIMER2_ENABLED)
        #undef NRFX_TIMER2_ENABLED
        #define NRFX_TIMER2_ENABLED  TIMER2_ENABLED
    #endif
    #if defined(TIMER3_ENABLED)
        #undef NRFX_TIMER3_ENABLED
        #define NRFX_TIMER3_ENABLED  TIMER3_ENABLED
    #endif
    #if defined(TIMER4_ENABLED)
        #undef NRFX_TIMER4_ENABLED
        #define NRFX_TIMER4_ENABLED  TIMER4_ENABLED
    #endif
    
    #if defined(TIMER_DEFAULT_CONFIG_FREQUENCY)
        #undef NRFX_TIMER_DEFAULT_CONFIG_FREQUENCY
        #define NRFX_TIMER_DEFAULT_CONFIG_FREQUENCY  TIMER_DEFAULT_CONFIG_FREQUENCY
    #endif
    #if defined(TIMER_DEFAULT_CONFIG_MODE)
        #undef NRFX_TIMER_DEFAULT_CONFIG_MODE
        #define NRFX_TIMER_DEFAULT_CONFIG_MODE  TIMER_DEFAULT_CONFIG_MODE
    #endif
    #if defined(TIMER_DEFAULT_CONFIG_BIT_WIDTH)
        #undef NRFX_TIMER_DEFAULT_CONFIG_BIT_WIDTH
        #define NRFX_TIMER_DEFAULT_CONFIG_BIT_WIDTH  TIMER_DEFAULT_CONFIG_BIT_WIDTH
    #endif
    #if defined(TIMER_DEFAULT_CONFIG_IRQ_PRIORITY)
        #undef NRFX_TIMER_DEFAULT_CONFIG_IRQ_PRIORITY
        #define NRFX_TIMER_DEFAULT_CONFIG_IRQ_PRIORITY  TIMER_DEFAULT_CONFIG_IRQ_PRIORITY
    #endif
    
    #if defined(TIMER_CONFIG_LOG_ENABLED)
        #undef NRFX_TIMER_CONFIG_LOG_ENABLED
        #define NRFX_TIMER_CONFIG_LOG_ENABLED  TIMER_CONFIG_LOG_ENABLED
    #endif
    #if defined(TIMER_CONFIG_LOG_LEVEL)
        #undef NRFX_TIMER_CONFIG_LOG_LEVEL
        #define NRFX_TIMER_CONFIG_LOG_LEVEL  TIMER_CONFIG_LOG_LEVEL
    #endif
    #if defined(TIMER_CONFIG_INFO_COLOR)
        #undef NRFX_TIMER_CONFIG_INFO_COLOR
        #define NRFX_TIMER_CONFIG_INFO_COLOR  TIMER_CONFIG_INFO_COLOR
    #endif
    #if defined(TIMER_CONFIG_DEBUG_COLOR)
        #undef NRFX_TIMER_CONFIG_DEBUG_COLOR
        #define NRFX_TIMER_CONFIG_DEBUG_COLOR  TIMER_CONFIG_DEBUG_COLOR
    #endif
    
#endif // defined(TIMER_ENABLED)

//------------------------------------------------------------------------------
// TWI
#define TWI_ONLY      ( defined(TWI_PRESENT) && !defined(TWIM_PRESENT))
#define TWIM_ONLY     (!defined(TWI_PRESENT) &&  defined(TWIM_PRESENT))
#define TWI_AND_TWIM  ( defined(TWI_PRESENT) &&  defined(TWIM_PRESENT))

#if defined(TWI_ENABLED)
    
    #undef NRFX_TWI_ENABLED
    #define NRFX_TWI_ENABLED   (TWI_ENABLED && (NRFX_TWI0_ENABLED  || NRFX_TWI1_ENABLED))
    #undef NRFX_TWIM_ENABLED
    #define NRFX_TWIM_ENABLED  (TWI_ENABLED && (NRFX_TWIM0_ENABLED || NRFX_TWIM1_ENABLED))
    
    #if defined(TWI_PRESENT) && !defined(TWIM_PRESENT)
        
        #undef NRFX_TWI0_ENABLED
        #define NRFX_TWI0_ENABLED   TWI0_ENABLED
        #undef NRFX_TWIM0_ENABLED
        #define NRFX_TWIM0_ENABLED  0
        
        #undef NRFX_TWI1_ENABLED
        #define NRFX_TWI1_ENABLED   TWI1_ENABLED
        #undef NRFX_TWIM1_ENABLED
        #define NRFX_TWIM1_ENABLED  0
        
    #elif !defined(TWI_PRESENT) && defined(TWIM_PRESENT)
        
        #undef NRFX_TWI0_ENABLED
        #define NRFX_TWI0_ENABLED   0
        #undef NRFX_TWIM0_ENABLED
        #define NRFX_TWIM0_ENABLED  TWI0_ENABLED
        
        #undef NRFX_TWI1_ENABLED
        #define NRFX_TWI1_ENABLED   0
        #undef NRFX_TWIM1_ENABLED
        #define NRFX_TWIM1_ENABLED  TWI1_ENABLED
        
    #else // -> defined(TWI_PRESENT) && defined(TWIM_PRESENT)
        
        #undef NRFX_TWI0_ENABLED
        #define NRFX_TWI0_ENABLED   (TWI0_ENABLED && !TWI0_USE_EASY_DMA)
        #undef NRFX_TWIM0_ENABLED
        #define NRFX_TWIM0_ENABLED  (TWI0_ENABLED && TWI0_USE_EASY_DMA)
        
        #undef NRFX_TWI1_ENABLED
        #define NRFX_TWI1_ENABLED   (TWI1_ENABLED && !TWI1_USE_EASY_DMA)
        #undef NRFX_TWIM1_ENABLED
        #define NRFX_TWIM1_ENABLED  (TWI1_ENABLED && TWI1_USE_EASY_DMA)
        
    #endif // -> defined(TWI_PRESENT) && defined(TWIM_PRESENT)
    
    #if defined(TWI_DEFAULT_CONFIG_FREQUENCY)
        #undef NRFX_TWI_DEFAULT_CONFIG_FREQUENCY
        #define NRFX_TWI_DEFAULT_CONFIG_FREQUENCY  TWI_DEFAULT_CONFIG_FREQUENCY
        #undef NRFX_TWIM_DEFAULT_CONFIG_FREQUENCY
        #define NRFX_TWIM_DEFAULT_CONFIG_FREQUENCY  TWI_DEFAULT_CONFIG_FREQUENCY
    #endif
    #if defined(TWI_DEFAULT_CONFIG_HOLD_BUS_UNINIT)
        #undef NRFX_TWI_DEFAULT_CONFIG_HOLD_BUS_UNINIT
        #define NRFX_TWI_DEFAULT_CONFIG_HOLD_BUS_UNINIT  TWI_DEFAULT_CONFIG_HOLD_BUS_UNINIT
        #undef NRFX_TWIM_DEFAULT_CONFIG_HOLD_BUS_UNINIT
        #define NRFX_TWIM_DEFAULT_CONFIG_HOLD_BUS_UNINIT  TWI_DEFAULT_CONFIG_HOLD_BUS_UNINIT
    #endif
    #if defined(TWI_DEFAULT_CONFIG_IRQ_PRIORITY)
        #undef NRFX_TWI_DEFAULT_CONFIG_IRQ_PRIORITY
        #define NRFX_TWI_DEFAULT_CONFIG_IRQ_PRIORITY  TWI_DEFAULT_CONFIG_IRQ_PRIORITY
        #undef NRFX_TWIM_DEFAULT_CONFIG_IRQ_PRIORITY
        #define NRFX_TWIM_DEFAULT_CONFIG_IRQ_PRIORITY  TWI_DEFAULT_CONFIG_IRQ_PRIORITY
    #endif
    
    #if defined(TWI_CONFIG_LOG_ENABLED)
        #undef NRFX_TWI_CONFIG_LOG_ENABLED
        #define NRFX_TWI_CONFIG_LOG_ENABLED  TWI_CONFIG_LOG_ENABLED
        #undef NRFX_TWIM_CONFIG_LOG_ENABLED
        #define NRFX_TWIM_CONFIG_LOG_ENABLED  TWI_CONFIG_LOG_ENABLED
    #endif
    #if defined(TWI_CONFIG_LOG_LEVEL)
        #undef NRFX_TWI_CONFIG_LOG_LEVEL
        #define NRFX_TWI_CONFIG_LOG_LEVEL  TWI_CONFIG_LOG_LEVEL
        #undef NRFX_TWIM_CONFIG_LOG_LEVEL
        #define NRFX_TWIM_CONFIG_LOG_LEVEL  TWI_CONFIG_LOG_LEVEL
    #endif
    #if defined(TWI_CONFIG_INFO_COLOR)
        #undef NRFX_TWI_CONFIG_INFO_COLOR
        #define NRFX_TWI_CONFIG_INFO_COLOR  TWI_CONFIG_INFO_COLOR
        #undef NRFX_TWIM_CONFIG_INFO_COLOR
        #define NRFX_TWIM_CONFIG_INFO_COLOR  TWI_CONFIG_INFO_COLOR
    #endif
    #if defined(TWI_CONFIG_DEBUG_COLOR)
        #undef NRFX_TWI_CONFIG_DEBUG_COLOR
        #define NRFX_TWI_CONFIG_DEBUG_COLOR  TWI_CONFIG_DEBUG_COLOR
        #undef NRFX_TWIM_CONFIG_DEBUG_COLOR
        #define NRFX_TWIM_CONFIG_DEBUG_COLOR  TWI_CONFIG_DEBUG_COLOR
    #endif
    
    #if defined(TWIM_NRF52_ANOMALY_109_WORKAROUND_ENABLED)
        #undef NRFX_TWIM_NRF52_ANOMALY_109_WORKAROUND_ENABLED
        #define NRFX_TWIM_NRF52_ANOMALY_109_WORKAROUND_ENABLED  TWIM_NRF52_ANOMALY_109_WORKAROUND_ENABLED
    #endif
    
#endif // defined(TWI_ENABLED)

//------------------------------------------------------------------------------
// TWIS

#if defined(TWIS_ENABLED)
    
    #undef NRFX_TWIS_ENABLED
    #define NRFX_TWIS_ENABLED  TWIS_ENABLED
    
    #if defined(TWIS0_ENABLED)
        #undef NRFX_TWIS0_ENABLED
        #define NRFX_TWIS0_ENABLED  TWIS0_ENABLED
    #endif
    #if defined(TWIS1_ENABLED)
        #undef NRFX_TWIS1_ENABLED
        #define NRFX_TWIS1_ENABLED  TWIS1_ENABLED
    #endif
    
    #if defined(TWIS_ASSUME_INIT_AFTER_RESET_ONLY)
        #undef NRFX_TWIS_ASSUME_INIT_AFTER_RESET_ONLY
        #define NRFX_TWIS_ASSUME_INIT_AFTER_RESET_ONLY  TWIS_ASSUME_INIT_AFTER_RESET_ONLY
    #endif
    #if defined(TWIS_NO_SYNC_MODE)
        #undef NRFX_TWIS_NO_SYNC_MODE
        #define NRFX_TWIS_NO_SYNC_MODE  TWIS_NO_SYNC_MODE
    #endif
    
    #if defined(TWIS_DEFAULT_CONFIG_ADDR0)
        #undef NRFX_TWIS_DEFAULT_CONFIG_ADDR0
        #define NRFX_TWIS_DEFAULT_CONFIG_ADDR0  TWIS_DEFAULT_CONFIG_ADDR0
    #endif
    #if defined(TWIS_DEFAULT_CONFIG_ADDR1)
        #undef NRFX_TWIS_DEFAULT_CONFIG_ADDR1
        #define NRFX_TWIS_DEFAULT_CONFIG_ADDR1  TWIS_DEFAULT_CONFIG_ADDR1
    #endif
    #if defined(TWIS_DEFAULT_CONFIG_SCL_PULL)
        #undef NRFX_TWIS_DEFAULT_CONFIG_SCL_PULL
        #define NRFX_TWIS_DEFAULT_CONFIG_SCL_PULL  TWIS_DEFAULT_CONFIG_SCL_PULL
    #endif
    #if defined(TWIS_DEFAULT_CONFIG_SDA_PULL)
        #undef NRFX_TWIS_DEFAULT_CONFIG_SDA_PULL
        #define NRFX_TWIS_DEFAULT_CONFIG_SDA_PULL  TWIS_DEFAULT_CONFIG_SDA_PULL
    #endif
    #if defined(TWIS_DEFAULT_CONFIG_IRQ_PRIORITY)
        #undef NRFX_TWIS_DEFAULT_CONFIG_IRQ_PRIORITY
        #define NRFX_TWIS_DEFAULT_CONFIG_IRQ_PRIORITY  TWIS_DEFAULT_CONFIG_IRQ_PRIORITY
    #endif
    
    #if defined(TWIS_CONFIG_LOG_ENABLED)
        #undef NRFX_TWIS_CONFIG_LOG_ENABLED
        #define NRFX_TWIS_CONFIG_LOG_ENABLED  TWIS_CONFIG_LOG_ENABLED
    #endif
    #if defined(TWIS_CONFIG_LOG_LEVEL)
        #undef NRFX_TWIS_CONFIG_LOG_LEVEL
        #define NRFX_TWIS_CONFIG_LOG_LEVEL  TWIS_CONFIG_LOG_LEVEL
    #endif
    #if defined(TWIS_CONFIG_INFO_COLOR)
        #undef NRFX_TWIS_CONFIG_INFO_COLOR
        #define NRFX_TWIS_CONFIG_INFO_COLOR  TWIS_CONFIG_INFO_COLOR
    #endif
    #if defined(TWIS_CONFIG_DEBUG_COLOR)
        #undef NRFX_TWIS_CONFIG_DEBUG_COLOR
        #define NRFX_TWIS_CONFIG_DEBUG_COLOR  TWIS_CONFIG_DEBUG_COLOR
    #endif
    
#endif // defined(TWIS_ENABLED)

//------------------------------------------------------------------------------
// UART

#if defined(UART_ENABLED)
    
    #undef NRFX_UART_ENABLED
    #define NRFX_UART_ENABLED   (UART_ENABLED && NRFX_UART0_ENABLED)
    #undef NRFX_UARTE_ENABLED
    #define NRFX_UARTE_ENABLED  (UART_ENABLED && (NRFX_UARTE0_ENABLED || NRFX_UARTE1_ENABLED))
    
    #if defined(UART0_ENABLED)
        #undef NRFX_UART0_ENABLED
        #define NRFX_UART0_ENABLED   (UART0_ENABLED && UART_LEGACY_SUPPORT)
        #undef NRFX_UARTE0_ENABLED
        #define NRFX_UARTE0_ENABLED  (UART0_ENABLED && UART_EASY_DMA_SUPPORT)
    #endif
    #if defined(UART1_ENABLED)
        #undef NRFX_UARTE1_ENABLED
        #define NRFX_UARTE1_ENABLED  (UART1_ENABLED && UART_EASY_DMA_SUPPORT)
    #endif
    
    #if defined(UART_DEFAULT_CONFIG_HWFC)
        #undef NRFX_UART_DEFAULT_CONFIG_HWFC
        #define NRFX_UART_DEFAULT_CONFIG_HWFC  UART_DEFAULT_CONFIG_HWFC
        #undef NRFX_UARTE_DEFAULT_CONFIG_HWFC
        #define NRFX_UARTE_DEFAULT_CONFIG_HWFC  UART_DEFAULT_CONFIG_HWFC
    #endif
    #if defined(UART_DEFAULT_CONFIG_PARITY)
        #undef NRFX_UART_DEFAULT_CONFIG_PARITY
        #define NRFX_UART_DEFAULT_CONFIG_PARITY  UART_DEFAULT_CONFIG_PARITY
        #undef NRFX_UARTE_DEFAULT_CONFIG_PARITY
        #define NRFX_UARTE_DEFAULT_CONFIG_PARITY  UART_DEFAULT_CONFIG_PARITY
    #endif
    #if defined(UART_DEFAULT_CONFIG_BAUDRATE)
        #undef NRFX_UART_DEFAULT_CONFIG_BAUDRATE
        #define NRFX_UART_DEFAULT_CONFIG_BAUDRATE  UART_DEFAULT_CONFIG_BAUDRATE
        #undef NRFX_UARTE_DEFAULT_CONFIG_BAUDRATE
        #define NRFX_UARTE_DEFAULT_CONFIG_BAUDRATE  UART_DEFAULT_CONFIG_BAUDRATE
    #endif
    #if defined(UART_DEFAULT_CONFIG_IRQ_PRIORITY)
        #undef NRFX_UART_DEFAULT_CONFIG_IRQ_PRIORITY
        #define NRFX_UART_DEFAULT_CONFIG_IRQ_PRIORITY  UART_DEFAULT_CONFIG_IRQ_PRIORITY
        #undef NRFX_UARTE_DEFAULT_CONFIG_IRQ_PRIORITY
        #define NRFX_UARTE_DEFAULT_CONFIG_IRQ_PRIORITY  UART_DEFAULT_CONFIG_IRQ_PRIORITY
    #endif
    
    #if defined(UART_CONFIG_LOG_ENABLED)
        #undef NRFX_UART_CONFIG_LOG_ENABLED
        #define NRFX_UART_CONFIG_LOG_ENABLED  UART_CONFIG_LOG_ENABLED
        #undef NRFX_UARTE_CONFIG_LOG_ENABLED
        #define NRFX_UARTE_CONFIG_LOG_ENABLED  UART_CONFIG_LOG_ENABLED
    #endif
    #if defined(UART_CONFIG_LOG_LEVEL)
        #undef NRFX_UART_CONFIG_LOG_LEVEL
        #define NRFX_UART_CONFIG_LOG_LEVEL  UART_CONFIG_LOG_LEVEL
        #undef NRFX_UARTE_CONFIG_LOG_LEVEL
        #define NRFX_UARTE_CONFIG_LOG_LEVEL  UART_CONFIG_LOG_LEVEL
    #endif
    #if defined(UART_CONFIG_INFO_COLOR)
        #undef NRFX_UART_CONFIG_INFO_COLOR
        #define NRFX_UART_CONFIG_INFO_COLOR  UART_CONFIG_INFO_COLOR
        #undef NRFX_UARTE_CONFIG_INFO_COLOR
        #define NRFX_UARTE_CONFIG_INFO_COLOR  UART_CONFIG_INFO_COLOR
    #endif
    #if defined(UART_CONFIG_DEBUG_COLOR)
        #undef NRFX_UART_CONFIG_DEBUG_COLOR
        #define NRFX_UART_CONFIG_DEBUG_COLOR  UART_CONFIG_DEBUG_COLOR
        #undef NRFX_UARTE_CONFIG_DEBUG_COLOR
        #define NRFX_UARTE_CONFIG_DEBUG_COLOR  UART_CONFIG_DEBUG_COLOR
    #endif
    
#endif // defined(UART_ENABLED)

//------------------------------------------------------------------------------
// WDT

#if defined(WDT_ENABLED)
    
    #undef NRFX_WDT_ENABLED
    #define NRFX_WDT_ENABLED  WDT_ENABLED
    
    #if defined(WDT_CONFIG_BEHAVIOUR)
        #undef NRFX_WDT_CONFIG_BEHAVIOUR
        #define NRFX_WDT_CONFIG_BEHAVIOUR  WDT_CONFIG_BEHAVIOUR
    #endif
    #if defined(WDT_CONFIG_RELOAD_VALUE)
        #undef NRFX_WDT_CONFIG_RELOAD_VALUE
        #define NRFX_WDT_CONFIG_RELOAD_VALUE  WDT_CONFIG_RELOAD_VALUE
    #endif
    #if defined(WDT_CONFIG_IRQ_PRIORITY)
        #undef NRFX_WDT_CONFIG_IRQ_PRIORITY
        #define NRFX_WDT_CONFIG_IRQ_PRIORITY  WDT_CONFIG_IRQ_PRIORITY
    #endif
    
    #if defined(WDT_CONFIG_LOG_ENABLED)
        #undef NRFX_WDT_CONFIG_LOG_ENABLED
        #define NRFX_WDT_CONFIG_LOG_ENABLED  WDT_CONFIG_LOG_ENABLED
    #endif
    #if defined(WDT_CONFIG_LOG_LEVEL)
        #undef NRFX_WDT_CONFIG_LOG_LEVEL
        #define NRFX_WDT_CONFIG_LOG_LEVEL  WDT_CONFIG_LOG_LEVEL
    #endif
    #if defined(WDT_CONFIG_INFO_COLOR)
        #undef NRFX_WDT_CONFIG_INFO_COLOR
        #define NRFX_WDT_CONFIG_INFO_COLOR  WDT_CONFIG_INFO_COLOR
    #endif
    #if defined(WDT_CONFIG_DEBUG_COLOR)
        #undef NRFX_WDT_CONFIG_DEBUG_COLOR
        #define NRFX_WDT_CONFIG_DEBUG_COLOR  WDT_CONFIG_DEBUG_COLOR
    #endif
    
#endif // defined(WDT_ENABLED)

#endif // APPLY_OLD_CONFIG_H__
