/**
    Copyright (c) 2016 - 2018 Nordic Semiconductor ASA and Luxoft Global Operations Gmbh.

    All Rights Reserved.

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
#ifndef RAL_IRQ_HANDLERS_H_INCLUDED
#define RAL_IRQ_HANDLERS_H_INCLUDED

#define RAL_NRF_BCC_COMPARE_VALUE     32
#define RAL_NRF_BCC_COMPARE_NONE     (RAL_NRF_BCC_COMPARE_VALUE + 16)
#define RAL_NRF_BCC_COMPARE_SHORT    (RAL_NRF_BCC_COMPARE_VALUE + 32)
#define RAL_NRF_BCC_COMPARE_LONG     (RAL_NRF_BCC_COMPARE_VALUE + 80)

/**
    @defgroup ral_api_irq_handlers RAL auxiliary functions
    @ingroup ral_api
    @{
*/

/** @brief   RAL IRQ handler symbol importer (dummy function).

    @details This function is only used to correctly import
            RADIO_IRQHandler symbol.
*/
void ral_irq_handler_import(void);

/** @} */

#endif// RAL_IRQ_HANDLERS_H_INCLUDED