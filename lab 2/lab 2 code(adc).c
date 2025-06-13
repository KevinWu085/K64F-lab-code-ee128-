
/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "fsl_device_registers.h"
static int i = 0;
const unsigned char decoder[] =  {0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x67 };
uint32_t count = 0;
uint32_t outval;
uint32_t inputRegister = 0;

unsigned short adc_read16b(){
    ADC0_SC1A = 0x00;
    while(ADC0_SC2 & ADC_SC2_ADACT_MASK);
    while(!(ADC0_SC1A & ADC_SC1_COCO_MASK));
    return ADC0_RA;
}

void PORTA_IRQHandler(void)
{
    inputRegister = GPIOB_PDIR;
    if((inputRegister & 0x8) == 0){
        outval = (adc_read16b() * 33) / 0xFFFF;
    }
    else {
        if((inputRegister & 0x04)== 1){
            count--;
            if (count < 0){
                count = 99;
            }
        }
        outval = count;
    }

    GPIOC_PCOR = 0xBF;
    GPIOD_PCOR = 0xFF;

    GPIOC_PSOR = (decoder[outval / 10] & 0x3F) | ((decoder[outval / 10] & 0x40) << 1);
    GPIOD_PSOR = decoder[outval % 10];
    PORTA_ISFR = (1 << 1);
}

void delay(){
    for(i = 0; i < 500000; i++);
}

int main(void)
{
    SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;
    SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
    SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;
    SIM_SCGC5 |= SIM_SCGC5_PORTD_MASK;
    SIM_SCGC6 |= SIM_SCGC6_ADC0_MASK;

    PORTA_PCR4 = 0x0100;
    PORTB_GPCLR = 0x0000C0100;
    PORTC_GPCLR = 0x01BF0100;
    PORTD_GPCLR = 0x00FF0100;

    PORTA_PCR1 = 0xA0100;
    ADC0_CFG1 = 0x0C;
    ADC0_SC1A = 0x1F;

    GPIOA_PDDR |= (0 << 1);
    GPIOB_PDDR |= ((0 << 3) | (0 << 2) | (1 << 2) | (1 << 10));
    GPIOC_PDDR |= 0x000001BF;
    GPIOD_PDDR |= 0x000000FF;

    PORTA_ISFR = (1 << 1);
    NVIC_EnableIRQ(PORTA_IRQn);

    for(;;){
        GPIOB_PTOR |= (1 << 10);
        PORTA_IRQHandler();
        delay();
    }
    return 0;
}
