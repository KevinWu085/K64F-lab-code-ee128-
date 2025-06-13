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
unsigned int nr_overflows = 0;
uint32_t count = 0;
uint32_t rflag = 0;
const unsigned char decoder[] =  {0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x67 };

void FTM3_IRQHandler(void) {
    nr_overflows++;
    uint32_t SC_VAL = FTM3_SC;
    FTM3_SC &= 0x7F; // clear TOF
}

void main(void) {
    SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK; // Port C clock enable
    SIM_SCGC3 |= SIM_SCGC3_FTM3_MASK; // FTM3 clock enable
    PORTC_PCR10 = 0x300; // Port C Pin 10 as FTM3_CH6 (ALT3)
    FTM3_MODE = 0x5; // Enable FTM3
    FTM3_MOD = 0xFFFF;
    FTM3_SC = 0x0D; // System clock / 32
    NVIC_EnableIRQ(FTM3_IRQn); // Enable FTM3 interrupts
    FTM3_SC |= 0x40; // Enable TOF

    //initialize port c and d
    SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;
    SIM_SCGC5 |= SIM_SCGC5_PORTD_MASK;
    //config port c and d
    PORTC_GPCLR = 0x01BF0100;  //port c
    PORTD_GPCLR = 0x00FF0100;  //port d
    //Set PD[7:0] and PC[8:7,5:0] for output;
    GPIOC_PDDR |= 0x000001BF;  // port c output
    GPIOD_PDDR |= 0x000000FF; // port d output

    unsigned int t1, t2, t3, pulse_width, period, dutyCycle;
    while (1) {
        FTM3_CNT = 0; nr_overflows = 0; // initialize counters
        FTM3_C6SC = 0x4; // rising edge
        while(!(FTM3_C6SC & 0x80)); // wait for CHF
        FTM3_C6SC &= ~(1 << 7);
        t1 = FTM3_C6V; // first edge
        FTM3_C6SC = 0x8; // falling edge
        while(!(FTM3_C6SC & 0x80)); // wait for CHF
        FTM3_C6SC &= ~(1 << 7);
        t2 = FTM3_C6V; // second edge
        if (t2 >= t1)
            pulse_width = (nr_overflows << 16) + (t2 - t1);
        else
            pulse_width = ((nr_overflows-1) << 16) + (t2 - t1);

        FTM3_C6SC = 0x4; // rising edge
        while(!(FTM3_C6SC & 0x80)); // wait for CHF
        FTM3_C6SC &= ~(1 << 7);
        t3 = FTM3_C6V; // first edge
        if (t3 >= t1)
            period = (nr_overflows << 16) + (t3 - t1);
        else
            period = ((nr_overflows-1) << 16) + (t3 - t1);

        //find duty cycles
        dutyCycle = (pulse_width * 100) / period;
        GPIOC_PCOR = 0xBF;
        GPIOD_PCOR = 0xFF;

        GPIOC_PSOR = (decoder[dutyCycle / 10] & 0x3F) | ((decoder[dutyCycle / 10] & 0x40) << 1);
        GPIOD_PSOR = decoder[dutyCycle % 10];
    }
}
