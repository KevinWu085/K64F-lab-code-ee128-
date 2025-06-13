/*

Copyright (c) 2015, Freescale Semiconductor, Inc.
All rights reserved.
*
Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:
*
o Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.
*
o Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.
*
o Neither the name of Freescale Semiconductor, Inc. nor the names of its
contributors may be used to endorse or promote products derived from this
software without specific prior written permission.
*
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "fsl_device_registers.h"
static int i = 0;
uint32_t light = 0;
uint32_t mode = 0;
void speed(int mode){
	if ((mode & 0x8) != 0 && (mode & 0x04) != 0){ // 1,1
		GPIOD_PDOR = 0x36;
		for(i = 0; i < 10000; i++);
		GPIOD_PDOR = 0x35;
		for(i = 0; i < 10000; i++);
		GPIOD_PDOR = 0x39;
		for(i = 0; i < 10000; i++);
		GPIOD_PDOR = 0x3A;
		for(i = 0; i < 10000; i++);
	}
	else if ((mode & 0x8) != 0 && (mode & 0x04) == 0){ // 1, 0
		GPIOD_PDOR = 0x36;
		for(i = 0; i < 20000; i++);
		GPIOD_PDOR = 0x35;
		for(i = 0; i < 20000; i++);
		GPIOD_PDOR = 0x39;
		for(i = 0; i < 20000; i++);
		GPIOD_PDOR = 0x3A;
		for(i = 0; i < 20000; i++);
	}
	else if ((mode & 0x8) == 0 && (mode & 0x04) != 0){ // 0, 1
		GPIOD_PDOR = 0x36;
		for(i = 0; i < 30000; i++);
		GPIOD_PDOR = 0x35;
		for(i = 0; i < 30000; i++);
		GPIOD_PDOR = 0x39;
		for(i = 0; i < 30000; i++);
		GPIOD_PDOR = 0x3A;
		for(i = 0; i < 30000; i++);
	}
	else if ((mode & 0x8) == 0 && (mode & 0x04) == 0){ // 0, 0
		GPIOD_PDOR = 0x36;
		for(i = 0; i < 40000; i++);
		GPIOD_PDOR = 0x35;
		for(i = 0; i < 40000; i++);
		GPIOD_PDOR = 0x39;
		for(i = 0; i < 40000; i++);
		GPIOD_PDOR = 0x3A;
		for(i = 0; i < 40000; i++);
	}
}

unsigned short adc_read16b(){
	ADC0_SC1A = 0x00;
	while(ADC0_SC2 & ADC_SC2_ADACT_MASK);
	while(!(ADC0_SC1A & ADC_SC1_COCO_MASK));
	return ADC0_RA;
}

void PORTA_IRQHandler(void)
{
	mode = GPIOB_PDIR;
	light = (adc_read16b()* 33) / 0xFFFF;
	if(light > 16){
		speed(mode);
	}
	PORTA_ISFR = (1 << 1);
}

int main(void)
{
    SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;
    SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
    SIM_SCGC5 |= SIM_SCGC5_PORTD_MASK;
    SIM_SCGC6 |= SIM_SCGC6_ADC0_MASK;

    PORTB_GPCLR = 0x000C0100;
    GPIOB_PDDR = 0x00000000;

    PORTD_GPCLR = 0x00FF0100;
	GPIOD_PDDR = 0x000000FF;
	GPIOD_PDOR = 0x01;

    PORTA_PCR4 = 0x0100;

    ADC0_CFG1 = 0x0C;
    ADC0_SC1A = 0x1F;
    GPIOA_PDDR |= (0 << 1);
    GPIOB_PDDR |= ((0 << 3) | (0 << 2) | (1 << 2) | (1 << 10));

    PORTA_ISFR = (1 << 1);

    for(;;){
    	GPIOB_PTOR |= (1 << 10);
    	PORTA_IRQHandler();
    }
    return 0;
}

// EOF
