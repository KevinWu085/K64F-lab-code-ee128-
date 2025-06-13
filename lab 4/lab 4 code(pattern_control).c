
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

int main(void)
{

    SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK; /*Enable Port B Clock Gate Control*/
    SIM_SCGC5 |= SIM_SCGC5_PORTD_MASK; /*Enable Port D Clock Gate Control*/
    PORTD_GPCLR = 0x00FF0100; /*Configure Port D Pins 0-7 for GPIO*/
    PORTB_GPCLR = 0x000C0100; /*Configure Port B Pin 2-3 for GPIO*/
    GPIOB_PDDR = 0x00000000; /*Configure Port B Pin 2-3 for Input*/
    GPIOD_PDDR = 0x000000FF; /*Configure Port D Pins 0-7 for Output*/
    GPIOD_PDOR = 0x01; /*Initialize Port D such that only 1 bit is ON*/

    uint32_t inputRegister = 0;

    for(;;){
        inputRegister = GPIOB_PDIR; /*read Port B*/
        if ((inputRegister & 0x8) == 0 && (inputRegister & 0x04) == 0){
            GPIOD_PDOR = 0x36;
            for(i = 0; i < 25000; i++);
            GPIOD_PDOR = 0x35;
            for(i = 0; i < 25000; i++);
            GPIOD_PDOR = 0x39;
            for(i = 0; i < 25000; i++);
            GPIOD_PDOR = 0x3A;
            for(i = 0; i < 25000; i++);
        }
        else if ((inputRegister & 0x8) != 0 && (inputRegister & 0x04) == 0){
            GPIOD_PDOR = 0x36;
            for(i = 0; i < 25000; i++);
            GPIOD_PDOR = 0x3A;
            for(i = 0; i < 25000; i++);
            GPIOD_PDOR = 0x39;
            for(i = 0; i < 25000; i++);
            GPIOD_PDOR = 0x35;
            for(i = 0; i < 25000; i++);
        }
        else if ((inputRegister & 0x8) == 0 && (inputRegister & 0x04) != 0){
            GPIOD_PDOR = 0x36;
            for(i = 0; i < 10000; i++);
            GPIOD_PDOR = 0x35;
            for(i = 0; i < 10000; i++);
            GPIOD_PDOR = 0x39;
            for(i = 0; i < 10000; i++);
            GPIOD_PDOR = 0x3A;
            for(i = 0; i < 10000; i++);
        }
        else if ((inputRegister & 0x8) != 0 && (inputRegister & 0x04) != 0){
            GPIOD_PDOR = 0x36;
            for(i = 0; i < 10000; i++);
            GPIOD_PDOR = 0x3A;
            for(i = 0; i < 10000; i++);
            GPIOD_PDOR = 0x39;
            for(i = 0; i < 10000; i++);
            GPIOD_PDOR = 0x35;
            for(i = 0; i < 10000; i++);
        }
    }
    return 0;
}
////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
