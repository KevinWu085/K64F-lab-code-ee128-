
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
#include <stdint.h>

static int i = 0;

int main(void)
{
    SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK; // Enable Port B Clock Gate Control
    SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK; // Enable Port C Clock Gate Control
    SIM_SCGC5 |= SIM_SCGC5_PORTD_MASK; // Enable Port D Clock Gate Control

    PORTD_GPCLR = 0x00FF0100; // Configure Port D Pins 0-7 for GPIO
    PORTC_GPCLR = 0x01BF0100; // Configure Port C Pins 0-5 and 7-8 for GPIO
    PORTB_GPCLR = 0x000C0100; // Configure Port B Pin 2-3 for GPIO

    GPIOB_PDDR = 0x00000000; // Configure Port B Pin 2-3 for Input
    GPIOD_PDDR = 0x000000FF; // Configure Port D Pins 0-7 for Output
    GPIOC_PDDR = 0x000001BF; // Configure Port C Pins 0-5 and 7-8 for Output

    GPIOC_PDOR = 0x00; // Initialize Port C to 0
    GPIOD_PDOR = 0x01; // Initialize Port D such that only 1 bit is ON

    uint8_t counter = 0;
    uint8_t shifter = 1;
    uint32_t inputRegister = 0;

    while (1)
    {
        for (i = 0; i < 100000; i++); // Simple delay

        inputRegister = GPIOB_PDIR; // Read Port B

        // Update counter based on input pin (bit 3 of Port B)
        if ((inputRegister & 0x8) == 0)
        {
            if (counter > 0)
                counter -= 1;
            else
                counter = 0xFF;
        }
        else
        {
            if (counter < 0xFF)
                counter += 1;
            else
                counter = 0;
        }

        // Update shifter based on input pin (bit 2 of Port B)
        if ((inputRegister & 0x04) == 0)
        {
            if ((shifter & 0x80) == 0)
                shifter = (shifter << 1); // Left rotate
            else
                shifter = 1;
        }
        else
        {
            if ((shifter & 0x01) == 0)
                shifter = (shifter >> 1); // Right rotate
            else
                shifter = 0x80;
        }

        // Write to Port C and Port D
        GPIOC_PDOR = (uint32_t)(counter & 0x3F) | ((uint32_t)(counter & 0xC0) << 1);
        GPIOD_PDOR = (uint32_t)shifter;
    }

    return 0;
}
