/*
 * Copyright (c) 2015, SICS Swedish ICT.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 */

/**
 * \file
 *         A simple ADC driver for RL78G14
 * \author
 *         Zhitao He <zhitao@sics.se>
 */

#include "adc.h"
#include "platform-conf.h" // for f_CLK

/* Initialize ADC to prepare one-shot conversions on a single channel */
void adc_init(uint8_t channel)
{
	/* See Fig 14-31 Setting up softare trigger mode */
	/* enable ADC */
	ADCEN = 1;
	/* set ports to analog input: AIN0 (P20) */
	if (channel == ADC_AIN0_CHANNEL) {
		ADPC =0x02; // Fig 4-7 Format of A/D port config register
		/* set ports to input mode */
		PM2 |= (1<<0);
	}
	/* set channel selection mode, clock rate, Vdd level, sampling time */
#if f_CLK <= 16000000UL
	ADM0 = 0x38; // single channel, f_AD = f_CLK/2, Vdd >=2.7V, sampling time = 7/f_AD
#else
	ADM0 = 0x30; // // single channel, f_AD = f_CLK/4, Vdd >=2.7V, sampling time = 7/f_AD
#endif
	/* set trigger mode, conversion operation mode */
	ADM1 = 0x20; // software trigger, one-shot conversion
	/* set Vref+ & Vref-, interrupt signal condition, SNOOZE mode, 8/10 bit resolution */
	ADM2 = 0x00; // Vref+ = Vdd, Vref- = Vss,  10-bit resolution
	ADUL = 0xFF; // default
	ADLL = 0x00; // default
	/* select the channel to be A/D converted */
	ADS = channel; 
	/* enter ADC standby status */
	ADCE = 1;

	/* enable interrupt */
	/* ADIF = 0; */
	/* ADMK = 0; */
}

/* Perform a one-shot A/D conversion and return a 10-bit result */
uint16_t
adc_read(void)
{
	/* Start AD convertion */
	ADCS = 1;
	/* Wait for conversion complete */
	/* conversion start time = 1/f_CLK (assume f_AD = f_CLK/2) */
	/* conversion time (sample + convert) = 19/f_AD (assume "normal 1" mode, 10-bit conversion) */
	/* total wait time = 39/f_CLK = 2.4375 us (assume 16MHz f_CLK) */
	while(ADCS);
	/* result stored in higher 10 bits */
	return ADCR>>6;
}

void
adc_standby(void)
{
	ADIF = 0;
	ADCE = 1;
}

void
adc_stop(void)
{
	ADCE = 0;
	ADIF = 0;
}

/* ADC ISR */
void __attribute__ ((interrupt))
ad_handler(void)
{
	;
}
