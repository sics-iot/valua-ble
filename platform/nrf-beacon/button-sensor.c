/*
 * Copyright (c) 2015, SICS Swedish ICT
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
 * This file is NOT part of the Contiki operating system.
 *
 */
#include "contiki.h"
#include "lib/sensors.h"
#include "dev/button-sensor.h"
#include "sfrs.h"

const struct sensors_sensor button_sensor;

static struct timer debouncetimer;
static int status(int type);

/*---------------------------------------------------------------------------*/
/* Button (INTP4) ISR */
void __attribute__ ((interrupt))
p4_handler(void)
{
	PMK4 = 1;
	if(timer_expired(&debouncetimer) && P31 == 0) {
		timer_set(&debouncetimer, CLOCK_SECOND / 4);
		sensors_changed(&button_sensor);
	}
	PMK4 = 0;
	// MAYBE: exit to low power mode
}
/*---------------------------------------------------------------------------*/
static int
value(int type)
{
	/* return BUTTON_READ() || !timer_expired(&debouncetimer); */
	return P31;
}
/*---------------------------------------------------------------------------*/
static int
configure(int type, int c)
{
	switch (type) {
	case SENSORS_ACTIVE:
		if (c) {
			if (!status(SENSORS_ACTIVE)) {
				timer_set(&debouncetimer, 0);

				/* set up button's port (P31) and irq (INTP4) registers */
				// falling edge trigger
				EGP4 = 0;
				EGN4 = 1;
				// direction : input
				PM3 |= 1<<1;
				// enable edge detection irq
				PIF4 = 0;
				PMK4 = 0;
			}
		} else {
			PMK4 = 1;
		}
		return 1;
	}
	return 0;
}
/*---------------------------------------------------------------------------*/
static int
status(int type)
{
	switch (type) {
	case SENSORS_ACTIVE:
	case SENSORS_READY:
		return !PMK4;
	}
	return 0;
}
/*---------------------------------------------------------------------------*/
SENSORS_SENSOR(button_sensor, BUTTON_SENSOR,
               value, configure, status);
