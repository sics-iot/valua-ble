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
 * This file is NOT part of the Contiki operating system.
 *
 */

/**
 * \file
 *         A Contiki application
 * \author
 *         Zhitao He <zhitao@sics.se>
 */

#include "contiki.h"
#include "platform-conf.h"
#include "sensors.h"
#include "button-sensor.h"

#include <stdio.h> /* For printf() */

extern unsigned long et_process_polled;
/*---------------------------------------------------------------------------*/
PROCESS(blink_process, "blink process");
PROCESS(button_process, "button process");
AUTOSTART_PROCESSES(&blink_process, &button_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(blink_process, ev, data)
{
	static struct etimer et;
	static int n;
	
	PROCESS_BEGIN();

	/* iprintf works the same way as printf, except ignoring floating point %f %g etc. */
	iprintf("Hello, blink\n");

	/* TEST: led on for 3 seconds */
	LED1 = 0;
	etimer_set(&et, CLOCK_SECOND * 3);
	PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
	LED1 = 1;

	etimer_set(&et, CLOCK_SECOND * 1);

	while(1) {
		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
		LED1 = 0;
		etimer_set(&et, CLOCK_SECOND / 4);
		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
		LED1 = 1;
		etimer_set(&et, CLOCK_SECOND / 4);
		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
		LED1 = 0;
		etimer_set(&et, CLOCK_SECOND / 4);
		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
		LED1 = 1;
		iprintf("tick %d uptime %lu clock_time %lu et_process_polled %lu\n",
		        n++, clock_seconds(), clock_time(), et_process_polled);
		etimer_set(&et, CLOCK_SECOND / 4 * 5);
	}
	  
	PROCESS_END();
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(button_process, ev, data)
{
	PROCESS_BEGIN();

	/* Init. button sensor */
	if (SENSORS_ACTIVATE(button_sensor) == 0) {
		puts("button activation failed");
		process_exit(process_current);
	}

	puts("Hello, button");

	while(1) {
		/* Wait on button event */
		PROCESS_WAIT_EVENT_UNTIL(
		                         ev == sensors_event &&
		                         data == &button_sensor);
		/* LED1 = ~LED1; */
		puts("Ouch, don't push me!");
	}

	PROCESS_END();
}
/*---------------------------------------------------------------------------*/
