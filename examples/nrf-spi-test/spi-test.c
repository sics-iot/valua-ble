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
 *         An example to demonstrate basic SPI control of the nRF radio
 * \author
 *         Zhitao He <zhitao@sics.se>
 */

#include "contiki.h"
#include "platform-conf.h"
#include "sensors.h"
#include "button-sensor.h"
#include "csi00.h"

#include <stdio.h> /* For printf() */

extern unsigned long et_process_polled;
/*---------------------------------------------------------------------------*/
PROCESS(blink_process, "spi test process");
PROCESS(spi_test_process, "button process");
AUTOSTART_PROCESSES(&blink_process, &spi_test_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(blink_process, ev, data)
{
	static struct etimer et;
	
	PROCESS_BEGIN();

	iprintf("Hello, blink\n");

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
		etimer_set(&et, CLOCK_SECOND / 4 * 5);
	}
	  
	PROCESS_END();
}
/*---------------------------------------------------------------------------*/
/* nRF24L01+ commands */
#define FLUSH_TX 0xE1
#define FLUSH_RX 0xE2
#define REUSE_TX_PL 0xE3
#define NOP 0xFF

/* nRF24L01+ registers */
#define CONFIG 0x00
#define RF_CH 0x05

PROCESS_THREAD(spi_test_process, ev, data)
{
	static uint8_t  addr;
	static struct etimer et;
	
	PROCESS_BEGIN();

	puts("Hello, spi test");

	/* Init. button sensor */
	if (SENSORS_ACTIVATE(button_sensor) == 0) {
		puts("button activation failed");
		process_exit(process_current);
	}

	/* Set up digital outputs: CE and CSN */
	PM0 &= ~0x3;
	CE = 0; // deactivate RX/TX
	CSN = 1; // set SPI bus idle

	/* Read the values of nRF24L01+ registers */
	puts("nRF registers (HEX):");
	// insert Ugly tick delay btw. printfs to avoid overflowing the too small UART ring buffer
	etimer_set(&et, 1);
	for (addr = 0x00;addr <= 0x17;addr++) {
		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
		iprintf("%02X: %02X\n", addr, csi00_read(addr));
		etimer_reset(&et);
	}

	puts("Press button to test SPI access to nRF radio");

	while(1) {
		/* Wait on button event */
		PROCESS_WAIT_EVENT_UNTIL(ev == sensors_event && data == &button_sensor);
		/* NOP command returns radio status byte */
		iprintf("nRF status: 0x%02X\n", csi00_strobe(NOP));

		/* read-increment-verify RF channel */
		addr = RF_CH;
		uint8_t rf_ch = csi00_read(addr);
		iprintf("RF channel: %hu\n", rf_ch);
		rf_ch = (rf_ch + 1) % 128;
		csi00_write(0x20 | addr, rf_ch);
		iprintf("RF channel: %hu (updated)\n", csi00_read(addr));
	}

	PROCESS_END();
}
