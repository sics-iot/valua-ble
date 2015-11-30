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
 * \defgroup txrx-test nRF24L01+ broadcast TX/RX example
 * @{
 */

/**
 * \file
 *         An example to demonstrate basic TX/RX of the nRF radio
 * \author
 *         Zhitao He <zhitao@sics.se>
 */

#include "contiki.h"
#include "platform-conf.h"
#include "sensors.h"
#include "button-sensor.h"
#include "dev/serial-line.h"
#include "csi00.h"
#include "commands.h"
#include "watchdog.h"
#include "radio.h"
#include "random.h"
#include "nrf24l01p.h"

#include <stdio.h> /* For printf() */

/* modes */
#define PD 0
#define RX 1
#define TX 2

extern unsigned long et_process_polled;

static struct etimer et;
static unsigned seqno = 0;

#define TX_INTERVAL (CLOCK_SECOND / 10)
#define MAX_TX_PACKETS 10
#define PAYLOAD_LEN 10
static clock_time_t tx_interval = TX_INTERVAL;
static unsigned max_tx_packets = MAX_TX_PACKETS;
static unsigned payload_len = PAYLOAD_LEN;
static uint8_t rx_buf[32];
static struct radio_driver *radio;

static void
pd_mode(void)
{
	setreg(CONFIG,
	            (MASK_RX_DR|MASK_TX_DS|MASK_MAX_RT|PRIM_RX)&(~PWR_UP));
}

static void
rx_mode(void)
{
	(void)csi00_strobe(FLUSH_RX);
	setreg(CONFIG,
	            (MASK_TX_DS|MASK_MAX_RT|PWR_UP|PRIM_RX)&(~MASK_RX_DR));
	CE = 1;
}

static void
tx_mode(void)
{
	CE = 0;
	(void)csi00_strobe(FLUSH_TX);
	setreg(CONFIG,
	            (MASK_RX_DR|MASK_MAX_RT|PWR_UP)&(~MASK_TX_DS&~PRIM_RX));

	// timeout starts packet transmisison
	etimer_set(&et, CLOCK_SECOND);
}

/*---------------------------------------------------------------------------*/
PROCESS(blink_process, "blink process");
PROCESS(txrx_process, "txrx process");
AUTOSTART_PROCESSES(&blink_process, &txrx_process);
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

/* etimer expiration handler for TX mode */
static void
tx_et_handler(void)
{
	int i;
	int errno;
	uint8_t buf[32];

	if(seqno < max_tx_packets) {
		etimer_set(&et, tx_interval / 2 + random_rand() % tx_interval);

		for(i = 0;i < payload_len;++i) {
			buf[i] = (uint8_t)random_rand();
		}
		sniprintf((char *)buf, sizeof(buf), "%u", seqno);

		errno = radio->send(buf, payload_len);
		if (errno == RADIO_TX_OK) iprintf(". ");
		else iprintf("nRF send error: %d\n", errno);
		++seqno;
	} else {
		etimer_stop(&et);
		iprintf("Finished sending %u packets\n", seqno);
		seqno = 0;
	}
}

static int mode, last_mode;

struct mode {
	const char *display;
	void (*handler)(void);
	void (*prelog)(void);
	void (*prolog)(void);
	void (*et_handler)(void);
};

/* Radio modes */
const static struct mode mode_list[] = {
	{"Power down", pd_mode, NULL, NULL, NULL},
	{"RX", rx_mode, NULL, NULL, NULL},
	/* {UNMOD, "Unmodulated carrier", unmod_mode, stop_rtimer, NULL, attack_eth}, */
	{"TX broadcast", tx_mode, NULL, NULL, tx_et_handler},
};

/* Switch to a new radio mode */
static void
start_mode(int new_mode)
{
	if (new_mode<0 || new_mode>=sizeof(mode_list) / sizeof(mode_list[0]))
		return;

	if (mode_list[new_mode].prelog) mode_list[new_mode].prelog();
	mode_list[new_mode].handler();
	if (mode_list[new_mode].prolog) mode_list[new_mode].prolog();
	iprintf("%s mode\n", mode_list[new_mode].display);
	last_mode = mode;
	mode = new_mode;
}

/* Help message: available commands */
#define RF_CH 0x05
static void
channel_up(void)
{
	uint8_t rf_ch = getreg(RF_CH);
	rf_ch = (rf_ch + 1) % 128;
	setreg(RF_CH, rf_ch);
	iprintf("RF channel: %hu\n", getreg(RF_CH));
}
/*---------------------------------------------------------------------------*/
static void
channel_down(void)
{
	uint8_t rf_ch = getreg(RF_CH);
	rf_ch = (rf_ch - 1) % 128;
	setreg(RF_CH, rf_ch);
	iprintf("RF channel: %hu\n", getreg(RF_CH));
}

/*---------------------------------------------------------------------------*/
static void
status(void)
{
	iprintf("nRF status: 0x%02X\n", getreg(STATUS));
}

/*---------------------------------------------------------------------------*/
/* List of special commands */
static const struct command cmd_list[] =	{
	{'e', "Reboot", watchdog_reboot},
	{'+', "Channel+", channel_up},
	{'-', "Channel-", channel_down},
	/* {'v', "Successful pkt receptions", view_rx_statistics}, */
	/* {'V', "Unsuccessful pkt receptions", view_failed_rx_statistics}, */
	/* {'M', "MAC address update", mac_update}, */
	/* {'L', "Show all registers", show_all_registers}, */
	{'s', "nRF status byte", status},
	/* {'R', "Read TXFIFO", read_tx_fifo}, */
	/* {'W', "Write TXFIFO", write_tx_fifo}, */
	/* {'F', "Read RXFIFO", read_rx_fifo} */
};

/* List of field commands */
const static struct field field_list[] = {
	{'w', "RF_PWR", RF_SETUP, 2, 1},
};

/* List of user variable commands */
static struct variable const variable_list[] = {
	{'t', (union number*)&tx_interval, sizeof(tx_interval), "tx_interval", 0, (unsigned)~0},
	{'y', (union number*)&payload_len, sizeof(payload_len), "payload_len", 0, 32},
	{'p', (union number*)&max_tx_packets, sizeof(max_tx_packets), "max_tx_packets", 0, (unsigned)~0},
};

/*-----------------------------------------------------------------------------*/
/* Etimer timeout handler */
static void
et_handler(void)
{
	/* set next expiry time */
	etimer_reset(&et);
 /* call handler for current mode */
	if (mode_list[mode].et_handler)
		mode_list[mode].et_handler();
}
/*-----------------------------------------------------------------------------*/
PROCESS_THREAD(txrx_process, ev, data)
{
	PROCESS_BEGIN();

	iprintf("Hello, txrx test\n");

	/* Init. button sensor */
	if (SENSORS_ACTIVATE(button_sensor) == 0) {
		iprintf("button activation failed\n");
		process_exit(process_current);
	}

	/* Start radio  */
	radio = &nrf24l01p_driver;
	radio->init();
	start_mode(PD);

	/* Set serial commands: user callbacks and user data structures */
	commands_init(start_mode, getreg, setreg, cmd_list, sizeof(cmd_list) / sizeof(cmd_list[0]), field_list, sizeof(field_list) / sizeof(field_list[0]), variable_list, sizeof(variable_list) / sizeof(variable_list[0]));

	while(1) {
		PROCESS_WAIT_EVENT();
		if (ev == PROCESS_EVENT_POLL) {
			/* packet received */
			int i;
			int payload_len = radio->read(rx_buf, sizeof(rx_buf));
			iprintf("received (%hu): 0x", payload_len);
			for(i = 0;i < payload_len;i++) {
				iprintf("%02X", rx_buf[i]);
			}
			iprintf("\n");
		} else if (ev == PROCESS_EVENT_TIMER && etimer_expired(&et)) {
			/* call etimer expiry handler */
			et_handler();
		} else	if (ev == sensors_event && data == &button_sensor) {
			/* display help message about debug commands */
			do_command("h");
		} else if (ev == serial_line_event_message) {
			/* execute debug command */
			do_command((char *)data);
		}
	}

	PROCESS_END();
}
