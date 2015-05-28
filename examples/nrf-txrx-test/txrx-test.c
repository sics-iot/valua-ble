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

#include <stdio.h> /* For printf() */

extern unsigned long et_process_polled;

static struct etimer et;
static unsigned seqno = 0;

#define TX_INTERVAL (CLOCK_SECOND / 32)
#define MAX_TX_PACKETS 10
#define PAYLOAD_LEN 20
static clock_time_t tx_interval = TX_INTERVAL;
static unsigned max_tx_packets = MAX_TX_PACKETS;
static unsigned payload_len = PAYLOAD_LEN;

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
/*---------------------------------------------------------------------------*/
/* nRF24L01+ commands */
#define FLUSH_TX 0xE1
#define FLUSH_RX 0xE2
#define REUSE_TX_PL 0xE3
#define NOP 0xFF
#define R_RX_PL_WID 0x60
#define R_RX_PAYLOAD 0x61
#define W_TX_PAYLOAD 0xA0
#define W_TX_PAYLOAD_NO_ACK 0xB0

/* nRF24L01+ registers */
#define CONFIG 0x00
#define RF_CH 0x05
#define RF_SETUP 0x06
#define STATUS 0x07
#define RX_ADDR_P0 0x0A
#define TX_ADDR 0x10
#define DYNPD 0x1C
#define FEATURE 0x1D

/* nRF24L01+ register bits */
#define BV(b) (1<<(b))
// register CONFIG
#define MASK_RX_DR BV(6)
#define MASK_TX_DS BV(5)
#define MASK_MAX_RT BV(4)
#define EN_CRC BV(3)
#define CRCO BV(2)
#define PWR_UP BV(1)
#define PRIM_RX BV(0)
// register STATUS
#define RX_DR BV(6)
#define TX_DS BV(5)
#define MAX_RT BV(4)
#define RX_P_NO BV(3)|BV(2)|BV(1)
#define TX_FULL BV(0)

static uint8_t
getreg(uint8_t addr)
{
	return csi00_read(addr);
}

static void
setreg(uint8_t addr, uint8_t val)
{
	csi00_write(0x20 | addr, val);
}

/* modes */
#define PD 0
#define RX 1
#define TX 2

static int mode, last_mode;
static uint8_t tx_addr[5] = {0x55, 0xAA, 0x56, 0xAB, 0x5A};
static uint8_t rx_addr_p0[5] = {0x55, 0xAA, 0x56, 0xAB, 0x5A};

struct mode {
	int mode;
	const char *display;
	void (*handler)(void);
	void (*prelog)(void);
	void (*prolog)(void);
	void (*et_handler)(void);
};

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

static int
nrf_send(const void *payload, unsigned short len)
{
	csi00_write_message(W_TX_PAYLOAD_NO_ACK, (uint8_t *)payload, len);
	CE = 1; // activate RX/TX
	return RADIO_TX_OK;
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

		errno = nrf_send(buf, payload_len);
		if (errno == RADIO_TX_OK) iprintf(". ");
		else iprintf("nRF send error: %d\n", errno);
		++seqno;
	} else {
		etimer_stop(&et);
		iprintf("Finished sending %u packets\n", seqno);
		seqno = 0;
	}
}

const static struct mode mode_list[] = {
	{PD, "Power down", pd_mode, NULL, NULL, NULL},
	{RX, "RX", rx_mode, NULL, NULL, NULL},
	/* {UNMOD, "Unmodulated carrier", unmod_mode, stop_rtimer, NULL, attack_eth}, */
	{TX, "TX broadcast", tx_mode, NULL, NULL, tx_et_handler},
};

void
start_mode(int new_mode)
{
	if (mode_list[new_mode].prelog) mode_list[new_mode].prelog();
	mode_list[new_mode].handler();
	if (mode_list[new_mode].prolog) mode_list[new_mode].prolog();
	iprintf("%s mode\n", mode_list[mode].display);
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
static const struct command command_table[] =	{
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
	{'\0', "", NULL},
};

const static struct field field_list[] = {
	{'w', "RF_PWR", RF_SETUP, 2, 1},
	{'\0', NULL, 0x00, 0, 0},
};

/* user variables modifiable by serial commands */
static struct variable const user_variable_list[] = {
	{'t', (union number*)&tx_interval, sizeof(tx_interval), "tx_interval", 0, (unsigned)~0},
	{'y', (union number*)&payload_len, sizeof(payload_len), "payload_len", 0, 32},
	{'p', (union number*)&max_tx_packets, sizeof(max_tx_packets), "max_tx_packets", 0, (unsigned)~0},
	{'\0', NULL, 0, NULL, -1, -1}
};

/*-----------------------------------------------------------------------------*/
/* Etimer timeout handler */
static void
et_handler(void)
{
	etimer_reset(&et);
	if (mode_list[mode].et_handler)
		mode_list[mode].et_handler();
}

/*-----------------------------------------------------------------------------*/
PROCESS_THREAD(txrx_process, ev, data)
{
	/* static uint8_t  addr; */
	uint8_t tx_buf[32];
	uint8_t rx_buf[32];
	
	PROCESS_BEGIN();

	iprintf("Hello, txrx test\n");

	/* Init. button sensor */
	if (SENSORS_ACTIVATE(button_sensor) == 0) {
		iprintf("button activation failed\n");
		process_exit(process_current);
	}

	/* Set up digital output: CE and CSN */
	PM0 &= ~0x3;
	CSN = 1; // set SPI bus idle
	CE = 0; // deactivate RX/TX

	/* set up irq (INTP0) registers */
	// falling edge trigger
	EGP0bits.egp0 = 0; // Caution: both the register and its bit 0 are named EGP0 in the manual
	EGN0bits.egn0 = 1; // Caution: both the register and its bit 0 are named EGN0 in the manual
	// enable edge detection irq
	PIF0 = 0;
	PMK0 = 0;
	// no need to set port direction as P137 is input-only

	/* Enable all three features: dynamic payload length, payload with ack, w_tx_payload_noack command */
	setreg(FEATURE, 0x07);
	/* Enable dynamic pl length in data pipe 0 */
	setreg(DYNPD, 0x01);

	/* Set addresses */
	int i;
	csi00_write_message(0x20 | TX_ADDR, tx_addr, sizeof(tx_addr));
	(void)csi00_read_message(TX_ADDR, tx_addr, sizeof(tx_addr));
	iprintf("TX addr: 0x");
	for(i = 0;i < sizeof(tx_addr);i++) {
		iprintf("%02X", tx_addr[i]);
	}
	iprintf("\n");

	csi00_write_message(0x20 | RX_ADDR_P0, rx_addr_p0, sizeof(rx_addr_p0));
	(void)csi00_read_message(RX_ADDR_P0, rx_addr_p0, sizeof(rx_addr_p0));
	iprintf("RX addr p0: 0x");
	for(i = 0;i < sizeof(rx_addr_p0);i++) {
		iprintf("%02X", rx_addr_p0[i]);
	}
	iprintf("\n");

	/* Set RF channel */
	setreg(RF_CH, 63);
	iprintf("RF channel: %hu\n", getreg(RF_CH));

	/* Set radio mode  */
	start_mode(PD);

	PROCESS_PAUSE();

	/* Set serial commands: user callbacks and user data structures */
	commands_init(start_mode, getreg, setreg, command_table, field_list, user_variable_list);

	while(1) {
		PROCESS_WAIT_EVENT();
		if (ev == PROCESS_EVENT_POLL) {
			uint8_t payload_len = getreg(R_RX_PL_WID);
			(void)csi00_read_message(R_RX_PAYLOAD, rx_buf, payload_len);
			iprintf("received (%hu): 0x", payload_len);
			for(i = 0;i < payload_len;i++) {
				iprintf("%02X", rx_buf[i]);
			}
			iprintf("\n");
		} else if (ev == PROCESS_EVENT_TIMER && etimer_expired(&et)) {
			et_handler();
		} else	if (ev == sensors_event && data == &button_sensor) {
			iprintf("seqno: %u\n", seqno);
			int n = sniprintf((char *)tx_buf, sizeof(tx_buf), "%u", seqno);
			csi00_write_message(W_TX_PAYLOAD_NO_ACK, (uint8_t *)tx_buf, n);
			CE = 1; // activate RX/TX
			seqno++;
			/* clock_delay_usec(10); */
			/* CE = 0; */
		} else if (ev == serial_line_event_message) {
			do_command((char *)data);
		}
	}

	PROCESS_END();
}

/*---------------------------------------------------------------------------*/
static void
packet_sent(void)
{
	iprintf("sent\n");
}
/*---------------------------------------------------------------------------*/
static void
rtx_timeout(void)
{
	iprintf("retransmission timeout\n");
}
/*---------------------------------------------------------------------------*/
/* nRF IRQ (INTP0) ISR */
void __attribute__ ((interrupt))
p0_handler(void)
{
	PMK0 = 1;
	uint8_t status = getreg(STATUS);
	// write '1' to clear bit
	if (status & TX_DS) {
		setreg(STATUS, status | TX_DS);
		packet_sent();
		CE = 0; // radio power-down mode
	} else if (status & RX_DR) {
		setreg(STATUS, status | RX_DR);
		process_poll(&txrx_process);
	} else if (status & MAX_RT) {
		setreg(STATUS, status | MAX_RT);
		rtx_timeout();
	}
 	PMK0 = 0;
}

