/*
 * Copyright (c) 2008, Swedish Institute of Computer Science.
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
 * This file is part of the Contiki operating system.
 *
 */

/**
 * \file
 *         Channel jammer program for MSB430
 * \author
 *         Zhitao He <zhitao@sics.se>
 */

#include "dev/button-sensor.h"
#include "contiki.h"
#include "net/rime.h"
#include "dev/leds.h"
#include "dev/serial-line.h"
#include "dev/spi.h"
#include "dev/cc1020.h"
#include "dev/cc1020-internal.h"
#include "node-id.h"
#include "lib/random.h"
#include "dev/radio.h"
#include "dev/watchdog.h"

#include <stdio.h>
#include <string.h>

#define DEBUG 0
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#define CHANNEL 16
#define DEFAULT_TXPOWER_LEVEL 27

uint8_t cc1020_read_reg(uint8_t addr);
void cc1020_write_reg(uint8_t addr, uint8_t adata);

int jam_ena;

const unsigned char tx_power_level[10] = {0,1,3,7,11,15,19,23,27,31};

/* enum modes {RX, TX, SNIFF, JAM, MOD, UNMOD, CH}; */
enum modes {RX, TX, CH, MOD};

enum modes mode;
#define NUM_MODES 4

const enum modes INITIAL_MODE = RX;

static struct etimer et;

/* static unsigned time_send; */

static uint16_t seqno;

PROCESS(test_process, "MSB430 jammer");
AUTOSTART_PROCESSES(&test_process);

/* /\*---------------------------------------------------------------------------*\/ */
/* /\* Reset transmitter to normal packet mode *\/ */
/* void */
/* reset_transmitter(void) */
/* { */
/*   unsigned reg; */

/*   /\* set normal TX mode 0, normal RX mode 0 *\/ */
/* #define TX_MODE_BV (3<<2); */
/* #define RX_MODE_BV (3<<0); */
/* #define RX_MODE_1 (1<<0); */
/*   reg = getreg(CC2420_MDMCTRL1); */
/* 	reg &= ~TX_MODE_BV; */
/* 	reg &= ~RX_MODE_BV; */
/* 	/\* reg |= RX_MODE_1; // test: set rx mode to unbuffered mode *\/ */
/*   setreg(CC2420_MDMCTRL1, reg); */
/* 	printf("0x%04x\n", reg); */

/*   /\* set TX DAC's data source to modulator (normal operation) *\/ */
/*   /\* reg = getreg(CC2420_DACTST); *\/ */
/*   /\* setreg(CC2420_DACTST, reg & 0x0FFF); *\/ */

/*   /\* enter RX mode *\/ */
/*   strobe(CC2420_SRXON); */
/* } */
/*---------------------------------------------------------------------------*/
#define PN9_ENABLE_BIT 2
#define DIO_BIT 1
/* static volatile char dma_done; */
/* Transmit a continuous carrier */
void
send_carrier(int mode)
{
	uint8_t reg = cc1020_read_reg(CC1020_MODEM);
	printf("MODEM = 0x%02x\n", (unsigned)reg);
	//The PN9_ENABLE bit in the MODEM register enables the PN9 generator
	reg |= 1<<PN9_ENABLE_BIT; 
	cc1020_write_reg(CC1020_MODEM, reg);
	reg = cc1020_read_reg(CC1020_MODEM);
	printf("MODEM = 0x%02x\n", (unsigned)reg);
	//A transition on the DIO pin is required after enabling the PN9 pseudo random sequence.
	reg = P3DIR;
	printf("P3DIR = 0x%02x\n", (unsigned)reg);
	reg |= 1<<DIO_BIT;
	P3DIR = reg;
	printf("P3DIR = 0x%02x\n", (unsigned)reg);
	reg = P3OUT;
	printf("P3OUT = 0x%02x\n", (unsigned)reg);
	reg = reg & (1<<DIO_BIT) ? reg & ~(1<<DIO_BIT) : reg | (1<<DIO_BIT);
	P3OUT = reg;
	printf("P3OUT = 0x%02x\n", (unsigned)reg);

	/* uint8_t foo = 0xAA; */
  /* dma_done = 0; */
  /* dma_transfer((unsigned char *)&TXBUF0, &foo, 1); */
  /* while(!dma_done); */
}
/*---------------------------------------------------------------------------*/
#define TX_INTERVAL CLOCK_SECOND / 1
static void
start_mode(enum modes to)
{
	uint8_t reg;
	/* stop PN9 generator */
	if(mode == MOD) {
		reg = cc1020_read_reg(CC1020_MODEM);
		reg &= ~(0x1<<PN9_ENABLE_BIT); 
		cc1020_write_reg(CC1020_MODEM, reg);
		reg = cc1020_read_reg(CC1020_MODEM);
		printf("MODEM = 0x%02x\n", (unsigned)reg);
	}
  mode = to;
	/* DISABLE_CCA_INT(); */
	/* DISABLE_FIFO_INT(); */
	/* ENABLE_FIFOP_INT(); */
	jam_ena = 0;
	/* reset_transmitter(); */
  switch(mode) {
  case RX: 
    printf("RX mode\n"); 
    break;
  case CH:
    printf("Channel sample mode\n");
		etimer_set(&et, CLOCK_SECOND / 1);
    break;
  case TX: 
		printf("TX mode\n"); 
		seqno = 0;
		/* DISABLE_FIFOP_INT(); // disable cc2420 interrupt */
		etimer_set(&et, TX_INTERVAL);
		break;
/*   case UNMOD:  */
/* 		printf("Unmodulated carrier mode\n");  */
/* 		send_carrier(mode);  */
/* 		break; */
  case MOD:
		printf("Modulated carrier mode\n");
		send_carrier(mode);
		break;
/* 	case JAM: */
/* 		printf("Jam mode\n"); */
/* 		jam_ena = 1; */
/* 		DISABLE_FIFOP_INT(); // disable cc2420 "packet data received" interrupt */
/* #if ENABLE_CCA_INTERRUPT */
/* 		ENABLE_CCA_INT(); // enable cc2420 "packet header detected" interrupt */
/* #elif ENABLE_FIFO_INTERRUPT */
/* 		ENABLE_FIFO_INT(); // enable cc2420 "packet header detected" interrupt */
/* #endif */
/* 		break; */
/* 	case SNIFF: */
/* 		printf("Sniff mode\n"); */
/* 		DISABLE_FIFOP_INT(); // disable cc2420 "packet data received" interrupt */
/* #if ENABLE_CCA_INTERRUPT */
/* 		ENABLE_CCA_INT(); // enable cc2420 "packet header detected" interrupt */
/* #elif ENABLE_FIFO_INTERRUPT */
/* 		ENABLE_FIFO_INT(); // enable cc2420 "packet header detected" interrupt */
/* #elif ENABLE_UNBUFFERED_MODE */
/* #define RX_MODE_BV (3<<0); */
/* #define RX_MODE_1 (1<<0); */
/* #define RX_MODE_2 (2<<0); */
/* #define RX_MODE_3 (3<<0); */
/* 		reg = getreg(CC2420_MDMCTRL1); */
/* 		reg &= ~RX_MODE_BV; */
/* 		reg |= RX_MODE_1; */
/* 		setreg(CC2420_MDMCTRL1, reg); */
/* 		ENABLE_FIFOP_INT(); */
/* #endif */
/* 		break; */
  default:;
  }
}

#define MAX_TX_PACKETS 999
#define MAX_PHY_LEN 5
/* void */
/* cc2420_set_receiver(void (*f)(const struct radio_driver *)); */
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(test_process, ev, data)
{
  /* char str[41];// PHY LEN = strlen + broadcast hdrlen (9 bytes) */
	int errno;
	const struct radio_driver *d;
	static char last_ch;

  PROCESS_BEGIN();

	/* node_id_burn(1); */

	d = &cc1020_driver;

	uint8_t reg = cc1020_read_reg(CC1020_MODEM);
	printf("MODEM = 0x%02x\n", reg);

	/* button_sensor.configure(SENSORS_ACTIVE, 1); */

	/* initialize GIO pins */
  /* GIO_PxDIR |= (GIO2_BV | GIO3_BV); */
  /* GIO_PxOUT &= ~(GIO2_BV | GIO3_BV); */
	/* // toggle GIO2 for testing */
	/* GIO_PxOUT |= GIO2_BV; */
	/* GIO_PxOUT &= ~GIO2_BV; */

  /* cc2420_set_channel(CHANNEL); */
	/* cc2420_set_txpower(DEFAULT_TXPOWER_LEVEL); */

	/* d->set_receive_function(&packet_received); */

  /* initialize transceiver mode */
  start_mode(INITIAL_MODE);

  printf("Test process starts: channel = %d, txpower level = %u\n", CHANNEL, DEFAULT_TXPOWER_LEVEL);

  while(1) {
    PROCESS_WAIT_EVENT();
    if(ev == PROCESS_EVENT_TIMER && etimer_expired(&et)) {
			etimer_reset(&et);
      if(mode == TX && seqno < MAX_TX_PACKETS) {
				/* time_send = RTIMER_NOW(); */
				/* clock_delay(seqno % 4 + 1); */
#if ENABLE_FAST_TX
				char str[1+MAX_PHY_LEN]; // PHY HDR + payload
				memset(str, 0, sizeof(str));
				str[0] = MAX_PHY_LEN + 2; // PHY HDR= payload size + CRC size
				snprintf(&str[1], sizeof(str)-1, "%u", seqno);
				errno = d->send(str, sizeof(str)); // PHY HDR + PHY payload -> TX FIFO
#else
				char str[MAX_PHY_LEN];//
				memset(str, 0, sizeof(str));
				snprintf(str, sizeof(str), "%u", seqno);
				errno = d->send(str, sizeof(str));
#endif
				if (errno == RADIO_TX_OK) {
					printf("cc2420_send ok: %d bytes\n", sizeof(str));
				} else {
					printf("cc2420_send error: %d\n", errno);
				}
				++seqno;
      } else if(mode == CH) {
				uint8_t rssi = cc1020_get_rssi();
				printf("rssi = %u\n", (unsigned)rssi);
			/* 	/\* int8_t rssi = cc2420_rssi(); *\/ */
			/* 	int rssi; */
			/* 	while(!(status() & BV(CC2420_RSSI_VALID))) { */
			/* 		/\*    printf("cc2420_rssi: RSSI not valid.\n");*\/ */
			/* 	} */
			/* 	rssi = (int)((signed char)getreg(CC2420_RSSI)); */
			/* 	unsigned lna = getreg(CC2420_AGCCTRL) & 0x0003; */
			/* 	printf("rssi = %d, lna = %u\n", rssi, lna); */
			/* 	if(!CCA_IS_1) { */
			/* 		leds_on(LEDS_BLUE); */
			/* 		clock_delay(200); */
			/* 		leds_off(LEDS_BLUE); */
			}
    } else if(ev == serial_line_event_message) {
				char ch = *(char *)data;
				if(ch == '\0') {
					ch = last_ch;
				}
				last_ch = ch;
			/* if(ch >= '0' && ch <= '9') { */
			/* 		cc2420_set_txpower(tx_power_level[ch - '0']); */
			/* 		printf("tx power = %u\n", cc2420_get_txpower()); */
      /* } else if(ch == 'r') { */
			/* 	int8_t val = cc2420_rssi(); */
			/* 	printf("rssi = %d\n", val); */
      /* } else if(ch == 'n') { */
				if(ch == 'n') {
					start_mode((mode + 1) % NUM_MODES);
				} else if(ch == 'p') {
					start_mode((mode - 1) % NUM_MODES);
				} else if(ch == 'e') {
					watchdog_reboot();
				} else {
					printf("invalid input\n");
				}
			}
	}
	PROCESS_END();
}
/*---------------------------------------------------------------------------*/
int
decision(long int n, uint8_t len, uint8_t *buf)
{
	return (n % 2 == 1);
	/* if(len > 0) { */
	/* 	if(random_rand() < ((uint16_t)(~0) / 5)) { */
	/* 		return 1; */
	/* 	} */
	/* }  */
	/* if(len == 43) { */
	/* 	return 1; */
	/* } */
	/* if(buf[7] == 0x00 && buf[8] == 0x00 && buf[0] == 0x41) { */
	/* 	return 1; */
	/* } */
	/* return 0; */
}

#include "net/mac/nullrdc-noframer.h"
#include "net/packetbuf.h"
#include "net/queuebuf.h"
#include "net/netstack.h"
#include <string.h>

/*---------------------------------------------------------------------------*/
static void
send_packet(mac_callback_t sent, void *ptr)
{
  int ret;
  if(NETSTACK_RADIO.send(packetbuf_hdrptr(), packetbuf_totlen()) == RADIO_TX_OK) {
    ret = MAC_TX_OK;
  } else {
    ret =  MAC_TX_ERR;
  }
  mac_call_sent_callback(sent, ptr, ret, 1);
}
/*---------------------------------------------------------------------------*/
static void
send_list(mac_callback_t sent, void *ptr, struct rdc_buf_list *buf_list)
{
  if(buf_list != NULL) {
    queuebuf_to_packetbuf(buf_list->buf);
    send_packet(sent, ptr);
  }
}
/*---------------------------------------------------------------------------*/
static void
packet_input(void)
{
	uint8_t *rxbuf_ptr;
	PRINTF("%d\n", packetbuf_datalen());
	if(packetbuf_datalen() > 0) {
		printf("%d: 0x", packetbuf_datalen());
		for(rxbuf_ptr = packetbuf_dataptr();rxbuf_ptr < packetbuf_dataptr() + packetbuf_datalen();rxbuf_ptr++) {
			printf("%02x", *rxbuf_ptr);
		}
		printf(" RSSI %u\n", (unsigned)cc1020_get_packet_rssi());
		printf("\n");
	}
}
/*---------------------------------------------------------------------------*/
static int
on(void)
{
  return NETSTACK_RADIO.on();
}
/*---------------------------------------------------------------------------*/
static int
off(int keep_radio_on)
{
  if(keep_radio_on) {
    return NETSTACK_RADIO.on();
  } else {
    return NETSTACK_RADIO.off();
  }
}
/*---------------------------------------------------------------------------*/
static unsigned short
channel_check_interval(void)
{
  return 0;
}
/*---------------------------------------------------------------------------*/
static void
init(void)
{
  on();
}
/*---------------------------------------------------------------------------*/
const struct rdc_driver rawrdc_driver = {
  "rawrdc",
  init,
  send_packet,
  send_list,
  packet_input,
  on,
  off,
  channel_check_interval,
};
/*---------------------------------------------------------------------------*/
