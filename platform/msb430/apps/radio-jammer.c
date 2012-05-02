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
#include "node-id.h"
#include "lib/random.h"
#include "dev/radio.h"
#include "dev/watchdog.h"

#include <stdio.h>
#include <string.h>

#define DEBUG 1
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

enum modes {RX, TX, SNIFF, JAM, MOD, UNMOD, CH};

enum modes mode;
#define NUM_MODES 7

const enum modes INITIAL_MODE = RX;

static struct etimer et;

/* static unsigned time_send; */

static uint16_t seqno;

PROCESS(test_process, "MSB430 jammer");
AUTOSTART_PROCESSES(&test_process);

/*---------------------------------------------------------------------------*/
static void
strobe(enum cc2420_register regname)
{
  FASTSPI_STROBE(regname);
}
/*---------------------------------------------------------------------------*/
static unsigned int
status(void)
{
  uint8_t status;
  FASTSPI_UPD_STATUS(status);
  return status;
}
/*---------------------------------------------------------------------------*/
static void
flushrx(void)
{
  uint8_t dummy;

  FASTSPI_READ_FIFO_BYTE(dummy);
  FASTSPI_STROBE(CC2420_SFLUSHRX);
  FASTSPI_STROBE(CC2420_SFLUSHRX);
  /* rxptr = 0; */
}
/*---------------------------------------------------------------------------*/
/* Reset transmitter to normal packet mode */
void
reset_transmitter(void)
{
  unsigned reg;

  /* set normal TX mode 0, normal RX mode 0 */
#define TX_MODE_BV (3<<2);
#define RX_MODE_BV (3<<0);
#define RX_MODE_1 (1<<0);
  reg = getreg(CC2420_MDMCTRL1);
	reg &= ~TX_MODE_BV;
	reg &= ~RX_MODE_BV;
	/* reg |= RX_MODE_1; // test: set rx mode to unbuffered mode */
  setreg(CC2420_MDMCTRL1, reg);
	printf("0x%04x\n", reg);

  /* set TX DAC's data source to modulator (normal operation) */
  /* reg = getreg(CC2420_DACTST); */
  /* setreg(CC2420_DACTST, reg & 0x0FFF); */

  /* enter RX mode */
  strobe(CC2420_SRXON);
}
/*---------------------------------------------------------------------------*/
/* Transmit a continuous carrier */
void
send_carrier(int mode)
{
  unsigned reg = getreg(CC2420_MDMCTRL1);
  if(mode == UNMOD) {
		/* unmodulated carrier */
    reg = (reg & 0xFFF3) | (3 << 2);
    setreg(CC2420_MDMCTRL1, reg);
    setreg(CC2420_DACTST, 0x1800);
  } else if(mode == MOD) {
		/* randomly modulated carrier */
    reg = (reg & 0xFFF3) | (3 << 2);
    setreg(CC2420_MDMCTRL1, reg);
  }
	strobe(CC2420_STXON);
}
/*---------------------------------------------------------------------------*/
#define TX_INTERVAL CLOCK_SECOND / 1
static void
start_mode(enum modes to)
{
	unsigned reg;
  mode = to;
	DISABLE_CCA_INT();
	DISABLE_FIFO_INT();
	ENABLE_FIFOP_INT();
	jam_ena = 0;
	reset_transmitter();
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
		DISABLE_FIFOP_INT(); // disable cc2420 interrupt
		etimer_set(&et, TX_INTERVAL);
		break;
  case UNMOD: 
		printf("Unmodulated carrier mode\n"); 
		send_carrier(mode); 
		break;
  case MOD: 
		printf("Modulated carrier mode\n"); 
		send_carrier(mode); 
		break;
	case JAM:
		printf("Jam mode\n");
		jam_ena = 1;
		DISABLE_FIFOP_INT(); // disable cc2420 "packet data received" interrupt
#if ENABLE_CCA_INTERRUPT
		ENABLE_CCA_INT(); // enable cc2420 "packet header detected" interrupt
#elif ENABLE_FIFO_INTERRUPT
		ENABLE_FIFO_INT(); // enable cc2420 "packet header detected" interrupt
#endif
		break;
	case SNIFF:
		printf("Sniff mode\n");
		DISABLE_FIFOP_INT(); // disable cc2420 "packet data received" interrupt
#if ENABLE_CCA_INTERRUPT
		ENABLE_CCA_INT(); // enable cc2420 "packet header detected" interrupt
#elif ENABLE_FIFO_INTERRUPT
		ENABLE_FIFO_INT(); // enable cc2420 "packet header detected" interrupt
#elif ENABLE_UNBUFFERED_MODE
#define RX_MODE_BV (3<<0);
#define RX_MODE_1 (1<<0);
#define RX_MODE_2 (2<<0);
#define RX_MODE_3 (3<<0);
		reg = getreg(CC2420_MDMCTRL1);
		reg &= ~RX_MODE_BV;
		reg |= RX_MODE_1;
		setreg(CC2420_MDMCTRL1, reg);
		ENABLE_FIFOP_INT();
#endif
		break;
  default:;
  }
}
/*---------------------------------------------------------------------------*/
// radio receiver callback
static uint16_t rxbuf[64];
/* static const uint8_t *rxbuf_ptr = (uint8_t *)rxbuf; */

static void
packet_received(const struct radio_driver * r)
{
	int len;
	uint8_t *rxbuf_ptr;
	/* int i; */
	len = r->read(rxbuf, sizeof(rxbuf));
	if(len > 0) {
	/* if(len != 0 && len == 41) { */
		printf("%d: 0x", len);
		for(rxbuf_ptr = (uint8_t *)&rxbuf[0];rxbuf_ptr < (uint8_t *)rxbuf + len;rxbuf_ptr++) {
			printf("%02x", *rxbuf_ptr);
		}
		printf("\n");
	}
}

#define MAX_TX_PACKETS 999
#define MAX_PHY_LEN 3
void
cc2420_set_receiver(void (*f)(const struct radio_driver *));
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

	button_sensor.configure(SENSORS_ACTIVE, 1);

	/* initialize GIO pins */
  GIO_PxDIR |= (GIO2_BV | GIO3_BV);
  GIO_PxOUT &= ~(GIO2_BV | GIO3_BV);
	// toggle GIO2 for testing
	GIO_PxOUT |= GIO2_BV;
	GIO_PxOUT &= ~GIO2_BV;

  cc2420_set_channel(CHANNEL);
	cc2420_set_txpower(DEFAULT_TXPOWER_LEVEL);

	d->set_receive_function(&packet_received);

  /* initialize transceiver mode */
  start_mode(INITIAL_MODE);

  printf("Sampling process starts: channel = %d, txpower level = %u\n", CHANNEL, DEFAULT_TXPOWER_LEVEL);

	unsigned reg;
	reg = getreg(CC2420_IOCFG0);
	printf("IOCFG0 = 0x%04x\n", reg);
	/* set test output signal */
	reg = getreg(CC2420_IOCFG1);
#define CCAMUX_BV (31<<0)
#define SFDMUX_BV (31<<5)
	/* reg = (reg & (~CCAMUX_BV)) | (23<<0); */
	/* setreg(CC2420_IOCFG1, reg); */
	printf("IOCFG1 = 0x%04x\n", reg);
	reg = getreg(CC2420_MDMCTRL0);
	printf("MDMCTRL0 = 0x%04x\n", reg);
	reg = getreg(CC2420_MDMCTRL1);
	printf("MDMCTRL1 = 0x%04x\n", reg);


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
				/* int8_t rssi = cc2420_rssi(); */
				int rssi;
				while(!(status() & BV(CC2420_RSSI_VALID))) {
					/*    printf("cc2420_rssi: RSSI not valid.\n");*/
				}
				rssi = (int)((signed char)getreg(CC2420_RSSI));
				unsigned lna = getreg(CC2420_AGCCTRL) & 0x0003;
				printf("rssi = %d, lna = %u\n", rssi, lna);
				if(!CCA_IS_1) {
					leds_on(LEDS_BLUE);
					clock_delay(200);
					leds_off(LEDS_BLUE);
				}
      }
    } else if(ev == serial_line_event_message) {
      char ch = *(char *)data;
      if(ch == '\0') {
				ch = last_ch;
			}
			last_ch = ch;
			if(ch >= '0' && ch <= '9') {
					cc2420_set_txpower(tx_power_level[ch - '0']);
					printf("tx power = %u\n", cc2420_get_txpower());
      } else if(ch == 'r') {
				int8_t val = cc2420_rssi();
				printf("rssi = %d\n", val);
      } else if(ch == 'n') {
				start_mode((mode + 1) % NUM_MODES);
      } else if(ch == 'p') {
				start_mode((mode - 1) % NUM_MODES);
      } else if(ch == 'e') {
				watchdog_reboot();
      } else if(ch == '+') {
				int chn = cc2420_get_channel();
				chn = (chn - 11 + 1) % 16 + 11;
				cc2420_set_channel(chn);
				printf("channel = %d\n", chn);
      } else if(ch == '-') {
				int chn = cc2420_get_channel();
				chn = (chn - 11 + 15) % 16 + 11;
				cc2420_set_channel(chn);
				printf("channel = %d\n", chn);
			} else if(ch == 'c') {
				reg = getreg(CC2420_IOCFG1);
				unsigned ccamux = (reg & CCAMUX_BV) >> 0;
				ccamux = ccamux==31 ? 0 : ccamux+1;
				reg = (reg & (~CCAMUX_BV)) | (ccamux<<0);
				setreg(CC2420_IOCFG1, reg);
				printf("IOCFG1 = 0x%04x\n", reg);
			} else if(ch == 's') {
				reg = getreg(CC2420_IOCFG1);
				unsigned sfdmux = (reg & SFDMUX_BV) >> 5;
				sfdmux = sfdmux==31 ? 0 : sfdmux+1;
				reg = (reg & (~SFDMUX_BV)) | (sfdmux<<5);
				setreg(CC2420_IOCFG1, reg);
				printf("SFDMUX = %02hu\n", sfdmux);
			} else if(ch =='w') {
					printf("tx power = %u\n", cc2420_get_txpower());
			} else {
				printf("invalid input\n");
      }
    } else if(ev == sensors_event && data == &button_sensor) {
      start_mode((mode + 1) % NUM_MODES);
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
