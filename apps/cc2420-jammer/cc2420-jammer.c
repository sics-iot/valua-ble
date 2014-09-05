/*
 * Copyright (c) 2014, Swedish Institute of Computer Science.
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
 *         Channel jammer program for CC2420
 * \author
 *         Zhitao He <zhitao@sics.se>
 */

#include "dev/button-sensor.h"
#include "contiki.h"
#include "net/rime.h"
#include "dev/leds.h"
#include "dev/serial-line.h"
#include "dev/spi.h"
#include "dev/cc2420.h"
#include "dev/cc2420_const.h"
#include "node-id.h"
#include "lib/random.h"
#include "dev/radio.h"
#include "dev/watchdog.h"
#include "cc2420-jammer.h"
#include "commands.h"

#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#define DEBUG 1
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#define BUSYWAIT_UNTIL(cond, max_time)                                  \
  do {                                                                  \
    rtimer_clock_t t0;                                                  \
    t0 = RTIMER_NOW();                                                  \
    while(!(cond) && RTIMER_CLOCK_LT(RTIMER_NOW(), t0 + (max_time)));   \
  } while(0)

#define CHANNEL 20
#define TXPOWER_LEVEL 7

#define TX_INTERVAL (CLOCK_SECOND / 32)
#define MAX_TX_PACKETS 10
#define PAYLOAD_LEN 20
/* exprimental value used to occupy near full bandwidth, assuming RIMTER_SECOND = 4096 * N */
/* "SFD gap" = 298 us out of 4395 us Droplet interval => actual free air time = 298 - 160 (preamble+SFD) = 138 us => free bandwidth = 138/4395 = 3.1% */
/* #define DEFAULT_RTIMER_INTERVAL (RTIMER_SECOND / 128) */
#define RTIMER_INTERVAL ((RTIMER_SECOND + (225/2)) / 225 )

#define AUTOACK (1 << 4)
#define ADR_DECODE (1 << 11)
#define CCAMUX_BV (31<<0)
#define SFDMUX_BV (31<<5)
#define AUTOCRC (1<<5)
#define FIFOP_THR_MSB 6
#define FIFOP_THR_LSB 0
#define RX_MODE_BV (3<<0)
#define RX_MODE_1 (1<<0)
#define RX_MODE_2 (2<<0)
#define RX_MODE_3 (3<<0)
#define TX_MODE_BV (3<<2)
#define TX_MODE_1 (1<<2) // Tx mode 1: serial mode
#define TX_MODE_2 (2<<2)
#define TX_MODE_3 (3<<2)

int mode;

static clock_time_t tx_interval = TX_INTERVAL;
static unsigned max_tx_packets = MAX_TX_PACKETS;
static int payload_len = PAYLOAD_LEN;
static rtimer_clock_t rtimer_interval = RTIMER_INTERVAL;

static struct etimer et;
static struct rtimer rt;
static uint16_t seqno;

/* A byte sequence of certain length */
struct hex_seq
{
	const uint8_t *data;
	const size_t size;
};

// No. zero preamble nimbles should be equal or greater than target network's SYNWORD setting.
const static uint8_t hex_seq_1[] = {127, 1, 0x00, 0xA7};
const static uint8_t hex_seq_2[] = {127, 1, 0x00, 0x00, 0xA7};
const static uint8_t hex_seq_3[] = {127, 1, 0x06, 0xA7};

const static struct hex_seq droplets[] =	{
	{hex_seq_1, sizeof(hex_seq_1)},
	{hex_seq_2, sizeof(hex_seq_2)},
	{hex_seq_3, sizeof(hex_seq_3)}
};
static int droplet_index;
static uint8_t txfifo_data[128];

const unsigned char tx_power_level[10] = {0,1,3,7,11,15,19,23,27,31};
struct variable const user_variable_list[] = {
	{'l', (union number*)&len_hdr, sizeof(len_hdr), "len_hdr", 0, 127},
	{'t', (union number*)&tx_interval, sizeof(tx_interval), "tx_interval", 0, (unsigned)~0},
	{'r', (union number*)&rtimer_interval, sizeof(rtimer_interval), "rtimer_interval", 0, (unsigned)~0},
	{'y', (union number*)&payload_len, sizeof(payload_len), "payload_len", 0, 127},
	{'p', (union number*)&max_tx_packets, sizeof(max_tx_packets), "max_tx_packets", 0, (unsigned)~0},
	/* {'h', (union number*)&hex_seq[0], 1, "hex_seq[0]", 0, 127}, */
	{'h', (union number*)&droplet_index, 2, "droplet_index", 0, sizeof(droplets)/sizeof(struct hex_seq)-1},
	{'0', NULL, 0, NULL, -1, -1},
};

PROCESS(test_process, "CC2420 jammer");
AUTOSTART_PROCESSES(&test_process);

/*---------------------------------------------------------------------------*/
static void
pad(uint8_t* dst, size_t dst_len, const uint8_t* src, const size_t src_len, void (*update_src)(uint8_t *src))
{
	int i;
	uint8_t tmp[src_len];

	memcpy(tmp, src, src_len);
	for(i = 0;i < dst_len - dst_len % src_len;i+=src_len) {
		memcpy(dst+i, tmp, src_len);
		if(update_src) {
			update_src(tmp);
		}
	}
	memcpy(dst+i, src, dst_len % src_len);
}

/*---------------------------------------------------------------------------*/
static void
print_hex_seq(const char *prefix, const uint8_t *data, size_t size)
{
	int i;
	char dbuf[size*2+1];
	char *dbuf_ptr = dbuf;
	for(i = 0;i < size;++i){
		dbuf_ptr += snprintf(dbuf_ptr, sizeof(dbuf), "%02x", data[i]);
	}
	printf("%s%s\n", prefix, dbuf);
}

/*---------------------------------------------------------------------------*/
/* CC2420 Serial TX mode */
void
send_len(struct rtimer *t, void *ptr)
{
		if (ptr) {
			/* schedule next send */
			rtimer_set(&rt, RTIMER_NOW() + RTIMER_SECOND / 2, 0, send_len, (void *)1);
		
			/* Test: set first transmitted bit = 1 */
			CC2420_FIFO_PORT(OUT) |= BV(CC2420_FIFO_PIN);
			/* Start serial data transmission */
			CC2420_CLEAR_FIFOP_INT();
			CC2420_ENABLE_FIFOP_INT();
			strobe(CC2420_STXON);
		}
}
/*---------------------------------------------------------------------------*/
/* Periodic Droplet transmission */
void
send_len_buf(struct rtimer *t, void *ptr)
{
	if (ptr) {
		/* Write empty frame to TX FIFO. */
		// frame length is appended by a bogus byte that induces a minimal delay to the power-down of the TX circuit, thus avoiding corruption of transmitted frame length.
		static uint8_t total_len[2] = {127, 0xA5};
		total_len[0] = len_hdr;
		CC2420_WRITE_FIFO_BUF(&total_len, 2);

		/* Transmit */
		strobe(CC2420_STXON);

		/* We wait until underflow has occured, which should take at least
			 tx touraround time (128 us) + 4 preamble bytes + SFD byte (5*32=160 us) = 288 us */
		BUSYWAIT_UNTIL((status() & BV(CC2420_TX_UNDERFLOW)), RTIMER_SECOND / 2000);

		/* Flush TX FIFO */
		strobe(CC2420_SFLUSHTX);

		/* schedule next send */
		/* rtimer_set(&rt, RTIMER_NOW() + rtimer_interval, 0, send_len_buf, (void *)1); */
		/* if (rt.clock + rtimer_interval - RTIMER_NOW() > 0) { */
			rtimer_set(&rt, rt.time + rtimer_interval, 0, send_len_buf, (void *)1);
		/* } else { */
		/* 	rtimer_set(&rt, RTIMER_NOW() + rtimer_interval, 0, send_len_buf, (void *)1); */
		/* } */
	}
}
/*---------------------------------------------------------------------------*/
/* Reset transmitter to normal packet mode */
void
reset_transmitter(void)
{
  unsigned reg;

  /* set normal TX mode 0, normal RX mode 0 */
  reg = getreg(CC2420_MDMCTRL1);
	reg &= ~(TX_MODE_BV | RX_MODE_BV);
  setreg(CC2420_MDMCTRL1, reg);
	/* printf("0x%04x\n", reg); */

  /* set TX DAC's data source to modulator (normal operation) */
  reg = getreg(CC2420_DACTST);
  setreg(CC2420_DACTST, reg & 0x0FFF);

	/* enable AUTOCRC */
	reg = getreg(CC2420_MDMCTRL0);
	reg |= AUTOCRC;
	setreg(CC2420_MDMCTRL0, reg);

	/* reset FIFOP threshold to maximum */
		reg = getreg(CC2420_IOCFG0);
		setreg(CC2420_IOCFG0, SETFD(reg, 127, FIFOP_THR_MSB, FIFOP_THR_LSB));

		/* Turn off autoack */
		reg = getreg(CC2420_MDMCTRL0);
		reg &= ~(AUTOACK | ADR_DECODE);
		setreg(CC2420_MDMCTRL0, reg);

  /* enter RX mode */
  /* strobe(CC2420_SRXON); */
	strobe(CC2420_SRFOFF);
}
/*---------------------------------------------------------------------------*/
/* Transmit a continuous carrier */
void
send_carrier(int mode)
{
  unsigned reg = getreg(CC2420_MDMCTRL1);
  if(mode == UNMOD) {
		/* unmodulated carrier */
		reg &= ~TX_MODE_BV;
		reg |= TX_MODE_3;
    setreg(CC2420_MDMCTRL1, reg);
    setreg(CC2420_DACTST, 0x1800);
  } else if(mode == MOD) {
		/* randomly modulated carrier */
		reg &= ~TX_MODE_BV;
		reg |= TX_MODE_3;
    setreg(CC2420_MDMCTRL1, reg);
  } else if(mode == DRIZZLE) {
		reg &= ~TX_MODE_BV;
		reg |= TX_MODE_2;
    setreg(CC2420_MDMCTRL1, reg);
	}
	strobe(CC2420_STXON);
}
/*---------------------------------------------------------------------------*/
static void
inc_first_byte(uint8_t *src)
{
	(*(src+1))++;
}
/*---------------------------------------------------------------------------*/
static void
stop_rtimer(int new_mode)
{
	rt.ptr = NULL; 		// stop rtimer
}

/*---------------------------------------------------------------------------*/
static void
rx_mode(int new_mode)
{
	rimestats.llrx = 0;
	rimestats.badcrc = 0; 
	rimestats.badsynch = 0; 
	rimestats.toolong = 0;
	sum_lqi = 0;
	sum_rssi = 0;

	CC2420_CLEAR_FIFOP_INT();
	CC2420_ENABLE_FIFOP_INT();
	flushrx();
	strobe(CC2420_SRXON);
}

static void
off_mode(int new_mode)
{
	strobe(CC2420_SRFOFF);
}

static void
cs_mode(int new_mode)
{
	strobe(CC2420_SRXON);
	etimer_set(&et, CLOCK_SECOND / 1);
}

static void
unmod_mode(int new_mode)
{
	send_carrier(new_mode);
}

static void
mod_mode(int new_mode)
{
	etimer_set(&et, max_tx_packets * tx_interval + CLOCK_SECOND);
	send_carrier(new_mode);
}

static void
tx_mode(int new_mode)
{
	unsigned reg;
	seqno = 0;
	/* CC2420_DISABLE_FIFOP_INT(); // disable cc2420 interrupt */
	/* Turn off automatic packet acknowledgment and address decoding. */
	reg = getreg(CC2420_MDMCTRL0);
	reg &= ~(AUTOACK | ADR_DECODE);
	strobe(CC2420_SRXON);

	etimer_set(&et, tx_interval);
}

static void
tx2_mode(int new_mode)
{
	unsigned reg;
	seqno = 0;
	/* Turn on automatic packet acknowledgment and address decoding. */
	reg = getreg(CC2420_MDMCTRL0);
	reg |= AUTOACK | ADR_DECODE;
	setreg(CC2420_MDMCTRL0, reg);
	strobe(CC2420_SRXON);

	etimer_set(&et, tx_interval);
}

static void
droplet_mode(int new_mode)
{
	CC2420_DISABLE_FIFOP_INT(); // disable cc2420 interrupt
	// Test: CCA interrupt might cause extra delay
	CC2420_DISABLE_CCA_INT();
	CC2420_CLEAR_CCA_INT();

	etimer_set(&et, max_tx_packets * tx_interval + CLOCK_SECOND / 2);
	/* Start sending Droplets */
	// add a small jitter (+/-122 us, assuming RTIMER_SECOND = 32768) around the average interval to randomize phase
	rtimer_set(&rt, RTIMER_NOW() + rtimer_interval - 4 + (random_rand()%9) , 0, send_len_buf, (void *)1);
}

static void
drizzle_mode(int new_mode)
{
	// TEST: send Drizzle with wrong synch header, in order to create attack solely by including sending Droplet headers in the payload
	setreg(CC2420_SYNCWORD, 0xCD0F);
	printf("SYNCWORD=%0x02x\n", getreg(CC2420_SYNCWORD));
	
	pad(txfifo_data, sizeof(txfifo_data), droplets[droplet_index].data, droplets[droplet_index].size, inc_first_byte);

	// Debug print
	print_hex_seq("Droplet header: ", droplets[droplet_index].data, droplets[droplet_index].size);
	print_hex_seq("TXFIFO: ", txfifo_data, sizeof(txfifo_data));

	CC2420_WRITE_FIFO_BUF(txfifo_data, 128);
	etimer_set(&et, max_tx_packets * tx_interval + CLOCK_SECOND);
	send_carrier(mode);
}

/* Reactive jamming */
static void
jam_mode(int new_mode)
{
	unsigned reg;
	jam_ena = 0;
	CC2420_DISABLE_CCA_INT();
	CC2420_CLEAR_CCA_INT();
	// clear any jam data in TX
	strobe(CC2420_SFLUSHTX);
	/* // clear buffer */
	/* packetbuf_clear(); */
	/* CC2420_DISABLE_FIFOP_INT(); // disable cc2420 "packet data received" interrupt */
	/* CC2420_CLEAR_FIFOP_INT(); */
	flushrx();
#if ENABLE_CCA_INTERRUPT
	/* CC2420_CLEAR_CCA_INT(); */
	/* CC2420_ENABLE_CCA_INT(); // enable cc2420 "packet header detected" interrupt */
#endif
	jam_ena = 1;

	/* TEST: turn off AUTOCRC in order to send frame headers as normal packets */
	reg = getreg(CC2420_MDMCTRL0);
	reg &= ~AUTOCRC;
	setreg(CC2420_MDMCTRL0, reg);

	/* Raise FIFOP interrupt immediately after the length byte, i.e. the 1st byte into RXFIFO, is received */
	reg = getreg(CC2420_IOCFG0);
	setreg(CC2420_IOCFG0, SETFD(reg, 1, FIFOP_THR_MSB, FIFOP_THR_LSB));

	/* Pre-fill TX FIFO for next jam */
	CC2420_WRITE_FIFO_BUF(&jam_data, sizeof(jam_data));

	strobe(CC2420_SRXON);

	PRINTF("CCA interrupt enabled = %s\n", CC2420_CCA_PORT(IE) & BV(CC2420_CCA_PIN) ? "true":"false");
	PRINTF("CCA inerrtupt on falling edge = %s\n", CC2420_CCA_PORT(IES) & BV(CC2420_CCA_PIN) ? "true":"false");
}

static void
tx_eth(void)
{
	int i;
	int errno;
	uint16_t buf[127/2+1];
	uint8_t *buf_ptr = (uint8_t *)buf;

	if(seqno < max_tx_packets) {
		etimer_set(&et, tx_interval / 2 + random_rand() % tx_interval);

		/* int i; */
		/* for(i = 0;i < payload_len;++i) { */
		/* 	buf_ptr[i] = (uint8_t)random_rand(); */
		/* } */
		/* snprintf((char *)buf, sizeof(buf), "%u", seqno); */

		/* Test: fill payload with SYNC header */
#define REVERSE_PHASE(octet)	( ((((octet>>4)+8)%16)<<4) + ((octet&0x0f)+8)%16 )
#define MOD_MODE_BIT 4
#define MOD_MODE_BV (1<<MOD_MODE_BIT)
		uint8_t sync_hdr[] = {0x00, 0xA7, 0x7F}; // RP: 0x88, 0x2F, 0xF7
		// In case in RP mode, reverse SYNC header back to normal mode
		if (getreg(CC2420_MDMCTRL1) & MOD_MODE_BV) {
			for (i = 0;i < sizeof(sync_hdr);i++) {
				sync_hdr[i] = REVERSE_PHASE(sync_hdr[i]);
			}
		};
		memcpy(buf_ptr+payload_len-sizeof(sync_hdr), &sync_hdr[0], sizeof(sync_hdr));
		/* pad(buf_ptr, payload_len, sync_hdr, sizeof(sync_hdr), NULL); */ // pad payload with SYNC headers
		snprintf((char *)buf, sizeof(buf), "%u", seqno);

		errno = cc2420_driver.send(buf, payload_len);
		if (errno == RADIO_TX_OK) printf(". ");
		else printf("cc2420_send error: %d\n", errno);
		++seqno;
	} else {
		etimer_stop(&et);
		printf("Finished sending %u packets\n", seqno);
	}
}

static void
tx2_eth(void)
{
	int i;
	int errno;
	uint16_t buf[127/2+1];
	uint8_t *buf_ptr = (uint8_t *)buf;

	if(seqno < max_tx_packets) {
		etimer_set(&et, tx_interval / 2 + random_rand() % tx_interval);

#define MHR_LEN 7 // 16-bit FCF + 8-bit SEQ + 16-bit PAN + 16-bit DST
		// FCF: frame type Data, ack req = 1, dst add mode = 2, frame ver = 1 src add mode = 0
		buf[0] = 0x01<<0 | 0<<3 | 0<<4 | 1<<5 | 0<<6 | 0x0<<7 | 2<<10 | 1<<12 | 0<<14;
		buf_ptr[2] = (uint8_t)(seqno & 0x00FF);
		buf_ptr[3] = 0xFF;
		buf_ptr[4] = 0xFF;
		buf_ptr[5] = 0x02; // dest. rimeaddr lower byte
		buf_ptr[6] = 0x00; // dest. rimeaddr higher byte
		for(i = MHR_LEN;i < payload_len;++i) {
			buf_ptr[i] = (uint8_t)random_rand();
		}
		/* snprintf((char *)buf, sizeof(buf), "%u", seqno); */

		errno = cc2420_driver.send(buf, payload_len);
		if (errno == RADIO_TX_OK) printf(". ");
		else	printf("cc2420_send error: %d\n", errno);
		++seqno;
	} else {
		etimer_stop(&et);
		printf("Finished sending %u packets\n", seqno);
	}
}

static void
cs_eth(void)
{
	int rssi = -127;
	unsigned lna = 0;
	/* BUSYWAIT_UNTIL(status() & BV(CC2420_RSSI_VALID), RTIMER_SECOND / 100); */
	if (status() & BV(CC2420_RSSI_VALID)) {
		rssi = (int)((signed char)getreg(CC2420_RSSI));
		lna = getreg(CC2420_AGCCTRL) & 0x0003;
	}
	printf("rssi = %d, lna = %u\n", rssi, lna);
}

static void
attack_eth(void)
{
	etimer_stop(&et);
	rt.ptr = NULL; 		// stop rtimer
	reset_transmitter();
	printf("Finished transmission after %lu seconds\n",
				 (et.timer.interval + CLOCK_SECOND/2) / CLOCK_SECOND);
}

struct mode {
	int mode;
	const char *display;
	void (*handler)(int mode);
	void (*prelog)(int mode);
	void (*prolog)(int mode);
	void (*et_handler)(void);
};

const static struct mode mode_list[] = {
	{RX, "RX", rx_mode, NULL, NULL, NULL},
	{OFF, "OFF", off_mode, NULL, NULL, NULL},
	{CH, "Channel sampling", cs_mode, NULL, NULL, cs_eth},
	{UNMOD, "Unmodulated carrier", unmod_mode, stop_rtimer, NULL, NULL},
	{MOD, "Modulated carrier", mod_mode, stop_rtimer, NULL, attack_eth},
	{TX, "TX broadcast", tx_mode, stop_rtimer, NULL, tx_eth},
	{TX2, "TX unicast", tx2_mode, stop_rtimer, NULL, tx2_eth},
	{DRIZZLE, "Drizzle", drizzle_mode, stop_rtimer, NULL, attack_eth},
	{DROPLET, "Droplet", droplet_mode, stop_rtimer, NULL, attack_eth},
	{JAM, "Reactive jamming", jam_mode, NULL, NULL, NULL},
	{-1, NULL, NULL, NULL, NULL, NULL}
};

void
start_mode(int new_mode)
{
	static int last_mode;
	const struct mode *p;
	for (p=&mode_list[0];p->handler;p++) {
		if(p->mode == new_mode) {
			if(p->prelog) p->prelog(last_mode);
			reset_transmitter();
			p->handler(new_mode);
			last_mode = mode;
			mode = new_mode;
			if(p->prolog) p->prolog(mode);
			printf("%s mode\n", p->display);
			return;
		}
	}
	printf("Unknown mode\n");
}

/*-----------------------------------------------------------------------------*/
/* Etimer timeout handler */
static void
et_handler(void)
{
	const struct mode *p;

	etimer_reset(&et);
	for (p=&mode_list[0];p->handler;p++) {
		if(p->mode == mode) {
			if (p->et_handler) p->et_handler();
			else return;
		}
	}
}

/*---------------------------------------------------------------------------*/
// radio receiver callback
/* static uint16_t rxbuf[64]; */
/* static const uint8_t *rxbuf_ptr = (uint8_t *)rxbuf; */

void
cc2420_set_receiver(void (*f)(const struct radio_driver *));
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(test_process, ev, data)
{
  PROCESS_BEGIN();

	PROCESS_PAUSE();

	// debug prints
	printf("F_CPU %lu CLOCK_CONF_SECOND %lu RTIMER_CONF_SECOND %u\n", F_CPU, CLOCK_CONF_SECOND, RTIMER_SECOND);

	/* pre-fill pseudo random data for TXFIFO */
	pad(txfifo_data, sizeof(txfifo_data), droplets[droplet_index].data, droplets[droplet_index].size, inc_first_byte);
	print_hex_seq("TXFIFO: ", txfifo_data, sizeof(txfifo_data));

	// change MAC address
	/* node_id_burn(100); */
	unsigned shortaddr = 1;
	CC2420_WRITE_RAM(&shortaddr,CC2420RAM_SHORTADDR, 2);
	CC2420_READ_RAM(&shortaddr,CC2420RAM_SHORTADDR, 2);
	printf("shortaddr: %u\n", shortaddr);

	commands_set_callback(start_mode);

	button_sensor.configure(SENSORS_ACTIVE, 1);

	/* initialize GPIO pins */
	GPIO1_PORT(DIR) |= BV(GPIO1_PIN);
	GPIO1_PORT(OUT) &= ~BV(GPIO1_PIN);
	GPIO1_PORT(OUT) |= BV(GPIO1_PIN);
	GPIO1_PORT(OUT) &= ~BV(GPIO1_PIN);
	GPIO2_PORT(DIR) |= BV(GPIO2_PIN);
	GPIO2_PORT(OUT) &= ~BV(GPIO2_PIN);
	GPIO2_PORT(OUT) |= BV(GPIO2_PIN);
	GPIO2_PORT(OUT) &= ~BV(GPIO2_PIN);

	/* Change channel */
  cc2420_set_channel(CHANNEL);
	cc2420_set_txpower(TXPOWER_LEVEL);

	etimer_set(&et, CLOCK_SECOND *1);
	PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

  printf("Sampling process starts: channel = %d, txpower level = %u\n", CHANNEL, TXPOWER_LEVEL);

  /* initialize transceiver mode */
  start_mode(0);

	unsigned reg;
	reg = getreg(CC2420_IOCFG0);
	printf("IOCFG0 = 0x%04x\n", reg);
	/* set test output signal */
	reg = getreg(CC2420_IOCFG1);
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
			et_handler();
    } else if(ev == serial_line_event_message) {
			/* Command handlers */
			do_command((char *)data);
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

