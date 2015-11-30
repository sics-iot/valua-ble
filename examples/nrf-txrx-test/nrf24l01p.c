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
 *         A basic device driver for nRF24l01+ radio
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
#include "nrf24l01p.h"

/* modes */
#define PD 0
#define RX 1
#define TX 2

PROCESS_NAME(txrx_process);

static uint8_t tx_addr[5] = {0x55, 0xAA, 0x56, 0xAB, 0x5A};
static uint8_t rx_addr_p0[5] = {0x55, 0xAA, 0x56, 0xAB, 0x5A};
struct radio_driver *radio;

unsigned
getreg(unsigned addr)
{
	return csi00_read((uint8_t)addr);
}

void
setreg(unsigned addr, unsigned val)
{
	csi00_write(0x20 | (uint8_t)addr, (uint8_t)val);
}

static int
nrf_init(void)
{
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

	return 0;
}

static int
nrf_send(const void *payload, unsigned short len)
{
	csi00_write_message(W_TX_PAYLOAD_NO_ACK, (uint8_t *)payload, len);
	CE = 1; // activate RX/TX
	/* clock_delay_usec(10); */
	/* CE = 0; */
	return RADIO_TX_OK;
}

static int
nrf_read(void *buf, unsigned short buf_len)
{
	uint8_t payload_len = getreg(R_RX_PL_WID);

	return csi00_read_message(R_RX_PAYLOAD, buf, payload_len);
}

static int
nrf_on(void)
{
	(void)csi00_strobe(FLUSH_RX);
	setreg(CONFIG,
	            (MASK_TX_DS|MASK_MAX_RT|PWR_UP|PRIM_RX)&(~MASK_RX_DR));
	CE = 1;
	return 0;
}

static int
nrf_off(void)
{
	setreg(CONFIG,
	            (MASK_RX_DR|MASK_TX_DS|MASK_MAX_RT|PRIM_RX)&(~PWR_UP));
	return 0;
}

/* Contiki radio driver struct for nRF */
struct radio_driver nrf24l01p_driver = {
	nrf_init,
	NULL, // int (* prepare)(const void *payload, unsigned short payload_len);
	NULL, //  int (* transmit)(unsigned short transmit_len);
	nrf_send,
	nrf_read,
	NULL, // int (* channel_clear)(void);
	NULL, // int (* receiving_packet)(void);
	NULL, // int (* pending_packet)(void);
	nrf_on,
	nrf_off,
	NULL, // radio_result_t (* get_value)(radio_param_t param, radio_value_t *value);
	NULL, // radio_result_t (* set_value)(radio_param_t param, radio_value_t value);
	NULL, // radio_result_t (* get_object)(radio_param_t param, void *dest, size_t size);
	NULL, // radio_result_t (* set_object)(radio_param_t param, const void *src, size_t size);
};

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
/** @} */
