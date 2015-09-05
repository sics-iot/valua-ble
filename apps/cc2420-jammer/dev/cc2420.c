/*
 * Copyright (c) 2007, Swedish Institute of Computer Science
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
 * @(#)$Id: cc2420.c,v 1.37 2010/01/14 23:32:05 adamdunkels Exp $
 */
/*
 * This code is almost device independent and should be easy to port.
 */

#include <stdio.h>
#include <string.h>

#include "contiki.h"

#if defined(__AVR__)
#include <avr/io.h>
#endif

#include "dev/leds.h"
#include "dev/spi.h"
#include "cc2420.h"
#include "dev/cc2420/cc2420_const.h"

#include "net/packetbuf.h"
#include "net/rime/rimestats.h"
#include "net/netstack.h"

/* #include "cc2420-jammer.h" */

#define WITH_SEND_CCA 0


#define FOOTER_LEN 2

#ifndef CC2420_CONF_CHECKSUM
#define CC2420_CONF_CHECKSUM 0
#endif /* CC2420_CONF_CHECKSUM */

#ifndef CC2420_CONF_AUTOACK
#define CC2420_CONF_AUTOACK 0
#endif /* CC2420_CONF_AUTOACK */

#if CC2420_CONF_CHECKSUM
#include "lib/crc16.h"
#define CHECKSUM_LEN 2
#else
#define CHECKSUM_LEN 0
#endif /* CC2420_CONF_CHECKSUM */

#define AUX_LEN (CHECKSUM_LEN + FOOTER_LEN)


#define FOOTER1_CRC_OK      0x80
#define FOOTER1_CORRELATION 0x7f

#define DEBUG 0
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...) do {} while (0)
#endif

#define DEBUG_LEDS DEBUG
#undef LEDS_ON
#undef LEDS_OFF
#if DEBUG_LEDS
#define LEDS_ON(x) leds_on(x)
#define LEDS_OFF(x) leds_off(x)
#else
#define LEDS_ON(x)
#define LEDS_OFF(x)
#endif

void cc2420_arch_init(void);

/* XXX hack: these will be made as Chameleon packet attributes */
rtimer_clock_t cc2420_time_of_arrival, cc2420_time_of_departure;

int cc2420_authority_level_of_sender;

int cc2420_packets_seen, cc2420_packets_read;

volatile int jam_ena;
static void (*rx_callback)(void);

static uint8_t volatile pending;

#define BUSYWAIT_UNTIL(cond, max_time)                                  \
  do {                                                                  \
    rtimer_clock_t t0;                                                  \
    t0 = RTIMER_NOW();                                                  \
    while(!(cond) && RTIMER_CLOCK_LT(RTIMER_NOW(), t0 + (max_time)));   \
  } while(0)

#define FASTSPI_READ_FIFO_GARBAGE(c)\
	 do {\
	 uint8_t dummy;\
    uint8_t i;                                                          \
    CC2420_SPI_ENABLE();                                                \
    SPI_WRITE(CC2420_RXFIFO | 0x40);                                    \
    (void)SPI_RXBUF;                                                    \
    for(i = 0; i < (c); i++) {                                      \
      SPI_READ(dummy);                               \
    }                                                                   \
    clock_delay(1);                                                     \
    CC2420_SPI_DISABLE();                                               \
	 } while (0)

volatile uint8_t cc2420_sfd_counter;
volatile uint16_t cc2420_sfd_start_time;
volatile uint16_t cc2420_sfd_end_time;

static volatile uint16_t last_packet_timestamp;
/*---------------------------------------------------------------------------*/
PROCESS(cc2420_process, "CC2420 driver");
PROCESS(cc2420_debug_process, "CC2420 debug print");
/*---------------------------------------------------------------------------*/


int cc2420_on(void);
int cc2420_off(void);

static int cc2420_read(void *buf, unsigned short bufsize);

static int cc2420_prepare(const void *data, unsigned short len);
static int cc2420_transmit(unsigned short len);
static int cc2420_send(const void *data, unsigned short len);

static int cc2420_receiving_packet(void);
static int pending_packet(void);
static int cc2420_cca(void);
/*static int detected_energy(void);*/

signed char cc2420_last_rssi;
uint8_t cc2420_last_correlation;

const struct radio_driver cc2420_driver =
  {
    cc2420_init,
    cc2420_prepare,
    cc2420_transmit,
    cc2420_send,
    cc2420_read,
    /* cc2420_set_channel, */
    /* detected_energy, */
    cc2420_cca,
    cc2420_receiving_packet,
    pending_packet,
    cc2420_on,
    cc2420_off,
  };

static uint8_t receive_on;

static int channel;

/*---------------------------------------------------------------------------*/

static void
getrxdata(void *buf, int len)
{
  CC2420_READ_FIFO_BUF(buf, len);
}
static void
getrxbyte(uint8_t *byte)
{
  CC2420_READ_FIFO_BYTE(*byte);
}
void
flushrx(void)
{
  uint8_t dummy;

  CC2420_READ_FIFO_BYTE(dummy);
  CC2420_STROBE(CC2420_SFLUSHRX);
  CC2420_STROBE(CC2420_SFLUSHRX);
}
/*---------------------------------------------------------------------------*/
void
strobe(enum cc2420_register regname)
{
  CC2420_STROBE(regname);
}
/*---------------------------------------------------------------------------*/
unsigned int
status(void)
{
  uint8_t status;
  CC2420_GET_STATUS(status);
  return status;
}
/*---------------------------------------------------------------------------*/
static uint8_t locked, lock_on, lock_off;

static void
on(void)
{
  CC2420_ENABLE_FIFOP_INT();
  strobe(CC2420_SRXON);

  BUSYWAIT_UNTIL(status() & (BV(CC2420_XOSC16M_STABLE)), RTIMER_SECOND / 100);

  ENERGEST_ON(ENERGEST_TYPE_LISTEN);
  receive_on = 1;
}
static void
off(void)
{
  /*  PRINTF("off\n");*/
  receive_on = 0;

  /* Wait for transmission to end before turning radio off. */
  BUSYWAIT_UNTIL(!(status() & BV(CC2420_TX_ACTIVE)), RTIMER_SECOND / 10);

  ENERGEST_OFF(ENERGEST_TYPE_LISTEN);
  strobe(CC2420_SRFOFF);
  CC2420_DISABLE_FIFOP_INT();

  if(!CC2420_FIFOP_IS_1) {
    flushrx();
  }
}
/*---------------------------------------------------------------------------*/
#define GET_LOCK() locked++
static void RELEASE_LOCK(void) {
  if(locked == 1) {
    if(lock_on) {
      on();
      lock_on = 0;
    }
    if(lock_off) {
      off();
      lock_off = 0;
    }
  }
  locked--;
}
/*---------------------------------------------------------------------------*/
static unsigned
getreg(enum cc2420_register regname)
{
  unsigned reg;
  CC2420_READ_REG(regname, reg);
  return reg;
}
/*---------------------------------------------------------------------------*/
static void
setreg(enum cc2420_register regname, unsigned value)
{
  CC2420_WRITE_REG(regname, value);
}
/*---------------------------------------------------------------------------*/
static void
set_txpower(uint8_t power)
{
  uint16_t reg;

  reg = getreg(CC2420_TXCTRL);
  reg = (reg & 0xffe0) | (power & 0x1f);
  setreg(CC2420_TXCTRL, reg);
}
/*---------------------------------------------------------------------------*/
#define AUTOACK (1 << 4)
#define ADR_DECODE (1 << 11)
#define RXFIFO_PROTECTION (1 << 9)
#define CORR_THR(n) (((n) & 0x1f) << 6)
#define FIFOP_THR(n) ((n) & 0x7f)
#define RXBPF_LOCUR (1 << 13);
/*---------------------------------------------------------------------------*/
int
cc2420_init(void)
{
  uint16_t reg;
  {
    int s = splhigh();
    cc2420_arch_init();		/* Initalize ports and SPI. */
    CC2420_DISABLE_FIFOP_INT();
    CC2420_FIFOP_INT_INIT();
    splx(s);
  }

  /* Turn on voltage regulator and reset. */
  SET_VREG_ACTIVE();
  clock_delay(250*(F_CPU/(4000000uL)));
  SET_RESET_ACTIVE();
  clock_delay(127*(F_CPU/(4000000uL)));
  SET_RESET_INACTIVE();
  clock_delay(125*(F_CPU/(4000000uL)));


  /* Turn on the crystal oscillator. */
  strobe(CC2420_SXOSCON);

  /* Turn on/off automatic packet acknowledgment and address decoding. */
  reg = getreg(CC2420_MDMCTRL0);

#if CC2420_CONF_AUTOACK
  reg |= AUTOACK | ADR_DECODE;
#else
  reg &= ~(AUTOACK | ADR_DECODE);
#endif /* CC2420_CONF_AUTOACK */

#if ENABLE_CCA_INTERRUPT
#if CCA_TEST
		CC2420_CCA_PORT(IES) &= ~BV(CC2420_CCA_PIN); // set CCA interrupt on rising edge
#else
		CC2420_CCA_PORT(IES) |= BV(CC2420_CCA_PIN); // set CCA interrupt on falling edge
#endif
		CC2420_CLEAR_CCA_INT();
	/* Turn on CCA */
#define CCA_MODE_BV (3<<6);
#define CCA_MODE_1 (1<<6);
#define CCA_MODE_2 (2<<6);
#define CCA_MODE_3 (3<<6);
	reg &= ~CCA_MODE_BV; //clear cca mode
	reg |= CCA_MODE_2; // set cca mode
#endif

	/* Set preamble length */
#define PREAMBLE_LENGTH_BV (15<<0)
	reg &= ~PREAMBLE_LENGTH_BV;
	reg |= (2 & PREAMBLE_LENGTH_BV); // set preamble length to N+1 bytes

  setreg(CC2420_MDMCTRL0, reg);

	/* Set unbuffered RX mode*/
#if ENABLE_UNBUFFERED_MODE
#define RX_MODE_BV (3<<0);
#define RX_MODE_1 (1<<0);
#define RX_MODE_2 (2<<0);
#define RX_MODE_3 (3<<0);
	/* reg = getreg(CC2420_MDMCTRL1); */
	/* reg &= ~RX_MODE_BV; //clear rx mode */
	/* reg |= RX_MODE_1; // set rx mode */
#endif
  /* Change default values as recomended in the data sheet, */
  /* correlation threshold = 20, RX bandpass filter = 1.3uA. */
  setreg(CC2420_MDMCTRL1, CORR_THR(20));
  reg = getreg(CC2420_RXCTRL1);
  reg |= RXBPF_LOCUR;
  setreg(CC2420_RXCTRL1, reg);

  /* Set the FIFOP threshold to maximum. */
  setreg(CC2420_IOCFG0, FIFOP_THR(127));
  /* Zhitao: Set the FIFOP threshold to PHY + MAC header length */
/* #define HDR_LEN (2+1+2+2+2) // FCS+SEQNO+PANID+DST+SRC */
#define HDR_LEN 0
/*   setreg(CC2420_IOCFG0, FIFOP_THR(1+HDR_LEN-1)); */

  /* Turn off "Security enable" (page 32). */
  reg = getreg(CC2420_SECCTRL0);
  reg &= ~RXFIFO_PROTECTION;
  setreg(CC2420_SECCTRL0, reg);

	/* Set STXON - radio transmission delay to 8 symbol periods (default=12) */
#define TX_TURNAROUND_BV (1<<13)
	reg = getreg(CC2420_TXCTRL);
	reg &= ~TX_TURNAROUND_BV;
	setreg(CC2420_TXCTRL, reg);

  cc2420_set_pan_addr(0xffff, 0x0000, NULL);
  cc2420_set_channel(26);

  flushrx();

  process_start(&cc2420_process, NULL);
  process_start(&cc2420_debug_process, NULL);
  return 1;
}
/*---------------------------------------------------------------------------*/
static int
cc2420_transmit(unsigned short payload_len)
{
  int i, txpower;
  uint8_t total_len;
#if CC2420_CONF_CHECKSUM
  uint16_t checksum;
#endif /* CC2420_CONF_CHECKSUM */

  GET_LOCK();

  txpower = 0;
  if(packetbuf_attr(PACKETBUF_ATTR_RADIO_TXPOWER) > 0) {
    /* Remember the current transmission power */
    txpower = cc2420_get_txpower();
    /* Set the specified transmission power */
    set_txpower(packetbuf_attr(PACKETBUF_ATTR_RADIO_TXPOWER) - 1);
  }


  /* The TX FIFO can only hold one packet. Make sure to not overrun
   * FIFO by waiting for transmission to start here and synchronizing
   * with the CC2420_TX_ACTIVE check in cc2420_send.
   *
   * Note that we may have to wait up to 320 us (20 symbols) before
   * transmission starts.
   */
#ifndef CC2420_CONF_SYMBOL_LOOP_COUNT
#error CC2420_CONF_SYMBOL_LOOP_COUNT needs to be set!!!
#else
#define LOOP_20_SYMBOLS CC2420_CONF_SYMBOL_LOOP_COUNT
#endif

#if WITH_SEND_CCA
  strobe(CC2420_SRXON);
  BUSYWAIT_UNTIL(status() & BV(CC2420_RSSI_VALID), RTIMER_SECOND / 10);
  strobe(CC2420_STXONCCA);
#else /* WITH_SEND_CCA */
  strobe(CC2420_STXON);
#endif /* WITH_SEND_CCA */
  for(i = LOOP_20_SYMBOLS; i > 0; i--) {
    if(CC2420_SFD_IS_1) {
      {
        rtimer_clock_t sfd_timestamp;
        sfd_timestamp = cc2420_sfd_start_time;
        if(packetbuf_attr(PACKETBUF_ATTR_PACKET_TYPE) ==
           PACKETBUF_ATTR_PACKET_TYPE_TIMESTAMP) {
          /* Write timestamp to last two bytes of packet in TXFIFO. */
          CC2420_WRITE_RAM(&sfd_timestamp, CC2420RAM_TXFIFO + payload_len - 1, 2);
        }
      }

      if(!(status() & BV(CC2420_TX_ACTIVE))) {
        /* SFD went high but we are not transmitting. This means that
           we just started receiving a packet, so we drop the
           transmission. */
        RELEASE_LOCK();
        return RADIO_TX_COLLISION;
      }
      if(receive_on) {
	ENERGEST_OFF(ENERGEST_TYPE_LISTEN);
      }
      ENERGEST_ON(ENERGEST_TYPE_TRANSMIT);
      /* We wait until transmission has ended so that we get an
	 accurate measurement of the transmission time.*/
      BUSYWAIT_UNTIL(!(status() & BV(CC2420_TX_ACTIVE)), RTIMER_SECOND / 10);

#ifdef ENERGEST_CONF_LEVELDEVICE_LEVELS
      ENERGEST_OFF_LEVEL(ENERGEST_TYPE_TRANSMIT,cc2420_get_txpower());
#endif
      ENERGEST_OFF(ENERGEST_TYPE_TRANSMIT);
      if(receive_on) {
	ENERGEST_ON(ENERGEST_TYPE_LISTEN);
      } else {
	/* We need to explicitly turn off the radio,
	 * since STXON[CCA] -> TX_ACTIVE -> RX_ACTIVE */
	off();
      }

      if(packetbuf_attr(PACKETBUF_ATTR_RADIO_TXPOWER) > 0) {
        /* Restore the transmission power */
        set_txpower(txpower & 0xff);
      }

      RELEASE_LOCK();

      return RADIO_TX_OK;
    }
  }

  /* If we are using WITH_SEND_CCA, we get here if the packet wasn't
     transmitted because of other channel activity. */
  RIMESTATS_ADD(contentiondrop);
  printf("cc2420: do_send() transmission never started\n");

  if(packetbuf_attr(PACKETBUF_ATTR_RADIO_TXPOWER) > 0) {
    /* Restore the transmission power */
    set_txpower(txpower & 0xff);
  }

  RELEASE_LOCK();
  return RADIO_TX_COLLISION;
}
/*---------------------------------------------------------------------------*/
static int
cc2420_prepare(const void *payload, unsigned short payload_len)
{
  uint8_t total_len;
#if CC2420_CONF_CHECKSUM
  uint16_t checksum;
#endif /* CC2420_CONF_CHECKSUM */
  GET_LOCK();

  printf("cc2420: sending %d bytes\n", payload_len);

  RIMESTATS_ADD(lltx);

  /* Wait for any previous transmission to finish. */
  /*  while(status() & BV(CC2420_TX_ACTIVE));*/

  /* Write packet to TX FIFO. */
  strobe(CC2420_SFLUSHTX);

#if CC2420_CONF_CHECKSUM
  checksum = crc16_data(payload, payload_len, 0);
#endif /* CC2420_CONF_CHECKSUM */

#if ENABLE_FAST_TX==0
  /* total_len = payload_len + AUX_LEN; */
  // test: don't append crc len
  total_len = payload_len;
  CC2420_WRITE_FIFO_BUF(&total_len, 1);
  CC2420_WRITE_FIFO_BUF(payload, payload_len);
#elif ENABLE_FAST_TX==2
#define BLASTSPI_WRITE_FIFO(p,c)\
	do {\
		CC2420_SPI_ENABLE();\
			FASTSPI_TX_ADDR(CC2420_SFLUSHTX);						\
		u8_t i;\
		FASTSPI_TX_ADDR(CC2420_TXFIFO);\
		for (i = 0; i < (c); i++) {\
		    FASTSPI_TX(((u8_t*)(p))[i]);\
		}\
                SPI_WAITFORTx_ENDED();\
								CC2420_SPI_DISABLE();\
    } while (0)

  BLASTSPI_WRITE_FIFO(payload, payload_len);
#elif ENABLE_FAST_TX==1
  CC2420_WRITE_FIFO(payload, payload_len);
#endif

#if CC2420_CONF_CHECKSUM
  CC2420_WRITE_FIFO_BUF(&checksum, CHECKSUM_LEN);
#endif /* CC2420_CONF_CHECKSUM */

  RELEASE_LOCK();
  return 0;
}
/*---------------------------------------------------------------------------*/
static int
cc2420_send(const void *payload, unsigned short payload_len)
{
  cc2420_prepare(payload, payload_len);
  return cc2420_transmit(payload_len);
}
/*---------------------------------------------------------------------------*/
int
cc2420_off(void)
{
  /* Don't do anything if we are already turned off. */
  if(receive_on == 0) {
    return 1;
  }

  /* If we are called when the driver is locked, we indicate that the
     radio should be turned off when the lock is unlocked. */
  if(locked) {
    /*    printf("Off when locked (%d)\n", locked);*/
    lock_off = 1;
    return 1;
  }

  GET_LOCK();
  /* If we are currently receiving a packet (indicated by SFD == 1),
     we don't actually switch the radio off now, but signal that the
     driver should switch off the radio once the packet has been
     received and processed, by setting the 'lock_off' variable. */
  if(status() & BV(CC2420_TX_ACTIVE)) {
    lock_off = 1;
  } else {
    off();
  }
  RELEASE_LOCK();
  return 1;
}
/*---------------------------------------------------------------------------*/
int
cc2420_on(void)
{
  if(receive_on) {
    return 1;
  }
  if(locked) {
    lock_on = 1;
    return 1;
  }

  GET_LOCK();
  on();
  RELEASE_LOCK();
  return 1;
}
/*---------------------------------------------------------------------------*/
int
cc2420_get_channel(void)
{
  return channel;
}
/*---------------------------------------------------------------------------*/
int
cc2420_set_channel(int c)
{
  uint16_t f;

  GET_LOCK();
  /*
   * Subtract the base channel (11), multiply by 5, which is the
   * channel spacing. 357 is 2405-2048 and 0x4000 is LOCK_THR = 1.
   */
  channel = c;

  f = 5 * (c - 11) + 357 + 0x4000;
  /*
   * Writing RAM requires crystal oscillator to be stable.
   */
  BUSYWAIT_UNTIL((status() & (BV(CC2420_XOSC16M_STABLE))), RTIMER_SECOND / 10);

  /* Wait for any transmission to end. */
  BUSYWAIT_UNTIL(!(status() & BV(CC2420_TX_ACTIVE)), RTIMER_SECOND / 10);

  setreg(CC2420_FSCTRL, f);

  /* If we are in receive mode, we issue an SRXON command to ensure
     that the VCO is calibrated. */
  if(receive_on) {
    strobe(CC2420_SRXON);
  }

  RELEASE_LOCK();
  return 1;
}

/*---------------------------------------------------------------------------*/
int
cc2420_set_frequency(uint16_t f)
{
  GET_LOCK();
  /*
   * Writing RAM requires crystal oscillator to be stable.
   */
  BUSYWAIT_UNTIL((status() & (BV(CC2420_XOSC16M_STABLE))), RTIMER_SECOND / 10);

  /* Wait for any transmission to end. */
  BUSYWAIT_UNTIL(!(status() & BV(CC2420_TX_ACTIVE)), RTIMER_SECOND / 10);

  setreg(CC2420_FSCTRL, (f - 2048) + 0x4000);

  /* If we are in receive mode, we issue an SRXON command to ensure
     that the VCO is calibrated. */
  if(receive_on) {
    strobe(CC2420_SRXON);
  }

  RELEASE_LOCK();
  return 1;
}
/*---------------------------------------------------------------------------*/
uint16_t
cc2420_get_frequency(void)
{
	uint16_t f;

  f = 2048 + (getreg(CC2420_FSCTRL) & 0x03FF);

  return f;
}
/*---------------------------------------------------------------------------*/
void
cc2420_set_pan_addr(unsigned pan,
                    unsigned addr,
                    const uint8_t *ieee_addr)
{
  uint16_t f = 0;
  uint8_t tmp[2];

  GET_LOCK();
  
  /*
   * Writing RAM requires crystal oscillator to be stable.
   */
  BUSYWAIT_UNTIL(status() & (BV(CC2420_XOSC16M_STABLE)), RTIMER_SECOND / 10);

  tmp[0] = pan & 0xff;
  tmp[1] = pan >> 8;
  CC2420_WRITE_RAM(&tmp, CC2420RAM_PANID, 2);

  tmp[0] = addr & 0xff;
  tmp[1] = addr >> 8;
  CC2420_WRITE_RAM(&tmp, CC2420RAM_SHORTADDR, 2);
  if(ieee_addr != NULL) {
    uint8_t tmp_addr[8];
    /* LSB first, MSB last for 802.15.4 addresses in CC2420 */
    for (f = 0; f < 8; f++) {
      tmp_addr[7 - f] = ieee_addr[f];
    }
    CC2420_WRITE_RAM(tmp_addr, CC2420RAM_IEEEADDR, 8);
  }
  RELEASE_LOCK();
}
/*---------------------------------------------------------------------------*/
// Sending a Droplet header followed by a short 802.15.4 MAC header segment as jam signal
					// FCF: frame type=2 (ACK), security=0, frame pending=0, ack req=0, panid compression=0, reserved=0
					/* buf[0] = 0x01<<0 | 0<<3 | 0<<4 | 1<<5 | 0<<6 | 0x0<<7 | 2<<10 | 1<<12 | 0<<14; */
	/* LEN + FCF[0] */
/* const uint8_t jam_data[2] = {5, 0x02<<0|0<<3|0<<4|0<<5|0<<6|0x0<<7}; */
// ACK frame for seq=0, manually appended CRC16 (CRC-CCITT (kermit)) http://www.lammertbies.nl/comm/info/crc-calculation.html
/* const uint8_t jam_data[6] = {5, 0x02, 0x00, 0x00, 0xB8, 0xB5}; */
// ACK frame for seq=254, manually appended CRC16 (CRC-CCITT (kermit)) 
const uint8_t jam_data[6] = {5, 0x02, 0x00, 0xFE, 0x49, 0xAB};
// DATA frame for non-present dst|src address fields, seq=254, manually appended CRC16 (CRC-CCITT (kermit)) 
/* const uint8_t jam_data[6] = {5, 0x01, 0x00, 0xFE, 0x2D, 0x44}; */

static void
send_jam(int len)
{
	/* leds_on(LEDS_BLUE); */
	/* /\* unsigned reg = getreg(CC2420_MDMCTRL1); *\/ */
	/* /\* setreg(CC2420_MDMCTRL1, (reg & 0xFFF3) | (3 << 2)); *\/ */
	/* setreg(CC2420_MDMCTRL1, 0x064C); */
	/* strobe(CC2420_STXON); */
	/* /\* clock_delay(255); *\/ */
	/* clock_delay(16*4*F_CPU/1000000); */
  /* /\* setreg(CC2420_MDMCTRL1, reg & 0xFFF0); *\/ */
  /* setreg(CC2420_MDMCTRL1, 0x0640); */
  /* strobe(CC2420_SRXON); */
	/* leds_off(LEDS_BLUE); */

#define TICKS_PER_DELAY 4 // cmp #0; jne; add (-1), r15;
		// to get len bytes delay in us: x = len (bytes) * 8 (bits/byte) *4 (us/bit) = len * 32
		// to get x us delay in ticks: ticks = x * F_CPU (ticks/s) / 1000000 (us/s) = len * 32 * F_CPU/100000
		// to get tic delays in n clock delays: n = ticks / N (ticks/clock_delay) = len * 32 * F_CPU / TICKS_PER_DELAY / 1000000
	clock_delay((len*32+69-53)*((F_CPU/1000000)/TICKS_PER_DELAY));
	asm("nop");asm("nop");asm("nop");asm("nop");

		/* Transmit */
		strobe(CC2420_STXON);

		/* We wait until underflow has occured, which should take at least
			 tx touraround time (128 us) + 4 preamble bytes + SFD byte (5*32=160 us) = 288 us */
		/* BUSYWAIT_UNTIL((status() & BV(CC2420_TX_UNDERFLOW)), RTIMER_SECOND / 2000); */

		/* Flush TX FIFO */
		/* strobe(CC2420_SFLUSHTX); */

		/* TEST: full packet transmitted (no underflow), wait until transmission completed */
		BUSYWAIT_UNTIL(!(status() & BV(CC2420_TX_ACTIVE)), RTIMER_SECOND/1000);
		if (status() & BV(CC2420_TX_UNDERFLOW))
				strobe(CC2420_SFLUSHTX);

		/* Pre-fill TX FIFO for next jam */
		CC2420_WRITE_FIFO_BUF(&jam_data, sizeof(jam_data));
}

int 
decision(long int n, uint8_t len, uint8_t *buf);
/*---------------------------------------------------------------------------*/
/*
 * CCA interrupt
 *
 */
#define CCA_NUM_FSM_STATES 0
#define INFO_STR_NUM 4
/* static unsigned cca_fsm_states[CCA_NUM_FSM_STATES]; */
static char info_str[INFO_STR_NUM][32];
/* static char **info_str_ptr = &info_str[0]; */
static int info_str_index = 0;

volatile uint16_t last_cc2420_cca_interrupt_time;
volatile uint16_t cc2420_cca_interrupt_time;
volatile uint16_t cc2420_cca_interrupt_interval;

#define TEST_INTERVAL 80

int
cc2420_cca_interrupt(void)
{
	/* GPIO1_PORT(OUT) &= ~BV(GPIO1_PIN); */
	/* GPIO1_PORT(OUT) |= BV(GPIO1_PIN); */
	/* GPIO1_PORT(OUT) &= ~BV(GPIO1_PIN); */
	// Temp fix: ignore CCA interrupt caused by self generated serial jamming
	/* if(mode == SERIAL_JAM) { */
	/* 	CC2420_CLEAR_CCA_INT(); */
	/* 	return 1; */
	/* } */
#if CCA_TEST
	CC2420_CLEAR_CCA_INT();
	
	GPIO1_PORT(OUT) &= ~BV(GPIO1_PIN);
	GPIO1_PORT(OUT) |= BV(GPIO1_PIN);
	
	// time stamp
	TBCCTL1 ^= CCIS0;
	cc2420_cca_interrupt_time = TBCCR1;
	/* cc2420_cca_interrupt_interval = cc2420_cca_interrupt_time - last_cc2420_cca_interrupt_time; */
	
	if (TBCCTL1 & COV) {
		GPIO2_PORT(OUT) &= ~BV(GPIO2_PIN);
		GPIO2_PORT(OUT) |= BV(GPIO2_PIN);
		TBCCTL1 &= ~COV;
	}

	/* last_cc2420_cca_interrupt_time = cc2420_cca_interrupt_time; */
	
	GPIO1_PORT(OUT) &= ~BV(GPIO1_PIN);
	return 1;
	
#else
	/* GPIO1_PORT(OUT) &= ~BV(GPIO1_PIN); */
	/* GPIO1_PORT(OUT) |= BV(GPIO1_PIN); */
	/* GPIO1_PORT(OUT) &= ~BV(GPIO1_PIN); */
	
	/* rtimer_clock_t start, end; */
	uint8_t len;
	
	/* while(!CC2420_FIFO_IS_1) {;} // wait till first byte read */
  if(!CC2420_FIFO_IS_1) {
		/* If FIFO is 0, there is no packet in the RXFIFO. */
		// NOTE: delay time depends on previous processing delay in ISR
		/* clock_delay(22*F_CPU/1000000/4); */
		clock_delay(22*(F_CPU/1000000/TICKS_PER_DELAY));
	}
  if(CC2420_FIFO_IS_1) {
		getrxbyte(&len);
	} else {
		flushrx();
		CC2420_CLEAR_CCA_INT();
		GPIO2_PORT(OUT) &= ~BV(GPIO2_PIN);
		GPIO2_PORT(OUT) |= BV(GPIO2_PIN);
		GPIO2_PORT(OUT) &= ~BV(GPIO2_PIN);
		
		return 1;
	}
	
	/* GPIO1_PORT(OUT) &= ~BV(GPIO1_PIN); */
	/* GPIO1_PORT(OUT) |= BV(GPIO1_PIN); */
	/* GPIO1_PORT(OUT) &= ~BV(GPIO1_PIN); */

	/* if(jam_ena && decision(len, NULL) > 0) { */
	if(jam_ena) {// bypass decision to ensure constant latency
		send_jam(len);
		snprintf(info_str[info_str_index], sizeof(info_str), "%lu %u !", clock_seconds(), len);
	} else {
		/* read remaining payload bytes */
		/* clock_delay(32*len/2*3); // 32 us per byte, 0.77 us per delay tick */
		while(CC2420_SFD_IS_1) {;} // wait until reception finishes
		FASTSPI_READ_FIFO_GARBAGE(len);
		snprintf(info_str[info_str_index], sizeof(info_str), "%lu %u", clock_seconds(), len);
	}
	process_post(&cc2420_debug_process, PROCESS_EVENT_MSG, info_str[info_str_index]);
	info_str_index = info_str_index == INFO_STR_NUM - 1 ? 0 : info_str_index+1;
	CC2420_CLEAR_CCA_INT();
	if(CC2420_FIFOP_IS_1) {
		flushrx();
		CC2420_CLEAR_FIFOP_INT();
	}
	return 1;
	
#endif
}

/*---------------------------------------------------------------------------*/
/*
 * Unbuffered RX mode using FIFOP as clock input for serial data
 */
void
cc2420_fifop_interrupt(void)
{
	/* static uint8_t octet = (0xA5>>1); */
#define NUM_BITS 7

	static int bit = NUM_BITS;
	/* static int this_bit; */

/* Debug toggle GIO pin */
	GPIO1_PORT(OUT) |= BV(GPIO1_PIN);

  CC2420_CLEAR_FIFOP_INT();
	
	if (bit==0) {
		bit = NUM_BITS;
		CC2420_DISABLE_FIFOP_INT();
		CC2420_CLEAR_FIFOP_INT();
		CC2420_FIFO_PORT(OUT) &= ~BV(CC2420_FIFO_PIN);
		/* some delay needed to ensure the last few bits get transmitted before transmitter shuts down */
		clock_delay(16); // TODO: replace magic number with F_CPU-dependent formula
		/* strobe(CC2420_SRXON); */
		strobe(CC2420_SRFOFF);
		/* CC2420_DISABLE_FIFOP_INT(); */
	} else {
		if (bit==1) {
			CC2420_FIFO_PORT(OUT) &= ~BV(CC2420_FIFO_PIN);
		} else {
			CC2420_FIFO_PORT(OUT) |= BV(CC2420_FIFO_PIN);
		}
		--bit;
	}

	// send predifiend len byte
	/* if (mode == SERIAL_JAM) { */
	/* if (bit==NUM_BITS) { */
	/* 		bit = 0; */
	/* 		octet = (0xA5>>1); */
	/* 		CC2420_DISABLE_FIFOP_INT(); */
	/* 		CC2420_CLEAR_FIFOP_INT(); */
	/* 		CC2420_FIFO_PORT(OUT) &= ~BV(CC2420_FIFO_PIN); */
	/* 		clock_delay(300); */
	/* 		strobe(CC2420_SRXON); */
	/* } else { */
	/* 	if (this_bit) { */
	/* 		CC2420_FIFO_PORT(OUT) |= BV(CC2420_FIFO_PIN); */
	/* 	} else { */
	/* 	CC2420_FIFO_PORT(OUT) &= ~BV(CC2420_FIFO_PIN); */
	/* 	} */
	/* 	++bit; */
	/* } */
	/* } */
	/* this_bit = octet & 0x01; */
	/* octet>>=1; */

	GPIO1_PORT(OUT) &= ~BV(GPIO1_PIN);

  return;
}
/*---------------------------------------------------------------------------*/
/*
 * Interrupt leaves frame intact in FIFO.
 */
#if CC2420_TIMETABLE_PROFILING
#define cc2420_timetable_size 16
TIMETABLE(cc2420_timetable);
TIMETABLE_AGGREGATE(aggregate_time, 10);
#endif /* CC2420_TIMETABLE_PROFILING */
int
cc2420_interrupt(void)
{
  CC2420_CLEAR_FIFOP_INT();
	/* GPIO2_PORT(OUT) &= ~BV(GPIO2_PIN); */
	/* GPIO2_PORT(OUT) |= BV(GPIO2_PIN); */
	/* CC2420_CLEAR_CCA_INT(); */
	
	// TEST: in JAM mode, read len and send jam, then flush RXFIFO 
	if (jam_ena) {
		uint8_t len;
		getrxbyte(&len);
		send_jam(len);
		/* send_jam(20); // TEST ONLY: cheating by bypassing SPI access altogether to eliminate time jitter with known frame length */
		flushrx();
		CC2420_CLEAR_FIFOP_INT();
	} else {
	process_poll(&cc2420_process);
	}
	/* process_poll(&cc2420_process); */

#if CC2420_TIMETABLE_PROFILING
  timetable_clear(&cc2420_timetable);
  TIMETABLE_TIMESTAMP(cc2420_timetable, "interrupt");
#endif /* CC2420_TIMETABLE_PROFILING */

  last_packet_timestamp = cc2420_sfd_start_time;
  pending++;
  cc2420_packets_seen++;

	/* GPIO2_PORT(OUT) &= ~BV(GPIO2_PIN); */

  return 1;
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(cc2420_debug_process, ev, data)
{
	PROCESS_BEGIN();
	while(1) {
		PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_MSG);
		printf("%s\n", (char *)data); //data must be string
	}
	PROCESS_END();
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(cc2420_process, ev, data)
{
  int len;
  PROCESS_BEGIN();

  PRINTF("cc2420_process: started\n");

  while(1) {
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);
#if CC2420_TIMETABLE_PROFILING
    TIMETABLE_TIMESTAMP(cc2420_timetable, "poll");
#endif /* CC2420_TIMETABLE_PROFILING */
    
    /* PRINTF("cc2420_process: calling receiver callback\n"); */ // Zhitao

    packetbuf_clear();
    packetbuf_set_attr(PACKETBUF_ATTR_TIMESTAMP, last_packet_timestamp);
    len = cc2420_read(packetbuf_dataptr(), PACKETBUF_SIZE);
		PRINTF("len = %d\n", len);
    
    packetbuf_set_datalen(len);
    
    /* NETSTACK_RDC.input(); */
		rx_callback();

#if CC2420_TIMETABLE_PROFILING
    TIMETABLE_TIMESTAMP(cc2420_timetable, "end");
    timetable_aggregate_compute_detailed(&aggregate_time,
                                         &cc2420_timetable);
      timetable_clear(&cc2420_timetable);
#endif /* CC2420_TIMETABLE_PROFILING */
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
int
cc2420_read(void *buf, unsigned short bufsize)
{
  uint8_t footer[2];
  uint8_t len;
#if CC2420_CONF_CHECKSUM
  uint16_t checksum;
#endif /* CC2420_CONF_CHECKSUM */

  if(!CC2420_FIFOP_IS_1) {
    return 0;
  }
  /*  if(!pending) {
    return 0;
    }*/
  
  pending = 0;
  
  GET_LOCK();

  cc2420_packets_read++;

  getrxbyte(&len);

  if(len > CC2420_MAX_PACKET_LEN) {
    /* /\* Oops, we must be out of sync. *\/ */
    flushrx();
    RIMESTATS_ADD(badsynch);
    RELEASE_LOCK();
    return 0;
		/* zhitao: TEST - return len as if received in reverse phase mode */
		/* flushrx(); */
		/* len = ((((len>>4)+8)%16)<<4) + ((len&0x0f)+8)%16 - AUX_LEN; */
    /* RELEASE_LOCK(); */
		/* return len; */
  }

  if(len <= AUX_LEN) {
    flushrx();
    RIMESTATS_ADD(tooshort);
    RELEASE_LOCK();
    return 0;
  }

  if(len - AUX_LEN > bufsize) {
    flushrx();
    RIMESTATS_ADD(toolong);
    RELEASE_LOCK();
    return 0;
  }

  getrxdata(buf, len - AUX_LEN);
#if CC2420_CONF_CHECKSUM
  getrxdata(&checksum, CHECKSUM_LEN);
#endif /* CC2420_CONF_CHECKSUM */
  getrxdata(footer, FOOTER_LEN);

#if CC2420_CONF_CHECKSUM
  if(checksum != crc16_data(buf, len - AUX_LEN, 0)) {
    PRINTF("checksum failed 0x%04x != 0x%04x\n",
	   checksum, crc16_data(buf, len - AUX_LEN, 0));
  }

  if(footer[1] & FOOTER1_CRC_OK &&
     checksum == crc16_data(buf, len - AUX_LEN, 0)) {
#else
  if(footer[1] & FOOTER1_CRC_OK) {
#endif /* CC2420_CONF_CHECKSUM */
    cc2420_last_rssi = footer[0];
    cc2420_last_correlation = footer[1] & FOOTER1_CORRELATION;


    packetbuf_set_attr(PACKETBUF_ATTR_RSSI, cc2420_last_rssi);
    packetbuf_set_attr(PACKETBUF_ATTR_LINK_QUALITY, cc2420_last_correlation);

    RIMESTATS_ADD(llrx);

  } else {
    RIMESTATS_ADD(badcrc);
    len = AUX_LEN;
  }

  if(CC2420_FIFOP_IS_1) {
    if(!CC2420_FIFO_IS_1) {
      /* Clean up in case of FIFO overflow!  This happens for every
       * full length frame and is signaled by FIFOP = 1 and FIFO =
       * 0. */
      flushrx();
    } else {
      /* Another packet has been received and needs attention. */
      process_poll(&cc2420_process);
    }
  }

  RELEASE_LOCK();

  if(len < AUX_LEN) {
    return 0;
  }

  return len - AUX_LEN;
}
/*---------------------------------------------------------------------------*/
void
cc2420_set_txpower(uint8_t power)
{
  GET_LOCK();
  set_txpower(power);
  RELEASE_LOCK();
}
/*---------------------------------------------------------------------------*/
int
cc2420_get_txpower(void)
{
  int power;
  GET_LOCK();
  power = (int)(getreg(CC2420_TXCTRL) & 0x001f);
  RELEASE_LOCK();
  return power;
}
/*---------------------------------------------------------------------------*/
int
cc2420_rssi(void)
{
  int rssi;
  int radio_was_off = 0;

  if(locked) {
    return 0;
  }
  
  GET_LOCK();

  if(!receive_on) {
    radio_was_off = 1;
    cc2420_on();
  }
  BUSYWAIT_UNTIL(status() & BV(CC2420_RSSI_VALID), RTIMER_SECOND / 100);

  rssi = (int)((signed char)getreg(CC2420_RSSI));

  if(radio_was_off) {
    cc2420_off();
  }
  RELEASE_LOCK();
  return rssi;
}
/*---------------------------------------------------------------------------*/
/*
static int
detected_energy(void)
{
  return cc2420_rssi();
}
*/
/*---------------------------------------------------------------------------*/
int
cc2420_cca_valid(void)
{
  int valid;
  if(locked) {
    return 1;
  }
  GET_LOCK();
  valid = !!(status() & BV(CC2420_RSSI_VALID));
  RELEASE_LOCK();
  return valid;
}
/*---------------------------------------------------------------------------*/
static int
cc2420_cca(void)
{
  int cca;
  int radio_was_off = 0;

  /* If the radio is locked by an underlying thread (because we are
     being invoked through an interrupt), we preted that the coast is
     clear (i.e., no packet is currently being transmitted by a
     neighbor). */
  if(locked) {
    return 1;
  }

  GET_LOCK();
  if(!receive_on) {
    radio_was_off = 1;
    cc2420_on();
  }

  /* Make sure that the radio really got turned on. */
  if(!receive_on) {
    RELEASE_LOCK();
    if(radio_was_off) {
      cc2420_off();
    }
    return 1;
  }

  BUSYWAIT_UNTIL(status() & BV(CC2420_RSSI_VALID), RTIMER_SECOND / 100);

  cca = CC2420_CCA_IS_1;

  if(radio_was_off) {
    cc2420_off();
  }
  RELEASE_LOCK();
  return cca;
}
/*---------------------------------------------------------------------------*/
int
cc2420_receiving_packet(void)
{
  return CC2420_SFD_IS_1;
}
/*---------------------------------------------------------------------------*/
static int
pending_packet(void)
{
  return CC2420_FIFOP_IS_1;
}
/*---------------------------------------------------------------------------*/
void
cc2420_set_cca_threshold(int value)
{
  uint16_t shifted = value << 8;
  GET_LOCK();
  setreg(CC2420_RSSI, shifted);
  RELEASE_LOCK();
}
/*---------------------------------------------------------------------------*/
void
cc2420_set_rx_callback(void (*callback)(void))
{
	rx_callback = callback;
}
