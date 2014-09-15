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
 * @(#)$Id: cc2420.c,v 1.51 2010/04/08 18:23:24 adamdunkels Exp $
 */
/*
 * This code is almost device independent and should be easy to port.
 */

#include <stdio.h>
#include <string.h>

#include "contiki.h"

#include "dev/leds.h"
#include "dev/spi.h"
#include "dev/cc2420/cc2420.h"
#include "dev/cc2420/cc2420_const.h"
#include "glossy.h"

/*---------------------------------------------------------------------------*/
#define AUTOACK (1 << 4)
#define ADR_DECODE (1 << 11)
#define RXFIFO_PROTECTION (1 << 9)
#define CORR_THR(n) (((n) & 0x1f) << 6)
#define FIFOP_THR(n) ((n) & 0x7f)
#define RXBPF_LOCUR (1 << 13);

#define BUSYWAIT_UNTIL(cond, max_time)                                  \
  do {                                                                  \
    rtimer_clock_t t0;                                                  \
    t0 = RTIMER_NOW();                                                  \
    while(!(cond) && RTIMER_CLOCK_LT(RTIMER_NOW(), t0 + (max_time)));   \
  } while(0)

static int channel;

void cc2420_arch_init(void);

static unsigned int
status(void)
{
  uint8_t status;
  CC2420_GET_STATUS(status);
  return status;
}
/*---------------------------------------------------------------------------*/
static void
wait_for_status(unsigned int mask)
{
  BUSYWAIT_UNTIL((status() & mask), RTIMER_SECOND / 10);
}
/*---------------------------------------------------------------------------*/
static void
wait_for_not_status(unsigned int mask)
{
  BUSYWAIT_UNTIL(!(status() & mask), RTIMER_SECOND / 10);
}
/*---------------------------------------------------------------------------*/
static void
strobe(enum cc2420_register regname)
{
  CC2420_STROBE(regname);
}
/*---------------------------------------------------------------------------*/
unsigned
getreg(enum cc2420_register regname)
{
  unsigned reg;
  CC2420_READ_REG(regname, reg);
  return reg;
}
/*---------------------------------------------------------------------------*/
void
setreg(enum cc2420_register regname, unsigned value)
{
  CC2420_WRITE_REG(regname, value);
}
/*---------------------------------------------------------------------------*/
static void
write_ram(const void *buf, uint16_t addr, int count)
{
  CC2420_WRITE_RAM(buf, addr, count);
}
/*---------------------------------------------------------------------------*/
void
flushrx(void)
{
  uint8_t dummy;

  CC2420_READ_FIFO_BYTE(dummy);
  CC2420_STROBE(CC2420_SFLUSHRX);
  CC2420_STROBE(CC2420_SFLUSHRX);
}
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
  clock_delay(250);
  SET_RESET_ACTIVE();
  clock_delay(127);
  SET_RESET_INACTIVE();
  clock_delay(125);


  /* Turn on the crystal oscillator. */
  strobe(CC2420_SXOSCON);

  /* Turn on/off automatic packet acknowledgment and address decoding. */
  reg = getreg(CC2420_MDMCTRL0);

#if CC2420_CONF_AUTOACK
  reg |= AUTOACK | ADR_DECODE;
#else
  reg &= ~(AUTOACK | ADR_DECODE);
#endif /* CC2420_CONF_AUTOACK */
  setreg(CC2420_MDMCTRL0, reg);

  /* Set transmission turnaround time to the lower setting (8 symbols
     = 0.128 ms) instead of the default (12 symbols = 0.192 ms). */
  /*  reg = getreg(CC2420_TXCTRL);
  reg &= ~(1 << 13);
  setreg(CC2420_TXCTRL, reg);*/

  
  /* Change default values as recomended in the data sheet, */
  /* correlation threshold = 20, RX bandpass filter = 1.3uA. */
  setreg(CC2420_MDMCTRL1, CORR_THR(20));
  reg = getreg(CC2420_RXCTRL1);
  reg |= RXBPF_LOCUR;
  setreg(CC2420_RXCTRL1, reg);

  /* Set the FIFOP threshold to maximum. */
  setreg(CC2420_IOCFG0, FIFOP_THR(127));

  /* Turn off "Security enable" (page 32). */
  reg = getreg(CC2420_SECCTRL0);
  reg &= ~RXFIFO_PROTECTION;
  setreg(CC2420_SECCTRL0, reg);

  cc2420_set_pan_addr(0xffff, 0x0000, NULL);
  cc2420_set_channel(CC2420_CONF_CHANNEL);
  cc2420_set_cca_threshold(CC2420_CONF_CCA_THRESH);

  flushrx();

  return 1;
}
/* int */
/* cc2420_init(void) */
/* { */
/*   uint16_t reg; */
/*   { */
/*     int s = splhigh(); */
/*     spi_init(); */

/*     /\* all input by default, set these as output *\/ */
/*     P4DIR |= BV(CC2420_CSN_PIN) | BV(CC2420_VREG_PIN) | BV(CC2420_RESET_PIN); */

/*     SPI_DISABLE();                /\* Unselect radio. *\/ */
/*     DISABLE_FIFOP_INT(); */
/*     FIFOP_INT_INIT(); */
/*     splx(s); */
/*   } */

/*   /\* Turn on voltage regulator and reset. *\/ */
/*   SET_VREG_ACTIVE(); */
/*   SET_RESET_ACTIVE(); */
/*   clock_delay(127); */
/*   SET_RESET_INACTIVE(); */

/*   /\* Turn on the crystal oscillator. *\/ */
/*   FASTSPI_STROBE(CC2420_SXOSCON); */

/*   /\* Turn off automatic packet acknowledgment and address decoding. *\/ */
/*   FASTSPI_GETREG(CC2420_MDMCTRL0, reg); */
/*   reg &= ~(AUTOACK | ADR_DECODE); */
/*   FASTSPI_SETREG(CC2420_MDMCTRL0, reg); */

/*   /\* Change default values as recomended in the data sheet, *\/ */
/*   /\* correlation threshold = 20, RX bandpass filter = 1.3uA. *\/ */
/*   FASTSPI_SETREG(CC2420_MDMCTRL1, CORR_THR(20)); */
/*   FASTSPI_GETREG(CC2420_RXCTRL1, reg); */
/*   reg |= RXBPF_LOCUR; */
/*   FASTSPI_SETREG(CC2420_RXCTRL1, reg); */

/*   /\* Set the FIFOP threshold to maximum. *\/ */
/*   FASTSPI_SETREG(CC2420_IOCFG0, FIFOP_THR(127)); */

/*   /\* Turn off "Security enable" (page 32). *\/ */
/*   FASTSPI_GETREG(CC2420_SECCTRL0, reg); */
/*   reg &= ~RXFIFO_PROTECTION; */
/*   FASTSPI_SETREG(CC2420_SECCTRL0, reg); */

/*   flushrx(); */
  
/*   return 1; */
/* } */
/*---------------------------------------------------------------------------*/
void
cc2420_set_pan_addr(unsigned pan,
                    unsigned addr,
                    const uint8_t *ieee_addr)
{
  uint16_t f = 0;
  uint8_t tmp[2];

  /*
   * Writing RAM requires crystal oscillator to be stable.
   */
  wait_for_status(BV(CC2420_XOSC16M_STABLE));

  tmp[0] = pan & 0xff;
  tmp[1] = pan >> 8;
  write_ram(&tmp, CC2420RAM_PANID, 2);

  tmp[0] = addr & 0xff;
  tmp[1] = addr >> 8;
  write_ram(&tmp, CC2420RAM_SHORTADDR, 2);
  if(ieee_addr != NULL) {
    uint8_t tmp_addr[8];
    /* LSB first, MSB last for 802.15.4 addresses in CC2420 */
    for (f = 0; f < 8; f++) {
      tmp_addr[7 - f] = ieee_addr[f];
    }
    write_ram(tmp_addr, CC2420RAM_IEEEADDR, 8);
  }

}
/*---------------------------------------------------------------------------*/
int
cc2420_set_channel(int c)
{
  uint16_t f;

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
  return 1;
}
/*---------------------------------------------------------------------------*/
void
cc2420_set_cca_threshold(int value)
{
  uint16_t shifted = value << 8;
  setreg(CC2420_RSSI, shifted);
}
/*---------------------------------------------------------------------------*/
