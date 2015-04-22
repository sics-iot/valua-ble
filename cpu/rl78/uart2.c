/*
 * Copyright (c) 2014, Analog Devices, Inc.
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
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/**
 * \author Maxim Salov <max.salov@gmail.com>, Ian Martin <martini@redwirellc.com>
 */

#include "rl78.h"     /* for f_CLK */
#include "sfrs.h"
#include "sfrs-ext.h"
#include "uart2.h"

#include "lib/ringbuf.h"

#pragma GCC optimize ("Os")

//#define DESIRED_BAUDRATE 38400
#define DESIRED_BAUDRATE 115200

/* Note that only 9600, 38400, and 115200 bps were tested. */
#define PRESCALE_THRESH  ((38400 + 115200) / 2)
#define PRS_VALUE        ((DESIRED_BAUDRATE < PRESCALE_THRESH) ? 4 : 1)
#define f_MCK            (f_CLK / (1 << PRS_VALUE))
#define SDR_VALUE        (f_MCK / DESIRED_BAUDRATE / 2 - 1)

#define TXBUFSIZE 128 // size must be power of two
static struct ringbuf txbuf;
static uint8_t txbuf_data[TXBUFSIZE];

static volatile int transmitting;

void
uart2_init(void)
{
  /* Refer to hardware manual for details */
  PIOR = 0U;                                           /* TxD2 and RxD2 are not redirectable, use reset value 00F for PIOR01 */

  SAU1EN = 1;                                               /* Supply clock to serial array unit 1, where UART2 is located */
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
  SPS1 = (PRS_VALUE << 4) | PRS_VALUE;                  /* Set input clock (CK10 and CK11) to fclk/16 = 2MHz */
  ST1 = 0x03U;                                          /* Stop operation of channel 0 and 1 */
  /* Setup interrupts (disable) */
  STMK2 = 1;                                                /* Disable INTST2 interrupt */
  STIF2 = 0;                                                /* Clear INTST2 interrupt request flag */
  STPR12 = 1;                                               /* Set INTST2 priority: lowest  */
  STPR02 = 1;
  SRMK2 = 1;                                                /* Disable INTSR2 interrupt */
  SRIF2 = 0;                                                /* Clear INTSR2 interrupt request flag */
  SRPR12 = 1;                                               /* Set INTSR2 priority: lowest */
  SRPR02 = 1;
  SREMK2 = 1;                                               /* Disable INTSRE2 interrupt */
  SREIF2 = 0;                                               /* Clear INTSRE2 interrupt request flag */
  SREPR12 = 1;                                              /* Set INTSRE2 priority: lowest */
  SREPR02 = 1;
  /* Setup operation mode for transmitter (channel 0) */
  /* SMR10 = 0x0023U;                                    /\* Operation clock : CK10, */
  SMR10 = 0x0022U;                                    /* Operation clock : CK10,
                                                               Transfer clock : division of CK10
                                                               Start trigger : software
                                                               Detect falling edge as start bit
                                                               Operation mode : UART
                                                         //      Interrupt source : buffer empty
                                                               Interrupt source : tx completed
                                                       */
  SCR10 = 0x8097U;                                    /* Transmission only
                                                               Reception error interrupt masked
                                                               Phase clock : type 1
                                                               No parity
                                                               LSB first
                                                               1 stop bit
                                                               8-bit data length
                                                       */
  SDR10 = SDR_VALUE << 9;
  /* Setup operation mode for receiver (channel 1) */
  NFEN0 |= 0x10;                                         /* Enable noise filter on RxD2 pin */
  SIR11 = 0x0007U;                                    /* Clear error flags */
  SMR11 = 0x0122U;                                    /* Operation clock : CK10
                                                               Transfer clock : division of CK10
                                                               Start trigger : valid edge on RxD pin
                                                               Detect falling edge as start bit
                                                               Operation mode : UART
                                                               Interrupt source : transfer end
                                                       */
  SCR11 = 0x4097U;                                    /* Reception only
                                                               Reception error interrupt masked
                                                               Phase clock : type 1
                                                               No parity
                                                               LSB first
                                                               1 stop bit
                                                               8-bit data length
                                                       */
  SDR11 = SDR_VALUE << 9;
  SO1 |= 1;                                             /* Prepare for use of channel 0 */
  SOE1 |= 1;
  P1 |= (1 << 3);                                        /* Set TxD2 high */
  PM1 &= ~(1 << 3);                                      /* Set output mode for TxD2 */
  PM1 |= (1 << 4);                                       /* Set input mode for RxD2 */
  SS1 = 0x03U;                                         /* Enable UART2 operation (both channels) */
	//  SS1 = 0x01U;                                         /* Enable UART2 operation (TxD channel only) */

  ringbuf_init(&txbuf, txbuf_data, sizeof(txbuf_data));
  /* STMK2 = 0;                                                /\* ENABLE INTST2 interrupt *\/ */
	transmitting = 0;
}

void
uart2_putchar(int c)
{
  /* Put the outgoing byte on the transmission buffer, keep trying when buffer full */ 
	/* while(!ringbuf_put(&txbuf, c)); */
  /* Put the outgoing byte on the transmission buffer, abort if buffer full */ 
	if (!ringbuf_put(&txbuf, c)) return;

  /* If there is no transmission going, we need to start it by putting
     the first byte into the UART. */
  if(transmitting == 0) {
    transmitting = 1;
		STMK2 = 1;                                                /* Disable INTST2 interrupt */
    TXD2 = ringbuf_get(&txbuf);
		STMK2 = 0;                                                /* Enable INTST2 interrupt */
  }
}

char
uart2_getchar(void)
{
  char c;
  while(!uart2_can_getchar()) ;
  c = SDR11;
  SRIF2 = 0;
  return c;
}

int
uart2_puts(const char *s)
{
  int len = 0;
  while('\0' != *s) {
    uart2_putchar(*s);
    s++;
    ++len;
  }
	uart2_putchar('\n');
  return len;
}

/* UART2 transmission ISR */
void __attribute__ ((interrupt))
st2_handler(void)
{
	/* more data => send next byte */
  if(ringbuf_elements(&txbuf) == 0) {
    transmitting = 0;
  } else {
    TXD2 = ringbuf_get(&txbuf);
  }
}
