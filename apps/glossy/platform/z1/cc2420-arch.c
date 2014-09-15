/*
 * Copyright (c) 2006, Swedish Institute of Computer Science
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
 * @(#)$Id: cc2420-arch.c,v 1.7 2010/01/19 20:35:41 adamdunkels Exp $
 */

#include <msp430.h>
#include <signal.h>

#include "contiki.h"
#include "contiki-net.h"

#include "dev/spi.h"
#include "dev/cc2420/cc2420.h"
#include "isr_compat.h"
#include "cc2420-jammer.h"

#define CONF_SFD_TIMESTAMPS 0

#ifdef CONF_SFD_TIMESTAMPS
#include "cc2420-arch-sfd.h"
#endif

extern rtimer_clock_t cc2420_interrupt_time;

/*---------------------------------------------------------------------------*/
#if CONTIKI_TARGET_Z1
#define RETURN return 1
#else
#define RETURN LPM4_EXIT
#endif

/* #if CONTIKI_TARGET_Z1 */
/* // Z1: called from real ISR from PORT1 */
/* int */
/* cc2420_port1_interrupt(void) */
/* #elif CONTIKI_TARGET_SKY */
/* ISR(CC2420_IRQ, cc2420_port1_interrupt) */
/* #else */
/* ISR(CC2420_IRQ, cc2420_port1_interrupt) */
/* #endif */
/* { */
/* 	/\* indicates packet size has exceeded RXFIFO threshold or packet reception has ended *\/ */
/* 	if(!!(CC2420_FIFOP_PORT(IFG) & BV(CC2420_FIFOP_PIN))) { */
/* 		/\* GPIO2_PORT(OUT) &= ~BV(GPIO2_PIN); *\/ */
/* 		/\* GPIO2_PORT(OUT) |= BV(GPIO2_PIN); *\/ */
/* 		/\* GPIO2_PORT(OUT) &= ~BV(GPIO2_PIN); *\/ */
/* 		if (cc2420_interrupt()) { */
/* 				RETURN; */
/* 		} */
/* 	} */
/* /\* #if ENABLE_CCA_INTERRUPT *\/ */
/* 	/\* CCA flag on while FIFOP flag off: packet header just arrived *\/ */
/* 	else if(!!(CC2420_CCA_PORT(IFG) & BV(CC2420_CCA_PIN))) { */
/* 		/\* GPIO1_PORT(OUT) &= ~BV(GPIO1_PIN); *\/ */
/* 		/\* GPIO1_PORT(OUT) |= BV(GPIO1_PIN); *\/ */
/* 		/\* GPIO1_PORT(OUT) &= ~BV(GPIO1_PIN); *\/ */
/* 		if(cc2420_cca_interrupt()) { */
/* 			RETURN; */
/* 		} */
/* 	} */
/* /\* #endif *\/ */
/* 	/\* GPIO2_PORT(OUT) &= ~BV(GPIO2_PIN); *\/ */
/* 	/\* GPIO2_PORT(OUT) |= BV(GPIO2_PIN); *\/ */
/* 	/\* GPIO2_PORT(OUT) |= BV(GPIO2_PIN); *\/ */
/* 	/\* GPIO2_PORT(OUT) &= ~BV(GPIO2_PIN); *\/ */
/* 	RETURN; */
/* } */
/*---------------------------------------------------------------------------*/
void
cc2420_arch_init(void)
{
  spi_init();

  /* all input by default, set these as output */
  CC2420_CSN_PORT(DIR) |= BV(CC2420_CSN_PIN);
  CC2420_VREG_PORT(DIR) |= BV(CC2420_VREG_PIN);
  CC2420_RESET_PORT(DIR) |= BV(CC2420_RESET_PIN);

/* #if CONF_SFD_TIMESTAMPS */
  cc2420_arch_sfd_init();
/* #endif */

  CC2420_SPI_DISABLE();                /* Unselect radio. */
}
/*---------------------------------------------------------------------------*/
