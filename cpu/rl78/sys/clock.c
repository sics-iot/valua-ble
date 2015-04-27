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
 * \author Ian Martin <martini@redwirellc.com>
 */

/* #include <time.h> */

#include "contiki.h"
#include "platform-conf.h"

#ifndef BIT
#define BIT(n) (1 << (n))
#endif

#define CLOCK_CHANNEL 0

#define NBITS2(n) ((n&2)?1:0)
#define NBITS4(n) ((n&(0xC))?(2+NBITS2(n>>2)):(NBITS2(n)))
#define NBITS8(n) ((n&0xF0)?(4+NBITS4(n>>4)):(NBITS4(n)))
#define NBITS16(n) ((n&0xFF00)?(8+NBITS8(n>>8)):(NBITS8(n)))
#define NBITS32(n) ((n&0xFFFF0000)?(16+NBITS16(n>>16)):(NBITS16(n)))
#define NBITS(n) (n==0?0:NBITS32(n)+1)
#define LOG2(n) (NBITS(n) - 1)

#define CLOCK_SCALER LOG2((f_CLK / f_TMR))
#define TIMER00_CLK_FREQ (f_CLK >> CLOCK_SCALER)

#define INTERVAL (TIMER00_CLK_FREQ / CLOCK_CONF_SECOND)

#define MAX_TICKS (~((clock_time_t)0) / 2)

#define clock() (0xffff - TCR[CLOCK_CHANNEL])

static volatile unsigned long seconds = 0;
static volatile clock_time_t count = 0;
unsigned long et_process_polled;

void
clock_init(void)
{
	/* Enable Timer Array Unit 0. */
	TAU0EN = 1;
	/* Supply clock frm CK00 */
	TPS0 = (TPS0 & 0xFFF0) | CLOCK_SCALER;
	/* Stop all channels (4*16bit + 2*8bit: 0,1,2,3,H1,H3 */
	TT0 = 0x0A0F;
	/* Mask channel interrupt */
	TMMK00 = 1;

	/* Timer mode: CKS0, interval timer */
	TMR[CLOCK_CHANNEL] = 0x0000;

	/* Interval period = tick period * (TDRmn + 1)*/
	TDR00 = INTERVAL - 1;

	/* Enable channel interrupt */
	TMIF00 = 0;
	TMMK00 = 0;

	/* Start counting */
	TS0 |= BIT(CLOCK_CHANNEL);
}
/*---------------------------------------------------------------------------*/
clock_time_t
clock_time(void)
{
  clock_time_t t1, t2;
  do {
    t1 = count;
    t2 = count;
  } while(t1 != t2);
  return t1;
}
/*---------------------------------------------------------------------------*/
unsigned long
clock_seconds(void)
{
	/* return clock() / CLOCK_CONF_SECOND; */
	return seconds;
}
/*---------------------------------------------------------------------------*/

void
clock_wait(clock_time_t t)
{
	clock_time_t t0;
	t0 = clock();
	while(clock() - t0 < t);
}

/* System tick ISR */
void __attribute__ ((interrupt))
tm00_handler(void)
{
	// (TCR00 is automatically reloaded with TDR00's value at interrupt)
	++count;

	if (count % CLOCK_CONF_SECOND == 0) ++seconds;

	/* expiring or expired etimer? => poll etimer process */
	if(etimer_pending() &&
	   (etimer_next_expiration_time() - count - 1) > MAX_TICKS) {
		etimer_request_poll();
		et_process_polled++;
	}
}
