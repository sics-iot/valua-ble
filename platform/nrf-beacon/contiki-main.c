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

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "contiki.h"
#include "net/netstack.h"

#include "dev/serial-line.h"

#include "net/ip/uip.h"

#include "dev/button-sensor.h"

#if NETSTACK_CONF_WITH_IPV6
#include "net/ipv6/uip-ds6.h"
#endif /* NETSTACK_CONF_WITH_IPV6 */

#include "uart0.h"
#include "contiki-uart.h"
#include "watchdog.h"
#include "slip-arch.h"

#if __GNUC__
#include "write.h"
#endif

SENSORS(&button_sensor);

static uint16_t node_id = 0x0102;

/*---------------------------------------------------------------------------*/
int contiki_argc = 0;
char **contiki_argv;

static void
delay_1sec(void)
{
  /* Delay 1 second */
  register unsigned long int i;
  for(i = 0x000FFFFFUL; i; --i) {
    asm ("nop");
  }
}
int
main(int argc, char **argv)
{
  asm ("di");
  /* Setup clocks */
  CMC = 0x00U;                                        /* Default value: use on-chip osc. as system clock */
//  CSC = 0x80U;                                        /* Start XT1 and HOCO, stop X1 */
  MSTOP = 1U;			/* Stop X1 */
//  CKC = 0x00U;
  MCM0 = 0U;
  delay_1sec();
//  OSMC = 0x00;                                       /* Supply fsub to peripherals, including Interval Timer */
  OSMC= 0x10U;

  uart0_init();

#if __GNUC__
  /* Force linking of custom write() function: */
  write(1, NULL, 0);
#endif

  /* Setup 12-bit interval timer */
  RTCEN = 1;                                              /* Enable 12-bit interval timer and RTC */
  ITMK = 1;                                               /* Disable IT interrupt */
  ITPR0 = 0;                                              /* Set interrupt priority - highest */
  ITPR1 = 0;
  ITMC = 0x8FFFU;                                    /* Set maximum period 4096/32768Hz = 1/8 s, and start timer */
  ITIF = 0;                                               /* Clear interrupt request flag */
  ITMK = 0;                                               /* Enable IT interrupt */
  /* asm ("ei");                                             / * Enable interrupts * / */

  /* Disable analog inputs because they can conflict with the SPI buses: */
  ADPC = 0x01;  /* Configure all analog pins as digital I/O. */
  PMC0 &= 0xF0; /* Disable analog inputs. */

  clock_init();

  /* Initialize Joystick Inputs: */
//  PM5 |= BIT(5) | BIT(4) | BIT(3) | BIT(2) | BIT(1); /* Set pins as inputs. */
//  PU5 |= BIT(5) | BIT(4) | BIT(3) | BIT(2) | BIT(1); /* Enable internal pull-up resistors. */

  /* Initialize LED outputs: */
#define BIT(n) (1 << (n))
  PM2 &= ~BIT(1); /* LED1 */
//  delay_1sec();
//  delay_1sec();
  int i;
  for (i=0;i<3;i++) {
  LED1 = 0; // TEST: blink led
  delay_1sec();
  LED1 = 1;
  delay_1sec();
  }
//  delay_1sec();
//  LED1 = 0; // TEST: blink led
//  delay_1sec();
//  LED1 = 1;

  /* crappy way of remembering and accessing argc/v */
  contiki_argc = argc;
  contiki_argv = argv;

  process_init();
  process_start(&etimer_process, NULL);
  ctimer_init();

  serial_line_init();

  autostart_start(autostart_processes);

  printf("node_id = %hu\n", node_id);

  while(1) {
    watchdog_periodic();

    while(uart0_can_getchar()) {
      char c;
      c = uart0_getchar();
      if(uart0_input_handler) {
        uart0_input_handler(c);
      }
    }

    process_run();

    etimer_request_poll();
  }

  return 0;
}
/*---------------------------------------------------------------------------*/
void
log_message(char *m1, char *m2)
{
  printf("%s%s" NEWLINE, m1, m2);
}
/*---------------------------------------------------------------------------*/
void
uip_log(char *m)
{
  printf("%s" NEWLINE, m);
}
/*---------------------------------------------------------------------------*/