/*
 * Copyright (c) 2007, Swedish Institute of Computer Science.
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
 *         Best-effort single-hop unicast example
 * \author
 *         Adam Dunkels <adam@sics.se>
 */

#include "contiki.h"
#include "net/rime.h"
#include "dev/button-sensor.h"
#include "dev/serial-line.h"
#include "dev/leds.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#include "dev/cc2420.h"
#include "dev/cc2420_const.h"
#include "lib/random.h"

#include "commands.h"

#define CHANNEL 20
#define DEFAULT_TXPOWER_LEVEL 7

#define TX_INTERVAL CLOCK_SECOND / 2
#define MAX_TX_PACKETS 100
#define PAYLOAD_LEN (43-17) // contikimac SHORTEST_PACKET_SIZE - rime header size

uint16_t seqno;

/*---------------------------------------------------------------------------*/
PROCESS(example_unicast_process, "Example unicast");
AUTOSTART_PROCESSES(&example_unicast_process);
/*---------------------------------------------------------------------------*/
static void
sent_uc(struct unicast_conn *c, int status, int num_tx)
{
  printf("unicast message sent: seqno %u num_tx %d rx_time %u tx_time %u\n",
				 seqno-1, num_tx, (unsigned)packetbuf_attr(PACKETBUF_ATTR_LISTEN_TIME),
    (unsigned)packetbuf_attr(PACKETBUF_ATTR_TRANSMIT_TIME));
}
/*---------------------------------------------------------------------------*/
static void
recv_uc(struct unicast_conn *c, const rimeaddr_t *from)
{
  /* printf("from %d.%d: %s\n",  */
	/* 			 from->u8[0], from->u8[1], (char *)packetbuf_dataptr()); */
  printf("%s\t", 
				 (char *)packetbuf_dataptr());
}

static const struct unicast_callbacks unicast_callbacks = {recv_uc, sent_uc};
static struct unicast_conn uc;

void
restart(int tx_freq)
{
	memset(&rimestats, 0, sizeof(rimestats));
	sum_lqi = 0;
	sum_rssi = 0;
	seqno = 0;
	tx_interval = CLOCK_SECOND >> tx_freq;
	if(tx_interval > 0) {
		etimer_set(&et, CLOCK_SECOND/2);
		printf("restart\n");
	} else {
		etimer_stop(&et);
		printf("stopped\n");
	}
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(example_unicast_process, ev, data)
{
	static char last_ch;

  PROCESS_EXITHANDLER(unicast_close(&uc);)
    
  PROCESS_BEGIN();

	commands_set_callback(restart);

	button_sensor.configure(SENSORS_ACTIVE, 1);

	/* Change channel */
  cc2420_set_channel(CHANNEL);

	cc2420_set_txpower(DEFAULT_TXPOWER_LEVEL);

  unicast_open(&uc, 146, &unicast_callbacks);

  while(1) {
		rimeaddr_t addr;
    PROCESS_WAIT_EVENT();
    if(ev == PROCESS_EVENT_TIMER && etimer_expired(&et)) {
			if(seqno < max_tx_packets) {
				etimer_set(&et, tx_interval * 3 / 4 + random_rand() % (tx_interval/2));
				/* etimer_set(&et, tx_interval); */
				char str[payload_len];
				memset(str, 0, sizeof(str));
				snprintf(str, sizeof(str), "%u", seqno);

				packetbuf_copyfrom(str, payload_len);
				addr.u8[0] = 1;
				addr.u8[1] = 0;
				if(!rimeaddr_cmp(&addr, &rimeaddr_node_addr)) {
					unicast_send(&uc, &addr);
					++seqno;
				}
			} else {
				etimer_stop(&et);
				printf("Finished sending %u packets\n", seqno);
			}
    } else if(ev == serial_line_event_message) {
      char ch = *(char *)data;
      if(ch == '\0') {
				ch = last_ch;
			}
			last_ch = ch;

			/* Single character command handlers */
			do_command(ch);
		}
  }

  PROCESS_END();
}

/* const struct command command_table[] =	{ */
/* 	/\* {'n', next_mode}, *\/ */
/* 	/\* {'p', previous_mode}, *\/ */
/* 	{'e', reboot}, */
/* 	{'u', power_up}, */
/* 	{'d', power_down}, */
/* 	{'h', tx_frequency_up}, */
/* 	{'l', tx_frequency_down}, */
/* 	/\* {'H', rtimer_frequency_up}, *\/ */
/* 	/\* {'L', rtimer_frequency_down}, *\/ */
/* 	{'r', rssi}, */
/* 	{'+', channel_up}, */
/* 	{'-', channel_down}, */
/* 	{'>', frequency_up}, */
/* 	{'<', frequency_down}, */
/* 	{'c', cca_mux_up}, */
/* 	{'s', sfd_mux_up}, */
/* 	{'v', view_rx_statistics}, */
/* 	{'V', view_failed_rx_statistics}, */
/* 	{'w', view_tx_power_level}, */
/* 	{'m', tx_packets_up}, */
/* 	{'f', tx_packets_down}, */
/* 	/\* {'a', this_mode_again}, *\/ */
/* 	{'Y', payload_len_up}, */
/* 	{'y', payload_len_down}, */
/* 	{'g', agc_lnamix_gainmode_up}, */
/* 	{'G', agc_vga_gain_up}, */
/* 	{'D', debug_hssd}, */
/* 	{'A', debug_analog}, */
/* }; */

