#include "contiki.h"
#include "dev/leds.h"

#include <stdio.h>

PROCESS(blinky_process, "blinky");
AUTOSTART_PROCESSES(&blinky_process);

PROCESS_THREAD(blinky_process, ev, data)
{
	static struct etimer et;

	PROCESS_BEGIN();

	printf("*blinky* started\n");

	while(1) {
		// on
		leds_on(LEDS_RED);
		etimer_set(&et, CLOCK_SECOND * 1);
		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

		// off
		leds_off(LEDS_RED);
		etimer_set(&et, CLOCK_SECOND / 2);
		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
	}

	PROCESS_END();
}
