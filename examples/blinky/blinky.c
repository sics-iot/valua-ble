#include "contiki.h"
#include "dev/leds.h"

#include <stdio.h>

PROCESS(blinky_process, "blinky");
AUTOSTART_PROCESSES(&blinky_process);

PROCESS_THREAD(blinky_process, ev, data)
{
	static struct etimer et;

	PROCESS_BEGIN();

	printf("blinky started");

	while(1) {
		leds_on(LEDS_RED);
		etimer_set(&et, 1 * CLOCK_SECOND);
		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
		leds_off(LEDS_RED);
		etimer_reset(&et);
	}

	PROCESS_END();
}
