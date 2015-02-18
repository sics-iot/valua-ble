#include "contiki.h"
#include "dev/leds.h"
#include "dev/button-sensor.h"

#include <stdio.h>

PROCESS(my_process, "Good morning");
AUTOSTART_PROCESSES(&my_process);

PROCESS_THREAD(my_process, ev, data)
{
	static struct etimer et;

	PROCESS_BEGIN();

	printf("program started\n");

	while(1) {
		// Bangalore sunrise, red on
		leds_on(LEDS_RED);
		printf("Good morning, Bangalore!\n");
		etimer_set(&et, CLOCK_SECOND * 7 / 2);
		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

		// Stockholm sunrise, green on 
		leds_on(LEDS_GREEN);
		printf("God morgon, Stockholm!\n");
		etimer_set(&et, CLOCK_SECOND * 7);
		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

		// Stockholm sunset, green off
		leds_off(LEDS_GREEN);
		printf("Stockholm, god natt...\n");
		etimer_set(&et, CLOCK_SECOND * 3 / 2);
		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

		// Bangalore sunset, red off
		leds_off(LEDS_RED);
		printf("Bangalore, good night...\n");
		etimer_set(&et, CLOCK_SECOND * 12);
		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
	}

	PROCESS_END();
}
