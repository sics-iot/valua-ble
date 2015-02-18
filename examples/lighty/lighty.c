#include "contiki.h"
#include "dev/leds.h"
#include "dev/button-sensor.h"

#include <stdio.h>

PROCESS(blinky_process, "blinky");
PROCESS(button_process, "blinky");
AUTOSTART_PROCESSES(&blinky_process, &button_process);

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

PROCESS_THREAD(button_process, ev, data)
{
	static struct etimer et;

	PROCESS_BEGIN();

	// init button sensor
	button_sensor.configure(SENSORS_ACTIVE, 1);
	/* SENSORS_ACTIVATE(button_sensor); */

	while(1) {
		PROCESS_WAIT_EVENT();
		if(ev == sensors_event && data == &button_sensor) {
			leds_on(LEDS_GREEN);
			/* off after half a second */
			etimer_set(&et, CLOCK_SECOND / 2);
			PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

			/* while(button_sensor.value(BUTTON_SENSOR) != 0) { */
			/* 	PROCESS_PAUSE(); */
			/* } */
			leds_off(LEDS_GREEN);
		}
	}
	
	PROCESS_END();
}
