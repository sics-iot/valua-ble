#include "contiki.h"
#include "dev/leds.h"
#include "dev/button-sensor.h"

#include <stdio.h>

PROCESS(my_process, "pushya");
AUTOSTART_PROCESSES(&my_process);

PROCESS_THREAD(my_process, ev, data)
{
	static struct etimer et;
	static int n;

	PROCESS_BEGIN();

	/* Init. button sensor */
	SENSORS_ACTIVATE(button_sensor);

	while(1) {
		/* Wait on any event */
		PROCESS_WAIT_EVENT();
		if(ev == sensors_event &&
			 data == &button_sensor) {
			leds_toggle(LEDS_RED);
			printf("Ouch, don't push me...");
			etimer_set(&et, CLOCK_SECOND * 2);
		} else if(etimer_expired(&et)) {
			printf("to use Contiki!\n");
			n++;
			printf("I've said it %d time%s%s\n", n, n>1?"s":"", n>9?"!!!":"...");
		}
	}
	
	PROCESS_END();
}
