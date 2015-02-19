#include "contiki.h"
#include "dev/leds.h"
#include "dev/button-sensor.h"

#include <stdio.h>

PROCESS(my_process, "pushy");
AUTOSTART_PROCESSES(&my_process);

PROCESS_THREAD(my_process, ev, data)
{
	PROCESS_BEGIN();

	/* Init. button sensor */
	SENSORS_ACTIVATE(button_sensor);

	while(1) {
		/* Wait on any event */
		PROCESS_WAIT_EVENT();
		if(ev == sensors_event &&
			 data == &button_sensor) {
			leds_toggle(LEDS_BLUE);
			printf("Ouch, don't push me!\n");
		}
	}
	
	PROCESS_END();
}
