#include "contiki.h"
#include "dev/leds.h"
#include "dev/button-sensor.h"

#include <stdio.h>

#define DEBUG 0
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#define EDEER 0 // any number in [0, 127]

PROCESS(rama_process, "rama");
PROCESS(deer_process, "deer");
AUTOSTART_PROCESSES(&rama_process, &deer_process);

static void
leds_on_deer(int n)
{
	switch(n) {
	case 0:
		leds_off(LEDS_ALL);
		break;
	case 1:
		leds_on(LEDS_RED);leds_off(LEDS_GREEN);leds_off(LEDS_BLUE);
		break;
	case 2:
		leds_on(LEDS_RED);leds_on(LEDS_GREEN);leds_off(LEDS_BLUE);
		break;
	case 3:
		leds_on(LEDS_ALL);
		break;
	default:;
	}
}

PROCESS_THREAD(rama_process, ev, data)
{
	int n; // No. deers seen

	PROCESS_BEGIN();

	/* Init. button sensor */
	SENSORS_ACTIVATE(button_sensor);

	while(1) {
		/* Wait on any event */
		PROCESS_WAIT_EVENT_UNTIL(
														 ev == sensors_event &&
														 data == &button_sensor);
		printf("Rama: I'll go to the forest, Sita. ");
		process_post(&deer_process, EDEER, NULL);
		PROCESS_WAIT_EVENT_UNTIL(ev == EDEER);
		n = (int)data;
		printf("I find %s!\n", n>0?"a DEER, dear":"nothing");
	}
	
	PROCESS_END();
}

PROCESS_THREAD(deer_process, ev, data)
{
	static struct etimer et;
	static int n;  // No. deers

	PROCESS_BEGIN();

	etimer_set(&et, CLOCK_SECOND * 1);

	while(1) {
		PROCESS_WAIT_EVENT();
		if(ev == EDEER) {
			process_post(&rama_process, EDEER, (int)n);
			n = n>0 ? n-1 : 0;
			leds_on_deer(n);
			PRINTF("%d\n", n);
		} else if(etimer_expired(&et)) {
			n = n<3 ? n+1 : 3;
			leds_on_deer(n);
			PRINTF("%d\n", n);
			etimer_set(&et, CLOCK_SECOND * 3);
		}
	}

	PROCESS_END();

}
