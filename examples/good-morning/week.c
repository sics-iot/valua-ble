#include "contiki.h"
#include "dev/leds.h"

#include <stdio.h>

PROCESS(my_process, "Good morning");
AUTOSTART_PROCESSES(&my_process);

const static char *days[] = {"Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday", "Sunday"};

PROCESS_THREAD(my_process, ev, data)
{
	static struct etimer et;
	static int n; // day of week index [0..6]
	const char *str; // day string pointer ["Monday"..."Sunday"]

	PROCESS_BEGIN();

	printf("program started\n");

	while(1) {
		// new day
		str = days[n];
		n = (n+1) % 7;
		printf("---%s has come---\n", str);

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
