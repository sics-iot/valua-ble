#ifndef CC2420_JAMMER
#define CC2420_JAMMER

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>

#include "contiki.h"
#include "dev/button-sensor.h"
#include "net/rime/rime.h"
#include "dev/leds.h"
#include "dev/serial-line.h"
#include "dev/spi.h"
#include "cc2420.h"
#include "dev/cc2420/cc2420_const.h"
#include "lib/random.h"
#include "dev/radio.h"
#include "dev/watchdog.h"
#include "lib/crc16.h"
#include "sys/node-id.h"

#define RX 0
#define TX 1
#define DROPLET 2
#define MOD 3
#define UNMOD 4
#define CH 5
#define DRIZZLE 6
#define TX2 7
/* #define JAM 8 */
#define ACK 8
#define OFF 9
#define LAST_MODE OFF
#define NUM_MODES (LAST_MODE+1)

#define TX_SOURCE_SEQNO 0
#define TX_SOURCE_DROPLETS 1
#define TX_SOURCE_FRAME_BUF 2
#define MAX_TX_SOURCE 3

int cc2420_set_frequency(uint16_t f);
uint16_t cc2420_get_frequency(void);
void strobe(enum cc2420_register regname);
unsigned int status(void);
void flushrx(void);

void cc2420_set_rx_callback(void (*callback)(void));

#endif
