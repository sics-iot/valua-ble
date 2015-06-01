#ifndef CC2420_JAMMER
#define CC2420_JAMMER

#define RX 0
#define TX 1
#define DROPLET 2
#define MOD 3
#define UNMOD 4
#define CH 5
#define DRIZZLE 6
#define OFF 7
#define TX2 8
#define JAM 9
#define LAST_MODE JAM
#define NUM_MODES (LAST_MODE+1)

int cc2420_set_frequency(uint16_t f);
uint16_t cc2420_get_frequency(void);
void strobe(enum cc2420_register regname);
unsigned int status(void);
void flushrx(void);

void cc2420_set_rx_callback(void (*callback)(void));

#endif
