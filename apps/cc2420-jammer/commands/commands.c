#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "dev/cc2420.h"
#include "dev/cc2420_const.h"
#include "net/rime.h"
#include "dev/watchdog.h"

#include "commands.h"

// Field Vector (FV) generator macro based on user defined MSB and LSB
#define FV(MSB, LSB) \
	(((0x0001<<(MSB - LSB +1)) - 1) << LSB) // 2 ^ nbits - 1, then left shift

// Field value (FVAL) generator macro
#define FVAL(REGVAL, MSB, LSB) \
	((REGVAL & FV(MSB, LSB)) >> LSB)

// Set Field macro
#define SETFD(REGVAL, FVAL, MSB, LSB) \
	((REGVAL & ~FV(MSB, LSB)) | (FVAL << LSB))

#define FDS(FVAL, LSB) \
	((FVAL) << (LSB))

// Set Fields
#define SETFDS(REGVAL, FV, FDS)	\
	((REGVAL & ~FV) | (FDS))

#define CCAMUX_BV (31<<0)
#define SFDMUX_BV (31<<5)

int mode;
clock_time_t tx_interval = DEFAULT_TX_INTERVAL;
int max_tx_packets = DEFAULT_MAX_TX_PACKETS;
int payload_len = DEFAULT_PAYLOAD_LEN;
struct etimer et;
struct rtimer rt;
rtimer_clock_t rtimer_interval = DEFAULT_RTIMER_INTERVAL;
long int sum_rssi;
long unsigned sum_lqi;
uint8_t len_hdr = 127;

extern int jam_ena;
extern const unsigned char tx_power_level[10];
extern int cc2420_packets_seen, cc2420_packets_read;

unsigned getreg(enum cc2420_register regname);
void setreg(enum cc2420_register regname, unsigned value);

static void (*callback)(int v);

// Individual command handlers
/*---------------------------------------------------------------------------*/
static void
next_mode(void)
{
	/* start_mode((mode + 1) % NUM_MODES); */
}
/*---------------------------------------------------------------------------*/
static void
previous_mode(void)
{
	/* start_mode((mode - 1) % NUM_MODES); */
}
/*---------------------------------------------------------------------------*/
static void
reboot(void)
{
	watchdog_reboot();
}
/*---------------------------------------------------------------------------*/
static void
power_up(void)
{
	int power = cc2420_get_txpower();
	if (power < 31) {
		cc2420_set_txpower(power+1);
	}
	printf("tx power = %u\n", cc2420_get_txpower());
}
/*---------------------------------------------------------------------------*/
static void
power_down(void)
{
	int power = cc2420_get_txpower();
	if (power > 0) {
		cc2420_set_txpower(power-1);
	}
	printf("tx power = %u\n", cc2420_get_txpower());
}
/*---------------------------------------------------------------------------*/
static void
tx_frequency_up(void)
{
	tx_interval /= 2;
	printf("tx frequency = %lu\n", CLOCK_SECOND / tx_interval);
}
/*---------------------------------------------------------------------------*/
static void
tx_frequency_down(void)
{
	tx_interval *= 2;
	printf("tx frequency = %lu\n", CLOCK_SECOND / tx_interval);
}
/*---------------------------------------------------------------------------*/
static void
rtimer_frequency_up(void)
{
	/* rtimer_interval /= 2; */
	--rtimer_interval;
	printf("rtimer interval %u (%u Hz)\n", rtimer_interval,
				 (RTIMER_SECOND+rtimer_interval/2) / rtimer_interval);
}
/*---------------------------------------------------------------------------*/
static void
rtimer_frequency_down(void)
{
	rtimer_interval *= 2;
	printf("rtimer interval %u (%u Hz)\n", rtimer_interval,
				 (RTIMER_SECOND+rtimer_interval/2) / rtimer_interval);
}
/*---------------------------------------------------------------------------*/
static void
rssi(void)
{
	int8_t val = cc2420_rssi();
	uint16_t reg = getreg(CC2420_AGCCTRL);
	unsigned lna = reg & 3;
	unsigned vga_gain = (reg &  (0x7F<<4)) >> 4;
	printf("RSSI = %d LNAMIX_GAINMODE = %u VGA_GAIN = %u\n", val, lna, vga_gain);
}
/*---------------------------------------------------------------------------*/
static void
channel_up(void)
{
	int chn = cc2420_get_channel();
	chn = (chn - 11 + 1) % 16 + 11;
	cc2420_set_channel(chn);
	printf("channel = %d\n", chn);
}
/*---------------------------------------------------------------------------*/
static void
channel_down(void)
{
	int chn = cc2420_get_channel();
	chn = (chn - 11 + 15) % 16 + 11;
	cc2420_set_channel(chn);
	printf("channel = %d\n", chn);
}
/*---------------------------------------------------------------------------*/
static void
frequency_up(void)
{
	/* uint16_t f = cc2420_get_frequency(); */
	/* cc2420_set_frequency(f + 1); */
	/* printf("cc2420 frequency = %u\n", cc2420_get_frequency()); */
}
/*---------------------------------------------------------------------------*/
static void
frequency_down(void)
{
	/* uint16_t f = cc2420_get_frequency(); */
	/* cc2420_set_frequency(f - 1); */
	/* printf("cc2420 frequency = %u\n", cc2420_get_frequency()); */
}
/*---------------------------------------------------------------------------*/
static void
cca_mux_up(void)
{
	uint16_t reg;
	reg = getreg(CC2420_IOCFG1);
	unsigned ccamux = (reg & CCAMUX_BV) >> 0;
	ccamux = ccamux==31 ? 0 : ccamux+1;
	reg = (reg & (~CCAMUX_BV)) | (ccamux<<0);
	setreg(CC2420_IOCFG1, reg);
	printf("CCAMUX = %02u\n", ccamux);
}
/*---------------------------------------------------------------------------*/
static void
sfd_mux_up(void)
{
	uint16_t reg;
	reg = getreg(CC2420_IOCFG1);
	unsigned sfdmux = (reg & SFDMUX_BV) >> 5;
	sfdmux = sfdmux==31 ? 0 : sfdmux+1;
	reg = (reg & (~SFDMUX_BV)) | (sfdmux<<5);
	setreg(CC2420_IOCFG1, reg);
	printf("SFDMUX = %02u\n", sfdmux);
}
/*---------------------------------------------------------------------------*/
static void
view_rx_statistics(void)
{
	printf("rx = %lu llrx = %lu avg_rssi = %ld avg_lqi = %lu packets_seen = %d packets_read = %d\n",
				 rimestats.rx,
				 rimestats.llrx,
				 (sum_rssi + (long int)rimestats.llrx/2) / (long int)rimestats.llrx,
				 (sum_lqi + rimestats.llrx/2) / rimestats.llrx,
				 cc2420_packets_seen,
				 cc2420_packets_read);

	// reset stat counters
	memset(&rimestats, 0, sizeof(rimestats));
	sum_lqi = 0;
	sum_rssi = 0;
	cc2420_packets_seen = 0;
	cc2420_packets_read = 0;
}
/*---------------------------------------------------------------------------*/
static void
view_failed_rx_statistics(void)
{
	printf("bad crc = %lu bad synch = %lu too long = %lu too short = %lu\n",
				 rimestats.badcrc, 
				 rimestats.badsynch, 
				 rimestats.toolong,
				 rimestats.tooshort);
}
/*---------------------------------------------------------------------------*/
static void
view_tx_power_level(void){
	printf("tx power = %u\n", cc2420_get_txpower());
}
/*---------------------------------------------------------------------------*/
static void
tx_packets_up(void)
{
	// send More packets
	max_tx_packets += 100;
	printf("max tx packets = %d\n", max_tx_packets);
}
/*---------------------------------------------------------------------------*/
static void
tx_packets_down(void)
{
	// send Fewer packets
	max_tx_packets -= 100;
	printf("max tx packets = %d\n", max_tx_packets);
}
/*---------------------------------------------------------------------------*/
static void
this_mode_again(void)
{
	/* start_mode(mode); */
}
/*---------------------------------------------------------------------------*/
static void
payload_len_up(void)
{
	payload_len++;
	printf("payload len =%d\n", payload_len);
}
/*---------------------------------------------------------------------------*/
static void
payload_len_down(void)
{
	payload_len--;
	printf("payload len =%d\n", payload_len);
}
/*---------------------------------------------------------------------------*/
static void
agc_lnamix_gainmode_up(void)
{
	uint16_t reg = getreg(CC2420_AGCCTRL);
	unsigned lna_o = (reg &  (3<<2)) >> 2;
	lna_o = (lna_o + 1) % 4;
	reg = (reg & ~(0x0FFF)) | (reg & ~(3<<2)) | (lna_o << 2);
	setreg(CC2420_AGCCTRL, reg);
	printf("LNAMIX_GAINMODE = %u\n", getreg(CC2420_AGCCTRL) & 0x03);
}
/*---------------------------------------------------------------------------*/
static void
agc_vga_gain_up(void)
{
	uint16_t reg = getreg(CC2420_AGCCTRL);
	unsigned vga_gain = (reg &  (0x7F<<4)) >> 4;
	unsigned vga_gain_oe = (reg & (1<<11)) >>11;
	if(!vga_gain_oe) {
		vga_gain_oe = 1;
		vga_gain = 1;
	} else if (vga_gain < 0x7F) {
		/* vga_gain = (vga_gain<<1) + 1; */
		vga_gain = vga_gain + 1;
	} else {
		vga_gain_oe = 0;
	}
	reg = (reg & ~(0x0FFF)) | (reg & ~(0xFF<<4)) | (vga_gain_oe<<11) | (vga_gain<<4);
	setreg(CC2420_AGCCTRL, reg);
	reg = getreg(CC2420_AGCCTRL);
	vga_gain = (reg &  (0x7F<<4)) >> 4;
	vga_gain_oe = (reg & (1<<11)) >>11;
	printf("LNAMIX_GAINMODE = %u, VGA_GAIN_OE = %u, VGA_GAIN = %u\n",
				 reg & 0x03,
				 vga_gain_oe,
				 vga_gain);
}
/*---------------------------------------------------------------------------*/
static void
debug_hssd(void)
{
#define HSSD_SRC_MSB 12
#define HSSD_SRC_LSB 10
	uint16_t reg = getreg(CC2420_IOCFG1);
	/* uint16_t hssd_src = (reg & (0x0007 << 10)) >> 10; */
	uint16_t hssd_src = FVAL(reg, HSSD_SRC_MSB, HSSD_SRC_LSB);
	hssd_src = (hssd_src + 1) % 8;
	/* reg = (reg & ~(0x0007 << 10)) | (hssd_src << 10); */
	/* reg = SETFD(reg, hssd_src, HSSD_SRC_MSB, HSSD_SRC_LSB); */
	/* reg = SETFDS(reg, FV(HSSD_SRC_MSB, HSSD_SRC_LSB), hssd_src << HSSD_SRC_LSB); */
	reg = SETFDS(reg, FV(HSSD_SRC_MSB, HSSD_SRC_LSB), FDS(hssd_src, HSSD_SRC_LSB));
	setreg(CC2420_IOCFG1, reg);
	reg = getreg(CC2420_IOCFG1);
	printf("HSSD_SRC: %u \n", FVAL(reg, HSSD_SRC_MSB, HSSD_SRC_LSB));
}

/*---------------------------------------------------------------------------*/
static void
debug_analog(void)
{
#define ATESTMOD_MODE_MSB 3
#define ATESTMOD_MODE_LSB 0
#define ATESTMOD_PD_BV (0x0001 <<4)
	uint16_t reg = getreg(CC2420_TOPTST);
	uint16_t atestmod_pd = (reg & ATESTMOD_PD_BV) >> 4;
	uint16_t atestmod_mode = FVAL(reg, ATESTMOD_MODE_MSB, ATESTMOD_MODE_LSB);

	if (atestmod_pd) {
		// Power is down: start Atest and start from mode 0
		atestmod_pd = 0;
		atestmod_mode = 0;
	} else {
		// next mode
		++atestmod_mode;
		if (atestmod_mode > 8) {
			// Last mode: stop Atest
			atestmod_pd = 1;
		}
	}

	reg = (reg & ~0x0001F) | (atestmod_pd << 4);
	reg = SETFD(reg, atestmod_mode, ATESTMOD_MODE_MSB, ATESTMOD_MODE_LSB);
	setreg(CC2420_TOPTST, reg);
	reg = getreg(CC2420_TOPTST);
	printf("ATESTMOD_PD: %u ATESTMOD_MODE: %u\n",
				 (reg & (0x0001 << 4)) >> 4,
				 FVAL(reg, ATESTMOD_MODE_MSB, ATESTMOD_MODE_LSB));
}
/*---------------------------------------------------------------------------*/
static void
reverse_phase(void)
{
#define MOD_MODE_BIT 4
#define MOD_MODE_BV (1<<MOD_MODE_BIT)
	uint16_t reg = getreg(CC2420_MDMCTRL1);
	if(reg & MOD_MODE_BV) {
		reg &= ~MOD_MODE_BV;
	} else {
		reg |= MOD_MODE_BV;
	}
	setreg(CC2420_MDMCTRL1, reg);
	reg = getreg(CC2420_MDMCTRL1);
	printf("MODULATION_MODE: %u \n", (reg & MOD_MODE_BV)>>MOD_MODE_BIT);
}
/*---------------------------------------------------------------------------*/
static void
reverse_syncword(void)
{
		/* test: send with non-standard sync word to avoid causing synchronization */
	uint16_t reg;
	reg = getreg(CC2420_SYNCWORD);
	/* reg = (reg & 0x00FF) | (~reg  & 0xFF00); */
	
	if (reg == 0xA70F) {reg = 0xA60F;}
	else if (reg == 0xA60F) {reg = 0x2E88;}
	else {reg = 0xA60F;}
	setreg(CC2420_SYNCWORD, reg);
	reg = getreg(CC2420_SYNCWORD);
	printf("Syncword: 0x%X\n", reg);
}
/*---------------------------------------------------------------------------*/
static void
len_hdr_up(void)
{
	len_hdr = (len_hdr+1) % 128;
	printf("len_hdr = %u\n", len_hdr);
}
/*---------------------------------------------------------------------------*/
const struct command command_table[] =	{
	{'n', next_mode},
	{'p', previous_mode},
	{'e', reboot},
	{'u', power_up},
	{'d', power_down},
	{'h', tx_frequency_up},
	{'l', tx_frequency_down},
	{'H', rtimer_frequency_up},
	{'L', rtimer_frequency_down},
	{'r', rssi},
	{'+', channel_up},
	{'-', channel_down},
	{'>', frequency_up},
	{'<', frequency_down},
	{'c', cca_mux_up},
	{'s', sfd_mux_up},
	{'v', view_rx_statistics},
	{'V', view_failed_rx_statistics},
	{'w', view_tx_power_level},
	{'m', tx_packets_up},
	{'f', tx_packets_down},
	{'a', this_mode_again},
	{'Y', payload_len_up},
	{'y', payload_len_down},
	{'g', agc_lnamix_gainmode_up},
	{'G', agc_vga_gain_up},
	/* {'D', debug_hssd}, */
	{'A', debug_analog},
	{'P', reverse_phase},
	{'S', reverse_syncword},
	{'R', len_hdr_up},
	{'\0', NULL},
};

void
do_command(char ch)
{
	if(ch >= '0' && ch <= '9' && callback != NULL) {
		if(callback != NULL) {
			callback(ch - '0');
		} else {
			printf("no callback found\n");
		}
		return;
	} else {
	const struct command *ptr = &command_table[0];
	while(ptr->f != NULL) {
		if(ch == ptr->ch) {
			ptr->f();
			return;
		} 
		++ptr;
	}
	
	printf("unkown command: %c\n", ch);
	}
}

void
commands_set_callback(void (*f)(int))
{
	callback = f;
}

