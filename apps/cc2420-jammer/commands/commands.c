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

long int sum_rssi;
long unsigned sum_lqi;
int min_rssi = 15, max_rssi = -55;
unsigned min_lqi = 108, max_lqi = 0;
uint8_t len_hdr = 127;

extern int jam_ena;
extern const unsigned char tx_power_level[10];
extern int cc2420_packets_seen, cc2420_packets_read;

unsigned getreg(enum cc2420_register regname);
void setreg(enum cc2420_register regname, unsigned value);
uint16_t cc2420_get_frequency(void);
int cc2420_set_frequency(uint16_t f);

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
	printf("rx = %lu llrx = %lu avg_rssi = %ld[%d,%d] avg_lqi = %lu[%u,%u] pkts_seen = %d pkts_read = %d\n",
				 rimestats.rx,
				 rimestats.llrx,
				 (sum_rssi - (long int)rimestats.llrx/2) / (long int)rimestats.llrx,
				 min_rssi, max_rssi,
				 (sum_lqi + rimestats.llrx/2) / rimestats.llrx,
				 min_lqi, max_lqi,
				 cc2420_packets_seen,
				 cc2420_packets_read);

	// reset stat counters
	memset(&rimestats, 0, sizeof(rimestats));
	sum_lqi = 0;
	sum_rssi = 0;
	min_rssi = 15;
	max_rssi = -55;
	min_lqi = 108;
	max_lqi = 0;
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
this_mode_again(void)
{
	/* start_mode(mode); */
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
		/* vga_gain = 0x40; // switches on first gain stage */
	} else if (vga_gain < 0x7F) {
		vga_gain = vga_gain + 1;
		/* vga_gain = 0x40 | (vga_gain>>1); // switches on successive gain stages */
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
/* #define HSSD_SRC_MSB 12 */
/* #define HSSD_SRC_LSB 10 */
/* 	uint16_t reg = getreg(CC2420_IOCFG1); */
/* 	/\* uint16_t hssd_src = (reg & (0x0007 << 10)) >> 10; *\/ */
/* 	uint16_t hssd_src = FVAL(reg, HSSD_SRC_MSB, HSSD_SRC_LSB); */
/* 	hssd_src = (hssd_src + 1) % 8; */
/* 	/\* reg = (reg & ~(0x0007 << 10)) | (hssd_src << 10); *\/ */
/* 	/\* reg = SETFD(reg, hssd_src, HSSD_SRC_MSB, HSSD_SRC_LSB); *\/ */
/* 	/\* reg = SETFDS(reg, FV(HSSD_SRC_MSB, HSSD_SRC_LSB), hssd_src << HSSD_SRC_LSB); *\/ */
/* 	reg = SETFDS(reg, FV(HSSD_SRC_MSB, HSSD_SRC_LSB), FDS(hssd_src, HSSD_SRC_LSB)); */
/* 	setreg(CC2420_IOCFG1, reg); */
/* 	reg = getreg(CC2420_IOCFG1); */
/* 	printf("HSSD_SRC: %u \n", FVAL(reg, HSSD_SRC_MSB, HSSD_SRC_LSB)); */
	printf("Command disabled\n");
}

/*---------------------------------------------------------------------------*/
static void
debug_analog(void)
{
#define ATESTMOD_MODE_MSB 3
#define ATESTMOD_MODE_LSB 0
#define ATESTMOD_PD_BV (0x0001 <<4)
#define ATESTMOD_START 4 // first invocation jumpstarts to mode [0,8]

	uint16_t reg = getreg(CC2420_TOPTST);
	uint16_t atestmod_pd = (reg & ATESTMOD_PD_BV) >> 4;
	uint16_t atestmod_mode = FVAL(reg, ATESTMOD_MODE_MSB, ATESTMOD_MODE_LSB);

	if (atestmod_pd) {
		// Power is down: start Atest and start from mode 0
		atestmod_pd = 0;
		atestmod_mode = ATESTMOD_START; // start from mode 4
	} else {
		// next mode
		atestmod_mode = (atestmod_mode+1) % 9;
		// Last mode: stop Atest
		if (atestmod_mode == ATESTMOD_START)
			atestmod_pd++;
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
	
	/* if (reg == 0xA70F) {reg = 0xA60F;} */
	/* else if (reg == 0xA60F) {reg = 0x2E88;} */
	/* else {reg = 0xA60F;} */
	if (reg == 0xA70F) {reg = 0xA7FF;}
	else if (reg == 0xA7FF) {reg = 0xA700;}
	else {reg = 0xA70F;}
	setreg(CC2420_SYNCWORD, reg);
	reg = getreg(CC2420_SYNCWORD);
	printf("Syncword: 0x%X\n", reg);
}
/*---------------------------------------------------------------------------*/
static void
preamble_size_down(void)
{
#define PREAMBLE_LENGTH_MSB 3
#define PREAMBLE_LENGTH_LSB 0
	uint8_t reg = getreg(CC2420_MDMCTRL0);
	uint8_t preamble_len = FVAL(reg, PREAMBLE_LENGTH_MSB, PREAMBLE_LENGTH_LSB);
	preamble_len = preamble_len>0 ? preamble_len-1 : 15;
	setreg(CC2420_MDMCTRL0, SETFD(reg, preamble_len, PREAMBLE_LENGTH_MSB, PREAMBLE_LENGTH_LSB));
	printf("Preamble length: %u bytes\n", preamble_len+1);
}
/*---------------------------------------------------------------------------*/
const struct command command_table[] =	{
	{'n', '\0', next_mode},
	{'p', '\0', previous_mode},
	{'e', '\0', reboot},
	{'u', '\0', power_up},
	{'d', '\0', power_down},
	{'r', '\0', rssi},
	{'+', '\0', channel_up},
	{'-', '\0', channel_down},
	{'>', '\0', frequency_up},
	{'<', '\0', frequency_down},
	{'c', '\0', cca_mux_up},
	{'s', '\0', sfd_mux_up},
	{'v', '\0', view_rx_statistics},
	{'V', '\0', view_failed_rx_statistics},
	{'w', '\0', view_tx_power_level},
	{'a', '\0', this_mode_again},
	{'g', '\0', agc_lnamix_gainmode_up},
	{'G', '\0', agc_vga_gain_up},
	{'D', '\0', debug_hssd},
	{'A', '\0', debug_analog},
	{'P', '\0', reverse_phase},
	{'S', '\0', reverse_syncword},
	{'E', '\0', preamble_size_down},
	{'\0', '\0', NULL},
};

void
do_command(char *s)
{
	static char last_cmd[2];
	static char last_last_cmd[2];

	if(s[0] == '\0') {
		s = &last_cmd[0];
	} else if(s[0] == '~') {
		s = &last_last_cmd[0];
	} else {
		strncpy(last_last_cmd, last_cmd, sizeof(last_last_cmd));
		strncpy(last_cmd, s, sizeof(last_cmd));
	}

	if(s[0] >= '0' && s[0] <= '9' && callback != NULL) {
		callback(s[0] - '0');
		return;
	}
	else if(s[1] != '\0' && (s[0]=='+' || s[0]=='-' || s[0]=='*' || s[0]=='/' || s[0]=='<' || s[0]=='>' || s[0]=='^' || s[0]=='\''))
		var_update(s[0], s[1]);
	else {
		const struct command *ptr = &command_table[0];
		while(ptr->f != NULL) {
			if(s[0] == ptr->ch1) {
				ptr->f();
				return;
			}
			++ptr;
		}
		printf("unkown command: %c\n", s[0]);
	}
}

void
commands_set_callback(void (*f)(int))
{
	callback = f;
}

void
var_update(char op, char var)
{
	const struct variable *vp = &user_variable_list[0];
	uint8_t *u8;
	uint16_t *u16;
	uint32_t *u32;
	while(vp->ch != '\0') {
		if(vp->ch == var) {
			switch(vp->width) {
			case 1:
				u8 = &(vp->n->u8);
				OP(*u8, op);
				*u8 = (*u8) % (vp->ceiling - vp->floor + 1);
				printf("%s=%u\n", vp->long_name, *u8);
				break;
			case 2:
				u16 = &(vp->n->u16);
				OP(*u16, op);
				*u16 = (*u16) % (vp->ceiling - vp->floor + 1);
				printf("%s=%u\n", vp->long_name, *u16);
				break;
			case 4:
				u32 = &(vp->n->u32);
				OP(*u32, op);
				*u32 = (*u32) % (vp->ceiling - vp->floor + 1);
				printf("%s=%lu\n", vp->long_name, *u32);
				break;
			default:;
			}
		}
		vp++;
	}
}
