#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "dev/cc2420.h"
#include "dev/cc2420_const.h"
#include "net/rime.h"
#include "dev/watchdog.h"

#include "commands.h"

long int sum_rssi;
long unsigned sum_lqi;
int min_rssi = 15, max_rssi = -55;
unsigned min_lqi = 108, max_lqi = 0;

extern int jam_ena;
extern const unsigned char tx_power_level[10];
extern int cc2420_packets_seen, cc2420_packets_read;

static void (*callback)(int v);

// Individual command handlers
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
agc_lnamix_gainmode_up(void)
{
#define LNAMIX_GAINMODE_O_MSB 3
#define LNAMIX_GAINMODE_O_LSB 2
#define LNAMIX_GAINMODE_MSB 1
#define LNAMIX_GAINMODE_LSB 0

	uint16_t reg = getreg(CC2420_AGCCTRL);
	unsigned lna_o = FV(reg, LNAMIX_GAINMODE_O_MSB, LNAMIX_GAINMODE_O_LSB);
	lna_o = (lna_o + 1) % 4;
	reg = SETFV(reg, lna_o, LNAMIX_GAINMODE_O_MSB, LNAMIX_GAINMODE_O_LSB);
	setreg(CC2420_AGCCTRL, reg);
	printf("LNAMIX_GAINMODE = %u\n", FV(getreg(CC2420_AGCCTRL), LNAMIX_GAINMODE_MSB, LNAMIX_GAINMODE_LSB));
}
/*---------------------------------------------------------------------------*/
static void
agc_vga_gain_up(void)
{
#define VGA_GAIN_OE_MSB 11 
#define VGA_GAIN_OE_LSB 11
#define VGA_GAIN_MSB 10
#define VGA_GAIN_LSB 4

	uint16_t reg = getreg(CC2420_AGCCTRL);
	/* unsigned vga_gain = (reg &  (0x7F<<4)) >> 4; */
	/* unsigned vga_gain_oe = (reg & (1<<11)) >>11; */
	unsigned vga_gain_oe = FV(reg, VGA_GAIN_OE_MSB, VGA_GAIN_OE_LSB);
	unsigned vga_gain = FV(reg, VGA_GAIN_MSB, VGA_GAIN_LSB);
	if(!vga_gain_oe) {
		vga_gain_oe = 1;
		vga_gain = 1;
	} else if (vga_gain < 0x7F) {
		vga_gain = vga_gain + 1;
	} else {
		vga_gain_oe = 0;
	}
	reg = SETFV(reg, vga_gain_oe, VGA_GAIN_OE_MSB, VGA_GAIN_OE_LSB);
	reg = SETFV(reg, vga_gain, VGA_GAIN_MSB, VGA_GAIN_LSB);
	setreg(CC2420_AGCCTRL, reg);
	reg = getreg(CC2420_AGCCTRL);
	vga_gain_oe = FV(reg, VGA_GAIN_OE_MSB, VGA_GAIN_OE_LSB);
	vga_gain = FV(reg, VGA_GAIN_MSB, VGA_GAIN_LSB);
	printf("LNAMIX_GAINMODE = %u, VGA_GAIN_OE = %u, VGA_GAIN = %u\n",
		FV(reg, LNAMIX_GAINMODE_MSB, LNAMIX_GAINMODE_LSB),
		vga_gain_oe,
		vga_gain);
}
/*---------------------------------------------------------------------------*/
/* High speed serial debug mode */
// 8 modes: 0: Off (default) 1: AGC status output 2: ADC I/Q output 3: down mixer I/Q output 4: na 5: na 6: adc input 7: dac input
static void
debug_hssd(void)
{
#define HSSD_SRC_MSB 12
#define HSSD_SRC_LSB 10
	uint16_t reg = getreg(CC2420_IOCFG1);
	uint16_t hssd_src = FV(reg, HSSD_SRC_MSB, HSSD_SRC_LSB);
	hssd_src = (hssd_src + 1) % 8;
	if(hssd_src == 0) CC2420_ENABLE_FIFOP_INT();
	else	CC2420_DISABLE_FIFOP_INT();

	CC2420_FIFO_PORT(DIR) &= ~BV(CC2420_FIFO_PIN);
	reg = SETFV(reg, hssd_src, HSSD_SRC_MSB, HSSD_SRC_LSB);
	setreg(CC2420_IOCFG1, reg);
	reg = getreg(CC2420_IOCFG1);
	printf("HSSD_SRC: %u \n", FV(reg, HSSD_SRC_MSB, HSSD_SRC_LSB));
}

/*---------------------------------------------------------------------------*/
static void
debug_analog(void)
{
#define ATESTMOD_MODE_MSB 3
#define ATESTMOD_MODE_LSB 0
#define ATESTMOD_PD_MSB 4
#define ATESTMOD_PD_LSB 4
#define ATESTMOD_START 4 // first invocation jumpstarts to mode [0,8]

	uint16_t reg = getreg(CC2420_TOPTST);
	uint16_t atestmod_pd = FV(reg, ATESTMOD_PD_MSB, ATESTMOD_PD_LSB);
	uint16_t atestmod_mode = FV(reg, ATESTMOD_MODE_MSB, ATESTMOD_MODE_LSB);

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

	reg = SETFV(reg, atestmod_pd, ATESTMOD_PD_MSB, ATESTMOD_PD_LSB);
	reg = SETFV(reg, atestmod_mode, ATESTMOD_MODE_MSB, ATESTMOD_MODE_LSB);
	setreg(CC2420_TOPTST, reg);
	reg = getreg(CC2420_TOPTST);
	printf("ATESTMOD_PD: %u ATESTMOD_MODE: %u\n",
				 FV(reg, ATESTMOD_PD_MSB, ATESTMOD_PD_LSB),
				 FV(reg, ATESTMOD_MODE_MSB, ATESTMOD_MODE_LSB));
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
mac_update(void)
{
	unsigned shortaddr;
	CC2420_READ_RAM(&shortaddr,CC2420RAM_SHORTADDR, 2);
	shortaddr = (shortaddr+1) & 0x000F;
	CC2420_WRITE_RAM(&shortaddr,CC2420RAM_SHORTADDR, 2);
	printf("shortaddr: %u\n", shortaddr);
}

/*---------------------------------------------------------------------------*/
static char*
u16_to_bits(uint16_t n, char bits[])
{
	int i;

	for(i=0;i<16;i++) {
		bits[i] = (char)((n>>(15-i) & 1) + '0');
	}

	bits[i] = '\0';

	return bits;
}

static void
show_all_registers(void)
{
	uint16_t reg;
	enum cc2420_register addr;
	char bits[17]; //16 bits + '\0'

	for (addr = CC2420_MAIN;addr <= CC2420_RESERVED;addr++) {
		reg = getreg(addr);
		printf("0x%02X: 0x%04X %s\n", addr, reg,	u16_to_bits(reg, bits));
	}
}

/*---------------------------------------------------------------------------*/
const struct command command_table[] =	{
	{'e', '\0', reboot},
	{'u', '\0', power_up},
	{'d', '\0', power_down},
	{'r', '\0', rssi},
	{'+', '\0', channel_up},
	{'-', '\0', channel_down},
	{'>', '\0', frequency_up},
	{'<', '\0', frequency_down},
	{'v', '\0', view_rx_statistics},
	{'V', '\0', view_failed_rx_statistics},
	{'w', '\0', view_tx_power_level},
	{'g', '\0', agc_lnamix_gainmode_up},
	{'G', '\0', agc_vga_gain_up},
	{'H', '\0', debug_hssd},
	{'A', '\0', debug_analog},
	{'S', '\0', reverse_syncword},
	{'M', '\0', mac_update},
	{'L', '\0', show_all_registers},
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
	else if((s[0]=='+' || s[0]=='-' || s[0]=='*' || s[0]=='/' || s[0]=='<' || s[0]=='>' || s[0]=='^' || s[0]=='\'') && s[1] != '\0')
		var_update(s[0], s[1]);
	else if(s[1]=='+' || s[1]=='-' || s[1]=='<' || s[1]=='>' || s[1]==s[0])
		field_update(s[0], s[1]);
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

/*-----------------------------------------------------------------------------*/ 
// Generic user variable update operations
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

/*-----------------------------------------------------------------------------*/ 
// Generic CC2420 field update operations
struct field
{
	char cmd;
	const char *name;
	enum cc2420_register addr;
	unsigned msb;
	unsigned lsb;
};
 
const static struct field field_list[] = {
	{'c', "CCAMUX", CC2420_IOCFG1, 4, 0},
	{'s', "SFDMUX", CC2420_IOCFG1, 9, 5},
/* the TX DACs data source is selected by DAC_SRC according to: */
/* 0: Normal operation (from modulator). */
/* 1: The DAC_I_O and DAC_Q_O override values below.- */
/* 2: From ADC, most significant bits */
/* 3: I/Q after digital down mixing and channel filtering. */
/* 4: Full-spectrum White Noise (from CRC) */
/* 5: From ADC, least significant bits */
/* 6: RSSI / Cordic Magnitude Output */
/* 7: HSSD module. */
	{'d', "DAC_SRC", CC2420_DACTST, 14, 12},
	{'g', "LNAMIX_GAINMODE_O", CC2420_AGCCTRL, 3, 2},
	{'G', "VGA_GAIN", CC2420_AGCCTRL, 10, 4},
/* Modulation mode: 0=normal, 1=reverse phase */
	{'P', "MOD_MODE", CC2420_MDMCTRL1, 4, 4},
/* Preamble length 0-15 => 1-16 bytes */
	{'E', "PREAMBLE_LENGTH", CC2420_MDMCTRL0, 3, 0},
};

void
field_update(char c, char op)
{
	const struct field *fp;
	unsigned reg, fv;
	
	for(fp=&field_list[0]; fp < &field_list[0] + sizeof(field_list)/sizeof(struct field); fp++) {
		if(fp->cmd == c) {
			reg = getreg(fp->addr);
			fv = FV(reg, fp->msb, fp->lsb);
			if(c != op) {
				OP(fv, op);
				fv = fv % (0x1<<(fp->msb - fp->lsb +1));
				reg = SETFV(reg, fv, fp->msb, fp->lsb);
				setreg(fp->addr, reg);
			}
			printf("%s=%u\n", fp->name, fv);
		}
	}
}
