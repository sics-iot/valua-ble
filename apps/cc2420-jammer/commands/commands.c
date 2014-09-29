#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "dev/cc2420/cc2420.h"
#include "dev/cc2420/cc2420_const.h"
#include "net/rime/rime.h"
#include "dev/watchdog.h"

#include "commands.h"

long int sum_rssi;
long unsigned sum_lqi;
int min_rssi = 15, max_rssi = -55;
unsigned min_lqi = 108, max_lqi = 0;

static const struct variable *user_vars;

extern int jam_ena;
extern const unsigned char tx_power_level[10];
extern int cc2420_packets_seen, cc2420_packets_read;

static void (*callback)(int v);

/* TODO: a battery voltage reading command: scan through 32 BATTMON_VOLTAGE levels to find level at which the BATTMON_OK toggles, the convert to volt based formular in datasheet */

struct field
{
	char ch;
	const char *name;
	enum cc2420_register addr;
	unsigned msb;
	unsigned lsb;
};

/*---------------------------------------------------------------------------*/
struct command
{
	const char ch;
	const char *name;
	void (*f)(void);
};

static void help(void);

/*---------------------------------------------------------------------------*/
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
	/* DAC override values, 5-bit */
	{'i', "DAC_I_O", CC2420_DACTST, 11, 5},
	{'q', "DAC_I_Q", CC2420_DACTST, 4, 0},
	/* LNA and mixer gain: 0=automatic, 1,2,3=low,mid,high */
	{'g', "LNAMIX_GAINMODE_O", CC2420_AGCCTRL, 3, 2},
	/* VGA gain override: 0=automatic, 1=manual  */
	{'V', "VGA_GAIN_OE", CC2420_AGCCTRL, 11, 11},
	/* VGA gain: 0-127, non-linear, non-monotonic relationship btw. value and gain  */
	/* read <= current gain, write => manual gain */
	{'v', "VGA_GAIN", CC2420_AGCCTRL, 10, 4},
	/* Modulation mode: 0=normal, 1=reverse phase */
	{'P', "MOD_MODE", CC2420_MDMCTRL1, 4, 4},
	/* Preamble length in bytes: 0-15=1-16 */
	{'E', "PREAMBLE_LENGTH", CC2420_MDMCTRL0, 3, 0},
	/* Carrier frequency in 1 MHz steps: 0-1023=2048-3071 MHz, 357=2405 MHz */
	{'f', "FREQ", CC2420_FSCTRL, 9, 0},
	/* Output PA level: 3=-25dB, 7=-15dB, 11=-10dB 15=-7dB 19=-5dB 23=-3dB 27=-1dB 31=0dB */
	{'p', "PA_LEVEL", CC2420_TXCTRL, 4, 0},
	/* 16-bit Syncword: 0xA70F="00" preamble, 0xA7FF="0" preamble, 0xA700="000" preamble */
	{'S', "SYNCWORD", CC2420_SYNCWORD, 15, 0},
	/* The time after the last chip in the packet is sent, and the TXRX switch is disabled. In μs. */
	{'t', "TC_TXEND2SWITCH[2:0]", CC2420_FSMTC, 5, 3},
	/* The time after the last chip in the packet is sent, and the PA is set in power-down. Also the time at which the modulator is disabled. In μs. */
	{'T', "TC_TXEND2PAOFF[2:0]", CC2420_FSMTC, 2, 0},
	{'A', "ATESTMOD_PD", CC2420_TOPTST, 4, 4},
	/* ATEST mode: 0-7, we skip mode 8 */
	{'a', "ATESTMOD_MODE[2:0]", CC2420_TOPTST, 2, 0},
	{'K', "ADR_DECODE", CC2420_MDMCTRL0, 11, 11},
	{'k', "AUTOACK", CC2420_MDMCTRL0, 4, 4},
	{'B', "BATTMON_EN", CC2420_BATTMON, 5, 5},
	// Vtoggle=1.25/27*(72-BATTMON_VOLTAGE) (BATTMON_VOLTAGE=[0, 31] => Vtoggle=[1.90, 3.33])
	{'b', "BATTMON_VOLTAGE", CC2420_BATTMON, 4, 0},
	{'O', "BATTMON_OK", CC2420_BATTMON, 6, 6}, // read-only
	{'F', "FIFO_THR", CC2420_IOCFG0, 6, 0},
	{'I', "ADC_I", CC2420_ADCTST, 14, 8},
	{'Q', "ADC_Q", CC2420_ADCTST, 6, 0},
	{'e', "RESETn", CC2420_MAIN, 15, 15}, // wrtie '0' to reset radio
	{'R', "RESERVED", CC2420_RESERVED, 15, 0},
};

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
/* Change my MAC address, can be used to alter ACK behaviour */
static void
mac_update(void)
{
	unsigned shortaddr;
	CC2420_READ_RAM(&shortaddr,CC2420RAM_SHORTADDR, 2);
	// update higher byte only, must correspond to linkaddr_node_addr.u8[0] if Rime stack used
	shortaddr = (shortaddr+(1<<8))%(4<<8);
	CC2420_WRITE_RAM(&shortaddr,CC2420RAM_SHORTADDR, 2);
	printf("16-bit MAC address: 0x%04X\n", shortaddr);
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

static void
status(void)
{
  uint8_t status;
  CC2420_GET_STATUS(status);
	printf("0x%01X\n", status);
}

/* Show current battery level */
/* Vtoggle=1.25/27*(72-BATTMON_VOLTAGE) */
static void
battery_level(void)
{
	unsigned lv, lv_orig;
	unsigned ok;
	unsigned reg;

	reg = getreg(CC2420_BATTMON);
	lv_orig = FV(reg, 4, 0);
	setreg(CC2420_BATTMON, SETFV(reg, 1, 5, 5)); // BATTMON_EN=1
	reg = getreg(CC2420_BATTMON);
	for(lv=0;lv<32;lv++) {
		setreg(CC2420_BATTMON, SETFV(reg, lv, 4, 0));
		reg = getreg(CC2420_BATTMON);
		ok = FV(reg, 6, 6);
		/* printf("%u: %u\n", lv, ok); */
		if(ok) break;
	}

	printf("Battery level is between %u and %u\n", lv-1, lv);
	setreg(CC2420_BATTMON, SETFV(reg, 0, 5, 5)); // BATTMON_EN=0
	setreg(CC2420_BATTMON, SETFV(reg, lv_orig, 4, 0)); // restore original monitor level
}

static const struct command command_table[] =	{
	{'h', "Help", help},
	{'e', "Reboot", reboot},
	{'w', "TX power level", view_tx_power_level},
	{'u', "TX power+", power_up},
	{'d', "TX power-", power_down},
	{'r', "RSSI", rssi},
	{'+', "Channel+", channel_up},
	{'-', "Channel-", channel_down},
	{'v', "Successful pkt receptions", view_rx_statistics},
	{'V', "Unsuccessful pkt receptions", view_failed_rx_statistics},
	{'H', "Enter HSSD mode", debug_hssd},
	{'A', "Enaable analog test mode", debug_analog},
	{'S', "Reverse sync word", reverse_syncword},
	{'M', "MAC address update", mac_update},
	{'L', "Show all registers", show_all_registers},
	{'s', "CC2420 status byte", status},
	{'b', "Battery level", battery_level}
};

/* Help message: available commands */
static void
help(void)
{
	const struct command *cmd;
	const struct field *fp;
	printf("Single character commands:\n");
	for(cmd=command_table; cmd < command_table + sizeof(command_table)/sizeof(struct command); cmd++)
		printf("%c %s\n", cmd->ch, cmd->name);

	printf("---------\n");
	printf("CC2420 field update commands <cmd name width(bits)>:\n");
	for(fp=field_list; fp < field_list + sizeof(field_list)/sizeof(struct field); fp++)
		printf("%c %s %u\n", fp->ch, fp->name, fp->msb - fp->lsb +1);
}
/*-----------------------------------------------------------------------------*/
static void
exec_command(char c)
{
	const struct command *ptr = &command_table[0];
	for(;ptr<command_table+sizeof(command_table)/sizeof(struct command);ptr++) {
		if(c == ptr->ch) {
			ptr->f();
			return;
		}
	}
}


static unsigned
hexstr_to_unsigned(const char *s)
{
	unsigned n,m;

	for(n=0;(*s>='0' && *s<='9')||(*s>='a' && *s<='f')||(*s>='A' && *s<='F');s++) {
		if(*s>='0' && *s<='9') {m=*s-'0';}
		else if(*s>='a' && *s<='f') {m=*s-'a'+10;}
		else {m=*s-'A'+10;}
		n=n*16+m;
	}

	return n;
}

static void
print_reg(const char* hex_str)
{
	uint16_t reg;
	enum cc2420_register addr;
	char bits[17]; //16 bits + '\0'
	
	addr = hexstr_to_unsigned(hex_str);
  /* if(addr >= CC2420_MAIN && addr <= CC2420_RESERVED) { */
  if(addr >= CC2420_MAIN && addr <= CC2420_RESERVED) {
		reg = getreg(addr);
		printf("0x%02X: 0x%04X %s\n", addr, reg,	u16_to_bits(reg, bits));
	} else if(addr < CC2420_foo) {
		printf("Strobe 0x%02X\n", addr);
		CC2420_STROBE(addr);
	} else {
		printf("Unknown register: 0x%s\n", hex_str);
	}
}

void
do_command(char *s)
{
	static char last_cmd[2];
	static char last_last_cmd[2];

	/* update command history */
	if(s[0] == '\0') {s = &last_cmd[0];}
	else {
		if(s[0] == '~') {strncpy(s, last_last_cmd, sizeof(last_last_cmd));}
		strncpy(last_last_cmd, last_cmd, sizeof(last_last_cmd));
		strncpy(last_cmd, s, sizeof(last_cmd));
	}

	if(s[0] >= '0' && s[0] <= '9' && callback != NULL)	{callback(s[0] - '0');}
	else if (s[0]==s[1]) {exec_command(s[0]);}
	else if((s[0]=='+' || s[0]=='-' || s[0]=='*' || s[0]=='/' || s[0]=='<' || s[0]=='>' || s[0]=='^' || s[0]=='\'')) {var_update(s[0], s[1]);}
	else if(s[1]=='+' || s[1]=='-' || s[1]=='<' || s[1]=='>' || s[1]=='\0') {field_update(s[0], s[1]);}
	else if(s[0]=='x') {print_reg(&s[1]);}
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
	if(!user_vars) return;

	const struct variable *vp = user_vars;
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
void
field_update(char c, char op)
{
	const struct field *fp;
	unsigned reg, fv;
	
	for(fp=&field_list[0]; fp < &field_list[0] + sizeof(field_list)/sizeof(struct field); fp++) {
		if(fp->ch == c) {
			reg = getreg(fp->addr);
			fv = FV(reg, fp->msb, fp->lsb);
			/* skip update when no op */
			if(op != '\0') {
				OP(fv, op);
				fv = fv % (0x1<<(fp->msb - fp->lsb +1));
				reg = SETFV(reg, fv, fp->msb, fp->lsb);
				setreg(fp->addr, reg);
				fv = FV(getreg(fp->addr), fp->msb, fp->lsb);
			}
			printf("%s=%u\n", fp->name, fv);
			return;
		}
	}
}

/* Initialize pointer to user variables */
void
commands_set_user_vars(const struct variable *vars)
{
	if(!user_vars) printf("User variables initialized\n");
	else printf("User variables updated\n");
	user_vars = vars;

	// debug info: variables command, name and width
	const struct variable *vp;
	printf("Cmd Variable      Width\n");
	for(vp=user_vars;vp->ch != '\0';vp++) {
		printf("%c   %s      %d\n", vp->ch, vp->long_name, vp->width);
	}
}
