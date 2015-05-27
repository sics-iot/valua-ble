#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "commands.h"

static const struct variable *user_vars;

static void (*callback)(int v);

struct field
{
	char ch;
	const char *name;
	uint8_t addr;
	unsigned msb;
	unsigned lsb;
};

/*---------------------------------------------------------------------------*/
const static struct field field_list[] = {
	{'f', "FOO", 0xFE, 4, 0},
};

// Individual command handlers
/* static void */
/* reboot(void) */
/* { */
/* 	watchdog_reboot(); */
/* } */
/*---------------------------------------------------------------------------*/
/* static void */
/* power_up(void) */
/* { */
/* 	int power = cc2420_get_txpower(); */
/* 	if (power < 31) { */
/* 		cc2420_set_txpower(power+1); */
/* 	} */
/* 	iprintf("tx power = %u\n", cc2420_get_txpower()); */
/* } */
/* /\*---------------------------------------------------------------------------*\/ */
/* static void */
/* power_down(void) */
/* { */
/* 	int power = cc2420_get_txpower(); */
/* 	if (power > 0) { */
/* 		cc2420_set_txpower(power-1); */
/* 	} */
/* 	iprintf("tx power = %u\n", cc2420_get_txpower()); */
/* } */
/*---------------------------------------------------------------------------*/
/* static void */
/* rssi(void) */
/* { */
/* 	int8_t val = cc2420_rssi(); */
/* 	uint16_t reg = getreg(CC2420_AGCCTRL); */
/* 	unsigned lna = reg & 3; */
/* 	unsigned vga_gain = (reg &  (0x7F<<4)) >> 4; */
/* 	iprintf("RSSI = %d LNAMIX_GAINMODE = %u VGA_GAIN = %u\n", val, lna, vga_gain); */
/* } */
/*---------------------------------------------------------------------------*/
static const struct command *command_table_base;

void
commands_set_command_table(const struct command *ptr)
{
	command_table_base = ptr;
}
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/* static void */
/* view_rx_statistics(void) */
/* { */
/* 	iprintf("rx = %lu llrx = %lu avg_rssi = %ld[%d,%d] avg_lqi = %lu[%u,%u] pkts_seen = %d pkts_read = %d\n", */
/* 				 rimestats.rx, */
/* 				 rimestats.llrx, */
/* 				 (sum_rssi - (long int)rimestats.llrx/2) / (long int)rimestats.llrx, */
/* 				 min_rssi, max_rssi, */
/* 				 (sum_lqi + rimestats.llrx/2) / rimestats.llrx, */
/* 				 min_lqi, max_lqi, */
/* 				 cc2420_packets_seen, */
/* 				 cc2420_packets_read); */

/* 	// reset stat counters */
/* 	memset(&rimestats, 0, sizeof(rimestats)); */
/* 	sum_lqi = 0; */
/* 	sum_rssi = 0; */
/* 	min_rssi = 15; */
/* 	max_rssi = -55; */
/* 	min_lqi = 108; */
/* 	max_lqi = 0; */
/* 	cc2420_packets_seen = 0; */
/* 	cc2420_packets_read = 0; */
/* } */
/* /\*---------------------------------------------------------------------------*\/ */
/* static void */
/* view_failed_rx_statistics(void) */
/* { */
/* 	iprintf("bad crc = %lu bad synch = %lu too long = %lu too short = %lu\n", */
/* 				 rimestats.badcrc,  */
/* 				 rimestats.badsynch,  */
/* 				 rimestats.toolong, */
/* 				 rimestats.tooshort); */
/* } */
/*---------------------------------------------------------------------------*/
/* static void */
/* view_tx_power_level(void){ */
/* 	iprintf("tx power = %u\n", cc2420_get_txpower()); */
/* } */
/*---------------------------------------------------------------------------*/
/* High speed serial debug mode */
// 8 modes: 0: Off (default) 1: AGC status output 2: Output ADC I/Q 3: Output down mixer I/Q 4: NA 5: NA 6: Input ADC I/Q 7: Input DAC I/Q
/* static void */
/* debug_hssd(void) */
/* { */
/* #define HSSD_SRC_MSB 12 */
/* #define HSSD_SRC_LSB 10 */
/* 	uint16_t reg = getreg(CC2420_IOCFG1); */
/* 	uint16_t hssd_src = FV(reg, HSSD_SRC_MSB, HSSD_SRC_LSB); */
/* 	hssd_src = (hssd_src + 1) % 8; */
/* 	if(hssd_src == 0) CC2420_ENABLE_FIFOP_INT(); */
/* 	else	CC2420_DISABLE_FIFOP_INT(); */

/* 	CC2420_FIFO_PORT(DIR) &= ~BV(CC2420_FIFO_PIN); */
/* 	reg = SETFV(reg, hssd_src, HSSD_SRC_MSB, HSSD_SRC_LSB); */
/* 	setreg(CC2420_IOCFG1, reg); */
/* 	reg = getreg(CC2420_IOCFG1); */
/* 	iprintf("HSSD_SRC: %u \n", FV(reg, HSSD_SRC_MSB, HSSD_SRC_LSB)); */
/* } */

/*---------------------------------------------------------------------------*/
/* static void */
/* debug_analog(void) */
/* { */
/* #define ATESTMOD_MODE_MSB 3 */
/* #define ATESTMOD_MODE_LSB 0 */
/* #define ATESTMOD_PD_MSB 4 */
/* #define ATESTMOD_PD_LSB 4 */
/* #define ATESTMOD_START 4 // first invocation jumpstarts to mode [0,8] */

/* 	uint16_t reg = getreg(CC2420_TOPTST); */
/* 	uint16_t atestmod_pd = FV(reg, ATESTMOD_PD_MSB, ATESTMOD_PD_LSB); */
/* 	uint16_t atestmod_mode = FV(reg, ATESTMOD_MODE_MSB, ATESTMOD_MODE_LSB); */

/* 	if (atestmod_pd) { */
/* 		// Power is down: start Atest and start from mode 0 */
/* 		atestmod_pd = 0; */
/* 		atestmod_mode = ATESTMOD_START; // start from mode 4 */
/* 	} else { */
/* 		// next mode */
/* 		atestmod_mode = (atestmod_mode+1) % 9; */
/* 		// Last mode: stop Atest */
/* 		if (atestmod_mode == ATESTMOD_START) */
/* 			atestmod_pd++; */
/* 	} */

/* 	reg = SETFV(reg, atestmod_pd, ATESTMOD_PD_MSB, ATESTMOD_PD_LSB); */
/* 	reg = SETFV(reg, atestmod_mode, ATESTMOD_MODE_MSB, ATESTMOD_MODE_LSB); */
/* 	setreg(CC2420_TOPTST, reg); */
/* 	reg = getreg(CC2420_TOPTST); */
/* 	iprintf("ATESTMOD_PD: %u ATESTMOD_MODE: %u\n", */
/* 				 FV(reg, ATESTMOD_PD_MSB, ATESTMOD_PD_LSB), */
/* 				 FV(reg, ATESTMOD_MODE_MSB, ATESTMOD_MODE_LSB)); */
/* } */
/*---------------------------------------------------------------------------*/
/* static void */
/* alter_syncword(void) */
/* { */
/* 	#define SW 4 */
/* 	const static uint16_t syncwords[SW] = {0xA70F, 0xA7FF, 0xA700, 0xCD0F}; */
/* 	int i; */
/* 	uint16_t reg; */

/* 	reg = getreg(CC2420_SYNCWORD); */
/* 	for(i = 0;i < SW;i++) { */
/* 		if(syncwords[i] == reg) { */
/* 			reg = syncwords[(i+1) % SW]; */
/* 			break; */
/* 		} */
/* 	} */
/* 	if (i==SW) reg = syncwords[0]; */

/* 	setreg(CC2420_SYNCWORD, reg); */
/* 	reg = getreg(CC2420_SYNCWORD); */
/* 	iprintf("Syncword: 0x%X\n", reg); */
/* } */
/*---------------------------------------------------------------------------*/
/* Change my MAC address, can be used to alter ACK behaviour */
/* static void */
/* mac_update(void) */
/* { */
/* 	unsigned shortaddr; */
/* 	CC2420_READ_RAM(&shortaddr,CC2420RAM_SHORTADDR, 2); */
/* 	// increment higher byte and clear lower byte, must correspond to linkaddr_node_addr.u8[0] if Rime stack used */
/* 	shortaddr = (shortaddr+(1<<8))%(8<<8) & 0xFF00; */
/* 	CC2420_WRITE_RAM(&shortaddr,CC2420RAM_SHORTADDR, 2); */
/* 	iprintf("16-bit MAC address: 0x%04X\n", shortaddr); */
/* } */

/*---------------------------------------------------------------------------*/
/* Store 16 MAC address as permanent node ID in XMEM */
/* static void */
/* burn_node_id(void) */
/* { */
/* 	unsigned shortaddr; */
/* 	unsigned id; */
/* 	CC2420_READ_RAM(&shortaddr,CC2420RAM_SHORTADDR, 2); */
/* 	id = (shortaddr<<8)|(shortaddr>>8); */
/* 	node_id_burn(id); */
/* 	node_id_restore(); */
/* 	iprintf("node ID now set to %u\n", node_id); */
/* } */
//
/* static void */
/* read_tx_fifo(void) */
/* { */
/* 	uint8_t byte[128]; */
/* 	CC2420_READ_RAM(&byte,CC2420RAM_TXFIFO, 128); */
/* 	iprintf("TXFIFO content:\n"); */
/* 	int i; */
/* 	for (i = 0;i < 128;i++) { */
/* 		iprintf("%02x", byte[i]); */
/* 	} */
/* 	iprintf("\n"); */
/* } */

/* static void */
/* write_tx_fifo(void) */
/* { */
/* 	/\* uint8_t byte = 3; *\/ */
/* 	/\* CC2420_WRITE_RAM(&byte,CC2420RAM_TXFIFO+10, 1); *\/ */
/* 	/\* iprintf("TXFIFO byte one %u\n", byte); *\/ */
/* 	uint8_t snippet[6] = {0x00, 0x00, 0xA7, 4, 0x30, 0x31}; */
/* 	unsigned short checksum; */
/* 	checksum = crc16_data(&snippet[4], sizeof(snippet)-4, 0); */
/* 	/\* snippet[sizeof(snippet) - 2] = crc16>>8; *\/ */
/* 	/\* snippet[sizeof(snippet) - 1] = crc16&0xFF; *\/ */
/* 	CC2420_WRITE_RAM(&snippet, CC2420RAM_TXFIFO, sizeof(snippet)); */
/* 	CC2420_WRITE_RAM(&checksum, CC2420RAM_TXFIFO+sizeof(snippet), 2); */
/* } */

/* static void */
/* read_rx_fifo(void) */
/* { */
/* 	uint8_t byte[128]; */
/* 	CC2420_READ_RAM(&byte,CC2420RAM_RXFIFO, 128); */
/* 	iprintf("RXFIFO content:\n"); */
/* 	int i; */
/* 	for (i = 0;i < 128;i++) { */
/* 		iprintf("%02x", byte[i]); */
/* 	} */
/* 	iprintf("\n"); */
/* } */

/*---------------------------------------------------------------------------*/
/* Convert a hex number into 16 bit characters, MSB first */
/* static char* */
/* u16_to_bits(uint16_t n, char bits[]) */
/* { */
/* 	int i; */

/* 	for(i=0;i<16;i++) { */
/* 		bits[i] = (char)((n>>(15-i) & 1) + '0'); */
/* 	} */

/* 	bits[i] = '\0'; */

/* 	return bits; */
/* } */

/* static void */
/* show_all_registers(void) */
/* { */
/* 	uint16_t reg; */
/* 	enum cc2420_register addr; */
/* 	char bits[17]; //16 bits + '\0' */

/* 	for (addr = CC2420_MAIN;addr <= CC2420_RESERVED;addr++) { */
/* 		reg = getreg(addr); */
/* 		iprintf("0x%02X: 0x%04X %s\n", addr, reg,	u16_to_bits(reg, bits)); */
/* 	} */
/* } */

/* static void */
/* status(void) */
/* { */
/*   uint8_t status; */
/*   CC2420_GET_STATUS(status); */
/* 	iprintf("Status 0x%01X\n", status); */
/* } */

/*-----------------------------------------------------------------------------*/
static void
exec_command(char c)
{
	if(command_table_base) {
		const struct command *cmd_ptr = command_table_base;
		while(cmd_ptr->f) {
			if(c == cmd_ptr->ch) {
				cmd_ptr->f();
				return;
			}
			cmd_ptr++;
		}
		// error: no such command
	}
	// error: command table not set
}

/* static unsigned */
/* hexstr_to_unsigned(const char *s) */
/* { */
/* 	unsigned n,m; */

/* 	for(n=0;(*s>='0' && *s<='9')||(*s>='a' && *s<='f')||(*s>='A' && *s<='F');s++) { */
/* 		if(*s>='0' && *s<='9') {m=*s-'0';} */
/* 		else if(*s>='a' && *s<='f') {m=*s-'a'+10;} */
/* 		else {m=*s-'A'+10;} */
/* 		n=n*16+m; */
/* 	} */

/* 	return n; */
/* } */

/* static void */
/* print_reg(const char* hex_str) */
/* { */
/* 	uint16_t reg; */
/* 	enum cc2420_register addr; */
/* 	char bits[17]; //16 bits + '\0' */
	
/* 	addr = hexstr_to_unsigned(hex_str); */
/*   if(addr >= CC2420_MAIN && addr <= CC2420_RESERVED) { */
/* 		reg = getreg(addr); */
/* 		iprintf("0x%02X: 0x%04X %s\n", addr, reg,	u16_to_bits(reg, bits)); */
/* 	} else if(addr < CC2420_foo) { */
/* 		iprintf("Strobe 0x%02X\n", addr); */
/* 		CC2420_STROBE(addr); */
/* 	} else { */
/* 		iprintf("Unknown register: 0x%s\n", hex_str); */
/* 	} */
/* } */

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

	/* 0-9: dialpad callback */
	if(s[0] >= '0' && s[0] <= '9' && callback != NULL)	{callback(s[0] - '0');}
	/* twin chars: special command */
	else if (s[0]==s[1]) {exec_command(s[0]);}
	/* arithmetic operator plus char: update variable */
	else if((s[0]=='+' || s[0]=='-' || s[0]=='*' || s[0]=='/' || s[0]=='<' || s[0]=='>' || s[0]=='^' || s[0]=='\'')) {var_update(s[0], s[1]);}
	/* else if(s[1]=='+' || s[1]=='-' || s[1]=='<' || s[1]=='>' || s[1]=='\0') {field_update(s[0], s[1]);} */
	/* else if(s[0]=='x') {print_reg(&s[1]);} */
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
				iprintf("%s=%u\n", vp->long_name, *u8);
				break;
			case 2:
				u16 = &(vp->n->u16);
				OP(*u16, op);
				*u16 = (*u16) % (vp->ceiling - vp->floor + 1);
				iprintf("%s=%u\n", vp->long_name, *u16);
				break;
			case 4:
				u32 = &(vp->n->u32);
				OP(*u32, op);
				*u32 = (*u32) % (vp->ceiling - vp->floor + 1);
				iprintf("%s=%lu\n", vp->long_name, *u32);
				break;
			default:;
			}
		}
		vp++;
	}
}

/*-----------------------------------------------------------------------------*/ 
// Generic CC2420 field update operations
/* void */
/* field_update(char c, char op) */
/* { */
/* 	const struct field *fp; */
/* 	unsigned reg, fv; */
	
/* 	for(fp=&field_list[0]; fp < &field_list[0] + sizeof(field_list)/sizeof(struct field); fp++) { */
/* 		if(fp->ch == c) { */
/* 			reg = getreg(fp->addr); */
/* 			fv = FV(reg, fp->msb, fp->lsb); */
/* 			/\* skip update when no op *\/ */
/* 			if(op != '\0') { */
/* 				OP(fv, op); */
/* 				fv = fv % (0x1<<(fp->msb - fp->lsb +1)); */
/* 				reg = SETFV(reg, fv, fp->msb, fp->lsb); */
/* 				setreg(fp->addr, reg); */
/* 				fv = FV(getreg(fp->addr), fp->msb, fp->lsb); */
/* 			} */
/* 			iprintf("%s=%u\n", fp->name, fv); */
/* 			return; */
/* 		} */
/* 	} */
/* } */

/* Initialize pointer to user variables */
void
commands_set_user_vars(const struct variable *vars)
{
	if(!user_vars) iprintf("User variables initialized\n");
	else iprintf("User variables updated\n");
	user_vars = vars;

	// debug info: variables command, name and width
	const struct variable *vp;
	iprintf("Cmd Variable      Width\n");
	for(vp=user_vars;vp->ch != '\0';vp++) {
		iprintf("%c   %s      %d\n", vp->ch, vp->long_name, vp->width);
	}
}
