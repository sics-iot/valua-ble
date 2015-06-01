#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "contiki.h"

#include "commands.h"

#if TARGET!=nrf-beacon
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...) iprintf(__VA_ARGS__)
#endif

static const struct variable *user_vars;
static const struct field *field_list;
static const struct command *command_table_base;
static void (*_dialpad)(int v);
static unsigned (*_getreg)(unsigned addr);
static void (*_setreg)(unsigned addr, unsigned val);

#define OP(n, op)\
				switch(op) {\
 				case '+':	n += 1;	break;\
				case '-': n -= 1;	break;\
 				case '*': n *= 10;	break;\
 				case '/': n /= 10;	break;\
 				case '>': n <<= 1;	break;\
 				case '<': n >>= 1;	break;\
 				case '^': n += 10;	break;\
 				case '\'': n -= 10;	break;\
				default:;\
				}
/*---------------------------------------------------------------------------*/
void commands_init(void (*dialpad)(int),
                   unsigned (*getreg)(unsigned addr),
                   void (*setreg)(unsigned addr, unsigned val),
                   const struct command *cmd_list,
                   const struct field *flist,
                   const struct variable *vars)
{
	_dialpad = dialpad;
	_getreg = getreg;
	_setreg = setreg;
	command_table_base = cmd_list;
	field_list = flist;
	user_vars = vars;

	PRINTF("press h for command list\n");
}

/*-----------------------------------------------------------------------------*/ 
/* Generic 8-bit register field update operations */
static void
field_update(char c, char op)
{
	const struct field *fp;
	unsigned reg, fv;

	for(fp=field_list; fp->ch; fp++) {
		if(fp->ch == c) {
			reg = _getreg(fp->addr);
			fv = FV(reg, fp->msb, fp->lsb);
			/* skip update when no op */
			if(op != c) {
				OP(fv, op);
				fv = fv % (0x1<<(fp->msb - fp->lsb +1));
				reg = SETFV(reg, fv, fp->msb, fp->lsb);
				_setreg(fp->addr, reg);
				fv = FV(_getreg(fp->addr), fp->msb, fp->lsb);
			}
			PRINTF("%s=%u\n", fp->name, fv);
			return;
		}
	}
}

static void
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
				PRINTF("%s=%u\n", vp->long_name, *u8);
				break;
			case 2:
				u16 = &(vp->n->u16);
				OP(*u16, op);
				*u16 = (*u16) % (vp->ceiling - vp->floor + 1);
				PRINTF("%s=%u\n", vp->long_name, *u16);
				break;
			case 4:
				u32 = &(vp->n->u32);
				OP(*u32, op);
				*u32 = (*u32) % (vp->ceiling - vp->floor + 1);
				PRINTF("%s=%lu\n", vp->long_name, *u32);
				break;
			default:;
			}
		}
		vp++;
	}
}
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
/* 		PRINTF("0x%02X: 0x%04X %s\n", addr, reg,	u16_to_bits(reg, bits)); */
/* 	} */
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
/* 		PRINTF("0x%02X: 0x%04X %s\n", addr, reg,	u16_to_bits(reg, bits)); */
/* 	} else if(addr < CC2420_foo) { */
/* 		PRINTF("Strobe 0x%02X\n", addr); */
/* 		CC2420_STROBE(addr); */
/* 	} else { */
/* 		PRINTF("Unknown register: 0x%s\n", hex_str); */
/* 	} */
/* } */
static void
help(void *foo)
{
	static struct ctimer ct;
	static int state = 0;
	const struct command *cmd_ptr;
	const struct field *fp;
	const struct variable *vp;

	switch(state) {
	case 0:
		PRINTF("Special cmds:\n");
		cmd_ptr = command_table_base;
		while(cmd_ptr->f) {
			PRINTF("%c\t%s\n", cmd_ptr->ch, cmd_ptr->name);
			cmd_ptr++;
		}
		/* Ugly: pause print for a moment to avoid UART buffer overflow */
		state++;
		ctimer_set(&ct, CLOCK_SECOND/100, help, NULL);
	break;

	case 1:
		PRINTF("---------\n");
		PRINTF("register field cmds <cmd name width(bits)>:\n");
		for(fp=field_list; fp->name; fp++)
			PRINTF("%c\t%s %u\n", fp->ch, fp->name, fp->msb - fp->lsb +1);

		/* Ugly pause */
		state++;
		ctimer_set(&ct, CLOCK_SECOND/100, help, NULL);
		break;

	case 2:
		PRINTF("---------\n");
		PRINTF("User variables <Cmd\tVariable Width>:\n");
		for(vp=user_vars;vp->ch != '\0';vp++) {
			PRINTF("%c\t%s %d\n", vp->ch, vp->long_name, vp->width);
		}

		state = 0;
	default:
		;
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

	/* single char command */
	if(s[1] == '\0') {
		/* h: help */
		if(s[0] == 'h') {help(NULL);}
		/* 0-9: dialpad callback */
		if(s[0] >= '0' && s[0] <= '9' && _dialpad != NULL) {_dialpad(s[0] - '0');}
		/* others: special command */
		else {exec_command(s[0]);}
	} else {
		/* multi char command */
		/* arithmetic operator plus char: update variable */
		if((s[0]=='+' || s[0]=='-' || s[0]=='*' || s[0]=='/' || s[0]=='<' || s[0]=='>' || s[0]=='^' || s[0]=='\'')) {var_update(s[0], s[1]);}
		/* char plus operator or twin chars: update field/print field */
		else if(s[1]=='+' || s[1]=='-' || s[1]=='<' || s[1]=='>' || s[1]==s[0]) {field_update(s[0], s[1]);}
	/* else if(s[0]=='x') {print_reg(&s[1]);} */
	}
}
