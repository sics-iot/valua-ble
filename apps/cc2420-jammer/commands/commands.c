/*
 * Copyright (c) 2015, SICS Swedish ICT.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 */

/**
 * \addtogroup serial-debug
 * @{
 */

/**
 * \file
 *         Debugging commands using the serial-line facility
 * \author
 *         Zhitao He <zhitao@sics.se>
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "contiki.h"
#include "commands.h"

#if !CONTIKI_TARGET_NRF_BEACON
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...) iprintf(__VA_ARGS__)
#endif

static const struct variable *_var_list;
static const struct field *_fd_list;
static const struct command *_cmd_list;
static int _cmd_list_len, _fd_list_len, _var_list_len;
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
void
commands_init(void (*dialpad)(int), 
              unsigned (*getreg)(unsigned addr),
              void (*setreg)(unsigned addr, unsigned val), 
              const struct command *cmd_list, int cmd_list_len,
              const struct field *fd_list, int fd_list_len, 
              const struct variable *var_list, int var_list_len)
{
	_dialpad = dialpad;
	_getreg = getreg;
	_setreg = setreg;
	_cmd_list = cmd_list;
	_cmd_list_len = cmd_list_len;
	_fd_list = fd_list;
	_fd_list_len = fd_list_len;
	_var_list = var_list;
	_var_list_len = var_list_len;

	PRINTF("press h for command list\n");
}
/*-----------------------------------------------------------------------------*/ 
/* Generic 8-bit register field update operations */
static void
field_update(char c, char op)
{
	unsigned reg, fv;
	int i;

	if(_fd_list && _fd_list_len) {
		for(i = 0; i < _fd_list_len; i++) {
			if(_fd_list[i].ch == c) {
				reg = _getreg(_fd_list[i].addr);
				fv = FV(reg, _fd_list[i].msb, _fd_list[i].lsb);
				/* skip update when no op */
				if(op != c) {
					OP(fv, op);
					fv = fv % (0x1<<(_fd_list[i].msb - _fd_list[i].lsb +1));
					reg = SETFV(reg, fv, _fd_list[i].msb, _fd_list[i].lsb);
					_setreg(_fd_list[i].addr, reg);
					fv = FV(_getreg(_fd_list[i].addr), _fd_list[i].msb, _fd_list[i].lsb);
				}
				PRINTF("%s=%u\n", _fd_list[i].name, fv);
				return;
			}
		}
	}
}
/*-----------------------------------------------------------------------------*/
static void
var_update(char op, char var)
{
	uint8_t *u8;
	uint16_t *u16;
	uint32_t *u32;

	if(_var_list && _var_list_len) {
		for(int i = 0; i < _var_list_len; i++) {
			if(_var_list[i].ch == var) {
				switch(_var_list[i].width) {
				case 1:
					u8 = &(_var_list[i].n->u8);
					OP(*u8, op);
					*u8 = (*u8) % (_var_list[i].ceiling - _var_list[i].floor + 1);
					PRINTF("%s=%u\n", _var_list[i].long_name, *u8);
					return;
				case 2:
					u16 = &(_var_list[i].n->u16);
					OP(*u16, op);
					*u16 = (*u16) % (_var_list[i].ceiling - _var_list[i].floor + 1);
					PRINTF("%s=%u\n", _var_list[i].long_name, *u16);
					return;
				case 4:
					u32 = &(_var_list[i].n->u32);
					OP(*u32, op);
					*u32 = (*u32) % (_var_list[i].ceiling - _var_list[i].floor + 1);
					PRINTF("%s=%lu\n", _var_list[i].long_name, *u32);
					return;
				default:;
				}
			}
		}
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
	if(_cmd_list && _cmd_list_len) {
		for(int i = 0; i < _cmd_list_len; i++) {
			if(c == _cmd_list[i].ch) {
				_cmd_list[i].f();
				return;
			}
		}
	}
}
/*-----------------------------------------------------------------------------*/
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
/*-----------------------------------------------------------------------------*/
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
/*-----------------------------------------------------------------------------*/
static void
help(void *foo)
{
	static struct ctimer ct;
	static int state = 0;
	int i;

	switch(state) {
	case 0:
		if(_cmd_list && _cmd_list_len) {
			PRINTF("Special cmds <cmd name>\n");
			for(i = 0; i < _cmd_list_len; i++) {
				PRINTF("%c\t%s\n", _cmd_list[i].ch, _cmd_list[i].name);
			}
		}
		/* Ugly: pause print for a moment to avoid UART buffer overflow */
		state++;
		ctimer_set(&ct, CLOCK_SECOND/100, help, NULL);
	break;

	case 1:
		if(_fd_list && _fd_list_len) {
		PRINTF("---------\n");
		PRINTF("Register field cmds <cmd name bits>:\n");
			for(i = 0; i < _fd_list_len; i++) {
			PRINTF("%c\t%s %u\n", _fd_list[i].ch, _fd_list[i].name, _fd_list[i].msb - _fd_list[i].lsb +1);
			}
		}

		/* Ugly pause */
		state++;
		ctimer_set(&ct, CLOCK_SECOND/100, help, NULL);
		break;

	case 2:
		if(_var_list && _var_list_len) {
			PRINTF("---------\n");
			PRINTF("User variables <cmd variable bytes>:\n");
			for(i = 0; i < _var_list_len; i++) {
				PRINTF("%c\t%s %d\n", _var_list[i].ch, _var_list[i].long_name, _var_list[i].width);
			}
		}

		state = 0;
	default:
		;
	}
}
/*-----------------------------------------------------------------------------*/
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
/*-----------------------------------------------------------------------------*/
/** @} */
