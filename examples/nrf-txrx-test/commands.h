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
 * \defgroup serial-debug Serial debug commands
 * @{
 */

/**
 * \file
 *         Debugging commands using the serial-line facility
 * \author
 *         Zhitao He <zhitao@sics.se>
 */

#ifndef __COMMANDS__
#define __COMMANDS__

// Field mask, generated from MSB and LSB. E.g. FM(4,0)=0x000F, FM(5,1)=0x001E
#define FM(MSB, LSB) \
	(((0x0001<<(MSB - LSB +1)) - 1) << LSB) // 2 ^ nbits - 1, then left shift

// Field value, extracted from register value, MSB and LSB
#define FV(REGVAL, MSB, LSB) \
	((REGVAL & FM(MSB, LSB)) >> LSB)

// Register value, with updated field
#define SETFV(REGVAL, FV, MSB, LSB) \
	((REGVAL & ~FM(MSB, LSB)) | FV << LSB)

/** \struct command
 * \brief A special command is denoted and executed by a single character
 *
 * if ch is 'h': display of all the commands, fields and registers.
 * If ch is '0'~'9': the dialpad callback is run with the corresponding number as parameter.
 * if ch is another char: a matching callback function f is called.
 */
struct command
{
	const char ch;
	const char *name;
	void (*f)(void);
};

/** \struct field
 * \brief A register field command is denoted by a character
 * and executed by a two-character string
 *
 * For example, a user can define a 'w' field command to access the two-bit
 * "RF_PWR" field of the RF_SETUP register in the radio by specifying the register address
 * and the MSB and LSB of the field. 
 * 
 * A set of sub-commands are then executable by thesetwo-charactor strings:
 * "ww": displays the current value of RF_PWR
 * "w+": increments RF_PWR by one
 * "w-": decrements RF_PWR by one
 * "w>": doubles RF_PWR
 * "w<": halfs RF_PWR
 */
struct field
{
	char ch;
	const char *name;
	uint8_t addr;
	unsigned msb;
	unsigned lsb;
};

/** \struct variable
 * \brief A variable update command is denoted by a character
 * and executed by a two-character string
 *
 * For example, a user can define a 'y' variable command to update a user variable
 * "payload_len", an unsigned integer, by an arithmetic operation. 
 * Specifying the variable's size in bytes as well as its floor and ceiling
 * allows wrap-around to be performed correctly .

 * "+y": payload_len + 1
 * "-y": payload_len - 1
 * "^y": payload_len + 10
 * "\y": payload_len - 10
 * ">y": payload_len * 2
 * "<y": payload_len / 2
 * "*y": payload_len * 10
 * "/y": payload_len / 10
 */
/* An unsigned integer */
union number {
	uint8_t u8;
	uint16_t u16;
	uint32_t u32;
};

struct variable
{
	const char ch; // single char denoting variable
	union number *n;
	int width;
	const char *long_name; // long descriptive name
	unsigned floor; // positive (floor value) or negative (no floor)
	unsigned ceiling;
};


/**
 * \brief      Run a serial debug command.
 * \param cmd A string
 *
 * This function runs a pre-defined user command
 * after receiving a single-character or two-character string from the serial port
 */
void do_command(char *cmd);

/**
 * \brief      Initialize the serial debug commands.
 * \param dialpad A user function taking a single digit number (0~9) as its parameter.
 * \param getreg A user function that returns a register's value, given its address.
 * \param setreg A user function that sets a register's value, given its address.
 * \param cmd_list A pointer to an array of command structures
 * \param cmd_list_len Size of the array of command structures
 * \param fd_list A pointer to an array of register fields
 * \param fd_list_len Size of the array of register fields
 * \param var_list A pointer to an array of user variables
 * \param var_list_len Size of the array of user variables
 *
 * This function initializes the serial debug commands
 * with a set of user defined callback functions and data structures
 */
void
commands_init(void (*dialpad)(int),
              unsigned (*getreg)(unsigned addr),
              void (*setreg)(unsigned addr, unsigned val),
              const struct command *cmd_list, int cmd_list_len,
              const struct field *fd_list, int fd_list_len,
              const struct variable *var_list, int var_list_len);

#endif /*__COMMANDS__*/
/*-----------------------------------------------------------------------------*/
/** @} */