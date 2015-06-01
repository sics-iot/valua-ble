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

struct command
{
	const char ch;
	const char *name;
	void (*f)(void);
};

struct field
{
	char ch;
	const char *name;
	uint8_t addr;
	unsigned msb;
	unsigned lsb;
};

// User-adjustable variable (non-negative integer type)
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

void do_command(char *cmd);
void commands_init(void (*dialpad)(int),
                   unsigned (*getreg)(unsigned addr),
                   void (*setreg)(unsigned addr, unsigned val),
                   const struct command *cmd_list,
                   const struct field *flist,
                   const struct variable *vars);

#endif /*__COMMANDS__*/
