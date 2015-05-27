#ifndef __COMMANDS__
#define __COMMANDS__

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
                   uint8_t (*getreg)(uint8_t addr),
                   void (*setreg)(uint8_t addr, uint8_t val),
                   const struct command *cmd_list,
                   const struct field *flist,
                   const struct variable *vars);

#endif /*__COMMANDS__*/
