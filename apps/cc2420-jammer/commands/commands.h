#ifndef COMMANDS
#define COMMANDS

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

extern long int sum_rssi;
extern long unsigned sum_lqi;
extern uint8_t len_hdr;
/* extern uint8_t hex_seq[]; */

unsigned getreg(enum cc2420_register regname);
void setreg(enum cc2420_register regname, unsigned value);
uint16_t cc2420_get_frequency(void);
int cc2420_set_frequency(uint16_t f);

void do_command(char *cmd);
void commands_set_callback(void (*f)(int));
void var_update(char op, char var);

struct command
{
	char ch1;
	char ch2;
	void (*f)(void);
};

// User-adjustable variable (non-negative integer type)
union number {
		uint8_t u8;
		uint16_t u16;
		uint32_t u32;
};

struct variable
{
	char ch; // single char denoting variable
	union number *n;
	int width;
	const char *long_name; // long descriptive name
	unsigned floor; // positive (floor value) or negative (no floor)
	unsigned ceiling;
};

extern const struct variable user_variable_list[];

#define OP(n, op)\
				switch(op) {\
 				case '+':	n += 1;	break;\
				case '-': n -= 1;	break;\
 				case '*': n *= 10;	break;\
 				case '/': n /= 10;	break;\
 				case '<': n <<= 1;	break;\
 				case '>': n >>= 1;	break;\
 				case '^': n += 10;	break;\
 				case '\'': n -= 10;	break;\
				default:;\
				}

#endif
