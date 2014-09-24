#ifndef COMMANDS
#define COMMANDS

// Field mask, generated from MSB and LSB. E.g. FM(4,0)=0x000F, FM(5,1)=0x001E
#define FM(MSB, LSB) \
	(((0x0001<<(MSB - LSB +1)) - 1) << LSB) // 2 ^ nbits - 1, then left shift

// Field value, extracted from register value, MSB and LSB
#define FV(REGVAL, MSB, LSB) \
	((REGVAL & FM(MSB, LSB)) >> LSB)

// Register value, with updated field
#define SETFV(REGVAL, FV, MSB, LSB) \
	((REGVAL & ~FM(MSB, LSB)) | FV << LSB)

extern long int sum_rssi;
extern long unsigned sum_lqi;
/* extern uint8_t hex_seq[]; */

unsigned getreg(enum cc2420_register regname);
void setreg(enum cc2420_register regname, unsigned value);
uint16_t cc2420_get_frequency(void);
int cc2420_set_frequency(uint16_t f);

void do_command(char *cmd);
void commands_set_callback(void (*f)(int));
void var_update(char op, char var);
void field_update(char c, char op);

struct command
{
	char ch;
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
#endif

void commands_set_user_vars(const struct variable *vars);
