CONTIKI = ../../
ifndef TARGET
TARGET=z1
endif

PROJECT_SOURCEFILES += commands.c

# save intermediate output files, i.e. preprocessor .i outputs and assembler .S outputs, in the same dir as the .o ojbect files
# e.g. "make TARGET=z1" puts cc2420.s and cc2420.i along with cc2420.o in dir "./obj_z1"
SAVE_TEMPS= -save-temps=obj

CFLAGS = -DENABLE_CCA_INTERRUPT=1 -DENABLE_UNBUFFERED_MODE=0 -DENABLE_FAST_TX=0 -DCC2420_CONF_AUTOACK=0 -DCCA_TEST=0 -DRIMESTATS_CONF_ENABLED=1 $(SAVE_TEMPS)

# CFLAGS += -DRTIMER_CONF_SECOND=\(8192\)

all: cc2420-jammer

# shows compiled assembly code alongside source code
%.lst: %.c
	$(CC) -c -g -Wa,-a,-ad $(CFLAGS) $< > $@

# add paths of overloading Contiki source files, which should take precedence over standard Contiki paths
CONTIKI_OVERLOAD = .
PROJECTDIRS += ${addprefix $(CONTIKI_OVERLOAD)/, core/net/mac core/dev}
ifeq ($(TARGET), z1)
PROJECTDIRS += ${addprefix $(CONTIKI_OVERLOAD)/, platform/z1 cpu/msp430/f2xxx cpu/msp430/f1xxx}
else
	ifeq ($(TARGET), sky)
	PROJECTDIRS += ${addprefix $(CONTIKI_OVERLOAD)/, platform/sky cpu/msp430/f1xxx}
	endif
endif
PROJECTDIRS += ./commands

include $(CONTIKI)/Makefile.include
