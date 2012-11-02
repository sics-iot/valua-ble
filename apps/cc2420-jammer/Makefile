CONTIKI = ../..
ifndef TARGET
TARGET=z1
endif

# save intermediate assembled files in the same directory of the ojbect files
# e.g. "make TARGET=z1" puts cc2420.s and cc2420.i along with cc2420.o in dir "./obj_z1"
SAVE_TEMPS= -save-temps=obj

CFLAGS = -DENABLE_CCA_INTERRUPT=1 -DENABLE_UNBUFFERED_MODE=1 -DENABLE_FAST_TX=0 -DCC2420_CONF_AUTOACK=0 -DCCA_TEST=0 $(SAVE_TEMPS)

all: cc2420-jammer

# shows compiled assembly code alongside source code
%.lst: %.c
	$(CC) -c -g -Wa,-a,-ad $(CFLAGS) $< > $@

include $(CONTIKI)/Makefile.include
