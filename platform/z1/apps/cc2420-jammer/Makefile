CONTIKI = ../../../..
ifndef TARGET
TARGET=z1
endif

CFLAGS = -DENABLE_CCA_INTERRUPT=1 -DENABLE_FIFO_INTERRUPT=0 -DENABLE_UNBUFFERED_MODE=0 -DENABLE_FAST_TX=0 -DCC2420_CONF_AUTOACK=0 -DCCA_TEST=1 -save-temps

all: cc2420-jammer

%.lst: %.c
	$(CC) -c -g -Wa,-a,-ad $(CFLAGS) $< > $@

include $(CONTIKI)/Makefile.include
