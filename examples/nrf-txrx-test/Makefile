CONTIKI = ../..
CONTIKI_PROJECT = txrx-test
TARGET=nrf-beacon

CONTIKI_SOURCEFILES += commands.c nrf24l01p.c

CFLAGS = -DCONTIKI_WITH_RIME=1
CFLAGS += -DENERGEST_CONF_ON=0

# size optimizations suggested on Contiki wiki
CFLAGS += -ffunction-sections -std=gnu99 #-save-temps=obj

LDFLAGS += -Wl,--gc-sections,--undefined=_reset_vector__,--undefined=InterruptVectors,--undefined=_copy_data_init__,--undefined=_clear_bss_init__,--undefined=_end_of_init__

# shows compiled assembly code alongside source code
%.lst: %.c
	$(CC) -c -g -Wa,-a,-ad $(CFLAGS) $< > $@

%.p: %.c
	$(CC) -E -dD $(CFLAGS) $< > $@

all: $(CONTIKI_PROJECT)

srec: $(CONTIKI_PROJECT).$(TARGET).srec

include $(CONTIKI)/Makefile.include
