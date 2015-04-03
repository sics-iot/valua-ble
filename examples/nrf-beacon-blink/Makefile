CONTIKI = ../..
CONTIKI_PROJECT = blink

CFLAGS = -DCONTIKI_WITH_RIME=1
CFLAGS += -DENERGEST_CONF_ON=0

# size optimizations suggested on Contiki wiki
CFLAGS += -ffunction-sections
LDFLAGS += -Wl,--gc-sections,--undefined=_reset_vector__,--undefined=InterruptVectors,--undefined=_copy_data_init__,--undefined=_clear_bss_init__,--undefined=_end_of_init__


all: $(CONTIKI_PROJECT)

TARGET=nrf-beacon


include $(CONTIKI)/Makefile.include
