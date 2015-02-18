CONTIKI = ../../
ifndef TARGET
TARGET=sky
endif

CFLAGS = -DCONTIKI_WITH_RIME=0
CFLAGS += -DENERGEST_CONF_ON=0

# size optimizations suggested on Contiki wiki
CFLAGS += -ffunction-sections
LDFLAGS += -Wl,--gc-sections,--undefined=_reset_vector__,--undefined=InterruptVectors,--undefined=_copy_data_init__,--undefined=_clear_bss_init__,--undefined=_end_of_init__

all: good-morning week pushy

# shows compiled assembly code alongside source code
%.lst: %.c
	$(CC) -c -g -Wa,-a,-ad $(CFLAGS) $< > $@

include $(CONTIKI)/Makefile.include
