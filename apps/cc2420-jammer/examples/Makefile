CONTIKI = ../../../../contiki
ifndef TARGET
TARGET=z1
endif

PROJECT_SOURCEFILES += commands.c
PROJECTDIRS += ../commands

CFLAGS = -DWITH_PHASE_OPTIMIZATION=0 -DRIMESTATS_CONF_ENABLED=1
all: example-unicast hello-world

include $(CONTIKI)/Makefile.include
