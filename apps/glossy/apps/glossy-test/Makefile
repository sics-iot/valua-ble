CONTIKI_PROJECT = glossy-test
all: $(CONTIKI_PROJECT)

ifndef TARGET
TARGET=sky
endif

CONTIKI = ../../../..

PROJECT_SOURCEFILES += glossy.c

# add paths of overloading Contiki source files, which should take precedence over standard Contiki paths
CONTIKI_OVERLOAD = ../..
# PROJECTDIRS += ${addprefix $(CONTIKI_OVERLOAD)/, core core/dev core/lib core/loader core/sys cpu/msp430}
PROJECTDIRS += ${addprefix $(CONTIKI_OVERLOAD)/, core/dev}
ifeq ($(TARGET), z1)
# PROJECTDIRS += ${addprefix $(CONTIKI_OVERLOAD)/, platform/z1 cpu/msp430/f2xxx cpu/msp430/f1xxx}
	PROJECTDIRS += ${addprefix $(CONTIKI_OVERLOAD)/, platform/z1}
else
	ifeq ($(TARGET), sky)
	PROJECTDIRS += ${addprefix $(CONTIKI_OVERLOAD)/, platform/sky}
	endif
endif


include $(CONTIKI)/Makefile.include
