PROJECT = mtr-ctrl
BUILD_DIR = bin

SHARED_DIR =
CFILES = main.c

DEVICE=stm32f303ret6
OOCD_FILE = board/st_nucleo_f3.cfg

VPATH += $(SHARED_DIR)
INCLUDES += $(patsubst %,-I%, . $(SHARED_DIR))
OPENCM3_DIR=../libopencm3

include $(OPENCM3_DIR)/mk/genlink-config.mk
include ../rules.mk
include $(OPENCM3_DIR)/mk/genlink-rules.mk

flash:
	st-flash --reset write $(PROJECT).bin 0x8000000
