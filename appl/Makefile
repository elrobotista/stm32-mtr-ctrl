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

debug: $(PROJECT).elf
	pkill openocd || true
	$(HOME)/scripts/launch_openocd.sh
	arm-none-eabi-gdb-py3 --symbols=$(PROJECT).elf \
		--ex="target extended-remote :3333" \
		--ex="b main" \
		--ex="mon reset" \
		--ex="source ../../svd-tools/gdb-svd.py" \
		--ex="svd ../../cmsis-svd/data/STMicro/STM32F303.svd"
	pkill openocd || true

flash: $(PROJECT).bin
	killall openocd || true
	st-flash --reset write $(PROJECT).bin 0x8000000

erase:
	st-flash erase
