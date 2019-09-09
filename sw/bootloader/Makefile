include ../../config_default.mk

PROJ=bootloader

SRC_FILES = crt.S bootloader.c

all: bootloader.hex tiny.hex miv.hex bl.hex

bootloader.elf: $(SRC_FILES) bootloader.ld
	$(RV32_PREFIX)gcc -O2 -fPIC -march=rv32i -mabi=ilp32 -I. -nostartfiles -Tbootloader.ld -o $@ $(SRC_FILES)
#CAUTION: with -O0 the code is too slow for 115200 baud at 12 MHz

%.elf: %.S
	$(RV32_PREFIX)gcc -nostartfiles -Tbootloader.ld -o $@ $<

%.bin: %.elf
	$(RV32_PREFIX)objcopy -O binary $< $@

%.hex: %.bin
	printf "@0 " > $@
	od -An -tx4 -w4 -v $< | cut -b2- >> $@


clean:
	-rm -f *.o *.elf *.bin *.hex


.SECONDARY:
.PHONY: all clean

