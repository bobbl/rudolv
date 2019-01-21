include ../../config_default.mk

SRC_FILES = crt.S syscalls.c dhrystone_main.c dhrystone.c

all: dhrystone.hex

dhrystone.elf: $(SRC_FILES) dhrystone.h util.h test.ld
	$(RV32I_PREFIX)gcc -O3 -fno-inline -march=rv32i -I. -nostartfiles -Ttest.ld -o $@ $(SRC_FILES)

%.bin: %.elf
	$(RV32I_PREFIX)objcopy -O binary $< $@

%.hex: %.bin
	printf "@0 " > $@
	od -An -tx4 -w4 -v $< | cut -b2- >> $@

clean:
	-rm -f *.o *.elf *.bin *.hex

.SECONDARY:
.PHONY: all clean

