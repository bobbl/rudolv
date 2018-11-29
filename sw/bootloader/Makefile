include ../../config_default.mk

PROJ=bootloader

SRC_FILES = crt.S $(PROJ).c

all: $(PROJ).hex

$(PROJ).elf: $(SRC_FILES) bootloader.ld
	$(RV32_PREFIX)gcc -O2 -march=rv32i -I. -nostartfiles -Tbootloader.ld -o $@ $(SRC_FILES)

%.bin: %.elf
	$(RV32_PREFIX)objcopy -O binary $< $@

%.hex: %.bin
	printf "@0 " > $@
	od -An -tx4 -w4 -v $< | cut -b2- >> $@


clean:
	-rm -f *.o *.elf *.bin *.hex


.SECONDARY:
.PHONY: all clean

