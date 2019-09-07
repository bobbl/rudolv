include ../../config_default.mk

all: uart.hex uart_miv.hex

uart.elf: crt.S uart.c link.ld
	$(RV32_PREFIX)gcc -O2 -march=rv32i -I. -nostartfiles -Tlink.ld -o $@ crt.S uart.c
#CAUTION: with -O0 the code is too slow for 115200 baud at 12 MHz

uart_miv.elf: crt_miv.S uart.c miv.ld
	$(RV32_PREFIX)gcc -O1 -march=rv32i -I. -nostartfiles -Tmiv.ld -o $@ crt_miv.S uart.c



%.bin: %.elf
	$(RV32_PREFIX)objcopy -O binary $< $@

%.hex: %.bin
	printf "@0 " > $@
	od -An -tx4 -w4 -v $< | cut -b2- >> $@


clean:
	-rm -f *.o *.elf *.bin *.hex


.SECONDARY:
.PHONY: all clean

