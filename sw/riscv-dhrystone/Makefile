include ../../config.mk
#RV32_PREFIX=/home/bobbl/git/rcmc/tools/rv64i/bin/riscv64-unknown-elf-

SRC_FILES = crt.S syscalls.c dhrystone_main.c dhrystone.c
#SRC_FILES = crt.S syscalls.c hello.c

all: dhrystone.hex

#%.elf: %.c link.ld
#	$(RV32_PREFIX)gcc -march=rv32i -I. -nostdlib -Tlink.ld -o $@ $<

#%.o: %.c
#	$(RV32_PREFIX)gcc -march=rv32i -I. -c -o $@ $<

#%.o: %.S
#	$(RV32_PREFIX)gcc -march=rv32i -I. -c -o $@ $<

#dhrystone.elf: crt.o syscalls.o dhrystone_main.o dhrystone.o
#	$(RV32_PREFIX)gcc -march=rv32i -I. -nostartfiles -Ttest.ld -o $@ $^




dhrystone.elf: $(SRC_FILES) dhrystone.h util.h test.ld
#	$(RV32_PREFIX)gcc -O3 -fno-inline -march=rv32i -I. -nostdlib -Ttest.ld -o $@ $(SRC_FILES) -lgcc
	$(RV32_PREFIX)gcc -O3 -fno-inline -march=rv32i -I. -nostartfiles -Ttest.ld -o $@ $(SRC_FILES)



%.bin: %.elf
	$(RV32_PREFIX)objcopy -O binary $< $@

%.hex: %.bin
	printf "@0 " > $@
	od -An -tx4 -w4 -v $< | cut -b2- >> $@


clean:
	-rm -f *.o *.elf *.bin *.hex


.SECONDARY:
.PHONY: all clean

