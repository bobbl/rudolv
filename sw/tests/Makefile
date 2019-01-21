include ../../config_default.mk

FILES_S = $(wildcard src/*.S)
FILES_HEX = $(FILES_S:src/%.S=build/%.hex)


build/%.elf: src/%.S ../riscv_test.h ../test_macros.h ../danzig.ld
	$(RV32_PREFIX)gcc -march=rv32i -mabi=ilp32 -I.. -nostdlib -T../danzig.ld -o $@ $<

%.bin: %.elf
	$(RV32_PREFIX)objcopy -O binary $< $@

%.hex: %.bin
	printf "@0 " > $@
	od -An -tx4 -w4 -v $< | cut -b2- >> $@

all: build/ $(FILES_HEX)

clean:
	-rm -f build/*
#	-rmdir build

build/:
	-mkdir build

.SECONDARY:
.PHONY: all clean

