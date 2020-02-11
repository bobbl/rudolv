include ../../config_default.mk

FILES_S = $(wildcard src/*.S)
FILES_HEX = $(FILES_S:src/%.S=build/%.hex)
files_elf = $(FILES_S:src/%.S=build/%.elf)

build/%.elf: src/%.S riscv_test.h test_macros.h ../rudolv.ld
	$(RV32I_PREFIX)gcc -march=rv32im -mabi=ilp32 -I. -nostdlib -T../rudolv.ld -o $@ $<

all: build/ $(files_elf)

clean:
	-rm -f build/*
#	-rmdir build

build/:
	-mkdir build

.SECONDARY:
.PHONY: all clean

