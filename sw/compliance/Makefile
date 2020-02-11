include ../../config_default.mk

FILES_S = $(wildcard src/*.S)
FILES_HEX = $(FILES_S:src/%.S=build/%.hex)
files_elf = $(FILES_S:src/%.S=build/%.elf)


build/%.elf: src/%.S ../rudolv.ld riscv_test.h aw_test_macros.h test_macros.h riscv_test_macros.h
	$(RV32I_PREFIX)gcc -march=rv32im -mabi=ilp32 -I. -nostdlib -T../rudolv.ld -o $@ $<

all: build/ $(files_elf)

clean:
	-rm -f build/*
#	-rmdir build

build/:
	-mkdir build

.SECONDARY:
.PHONY: all clean

