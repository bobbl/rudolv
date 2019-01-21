include ../../config_default.mk

USE_MYSTDLIB = 1
OBJS = dhry_1.o dhry_2.o stdlib.o
CFLAGS = -O3 -march=rv32i -mabi=ilp32 -DTIME -DRISCV

ifeq ($(USE_MYSTDLIB),1)
CFLAGS += -DUSE_MYSTDLIB -ffreestanding -nostdlib
OBJS += start.o
else
OBJS += syscalls.o
endif

all: dhrystone.hex

%.bin: %.elf
	$(RV32I_PREFIX)objcopy -O binary $< $@

%.hex: %.bin
	printf "@0 " > $@
	od -An -tx4 -w4 -v $< | cut -b2- >> $@



ifeq ($(USE_MYSTDLIB),1)
dhrystone.elf: $(OBJS) sections.lds
	$(RV32I_PREFIX)gcc $(CFLAGS) -Wl,-Bstatic,-T,sections.lds -o $@ $(OBJS) -lgcc
	chmod -x $@
else
dhrystone.elf: $(OBJS)
	$(RV32I_PREFIX)gcc $(CFLAGS) -Wl,-Bstatic,-T,riscv.ld -o $@ $(OBJS) -lgcc -lc
	chmod -x $@
endif

%.o: %.c
	$(RV32I_PREFIX)gcc -c $(CFLAGS) $<

%.o: %.S
	$(RV32I_PREFIX)gcc -c $(CFLAGS) $<

dhry_1.o dhry_2.o: CFLAGS += -Wno-implicit-int -Wno-implicit-function-declaration

clean:
	rm -rf *.o dhry.map dhrystone.elf dhrystone.map dhrystone.bin dhrystone.hex

.PHONY: all clean
.SECONDARY:


