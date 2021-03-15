include ../../../config_default.mk

CC = $(RV32I_PREFIX)gcc
LD = $(RV32I_PREFIX)ld
AS = $(RV32I_PREFIX)as
PORT_CFLAGS = -O2 -march=rv32im -mabi=ilp32
LFLAGS_END = -nostartfiles -T$(PORT_DIR)/link.ld
PORT_SRCS = $(PORT_DIR)/core_portme.c $(PORT_DIR)/ee_printf.c $(PORT_DIR)/crt.S

PORT_OBJS = $(PORT_DIR)/core_portme$(OEXT) $(PORT_DIR)/ee_printf$(OEXT)
PORT_CLEAN = *$(OEXT)

LOAD = @echo "No loading necessary"
RUN = $(VVP) -N tmp.vvp

OUTFLAG= -o
FLAGS_STR = "$(PORT_CFLAGS) $(XCFLAGS) $(XLFLAGS) $(LFLAGS_END)"
CFLAGS = $(PORT_CFLAGS) -I$(PORT_DIR) -I. -DFLAGS_STR=\"$(FLAGS_STR)\" 

#SEPARATE_COMPILE=1
OBJOUT = -o
LFLAGS =
ASFLAGS =
OFLAG = -o
COUT = -c
OEXT = .o
EXE = .elf
OPATH = ./
MKDIR = mkdir -p

vpath %.c $(PORT_DIR)
vpath %.s $(PORT_DIR)

$(OPATH)$(PORT_DIR)/%$(OEXT) : %.c
	$(CC) $(CFLAGS) $(XCFLAGS) $(COUT) $< $(OBJOUT) $@

$(OPATH)%$(OEXT) : %.c
	$(CC) $(CFLAGS) $(XCFLAGS) $(COUT) $< $(OBJOUT) $@

$(OPATH)$(PORT_DIR)/%$(OEXT) : %.s
	$(AS) $(ASFLAGS) $< $(OBJOUT) $@



.PHONY : port_prebuild port_postbuild port_prerun port_postrun port_preload port_postload

port_pre% :

port_postrun:

port_postload:

port_postbuild:
	$(RV32I_PREFIX)objcopy -O binary coremark.elf coremark.bin
	printf "@0 " > coremark.hex
	od -An -tx4 -w4 -v coremark.bin | cut -b2- >> coremark.hex
	$(IVERILOG) -o tmp.vvp -DCODE=\"coremark.hex\" \
	    $(PORT_DIR)/tb_coremark.v ../../../pipeline.v ../../../src/csr.v \
	    ../../../src/regset33.v ../../../src/memory.v

