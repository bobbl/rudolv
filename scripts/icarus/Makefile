include ../../config_default.mk

PATH_TESTS = ../../sw/tests
PATH_COMPLIANCE = ../../sw/compliance
VERILOG_FILES = ../../pipeline.v


FILES_S = $(wildcard $(PATH_TO_TESTS)/*.S)
FILES_HEX = $(FILES_S:%.S=%.hex)

all: test-all

test: $(CODE) tb_tests.v $(VERILOG_FILES)
	$(IVERILOG) -o tmp.vvp -DDEBUG -DCODE=\"$(CODE)\" tb_tests.v $(VERILOG_FILES)
	$(VVP) -N tmp.vvp

picorv32-dhrystone:
	$(MAKE) -C ../../sw/picorv32-dhrystone
	$(IVERILOG) -o tmp.vvp -DDHRYSTONE -DCODE=\"../../sw/picorv32-dhrystone/dhrystone.hex\" \
	    tb_dhrystone.v $(VERILOG_FILES)
	$(VVP) -N tmp.vvp

riscv-dhrystone:
	$(MAKE) -C ../../sw/riscv-dhrystone
	$(IVERILOG) -o tmp.vvp -DDHRYSTONE -DCODE=\"../../sw/riscv-dhrystone/dhrystone.hex\" \
	    tb_dhrystone.v $(VERILOG_FILES)
	$(VVP) -N tmp.vvp

uart:
	$(IVERILOG) -o tmp.vvp -DDEBUG -DCODE=\"../../sw/uart/uart.hex\" \
	    tb_uart.v $(VERILOG_FILES)
	$(VVP) -N tmp.vvp


clean:
	rm -f test.vvp

.SECONDARY:
.PHONY: all test-all compliance riscv-dhrystone dhrystone clean



