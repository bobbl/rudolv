include ../../config_default.mk

PATH_TESTS = ../../sw/tests
PATH_COMPLIANCE = ../../sw/compliance
VERILOG_FILES = ../../pipeline.v


FILES_S = $(wildcard $(PATH_TO_TESTS)/*.S)
FILES_HEX = $(FILES_S:%.S=%.hex)

help:
	@echo "  make CODE=foo.hex test          Simulate with extensive logging"
	@echo "  make CODE=foo.hex test-nodebug  Simulate without logging"
	@echo "  make CODE=foo.hex femcb         Simulate Future Electronics Microsemi Creative Board"
	@echo "  make picorv-dhrystone           Benchmark from picorrv32 repo"
	@echo "  make riscv-dhrystone            Benchmark from RISC-V repo"

test: $(CODE) tb_tests.v $(VERILOG_FILES)
	$(IVERILOG) -o tmp.vvp -DDEBUG -DCODE=\"$(CODE)\" \
	    tb_tests.v memory32.v $(VERILOG_FILES)
	$(VVP) -N tmp.vvp

test-nodebug: $(CODE) tb_tests.v $(VERILOG_FILES)
	$(IVERILOG) -o tmp.vvp -DCODE=\"$(CODE)\" \
	    tb_tests.v memory32.v $(VERILOG_FILES)
	$(VVP) -N tmp.vvp

picorv32-dhrystone:
	$(MAKE) -C ../../sw/picorv32-dhrystone
	$(IVERILOG) -o tmp.vvp -DCODE=\"../../sw/picorv32-dhrystone/dhrystone.hex\" \
	     -DDHRYSTONE tb_dhrystone.v memory32.v $(VERILOG_FILES)
	$(VVP) -N tmp.vvp

riscv-dhrystone:
	$(MAKE) -C ../../sw/riscv-dhrystone
	$(IVERILOG) -o tmp.vvp -DCODE=\"../../sw/riscv-dhrystone/dhrystone.hex\" \
	     -DDHRYSTONE tb_dhrystone.v memory32.v $(VERILOG_FILES)
	$(VVP) -N tmp.vvp

uart:
	$(IVERILOG) -o tmp.vvp -DDEBUG -DCODE=\"../../sw/uart/uart.hex\" \
	    tb_uart.v $(VERILOG_FILES)
	$(VVP) -N tmp.vvp

femcb:
	$(IVERILOG) -o tmp.vvp -DDEBUG -DCODE=\"$(CODE)\" \
	    tb_femcb.v memory32.v $(VERILOG_FILES)
	$(VVP) -N tmp.vvp

femcb-nodebug:
	$(IVERILOG) -o tmp.vvp -DCODE=\"$(CODE)\" \
	    tb_femcb.v memory32.v $(VERILOG_FILES)
	$(VVP) -N tmp.vvp



clean:
	rm -f test.vvp

.SECONDARY:
.PHONY: all test-all compliance riscv-dhrystone dhrystone clean
