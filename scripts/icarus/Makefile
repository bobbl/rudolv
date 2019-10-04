include ../../config_default.mk

VERILOG_FILES = ../../src/memory32.v ../../src/regset33.v ../../pipeline.v

help:
	@echo "  make CODE=foo.hex test          Simulate with extensive logging"
	@echo "  make CODE=foo.hex test-nodebug  Simulate without logging"
	@echo "  make CODE=foo.hex miv           Simulate Microsemi Mi-V memory map"
	@echo "  make CODE=foo.hex miv-nodebug   Simulate Microsemi Mi-V memory map w/o logging"
	@echo
	@echo "CODE="
	@echo "  ../../sw/picorv-dhrystone/dhrystone.hex        Benchmark from picorrv32 repo"
	@echo "  ../../sw/riscv-dhrystone/dhrystone.hex         Benchmark from RISC-V repo"

test: $(CODE) tb_tests.v $(VERILOG_FILES)
	$(IVERILOG) -o tmp.vvp -DDEBUG -DCODE=\"$(CODE)\" \
	    tb_tests.v $(VERILOG_FILES)
	$(VVP) -N tmp.vvp

test-nodebug: $(CODE) tb_tests.v $(VERILOG_FILES)
	$(IVERILOG) -o tmp.vvp -DCODE=\"$(CODE)\" \
	    tb_tests.v $(VERILOG_FILES)
	$(VVP) -N tmp.vvp

miv:
	$(IVERILOG) -o tmp.vvp -DDEBUG -DCODE=\"$(CODE)\" \
	    tb_miv.v ../../src/memory36.v $(VERILOG_FILES)
	$(VVP) -N tmp.vvp

miv-nodebug:
	$(IVERILOG) -o tmp.vvp -DCODE=\"$(CODE)\" \
	    tb_miv.v ../../src/memory36.v $(VERILOG_FILES)
	$(VVP) -N tmp.vvp



clean:
	rm -f test.vvp

.SECONDARY:
.PHONY: all test test-nodebug miv miv-nodebug clean
