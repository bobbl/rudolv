# clone CoreMark from github, build for Danzig and run with Icarus

all: build-uart run-icarus


# clone coremark from github and modify
clone: coremark/.git/ coremark/danzig_icarus/

coremark/.git/:
	git clone https://github.com/eembc/coremark.git

coremark/danzig_icarus/:
	cp -r portme/* coremark/

run-icarus: clone
	$(MAKE) -C coremark ITERATIONS=1 PORT_DIR=danzig_icarus clean run
	cp coremark/coremark.hex coremark_sim.hex

build-uart: clone
	$(MAKE) -C coremark PORT_DIR=danzig_uart clean coremark.elf
	cp coremark/coremark.bin coremark_uart.bin

clean:
	$(MAKE) -C coremark PORT_DIR=danzig_icarus clean
	$(MAKE) -C coremark PORT_DIR=danzig_uart clean


.SECONDARY:
.PHONY: all clone run-icarus build-uart clean

