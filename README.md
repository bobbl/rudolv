# RudolV

RISC-V processor for real-time systems. 32 bit in-order pipeline with 5 stages.

Features
  * [Multiple target FPGAs](#supported-fpgas)
  * [Predictable timing model](#instruction-timing)
  * [Security extension](#security-extension)
  * [CSR extension interface](#csr-extension-interface)
  * [Simulation with Icarus Verilog or Verilator](#simulation)

Compatibility
  * RV32IM Zcsr Zifence
  * Privileged machine-level ISA 1.11
  * [RISC-V compliance tests](#risc-v-compliance-tests)
  * [Zephyr 2.2.0 support](#zephyr-support)

Benchmarks
  * [Dhrystone](#Dhrystone-Benchmark): Dhrystone MIPS/MHz: 0.736 ... 1.815 
    (depending on Dhrystone implementation)
  * [CoreMark](#EEMBC-CoreMark-Benchmark):  1.295 per MHz




Structure of Repository
-----------------------

| Path              | Content
| ----------------- | --------------------------------------------------- |
| scripts/          | **Scripts for synthesis and simulation**            |
| scripts/sim/      | Simulation with Icarus Verilog or Verilator         |
| scripts/icestorm/ | Synthesis with ICEStorm for Lattice FPGAs           |
| scripts/libero/   | Synthesis with Libero for Microsemi FPGAs           |
| scripts/quartus/  | Synthesis with Quartus for Intel FPGAs              |
| scripts/vivado/   | Synthesis with Vivado for Xilinx FPGAs              |
| src/              | **CSR extensions and FPGA-specific verilog code**   |
| sw/               | **Software that can be executed on RudolV**         |
| sw/compliance/    | RISC-V compliance tests                             |
| sw/coremark/      | EEMBC CoreMark benchmark                            |
| sw/dhrystone/     | Multiple implementations of the Dhrystone benchmark |
| sw/tests/         | RISC-V tests                                        |
| sw/uart/          | UART driver and bootloader                          |
| sw/zephyr/        | Zephyr port                                         |
| config.sh         | Configure paths to external tools                   |




Supported FPGAs
---------------

| Manufacturer    | Intel      | Lattice       | Microsemi       | Xilinx    |
| --------------- |:----------:|:-------------:|:---------------:|:---------:|
| FPGA            | Cyclone IV | iCE40 UP5K    | IGLOO2 M2GL025  | Kinex-7   |
| Board           | DE2-115    | UltraPlus MDP | Future Creative | Genesys-2 |
| Tool            | Quartus    | IceStorm      | Libero          | Vivado    |
| Logic Elements  | 2625       | 3434          | 3804            | 545       |
| LUTs            | 2550       | 2661          | 3751            | 1728      |
| DFFs            | 1089       | 1147          | 2190            | 1077      |
| MHz             | 70         | 25            | 96              | 200       |
| CoreMark        | 86         | 32            | 124             | 259       |

Features:
  * 64 KiByte RAM (Microsemi only 56 KiByte)
  * UART with 115200 Bit/s
  * 32 bit timer and interrupt
  * Counters: `CYCLE` and `INSTRET`
  * Bootloader waits for an executable image via UART

### Synthesis

For FPGA support change to the manufacturer-specific directory and run the make
script

    cd scripts/TOOLNAME
    ./make.sh

to display the make targets. They differ from FPGA to FPGA, but all have in
common, that the first argument must be the board name. To do the synthesis, use

    ./make.sh BOARD synth

On some boards, a separate place and route pass is necessary:

    ./make.sh BOARD pnr

Programming the device is done with

    ./make.sh BOARD prog

Programming (and UART communication) requires some access rights that can be
granted by udev rules. The udev rules for all supported boards can be found in
[rudolv/scripts/53-fpgas-supported-by-rudolv.rules](rudolv/scripts/53-fpgas-supported-by-rudolv.rules).
Simply copy this file to `/etc/udev/rules.d/`.

### Bootloader

After programming or pushing the reset button, RudolV waits for the binary
application image to be transfered via the UART. The format is simple: first
send the length of the image (in bytes) as decimal ASCII string. Followed by a
blank (0x20) and then the binary data. The following script can be used:

    sw/bootloader/send_elf.sh /dev/ttyUSB0 IMAGE.elf

Use a terminal emulator like picocom at 115200 baud to communicate with the
program:

    picocom -b 115200 --imap lfcrlf /dev/ttyUSB0




Instruction timing
------------------

The architecture avoids speculative components to provide a predictable timing
as required by hard real-time systems.

Data hazards are avoided by operand forwarding, therefore most instructions are
executed in one cycle. Since the memory is single ported, memory accesses take
two cycles. The jump target is computed in the execute stage, resulting in a two
cycle latency. There is no dynamic branch prediction but subsequent instructions
are only killed in the case of a taken branch. This behaviour can be considered
as a static always not taken prediction.

| instruction class  | examples                                       | cycles |
| ------------------ | ---------------------------------------------- | ------ |
| RV32M              | MUL, MULH, MULHSU, MULHU, DIV, DIVU, REM, REMU | 34     |
| pipeline flush     | FENCE.I                                        | 3      |
| exception          | ECALL, EBREAK                                  | 3      |
| unconditional jump | JAL, JALR                                      | 3      |
| taken branch       | BEQ, BNE, BLT, BGE, BLTU, BGEU                 | 3      |
| CSR access         | CSRRW, CSRRS, CSRRC                            | 2      |
| memory access      | LB, LH, LW, LBU, LHU, SB, SH, SW               | 2      |
| not taken branch   | BEQ, BNE, BLT, BGE, BLTU, BGEU                 | 1      |
| barrel shifter     | SLL, SRL, SRA                                  | 1      |
| arithmetic         | ADD, SUB, SLT, SLTU, XOR, OR, AND, LUI, AUIPC  | 1      |




Security extension
------------------

RudolV started as a submission to the
[2018 RISC-V SoftCPU Contest](https://riscv.org/2018contest/) called Danzig.
Also participating in the 
[2019 RISC-V SoftCPU Contest on Security](https://riscv.org/2019/07/risc-v-softcpu-core-contest/)
it made the 
[third place](https://riscv.org/2019/10/announcing-the-winners-of-the-risc-v-soft-cpu-contest/)
.

The objective of the contest was to thwart 5 particular attacks of RIPE, the
[Runtime Intrusion Prevention Evaluator](https://github.com/johnwilander/RIPE).
RudolV can detect all these 5 attacks and responds with an exception. No
compiler modifications are necessary.

### Grubby attack detection

RudolV monitors if a memory location was written as a whole 32 bit word or if
narrow byte accesses modified it. Memory locations that were not written as a
whole are called _grubby_. If an instruction is fetched from such a grubby
memory location, an exception (14) is thrown. If the value from a grubby memory
location is used as function pointer or return address, exception 10 is throw.

### Technical implementation

Each memory word requires an additional grubby bit. The bit is cleared, if a 
aligned `sw` instruction writes the word. The other store instructions (`sh` and
`sb`) set the grubby bit. If a word with grubby bit set is fetched, execution
continues at the address `mtvec` and `mcause` is set to 14.

The detection of an indirect jump to an address from a grubby memory location is
a bit more complex. There is no single instruction to jump to the address in a
memory location. At least two instructions are involved instead. First, `lw`
reads the target address from memory to a register and then `jalr` jumps to the
address in the register. Therefore, all registers also need a grubby bit. It is
written by a load and tested by a jump.

### Hardware costs

The hardware costs are not very high. The logic is simple (only a few dozend LUT4s)
and most FPGAs provide block RAMs with parity bits that are 36 bits wide, not 32
bits. These parity bits can be used as grubby bits. Only the Lattice FPGAs do not
provide a parity bit.

| Manufacturer    | Intel      | Lattice       | Microsemi       | Xilinx    |
| --------------- |:----------:|:-------------:|:---------------:|:---------:|
| Logic Elements  |       2622 |          3434 |            3884 |       539 |
| LUTs            |       2562 |          2661 |            3822 |      1734 |
| DFFs            |       1096 |          1147 |            2195 |      1084 |
| MHz             |         71 |            22 |              96 |       200 |

If available on the board, a switch (DE2-115, Genesys-2) or a button (IGLOO2)
can disable the grubby detection.

### Detection quality

All buffer overflows that write the memory byte by byte can be detected with the
grubby mechanism. RIPE tests 850 different stack overflow attacks, subdivided by
five dimensions. Four dimensions (location, target code pointer, overflow technique
and attack code) are irrelevat for the grubby detection, the only dimension that
has an influence is the abused function. Grubby detection only works for functions
that write the memory byte by byte. Fortunately, 9 of the 10 functions use `sb`
to write to memory (at least in the glibc implementation). Only `memcpy()` writes
on a word basis and is resistant to the grubby detection.

Replacing `memcpy()` by a byte by byte implementation is not an option, because
`memcpy()` may be used for legal copies of function pointers that may lead to false
positives. So far, no other false positives are known, as long as no dirty hacks
are used in the source code.

### Comparison with Pulserain Rattlesnake

The grubby detection is very similar to the _Dirty Address Trace_ (DAT) technique
used by [Pulserain Rattlesnake](https://github.com/PulseRain/Rattlesnake). It
also needs an additional bit per memory word and register. The dirty bit is set,
if more than 8 consecutive writes to memory are detected. 

DAT also covers word by word like `memcpy()` attacks, but it is vulnerable to attacks
with short consecutive writes and contexts switches during a write sequence.
I may be biased, but in my opinion, the DAT and grubby detection rate are roughly
comparable. However, the hardware costs of the grubby detection are lower.




CSR Extension Interface
-----------------------

Extra CSRs can be added with the follwing interface:

    input         clk
    input         read
    input   [2:0] modify
    input  [31:0] wdata
    input  [11:0] addr
    output [31:0] rdata
    output        valid

`addr` is valid one cycle earlier that the other signals. Thus, address decoding
is separated from the actual read or write action.

In the following cycle, `read` is set if the CSR value should be read. If a valid
register was selected by `addr` in the previous cycle, `valid` should be set to
high and `rdata` to the value of the register. Both signals are only asserted for
one cycle, otherwise both are cleared to 0.

`modify` defines, how the current CSR value should be combined with `wdata` to
form the new value.

| `modify` | action | instruction | function             |
| -------- | ------ | ----------- | -------------------- |
| 001      | write  | CSRW        | `csr = wdata`        |
| 010      | set    | CSRS        | `csr = csr or wdata` |
| 011      | clear  | CSRC        | `csr = csr &~ wdata` |

This interface can also be used to connect external peripherals. Its advantage
over a memory mapped interface is less impact on the critcal path and thus a
potentially higher clock rate.

Examples of CSR extensions can be found in [src/csr.v](src/csr.v):

| verilog module | no   | CSRs / description                                  |
| -------------- | ---- | --------------------------------------------------- |
| CsrIDs         | 0F1x | `vendorid`, `archid`, `impid`, `hartid`, clock rate |
| CsrCounter     | 0C0x | `cycle`, `time`, `instret`                          |
| CsrUartBitbang | 07C0 | Minimal UART interface for software bitbanging      |
| CsrUartChar    | 0BC0 | Read and write bytes via UART                       |
| CsrPinsIn      | 0FC1 | Read external pins (e.g. buttons or switches)       |
| CsrPinsOut     | 0BC1 | Write external pins (e.g. LEDs)                     |
| CsrTimerAdd    | 0BC2 | Simple timer that asserts the timer interupt        |




Simulation
----------

For simulation use the make script in [scripts/sim/](scripts/sim):

    cd scripts/sim
    ./make.sh

will display the usage of the script.

Icarus Verilog and Verilator are supported and any .elf file can be executed
with optional debug output and .vcd generation.




RISC-V Compliance Tests
-----------------------

Meanwhile the [RISC-V compliance tests](https://github.com/riscv/riscv-compliance)
include most of the older [riscv-tests](https://github.com/riscv/riscv-tests),
but some tests for multiplication and division are missing. Therefore these
missing tests and some RudolV-specific tests were added. They can be found
under [sw/tests/src/](sw/tests/src/).

Build the compliance tests:

    make -C sw/compliance/

Build the remaining riscv-tests and rudolv-tests:

    make -C sw/tests/

Run all tests with Icarus:

    cd scripts/sim
    ./make.sh icarus tests

Simulate only a single test with debug output:

    cd scripts/sim
    ./make.sh icarus debug ../../sw/tests/build/rudolv_irq.elf





Zephyr Support
--------------

RudolV supports Zephyr with an out-of-tree board description. This means that
a plain, unpatched installation of Zephyr can be used to build an .elf file for
RudolV. To install Zephyr follow the instructions on 
https://docs.zephyrproject.org/latest/getting_started/index.html, particularly
topics 2 (dependencies) and 5 (SDK). Topics 3 (source code) and 4 (python 
dependencies) are integrated in RudolV's make script:

    cd sw/zephyr
    ./make.sh zephyr

Technically, a complete Zephyr port consists of board, DeviceTree, SoC and 
driver definitions. However, as of Zephyr 2.2.0, out-of-tree SoC definitions and
drivers do not yet work properly. Therefore, all RudolV customisations are
packed into the board definition at [sw/zephyr/board/](sw/zephyr/board/).

For example, tu build the hello_word sample, type

    cd sw/zephyr
    west build -b rudolv ./zephyrproject/zephyr/samples/hello_world -- -DBOARD_ROOT=.

or use the make script:

    ./make.sh elf ./zephyrproject/zephyr/samples/hello_world




Dhrystone Benchmark
-------------------

Dhrystone is widely used, but it depends to such an extend on the compiler
settings and libc implementation that it is hardly comparable. Here
are 3 different implementations from repositories of other RISC-V cores.
Build them with

    cd sw/dhrystone
    ./make.sh all

And send the images in `sw/dhrystone/build/` to the bootloader. Or run it with
Icarus Verilog:

    cd scripts/icarus
    ./make.sh run ../../sw/dhrystone/build/riscv.elf 
    ./make.sh run ../../sw/dhrystone/build/picorv32.elf 
    ./make.sh run ../../sw/dhrystone/build/scr1.elf 

| Dhrystone source | reference core | F<sub>max</sub> | cycles/iteration | RudolV c/i | DMIPS/MHz | RudolV D/M |
| ----------------------------------------------------------------- | -------- |:-------:|:----:|:---:|:-----:|:-----:|
| [riscv/riscv-tests](https://github.com/riscv/riscv-tests)         |          |         |      | 773 |       | 0.734 |
| [cliffordwolf/picorv32](https://github.com/cliffordwolf/picorv32) | PicoRV32 | 400 MHz | 1100 | 615 | 0.516 | 0.924 |
| [syntacore/scr1-sdk](https://github.com/syntacore/scr1-sdk)       | SCR1     |  30 MHz |  301 | 313 | 1.896 | 1.815 |




EEMBC CoreMark Benchmark
------------------------

Run one iteration of [EEMBC CoreMark](https://www.eembc.org/coremark/) with Icarus Verilog:

    cd sw/coremark/
    ./make.sh icarus

The results can be found in `sw/coremark/coremark/run1.log`. Since this is a
simulation, the error message can be ignored and the computed iterations/sec
corresponds to CoreMark/MHz.

To get the CoreMark of an FPGA implementation, build an ELF image with UART
output that executes 1000 iterations and send it to the bootloader:

    cd sw/coremark/
    ./make.sh elf 1000
    ../bootloader/send_elf.sh /dev/ttyUSB1 build/coremark_iter_1000.elf

CoreMark requires the benchmark to run for at least 10 seconds. If RudolV is too
fast, build an ELF file image with more iterations and send this one to the
bootloader.

The CoreMark/MHz of RudolV is 1.295 and 0.892 without bitwise multiplier and
divider.




Dependencies
------------

For simulation, either [Icarus Verilog](http://iverilog.icarus.com/) or
[Verilator](https://www.veripool.org/wiki/verilator/) are required. Most Linux
distributions offer packages for them. For example Ubuntu:

    sudo apt install iverilog verilator

Open source synthesis for Lattice FPGAs is possible with
[Project Icestorm](http://www.clifford.at/icestorm/). Installation guidelines
are on the website. Synthesis for the other FPGAs is only possible with
vendor-specific toolchains (Libero, Quartus, Vivado). A guide to install
Libero can be found 
[here](https://bobbl.github.io/fpga/microsemi/2019/09/23/install-libero.html).

To build the software, a RISC-V assembler and a C compiler with libgcc for
RV32IM is required. Install the GNU toolchain to `$DESTDIR` with

    git clone --recursive https://github.com/riscv/riscv-gnu-toolchain
    cd riscv-gnu-toolchain
    mkdir build
    cd build
    ../configure --prefix=$DESTDIR --with-arch=rv32im
    make

By default, the build scripts of RudolV expect that the external toolchains
are in the search path (see [config_default.sh](config_default.sh). To set
specific path for every tool, copy `config_default.sh` to `config.sh` and
edit the paths there.




License
-------
Licensed under the ISC licence (similar to the MIT/Expat license).
