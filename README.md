# RudolV

RISC-V processor for real-time systems. 32 bit in-order pipeline with 5 stages.

Features
  * [Multiple target FPGAs](#supported-fpgas)
  * [Predictable timing model](#instruction-timing)
  * [CSR extension interface](#csr-extension-interface)
  * [Simulation with Icarus Verilog or Verilator](#simulation)

Compatibility
  * RV32IMC Zicsr Zifence
  * Privileged machine-level ISA 1.11
  * Formal verification with [riscv-formal](https://github.com/YosysHQ/riscv-formal)
  * [Unit tests](#unit-tests)
      - riscv-tests
      - riscv-compliance (predecessor of riscv-arch-test)
      - riscv-arch-test ([modified version that fits in 64 KiByte](https://github.com/bobbl/riscv-arch-test))
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
| verify/           | Formal verification with riscv-formal               |
| config.sh         | Configure paths to external tools                   |




Supported FPGAs
---------------

| Manufacturer    | Intel      | Lattice       | Microsemi       | Xilinx    |
| --------------- |:----------:|:-------------:|:---------------:|:---------:|
| FPGA            | Cyclone IV | iCE40 UP5K    | IGLOO2 M2GL025  | Kinex-7   |
| Board           | DE2-115    | UltraPlus MDP | Future Creative | Genesys-2 |
| Tool            | Quartus    | IceStorm      | Libero          | Vivado    |
| Logic Elements  | 3088       | 3795          | 4524            | 684       |
| LUTs            | 2975       | 3162          | 4484            | 2136      |
| DFFs            | 1202       | 1176          | 2346            | 1183      |
| MHz             | 68         | 24            | 65              | 200       |
| CoreMark        | 88         | 31            | 844             | 259       |

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
two cycles.

The jump target is computed in the execute stage, resulting in a two cycle
latency. There is no dynamic branch prediction but subsequent instructions
are only killed in the case of a taken branch. This behaviour can be considered
as a static always not taken prediction. If the branch target is not aligned to
a 32-bit boundary, a taken branch or unconditional jump needs an additional
cycle.

| instruction class  | examples                                       | cycles |
| ------------------ | ---------------------------------------------- | ------ |
| RV32M              | MUL, MULH, MULHSU, MULHU, DIV, DIVU, REM, REMU | 35     |
| pipeline flush     | FENCE.I                                        | 3/4    |
| exception          | ECALL, EBREAK                                  | 3/4    |
| unconditional jump | JAL, JALR                                      | 3/4    |
| taken branch       | BEQ, BNE, BLT, BGE, BLTU, BGEU                 | 3/4    |
| CSR access         | CSRRW, CSRRS, CSRRC                            | 2      |
| memory access      | LB, LH, LW, LBU, LHU, SB, SH, SW               | 2      |
| not taken branch   | BEQ, BNE, BLT, BGE, BLTU, BGEU                 | 1      |
| barrel shifter     | SLL, SRL, SRA                                  | 1      |
| arithmetic         | ADD, SUB, SLT, SLTU, XOR, OR, AND, LUI, AUIPC  | 1      |




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
is separated from the actual read or write action. If `addr` holds an CSR
address that is supported by the extension, the `valid` must be set
asynchronously. The signal must be combinational to enable fast illegal CSR
detection.

In the following cycle, `read` is set if the CSR value should be read. If a
valid register was selected by `addr` in the previous cycle, `rdata` should be
set to the value of the register. It is only asserted for one cycle, otherwise
it is cleared to 0. Caution: `addr` is no longer valid in this cycle, therefore
the extension is responsible for registered select signals from the previous
cycle.

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
| CsrIDs         | 0F1x | `mvendorid`, `marchid`, `mimpid`, `mhartid`, `misa` |
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




Unit Tests
----------

The
[RISC-V Compliance Tests](https://github.com/riscv/riscv-compliance/releases/tag/1.0)
were renamed to RISC-V Architecture Tests and have undergone major revisons. 
There are much more tests and they are auto-generated, but unfortunately the old
compliance tests were removed and the integration of 
[riscv-tests](https://github.com/riscv/riscv-tests) was canceled.

Therefore local copies of the old tests are maintained in this repository.
To build the compliance tests run:

    make -C sw/compliance/

The riscv-tests are extended by some RudolV-specific tests and can be build by

    make -C sw/tests/

Run all tests with Icarus:

    cd scripts/sim
    ./make.sh icarus tests

Simulate only a single test with debug output:

    cd scripts/sim
    ./make.sh icarus debug ../../sw/tests/build/rudolv_irq.elf

There is another, more serious problem with the new
[RISC-V Architecture Tests](https://github.com/riscv/riscv-arch-test):
the binaries are extremely large and require up to 1.7 MiBytes of memory.
And there are
[no plans to reduce the memory footprint](https://github.com/riscv/riscv-arch-test/issues/157).

A closer look at the source code reveals, that the reason for the memory demand
is simple: When branches or jumps are tested, the memory between instruction
and target is filled with nops. Only 3 jumps are responsible for the 1.7 MiByte
binary. In addition, the branch tests can be optimised by overlapping branch
instructions and targets. With these two modifications, all riscv-arch-tests fit
into 64 KiBytes.

The modified tests are in a
[separate repository](https://github.com/bobbl/riscv-arch-test).
To test RudolV with them use

    cd sw/testsuites/
    ./make.sh riscv-arch-test




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




History
-------

RudolV started as a submission to the
[2018 RISC-V SoftCPU Contest](https://riscv.org/2018contest/) called Danzig.
Also participating in the 
[2019 RISC-V SoftCPU Contest on Security](https://riscv.org/2019/07/risc-v-softcpu-core-contest/)
it made the 
[third place](https://riscv.org/2019/10/announcing-the-winners-of-the-risc-v-soft-cpu-contest/)
.




License
-------
Licensed under the ISC licence (similar to the MIT/Expat license).
