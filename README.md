# RudolV

A 32 bit RISC-V processor with 5 pipeline stages and in-order execution.
The architecture avoids speculative components to provide a predictable timing
as required by hard real-time systems. It is based on a submission to the 
[2018 RISC-V SoftCPU Contest](https://riscv.org/2018contest/) called Danzig.
RudolV participates in the [2019 RISC-V SoftCPU Contest on Security](https://riscv.org/2019/07/risc-v-softcpu-core-contest/)

__EXCUSE:__ Due to the time consuming contest, the other parts of the
repository currently are not up-to-date.


RISC-V SoftCPU Contest on Security
----------------------------------

The objective of the contest is to thwart 5 particular attacks of RIPE, the
[Runtime Intrusion Prevention Evaluator](https://github.com/johnwilander/RIPE).
RudolV can detect all these 5 attacks and responds with an exception. No
compiler modifications are necessary.


### Requirements

  * Future Electronics - Microsemi Creative Development Board with 
    IGLOO 2 M2GL025-VF256 FPGA
  * Libero 12.1 ([Howto install Libero on Ubuntu](https://bobbl.github.io/fpga/microsemi/2019/09/23/install-libero.html))
  * Zephyr SDK 0.10.0 (only this version fits Zephyr OS v1.14.1-rc1 which is
    required by the contest)


### CPU

  * Plug in the Creative Board.
  * Open the project file `scripts/libero/proj/iot_contest/iot_contest.prjx`
    with Libero.
  * Choose the `schematic` tab and click on the leftmost icon to generate the
    component.
  * Click on _Program Design/Run PROGRAM Action_ within the `Design Flow` tab
    to synthesize the bitstream and program the FPGA board.

Alternatively the complete bitstream can be downloaded as release `iot_contest`
from the RudolV GitHub repository.

After pushing the reset button, RudolV waits for the binary application image to
be transfered via the UART. The format is simple: first send the length of the
image (in bytes) as decimal ASCII string. Followed by a blank (0x20) and then the
binary data. The following script can be used:

    sw/bootloader/send_image.sh /dev/ttyUSB0 image.bin

To check what happens without the grubby attack detection, edit line 74 in
`scripts/libero/proj/iot_contest/hdl/withmem.v` and set `MemRGrubby` to 0.
After synthesis and programming the attacks will be successful.


### Build the software

RudolV is binary compatible to Mi-V for the Creative Board therefore binaries
can be build the same way as descibed in the 
[contest rules](https://github.com/Thales-RISC-V/RISC-V-IoT-Contest). 
After building an application, the raw binary image `zephyr.bin` can be found
in the Zephyr build directory. To run it use

    sw/bootloader/send_image.sh /dev/ttyUSB0 zephyr.bin

To build the RIPE binaries make sure that the environmental variable
`ZEPHYR_SDK_INSTALL_DIR` points to the Zephyr SDK directory.

If there is already a Zephyr installation, set `ZEPHYR_BASE` accoringly and
`ZEPHYR_TOOLCHAIN_VARIANT=zephyr`. Otherwise Zephyr OS v1.14.1-rc1 can be 
instaled locally with

    cd sw/zephyr
    source ./make.sh zephyr

Build the RIPE attack binaries

    ./make.sh ripe



### Run the attacks

Use a terminal emulator like picocom at 115200 baud to receive the output of RIPE:

    picocom -b 115200 /dev/ttyUSB0

Now push the reset button on the Creative Board and transfer the binary image of
an attack to the bootloader of RudolV:

    sw/bootloader/send_image.sh /dev/ttyUSB0 sw/zephyr/build/ripe1.rv32im.bin

For the other attacks use `ripe2` to `ripe5`. 



### How does the detection work?

All attacks have in common, that they alter the memory byte by byte, not with
full 32 bit writes. Therefore RudolV uses an additional _grubby_ bit for each
memory location to keep track of words that were not written as a whole.
If RudolV fetches an instruction word with the grubby bit set, it throws
exception 14. If it loads a grubby word and uses it as target address for a
jump, exception 10 is thrown.


### Chip resources for the IGLOO 2 port of RudolV


| chip resources  | unit      |  number |
|:--------------- | ---------:| -------:|
| LUT4            |           |    3551 |
| flip-flops      |     1 Bit |    2293 |
| LEs             |           |    3633 |
| uRAM            |   1 KiBit |       7 |
| LSRAM           |  18 KiBit |      28 |
| clock frequency |       MHz |      99 |




Instruction timing
------------------

Data hatzards are avoided by operand forwarding, therefore most instructions 
are executed in one cycle. Since the memory is single ported, memory accesses
take two cycles. The jump target is computed in the execute stage, resulting in
a two cycle latency. There is no dynamic branch prediction but subsequent
instructions are only killed in the case of a taken branch. This behaviour can
be considered as a static always not taken prediction.

| instruction class  | examples        | cycles |
| ------------------ | --------------- | ------ |
| RV32M              | MUL, DIV, ...   | 34     |
| pipeline flush     | FENCE.I         | 3      |
| exception          | ECALL, EBREAK   | 3      |
| unconditional jump | JAL, JALR       | 3      |
| taken branch       | BEQ, ...        | 3      |
| CSR access         | CSRRW, ...      | 2      |
| memory access      | LB, LW, SH, ... | 2      |
| not taken branch   | BEQ, ...        | 1      |
| barrel shifter     | SLL, SRL, SRA   | 1      |
| arithmetic         | ADD, ...        | 1      |



Simulation and testing with Icarus Verilog
------------------------------------------

Run riscv-tests with Icarus Verilog

    make -C sw/tests/
    make -C scripts/icarus/ test-all

Run riscv-compliance tests with Icarus Verilog

    make -C sw/compliance/
    cd scripts/icarus/ 
    ./run_compliance_tests.h

Run Dhrystone benchmark from riscv-tests repository wirh Icarus Verilog

    make -C sw/riscv-dhrystone/
    make -C scripts/icarus/ riscv-dhrystone

Run Dhrystone benchmark from picorv32 repository wirh Icarus Verilog

    make -C sw/picorv32-dhrystone/
    make -C scripts/icarus/ picorv32-dhrystone

Dhrystone results:

|                      | DMIPS/MHz | Dhrystones/s/MHz | CPI   | cycles per Dhrystone |
| -------------------- | --------- | ---------------- | ----- | -------------------- |
| `riscv-dhrystone`    | 0.75      | 1362             | 1.66  | 734                  |
| `picorv32-dhrystone` | 0.968     | 1702             | 1.599 | 587                  |



Synthesis with IceStorm
-----------------------

Synthesize with Project IceStorm and flash to a Lattice iCE40 UltraPlus MDP board.
The board must be configured to flash and run FPGA U4.

    make -C sw/bootloader/
    make -C scripts/icestorm/ DEVICE=up5k ARACHNE_DEVICE="5k -P uwg30" clean arachne prog

Now the processor within the FPGA executes the bootloader and waits for data
from the UART. To run the Dhrystone benchmark on the FPGA use:

    make -C sw/uart-dhrystone/
    sw/bootloader/send_elf.sh /dev/ttyUSB1 sw/uart-dhrystone/dhrystone.elf

This Dhrystone version is derived from the risv-tests version, but with the
correct clock frequency of 24 MHz and 100'000 iterations. The result is 32697
Dhrystones per second. To see the output use a terminal emulator like picocom
at 115200 baud:

    picocom -b 115200 --imap lfcrlf /dev/ttyUSB1

The processor logic and the bootloader are written to the flash memory. Hence, the
processor can be reset with switch SW2 of the MDP board. After switching it back and
forth, `send_elf.sh` can be used again to start a new program on the processor.

| chip resources  | unit      | default | no counters | no exceptions | minimal |
|:--------------- | ---------:| -------:| -----------:| -------------:| -------:|
| LCs             |      LUT4 |    1829 |        1622 |          1619 |    1414 |
| BRAM            |   4 KiBit |       6 |           6 |             6 |       6 |
| SPRAM           | 256 KiBit |       2 |           2 |             2 |       2 |
| clock frequency |       MHz |      25 |          23 |            22 |      26 |



CoreMark EEMBC benchmark scores
-------------------------------

Run one iteration of [EEMBC CoreMark](https://www.eembc.org/coremark/) with Icarus Verilog:

    make -C sw/coremark/ run-icarus

The results can be found in `sw/coremark/coremark/run1.log`. Since this is a
simulation, the error message can be ignored and the computed iterations/sec
corresponds to CoreMark/MHz.

To get the CoreMark of an FPGA implementation, build an image with UART output
and send it to the bootloader:

    make -C sw/coremark/ build-uart
    sw/bootloader/send_elf.sh /dev/ttyUSB1 sw/coremark/coremark_uart.elf

The CoreMark/MHz of RudolV is 0.892.



License
-------
Licensed under the ISC licence (similar to the MIT/Expat license).
