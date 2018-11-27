# Danzig

A 32 bit RISC-V processor with 5 pipeline stages for the
[RISC-V SoftCPU Contest](https://riscv.org/2018contest/).
Only open source software is used for testing and synthesis.

| chip resources  | used | unit      |                             |
|:--------------- | ----:| ---------:|:--------------------------- |
| LCs             | 2552 |      LUT4 | 40% for CSRs                |
| BRAM            |    6 |   4 KiBit | registerset and boot loader |
| SPRAM           |    2 | 256 KiBit | main memory                 |
| clock frequency |   24 |       MHz |                             |

Edit `config.mk` if the tools are not in the search path.

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

Dhrystone results

|                      | DMIPS/MHz | Dhrystones/s/MHz | CPI   | cycles per Dhrystone |
| -------------------- | --------- | ---------------- | ----- | -------------------- |
| `riscv-dhrystone`    | 0.75      | 1362             | 1,66  | 734                  |
| `picorv32-dhrystone` | 0.968     | 1702             | 1,497 | 587                  |

Synthesize with icestorm and flash to a Lattice iCE40 UltraPlus MDP board.
The board must be configured to flash and run FPGA U4.

    make -C sw/bootloader/
    make -C scripts/icestorm/ DEVICE=up5k ARACHNE_DEVICE="5k -P uwg30" clean arachne prog

Now the processor within the FPGA executes the bootloader and waits for data
from the UART. To run the Dhrystone benchmark (riscv-tests version) on the FPGA
use:

    make -C sw/uart-dhrystone/
    sw/bootloader/send_image.sh /dev/ttyUSB1 sw/uart-dhrystone/dhrystone.bin

Another small test program for the UART is at `sw/uart/uart.c`. To see the
output of the programs use a terminal emulater like picocom at 115200 baud:

    picocom -b 115200 --imap lfcrlf /dev/ttyUSB1

The processor logic and the bootloader are written to the flash memory. Hence, the
processor can be reset with switch SW2 of the MDP board. After switching it back and
forth, `send_image.sh` can be used again to start a new program on the processor.




License
-------
Licensed under the ISC licence (similar to the MIT/Expat license).
