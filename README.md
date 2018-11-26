# Danzig

A 32 bit RISC-V processor with 5 pipeline stages for the 
RISC-V SoftCPU Contest https://riscv.org/2018contest/. 
Only open source software is used for testing and synthesis. 

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

|                      | DMIPS/MHz | Dhrystones/s/MHz | CPI   |
| -------------------- | --------- | ---------------- | ----- |
| `riscv-dhrystone`    | 0.75      | 1362             | 1,66  |
| `picorv32-dhrystone` | 0.968     | 1702             | 1,497 |

Synthesize with icestorm and flash to a Lattice iCE40 UltraPlus MDP board.
The boards must be configured to flash and run FPGA U4. It works at 24 MHz.

    make DEVICE=up5k ARACHNE_DEVICE="5k -P uwg30" clean arachne prog

Now processor in the FPGA executes the bootloader and waits for data from the UART.
To run the Dhrystone benchmark (riscv-tests version) on the FPGA use:

    make -C sw/uart-dhrystone/
    sw/bootloader/send_image.sh /dev/ttyUSB1 sw/uart-dhrystone/dhrystone.hex

Another small test program for the UART is at `sw/uart/`




License
-------
Licensed under the ISC licence (similar to the MIT/Expat license).
