#!/bin/sh

# CAUTION: offset is a word (4 byte) address, not a byte address
sed -e 's/@0 /@3f80 /' ../../sw/uart/build/bl_char.hex > bootloader.hex

/opt/Xilinx/Vivado/2018.3/bin/vivado -nojournal -mode batch -source genesys2_synth.tcl
/opt/Xilinx/Vivado/2018.3/bin/vivado -nojournal -mode batch -source prog.tcl

