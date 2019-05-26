#!/bin/sh
#
# Synthesize bitstream for Digilent Genesys 2
#   ./run_tcl.sh genesys2_synth.tcl

# Program bitstream to any Xilinx FPGA
#   ./run_tcl.sh prog.tcl
# 


#/opt/Xilinx/Vivado/2018.3/bin/vivado -nojournal -log vivado.log -mode batch -source $1
/opt/Xilinx/Vivado/2018.3/bin/vivado -nojournal -mode batch -source $1
