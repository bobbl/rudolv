
# synthesize
read_verilog ../../src/memory.v
read_verilog ../../src/csr.v
read_verilog ../../src/regset.v
read_verilog ../../pipeline.v
read_verilog genesys2.v

read_xdc genesys2.xdc

# add command line arguments
set Cmd "synth_design -part xc7k325tffg900-2 -top top "
append Cmd [join $argv]
eval $Cmd

opt_design
place_design
#phys_opt_design
route_design

report_utilization
report_timing
#write_verilog -force design.v
write_bitstream -force design.bit
#write_mem_info -force synth_system.mmi

