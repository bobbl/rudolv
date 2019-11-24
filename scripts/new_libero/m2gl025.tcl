set projname m2gl025
set projpath $projname

info commands
info procs

new_project \
  -location $projpath \
  -name $projname \
  -hdl VERILOG \
  -family IGLOO2 \
  -die "M2GL025" \
  -use_enhanced_constraint_flow 1

set_device \
  -family {IGLOO2} -die {PA4MGL2500_N} -package {vf256} \
  -speed {STD} -die_voltage {1.2} -part_range {COM} \
  -adv_options {IO_DEFT_STD:LVCMOS 2.5V} \
  -adv_options {RESERVEMIGRATIONPINS:1} \
  -adv_options {RESTRICTPROBEPINS:1} \
  -adv_options {RESTRICTSPIPINS:0} \
  -adv_options {TEMPR:COM} \
  -adv_options {UNUSED_MSS_IO_RESISTOR_PULL:None} \
  -adv_options {VCCI_1.2_VOLTR:COM} \
  -adv_options {VCCI_1.5_VOLTR:COM} \
  -adv_options {VCCI_1.8_VOLTR:COM} \
  -adv_options {VCCI_2.5_VOLTR:COM} \
  -adv_options {VCCI_3.3_VOLTR:COM} \
  -adv_options {VOLTR:COM} 

# create module for Chip Oscillator
create_and_configure_core -core_vlnv {Actel:SgCore:OSC:2.0.101} -component_name {OSC_C0} -params {\
"RCOSC_1MHZ_DRIVES_CCC:false"  \
"RCOSC_1MHZ_DRIVES_FAB:false"  \
"RCOSC_1MHZ_IS_USED:false"  \
"RCOSC_25_50MHZ_DRIVES_CCC:1"  \
"RCOSC_25_50MHZ_DRIVES_FAB:false"  \
"RCOSC_25_50MHZ_IS_USED:1"  \
"VOLTAGE_IS_1_2:true"  \
"XTLOSC_DRIVES_CCC:false"  \
"XTLOSC_DRIVES_FAB:false"  \
"XTLOSC_FREQ:20.00"  \
"XTLOSC_IS_USED:false"  \
"XTLOSC_SRC:CRYSTAL"   }

# create module for Clock Conditioning Circuitry
create_and_configure_core -core_vlnv {Actel:SgCore:FCCC:2.0.201} -component_name {FCCC_C0} -params {\
"ADVANCED_TAB_CHANGED:false"  \
"CLK0_IS_USED:false"  \
"CLK0_PAD_IS_USED:false"  \
"CLK1_IS_USED:false"  \
"CLK1_PAD_IS_USED:false"  \
"CLK2_IS_USED:false"  \
"CLK2_PAD_IS_USED:false"  \
"CLK3_IS_USED:false"  \
"CLK3_PAD_IS_USED:false"  \
"DYN_CONF_IS_USED:false"  \
"GL0_BP_IN_0_FREQ:100"  \
"GL0_BP_IN_0_SRC:IO_HARDWIRED_0"  \
"GL0_BP_IN_1_FREQ:100"  \
"GL0_BP_IN_1_SRC:IO_HARDWIRED_0"  \
"GL0_FREQUENCY_LOCKED:false"  \
"GL0_IN_0_SRC:PLL"  \
"GL0_IN_1_SRC:UNUSED"  \
"GL0_IS_INVERTED:false"  \
"GL0_IS_USED:true"  \
"GL0_OUT_0_FREQ:100"  \
"GL0_OUT_1_FREQ:50"  \
"GL0_OUT_IS_GATED:false"  \
"GL0_PLL_IN_0_PHASE:0"  \
"GL0_PLL_IN_1_PHASE:0"  \
"GL1_BP_IN_0_FREQ:100"  \
"GL1_BP_IN_0_SRC:IO_HARDWIRED_0"  \
"GL1_BP_IN_1_FREQ:100"  \
"GL1_BP_IN_1_SRC:IO_HARDWIRED_0"  \
"GL1_FREQUENCY_LOCKED:false"  \
"GL1_IN_0_SRC:PLL"  \
"GL1_IN_1_SRC:UNUSED"  \
"GL1_IS_INVERTED:false"  \
"GL1_IS_USED:false"  \
"GL1_OUT_0_FREQ:100"  \
"GL1_OUT_1_FREQ:50"  \
"GL1_OUT_IS_GATED:false"  \
"GL1_PLL_IN_0_PHASE:0"  \
"GL1_PLL_IN_1_PHASE:0"  \
"GL2_BP_IN_0_FREQ:100"  \
"GL2_BP_IN_0_SRC:IO_HARDWIRED_0"  \
"GL2_BP_IN_1_FREQ:100"  \
"GL2_BP_IN_1_SRC:IO_HARDWIRED_0"  \
"GL2_FREQUENCY_LOCKED:false"  \
"GL2_IN_0_SRC:PLL"  \
"GL2_IN_1_SRC:UNUSED"  \
"GL2_IS_INVERTED:false"  \
"GL2_IS_USED:false"  \
"GL2_OUT_0_FREQ:100"  \
"GL2_OUT_1_FREQ:50"  \
"GL2_OUT_IS_GATED:false"  \
"GL2_PLL_IN_0_PHASE:0"  \
"GL2_PLL_IN_1_PHASE:0"  \
"GL3_BP_IN_0_FREQ:100"  \
"GL3_BP_IN_0_SRC:IO_HARDWIRED_0"  \
"GL3_BP_IN_1_FREQ:100"  \
"GL3_BP_IN_1_SRC:IO_HARDWIRED_0"  \
"GL3_FREQUENCY_LOCKED:false"  \
"GL3_IN_0_SRC:PLL"  \
"GL3_IN_1_SRC:UNUSED"  \
"GL3_IS_INVERTED:false"  \
"GL3_IS_USED:false"  \
"GL3_OUT_0_FREQ:100"  \
"GL3_OUT_1_FREQ:50"  \
"GL3_OUT_IS_GATED:false"  \
"GL3_PLL_IN_0_PHASE:0"  \
"GL3_PLL_IN_1_PHASE:0"  \
"GPD0_IS_USED:false"  \
"GPD0_NOPIPE_RSTSYNC:true"  \
"GPD0_SYNC_STYLE:G3STYLE_AND_NO_LOCK_RSTSYNC"  \
"GPD1_IS_USED:false"  \
"GPD1_NOPIPE_RSTSYNC:true"  \
"GPD1_SYNC_STYLE:G3STYLE_AND_NO_LOCK_RSTSYNC"  \
"GPD2_IS_USED:false"  \
"GPD2_NOPIPE_RSTSYNC:true"  \
"GPD2_SYNC_STYLE:G3STYLE_AND_NO_LOCK_RSTSYNC"  \
"GPD3_IS_USED:false"  \
"GPD3_NOPIPE_RSTSYNC:true"  \
"GPD3_SYNC_STYLE:G3STYLE_AND_NO_LOCK_RSTSYNC"  \
"GPD_EXPOSE_RESETS:false"  \
"GPD_SYNC_STYLE:G3STYLE_AND_LOCK_RSTSYNC"  \
"INIT:0000007FB8000044D74000318C6318C1F18C61EC0404040400301"  \
"IO_HARDWIRED_0_IS_DIFF:false"  \
"IO_HARDWIRED_1_IS_DIFF:false"  \
"IO_HARDWIRED_2_IS_DIFF:false"  \
"IO_HARDWIRED_3_IS_DIFF:false"  \
"MODE_10V:false"  \
"NGMUX0_HOLD_IS_USED:false"  \
"NGMUX0_IS_USED:false"  \
"NGMUX1_HOLD_IS_USED:false"  \
"NGMUX1_IS_USED:false"  \
"NGMUX2_HOLD_IS_USED:false"  \
"NGMUX2_IS_USED:false"  \
"NGMUX3_HOLD_IS_USED:false"  \
"NGMUX3_IS_USED:false"  \
"NGMUX_EXPOSE_HOLD:false"  \
"PLL_DELAY:0"  \
"PLL_EXPOSE_BYPASS:false"  \
"PLL_EXPOSE_RESETS:false"  \
"PLL_EXT_FB_GL:EXT_FB_GL0"  \
"PLL_FB_SRC:CCC_INTERNAL"  \
"PLL_IN_FREQ:50.000"  \
"PLL_IN_SRC:OSC_50MHZ"  \
"PLL_IS_USED:true"  \
"PLL_LOCK_IND:1024"  \
"PLL_LOCK_WND:32000"  \
"PLL_SSM_DEPTH:0.5"  \
"PLL_SSM_ENABLE:false"  \
"PLL_SSM_FREQ:40"  \
"PLL_SUPPLY_VOLTAGE:25_V"  \
"PLL_VCO_TARGET:700"  \
"RCOSC_1MHZ_IS_USED:false"  \
"RCOSC_25_50MHZ_IS_USED:true"  \
"VCOFREQUENCY:800.000"  \
"XTLOSC_IS_USED:false"  \
"Y0_IS_USED:false"  \
"Y1_IS_USED:false"  \
"Y2_IS_USED:false"  \
"Y3_IS_USED:false"   }

import_files -hdl_source ../../pipeline.v
import_files -hdl_source ../../src/csr.v
import_files -hdl_source ${projname}.v
build_design_hierarchy 
set_root -module {top::work} 


# .hex files must be in synthesis subdir
file copy ../../sw/uart/build/grubby_char.hex ${projpath}/synthesis/grubby_char.hex

# Compute timing constraints
derive_constraints_sdc

# Pin constraints
import_files -io_pdc ${projname}.pdc



organize_tool_files -tool {SYNTHESIZE} \
  -file ${projpath}/constraint/top_derived_constraints.sdc \
  -module {top::work} \
  -input_type constraint

organize_tool_files -tool {PLACEROUTE} \
  -file ${projpath}/constraint/io/${projname}.pdc \
  -file ${projpath}/constraint/top_derived_constraints.sdc \
  -module top::work \
  -input_type {constraint}

organize_tool_files -tool {VERIFYTIMING} \
  -file ${projpath}/constraint/top_derived_constraints.sdc \
  -module top::work \
  -input_type {constraint}

# full synthesis flow
update_and_run_tool -name {PROGRAMDEVICE} 



#puts "\033\[1;36mSYNTHESIZE\033\[0m"
#update_and_run_tool -name {SYNTHESIZE}
#puts "\033\[1;36mPLACEROUTE\033\[0m"
#run_tool -name {PLACEROUTE}
##puts "\033\[1;36mVERIFYTIMING\033\[0m"
##run_tool -name {VERIFYTIMING}
#puts "\033\[1;36mGENERATEPROGRAMMINGDATA\033\[0m"
#run_tool -name {GENERATEPROGRAMMINGDATA}
#puts "\033\[1;36mGENERATEPROGRAMMINGFILE\033\[0m"
#run_tool -name {GENERATEPROGRAMMINGFILE}
#puts "\033\[1;36mPROGRAMDEVICE\033\[0m"
#run_tool -name {PROGRAMDEVICE}

## export_bitstream
#export_bitstream_file -trusted_facility_file 1 -trusted_facility_file_components {FABRIC}

exit 0

