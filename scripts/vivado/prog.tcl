# program FPGA with design.bit

open_hw

# disconnect before connecting
if { [catch {connect_hw_server} ] } {
    disconnect_hw_server
    connect_hw_server
}


if { [info exists ::env(VIVADO_HW_TARGET) ] } {
    set Target $::env(VIVADO_HW_TARGET)
    #set Target "*/xilinx_tcf/Digilent/200300730257B"
    open_hw_target [get_hw_targets $Target]
} else {
    puts "\033\[1;37m----------------------------------------------------------"
    puts "No Xilinx hardware target specified"
    puts "Please set VIVADO_HW_TARGET in config.sh to something like"
    puts "  VIVADO_HW_TARGET=*/xilinx_tcf/Digilent/100200300400Z"
    puts "Otherwise the JTAG connection cannot be opened reliably"
    puts "(usually it takes 3 tries)"
    puts "----------------------------------------------------------\033\[0m"

    open_hw_target
}

set Device [lindex [get_hw_devices] 0]
current_hw_device $Device
refresh_hw_device -update_hw_probes false $Device
set_property PROGRAM.FILE "design.bit" $Device
program_hw_devices $Device
refresh_hw_device $Device
disconnect_hw_server
