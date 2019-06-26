# program FPGA with design.bit

open_hw

# disconnect before connecting
if { [catch {connect_hw_server} ] } {
    disconnect_hw_server
    connect_hw_server
}

open_hw_target 
# Does not work reliable (usually it takes 3 tries).
# When you know your device number replace by:
# open_hw_target [get_hw_targets */xilinx_tcf/Digilent/DEVICENUMBER]


set Device [lindex [get_hw_devices] 0]
current_hw_device $Device
refresh_hw_device -update_hw_probes false $Device
set_property PROGRAM.FILE "design.bit" $Device
program_hw_devices $Device
refresh_hw_device $Device
disconnect_hw_server
