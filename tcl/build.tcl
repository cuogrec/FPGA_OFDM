# =============================================================================
#  build.tcl -- create the Vivado 2024.2 project for the 33 kHz OFDM receiver
#  Run:  vivado -mode batch -source build.tcl            (project + block design)
#        vivado -mode batch -source build.tcl -tclargs impl   (+ synth/impl/bitstream)
#  Folder layout:  <root>/tcl/build.tcl  <root>/rtl/*.v  <root>/xdc/*.xdc
# =============================================================================
set root  [file normalize [file dirname [info script]]/..]
set proj  ofdm33_rx
set part  xczu7ev-ffvc1156-2-e

create_project $proj $root/vivado/$proj -part $part -force
set bp [lindex [lsort -dictionary [get_board_parts -quiet *zcu104*]] end]
if {$bp ne ""} { set_property board_part $bp [current_project] } \
else { puts "WARNING: ZCU104 board files not found -- PS preset will not be applied" }

add_files -norecurse [glob $root/rtl/*.v]
set_property file_type Verilog [get_files -of_objects [get_filesets sources_1] *.v]
update_compile_order -fileset sources_1
add_files -fileset constrs_1 -norecurse $root/xdc/ofdm33_zcu104.xdc

source $root/tcl/ofdm33_rx_bd.tcl

set bdf [get_files ofdm33_rx_bd.bd]
generate_target all $bdf
set wrap [make_wrapper -files $bdf -top]
if {$wrap eq "" || ![file exists $wrap]} {
    set wrap [lindex [glob -nocomplain $root/vivado/$proj/$proj.gen/sources_1/bd/ofdm33_rx_bd/hdl/ofdm33_rx_bd_wrapper.v] 0]
}
puts "INFO: BD wrapper = $wrap"
if {[catch {add_files -norecurse $wrap} e]} { puts "INFO: add wrapper: $e" }
set_property top ofdm33_rx_bd_wrapper [current_fileset]
update_compile_order -fileset sources_1

if {[llength $argv] > 0 && [lindex $argv 0] eq "impl"} {
    launch_runs impl_1 -to_step write_bitstream -jobs 8
    wait_on_run impl_1
    write_hw_platform -fixed -include_bit -force $root/vivado/ofdm33_rx.xsa
    puts "INFO: bitstream + XSA written"
}
