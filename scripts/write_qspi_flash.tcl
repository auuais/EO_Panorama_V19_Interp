# Burn a bitstream into the board's QSPI flash, so it boots without JTAG.
#
#   vivado -mode batch -source scripts/write_qspi_flash.tcl -tclargs <path-to-.bit> ?<tag>?
#
# WRITES FLASH.  It replaces the boot image: until this completes the board has
# no valid image to boot, and a failure part way leaves the flash erased.
# Programming also loads Vivado's indirect-programming bitstream into the FPGA,
# so whatever was running over JTAG is gone; the script boots the device from
# flash at the end, which is the same thing a power cycle does.
#
# Geometry matches the images already in mcs/ (V19_M1_Timout.prm): MCS, SPIx4,
# 32 MB, loaded up from address 0.  The part is the one the saved hardware
# session used, a Micron 256 Mbit QSPI.
set project_root [file normalize [file join [file dirname [info script]] ..]]
set top KintexTop_EO_IR_HD_SDI_panorama_base
set part mt25qu256-spi-x1_x2_x4

if {[llength $argv] < 1} { error "usage: write_qspi_flash.tcl <bitfile> ?tag?" }
set bitfile [file normalize [lindex $argv 0]]
if {![file exists $bitfile]} { error "Bitstream not found: $bitfile" }
set tag [expr {[llength $argv] > 1 ? [lindex $argv 1] : "qspi"}]

set mcs [file join $project_root mcs "${top}_${tag}.mcs"]
file mkdir [file dirname $mcs]
write_cfgmem -force -format mcs -interface SPIx4 -size 32 \
    -loadbit "up 0x00000000 $bitfile" -file $mcs
puts "QSPI_MCS=$mcs"

open_hw_manager
connect_hw_server -url TCP:127.0.0.1:3121
open_hw_target
set dev [lindex [get_hw_devices] 0]
current_hw_device $dev
refresh_hw_device -quiet $dev

create_hw_cfgmem -hw_device $dev [lindex [get_cfgmem_parts $part] 0]
set cfg [get_property PROGRAM.HW_CFGMEM $dev]
set_property PROGRAM.FILES [list $mcs] $cfg
set_property PROGRAM.PRM_FILE [file rootname $mcs].prm $cfg
set_property PROGRAM.ADDRESS_RANGE {use_file} $cfg
set_property PROGRAM.UNUSED_PIN_TERMINATION {pull-none} $cfg
set_property PROGRAM.BLANK_CHECK 0 $cfg
set_property PROGRAM.ERASE 1 $cfg
set_property PROGRAM.CFG_PROGRAM 1 $cfg
set_property PROGRAM.VERIFY 1 $cfg
set_property PROGRAM.CHECKSUM 0 $cfg

# The indirect flash programmer runs in the FPGA itself.
create_hw_bitstream -hw_device $dev [get_property PROGRAM.HW_CFGMEM_BITFILE $dev]
program_hw_devices $dev
refresh_hw_device -quiet $dev

program_hw_cfgmem -hw_cfgmem $cfg
puts "QSPI_WRITTEN=$mcs"

# Boot the FPGA from the flash that was just written, as a power cycle would.
boot_hw_device $dev
after 8000
refresh_hw_device -quiet $dev
# The boot itself reports "Done pin status" above; this confirms the design,
# not just configuration, by finding its debug cores.
puts "ILA_CORES=[llength [get_hw_ilas -quiet]]"
puts "QSPI_BOOTED=$tag"
close_hw_target
close_hw_manager
